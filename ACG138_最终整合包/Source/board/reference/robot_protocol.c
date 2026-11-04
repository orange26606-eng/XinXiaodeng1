#include "robot_protocol.h"
#include <string.h>
static uint32_t le32(const uint8_t *p)
{ return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24; }
uint32_t robot_crc32(const uint8_t *p, size_t n)
{
    uint32_t crc=UINT32_MAX;
    for(size_t i=0;i<n;++i) {
        crc^=p[i];
        for(unsigned bit=0;bit<8;++bit) crc=(crc>>1)^((crc&1)?UINT32_C(0xedb88320):0);
    }
    return ~crc;
}
int robot_model_push(struct robot_model *m,const uint8_t *p,size_t n,uint64_t now)
{
    if(!m) return -1;
    if(!p || n!=ROBOT_PACKET_SIZE) goto bad;
    int recovery=p[24]==0xA3;
    int nav=p[24]==0xA2 || recovery;
    if(memcmp(p,"RBT1",4) || p[4]!=1 || p[5]>7 ||
       p[6]>(nav?7:6) || p[7]>(nav?7:6) || p[8]>(nav?7:6) || p[9]>1 || p[10]>8 || p[11]>(recovery?175:15) ||
       p[12]>10 || p[13]>(nav?11:4) || !((p[24]==0 && p[25]==0 && p[26]==0 && p[27]==0) || (p[24]==0xA1 && !(p[25]&0xF0)) || (nav && !(p[25]&0x80))) ||
       robot_crc32(p,28)!=le32(p+28)) goto bad;
    uint16_t mm[3];
    for(unsigned ch=0;ch<3;++ch) {
        mm[ch]=(uint16_t)((unsigned)p[14+2*ch]|(unsigned)p[15+2*ch]<<8);
        if((p[5]>>ch)&1) {
            if(nav) {
                unsigned e=p[6+ch];
                if(e==5 || e==6 || mm[ch]>4500 || (e==4?mm[ch]!=0:mm[ch]<30)) goto bad;
            } else if(p[6+ch] || mm[ch]<30 || mm[ch]>4500) goto bad;
        } else if(!p[6+ch]) mm[ch]=4500;
        else mm[ch]=0;
    }
    uint32_t sequence=le32(p+20);
    if(m->seen && now>=m->last_ms && now-m->last_ms<ROBOT_STALE_MS &&
       (int32_t)(sequence-m->sequence)<=0) { ++m->rejected; return 0; }
    for(unsigned ch=0;ch<3;++ch) { m->mm[ch]=mm[ch]; m->error[ch]=p[6+ch]; }
    m->valid=p[5];m->auto_mode=p[9];m->actual=p[10];m->feedback=p[11]&15;
    unsigned meta=recovery?p[11]>>4:0;
    m->feedback_supported=(uint8_t)recovery;
    m->feedback_phase=meta==10?4:meta?((meta-1)%3)+1:0;
    m->feedback_retries=meta==10?2:meta?(meta-1)/3:0;
    m->state=p[12];m->stop_reason=p[13];m->sequence=sequence;
    m->remote_supported=p[24]==0xA1 || nav;m->remote_flags=p[25]&15;
    m->nav_supported=(uint8_t)nav;m->near_mask=nav?(p[25]>>4)&7:0;
    m->remote_sequence=(uint16_t)((unsigned)p[26]|(unsigned)p[27]<<8);
    m->last_ms=now;m->seen=1;m->bad_packet=0;++m->received;
    return 1;
bad:
    ++m->rejected;m->bad_packet=1;return -1;
}
enum robot_distance_state robot_distance_state(const struct robot_model *m,unsigned ch,uint64_t now)
{
    if(!m || ch>=3) return ROBOT_INVALID;
    if(m->bad_packet) return ROBOT_INVALID;
    if(!m->seen) return ROBOT_WAITING;
    if(now<m->last_ms || now-m->last_ms>=ROBOT_STALE_MS) return ROBOT_STALE;
    if(m->nav_supported) {
        if(m->error[ch]==5) return ROBOT_WAITING;
        if(m->error[ch]==6) return ROBOT_LOST;
        if(m->error[ch]==4) return ROBOT_BLIND;
        if(m->error[ch] && (m->valid&(1u<<ch))) return ROBOT_HOLD;
    }
    if(m->error[ch]) return ROBOT_INVALID;
    return (m->valid&(1u<<ch))?ROBOT_DISTANCE:ROBOT_MAXIMUM;
}
enum robot_range_state robot_auto_range_state(const struct robot_model *m,unsigned ch,uint64_t now)
{
    enum robot_distance_state state=robot_distance_state(m,ch,now);
    if(state!=ROBOT_DISTANCE && state!=ROBOT_MAXIMUM)return ROBOT_RANGE_UNKNOWN;
    unsigned mm=state==ROBOT_MAXIMUM?4500:m->mm[ch];
    unsigned threshold=!m->nav_supported?300:m->auto_mode?(ch==0?350:180):(ch==0?150:100);
    return mm>=threshold && mm<=4500 && !(m->near_mask&(1u<<ch))?ROBOT_RANGE_ALLOWED:ROBOT_RANGE_BLOCKED;
}

enum robot_drive_mode robot_drive_mode(const struct robot_model *m,uint64_t now)
{
    if(!m || !m->seen || m->bad_packet || now<m->last_ms || now-m->last_ms>=ROBOT_STALE_MS)
        return ROBOT_MODE_UNKNOWN;
    return m->auto_mode?ROBOT_MODE_AUTO:ROBOT_MODE_MANUAL;
}

const char *robot_reason_text(unsigned reason)
{
    static const char *const names[]={"正常运行","前方过近","侧方过近","测距失效或等待",
        "电机反馈异常","测距保持·限速或等待","低速恢复或脱困","自动复测·寻找通路",
        "原地转向超时·自动复测","方向安全条件不满足","等待编码器","手动停车"};
    return reason<sizeof(names)/sizeof(names[0])?names[reason]:"未知状态";
}
const char *robot_action_text(unsigned action)
{
    static const char *const names[]={"停车","前进","后退","左弧线","右弧线","原地左转","原地右转","后退左转","后退右转"};
    return action<sizeof(names)/sizeof(names[0])?names[action]:"未知动作";
}

const char *robot_feedback_phase_text(unsigned phase)
{
    static const char *const names[]={"反馈正常","受限恢复","等待重试","低速验证","重试用尽·已停机"};
    return phase<5?names[phase]:"未知反馈阶段";
}
