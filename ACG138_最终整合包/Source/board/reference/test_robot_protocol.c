#include "robot_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void finish(uint8_t *p,uint32_t seq)
{
    for(unsigned i=0;i<4;++i)p[20+i]=(uint8_t)(seq>>(8*i));
    uint32_t crc=robot_crc32(p,28);
    for(unsigned i=0;i<4;++i)p[28+i]=(uint8_t)(crc>>(8*i));
}
int main(void)
{
    struct robot_model m={0};
    assert(robot_drive_mode(NULL,100)==ROBOT_MODE_UNKNOWN);
    assert(robot_drive_mode(&m,100)==ROBOT_MODE_UNKNOWN);
    assert(robot_distance_state(&m,0,100)==ROBOT_WAITING);
    uint8_t p[32]={ 'R','B','T','1',1,1,0,0,5,1,1,0,1,0,0xf4,1,0xee,0xee,0,0 };
    finish(p,UINT32_MAX);
    assert(robot_model_push(&m,p,32,100)==1);
    assert(robot_drive_mode(&m,100)==ROBOT_MODE_AUTO);
    assert(m.mm[0]==500 && m.mm[1]==4500 && m.mm[2]==0);
    assert(robot_distance_state(&m,0,100)==ROBOT_DISTANCE);
    assert(robot_distance_state(&m,1,100)==ROBOT_MAXIMUM);
    assert(robot_distance_state(&m,2,100)==ROBOT_INVALID);
    p[9]=0;finish(p,0);assert(robot_model_push(&m,p,32,200)==1); /* 检查序号回绕，以及自动转手动的遥测。 */
    assert(robot_drive_mode(&m,200)==ROBOT_MODE_MANUAL);
    assert(robot_model_push(&m,p,32,300)==0 && m.last_ms==200); /* 重复数据不能刷新数据时效。 */
    finish(p,UINT32_MAX);assert(robot_model_push(&m,p,32,300)==0); /* 旧报文。 */
    assert(robot_distance_state(&m,1,1200)==ROBOT_STALE);
    assert(robot_drive_mode(&m,1200)==ROBOT_MODE_UNKNOWN);
    finish(p,0);assert(robot_model_push(&m,p,32,1300)==1); /* 静默一段时间后FPGA重新启动。 */
    p[15]^=1;assert(robot_model_push(&m,p,32,1400)==-1);
    assert(robot_distance_state(&m,1,1400)==ROBOT_INVALID); /* 坏报文保持无效状态。 */
    assert(robot_drive_mode(&m,1400)==ROBOT_MODE_UNKNOWN);
    p[15]^=1;finish(p,1);assert(robot_model_push(&m,p,32,1500)==1);
    assert(robot_model_push(&m,p,31,1600)==-1);
    assert(robot_model_push(&m,p,33,1600)==-1);
    p[6]=2;finish(p,2);assert(robot_model_push(&m,p,32,1600)==-1); /* V=1与ERR非零的组合不一致。 */
    p[5]=0;finish(p,3);assert(robot_model_push(&m,p,32,1700)==1);
    assert(robot_distance_state(&m,0,1700)==ROBOT_INVALID);
    p[6]=0;finish(p,4);assert(robot_model_push(&m,p,32,1800)==1);
    assert(m.mm[0]==4500 && robot_distance_state(&m,0,1800)==ROBOT_MAXIMUM);
    p[5]=1;p[14]=29;p[15]=0;finish(p,5);assert(robot_model_push(&m,p,32,1900)==-1);
    p[14]=0x94;p[15]=0x11;finish(p,6);assert(robot_model_push(&m,p,32,1900)==1);
    assert(m.mm[0]==4500 && robot_distance_state(&m,0,1900)==ROBOT_DISTANCE);
    p[14]=0x95;finish(p,7);assert(robot_model_push(&m,p,32,1950)==-1);
    for(unsigned err=1;err<=6;++err) {
        p[5]=0;p[6]=(uint8_t)err;finish(p,10+err);
        assert(robot_model_push(&m,p,32,2000+err)==1);
        assert(robot_distance_state(&m,0,2000+err)==ROBOT_INVALID);
    }
    assert(robot_crc32((const uint8_t *)"123456789",9)==UINT32_C(0xcbf43926));
    m=(struct robot_model){.seen=1,.last_ms=3000,.valid=7,.mm={300,1500,4500}};
    for(unsigned ch=0;ch<3;++ch) {
        assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_ALLOWED);
        m.mm[ch]=30;assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_BLOCKED);
        m.mm[ch]=299;assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_BLOCKED);
        m.mm[ch]=300;assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_ALLOWED);
        m.mm[ch]=4500;assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_ALLOWED);
        m.mm[ch]=4501;assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_BLOCKED);
        m.valid&=(uint8_t)~(1u<<ch);assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_ALLOWED);
        m.error[ch]=2;assert(robot_auto_range_state(&m,ch,3000)==ROBOT_RANGE_UNKNOWN);
        m.error[ch]=0;m.valid=7;m.mm[ch]=1000;
    }
    assert(robot_auto_range_state(&m,0,4000)==ROBOT_RANGE_UNKNOWN);
    assert(robot_drive_mode(&m,3000)==ROBOT_MODE_MANUAL);
    m.auto_mode=1;assert(robot_drive_mode(&m,3000)==ROBOT_MODE_AUTO);
    m.error[0]=2;assert(robot_drive_mode(&m,3000)==ROBOT_MODE_AUTO); /* 模式状态独立于距离有效性。 */
    m.auto_mode=0;assert(robot_drive_mode(&m,3000)==ROBOT_MODE_MANUAL);
    assert(robot_drive_mode(&m,2999)==ROBOT_MODE_UNKNOWN);
    assert(robot_drive_mode(&m,4000)==ROBOT_MODE_UNKNOWN);
    puts("PASS live mode: manual, AUTO, distance faults, automatic fallback, missing/stale/CRC invalid telemetry");
    puts("PASS uniform distance condition: 30..450cm and healthy MAX allow motion; below 30cm, errors and timeout prohibit motion");
    puts("PASS robot protocol: range clamp, raw flags, six error classes, CRC, malformed packets, timeout, replay and sequence wrap");
    return 0;
}
