#include "phone_control_link.h"
#include <string.h>
static unsigned le16(const uint8_t*p){return (unsigned)p[0]|(unsigned)p[1]<<8;}
static uint32_t le32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static unsigned be16(const uint8_t*p){return (unsigned)p[0]<<8|p[1];}
static int ip_checksum(const uint8_t*p){unsigned sum=0;for(unsigned i=0;i<20;i+=2)sum+=be16(p+i);while(sum>>16)sum=(sum&65535)+(sum>>16);return sum==65535;}
int phone_link_push(struct phone_control_link*l,const uint8_t*f,size_t n,const uint8_t host[6],uint64_t stamp){
 static const uint8_t fpga[]={0,10,53,1,254,192},broadcast[]={255,255,255,255,255,255},ips[]={192,168,0,2,192,168,0,3};
 if(!l||!f||!host||n<42)return -1;
 if(memcmp(f+6,fpga,6)||(memcmp(f,host,6)&&memcmp(f,broadcast,6))||f[12]!=8||f[13]||f[14]!=0x45||
 (f[20]!=0&&f[20]!=0x40)||f[21]||f[23]!=17||!ip_checksum(f+14)||memcmp(f+26,ips,8))return 0;
 unsigned port=be16(f+34),bytes=be16(f+38);if((port!=5008&&port!=5013)||be16(f+36)!=port)return 0;
 if(bytes<8||n!=34u+bytes||be16(f+16)!=20u+bytes||f[40]||f[41])goto bad;
 if(port==5008){if(bytes!=40)goto bad;int r=robot_model_push(&l->robot,f+42,32,stamp);if(r>0)l->received++;else if(r<0)l->rejected++;return r;}
 if(bytes!=104)goto bad;const uint8_t*p=f+42;
 /* 可选NCD5伴随数据仅供诊断，不刷新状态或租约时效。 */
 if(!memcmp(p,"NCD5",4)){
  if(p[4]!=1||p[5]!=PHONE_POLICY_REVISION||le32(p+8)!=PHONE_CONFIG_ID||robot_crc32(p,92)!=le32(p+92))goto bad;
  if(p[6]>7||p[7]>7||p[56]>127||p[57]>7||p[58]>7||p[59]>7||p[60]>7||p[22]||p[23])goto bad;
  for(unsigned k=61;k<92;k++)if(p[k])goto bad;
  uint32_t diag_uptime=le32(p+12);if(l->have_diagnostic&&(int32_t)(diag_uptime-l->diagnostic_uptime)<=0)return 0;
  l->diagnostic_uptime=diag_uptime;l->diagnostic_ms=stamp;l->have_diagnostic=1;l->raw_available=p[6];l->diagnostic_zero=p[7];l->diagnostic_flags=p[56];l->diagnostic_fresh=p[57];l->diagnostic_near=p[58];l->diagnostic_pending=p[59];l->diagnostic_observation=p[60];
  for(unsigned k=0;k<3;k++){l->raw_mm[k]=(uint16_t)le16(p+16+2*k);l->raw_sequence[k]=le32(p+24+4*k);l->raw_age_us[k]=le32(p+36+4*k);}
  for(unsigned k=0;k<4;k++)l->effective_duty[k]=(uint16_t)le16(p+48+2*k);
  l->received++;return 1;
 }
 if(memcmp(p,"NCS1",4)||p[4]!=1||robot_crc32(p,92)!=le32(p+92))goto bad;
 l->max_ttl_ms=(uint16_t)le16(p+66);l->status_revision=(uint16_t)le16(p+70);l->max_rpm_centi=(uint16_t)le16(p+68);l->policy_mismatch=le32(p+28)!=PHONE_CONFIG_ID||l->status_revision!=PHONE_STATUS_REVISION||l->max_rpm_centi!=12000||l->max_ttl_ms!=PHONE_MOTION_TTL_MS;
 if(p[5]>6||(p[6]&7)!=7||p[7]>11||le16(p+66)!=PHONE_MOTION_TTL_MS||
 le16(p+68)!=12000||le16(p+70)<4||le16(p+70)>5||(p[84]&3)>2||robot_crc32(p,92)!=le32(p+92))goto bad;
 uint32_t uptime=le32(p+24);if(l->have_status&&(int32_t)(uptime-l->uptime)<=0&&stamp>=l->status_ms&&stamp-l->status_ms<PHONE_LINK_MS)return 0;
 l->status_revision=(uint16_t)le16(p+70);l->sonar_pending_mask=l->status_revision>=5?(p[6]>>3)&7:0;l->front_seq_mod4=l->status_revision>=5?p[6]>>6:0;l->policy_mismatch=le32(p+28)!=PHONE_CONFIG_ID||l->status_revision!=PHONE_STATUS_REVISION||l->max_rpm_centi!=12000||l->max_ttl_ms!=PHONE_MOTION_TTL_MS;l->status_ms=stamp;l->uptime=uptime;l->config=le32(p+28);l->encoder_mask=p[64]&15;l->fault_mask=p[65]&15;
 l->sonar_valid=(p[89]>>4)&7;l->sonar_age_bucket[0]=p[64]>>4;l->sonar_age_bucket[1]=p[65]>>4;l->sonar_age_bucket[2]=(p[84]>>2)&15;l->sonar_raw_zero_mask=(uint8_t)(((p[84]>>6)&3)|((p[90]>>5)&4));l->have_status=1;l->received++;return 1;
bad:l->rejected++;return -1;
}
int phone_link_ready(const struct phone_control_link*l,uint64_t now){
 return l&&l->have_status&&now>=l->status_ms&&now-l->status_ms<PHONE_LINK_MS&&l->config==PHONE_CONFIG_ID&&!l->policy_mismatch&&l->status_revision==PHONE_STATUS_REVISION&&l->encoder_mask==15&&
 !l->fault_mask&&l->robot.seen&&now>=l->robot.last_ms&&now-l->robot.last_ms<PHONE_LINK_MS&&l->robot.remote_supported&&
 !(l->robot.feedback_supported&&l->robot.feedback_phase==4);
}
