#include "chassis_private.h"
#include "continuous_policy.h"
#include <math.h>
#include "s1_motion_policy.h"
#include <string.h>
static uint16_t u16(const uint8_t *p){return (uint16_t)(p[0]|(uint16_t)p[1]<<8);}
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static uint64_t u64(const uint8_t *p){return u32(p)|(uint64_t)u32(p+4)<<32;}
static void w16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void w32(uint8_t *p,uint32_t v){for(int i=0;i<4;++i)p[i]=(uint8_t)(v>>(8*i));}
static void w64(uint8_t *p,uint64_t v){w32(p,(uint32_t)v);w32(p+4,(uint32_t)(v>>32));}
static int zeros(const uint8_t *p,size_t n){for(size_t i=0;i<n;++i)if(p[i])return 0;return 1;}
static int16_t signed16(uint16_t v){int32_t x=v;if(v&32768u)x-=65536;return (int16_t)x;}
uint32_t ch_crc32(const void *data,size_t size){const uint8_t *p=data;uint32_t c=UINT32_MAX;
 while(size--){c^=*p++;for(int b=0;b<8;++b)c=(c>>1)^(UINT32_C(0xedb88320)&(uint32_t)-(int32_t)(c&1));}return ~c;}
static int command_valid(const ch_wire_command *c){
 if(!c||c->op>CH_RELEASE)return 0;
 if(c->op==CH_PROBE){if(c->mode||c->session||c->sequence||c->lease||c->ttl_ms||c->config_id)return 0;}
 else if((c->mode!=CH_MANUAL&&c->mode!=CH_NAVIGATION)||!c->session||!c->lease||!c->config_id)return 0;
 if(c->op==CH_DRIVE){if(!c->ttl_ms||c->ttl_ms>R6_COMMAND_TTL_MS||c->rpm_centi[0]!=c->rpm_centi[2]||c->rpm_centi[1]!=c->rpm_centi[3]||!s1_intent_matches(c->intent,c->rpm_centi[0],-(int)c->rpm_centi[1]))return 0;}
 else{if(c->ttl_ms||c->intent)return 0;for(int i=0;i<4;++i)if(c->rpm_centi[i])return 0;}
 return 1;
}
int ch_command_encode(uint8_t out[64],const ch_wire_command *c){
 if(!out||!command_valid(c))return CH_ARGUMENT;memset(out,0,64);memcpy(out,"NCV1",4);out[4]=S1_COMMAND_VERSION;out[5]=c->op;out[6]=c->mode;out[7]=c->intent;
 w32(out+8,c->session);w32(out+12,c->sequence);w64(out+16,c->lease);w16(out+24,c->ttl_ms);
 for(int i=0;i<4;++i)w16(out+28+2*i,(uint16_t)c->rpm_centi[i]);w32(out+36,c->config_id);w32(out+60,ch_crc32(out,60));return CH_OK;
}
int ch_command_decode(ch_wire_command *out,const uint8_t *p,size_t size){
 if(!out||!p)return CH_ARGUMENT;memset(out,0,sizeof(*out));
 if(size!=64||memcmp(p,"NCV1",4)||p[4]!=S1_COMMAND_VERSION||!zeros(p+26,2)||!zeros(p+40,20))return CH_FORMAT;
 if(ch_crc32(p,60)!=u32(p+60))return CH_CRC;
 ch_wire_command c={p[5],p[6],u32(p+8),u32(p+12),u64(p+16),u16(p+24),{0},u32(p+36)};
 c.intent=p[7];for(int i=0;i<4;++i)c.rpm_centi[i]=signed16(u16(p+28+2*i));if(!command_valid(&c))return CH_FORMAT;*out=c;return CH_OK;
}
int ch_status_encode(uint8_t out[96],const ch_wire_status *s){
 if(!out||!s||s->state>CH_RX_EXPIRED||s->reason>CH_RX_ULTRASONIC_BLOCKED||s->valid_mask>15||s->fault_mask>15||
 !s->lease||!s->config_id||s->mode>CH_NAVIGATION||!s->max_ttl_ms||s->max_ttl_ms>R6_COMMAND_TTL_MS||!s->max_rpm_centi||s->max_rpm_centi>32767)return CH_ARGUMENT;
 memset(out,0,96);memcpy(out,"NCS1",4);out[4]=1;out[5]=s->state;out[6]=(uint8_t)((s->capabilities&7u)|((s->revision==4||s->revision==5)?(s->sonar_pending&7u)<<3:0));out[7]=s->reason;if(s->revision==5)out[6]|=(uint8_t)((s->sonar_front_sequence&3u)<<6);
 w32(out+8,s->session);w32(out+12,s->sequence);w64(out+16,s->lease);w32(out+24,s->uptime_ms);w32(out+28,s->config_id);
 for(int i=0;i<4;++i){w16(out+32+2*i,(uint16_t)s->target_centi[i]);w16(out+40+2*i,(uint16_t)s->measured_centi[i]);w32(out+48+4*i,(uint32_t)s->counts[i]);}
 out[64]=(uint8_t)(s->valid_mask|((s->revision==4||s->revision==5)?(s->sonar_age_bucket[0]&15u)<<4:0));out[65]=(uint8_t)(s->fault_mask|((s->revision==4||s->revision==5)?(s->sonar_age_bucket[1]&15u)<<4:0));w16(out+66,s->max_ttl_ms);w16(out+68,s->max_rpm_centi);w16(out+70,s->revision);
 w32(out+72,s->accepted);w32(out+76,s->rejected);w32(out+80,s->watchdogs);out[84]=(uint8_t)(s->mode|((s->revision==4||s->revision==5)?((s->sonar_age_bucket[2]&15u)<<2)|((s->sonar_raw_zero_mask&3u)<<6):0));
 if(s->revision==3||(s->revision==4||s->revision==5)){
  if((s->sonar_valid|s->sonar_fresh|s->sonar_near)>7||s->sonar_degraded>1||s->sonar_front_stable>1)return CH_ARGUMENT;
  uint64_t packed=((uint64_t)s->sonar_valid<<36)|((uint64_t)s->sonar_front_stable<<39);
  for(int i=0;i<3;i++){uint16_t mm=(s->sonar_valid&(1u<<i))?s->sonar_mm[i]:4095;if(mm>4095||(mm==4095&&(s->sonar_valid&(1u<<i))))return CH_ARGUMENT;packed|=(uint64_t)mm<<(12*i);}
  for(int i=0;i<5;i++)out[85+i]=(uint8_t)(packed>>(8*i));
  out[90]=(uint8_t)(s->sonar_fresh|(s->sonar_near<<3)|(s->sonar_degraded<<6)|((s->revision==4||s->revision==5)?(s->sonar_raw_zero_mask&4u)<<5:0));out[91]=s->local_stop_sources;
 }
 w32(out+92,ch_crc32(out,92));return CH_OK;
}
int ch_status_decode(ch_wire_status *out,const uint8_t *p,size_t size){
 if(!out||!p)return CH_ARGUMENT;memset(out,0,sizeof(*out));
 if(size!=96||memcmp(p,"NCS1",4)||p[4]!=1)return CH_FORMAT;
 uint16_t revision=u16(p+70);if(revision!=3&&revision!=4&&revision!=5&&!zeros(p+85,7))return CH_FORMAT;
 if(revision==3&&(p[90]&128u))return CH_FORMAT;
 if(ch_crc32(p,92)!=u32(p+92))return CH_CRC;
 ch_wire_status s={0};s.state=p[5];s.capabilities=p[6];s.reason=p[7];s.session=u32(p+8);s.sequence=u32(p+12);s.lease=u64(p+16);s.uptime_ms=u32(p+24);s.config_id=u32(p+28);
 for(int i=0;i<4;++i){s.target_centi[i]=signed16(u16(p+32+2*i));s.measured_centi[i]=signed16(u16(p+40+2*i));uint32_t raw=u32(p+48+4*i);memcpy(&s.counts[i],&raw,4);}
 s.valid_mask=(revision==4||revision==5)?p[64]&15:p[64];s.fault_mask=(revision==4||revision==5)?p[65]&15:p[65];s.max_ttl_ms=u16(p+66);s.max_rpm_centi=u16(p+68);s.revision=u16(p+70);s.accepted=u32(p+72);s.rejected=u32(p+76);s.watchdogs=u32(p+80);s.mode=(revision==4||revision==5)?p[84]&3:p[84];
 if(revision==4||revision==5){if(revision==4&&(p[6]&192u))return CH_FORMAT;if(revision==5)s.sonar_front_sequence=(p[6]>>6)&3;s.sonar_pending=(p[6]>>3)&7;s.sonar_age_bucket[0]=p[64]>>4;s.sonar_age_bucket[1]=p[65]>>4;s.sonar_age_bucket[2]=(p[84]>>2)&15;s.sonar_raw_zero_mask=((p[84]>>6)&3)|((p[90]>>5)&4);}
 if(revision==3||revision==4||revision==5){uint64_t packed=0;for(int i=0;i<5;i++)packed|=(uint64_t)p[85+i]<<(8*i);s.sonar_valid=(uint8_t)((packed>>36)&7);s.sonar_front_stable=(uint8_t)((packed>>39)&1);for(int i=0;i<3;i++){s.sonar_mm[i]=(uint16_t)((packed>>(12*i))&4095);if(((s.sonar_valid&(1u<<i))&&s.sonar_mm[i]==4095)||(!(s.sonar_valid&(1u<<i))&&s.sonar_mm[i]!=4095))return CH_FORMAT;}s.sonar_fresh=p[90]&7;s.sonar_near=(p[90]>>3)&7;s.sonar_degraded=(p[90]>>6)&1;s.local_stop_sources=p[91];}
 uint8_t checked[96];if(ch_status_encode(checked,&s)!=CH_OK)return CH_FORMAT;*out=s;return CH_OK;
}
int ch_output_command(const ch_output *o,uint64_t now,uint64_t lease,uint32_t config,ch_wire_command *out){
 if(!out)return CH_ARGUMENT;memset(out,0,sizeof(*out));
 if(!o||!lease||!config||!o->armed||o->reasons||!o->session||(o->mode!=CH_MANUAL&&o->mode!=CH_NAVIGATION)||
 (o->context.source!=CH_LIVE&&o->context.source!=CH_SIMULATION))return CH_READONLY;
 if(o->time_us>now||o->expires_us<=now||o->expires_us-now>CONT_COMMAND_US)return CH_TIME;
 uint64_t ttl=(o->expires_us-now)/1000;/* 期限向下取整，不向后延长。 */
 if(!ttl)return CH_TIME;
 out->op=CH_DRIVE;out->mode=(uint8_t)o->mode;out->session=o->session;out->sequence=o->sequence;
 out->lease=lease;out->ttl_ms=(uint16_t)ttl;out->config_id=config;
 for(int i=0;i<4;++i){double q=o->electrical_rpm[i]*100;if(!isfinite(q)||fabs(q)>32767){memset(out,0,sizeof(*out));return CH_ARGUMENT;}
  out->rpm_centi[i]=(int16_t)q;/* 轮速向零量化，不因舍入提高速度。 */}
 out->intent=(uint8_t)s1_classify_lr_centi(out->rpm_centi[0],-(int)out->rpm_centi[1]);return CH_OK;
}
static uint16_t checksum(const uint8_t *p,size_t size){uint32_t s=0;for(size_t i=0;i<size;i+=2)s+=(uint32_t)p[i]*256+p[i+1];while(s>>16)s=(s&65535)+(s>>16);return (uint16_t)~s;}
static void be16(uint8_t *p,uint16_t v){p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v;}
int ch_ethernet_command(uint8_t out[106],const uint8_t mac[6],const ch_wire_command *c){
 if(!out||!mac)return CH_ARGUMENT;uint8_t p[64];int r=ch_command_encode(p,c);if(r!=CH_OK)return r;
 memset(out,0,106);const uint8_t dst[6]={0,10,53,1,254,192},ips[8]={192,168,0,3,192,168,0,2};
 memcpy(out,dst,6);memcpy(out+6,mac,6);out[12]=8;out[14]=0x45;out[17]=92;out[20]=0x40;out[22]=64;out[23]=17;memcpy(out+26,ips,8);
 be16(out+24,checksum(out+14,20));be16(out+34,CH_COMMAND_PORT);be16(out+36,CH_COMMAND_PORT);be16(out+38,72);memcpy(out+42,p,64);return CH_OK;
}
int ch_ethernet_status(ch_wire_status *out,const uint8_t *f,size_t size){
 static const uint8_t ips[8]={192,168,0,2,192,168,0,3};
 static const uint8_t fpga[6]={0,10,53,1,254,192};
 if(!out||!f)return CH_ARGUMENT;
 /* 只接受无标签IPv4、头长5且无分片，网卡核验以太网FCS。 */
 if(size!=138||memcmp(f+6,fpga,6)||f[12]!=8||f[13]||f[14]!=0x45||f[16]||f[17]!=124||
 (f[20]!=0&&f[20]!=0x40)||f[21]||f[23]!=17||checksum(f+14,20)||memcmp(f+26,ips,8)||
 f[34]!=(CH_STATUS_PORT>>8)||f[35]!=(CH_STATUS_PORT&255)||f[36]!=f[34]||f[37]!=f[35]||
 f[38]||f[39]!=104||f[40]||f[41])return CH_FORMAT;
 return ch_status_decode(out,f+42,96);
}

int ch_diagnostic_encode(uint8_t out[96],const ch_wire_diagnostic*s){
 if(!out||!s||s->config_id!=S1_CONFIG_ID||(s->available_mask|s->zero_mask|s->fresh|s->near|s->pending|s->observation_block)>7||s->flags>127)return CH_ARGUMENT;
 for(unsigned i=0;i<4;i++)if(s->effective_pwm[i]>1750)return CH_ARGUMENT;
 memset(out,0,96);memcpy(out,"NCD5",4);out[4]=1;out[5]=5;out[6]=s->available_mask;out[7]=s->zero_mask;w32(out+8,s->config_id);w32(out+12,s->uptime_ms);
 for(unsigned i=0;i<3;i++){w16(out+16+i*2,s->raw_mm[i]);w32(out+24+i*4,s->nonzero_sequence[i]);w32(out+36+i*4,s->raw_age_us[i]);}
 for(unsigned i=0;i<4;i++)w16(out+48+i*2,s->effective_pwm[i]);
 out[56]=s->flags;out[57]=s->fresh;out[58]=s->near;out[59]=s->pending;out[60]=s->observation_block;w32(out+92,ch_crc32(out,92));return CH_OK;
}
int ch_diagnostic_decode(ch_wire_diagnostic*out,const uint8_t*p,size_t n){
 if(!out||!p)return CH_ARGUMENT;memset(out,0,sizeof(*out));
 if(n!=96||memcmp(p,"NCD5",4)||p[4]!=1||p[5]!=5||!zeros(p+22,2)||!zeros(p+61,31))return CH_FORMAT;
 if(ch_crc32(p,92)!=u32(p+92))return CH_CRC;
 ch_wire_diagnostic d={0};d.available_mask=p[6];d.zero_mask=p[7];d.config_id=u32(p+8);d.uptime_ms=u32(p+12);
 for(unsigned i=0;i<3;i++){d.raw_mm[i]=u16(p+16+i*2);d.nonzero_sequence[i]=u32(p+24+i*4);d.raw_age_us[i]=u32(p+36+i*4);}
 for(unsigned i=0;i<4;i++)d.effective_pwm[i]=u16(p+48+i*2);d.flags=p[56];d.fresh=p[57];d.near=p[58];d.pending=p[59];d.observation_block=p[60];
 uint8_t check[96];if(ch_diagnostic_encode(check,&d)!=CH_OK)return CH_FORMAT;*out=d;return CH_OK;
}
int ch_ethernet_diagnostic(ch_wire_diagnostic*out,const uint8_t*f,size_t n){
 static const uint8_t ips[8]={192,168,0,2,192,168,0,3},mac[6]={0,10,53,1,254,192};
 if(!out||!f)return CH_ARGUMENT;
 if(n!=138||memcmp(f+6,mac,6)||f[12]!=8||f[13]||f[14]!=0x45||f[16]||f[17]!=124||(f[20]!=0&&f[20]!=0x40)||f[21]||f[23]!=17||checksum(f+14,20)||memcmp(f+26,ips,8)||f[34]!=(CH_STATUS_PORT>>8)||f[35]!=(CH_STATUS_PORT&255)||f[36]!=f[34]||f[37]!=f[35]||f[38]||f[39]!=104||f[40]||f[41])return CH_FORMAT;
 return ch_diagnostic_decode(out,f+42,96);
}
