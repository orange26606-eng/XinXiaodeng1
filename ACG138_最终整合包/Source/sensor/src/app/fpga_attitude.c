#include "fpga_attitude.h"
#include "lab_codec.h"
#include <math.h>
#include <string.h>
#define F6_PI 3.14159265358979323846
static uint32_t f6_u32(const uint8_t*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static double f6_q24(const uint8_t*p){uint32_t raw=f6_u32(p);int32_t v;memcpy(&v,&raw,4);return v/16777216.0;}
static double f6_wrap(double a){return atan2(sin(a),cos(a));}
static void f6_quaternion(const double*r,double*q){double cr=cos(r[0]*.5),sr=sin(r[0]*.5),cp=cos(r[1]*.5),sp=sin(r[1]*.5),cy=cos(r[2]*.5),sy=sin(r[2]*.5);q[0]=cr*cp*cy+sr*sp*sy;q[1]=sr*cp*cy-cr*sp*sy;q[2]=cr*sp*cy+sr*cp*sy;q[3]=cr*cp*sy-sr*sp*cy;}
/* F6四元数含内部初始化偏航，发布的相对偏航为准，先构造等效旋转再做带符号轴映射。
 */
static void f6_body_rpy(const f6_selector*s,const f6_attitude*a,double*r){
 double q[4];f6_quaternion(a->rpy,q);double w=q[0],x=q[1],y=q[2],z=q[3];
 double rs[3][3]={{1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)},{2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)},{2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)}};
 double b[3][3];for(int k=0;k<3;k++){int axis=s->axes[k],idx=(axis<0?-axis:axis)-1;for(int j=0;j<3;j++)b[j][k]=(axis<0?-1:1)*rs[j][idx];}
 r[0]=atan2(b[2][1],b[2][2]);r[1]=asin(fmax(-1,fmin(1,-b[2][0])));r[2]=atan2(b[1][0],b[0][0]);
}
int f6_decode(f6_attitude*a,const uint8_t*p,size_t n){
 if(!a||!p||n!=100||memcmp(p,"F6A1",4)||p[4]!=1||(p[5]&~7u)||p[7]>6||p[83])return -1;
 if(f6_u32(p+96)!=lab_crc32(p,96))return -2;
 memset(a,0,sizeof(*a));a->flags=p[5];a->whoami=p[6];a->status=p[7];a->sequence=f6_u32(p+8);a->source_us=f6_u32(p+12);a->generated_us=f6_u32(p+16);a->raw_sequence=f6_u32(p+20);
 for(unsigned k=0;k<4;k++)a->q[k]=f6_q24(p+24+4*k);
 for(unsigned k=0;k<3;k++){a->rpy[k]=f6_q24(p+40+4*k);a->bias_lsb[k]=f6_q24(p+52+4*k)*16777216.;if(fabs(a->rpy[k])>F6_PI+1e-5)return -1;}
 a->i2c_errors=f6_u32(p+64);a->math_errors=f6_u32(p+68);a->gaps=f6_u32(p+72);a->age_us=f6_u32(p+76);a->cal_samples=(uint16_t)(p[80]|p[81]<<8);a->i2c_error=p[82];a->tx_drops=f6_u32(p+84);a->latest_source_us=f6_u32(p+88);a->epoch=f6_u32(p+92);
 if(a->age_us!=a->generated_us-a->source_us)return -1;
 if(a->flags&1){double norm=0;for(unsigned k=0;k<4;k++)norm+=a->q[k]*a->q[k];if((a->flags&3)!=3||a->whoami!=0x71||a->status!=2||norm<.98||norm>1.02)return -1;}
 return 0;
}
void f6_selector_init(f6_selector*s,unsigned mode,const int axes[3]){memset(s,0,sizeof(*s));s->mode=mode;for(unsigned k=0;k<3;k++)s->axes[k]=axes[k];strcpy(s->reason,mode==F6_MODE_RK?"RK_EXPLICIT":"WAIT_FPGA_PAIR");}
void f6_selector_clear(f6_selector*s){f6_selector old=*s;unsigned mode=s->mode;int axes[3];memcpy(axes,s->axes,sizeof axes);f6_selector_init(s,mode,axes);
 s->accepted=old.accepted;s->rejected=old.rejected;s->crc_errors=old.crc_errors;s->duplicates=old.duplicates;s->reorders=old.reorders;s->matches=old.matches;s->misses=old.misses;s->stale=old.stale;s->source_switches=old.source_switches;
}
int f6_selector_push(f6_selector*s,const uint8_t*p,size_t n,uint64_t rx){
 f6_attitude a;int code=f6_decode(&a,p,n);if(code){++s->rejected;if(code==-2)++s->crc_errors;return code;}
 if(s->have_last&&rx<s->last_received_us){++s->rejected;return -1;}
 /* 心跳仅更新链路状态，测量缓存保留采集时间。 */
 for(unsigned k=0;k<F6_CACHE_CAPACITY;k++){const f6_cache_entry*e=&s->cache[k];if(e->used&&e->sample.epoch==a.epoch&&e->sample.sequence==a.sequence&&e->sample.source_us==a.source_us){++s->duplicates;s->last=a;s->last_received_us=rx;s->have_last=1;return 0;}}
 if(s->have_last&&a.epoch==s->last.epoch&&(int32_t)(a.sequence-s->last.sequence)<0)++s->reorders;
 f6_cache_entry*e=&s->cache[s->cursor++%F6_CACHE_CAPACITY];e->used=1;e->sample=a;e->received_us=rx;
 s->last=a;s->last_received_us=rx;s->have_last=1;++s->accepted;return 1;
}
const f6_cache_entry*f6_selector_find(const f6_selector*s,const struct i9_packet*p,uint64_t now){
 if((p->flags&(I9_AG_VALID|I9_MPU_READY))!=(I9_AG_VALID|I9_MPU_READY)||p->whoami!=0x71||p->snapshot_us-p->ag_us>F6_MAX_AGE_US)return NULL;
 for(unsigned k=0;k<F6_CACHE_CAPACITY;k++){const f6_cache_entry*e=&s->cache[k];if(!e->used||e->sample.epoch!=p->reset_epoch||e->sample.sequence!=p->ag_sequence||e->sample.source_us!=p->ag_us)continue;
  if((e->sample.flags&3)!=3||e->sample.status!=2||e->sample.whoami!=0x71||e->sample.age_us>F6_MAX_AGE_US||now<e->received_us||now-e->received_us>F6_MAX_RX_AGE_US)return NULL;
  return e;
 }return NULL;
}
const char*f6_source_name(unsigned source){return source==F6_SOURCE_FPGA?"FPGA6":source==F6_SOURCE_RK?"RK":source==F6_SOURCE_UNAVAILABLE?"UNAVAILABLE":"SNAPSHOT_SOURCE_NOT_ENCODED";}
int f6_selector_hook(void*u,const struct i9_packet*p,const struct i9_state*f,struct mpu_attitude_selection*out){
 f6_selector*s=u;double rpy[3]={f->roll,f->pitch,f->yaw};unsigned source=F6_SOURCE_RK,bias=f->bias_ready,cal=f->cal_count;const char*reason=s->mode==F6_MODE_RK?"RK_EXPLICIT":"NO_MATCHING_FPGA_SAMPLE_RK_FALLBACK";
 const f6_cache_entry*entry=s->mode==F6_MODE_RK?NULL:f6_selector_find(s,p,s->raw_received_us+s->raw_wait_us);
 s->raw_epoch=p->reset_epoch;s->raw_ag_sequence=p->ag_sequence;s->raw_source_us=p->ag_us;
 if(s->mode!=F6_MODE_RK&&!entry){
  if((p->flags&(I9_AG_VALID|I9_MPU_READY))!=(I9_AG_VALID|I9_MPU_READY)||p->whoami!=0x71||p->snapshot_us-p->ag_us>F6_MAX_AGE_US)reason="RAW_AG_INVALID_RK_FALLBACK";
  else for(unsigned k=0;k<F6_CACHE_CAPACITY;k++){const f6_cache_entry*e=&s->cache[k];if(e->used&&e->sample.epoch==p->reset_epoch&&e->sample.sequence==p->ag_sequence&&e->sample.source_us==p->ag_us){reason=(e->sample.flags&3)!=3||e->sample.status!=2?"FPGA_STATUS_NOT_READY_RK_FALLBACK":"FPGA_PAIR_STALE_RK_FALLBACK";++s->stale;break;}}
 }

 memset(&s->paired,0,sizeof s->paired);s->paired_received_us=0;
 if(entry){s->paired=entry->sample;s->paired_received_us=entry->received_us;f6_body_rpy(s,&entry->sample,rpy);source=F6_SOURCE_FPGA;bias=1;cal=entry->sample.cal_samples;reason="EXACT_EPOCH_AG_SEQUENCE_SOURCE_TIME";++s->matches;}
 else if(s->mode==F6_MODE_FPGA){source=F6_SOURCE_UNAVAILABLE;bias=0;reason="FPGA_REQUIRED_PAIR_UNAVAILABLE";++s->misses;}
 else if(s->mode!=F6_MODE_RK)++s->misses;
 /* 切换姿态源时保持航向，不重复积分同一偏航；原始epoch变化或重启在调用前先重置状态。
 */
 double reference=s->last_yaw+(s->have_rk_yaw?f6_wrap(f->yaw-s->previous_rk_yaw):0);
 if(source!=F6_SOURCE_UNAVAILABLE){if(s->have_yaw&&s->source!=source)s->yaw_offset=f6_wrap(reference-rpy[2]);else if(!s->have_yaw)s->yaw_offset=0;if(s->yaw_offset!=0)rpy[2]=f6_wrap(rpy[2]+s->yaw_offset);s->last_yaw=rpy[2];s->have_yaw=1;}
 s->previous_rk_yaw=f->yaw;s->have_rk_yaw=1;
 if(s->source&&s->source!=source)++s->source_switches;s->source=source;s->selected_valid=bias&&source!=F6_SOURCE_UNAVAILABLE;s->selected_bias=bias;s->selected_cal=cal;memcpy(s->selected_rpy,rpy,sizeof rpy);if(source==F6_SOURCE_RK&&s->yaw_offset==0)memcpy(s->selected_q,f->q,sizeof s->selected_q);else f6_quaternion(rpy,s->selected_q);strcpy(s->reason,reason);
 memset(out,0,sizeof(*out));out->valid=1;out->bias_ready=bias;out->cal_samples=cal;out->source=source;memcpy(out->rpy,rpy,sizeof rpy);memcpy(out->q,s->selected_q,sizeof out->q);
 if(entry){for(unsigned k=0;k<3;k++){int axis=s->axes[k],index=(axis<0?-axis:axis)-1;out->bias[k]=(axis<0?-1:1)*entry->sample.bias_lsb[index]*(F6_PI/180./65.5);}}
 else memcpy(out->bias,f->bias,sizeof out->bias);return 1;
}
