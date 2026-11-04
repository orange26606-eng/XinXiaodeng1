#include "lab_real_time.h"
#include "lab_codec.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t le32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
int lab_real_od01_valid(const uint8_t *p,size_t n){return p&&n==64&&!memcmp(p,"OD01",4)&&p[4]==1&&p[5]<=1&&p[6]<=15&&p[7]<=7&&!le32(p+52)&&!le32(p+56)&&lab_crc32(p,60)==le32(p+60);}
void lab_real_time_profile_default(lab_real_time_profile *p){if(!p)return;memset(p,0,sizeof(*p));p->parameter_version=1;p->counts_per_revolution=1320;p->rate_error_ppm=500;p->warmup_samples=16;}
int lab_real_time_profile_valid(const lab_real_time_profile *p){return p&&p->parameter_version&&p->counts_per_revolution&&p->counts_per_revolution<=1000000&&p->rate_error_ppm<=10000&&p->warmup_samples>=2&&p->warmup_samples<=1000&&p->fpga_delay_declared<=1&&p->fpga_stamp_declared<=1&&p->sdk_time_declared<=1&&p->fpga_delay_min_us<=p->fpga_delay_max_us&&p->fpga_delay_max_us<=1000000&&(!p->sdk_time_declared||(p->sdk_time_error_us&&p->sdk_time_error_us<=1000000&&p->lidar_model));}
lab_status lab_real_time_profile_load(const char *path,lab_real_time_profile *out){
 if(!path||!out)return LAB_ERR_ARGUMENT;FILE *f=fopen(path,"rb");if(!f)return LAB_ERR_IO;char data[8193];size_t n=fread(data,1,8193,f);int err=ferror(f);fclose(f);if(err)return LAB_ERR_IO;if(n>8192)return LAB_ERR_LIMIT;if(memchr(data,0,n))return LAB_ERR_FORMAT;data[n]=0;uint32_t fingerprint=lab_crc32(data,n);
 lab_real_time_profile p;lab_real_time_profile_default(&p);uint32_t seen=0;unsigned version=0;char *line=data;
 while(*line){char *end=strchr(line,'\n');if(end)*end=0;char *hash=strchr(line,'#');if(hash)*hash=0;char *a=line;while(*a==' '||*a=='\t'||*a=='\r')++a;if(*a){char *eq=strchr(a,'=');if(!eq||eq==a)return LAB_ERR_FORMAT;*eq=0;char *z=eq-1;while(z>=a&&(*z==' '||*z=='\t'))*z--=0;char *v=eq+1;while(*v==' '||*v=='\t')++v;if(*v=='-'||*v=='+')return LAB_ERR_FORMAT;errno=0;char *tail;unsigned long long x=strtoull(v,&tail,!strcmp(a,"odometry_config_crc32")?0:10);while(*tail==' '||*tail=='\t'||*tail=='\r')++tail;if(errno||tail==v||*tail)return LAB_ERR_FORMAT;
  unsigned bit=0;
  if(!strcmp(a,"version")){bit=1u;version=(unsigned)x;if(x!=1)return LAB_ERR_VERSION;}
  else if(!strcmp(a,"parameter_version")){bit=2u;p.parameter_version=x;}
  else if(!strcmp(a,"counts_per_revolution")){bit=4u;if(x>UINT32_MAX)return LAB_ERR_LIMIT;p.counts_per_revolution=(uint32_t)x;}
  else if(!strcmp(a,"rate_error_ppm")){bit=8u;if(x>UINT32_MAX)return LAB_ERR_LIMIT;p.rate_error_ppm=(uint32_t)x;}
  else if(!strcmp(a,"warmup_samples")){bit=16u;if(x>UINT32_MAX)return LAB_ERR_LIMIT;p.warmup_samples=(uint32_t)x;}
  else if(!strcmp(a,"fpga_delay_declared")){bit=32u;if(x>1)return LAB_ERR_ARGUMENT;p.fpga_delay_declared=(uint32_t)x;}
  else if(!strcmp(a,"fpga_stamp_declared")){bit=64u;if(x>1)return LAB_ERR_ARGUMENT;p.fpga_stamp_declared=(uint32_t)x;}
  else if(!strcmp(a,"fpga_delay_min_us")){bit=128u;p.fpga_delay_min_us=x;}
  else if(!strcmp(a,"fpga_delay_max_us")){bit=256u;p.fpga_delay_max_us=x;}
  else if(!strcmp(a,"sdk_time_declared")){bit=512u;if(x>1)return LAB_ERR_ARGUMENT;p.sdk_time_declared=(uint32_t)x;}
  else if(!strcmp(a,"sdk_time_error_us")){bit=1024u;p.sdk_time_error_us=x;}
  else if(!strcmp(a,"lidar_model")){bit=2048u;if(x>UINT32_MAX)return LAB_ERR_LIMIT;p.lidar_model=(uint32_t)x;}
  else if(!strcmp(a,"lidar_firmware")){bit=4096u;if(x>UINT32_MAX)return LAB_ERR_LIMIT;p.lidar_firmware=(uint32_t)x;}
  else if(!strcmp(a,"odometry_config_crc32")){bit=8192u;if(x>UINT32_MAX)return LAB_ERR_LIMIT;p.odometry_config_crc32=(uint32_t)x;}
  else return LAB_ERR_FORMAT;
  if(seen&bit)return LAB_ERR_FORMAT;seen|=bit;
 }if(!end)break;line=end+1;}
 if(version!=1||!lab_real_time_profile_valid(&p))return LAB_ERR_ARGUMENT;
 p.profile_crc32=fingerprint;*out=p;return LAB_OK;
}
void lab_real_clock_init(lab_real_time_clock *c,const lab_real_time_profile *p){if(c){memset(c,0,sizeof(*c));if(p)c->profile=*p;snprintf(c->reason,sizeof(c->reason),"CLOCK_WAIT_FIRST_SAMPLE");}}
static lab_status fail(lab_real_time_clock *c,const char *reason){c->failed=1;snprintf(c->reason,sizeof(c->reason),"%s",reason);return LAB_ERR_CONTEXT;}
lab_status lab_real_clock_observe(lab_real_time_clock *c,const uint8_t p[64],uint64_t rx,uint64_t *t,uint64_t *error){
 if(t)*t=0;if(error)*error=UINT64_MAX;if(!c||!t||!error||!lab_real_time_profile_valid(&c->profile)||!lab_real_od01_valid(p,64))return LAB_ERR_ARGUMENT;if(c->failed)return LAB_ERR_CONTEXT;
 uint32_t seq=le32(p+8),ms=le32(p+12),epoch=le32(p+16),token=le32(p+48),delta=0;
 if(c->have){if(epoch!=c->epoch||token!=c->imu_token)return fail(c,"CLOCK_SOURCE_SESSION_CHANGED");if(rx<c->received_us)return fail(c,"CLOCK_HOST_ROLLBACK");uint32_t ds=seq-c->sequence;if(!ds||ds>=UINT32_C(0x80000000))return LAB_AGAIN;delta=ms-c->remote_ms;if(delta>=UINT32_C(0x80000000))return fail(c,"CLOCK_REMOTE_ROLLBACK");}
 if(rx>UINT64_MAX-10000000)return LAB_ERR_LIMIT;
 int enabled=c->profile.fpga_delay_declared&&c->profile.fpga_stamp_declared;
 if(enabled){
  /* OD01以毫秒量化，先约束其量化窗口起点在RK的时刻，输出再计入完整1000us量化区间。
 */
  uint64_t max=c->profile.fpga_delay_max_us,min=c->profile.fpga_delay_min_us;
  if(rx<max+1000)return fail(c,"CLOCK_DELAY_BEFORE_HOST_ORIGIN");uint64_t lo=rx-max-1000,hi=rx-min;
  if(c->have){uint64_t dt=(uint64_t)delta*1000;uint64_t drift=(dt*c->profile.rate_error_ppm+999999)/1000000;
   if(c->upper_us>UINT64_MAX-dt-drift)return LAB_ERR_LIMIT;uint64_t predicted_lo=c->lower_us+dt,predicted_hi=c->upper_us+dt+drift;predicted_lo=predicted_lo>drift?predicted_lo-drift:0;
   if(predicted_lo>lo)lo=predicted_lo;if(predicted_hi<hi)hi=predicted_hi;
  }
  if(lo>hi)return fail(c,"CLOCK_DELAY_OR_RATE_BOUND_VIOLATED");c->lower_us=lo;c->upper_us=hi;
 }
 c->have=1;c->sequence=seq;c->remote_ms=ms;c->epoch=epoch;c->imu_token=token;c->unwrapped_ms+=delta;c->received_us=rx;++c->samples;*t=rx;
 if(!enabled){snprintf(c->reason,sizeof(c->reason),"%s",!c->profile.fpga_stamp_declared?"FPGA_SAMPLE_TIME_UNCONFIRMED":"CLOCK_DELAY_BOUND_UNCONFIRMED");return LAB_OK;}
 if(c->samples<c->profile.warmup_samples){snprintf(c->reason,sizeof(c->reason),"CLOCK_WARMING_UP");return LAB_AGAIN;}
 uint64_t hi=c->upper_us+1000;if(hi>rx)hi=rx;uint64_t lo=c->lower_us;if(lo>hi)return fail(c,"CLOCK_EMPTY_SAMPLE_INTERVAL");uint64_t mid=lo+(hi-lo)/2;
 if(c->last_estimate_us&&mid<c->last_estimate_us){if(c->last_estimate_us>hi)return fail(c,"CLOCK_REFINEMENT_NONMONOTONIC");mid=c->last_estimate_us;}
 *t=mid;*error=mid-lo>hi-mid?mid-lo:hi-mid;if(!*error)*error=1;c->last_estimate_us=mid;snprintf(c->reason,sizeof(c->reason),"CLOCK_BOUNDED_BY_DECLARED_PROFILE");return LAB_OK;
}
