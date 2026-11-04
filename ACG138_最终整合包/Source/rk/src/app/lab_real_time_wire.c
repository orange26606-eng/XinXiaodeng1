#include "lab_real_time.h"
#include "lab_codec.h"
#include <math.h>
#include <string.h>
_Static_assert(sizeof(double)==8,"IEEE754 64-bit double required");
static void p32(uint8_t *p,uint32_t n){for(int i=3;i>=0;--i){p[i]=(uint8_t)n;n>>=8;}}
static uint32_t g32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static void p64(uint8_t *p,uint64_t n){p32(p,(uint32_t)(n>>32));p32(p+4,(uint32_t)n);}
static uint64_t g64(const uint8_t *p){return (uint64_t)g32(p)<<32|g32(p+4);}
static void pf(uint8_t *p,double x){uint64_t n;memcpy(&n,&x,8);p64(p,n);}
static double gf(const uint8_t *p){uint64_t n=g64(p);double x;memcpy(&x,&n,8);return x;}
static int scan_valid(const lab_real_scan_time *s){return s&&s->session&&s->count&&s->count<SCAN_MAX_POINTS&&s->sample_period_ns&&s->sample_period_ns<=100000000&&s->first_us<=s->last_us&&s->last_us<=s->fetched_us&&s->legacy_end_us==s->fetched_us&&s->last_us-s->first_us==(uint64_t)(s->count-1)*s->sample_period_ns/1000&&s->last_us-s->first_us<=1000000&&s->fetched_us-s->first_us<=2000000;}
lab_status lab_real_scan_time_encode(uint8_t out[112],const lab_real_scan_time *s){if(!out||!scan_valid(s))return LAB_ERR_ARGUMENT;memset(out,0,112);memcpy(out,"ST01",4);p32(out+4,112);p32(out+8,s->session);p32(out+12,s->sequence);p64(out+16,s->legacy_end_us);p64(out+24,s->first_us);p64(out+32,s->last_us);p64(out+40,s->fetched_us);p32(out+48,s->sample_period_ns);p32(out+52,s->count);p32(out+56,s->model);p32(out+60,s->firmware);p32(out+64,1);p32(out+108,lab_crc32(out,108));return LAB_OK;}
lab_status lab_real_scan_time_decode(lab_real_scan_time *s,const uint8_t *p,size_t n){if(!s||!p)return LAB_ERR_ARGUMENT;memset(s,0,sizeof(*s));if(n!=112||memcmp(p,"ST01",4)||g32(p+4)!=112||g32(p+64)!=1)return LAB_ERR_FORMAT;if(lab_crc32(p,108)!=g32(p+108))return LAB_ERR_CRC;for(unsigned i=68;i<108;++i)if(p[i])return LAB_ERR_FORMAT;*s=(lab_real_scan_time){.session=g32(p+8),.sequence=g32(p+12),.count=g32(p+52),.sample_period_ns=g32(p+48),.model=g32(p+56),.firmware=g32(p+60),.legacy_end_us=g64(p+16),.first_us=g64(p+24),.last_us=g64(p+32),.fetched_us=g64(p+40)};return scan_valid(s)?LAB_OK:LAB_ERR_FORMAT;}
lab_status lab_real_scan_time_apply(const lab_real_scan_time *s,const lab_real_time_profile *p,lab_live_sample *e){
 if(!scan_valid(s)||!lab_real_time_profile_valid(p)||!lab_live_sample_valid(e)||e->type!=LAB_EVENT_SCAN||e->meta.provider!=LAB_REAL_USB)return LAB_ERR_ARGUMENT;
 if(e->meta.parameter_version!=p->parameter_version||s->session!=e->meta.stream_session||s->sequence!=e->meta.sequence||s->count!=e->data.scan.count||s->legacy_end_us!=e->data.scan.timestamp_us)return LAB_ERR_CONTEXT;if(s->fetched_us>e->meta.received_us)return LAB_ERR_TIME;if(p->sdk_time_declared&&(s->model!=p->lidar_model||(p->lidar_firmware&&s->firmware!=p->lidar_firmware)))return LAB_ERR_VERSION;
 uint64_t span=s->last_us-s->first_us;e->meta.time_us=s->first_us+span/2;e->meta.flags|=LAB_REAL_TIME_ESTIMATED|LAB_REAL_TIME_VALID;
 /* 整扫仍只有一个参考位姿，误差计入半扫时间，不表示逐点运动补偿。
 */
 e->meta.time_uncertainty_us=p->sdk_time_declared?p->sdk_time_error_us+(span+1)/2:UINT64_MAX;return LAB_OK;
}
static int odom_valid(const lab_real_odom_time *s){const lab_pose *p=s?&s->pose:NULL;return s&&s->producer_id&&s->session&&s->parameter_version&&s->counts_per_revolution&&s->counts_per_revolution<=1000000&&s->received_us<=s->published_us&&!(s->generation&1)&&s->generation&&lab_real_od01_valid(s->raw_od01,64)&&isfinite(p->x_m)&&isfinite(p->y_m)&&isfinite(p->yaw_rad)&&isfinite(p->distance_m)&&p->distance_m>=0&&isfinite(p->uncertainty_m)&&p->uncertainty_m>=0&&!(p->quality&~63u)&&p->calibrated<=1;}
lab_status lab_real_odom_time_encode(uint8_t out[256],const lab_real_odom_time *s){if(!out||!odom_valid(s))return LAB_ERR_ARGUMENT;memset(out,0,256);memcpy(out,"ODT1",4);p32(out+4,256);p64(out+8,s->producer_id);p32(out+16,s->session);p32(out+20,s->accepted_sequence);p64(out+24,s->parameter_version);p32(out+32,s->counts_per_revolution);p64(out+40,s->received_us);p64(out+48,s->published_us);p32(out+56,s->config_crc32);memcpy(out+64,s->raw_od01,64);pf(out+128,s->pose.x_m);pf(out+136,s->pose.y_m);pf(out+144,s->pose.yaw_rad);pf(out+152,s->pose.distance_m);p32(out+160,s->pose.quality);p32(out+164,s->pose.calibrated);pf(out+168,s->pose.uncertainty_m);p32(out+180,s->generation);p32(out+252,lab_crc32(out,252));return LAB_OK;}
lab_status lab_real_odom_time_decode(lab_real_odom_time *s,const uint8_t *p,size_t n){if(!s||!p)return LAB_ERR_ARGUMENT;memset(s,0,sizeof(*s));if(n!=256||memcmp(p,"ODT1",4)||g32(p+4)!=256)return LAB_ERR_FORMAT;if(g32(p+180)&1)return LAB_AGAIN;if(lab_crc32(p,252)!=g32(p+252))return LAB_ERR_CRC;for(unsigned i=36;i<40;++i)if(p[i])return LAB_ERR_FORMAT;for(unsigned i=60;i<64;++i)if(p[i])return LAB_ERR_FORMAT;for(unsigned i=176;i<180;++i)if(p[i])return LAB_ERR_FORMAT;for(unsigned i=184;i<252;++i)if(p[i])return LAB_ERR_FORMAT;
 s->producer_id=g64(p+8);s->session=g32(p+16);s->accepted_sequence=g32(p+20);s->parameter_version=g64(p+24);s->counts_per_revolution=g32(p+32);s->received_us=g64(p+40);s->published_us=g64(p+48);s->config_crc32=g32(p+56);s->generation=g32(p+180);memcpy(s->raw_od01,p+64,64);s->pose=(lab_pose){gf(p+128),gf(p+136),gf(p+144),gf(p+152),g32(p+160),g32(p+164),gf(p+168)};return odom_valid(s)?LAB_OK:LAB_ERR_FORMAT;
}
