#include "lab_real_mpu.h"
#include "lab_real_mpu_vendor.h"
#include "lab_codec.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
struct lab_real_mpu {
 lab_real_mpu_config config;lab_real_context context;uint64_t version;
 struct mpu_adapter adapter;struct scan_receiver rx;
 lab_live_sample *queue,*scratch;unsigned head,count;uint64_t delivery_us;
 lab_real_mpu_stats stats;
 int have_clock,have_scan,have_timing;uint32_t raw_us,sequence,epoch,scan_session,scan_sequence;
 uint64_t remote_us,last_rx;int64_t low,high;
 uint32_t timing_session,timing_sequence,span,last_ag_sequence;int have_ag_sequence;uint64_t first,returned;
};
static uint32_t be32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static uint64_t be64(const uint8_t *p){return (uint64_t)be32(p)<<32|be32(p+4);}
static void put32(uint8_t *p,uint32_t n){for(int i=3;i>=0;--i){p[i]=(uint8_t)n;n>>=8;}}
static void put64(uint8_t *p,uint64_t n){put32(p,(uint32_t)(n>>32));put32(p+4,(uint32_t)n);}
static void why(lab_real_mpu *s,const char *r){snprintf(s->stats.reason,sizeof(s->stats.reason),"%s",r);}
void lab_real_mpu_config_default(lab_real_mpu_config *c){if(!c)return;memset(c,0,sizeof(*c));strcpy(c->bind_ip,"192.168.0.3");strcpy(c->peer_ip,"192.168.0.2");c->port=5016;c->drift_ppm=200;c->declared_version=1;}
/* 参数标识使用规范字节和非密码学校验，不包含结构填充；发布源码与程序另用SHA256核对。
 */
static uint64_t hash_bytes(uint64_t h,const void *data,size_t n){const uint8_t *p=data;for(size_t i=0;i<n;++i){h^=p[i];h*=UINT64_C(1099511628211);}return h;}
uint64_t lab_real_mpu_identity(const lab_real_mpu_config *c){
 if(!c)return 0;uint64_t h=UINT64_C(14695981039346656037);const uint64_t values[]={1,c->declared_version,c->experimental,c->delay_min_us,c->delay_max_us,c->lidar_error_us,c->imu_error_us,c->drift_ppm};uint8_t b[8];
 for(unsigned i=0;i<sizeof(values)/sizeof(values[0]);++i){put64(b,values[i]);h=hash_bytes(h,b,8);}for(unsigned i=0;i<3;++i){uint64_t v;memcpy(&v,&c->mount[i],8);put64(b,v);h=hash_bytes(h,b,8);}
 const char *paths[]={c->odom_config,c->imu_config,c->timing_profile};
 for(unsigned i=0;i<3;++i){uint8_t tag=(uint8_t)i;h=hash_bytes(h,&tag,1);if(!paths[i][0]){h=hash_bytes(h,"DEFAULT",7);continue;}FILE *f=fopen(paths[i],"rb");if(!f)return 0;uint8_t data[1024];size_t total=0,n;
  while((n=fread(data,1,sizeof(data),f))){total+=n;if(total>65536){fclose(f);return 0;}h=hash_bytes(h,data,n);}int bad=ferror(f);fclose(f);if(bad)return 0;put64(b,total);h=hash_bytes(h,b,8);
 }return h?h:1;
}
lab_status lab_real_mpu_profile(lab_real_mpu_config *c,const char *path){
 if(!c||!path||strlen(path)>=sizeof(c->timing_profile))return LAB_ERR_ARGUMENT;FILE *f=fopen(path,"r");if(!f)return LAB_ERR_IO;lab_real_mpu_config next=*c;char line[256],key[64],tail;unsigned seen=0;lab_status result=LAB_ERR_FORMAT;
 while(fgets(line,sizeof(line),f)){if(line[0]=='#'||line[0]=='\n'||line[0]=='\r')continue;unsigned long long v;
  if(sscanf(line," %63[^=]= %llu %c",key,&v,&tail)!=2)goto done;
  unsigned bit=0;if(!strcmp(key,"experimental")){bit=1;if(v!=1)goto done;next.experimental=1;}
  else if(!strcmp(key,"delay_min_us")){bit=2;next.delay_min_us=v;}
  else if(!strcmp(key,"delay_max_us")){bit=4;next.delay_max_us=v;}
  else if(!strcmp(key,"lidar_error_us")){bit=8;next.lidar_error_us=v;}
  else if(!strcmp(key,"imu_error_us")){bit=16;next.imu_error_us=v;}
  else if(!strcmp(key,"drift_ppm")){bit=32;if(v>1000)goto done;next.drift_ppm=(uint32_t)v;}else goto done;
  if(seen&bit)goto done;seen|=bit;
 }
 if(ferror(f)||seen!=63||next.delay_max_us<next.delay_min_us||next.delay_max_us>100000||!next.lidar_error_us||next.lidar_error_us>50000||!next.imu_error_us||next.imu_error_us>50000)goto done;
 strcpy(next.timing_profile,path);*c=next;result=LAB_OK;
done:fclose(f);return result;
}
lab_status lab_real_mpu_core_open(lab_real_mpu **out,const lab_real_mpu_config *c,const lab_real_context *ctx,uint64_t version){
 if(!out)return LAB_ERR_ARGUMENT;*out=NULL;if(!c||!lab_real_context_valid(ctx)||ctx->clock!=LAB_REAL_MONOTONIC||!version||c->experimental>1||c->drift_ppm>1000||c->delay_max_us<c->delay_min_us||c->delay_max_us>100000)return LAB_ERR_ARGUMENT;
 lab_real_mpu *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->config=*c;s->context=*ctx;s->version=version;s->queue=calloc(8,sizeof(*s->queue));s->scratch=calloc(1,sizeof(*s->scratch));if(!s->queue||!s->scratch){lab_real_mpu_core_close(s);return LAB_ERR_NOMEM;}
 struct odom_config odom;odom_default_config(&odom);struct i9_config imu;i9_default_config(&imu);
 if((c->odom_config[0]&&odom_load_config(&odom,c->odom_config))||(c->imu_config[0]&&mpu_config_load(&imu,c->imu_config))||mpu_adapter_init(&s->adapter,&odom,&imu)){lab_real_mpu_core_close(s);return LAB_ERR_FORMAT;}
 s->stats.have_profile=c->experimental;why(s,c->experimental?"EXPERIMENTAL_TIMING_NOT_BOARD_VERIFIED":"TIMING_UNKNOWN_READONLY");*out=s;return LAB_OK;
}
static lab_status clock_map(lab_real_mpu *s,const struct i9_packet *p,uint64_t rx,uint64_t *time,uint64_t *error){
 if(rx>INT64_MAX-1000000)return LAB_ERR_TIME;
 if(s->have_clock){uint32_t d=p->sequence-s->sequence;
  if(p->reset_epoch!=s->epoch||rx<s->last_rx){s->stats.clock_fault=1;why(s,"MPU_RESET_NEW_CONTEXT_REQUIRED");return LAB_ERR_CONTEXT;}
  if(d>=UINT32_C(0x80000000)&&p->sequence<1000&&p->snapshot_us<1000000&&rx-s->last_rx>300000){s->stats.clock_fault=1;why(s,"MPU_REBOOT_NEW_CONTEXT_REQUIRED");return LAB_ERR_CONTEXT;}
  if(!d||d>=UINT32_C(0x80000000)){++s->stats.duplicates;return LAB_AGAIN;}
  uint32_t dt=p->snapshot_us-s->raw_us;if(!dt||dt>=UINT32_C(0x80000000)){s->stats.clock_fault=1;why(s,"MPU_CLOCK_RESTART_NEW_CONTEXT_REQUIRED");return LAB_ERR_CONTEXT;}
  if(d>1)s->stats.gaps+=d-1;s->remote_us+=dt;
  int64_t drift=(int64_t)ceil((double)dt*s->config.drift_ppm/1000000.0)+1;s->low-=drift;s->high+=drift;
 }else s->remote_us=p->snapshot_us;
 if(s->remote_us>INT64_MAX-1000000)return LAB_ERR_TIME;
 if(s->config.experimental){
  if(rx<s->config.delay_max_us)return LAB_ERR_TIME;
  int64_t lo=(int64_t)(rx-s->config.delay_max_us)-(int64_t)s->remote_us,hi=(int64_t)(rx-s->config.delay_min_us)-(int64_t)s->remote_us;
  if(!s->have_clock){s->low=lo;s->high=hi;}else{s->low=s->low>lo?s->low:lo;s->high=s->high<hi?s->high:hi;}
  if(s->low>s->high){s->stats.clock_fault=1;why(s,"LATENCY_PROFILE_VIOLATED_NEW_CONTEXT_REQUIRED");return LAB_ERR_CONTEXT;}
  int64_t mapped=(int64_t)s->remote_us+s->low;if(mapped<0||(uint64_t)mapped>rx)return LAB_ERR_TIME;
  *time=(uint64_t)mapped;*error=(uint64_t)(s->high-s->low)+1;
 }else{*time=rx;*error=UINT64_MAX;}
 s->have_clock=1;s->raw_us=p->snapshot_us;s->sequence=p->sequence;s->epoch=p->reset_epoch;s->last_rx=rx;return LAB_OK;
}
static lab_real_meta metadata(lab_real_mpu *s,uint32_t provider,uint32_t stream,uint32_t sequence,uint64_t t,uint64_t rx,uint64_t error){return (lab_real_meta){.context=s->context,.provider=provider,.stream_id=stream,.stream_session=s->adapter.session,.sequence=sequence,.flags=LAB_REAL_TIME_VALID|LAB_REAL_TIME_ESTIMATED,.time_us=t,.received_us=rx,.time_uncertainty_us=error,.parameter_version=s->version};}
static void enqueue(lab_real_mpu *s,lab_live_sample *e){
 if(e->meta.received_us<s->delivery_us)e->meta.received_us=s->delivery_us;s->delivery_us=e->meta.received_us;
 if(s->count==8){s->head=(s->head+1)%8;--s->count;++s->stats.queue_drops;}s->queue[(s->head+s->count)%8]=*e;++s->count;
}
lab_status lab_real_mpu_feed(lab_real_mpu *s,const uint8_t *wire,size_t bytes,uint64_t rx){
 if(!s||!wire)return LAB_ERR_ARGUMENT;if(s->stats.clock_fault)return LAB_ERR_CONTEXT;
 struct i9_packet p;if(bytes!=100||wire[4]!=2||i9_decode(&p,wire,bytes)){++s->stats.crc_errors;return LAB_ERR_FORMAT;}
 uint64_t t,error;lab_status result=clock_map(s,&p,rx,&t,&error);if(result!=LAB_OK)return result;
 uint32_t old=s->adapter.odom.state.accepted;if(mpu_adapter_push(&s->adapter,wire,bytes,rx/1000)<0){++s->stats.crc_errors;return LAB_ERR_FORMAT;}++s->stats.packets;
 s->stats.mpu_id=p.whoami;s->stats.imu_flags=p.flags;s->stats.i2c_error=p.i2c_error;s->stats.roll_rad=s->adapter.fusion.roll;s->stats.pitch_rad=s->adapter.fusion.pitch;memcpy(s->stats.rpm,s->adapter.fusion.rpm,sizeof(s->stats.rpm));memcpy(s->stats.counts,p.counts,sizeof(p.counts));
 s->stats.bias_ready=s->adapter.fusion.bias_ready;s->stats.counts_mask=p.initialized;s->stats.uncertainty_us=error;
 if(s->adapter.odom.state.accepted==old)return LAB_OK;
 const struct odom_state *o=&s->adapter.odom.state;lab_live_sample *e=s->scratch;memset(e,0,sizeof(*e));
 uint32_t age=p.snapshot_us-p.ag_us;uint64_t pose_error=error;
 if(error!=UINT64_MAX)pose_error+=s->config.imu_error_us+(age<=50000?age:50000);
 e->type=LAB_EVENT_POSE;e->meta=metadata(s,LAB_REAL_ODOMETRY,102,p.sequence,t,rx,pose_error);
 uint32_t quality=LAB_POSE_UNCALIBRATED;if(o->quality==ODOM_UNCALIBRATED||o->quality==ODOM_FUSED||o->quality==ODOM_WHEELS)quality|=LAB_POSE_VALID;
 if(o->quality==ODOM_GAP)quality|=LAB_POSE_GAP;if(o->quality==ODOM_SLIP||o->slip)quality|=LAB_POSE_SLIP;if(o->imu_used)quality|=LAB_POSE_IMU_USED;
 if(!s->stats.bias_ready||p.initialized!=15)quality=(quality&~LAB_POSE_VALID)|LAB_POSE_STALE;
 e->data.odometry=(lab_pose){o->x,o->y,o->heading,o->total_distance,quality,0,o->uncertainty_m};
 s->stats.pose_valid=(quality&LAB_POSE_VALID)&&!(quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP));if(s->stats.pose_valid)e->meta.flags|=LAB_REAL_DATA_VALID;enqueue(s,e);s->stats.pose_time_us=t;
 s->stats.x_m=o->x;s->stats.y_m=o->y;s->stats.yaw_rad=o->heading;s->stats.distance_m=o->total_distance;
 memset(e,0,sizeof(*e));e->type=LAB_EVENT_ENCODER;e->meta=metadata(s,LAB_REAL_ODOMETRY,103,p.sequence,t,rx,error);memcpy(e->data.encoder.counts,p.counts,sizeof(p.counts));e->data.encoder.valid_mask=p.initialized;e->data.encoder.counts_per_revolution=1320;if(p.initialized==15)e->meta.flags|=LAB_REAL_DATA_VALID;enqueue(s,e);
 memset(e,0,sizeof(*e));e->type=LAB_EVENT_IMU;uint64_t imu_t=t>=age?t-age:0;e->meta=metadata(s,LAB_REAL_ODOMETRY,104,p.sequence,imu_t,rx,error==UINT64_MAX?error:error+s->config.imu_error_us);
 e->data.imu.yaw_rad=s->adapter.fusion.yaw;e->data.imu.yaw_rate_rad_s=s->adapter.fusion.gyro[2]-s->adapter.fusion.bias[2];
 if(s->stats.bias_ready&&age<=50000&&(p.flags&(I9_AG_VALID|I9_MPU_READY))==(I9_AG_VALID|I9_MPU_READY)&&p.whoami==0x71){e->data.imu.valid_flags=LAB_IMU_YAW_VALID|LAB_IMU_RATE_VALID;e->meta.flags|=LAB_REAL_DATA_VALID;}
 if(e->data.imu.valid_flags&&(!s->have_ag_sequence||p.ag_sequence!=s->last_ag_sequence)){enqueue(s,e);s->stats.imu_time_us=imu_t;s->last_ag_sequence=p.ag_sequence;s->have_ag_sequence=1;}return LAB_OK;
}
lab_status lab_real_mpu_scan(lab_real_mpu *s,const uint8_t *p,size_t bytes,uint64_t rx){
 if(!s||!p)return LAB_ERR_ARGUMENT;if(s->stats.clock_fault)return LAB_ERR_CONTEXT;
 if(bytes<64+SCAN_HEADER||bytes>64+SCAN_PACKET_MAX||memcmp(p,"LT01",4)||p[4]!=1||p[5]!=3||p[6]||p[7]!=64||be32(p+40)!=bytes-64){++s->stats.timing_rejected;return LAB_ERR_FORMAT;}
 for(unsigned i=36;i<40;++i)if(p[i])return LAB_ERR_FORMAT;for(unsigned i=44;i<60;++i)if(p[i])return LAB_ERR_FORMAT;
 uint32_t crc=lab_crc32_extend(lab_crc32(p,60),p+64,bytes-64);if(crc!=be32(p+60)){++s->stats.timing_rejected;return LAB_ERR_CRC;}
 uint32_t session=be32(p+8),sequence=be32(p+12),span=be32(p+32);uint64_t first=be64(p+16),returned=be64(p+24);
 if(!session||!span||span>250000||first>returned||returned>rx||span>returned-first||memcmp(p+64,"LS01",4)||be32(p+72)!=session||be32(p+76)!=sequence||be64(p+80)!=returned){++s->stats.timing_rejected;return LAB_ERR_TIME;}
 if(s->have_scan&&session!=s->scan_session){s->stats.clock_fault=1;why(s,"LIDAR_RECONNECTED_NEW_CONTEXT_REQUIRED");return LAB_ERR_CONTEXT;}
 if(s->have_timing&&session==s->timing_session&&sequence==s->timing_sequence&&(first!=s->first||returned!=s->returned||span!=s->span))return LAB_ERR_FORMAT;
 s->have_timing=1;s->timing_session=session;s->timing_sequence=sequence;s->first=first;s->returned=returned;s->span=span;
 int r=scan_receive(&s->rx,p+64,bytes-64,rx/1000);if(r<0)return LAB_ERR_FORMAT;if(!r)return LAB_AGAIN;
 if(s->have_scan&&s->rx.latest.sequence==s->scan_sequence)return LAB_AGAIN;
 /* 整帧扫描以采集时间中点对齐，单点时间差最多为半圈扫描周期；此处不做逐点去畸变。 */
 lab_live_sample *e=s->scratch;memset(e,0,sizeof(*e));e->type=LAB_EVENT_SCAN;e->data.scan=s->rx.latest;
 e->meta=metadata(s,LAB_REAL_USB,101,sequence,first+span/2,rx,s->config.experimental?s->config.lidar_error_us+(span+1)/2:UINT64_MAX);e->meta.stream_session=session;
 for(unsigned i=0;i<e->data.scan.count;++i)if(e->data.scan.points[i].quality&&e->data.scan.points[i].distance_q2){e->meta.flags|=LAB_REAL_DATA_VALID;break;}
 if(!lab_live_sample_valid(e))return LAB_ERR_FORMAT;s->stats.scan_valid=(e->meta.flags&LAB_REAL_DATA_VALID)!=0;enqueue(s,e);s->have_scan=1;s->scan_session=session;s->scan_sequence=sequence;++s->stats.scan_frames;s->stats.scan_time_us=first+span/2;return LAB_OK;
}
lab_status lab_real_mpu_take(lab_real_mpu *s,uint64_t now,lab_live_sample *out){if(!s||!out)return LAB_ERR_ARGUMENT;if(s->stats.clock_fault)return LAB_ERR_CONTEXT;if(!s->count||s->queue[s->head].meta.received_us>now)return LAB_AGAIN;*out=s->queue[s->head];s->head=(s->head+1)%8;--s->count;return LAB_OK;}
void lab_real_mpu_get_stats(const lab_real_mpu *s,lab_real_mpu_stats *out){if(s&&out)*out=s->stats;}
void lab_real_mpu_core_close(lab_real_mpu *s){if(s){free(s->queue);free(s->scratch);free(s);}}
