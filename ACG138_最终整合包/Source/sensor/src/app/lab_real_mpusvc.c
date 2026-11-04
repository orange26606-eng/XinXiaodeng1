#include "lab_real_mpusvc.h"
#include "lab_codec.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
_Static_assert(sizeof(double)==8 && sizeof(struct odom_shared)==4224,"supported float/native shared ABI");
typedef struct mpusvc_pending_raw {uint8_t bytes[100];uint64_t received_us;} mpusvc_pending_raw;
struct mpusvc_core {struct mpu_adapter adapter;mpusvc_view view;int have_rx;uint64_t last_rx;
 f6_selector attitude;mpusvc_pending_raw pending[16];unsigned pending_head,pending_count;uint64_t pending_overflow;
};
static void u32(uint8_t *p,uint32_t n){for(int i=3;i>=0;--i){p[i]=(uint8_t)n;n>>=8;}}
static uint32_t r32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static void u64(uint8_t *p,uint64_t n){u32(p,(uint32_t)(n>>32));u32(p+4,(uint32_t)n);}
static uint64_t r64(const uint8_t *p){return (uint64_t)r32(p)<<32|r32(p+4);}
static void f64(uint8_t *p,double v){uint64_t n;memcpy(&n,&v,8);u64(p,n);}
static double rf64(const uint8_t *p){uint64_t n=r64(p);double v;memcpy(&v,&n,8);return v;}
static uint32_t config_crc(const struct odom_config *o,const struct i9_config *i,uint64_t version){
 uint8_t b[144]={0};for(unsigned k=0;k<4;++k){f64(b+8*k,o->mm_per_count[k]);u32(b+64+4*k,(uint32_t)o->signs[k]);}
 f64(b+32,o->track_mm);f64(b+40,o->yaw_sign);f64(b+48,o->yaw_offset);f64(b+56,o->correction_tau);u32(b+80,(uint32_t)o->left_mask);u32(b+84,(uint32_t)o->calibrated);
 for(unsigned k=0;k<3;++k)u32(b+88+4*k,(uint32_t)i->axes[k]);f64(b+104,i->cal_seconds);f64(b+112,i->counts_per_rev);u64(b+120,version);return lab_crc32(b,sizeof(b));
}
lab_status mpusvc_open(mpusvc_core **out,uint32_t source,uint64_t instance,uint64_t version,const struct odom_config *o,const struct i9_config *imu){
 if(!out)return LAB_ERR_ARGUMENT;*out=NULL;if(!o||!imu||!instance||!version||source<1||source>3||!i9_config_valid(imu)||imu->counts_per_rev!=trunc(imu->counts_per_rev))return LAB_ERR_ARGUMENT;
 /* 逐字段核对复用的四轮里程计配置。 */
 if(o->left_mask<1||o->left_mask>14||o->calibrated<0||o->calibrated>1||!isfinite(o->track_mm)||o->track_mm<50||o->track_mm>1000||!isfinite(o->yaw_sign)||fabs(o->yaw_sign)!=1||!isfinite(o->yaw_offset)||!isfinite(o->correction_tau)||o->correction_tau<=0)return LAB_ERR_ARGUMENT;
 for(unsigned k=0;k<4;++k)if(!isfinite(o->mm_per_count[k])||o->mm_per_count[k]<=0||o->mm_per_count[k]>100||(o->signs[k]!=1&&o->signs[k]!=-1))return LAB_ERR_ARGUMENT;
 unsigned left=0;for(unsigned k=0;k<4;++k)left+=(o->left_mask>>k)&1;if(left!=2)return LAB_ERR_ARGUMENT;
 mpusvc_core *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;if(mpu_adapter_init(&s->adapter,o,imu)){free(s);return LAB_ERR_ARGUMENT;}
 f6_selector_init(&s->attitude,F6_MODE_RK,imu->axes);s->view.source=source;s->view.instance=instance;s->view.parameter_version=version;s->view.config_crc=config_crc(o,imu,version);s->view.time_uncertainty_us=UINT64_MAX;s->view.odom_config=*o;s->view.odometry=s->adapter.odom.state;s->view.cal_seconds=imu->cal_seconds;s->view.counts_per_rev=(uint32_t)imu->counts_per_rev;for(unsigned k=0;k<3;++k)s->view.axes[k]=imu->axes[k];s->view.q[0]=1;s->view.flags=MPUSVC_RUNNING;*out=s;return LAB_OK;
}
lab_status mpusvc_push(mpusvc_core *s,const uint8_t *p,size_t bytes,uint64_t rx){
 if(!s||!p)return LAB_ERR_ARGUMENT;if(s->have_rx&&rx<s->last_rx)return LAB_ERR_TIME;s->have_rx=1;s->last_rx=rx;
 struct i9_packet raw;if(bytes!=100||p[4]!=2||i9_decode(&raw,p,bytes)){++s->view.rejected;return LAB_ERR_FORMAT;}
 if(s->adapter.fusion.have_packet&&(int32_t)(raw.sequence-s->adapter.fusion.previous_sequence)<=0){
  int reboot=(int32_t)(raw.sequence-s->adapter.fusion.previous_sequence)<0&&(int32_t)(raw.snapshot_us-s->adapter.fusion.previous_us)<0&&raw.snapshot_us<1000000&&rx/1000>s->adapter.received_ms+300;
  if(!reboot){++s->view.duplicates;return LAB_AGAIN;}
 }
 uint32_t old_accepted=s->adapter.odom.state.accepted,old_session=s->adapter.session;uint32_t old_seq=s->adapter.fusion.packet.ag_sequence,old_mag=s->adapter.fusion.packet.mag_sequence;int seen=s->adapter.fusion.have_packet;
 int epoch_change=s->adapter.have_epoch&&raw.reset_epoch!=s->adapter.epoch;
 int reboot=s->adapter.fusion.have_packet&&(int32_t)(raw.sequence-s->adapter.fusion.previous_sequence)<0&&(int32_t)(raw.snapshot_us-s->adapter.fusion.previous_us)<0&&raw.snapshot_us<1000000&&rx/1000>s->adapter.received_ms+300;
 if(epoch_change||reboot)f6_selector_clear(&s->attitude);
 s->attitude.raw_received_us=rx;
 struct mpu_attitude_selection selected={0};int r=mpu_adapter_push_select(&s->adapter,p,bytes,rx/1000,f6_selector_hook,&s->attitude,&selected);if(r<0){++s->view.rejected;return LAB_ERR_FORMAT;}
 mpusvc_view *v=&s->view;struct i9_state *f=&s->adapter.fusion;if(!v->accepted)v->started_us=rx;
 v->flags|=MPUSVC_SEEN;v->flags&=~MPUSVC_STOPPED;v->flags|=MPUSVC_RUNNING;++v->accepted;memcpy(v->raw,p,100);v->raw_received_us=rx;v->raw_fpga_us=f->time_us;v->epoch=raw.reset_epoch;v->session=s->adapter.session;v->missing=f->missing;v->restarts=s->adapter.restarts;v->counts_mask=raw.initialized;v->bias_ready=selected.bias_ready;v->cal_samples=selected.cal_samples;v->fusion_quality=selected.source==F6_SOURCE_FPGA?(raw.initialized==15?I9_FUSED:I9_WAIT_ENCODER):(uint32_t)f->quality;
 if(old_session!=v->session){v->have_pose=0;v->pose_sequence=0;v->pose_received_us=v->pose_fpga_us=v->ag_received_us=v->ag_fpga_us=v->mag_received_us=v->mag_fpga_us=0;}
 uint32_t ag_age=raw.snapshot_us-raw.ag_us,mag_age=raw.snapshot_us-raw.mag_us;
 if((!seen||old_session!=v->session||raw.ag_sequence!=old_seq)&&(raw.flags&I9_AG_VALID)&&ag_age<=50000&&ag_age<=f->time_us){v->ag_fpga_us=f->time_us-ag_age;v->ag_received_us=rx;v->ag_sequence=raw.ag_sequence;}
 if((!seen||old_session!=v->session||raw.mag_sequence!=old_mag)&&(raw.flags&I9_MAG_VALID)&&mag_age<=1000000&&mag_age<=f->time_us){v->mag_fpga_us=f->time_us-mag_age;v->mag_received_us=rx;v->mag_sequence=raw.mag_sequence;}
 if(old_session!=v->session||s->adapter.odom.state.accepted!=old_accepted){v->odometry=s->adapter.odom.state;v->have_pose=s->adapter.odom.have_packet;v->pose_sequence=raw.sequence;v->pose_fpga_us=f->time_us;v->pose_received_us=rx;v->pose_imu_age_known=(raw.flags&I9_AG_VALID)&&ag_age<=50000;v->pose_imu_age_us=v->pose_imu_age_known?ag_age:0;}
 memcpy(v->q,selected.q,sizeof(v->q));memcpy(v->rpy,selected.rpy,sizeof(v->rpy));memcpy(v->accel,f->accel,sizeof(v->accel));memcpy(v->gyro,f->gyro,sizeof(v->gyro));memcpy(v->bias,selected.bias,sizeof(v->bias));memcpy(v->mag,f->mag,sizeof(v->mag));memcpy(v->rpm,f->rpm,sizeof(v->rpm));v->temperature=f->temperature_c;return LAB_OK;
}
lab_status mpusvc_attitude_mode(mpusvc_core*s,unsigned mode){if(!s||mode>F6_MODE_FPGA||s->view.accepted||s->pending_count)return LAB_ERR_ARGUMENT;f6_selector_init(&s->attitude,mode,s->view.axes);return LAB_OK;}
lab_status mpusvc_push_attitude(mpusvc_core*s,const uint8_t*p,size_t bytes,uint64_t rx){if(!s||!p)return LAB_ERR_ARGUMENT;int code=f6_selector_push(&s->attitude,p,bytes,rx);return code<0?LAB_ERR_FORMAT:code?LAB_OK:LAB_AGAIN;}
lab_status mpusvc_flush_pending(mpusvc_core*s,uint64_t now){
 if(!s)return LAB_ERR_ARGUMENT;lab_status result=LAB_AGAIN;
 while(s->pending_count){mpusvc_pending_raw*x=&s->pending[s->pending_head];if(now<x->received_us)return LAB_ERR_TIME;
  struct i9_packet p;if(i9_decode(&p,x->bytes,100))return LAB_ERR_FORMAT;
  if(s->attitude.mode!=F6_MODE_RK&&!f6_selector_find(&s->attitude,&p,now)&&now-x->received_us<12000)break;
  s->attitude.raw_wait_us=now-x->received_us;lab_status code=mpusvc_push(s,x->bytes,100,x->received_us);
  s->pending_head=(s->pending_head+1)%16;--s->pending_count;
  if(code<0)return code;if(code==LAB_OK)result=LAB_OK;
 }return result;
}
lab_status mpusvc_submit_raw(mpusvc_core*s,const uint8_t*p,size_t bytes,uint64_t rx,uint64_t now){
 if(!s||!p||bytes!=100||rx>now)return LAB_ERR_ARGUMENT;struct i9_packet decoded;if(i9_decode(&decoded,p,bytes)){++s->view.rejected;return LAB_ERR_FORMAT;}
 if(s->pending_count){const mpusvc_pending_raw*last=&s->pending[(s->pending_head+s->pending_count-1)%16];if(rx<last->received_us)return LAB_ERR_TIME;}
 if(s->attitude.mode==F6_MODE_RK&&!s->pending_count){s->attitude.raw_wait_us=0;return mpusvc_push(s,p,bytes,rx);}
 if(s->pending_count==16){mpusvc_pending_raw*x=&s->pending[s->pending_head];++s->pending_overflow;s->attitude.raw_wait_us=now-x->received_us;lab_status code=mpusvc_push(s,x->bytes,100,x->received_us);s->pending_head=(s->pending_head+1)%16;--s->pending_count;if(code<0)return code;}
 mpusvc_pending_raw*x=&s->pending[(s->pending_head+s->pending_count)%16];memcpy(x->bytes,p,100);x->received_us=rx;++s->pending_count;return mpusvc_flush_pending(s,now);
}
void mpusvc_snapshot(const mpusvc_core *s,uint64_t now,mpusvc_view *out){
 if(!s||!out)return;*out=s->view;out->published_us=now;const f6_selector*f=&s->attitude;
 out->attitude_mode=f->mode;out->attitude_source=f->source;snprintf(out->attitude_reason,sizeof out->attitude_reason,"%s",f->reason);
 out->attitude_matches=f->matches;out->attitude_misses=f->misses;out->attitude_received=f->accepted;out->attitude_rejected=f->rejected;out->attitude_crc_errors=f->crc_errors;out->attitude_duplicates=f->duplicates;out->attitude_reorders=f->reorders;out->attitude_switches=f->source_switches;out->raw_wait_us=f->raw_wait_us;out->raw_pending_overflow=s->pending_overflow;
 out->attitude_epoch=f->last.epoch;out->attitude_ag_sequence=f->last.sequence;out->attitude_source_us=f->last.source_us;out->attitude_status=f->last.status;out->attitude_age_us=f->last.age_us;out->attitude_flags=f->last.flags;out->attitude_received_us=f->last_received_us;out->attitude_yaw_offset=f->yaw_offset;out->attitude_paired_received_us=f->paired_received_us;out->attitude_paired_epoch=f->paired.epoch;out->attitude_paired_ag_sequence=f->paired.sequence;out->attitude_paired_source_us=f->paired.source_us;
}
void mpusvc_close(mpusvc_core *s){free(s);}
static int valid(const mpusvc_view *v){
 if(v&&(v->pose_imu_age_known>1||(v->pose_imu_age_known&&v->pose_imu_age_us>50000)))return 0;
 if(!v||v->source<1||v->source>3||!v->instance||!v->parameter_version||!v->counts_per_rev||v->counts_per_rev>1000000||v->flags&~15u||v->bias_ready>1||v->counts_mask>15||v->have_pose>1||v->odometry.quality>ODOM_SLIP||v->odometry.points>ODOM_POINTS||v->odometry.calibrated>1||v->published_us<v->raw_received_us||v->raw_received_us<v->pose_received_us||v->raw_received_us<v->ag_received_us||v->raw_received_us<v->mag_received_us)return 0;
 if((v->flags&(MPUSVC_RUNNING|MPUSVC_STOPPED))!=MPUSVC_RUNNING&&(v->flags&(MPUSVC_RUNNING|MPUSVC_STOPPED))!=MPUSVC_STOPPED)return 0;
 if(v->time_uncertainty_us!=UINT64_MAX)return 0;/* 本阶段不隐式进行时钟标定。 */
 for(unsigned i=0;i<4;++i)if(!isfinite(v->q[i])||!isfinite(v->rpm[i])||!isfinite(v->odom_config.mm_per_count[i]))return 0;
 for(unsigned i=0;i<3;++i)if(!isfinite(v->rpy[i])||!isfinite(v->accel[i])||!isfinite(v->gyro[i])||!isfinite(v->bias[i])||!isfinite(v->mag[i])||v->axes[i]<-3||v->axes[i]>3||v->axes[i]==0)return 0;
 const struct odom_state *o=&v->odometry;const double d[]={v->temperature,o->x,o->y,o->heading,o->total_distance,o->uncertainty_m,v->odom_config.track_mm,v->odom_config.yaw_sign,v->odom_config.yaw_offset,v->odom_config.correction_tau,v->cal_seconds};for(unsigned i=0;i<sizeof(d)/sizeof(d[0]);++i)if(!isfinite(d[i]))return 0;
 if(o->total_distance<0||o->uncertainty_m<0)return 0;
 if(v->flags&MPUSVC_SEEN){struct i9_packet p;if(i9_decode(&p,v->raw,100)||v->raw[4]!=2||p.initialized!=v->counts_mask||p.reset_epoch!=v->epoch)return 0;}
 return 1;
}
lab_status mpusvc_encode(uint8_t b[768],const mpusvc_view *v){
 if(!b||!valid(v))return LAB_ERR_ARGUMENT;memset(b,0,768);memcpy(b,"MPUSVC1",7);u32(b+8,1);u32(b+12,768);u32(b+16,v->source);u32(b+20,v->flags);
 const uint64_t a[]={v->instance,v->export_sequence,v->published_us,v->raw_received_us,v->raw_fpga_us,v->time_uncertainty_us,v->parameter_version};for(unsigned i=0;i<7;++i)u64(b+24+i*8,a[i]);
 u32(b+80,v->config_crc);u32(b+84,v->session);u32(b+88,v->pose_sequence);u32(b+92,v->have_pose);
 const uint64_t t[]={v->pose_fpga_us,v->pose_received_us,v->ag_fpga_us,v->ag_received_us,v->mag_fpga_us,v->mag_received_us};for(unsigned i=0;i<6;++i)u64(b+96+i*8,t[i]);u32(b+144,v->ag_sequence);u32(b+148,v->mag_sequence);memcpy(b+152,v->raw,100);
 const uint64_t c[]={v->accepted,v->rejected,v->duplicates,v->missing,v->restarts,v->latest_skipped,v->record_drops};for(unsigned i=0;i<7;++i)u64(b+256+i*8,c[i]);
 u32(b+312,v->epoch);u32(b+316,v->counts_mask);u32(b+320,v->bias_ready);u32(b+324,v->cal_samples);u32(b+328,v->fusion_quality);u32(b+332,v->odometry.quality);u32(b+336,v->odometry.calibrated);u32(b+340,v->odometry.imu_used);
 for(unsigned i=0;i<4;++i){f64(b+344+8*i,v->q[i]);f64(b+496+8*i,v->rpm[i]);u32(b+576+4*i,(uint32_t)v->odometry.raw_counts[i]);f64(b+608+8*i,v->odom_config.mm_per_count[i]);u32(b+652+4*i,(uint32_t)v->odom_config.signs[i]);}
 for(unsigned i=0;i<3;++i){f64(b+376+8*i,v->rpy[i]);f64(b+400+8*i,v->accel[i]);f64(b+424+8*i,v->gyro[i]);f64(b+448+8*i,v->bias[i]);f64(b+472+8*i,v->mag[i]);u32(b+668+4*i,(uint32_t)v->axes[i]);}
 f64(b+528,v->temperature);f64(b+536,v->odometry.x);f64(b+544,v->odometry.y);f64(b+552,v->odometry.heading);f64(b+560,v->odometry.total_distance);f64(b+568,v->odometry.uncertainty_m);u32(b+592,v->odometry.accepted);u32(b+596,v->odometry.gaps);u32(b+600,v->odometry.slip);u32(b+604,v->counts_per_rev);f64(b+640,v->odom_config.track_mm);u32(b+648,(uint32_t)v->odom_config.left_mask);f64(b+680,v->odom_config.yaw_sign);f64(b+688,v->odom_config.yaw_offset);f64(b+696,v->odom_config.correction_tau);f64(b+704,v->cal_seconds);u64(b+712,v->started_us);u64(b+732,v->record_count);u32(b+764,lab_crc32(b,764));return LAB_OK;
}
lab_status mpusvc_encode_timed(uint8_t b[768],const mpusvc_view*v){
 lab_status r=mpusvc_encode(b,v);if(r!=LAB_OK)return r;u32(b+8,2);memcpy(b+740,"PT02",4);u64(b+744,v->pose_imu_age_us);u32(b+752,v->pose_imu_age_known);u32(b+764,lab_crc32(b,764));return LAB_OK;
}
lab_status mpusvc_decode(mpusvc_view *v,const uint8_t *b,size_t n){
 if(!v||!b)return LAB_ERR_ARGUMENT;memset(v,0,sizeof(*v));if(n!=768)return LAB_ERR_TRUNCATED;uint32_t version=r32(b+8);if(memcmp(b,"MPUSVC1\0",8)||(version!=1&&version!=2)||r32(b+12)!=768)return LAB_ERR_VERSION;if(lab_crc32(b,764)!=r32(b+764))return LAB_ERR_CRC;
 for(unsigned i=252;i<256;++i)if(b[i])return LAB_ERR_FORMAT;for(unsigned i=720;i<732;++i)if(b[i])return LAB_ERR_FORMAT;
 if(version==2){if(memcmp(b+740,"PT02",4))return LAB_ERR_FORMAT;v->pose_imu_age_us=r64(b+744);v->pose_imu_age_known=r32(b+752);for(unsigned i=756;i<764;++i)if(b[i])return LAB_ERR_FORMAT;}
 else for(unsigned i=740;i<764;++i)if(b[i])return LAB_ERR_FORMAT;
 v->source=r32(b+16);v->flags=r32(b+20);v->instance=r64(b+24);v->export_sequence=r64(b+32);v->published_us=r64(b+40);v->raw_received_us=r64(b+48);v->raw_fpga_us=r64(b+56);v->time_uncertainty_us=r64(b+64);v->parameter_version=r64(b+72);v->config_crc=r32(b+80);v->session=r32(b+84);v->pose_sequence=r32(b+88);v->have_pose=r32(b+92);v->pose_fpga_us=r64(b+96);v->pose_received_us=r64(b+104);v->ag_fpga_us=r64(b+112);v->ag_received_us=r64(b+120);v->mag_fpga_us=r64(b+128);v->mag_received_us=r64(b+136);v->ag_sequence=r32(b+144);v->mag_sequence=r32(b+148);memcpy(v->raw,b+152,100);
 v->accepted=r64(b+256);v->rejected=r64(b+264);v->duplicates=r64(b+272);v->missing=r64(b+280);v->restarts=r64(b+288);v->latest_skipped=r64(b+296);v->record_drops=r64(b+304);v->epoch=r32(b+312);v->counts_mask=r32(b+316);v->bias_ready=r32(b+320);v->cal_samples=r32(b+324);v->fusion_quality=r32(b+328);v->odometry.quality=r32(b+332);v->odometry.calibrated=r32(b+336);v->odometry.imu_used=r32(b+340);
 for(unsigned i=0;i<4;++i){v->q[i]=rf64(b+344+8*i);v->rpm[i]=rf64(b+496+8*i);v->odometry.raw_counts[i]=(int32_t)r32(b+576+4*i);v->odom_config.mm_per_count[i]=rf64(b+608+8*i);v->odom_config.signs[i]=(int32_t)r32(b+652+4*i);}
 for(unsigned i=0;i<3;++i){v->rpy[i]=rf64(b+376+8*i);v->accel[i]=rf64(b+400+8*i);v->gyro[i]=rf64(b+424+8*i);v->bias[i]=rf64(b+448+8*i);v->mag[i]=rf64(b+472+8*i);v->axes[i]=(int32_t)r32(b+668+4*i);}
 v->temperature=rf64(b+528);v->odometry.x=rf64(b+536);v->odometry.y=rf64(b+544);v->odometry.heading=rf64(b+552);v->odometry.total_distance=rf64(b+560);v->odometry.uncertainty_m=rf64(b+568);v->odometry.accepted=r32(b+592);v->odometry.gaps=r32(b+596);v->odometry.slip=r32(b+600);v->counts_per_rev=r32(b+604);v->odom_config.track_mm=rf64(b+640);v->odom_config.left_mask=(int)r32(b+648);v->odom_config.yaw_sign=rf64(b+680);v->odom_config.yaw_offset=rf64(b+688);v->odom_config.correction_tau=rf64(b+696);v->cal_seconds=rf64(b+704);v->started_us=r64(b+712);v->record_count=r64(b+732);v->odometry.session=v->session;v->odometry.stamp_ms=v->pose_received_us/1000;v->odom_config.calibrated=(int)v->odometry.calibrated;
 if(!valid(v)){memset(v,0,sizeof(*v));return LAB_ERR_FORMAT;}return LAB_OK;
}
void mpusvc_imu_model(const mpusvc_view *v,uint64_t now,struct imu_model *m){
 memset(m,0,sizeof(*m));m->state=IMU_WAITING;if(!v)return;m->seen_packet=(v->flags&MPUSVC_SEEN)!=0;m->last_packet_ms=v->raw_received_us/1000;m->last_good_ms=v->ag_received_us/1000;m->sequence=v->ag_sequence;m->good_sequence=v->ag_sequence;m->have_sequence=m->seen_packet;m->good_count=v->accepted;m->invalid_count=v->rejected;m->missing_count=v->missing;
 if(now<v->raw_received_us||v->flags&MPUSVC_STOPPED||now-v->raw_received_us>250000||!v->ag_received_us||now<v->ag_received_us||now-v->ag_received_us>150000){m->state=IMU_STALE;return;}
 if(v->bias_ready&&(v->raw[5]&(I9_AG_VALID|I9_MPU_READY))==(I9_AG_VALID|I9_MPU_READY)&&v->raw[6]==0x71){m->have_angles=1;m->state=IMU_LIVE;for(unsigned i=0;i<3;++i)m->degrees[i]=v->rpy[i]*57.29577951308232;}
 else if(v->bias_ready){m->state=IMU_DEVICE_ERROR;m->status=5;}
}
void mpusvc_print(const mpusvc_view *v,uint64_t now){
 const char *source=v->source==1?"HARDWARE":v->source==2?"RECORDING":"TEST_FIXTURE";double age=now>=v->raw_received_us?(double)(now-v->raw_received_us)/1000:-1;
 printf("source=%s service=%llu seq=%u age_ms=%.1f state=%s bias=%u samples=%u mask=%x FPGA_us=%llu RK_rx_us=%llu time_error=UNKNOWN pose_seq=%u x=%.4f y=%.4f yaw=%.2f distance=%.4f accepted=%llu rejected=%llu duplicates=%llu record_drops=%llu motor_output=NONE attitude=%s attitude_reason=%s\n",source,(unsigned long long)v->instance,(unsigned)(v->raw[8]|(uint32_t)v->raw[9]<<8|(uint32_t)v->raw[10]<<16|(uint32_t)v->raw[11]<<24),age,v->flags&MPUSVC_STOPPED?"STOPPED":age>250?"STALE":v->bias_ready?"READY":"KEEP_STILL",v->bias_ready,v->cal_samples,v->counts_mask,(unsigned long long)v->raw_fpga_us,(unsigned long long)v->raw_received_us,v->pose_sequence,v->odometry.x,v->odometry.y,v->odometry.heading*57.29577951308232,v->odometry.total_distance,(unsigned long long)v->accepted,(unsigned long long)v->rejected,(unsigned long long)v->duplicates,(unsigned long long)v->record_drops,f6_source_name(v->attitude_source),v->attitude_reason[0]?v->attitude_reason:"SOURCE_IN_STATUS_JSON");fflush(stdout);
}
void mpusvc_capture_header(uint8_t b[64],uint32_t source,uint64_t id,uint64_t created,uint64_t version,uint32_t crc){memset(b,0,64);memcpy(b,"I9CAP1",6);u32(b+8,1);u32(b+12,64);u32(b+16,source);u32(b+20,128);u64(b+24,id);u64(b+32,created);u64(b+40,version);u32(b+48,crc);u32(b+60,lab_crc32(b,60));}
void mpusvc_capture_record(uint8_t b[128],const uint8_t raw[100],uint64_t rx,uint64_t ordinal){memset(b,0,128);memcpy(b,"I9R1",4);u32(b+4,100);u64(b+8,rx);u64(b+16,ordinal);memcpy(b+24,raw,100);u32(b+124,lab_crc32(b,124));}
void mpusvc_capture_footer(uint8_t b[40],uint64_t count,uint64_t drops,uint32_t stream,uint32_t flags){memset(b,0,40);memcpy(b,"I9END1",6);u64(b+8,count);u64(b+16,drops);u32(b+24,stream);u32(b+28,flags);u32(b+36,lab_crc32(b,36));}
lab_status mpusvc_reprocess(const char *path,const struct odom_config *o,const struct i9_config *imu,mpusvc_view *out){
 if(!path||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));FILE *f=fopen(path,"rb");if(!f)return LAB_ERR_IO;uint8_t b[128];mpusvc_core *s=NULL;lab_status r=LAB_ERR_TRUNCATED;uint64_t count=0,last=0;uint32_t stream=0;
 if(fread(b,1,64,f)!=64)goto done;r=LAB_ERR_FORMAT;if(memcmp(b,"I9CAP1\0\0",8)||r32(b+8)!=1||r32(b+12)!=64||r32(b+20)!=128||r32(b+16)<1||r32(b+16)>3)goto done;r=LAB_ERR_CRC;if(r32(b+60)!=lab_crc32(b,60))goto done;for(unsigned i=52;i<60;++i)if(b[i]){r=LAB_ERR_FORMAT;goto done;}
 uint32_t original=r32(b+16),crc=r32(b+48);r=mpusvc_open(&s,original==LAB_REAL_TEST_FIXTURE?LAB_REAL_TEST_FIXTURE:LAB_REAL_RECORDING,r64(b+24),r64(b+40),o,imu);if(r!=LAB_OK)goto done;if(s->view.config_crc!=crc){r=LAB_ERR_VERSION;goto done;}
 for(;;){r=LAB_ERR_TRUNCATED;if(fread(b,1,8,f)!=8)goto done;if(!memcmp(b,"I9END1\0\0",8)){
   if(fread(b+8,1,32,f)!=32)goto done;r=LAB_ERR_CRC;if(r32(b+36)!=lab_crc32(b,36))goto done;r=LAB_ERR_FORMAT;if(r64(b+8)!=count||r32(b+24)!=stream||r32(b+28)!=1||r32(b+32)||fgetc(f)!=EOF||ferror(f))goto done;
   mpusvc_snapshot(s,last,out);out->record_count=count;out->record_drops=r64(b+16);out->flags=(out->flags&~MPUSVC_RUNNING)|MPUSVC_STOPPED;r=out->record_drops?LAB_ERR_LIMIT:LAB_OK;goto done;
  }
  if(memcmp(b,"I9R1",4)||r32(b+4)!=100){r=LAB_ERR_FORMAT;goto done;}if(fread(b+8,1,120,f)!=120)goto done;r=LAB_ERR_CRC;if(r32(b+124)!=lab_crc32(b,124))goto done;r=LAB_ERR_FORMAT;if(r64(b+16)!=count+1)goto done;
  uint64_t rx=r64(b+8);if(count&&rx<last){r=LAB_ERR_TIME;goto done;}last=rx;lab_status a=mpusvc_push(s,b+24,100,rx);if(a==LAB_ERR_TIME){r=a;goto done;}/* 保留无效原始报文供诊断。 */
  stream=lab_crc32_extend(stream,b,128);if(++count>524288){r=LAB_ERR_LIMIT;goto done;}
 }
done:mpusvc_close(s);fclose(f);return r;
}
