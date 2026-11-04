#include "lab_real_snapshot.h"
#include "lab_real_mpusvc.h"
#include "lab_codec.h"
#include "../common/scan_motion_comp.h"
#include <math.h>
#include "continuous_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct lab_real_snapshot {
 lab_real_mpu_config config;lab_real_context context;uint64_t version;
 struct odom_config expected_odom;struct i9_config expected_imu;
 lab_real_snapshot_stats stats;lab_live_sample *queue,*scratch;unsigned head,count;
 struct scan_receiver receiver;
 uint64_t instance,service_version,export_sequence,raw_fpga,raw_rx,published,delivery;
 uint64_t pose_fpga,pose_mapped,imu_fpga,imu_mapped;int have,have_pose,have_imu;
 uint32_t session,epoch,service_crc,raw_sequence,pose_sequence,ag_sequence;int restart_pending;
 /* 当前输入只用于诊断。 */
 uint64_t diag_now,diag_instance,diag_raw_fpga,diag_raw_rx,diag_export,diag_published;
 uint32_t diag_session,diag_epoch,diag_crc,diag_ag_sequence;
 int64_t low,high;uint32_t scan_session,scan_sequence;int have_scan,have_timing;
 uint32_t timing_session,timing_sequence,span;uint64_t first,returned;
 r8_deskew_history deskew;lab_live_sample *deskew_pending;struct scan_frame *deskew_output;
 uint64_t deskew_first,deskew_started;uint32_t deskew_span;int have_deskew_pending;
 unsigned clock_conflict_streak;
};
static uint32_t be32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static uint64_t be64(const uint8_t *p){return (uint64_t)be32(p)<<32|be32(p+4);}
static void reason(lab_real_snapshot *s,const char *text){snprintf(s->stats.reason,sizeof(s->stats.reason),"%s",text);}
static lab_status fault(lab_real_snapshot *s,const char *text){fprintf(stderr,"SNAPSHOT_CONTEXT_FATAL reason=%s low=%lld high=%lld raw_fpga=%llu raw_rx=%llu exports=%llu\n",text,(long long)s->low,(long long)s->high,(unsigned long long)s->raw_fpga,(unsigned long long)s->raw_rx,(unsigned long long)s->stats.exports);fprintf(stderr,"MAP_CONTEXT_DETAIL code=%s message_zh=地图数据上下文失效需新会话 now_us=%llu old_instance=%llu new_instance=%llu old_session=%u new_session=%u old_epoch=%u new_epoch=%u old_config_crc=%u new_config_crc=%u old_raw_fpga_us=%llu new_raw_fpga_us=%llu old_raw_rx_us=%llu new_raw_rx_us=%llu new_export=%llu new_published_us=%llu new_ag_sequence=%u delay_min_us=%llu delay_max_us=%llu drift_ppm=%u pose_limit_us=%llu scan_limit_us=%llu old_lidar_session=%u old_lidar_sequence=%u clock_low=%lld clock_high=%lld\n",text,(unsigned long long)s->diag_now,(unsigned long long)s->instance,(unsigned long long)s->diag_instance,s->session,s->diag_session,s->epoch,s->diag_epoch,s->service_crc,s->diag_crc,(unsigned long long)s->raw_fpga,(unsigned long long)s->diag_raw_fpga,(unsigned long long)s->raw_rx,(unsigned long long)s->diag_raw_rx,(unsigned long long)s->diag_export,(unsigned long long)s->diag_published,s->diag_ag_sequence,(unsigned long long)s->config.delay_min_us,(unsigned long long)s->config.delay_max_us,s->config.drift_ppm,(unsigned long long)CONT_ODOM_US,(unsigned long long)CONT_SCAN_US,s->scan_session,s->scan_sequence,(long long)s->low,(long long)s->high);s->stats.failed=1;s->stats.pose_valid=s->stats.imu_valid=s->stats.scan_valid=0;s->count=0;s->have_deskew_pending=0;s->stats.deskew_pending=0;reason(s,text);return LAB_ERR_CONTEXT;}
static lab_status request_restart(lab_real_snapshot*s,const char*text);
static lab_status drop_sample(lab_real_snapshot*s,const char*text){++s->stats.rejected;reason(s,text);fprintf(stderr,"SNAPSHOT_SAMPLE_DROP reason=%s action=DROP_ONE_CONTINUE now_us=%llu raw_fpga_us=%llu raw_rx_us=%llu low=%lld high=%lld\n",text,(unsigned long long)s->diag_now,(unsigned long long)s->diag_raw_fpga,(unsigned long long)s->diag_raw_rx,(long long)s->low,(long long)s->high);return LAB_AGAIN;}
uint64_t lab_real_snapshot_identity(const lab_real_mpu_config *c){uint64_t h=lab_real_mpu_identity(c);return h?(h^UINT64_C(0x534e4150314d4944))|1u:0;}
lab_status lab_real_snapshot_open(lab_real_snapshot **out,const lab_real_mpu_config *c,const lab_real_context *ctx,uint64_t version){
 if(!out)return LAB_ERR_ARGUMENT;*out=NULL;
 if(!c||!lab_real_context_valid(ctx)||ctx->clock!=LAB_REAL_MONOTONIC||!version||c->experimental>1||c->drift_ppm>1000||c->delay_max_us<c->delay_min_us||c->delay_max_us>100000||
    (c->experimental&&(!c->lidar_error_us||c->lidar_error_us>50000||!c->imu_error_us||c->imu_error_us>50000)))return LAB_ERR_ARGUMENT;
 lab_real_snapshot *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->config=*c;s->context=*ctx;s->version=version;
 s->queue=calloc(8,sizeof(*s->queue));s->scratch=calloc(1,sizeof(*s->scratch));s->deskew_pending=calloc(1,sizeof(*s->deskew_pending));s->deskew_output=calloc(1,sizeof(*s->deskew_output));if(!s->queue||!s->scratch||!s->deskew_pending||!s->deskew_output){lab_real_snapshot_close(s);return LAB_ERR_NOMEM;}
 odom_default_config(&s->expected_odom);i9_default_config(&s->expected_imu);
 if((c->odom_config[0]&&odom_load_config(&s->expected_odom,c->odom_config))||(c->imu_config[0]&&mpu_config_load(&s->expected_imu,c->imu_config))){lab_real_snapshot_close(s);return LAB_ERR_FORMAT;}
 s->stats.deskew_storage_bytes=sizeof(*s)+8*sizeof(*s->queue)+sizeof(*s->scratch)+sizeof(*s->deskew_pending)+sizeof(*s->deskew_output);
 snprintf(s->stats.deskew_reason,sizeof(s->stats.deskew_reason),"WAIT_FIRST_TIMED_SCAN");
 s->stats.pose_error_us=UINT64_MAX;reason(s,"WAIT_SERVICE_SNAPSHOT_FILE_NO_UDP5016");*out=s;return LAB_OK;
}
static int equal(double a,double b){return fabs(a-b)<=1e-10*fmax(1,fmax(fabs(a),fabs(b)));}
static int configuration_matches(const lab_real_snapshot *s,const mpusvc_view *v){
 const struct odom_config *a=&s->expected_odom,*b=&v->odom_config;
 if(a->left_mask!=b->left_mask||a->calibrated!=b->calibrated||!equal(a->track_mm,b->track_mm)||!equal(a->yaw_sign,b->yaw_sign)||!equal(a->yaw_offset,b->yaw_offset)||!equal(a->correction_tau,b->correction_tau)||
    !equal(s->expected_imu.cal_seconds,v->cal_seconds)||!equal(s->expected_imu.counts_per_rev,v->counts_per_rev))return 0;
 for(unsigned i=0;i<4;++i)if(a->signs[i]!=b->signs[i]||!equal(a->mm_per_count[i],b->mm_per_count[i]))return 0;
 for(unsigned i=0;i<3;++i)if(s->expected_imu.axes[i]!=v->axes[i])return 0;return 1;
}
/* 时间界限取决于声明的原始快照到RK首次接收时延，文件发布及读取延迟不计入该传输时延。
 */
static lab_status clock_observe(lab_real_snapshot *s,const mpusvc_view *v){
 if(v->raw_received_us>INT64_MAX-1000000||v->raw_fpga_us>INT64_MAX-1000000)return fault(s,"CLOCK_RANGE_NEW_CONTEXT_REQUIRED");
 int64_t prior_low=s->low,prior_high=s->high;
 if(s->have){
  if(v->raw_fpga_us<=s->raw_fpga||v->raw_received_us<s->raw_rx)return request_restart(s,"CLOCK_ROLLBACK_AUTO_NEW_CONTEXT");
  uint64_t dt=v->raw_fpga_us-s->raw_fpga;if(dt>UINT64_C(10000000))return request_restart(s,"CLOCK_LONG_GAP_AUTO_NEW_CONTEXT");
  int64_t drift=(int64_t)((dt*s->config.drift_ppm+999999)/1000000)+1;prior_low-=drift;prior_high+=drift;
 }
 if(!s->config.experimental)return LAB_OK;
 if(v->raw_received_us<s->config.delay_max_us)return drop_sample(s,"CLOCK_DELAY_BEFORE_ORIGIN_DROP");
 int64_t low=(int64_t)(v->raw_received_us-s->config.delay_max_us)-(int64_t)v->raw_fpga_us;
 int64_t high=(int64_t)(v->raw_received_us-s->config.delay_min_us)-(int64_t)v->raw_fpga_us;
 if(s->have){low=prior_low>low?prior_low:low;high=prior_high<high?prior_high:high;}
 if(low>high){
  if(++s->clock_conflict_streak>=5)return request_restart(s,"LATENCY_INTERVAL_PERSISTENT_AUTO_RESYNC");
  return drop_sample(s,"LATENCY_INTERVAL_CONFLICT_DROP_ONE");
 }
 s->clock_conflict_streak=0;s->low=low;s->high=high;return LAB_OK;
}
static lab_status map_stamp(lab_real_snapshot *s,uint64_t fpga,uint64_t rx,uint64_t *stamp,uint64_t *error){
 if(!s->config.experimental){*stamp=rx;*error=UINT64_MAX;return LAB_OK;}
 if(fpga>s->raw_fpga||fpga>INT64_MAX-1000000)return drop_sample(s,"SAMPLE_TIME_AHEAD_OF_RAW_DROP");
 uint64_t age=s->raw_fpga-fpga;if(age>1000000)return LAB_AGAIN;
 int64_t drift=(int64_t)((age*s->config.drift_ppm+999999)/1000000)+1;
 int64_t lo=(int64_t)fpga+s->low-drift,hi=(int64_t)fpga+s->high+drift;
 if(hi>(int64_t)rx)hi=(int64_t)rx;
 if(lo<0||lo>hi)return drop_sample(s,"SAMPLE_TIME_INTERVAL_EMPTY_DROP");
 /* 消费者将误差视为time_us周围对称半径。使用区间左端加整宽会在年龄与误差中重复计入区间；中心加半宽覆盖同一时钟区间，无需降低新鲜度门限。


 */
 uint64_t width=(uint64_t)(hi-lo);*stamp=(uint64_t)lo+width/2;
 *error=(width+1)/2+1;return LAB_OK;
}
static lab_real_meta meta(lab_real_snapshot *s,uint32_t provider,uint32_t id,uint32_t seq,uint64_t t,uint64_t rx,uint64_t error){
 return (lab_real_meta){.context=s->context,.provider=provider,.stream_id=id,.stream_session=s->session,.sequence=seq,.flags=LAB_REAL_TIME_VALID|LAB_REAL_TIME_ESTIMATED,.time_us=t,.received_us=rx,.time_uncertainty_us=error,.parameter_version=s->version};
}
static void enqueue(lab_real_snapshot *s,lab_live_sample *e){if(e->meta.received_us<s->delivery)e->meta.received_us=s->delivery;s->delivery=e->meta.received_us;
 if(s->count==8){s->head=(s->head+1)%8;--s->count;++s->stats.queue_drops;}s->queue[(s->head+s->count)%8]=*e;++s->count;}
/* 复位事件在旧上下文发出，运行对象接受新epoch，并在任何新测量存在前调用source.reset。
 */
static lab_status request_restart(lab_real_snapshot*s,const char*text){
 if(s->restart_pending)return LAB_AGAIN;
 if(s->context.origin_epoch==UINT32_MAX)return fault(s,"CONTEXT_EPOCH_EXHAUSTED");
 s->count=s->head=0;s->have_deskew_pending=0;s->stats.deskew_pending=0;
 memset(&s->deskew,0,sizeof(s->deskew));s->stats.pose_valid=s->stats.imu_valid=s->stats.scan_valid=0;
 s->stats.pose_time_us=s->stats.imu_time_us=s->stats.scan_time_us=0;
 lab_live_sample*e=s->scratch;memset(e,0,sizeof(*e));e->type=LAB_EVENT_RESET;
 e->meta=meta(s,LAB_REAL_ODOMETRY,205,(uint32_t)s->diag_export,s->diag_now,s->diag_now,0);
 e->meta.flags=LAB_REAL_TIME_VALID|LAB_REAL_DATA_VALID;e->data.reset.reason=LAB_RESET_SOURCE_CHANGE;e->data.reset.next_origin_epoch=s->context.origin_epoch+1;
 enqueue(s,e);s->restart_pending=1;reason(s,text);
 fprintf(stderr,"SNAPSHOT_CONTEXT_RECOVER reason=%s old_epoch=%u next_epoch=%u old_instance=%llu new_instance=%llu old_sensor_epoch=%u new_sensor_epoch=%u action=DROP_OLD_CACHE_REQUEST_NEW_CONTEXT old_pose_reused=0\n",text,s->context.origin_epoch,e->data.reset.next_origin_epoch,(unsigned long long)s->instance,(unsigned long long)s->diag_instance,s->epoch,s->diag_epoch);return LAB_AGAIN;
}
/* 仅保留一笔最新待处理扫描，最多等待80ms补齐采集时间位姿；位姿接收独立进行。 */

static void deskew_release(lab_real_snapshot*s,uint64_t now){
 if(!s->have_deskew_pending||s->stats.failed)return;
 r8_deskew_result d;int code=r8_deskew_apply(&s->deskew,&s->deskew_pending->data.scan,s->deskew_output,s->deskew_first,s->deskew_span,s->config.mount,s->config.experimental,&d);
 if(code==R8_DESKEW_WAIT&&now>=s->deskew_started&&now-s->deskew_started<R8_DESKEW_WAIT_US){
  snprintf(s->stats.deskew_reason,sizeof(s->stats.deskew_reason),"%s",d.reason);return;
 }
 if(code==R8_DESKEW_APPLIED){s->deskew_pending->data.scan=*s->deskew_output;++s->stats.deskew_applied;}
 else ++s->stats.deskew_skipped;
 snprintf(s->stats.deskew_reason,sizeof(s->stats.deskew_reason),"%s",code==R8_DESKEW_WAIT?"SKIP_FUTURE_RAW_POSE_WAIT_80MS_EXPIRED":d.reason);
 s->stats.deskew_first_us=d.first_us;s->stats.deskew_reference_us=d.reference_us;s->stats.deskew_last_us=d.last_us;
 s->stats.deskew_span_us=d.span_us;s->stats.deskew_points=code==R8_DESKEW_APPLIED?d.corrected_points:0;
 s->stats.deskew_invalid_points=d.invalid_points;s->stats.deskew_translation_m=d.translation_m;s->stats.deskew_yaw_rad=d.yaw_rad;
 s->stats.deskew_max_origin_m=d.max_origin_m;s->stats.deskew_pose_error_us=d.pose_error_us;
 s->deskew_pending->meta.received_us=now;enqueue(s,s->deskew_pending);
 s->stats.scan_valid=(s->deskew_pending->meta.flags&LAB_REAL_DATA_VALID)!=0;
 s->stats.scan_time_us=s->deskew_pending->meta.time_us;++s->stats.scans;
 s->have_deskew_pending=0;s->stats.deskew_pending=0;
}
lab_status lab_real_snapshot_feed(lab_real_snapshot *s,const uint8_t *bytes,size_t size,uint64_t now){
 if(!s||!bytes)return LAB_ERR_ARGUMENT;if(s->stats.failed)return LAB_ERR_CONTEXT;if(s->restart_pending)return LAB_AGAIN;mpusvc_view v;lab_status r=mpusvc_decode(&v,bytes,size);
 if(r!=LAB_OK){++s->stats.rejected;reason(s,"SNAPSHOT_CRC_OR_FORMAT_REJECTED");return r;}
 s->diag_now=now;s->diag_instance=v.instance;s->diag_raw_fpga=v.raw_fpga_us;s->diag_raw_rx=v.raw_received_us;s->diag_export=v.export_sequence;s->diag_published=v.published_us;s->diag_session=v.session;s->diag_epoch=v.epoch;s->diag_crc=v.config_crc;s->diag_ag_sequence=v.ag_sequence;
 if(v.source!=s->context.source)return fault(s,"SNAPSHOT_SOURCE_KIND_MISMATCH");
 if(!(v.flags&MPUSVC_SEEN)||!v.accepted){reason(s,"WAIT_FIRST_SERVICE_SAMPLE");return LAB_AGAIN;}
 if(s->have&&(v.instance!=s->instance||v.session!=s->session||v.epoch!=s->epoch||v.parameter_version!=s->service_version||v.config_crc!=s->service_crc))return request_restart(s,"SERVICE_SESSION_OR_CONFIG_AUTO_NEW_CONTEXT");
 if(!configuration_matches(s,&v))return fault(s,"SERVICE_CONFIG_DIFFERS_FROM_MAPPING_CONFIG");
 if(now<v.published_us||now<v.raw_received_us||now-v.raw_received_us>CONT_ODOM_US||v.flags&MPUSVC_STOPPED){s->stats.pose_valid=s->stats.imu_valid=0;reason(s,"SERVICE_STOPPED_OR_STALE");return LAB_AGAIN;}
 if(s->have&&v.export_sequence<=s->export_sequence){if(v.export_sequence<s->export_sequence)return request_restart(s,"EXPORT_SEQUENCE_AUTO_NEW_CONTEXT");++s->stats.duplicates;return LAB_AGAIN;}
 uint32_t rawseq=(uint32_t)v.raw[8]|(uint32_t)v.raw[9]<<8|(uint32_t)v.raw[10]<<16|(uint32_t)v.raw[11]<<24;
 if(s->have){uint32_t d=rawseq-s->raw_sequence;if(!d){++s->stats.duplicates;s->export_sequence=v.export_sequence;s->published=v.published_us;return LAB_AGAIN;}if(d>=UINT32_C(0x80000000))return request_restart(s,"RAW_SEQUENCE_AUTO_NEW_CONTEXT");
  if(v.published_us<s->published)return request_restart(s,"PUBLICATION_CLOCK_AUTO_NEW_CONTEXT");
  if(v.export_sequence>s->export_sequence+1)s->stats.missed_exports+=v.export_sequence-s->export_sequence-1;
 }
 r=clock_observe(s,&v);if(r!=LAB_OK)return r;
 s->instance=v.instance;s->session=v.session;s->epoch=v.epoch;s->service_version=v.parameter_version;s->service_crc=v.config_crc;
 s->raw_sequence=rawseq;s->raw_fpga=v.raw_fpga_us;s->raw_rx=v.raw_received_us;s->published=v.published_us;s->export_sequence=v.export_sequence;s->have=1;++s->stats.exports;s->stats.raw_received_us=v.raw_received_us;
 int ready=v.bias_ready&&v.counts_mask==15&&v.raw[6]==0x71&&(v.raw[5]&(I9_AG_VALID|I9_MPU_READY))==(I9_AG_VALID|I9_MPU_READY);
 reason(s,!v.bias_ready?"KEEP_STILL_FOR_SERVICE_BIAS":!ready?"SERVICE_SENSOR_INVALID":s->config.experimental?"SNAPSHOT_EXPERIMENTAL_TIMING":"SNAPSHOT_TIMING_UNKNOWN_READONLY");
 if(v.have_pose&&(!s->have_pose||v.pose_sequence!=s->pose_sequence)){
  if(s->have_pose&&(v.pose_fpga_us<=s->pose_fpga||(uint32_t)(v.pose_sequence-s->pose_sequence)>=UINT32_C(0x80000000)))return drop_sample(s,"POSE_SEQUENCE_OR_TIME_ROLLBACK_DROP");
  uint64_t t,error;r=map_stamp(s,v.pose_fpga_us,v.pose_received_us,&t,&error);if(r<0)return r;
  if(r==LAB_OK&&now>=v.pose_received_us&&now-v.pose_received_us<=CONT_ODOM_US){
   if(s->have_pose&&t<=s->pose_mapped)return drop_sample(s,"POSE_MAPPED_TIME_ROLLBACK_DROP");
   /* 最新快照中的IMU可能新于保留位姿，旧IMU准确年龄未导出，因此覆盖完整50ms准入范围，不能借新IMU时间标记旧位姿。复用的里程计报文还将FPGA时间量化到毫秒。


 */
   uint64_t age=v.pose_imu_age_known?v.pose_imu_age_us:v.pose_fpga_us>=v.ag_fpga_us?v.pose_fpga_us-v.ag_fpga_us:50000;
   if(error!=UINT64_MAX)error+=1000+s->config.imu_error_us+(age<50000?age:50000);
   lab_live_sample *e=s->scratch;memset(e,0,sizeof(*e));e->type=LAB_EVENT_POSE;e->meta=meta(s,LAB_REAL_ODOMETRY,202,v.pose_sequence,t,now,error);
   uint32_t q=v.odometry.calibrated?0:LAB_POSE_UNCALIBRATED;
   if(ready&&(v.odometry.quality==ODOM_UNCALIBRATED||v.odometry.quality==ODOM_FUSED||v.odometry.quality==ODOM_WHEELS))q|=LAB_POSE_VALID;
   if(!ready||v.odometry.quality==ODOM_STALE||v.odometry.quality==ODOM_WAIT)q|=LAB_POSE_STALE;
   if(v.odometry.quality==ODOM_GAP)q|=LAB_POSE_GAP;if(v.odometry.quality==ODOM_SLIP||v.odometry.slip)q|=LAB_POSE_SLIP;if(v.odometry.imu_used)q|=LAB_POSE_IMU_USED;
   e->data.odometry=(lab_pose){v.odometry.x,v.odometry.y,v.odometry.heading,v.odometry.total_distance,q,v.odometry.calibrated,v.odometry.uncertainty_m};
   s->stats.pose_valid=(q&LAB_POSE_VALID)&&!(q&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP));if(s->stats.pose_valid)e->meta.flags|=LAB_REAL_DATA_VALID;if(v.odometry.calibrated)e->meta.flags|=LAB_REAL_CALIBRATED;
   r8_deskew_observe(&s->deskew,e);enqueue(s,e);s->stats.pose_time_us=t;s->stats.pose_error_us=error;++s->stats.pose_samples;
   memset(e,0,sizeof(*e));e->type=LAB_EVENT_ENCODER;e->meta=meta(s,LAB_REAL_ODOMETRY,203,v.pose_sequence,t,now,error);memcpy(e->data.encoder.counts,v.odometry.raw_counts,sizeof(v.odometry.raw_counts));e->data.encoder.valid_mask=ready?15:0;e->data.encoder.counts_per_revolution=v.counts_per_rev;if(ready)e->meta.flags|=LAB_REAL_DATA_VALID;enqueue(s,e);
   s->have_pose=1;s->pose_sequence=v.pose_sequence;s->pose_fpga=v.pose_fpga_us;s->pose_mapped=t;
  }
 }
 if(v.ag_received_us&&(!s->have_imu||v.ag_sequence!=s->ag_sequence)){
  uint64_t t,error;r=map_stamp(s,v.ag_fpga_us,v.ag_received_us,&t,&error);if(r<0)return r;
  if(r==LAB_OK&&now>=v.ag_received_us&&now-v.ag_received_us<=150000){
   if(s->have_imu&&(v.ag_fpga_us<=s->imu_fpga||t<=s->imu_mapped))return drop_sample(s,"IMU_MAPPED_TIME_ROLLBACK_DROP");
   lab_live_sample *e=s->scratch;memset(e,0,sizeof(*e));e->type=LAB_EVENT_IMU;e->meta=meta(s,LAB_REAL_ODOMETRY,204,v.ag_sequence,t,now,error==UINT64_MAX?error:error+s->config.imu_error_us);
   e->data.imu.yaw_rad=v.rpy[2];e->data.imu.yaw_rate_rad_s=v.gyro[2]-v.bias[2];s->stats.imu_valid=ready;
   if(ready){e->meta.flags|=LAB_REAL_DATA_VALID;e->data.imu.valid_flags=LAB_IMU_YAW_VALID|LAB_IMU_RATE_VALID;}enqueue(s,e);s->stats.imu_time_us=t;++s->stats.imu_samples;
   s->have_imu=1;s->ag_sequence=v.ag_sequence;s->imu_fpga=v.ag_fpga_us;s->imu_mapped=t;
  }
 }
 deskew_release(s,now);return LAB_OK;
}
lab_status lab_real_snapshot_scan(lab_real_snapshot *s,const uint8_t *p,size_t n,uint64_t now){
 if(!s||!p)return LAB_ERR_ARGUMENT;if(s->stats.failed)return LAB_ERR_CONTEXT;if(s->restart_pending)return LAB_AGAIN;
 if(n<64+SCAN_HEADER||n>64+SCAN_PACKET_MAX||memcmp(p,"LT01",4)||p[4]!=1||p[5]!=3||p[6]||p[7]!=64||be32(p+40)!=n-64){++s->stats.rejected;return LAB_ERR_FORMAT;}
 for(unsigned i=36;i<40;++i)if(p[i])return LAB_ERR_FORMAT;for(unsigned i=44;i<60;++i)if(p[i])return LAB_ERR_FORMAT;
 if(lab_crc32_extend(lab_crc32(p,60),p+64,n-64)!=be32(p+60)){++s->stats.rejected;return LAB_ERR_CRC;}
 uint32_t session=be32(p+8),seq=be32(p+12),span=be32(p+32);uint64_t first=be64(p+16),returned=be64(p+24);
 if(!session||!span||span>250000||first>returned||returned>now||span>returned-first||memcmp(p+64,"LS01",4)||be32(p+72)!=session||be32(p+76)!=seq||be64(p+80)!=returned){++s->stats.rejected;return LAB_ERR_TIME;}
 if(s->have_scan&&session!=s->scan_session){fprintf(stderr,"MAP_LIDAR_CONTEXT old_session=%u new_session=%u old_sequence=%u new_sequence=%u first_us=%llu returned_us=%llu span_us=%u now_us=%llu message_zh=雷达采集实例变化需重建地图会话\n",s->scan_session,session,s->scan_sequence,seq,(unsigned long long)first,(unsigned long long)returned,span,(unsigned long long)now);s->diag_now=now;return request_restart(s,"LIDAR_SESSION_AUTO_NEW_CONTEXT");}
 if(s->have_scan){uint32_t d=seq-s->scan_sequence;if(!d||d>=UINT32_C(0x80000000))return LAB_AGAIN;}
 if(s->have_timing&&session==s->timing_session&&seq==s->timing_sequence&&(first!=s->first||returned!=s->returned||span!=s->span))return LAB_ERR_FORMAT;
 s->have_timing=1;s->timing_session=session;s->timing_sequence=seq;s->first=first;s->returned=returned;s->span=span;
 int r=scan_receive(&s->receiver,p+64,n-64,now/1000);if(r<0)return LAB_ERR_FORMAT;if(!r)return LAB_AGAIN;
 lab_live_sample *e=s->scratch;memset(e,0,sizeof(*e));e->type=LAB_EVENT_SCAN;e->data.scan=s->receiver.latest;
 e->meta=meta(s,LAB_REAL_USB,201,seq,first+span/2,now,s->config.experimental?s->config.lidar_error_us+(span+1)/2:UINT64_MAX);e->meta.stream_session=session;
 for(unsigned i=0;i<e->data.scan.count;++i)if(e->data.scan.points[i].quality&&e->data.scan.points[i].distance_q2){e->meta.flags|=LAB_REAL_DATA_VALID;break;}
 if(!lab_live_sample_valid(e))return LAB_ERR_FORMAT;
 if(s->have_deskew_pending)++s->stats.deskew_replaced;
 *s->deskew_pending=*e;s->deskew_first=first;s->deskew_span=span;s->deskew_started=now;
 s->have_deskew_pending=1;s->stats.deskew_pending=1;++s->stats.deskew_waits;
 s->have_scan=1;s->scan_session=session;s->scan_sequence=seq;deskew_release(s,now);return LAB_OK;
}
lab_status lab_real_snapshot_take(lab_real_snapshot *s,uint64_t now,lab_live_sample *out){if(!s||!out)return LAB_ERR_ARGUMENT;if(s->stats.failed)return LAB_ERR_CONTEXT;deskew_release(s,now);if(!s->count||s->queue[s->head].meta.received_us>now)return LAB_AGAIN;*out=s->queue[s->head];s->head=(s->head+1)%8;--s->count;return LAB_OK;}
unsigned lab_real_snapshot_pending(const lab_real_snapshot*s){return s?s->count:0;}
void lab_real_snapshot_health(const lab_real_snapshot *s,uint64_t now,lab_live_health *out){
 if(!s||!out)return;memset(out,0,sizeof(*out));out->context=s->context;out->time_us=now;out->scan_time_us=s->stats.scan_time_us;out->odometry_time_us=out->encoder_time_us=s->stats.pose_time_us;out->imu_time_us=s->stats.imu_time_us;
 out->accepted=s->stats.exports+s->stats.scans;out->rejected=s->stats.rejected;out->gaps=s->stats.missed_exports+s->stats.queue_drops;
 int live=s->have&&!s->stats.failed&&!s->restart_pending&&now>=s->stats.raw_received_us&&now-s->stats.raw_received_us<=CONT_ODOM_US;
 out->odometry_valid=live&&s->stats.pose_valid&&now>=s->stats.pose_time_us&&now-s->stats.pose_time_us<=CONT_ODOM_US;out->encoder_valid_mask=out->odometry_valid?15:0;
 out->imu_valid_flags=live&&s->stats.imu_valid&&now>=s->stats.imu_time_us&&now-s->stats.imu_time_us<=150000?LAB_IMU_YAW_VALID|LAB_IMU_RATE_VALID:0;
 out->scan_valid=!s->stats.failed&&s->stats.scan_valid&&now>=s->stats.scan_time_us&&now-s->stats.scan_time_us<=CONT_SCAN_US;
 snprintf(out->reason,sizeof(out->reason),"%s",s->have&&!live&&!s->stats.failed?"SNAPSHOT_STALE_MAP_WAIT":s->stats.reason);
}
void lab_real_snapshot_stats_get(const lab_real_snapshot *s,lab_real_snapshot_stats *out){if(s&&out)*out=s->stats;}
void lab_real_snapshot_close(lab_real_snapshot *s){if(s){free(s->queue);free(s->scratch);free(s->deskew_pending);free(s->deskew_output);free(s);}}
