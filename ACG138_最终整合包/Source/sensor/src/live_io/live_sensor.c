#include "live_sensor.h"
#include "lab_codec.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define LI_PI 3.14159265358979323846

struct li_source {
 li_config config;
 li_context context;
 li_stats stats;
 struct odom_model fusion;
 struct scan_receiver receiver;
 struct scan_frame accepted_scan;
 li_pose_sample history[LAB_POSE_HISTORY_MAX];
 li_imu_sample imu;
 li_meta scan_meta;
 uint64_t now_us,last_shared_stamp;
 uint32_t last_accepted,last_shared_session,last_odom_epoch,last_imu_token,last_od01_seq;
 int have_now,have_shared,have_od01,have_scan,have_imu,reset_required;
};
static int ctx_valid(const li_context *c) {
 return c&&c->timeline_id&&c->origin_epoch&&c->odom_instance&&c->scan_instance&&
  (c->run_kind==LI_REAL_READONLY||c->run_kind==LI_OFFLINE_TEST);
}
static int newer(uint32_t a,uint32_t b) { uint32_t d=a-b;return d&&d<UINT32_C(0x80000000); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3]; }
static lab_status reject(li_source *s,lab_status result) {
 ++s->stats.rejected;
 if(result==LAB_ERR_CONTEXT) ++s->stats.context_changes;
 if(result==LAB_ERR_TIME) ++s->stats.time_errors;
 return result;
}
static lab_status changed(li_source *s) {s->reset_required=1;return reject(s,LAB_ERR_CONTEXT);}
static lab_status observe(li_source *s,uint64_t now) {
 if(s->reset_required)return LAB_ERR_CONTEXT;
 if(s->have_now&&now<s->now_us) {s->reset_required=1;return reject(s,LAB_ERR_TIME);}
 s->now_us=now;s->have_now=1;return LAB_OK;
}
static int fusion_valid(const struct odom_config *c) {
 if(!isfinite(c->track_mm)||c->track_mm<50||c->track_mm>1000||
  !isfinite(c->yaw_sign)||fabs(c->yaw_sign)!=1||!isfinite(c->yaw_offset)||
  !isfinite(c->correction_tau)||c->correction_tau<.05||c->correction_tau>10||
  c->left_mask<1||c->left_mask>14||(c->calibrated!=0&&c->calibrated!=1))return 0;
 unsigned left=0;
 for(int i=0;i<4;++i){left+=(c->left_mask>>i)&1;
  if((c->signs[i]!=1&&c->signs[i]!=-1)||!isfinite(c->mm_per_count[i])||c->mm_per_count[i]<.01||c->mm_per_count[i]>5)return 0;}
 return left==2;
}
void li_config_default(li_config *c) {
 if(!c)return;memset(c,0,sizeof(*c));c->input=LI_INPUT_SHARED;c->history_capacity=256;
 c->counts_per_revolution=1320;c->max_pose_age_us=c->max_scan_age_us=250000;
 c->max_interpolation_gap_us=100000;c->receipt_time_error_us=LI_TIME_ERROR_UNKNOWN;
 odom_default_config(&c->fusion);
}
lab_status li_open(li_source **out,const li_config *c,const li_context *ctx) {
 if(!out)return LAB_ERR_ARGUMENT;*out=NULL;
 if(!c||!ctx_valid(ctx)||(c->input!=LI_INPUT_SHARED&&c->input!=LI_INPUT_OD01_CALLBACK)||
  c->history_capacity<2||c->history_capacity>LAB_POSE_HISTORY_MAX||!c->counts_per_revolution||c->counts_per_revolution>1000000||
  c->allow_receipt_proxy>1||!c->receipt_time_error_us||!c->max_pose_age_us||c->max_pose_age_us>1000000||
  !c->max_scan_age_us||c->max_scan_age_us>1000000||!c->max_interpolation_gap_us||c->max_interpolation_gap_us>1000000||!fusion_valid(&c->fusion))return LAB_ERR_ARGUMENT;
 li_source *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;
 s->config=*c;s->context=*ctx;odom_init(&s->fusion,&c->fusion);*out=s;return LAB_OK;
}
lab_status li_reset(li_source *s,const li_context *ctx) {
 if(!s||!ctx_valid(ctx))return LAB_ERR_ARGUMENT;
 if(ctx->timeline_id==s->context.timeline_id&&ctx->origin_epoch==s->context.origin_epoch)return LAB_ERR_CONTEXT;
 li_config config=s->config;li_stats stats=s->stats;
 memset(s,0,sizeof(*s));s->config=config;s->context=*ctx;s->stats=stats;++s->stats.resets;
 odom_init(&s->fusion,&config.fusion);return LAB_OK;
}
static li_meta meta(li_source *s,uint32_t session,uint32_t sequence,uint64_t t,uint64_t rx) {
 li_meta m={.context=s->context,.producer_session=session,.sequence=sequence,
  .time_basis=LI_HOST_RECEIPT_PROXY,.flags=LI_HOST_TIME|LI_ESTIMATED_MEASUREMENT_TIME,
  .time_us=t,.received_us=rx,.time_error_us=s->config.receipt_time_error_us};
 if(s->context.run_kind==LI_OFFLINE_TEST)m.flags|=LI_SYNTHETIC_TEST;
 return m;
}
static int state_valid(const struct odom_state *p) {
 return p->quality<=ODOM_SLIP&&p->points<=ODOM_POINTS&&p->calibrated<=1&&p->imu_used<=1&&p->slip<=1&&
  isfinite(p->x)&&isfinite(p->y)&&isfinite(p->heading)&&isfinite(p->total_distance)&&p->total_distance>=0&&
  fabs(p->x)<=1e6&&fabs(p->y)<=1e6&&fabs(p->heading)<=1e6&&p->total_distance<=1e9&&
  isfinite(p->uncertainty_m)&&p->uncertainty_m>=0&&p->uncertainty_m<=1e6&&isfinite(p->origin_yaw)&&fabs(p->origin_yaw)<=1e6&&p->stamp_ms<=UINT64_MAX/1000;
}
static lab_status append_pose(li_source *s,const struct odom_state *p,uint32_t sequence,
                              uint32_t count_mask,uint64_t time,uint64_t rx,int inferred) {
 if(!state_valid(p))return reject(s,LAB_ERR_FORMAT);
 if(time>rx)return reject(s,LAB_ERR_TIME);
 if(s->stats.pose_history&&time<s->history[s->stats.pose_history-1].meta.time_us)return reject(s,LAB_ERR_TIME);
 li_pose_sample sample={0};sample.meta=meta(s,p->session,sequence,time,rx);sample.legacy_quality=p->quality;
 sample.pose=(lab_pose){p->x,p->y,remainder(p->heading,2*LI_PI),p->total_distance,0,p->calibrated,p->uncertainty_m};
 if(!p->calibrated)sample.pose.quality|=LAB_POSE_UNCALIBRATED;
 if(p->quality==ODOM_UNCALIBRATED||p->quality==ODOM_FUSED||p->quality==ODOM_WHEELS)sample.pose.quality|=LAB_POSE_VALID;
 if(p->quality==ODOM_WAIT||p->quality==ODOM_STALE)sample.pose.quality|=LAB_POSE_STALE;
 if(p->quality==ODOM_GAP)sample.pose.quality|=LAB_POSE_GAP;
 if(p->quality==ODOM_SLIP||p->slip)sample.pose.quality|=LAB_POSE_SLIP;
 if(p->imu_used)sample.pose.quality|=LAB_POSE_IMU_USED;
 memcpy(sample.counts,p->raw_counts,sizeof(sample.counts));sample.counts_valid_mask=count_mask;
 sample.counts_per_revolution=s->config.counts_per_revolution;
 if(inferred)sample.meta.flags|=LI_COUNTS_MASK_INFERRED;
 if(s->stats.pose_history&&time==s->history[s->stats.pose_history-1].meta.time_us){
  /* 毫秒量化可能产生同一时间戳的多个独立样本；缓存保留其中最新观测。 */

  s->history[s->stats.pose_history-1]=sample;++s->stats.pose_samples;++s->stats.time_aliases;return LAB_OK;
 }
 if(s->stats.pose_history==s->config.history_capacity){
  memmove(s->history,s->history+1,(s->stats.pose_history-1)*sizeof(*s->history));--s->stats.pose_history;}
 s->history[s->stats.pose_history++]=sample;++s->stats.pose_samples;return LAB_OK;
}
lab_status li_push_shared(li_source *s,const struct odom_shared *p,uint64_t instance,uint64_t now) {
 if(!s||!p)return LAB_ERR_ARGUMENT;
 if(s->config.input!=LI_INPUT_SHARED)return LAB_ERR_READONLY;
 if(instance!=s->context.odom_instance)return reject(s,LAB_ERR_CONTEXT);
 lab_status r=observe(s,now);if(r!=LAB_OK)return r;
 if(p->magic!=ODOM_MAGIC||p->version!=1||p->bytes!=sizeof(*p))return reject(s,LAB_ERR_VERSION);
 if(p->sequence&1)return LAB_AGAIN;
 const struct odom_state *v=&p->state;
 if(!state_valid(v))return reject(s,LAB_ERR_FORMAT);
 if(!v->accepted||!v->session)return LAB_AGAIN;
 if(s->have_shared){
  if(v->session!=s->last_shared_session||v->stamp_ms<s->last_shared_stamp||
   (v->accepted!=s->last_accepted&&!newer(v->accepted,s->last_accepted)))return changed(s);
  if(v->stamp_ms==s->last_shared_stamp&&v->accepted==s->last_accepted){++s->stats.duplicates;return LAB_AGAIN;}
 }
 if(v->stamp_ms*1000>now)return reject(s,LAB_ERR_TIME);
 uint32_t mask=(v->quality==ODOM_UNCALIBRATED||v->quality==ODOM_FUSED||v->quality==ODOM_WHEELS)?15:0;
 r=append_pose(s,v,v->accepted,mask,v->stamp_ms*1000,now,1);
 if(r==LAB_OK){s->have_shared=1;s->last_shared_session=v->session;s->last_shared_stamp=v->stamp_ms;s->last_accepted=v->accepted;}
 return r;
}
lab_status li_push_od01(li_source *s,const uint8_t *p,size_t n,uint64_t instance,uint64_t rx) {
 if(!s||!p)return LAB_ERR_ARGUMENT;
 if(s->config.input!=LI_INPUT_OD01_CALLBACK)return LAB_ERR_READONLY;
 if(instance!=s->context.odom_instance)return reject(s,LAB_ERR_CONTEXT);
 lab_status r=observe(s,rx);if(r!=LAB_OK)return r;
 if(n!=64||memcmp(p,"OD01",4)||p[4]!=1||p[5]>1||p[6]>15||p[7]>7||le32(p+52)||le32(p+56)||
  lab_crc32(p,60)!=le32(p+60))return reject(s,LAB_ERR_FORMAT);
 uint32_t seq=le32(p+8),ms=le32(p+12),epoch=le32(p+16),token=le32(p+48);
 if(s->have_od01){
  if(epoch!=s->last_odom_epoch||token!=s->last_imu_token)return changed(s);
  if(!newer(seq,s->last_od01_seq)){++s->stats.duplicates;return LAB_AGAIN;}
  /* 正常32位毫秒回绕仍是正向推进，倒退或重启另行处理。 */
  if((ms-s->fusion.fpga_ms)>=UINT32_C(0x80000000))return changed(s);
 }
 struct odom_model next=s->fusion;
 int accepted=odom_push(&next,p,n,rx/1000);
 if(next.state.session&&s->have_od01&&next.state.session!=s->fusion.state.session)return changed(s);
 s->fusion=next;s->have_od01=1;s->last_odom_epoch=epoch;s->last_imu_token=token;s->last_od01_seq=seq;
 uint32_t bits=le32(p+36);float yaw;memcpy(&yaw,&bits,4);
 uint32_t age=le32(p+40);
 li_imu_sample imu={0};imu.meta=meta(s,s->fusion.state.session,le32(p+44),rx,rx);imu.remote_fpga_ms=ms;imu.age_ms=age;imu.calibration_token=token;
 if(isfinite(yaw))imu.yaw_rad=yaw;
 if(!p[7]&&age<=150&&isfinite(yaw)&&fabs(yaw)<1000000&&rx>=(uint64_t)age*1000){
  imu.meta.time_us=rx-(uint64_t)age*1000;imu.valid_flags=LAB_IMU_YAW_VALID;}
 if(!s->have_imu||newer(imu.meta.sequence,s->imu.meta.sequence))s->imu=imu;
 else if(!imu.valid_flags)s->imu.valid_flags=0;
 s->have_imu=1;
 r=append_pose(s,&s->fusion.state,seq,p[5]?0:p[6],rx,rx,0);
 if(r!=LAB_OK)return r;
 if(accepted<0){++s->stats.rejected;return LAB_AGAIN;} /* 保留间隔屏障。 */
 return LAB_OK;
}
lab_status li_push_scan(li_source *s,const struct scan_frame *f,uint64_t instance,uint64_t now) {
 if(!s||!f)return LAB_ERR_ARGUMENT;
 if(instance!=s->context.scan_instance)return reject(s,LAB_ERR_CONTEXT);
 lab_status r=observe(s,now);if(r!=LAB_OK)return r;
 if(!f->session||!f->count||f->count>SCAN_MAX_POINTS||f->source<1||f->source>2||(f->flags&~1u))return reject(s,LAB_ERR_FORMAT);
 if(s->have_scan){
  if(f->session!=s->scan_meta.producer_session||f->source!=s->accepted_scan.source)return changed(s);
  if(!newer(f->sequence,s->scan_meta.sequence)){++s->stats.duplicates;return LAB_AGAIN;}
 }
 if((f->flags&1)&&f->timestamp_us>now)return reject(s,LAB_ERR_TIME);
 if((f->flags&1)&&s->have_scan&&f->timestamp_us<=s->scan_meta.time_us)return reject(s,LAB_ERR_TIME);
 /* scan_receive可能已持有同一个目标对象。 */
 if(f!=&s->accepted_scan)s->accepted_scan=*f;
 s->scan_meta=meta(s,f->session,f->sequence,(f->flags&1)?f->timestamp_us:0,now);
 s->scan_meta.time_basis=(f->flags&1)?LI_HOST_SCAN_END:LI_REMOTE_UNMAPPED;
 s->scan_meta.time_error_us=LI_TIME_ERROR_UNKNOWN;
 s->scan_meta.flags&=~(uint32_t)(LI_ESTIMATED_MEASUREMENT_TIME|LI_HOST_TIME);
 if(f->flags&1)s->scan_meta.flags|=LI_HOST_TIME|LI_ESTIMATED_MEASUREMENT_TIME;
 s->have_scan=1;++s->stats.scan_samples;return LAB_OK;
}
lab_status li_push_ls01(li_source *s,const uint8_t *p,size_t n,uint64_t instance,uint64_t now) {
 if(!s||!p)return LAB_ERR_ARGUMENT;
 if(instance!=s->context.scan_instance)return reject(s,LAB_ERR_CONTEXT);
 lab_status r=observe(s,now);if(r!=LAB_OK)return r;
 if(s->have_scan&&n>=SCAN_HEADER&&n<=SCAN_PACKET_MAX&&!memcmp(p,"LS01",4)&&
  scan_crc32(p,n)==be32(p+36)&&
  (be32(p+8)!=s->scan_meta.producer_session||p[5]!=s->accepted_scan.source))return changed(s);
 if(n>=SCAN_HEADER&&n<=SCAN_PACKET_MAX&&!memcmp(p,"LS01",4)&&scan_crc32(p,n)==be32(p+36)&&
  (p[7]&1u)){
  uint64_t t=((uint64_t)be32(p+16)<<32)|be32(p+20);if(t>now)return reject(s,LAB_ERR_TIME);
 }
 int result=scan_receive(&s->receiver,p,n,now/1000);
 if(result<0)return reject(s,LAB_ERR_FORMAT);
 if(!result)return LAB_AGAIN;
 return li_push_scan(s,&s->receiver.latest,instance,now);
}
static lab_status read_at(li_source *s,uint64_t now) {
 if(!s)return LAB_ERR_ARGUMENT;
 if(s->reset_required)return LAB_ERR_CONTEXT;
 if(s->have_now&&now<s->now_us){s->reset_required=1;return reject(s,LAB_ERR_TIME);}
 return LAB_OK;
}
lab_status li_latest_pose(li_source *s,uint64_t now,li_pose_sample *out) {
 if(out)memset(out,0,sizeof(*out));if(!s||!out)return LAB_ERR_ARGUMENT;
 lab_status r=read_at(s,now);if(r!=LAB_OK)return r;
 if(!s->stats.pose_history)return LAB_AGAIN;
 *out=s->history[s->stats.pose_history-1];
 if(out->meta.time_us>now)return LAB_ERR_TIME;
 if(now-out->meta.time_us>=s->config.max_pose_age_us)out->pose.quality=(out->pose.quality&~(uint32_t)LAB_POSE_VALID)|LAB_POSE_STALE;
 return LAB_OK;
}
lab_status li_latest_imu(li_source *s,uint64_t now,li_imu_sample *out) {
 if(out)memset(out,0,sizeof(*out));if(!s||!out)return LAB_ERR_ARGUMENT;
 lab_status r=read_at(s,now);if(r!=LAB_OK)return r;
 if(s->config.input==LI_INPUT_SHARED)return LAB_ERR_UNAVAILABLE;
 if(!s->have_imu)return LAB_AGAIN;*out=s->imu;
 if(now<out->meta.time_us)return LAB_ERR_TIME;
 if(now-out->meta.time_us>=s->config.max_pose_age_us)out->valid_flags=0;
 return LAB_OK;
}
lab_status li_latest_scan(li_source *s,uint64_t now,li_scan_view *out) {
 if(out)memset(out,0,sizeof(*out));if(!s||!out)return LAB_ERR_ARGUMENT;
 lab_status r=read_at(s,now);if(r!=LAB_OK)return r;
 if(!s->have_scan)return LAB_AGAIN;
 out->meta=s->scan_meta;out->frame=&s->accepted_scan;out->period_us=out->frame->period_us;
 if(out->meta.flags&LI_HOST_TIME)out->interval_start_us=out->meta.time_us>=out->period_us?out->meta.time_us-out->period_us:0;
 if(now-out->meta.received_us>=s->config.max_scan_age_us)return LAB_AGAIN;
 if((out->meta.flags&LI_HOST_TIME)&&now-out->meta.time_us>=s->config.max_scan_age_us)return LAB_AGAIN;
 return LAB_OK;
}
static int pose_usable(const li_pose_sample *p) {
 return (p->pose.quality&LAB_POSE_VALID)&&!(p->pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP));
}
lab_status li_pose_at(li_source *s,uint64_t t,uint64_t now,li_pose_sample *out) {
 if(out)memset(out,0,sizeof(*out));if(!s||!out)return LAB_ERR_ARGUMENT;
 lab_status r=read_at(s,now);if(r!=LAB_OK)return r;
 if(t>now)return LAB_ERR_TIME;
 if(now-t>=s->config.max_pose_age_us){++s->stats.query_rejected;return LAB_ERR_TIME;}
 uint32_t j=0;while(j<s->stats.pose_history&&s->history[j].meta.time_us<t)++j;
 if(j==s->stats.pose_history){++s->stats.query_waits;return LAB_AGAIN;}
 const li_pose_sample *right=&s->history[j];
 if(right->meta.time_basis==LI_HOST_RECEIPT_PROXY&&!s->config.allow_receipt_proxy){++s->stats.query_rejected;return LAB_ERR_UNAVAILABLE;}
 if(!pose_usable(right)){++s->stats.query_rejected;return LAB_ERR_UNAVAILABLE;}
 if(right->meta.time_us==t){*out=*right;return LAB_OK;}
 if(!j){++s->stats.query_waits;return LAB_AGAIN;}
 const li_pose_sample *left=right-1;
 if(!pose_usable(left)||right->meta.time_us-left->meta.time_us>s->config.max_interpolation_gap_us){++s->stats.query_rejected;return LAB_ERR_UNAVAILABLE;}
 *out=*left;double f=(double)(t-left->meta.time_us)/(double)(right->meta.time_us-left->meta.time_us);
 out->pose.x_m+=f*(right->pose.x_m-left->pose.x_m);out->pose.y_m+=f*(right->pose.y_m-left->pose.y_m);
 out->pose.yaw_rad=remainder(left->pose.yaw_rad+f*remainder(right->pose.yaw_rad-left->pose.yaw_rad,2*LI_PI),2*LI_PI);
 out->pose.distance_m+=f*(right->pose.distance_m-left->pose.distance_m);out->pose.quality|=right->pose.quality;
 out->pose.uncertainty_m=fmax(left->pose.uncertainty_m,right->pose.uncertainty_m);
 out->meta.time_us=t;out->meta.received_us=right->meta.received_us;
 out->meta.time_error_us=left->meta.time_error_us>right->meta.time_error_us?left->meta.time_error_us:right->meta.time_error_us;
 /* 插值得到的轮计数不是原始编码器观测。 */
 memset(out->counts,0,sizeof(out->counts));out->counts_valid_mask=0;
 return LAB_OK;
}
lab_status li_align_latest_scan(li_source *s,uint64_t now,li_scan_view *scan,li_pose_sample *pose) {
 if(!scan||!pose)return LAB_ERR_ARGUMENT;memset(scan,0,sizeof(*scan));memset(pose,0,sizeof(*pose));
 lab_status r=li_latest_scan(s,now,scan);if(r!=LAB_OK)return r;
 if(!(scan->meta.flags&LI_HOST_TIME))return LAB_ERR_TIME;
 return li_pose_at(s,scan->meta.time_us,now,pose);
}
void li_get_stats(const li_source *s,li_stats *out) {
 if(!s||!out)return;*out=s->stats;out->allocated_bytes=sizeof(*s);out->reset_required=(uint32_t)s->reset_required;
 out->capabilities=LI_CAP_POSE|LI_CAP_COUNTS|LI_CAP_SCAN;
 if(s->config.input==LI_INPUT_OD01_CALLBACK)out->capabilities|=LI_CAP_RAW_IMU_YAW;
}
void li_close(li_source *s) { free(s); }
