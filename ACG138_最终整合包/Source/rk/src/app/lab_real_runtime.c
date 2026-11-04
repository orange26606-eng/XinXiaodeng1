#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "lab_real_runtime.h"
#include "lab_real_runtime_control.h"
#include "lab_real_sample_diagnostic.h"
#include <time.h>
#include <math.h>
#include "continuous_policy.h"
#include "s2_navigation_limits.h"
#include "cadence_policy.h"
#include "object_time_policy.h"
#include "object_turn_smooth.h"
#include "chassis_private.h"
#ifdef LAB_REAL_POSIX_SOURCE
#include "phone_control_fence.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct real_stream {uint32_t used,provider,id,session,sequence;uint64_t time_us;} real_stream;
struct lab_real_runtime {
 lab_real_runtime_config config;lab_real_bindings bindings;lab_real_runtime_view view;
 lab_live_sample *sample,*scan;real_stream streams[16];uint64_t received_us,parameter_version,scan_uncertainty,odometry_uncertainty;
 uint64_t map_id,map_version,correction_version;
 lab_real_sample_observer observer;void *observer_user;
 lab_real_object_controller object_controller;void*object_user;obj_turn_smoother object_turn;
 uint64_t last_forwarded_pose_us,enable_started_us,last_stale_log_us;
 uint32_t last_drive_motion,command_sequence;
 uint64_t goal_generation,prepared_generation,start_deadline_us,first_apply_us;int prepared_candidate;int have_forwarded_pose;cadence_budget budget;uint64_t seen_revoke;
 lab_live_sample deferred_scans[4];unsigned deferred_count;
 int have_rx,have_tick,clock_fault,stop_fault,context_fault;
 lab_real_goal recovery_goal;double recovery_goal_x,recovery_goal_y,recovery_goal_yaw;int recovery_goal_pending;
 char sample_reject_reason[96];
};
static uint64_t runtime_pose_age(const lab_real_runtime*r){return r->view.object_mode?OBJECT_CONTROL_POSE_US:r->config.max_pose_age_us;}
static uint64_t runtime_scan_age(const lab_real_runtime*r){return r->view.object_mode?OBJECT_CONTROL_SCAN_US:r->config.max_scan_age_us;}
static uint64_t runtime_feedback_age(const lab_real_runtime*r){return r->view.object_mode?OBJECT_CONTROL_FEEDBACK_US:CONT_FEEDBACK_US;}
static int object_soft_cause(const lab_real_runtime*r,const char*cause){
 if(!r->view.object_mode||r->view.resume_requires_start||r->view.manual_revoked||r->view.feedback.faults||(r->view.feedback.local_stop_sources&0x3eu))return 0;
 if(!strcmp(cause,"SOURCE_CONTEXT_REBUILT_WAIT_NEW_DATA")||!strcmp(cause,"OBJECT_WAIT_FRESH_RAW_SCAN_ODOMETRY")||!strcmp(cause,"OBJECT_PUBLICATION_RAW_SOURCE_STALE")||!strcmp(cause,"INVALID_CHASSIS_FEEDBACK"))return 1;
 if((!strcmp(cause,"PROTECTION_STOP")||!strcmp(cause,"PROTECTION_OUTPUT_DISABLED"))&&r->view.last_status==LAB_AGAIN&&!(r->view.protected_output.reasons&(CH_R_ESTOP|CH_R_MOTOR|CH_R_CLOCK|CH_R_OBSTACLE)))return 1;
 return 0;
}
void lab_real_runtime_config_default(lab_real_runtime_config *c){if(!c)return;memset(c,0,sizeof(*c));c->mode=LAB_REAL_READONLY;c->max_linear_m_s=.10;c->max_angular_rad_s=S2_MAX_YAW_RAD_S;c->max_scan_age_us=c->max_pose_age_us=250000;c->command_valid_us=CONT_COMMAND_US;c->control_session=1;}
void lab_real_runtime_note_cost(lab_real_runtime*r,uint64_t elapsed){if(!r)return;cadence_observe(&r->budget,elapsed);r->view.control_elapsed_us=elapsed;r->view.control_warns=r->budget.warnings;r->view.control_soft_events=r->budget.events;r->view.control_soft_streak=r->budget.streak;r->view.control_load_reduced=r->budget.reduced;if(r->bindings.navigation.reduce_load)r->bindings.navigation.reduce_load(r->bindings.navigation.user,r->budget.reduced);}
static void zero(lab_real_runtime *r,uint64_t now){r->prepared_candidate=0;memset(&r->view.request,0,sizeof(r->view.request));r->view.request.context=r->view.context;r->view.request.generated_us=r->view.request.expires_us=now;memset(&r->view.protected_output,0,sizeof(r->view.protected_output));r->view.protected_output.command=r->view.request;}
static void halt(lab_real_runtime *r,uint64_t now,const char *cause){
 obj_turn_reset(&r->object_turn);r->view.object_turn_waiting=0;r->view.object_turn_selected_w=r->view.object_turn_applied_w=0;r->view.object_turn_reason[0]=0;
 int soft=object_soft_cause(r,cause);
 if(r->view.enabled||r->view.pending_enable)r->view.object_soft_rearm_pending=soft;
 else if(!soft)r->view.object_soft_rearm_pending=0;
 r->start_deadline_us=0;r->first_apply_us=0;
 if(r->view.enabled||r->view.pending_enable){
  snprintf(r->view.last_stop_reason,sizeof(r->view.last_stop_reason),"%s",cause);r->view.last_stop_us=now;
  const char*detail=r->view.navigation.blocking_reason;
  if(!detail[0]||!strcmp(detail,"NONE"))detail=r->view.navigation.reason;
  if(!strcmp(cause,"STALE_OR_INVALID_LOCALIZATION")){
   if(now>=r->view.odometry_time_us&&now-r->view.odometry_time_us+r->odometry_uncertainty>runtime_pose_age(r))detail="ODOMETRY_DATA_STALE";
   else if(now>=r->view.scan_time_us&&now-r->view.scan_time_us+r->scan_uncertainty>runtime_scan_age(r))detail="LIDAR_DATA_STALE";
   else detail=r->view.localization.reason;
  }
  if(!strcmp(cause,"INVALID_CHASSIS_FEEDBACK"))detail=r->view.feedback.reason;
  snprintf(r->view.last_stop_detail,sizeof(r->view.last_stop_detail),"%s",detail);
  fprintf(stderr,"STOP_EVENT cause=%s status=%d now_us=%llu goal_id=%u goal_active=%u enabled=%u pending=%u nav_state=%u nav_phase=%u nav_reason=%s blocking=%s path_points=%u loc_state=%u loc_reason=%s loc_quality=0x%x scan_age_us=%llu scan_unc_us=%llu odom_age_us=%llu odom_unc_us=%llu pose_age_us=%llu match_age_us=%llu fpga_age_us=%llu fpga_faults=%u fpga_reason=%s request_v=%.5f request_w=%.5f protected_reason=%s protected_bits=0x%x\n",
   cause,(int)r->view.last_status,(unsigned long long)now,r->view.goal.id,r->view.has_goal,r->view.enabled,r->view.pending_enable,
   r->view.navigation.state,r->view.navigation.phase,r->view.navigation.reason,r->view.navigation.blocking_reason,r->view.navigation.path_count,
   r->view.localization.state,r->view.localization.reason,r->view.localization.pose.quality,
   (unsigned long long)(now>=r->view.scan_time_us?now-r->view.scan_time_us:UINT64_MAX),(unsigned long long)r->scan_uncertainty,
   (unsigned long long)(now>=r->view.odometry_time_us?now-r->view.odometry_time_us:UINT64_MAX),(unsigned long long)r->odometry_uncertainty,
   (unsigned long long)(now>=r->view.localization.pose_time_us?now-r->view.localization.pose_time_us:UINT64_MAX),
   (unsigned long long)(now>=r->view.localization.last_matched_scan_us?now-r->view.localization.last_matched_scan_us:UINT64_MAX),
   (unsigned long long)(now>=r->view.feedback.time_us?now-r->view.feedback.time_us:UINT64_MAX),r->view.feedback.faults,r->view.feedback.reason,
   r->view.request.linear_m_s,r->view.request.angular_rad_s,r->view.protected_output.reason,r->view.protected_output.reasons);
 }
 zero(r,now);r->last_drive_motion=0;if(r->view.enabled||r->view.pending_enable){fprintf(stderr,"CHASSIS_HALT_DIAG stage=BEGIN enabled=%u pending=%u now_us=%llu\n",r->view.enabled,r->view.pending_enable,(unsigned long long)now);lab_status a=r->bindings.chassis.ops->stop(r->bindings.chassis.user,now);++r->view.hardware_calls;fprintf(stderr,"CHASSIS_HALT_DIAG stage=STOP result=%d\n",(int)a);lab_status b=r->bindings.chassis.ops->set_enabled(r->bindings.chassis.user,0,r->config.control_session,now);++r->view.hardware_calls;fprintf(stderr,"CHASSIS_HALT_DIAG stage=DISABLE result=%d\n",(int)b);r->view.enabled=r->view.pending_enable=0;if(a!=LAB_OK||b!=LAB_OK)r->stop_fault=1;}
}
static lab_status finish(lab_real_runtime *r,lab_status s,const char *why,uint64_t now){r->view.last_status=s;snprintf(r->view.reason,sizeof(r->view.reason),"%s",why);if(s!=LAB_OK){if(r->view.enabled||r->view.pending_enable)fprintf(stderr,"CHASSIS_HALT_DIAG stage=TRIGGER status=%d reason=%s now_us=%llu\n",(int)s,why,(unsigned long long)now);halt(r,now,why);}if(r->stop_fault){r->view.last_status=LAB_ERR_IO;snprintf(r->view.reason,sizeof(r->view.reason),"STOP_TRANSPORT_FAILURE_CONFIRM_DEVICE_STOPPED");return LAB_ERR_IO;}return s;}
/* 反馈读取可能收到晚于调用方tick的报文，推进消费者时钟，不回填接收时间或刷新源及命令日期。
 */
static int s5_after_feedback_clock(lab_real_runtime*r,uint64_t*now){
#ifdef LAB_REAL_POSIX_SOURCE
 if(r->config.allow_hardware_output&&r->view.context.source==LAB_REAL_HARDWARE){
  struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;
  uint64_t actual=(uint64_t)t.tv_sec*1000000u+t.tv_nsec/1000u;if(actual<*now)return 0;
  *now=actual;r->view.time_us=actual;
 }
#else
 (void)r;(void)now;
#endif
 return 1;
}
static int ops_valid(const lab_real_bindings *b){
 if(b->sensors.ops&&(b->sensors.ops->api_version!=LAB_REAL_API_VERSION||b->sensors.ops->struct_bytes!=sizeof(lab_live_source_ops)||!b->sensors.ops->poll||!b->sensors.ops->health||!b->sensors.ops->reset))return 0;
 if(b->localization.ops&&(b->localization.ops->api_version!=LAB_REAL_API_VERSION||b->localization.ops->struct_bytes!=sizeof(lab_localization_ops)||!b->localization.ops->push||!b->localization.ops->snapshot||!b->localization.ops->pose_at||!b->localization.ops->reset))return 0;
 if(b->chassis.ops&&(b->chassis.ops->api_version!=LAB_REAL_API_VERSION||b->chassis.ops->struct_bytes!=sizeof(lab_chassis_ops)||!b->chassis.ops->feedback||!b->chassis.ops->protect||!b->chassis.ops->set_enabled||!b->chassis.ops->apply||!b->chassis.ops->stop||!b->chassis.ops->reset))return 0;return 1;
}
lab_status lab_real_runtime_open(lab_real_runtime **out,const lab_real_runtime_config *c,const lab_real_context *ctx,const lab_real_bindings *b){
 if(!out)return LAB_ERR_ARGUMENT;*out=NULL;
 if(!c||!b||!lab_real_context_valid(ctx)||ctx->clock==LAB_REAL_UNMAPPED_CLOCK||c->mode<1||c->mode>5||c->allow_hardware_output>1||c->geometry_verification_required>1||c->geometry_verified>1||!c->control_session||
 !c->max_scan_age_us||c->max_scan_age_us>CONT_SCAN_US||!c->max_pose_age_us||c->max_pose_age_us>1000000||!c->command_valid_us||c->command_valid_us>CONT_COMMAND_US||
 !isfinite(c->max_linear_m_s)||c->max_linear_m_s<=0||c->max_linear_m_s>1||!isfinite(c->max_angular_rad_s)||c->max_angular_rad_s<=0||c->max_angular_rad_s>3||!ops_valid(b))return LAB_ERR_ARGUMENT;
 if(c->allow_hardware_output&&(c->mode!=LAB_REAL_NAVIGATION||ctx->source!=LAB_REAL_HARDWARE||ctx->clock!=LAB_REAL_MONOTONIC))return LAB_ERR_READONLY;
 lab_real_runtime *r=calloc(1,sizeof(*r));if(!r)return LAB_ERR_NOMEM;r->config=*c;r->bindings=*b;r->view.context=*ctx;r->view.mode=c->mode;
 r->sample=calloc(1,sizeof(*r->sample));r->scan=calloc(1,sizeof(*r->scan));if(!r->sample||!r->scan){lab_real_runtime_close(r);return LAB_ERR_NOMEM;}
 r->view.missing_modules=(!b->sensors.ops?1u:0u)|((c->mode>=LAB_REAL_MAPPING&&!b->localization.ops)?2u:0u)|((c->mode>=LAB_REAL_SHADOW&&!b->navigation.step)?4u:0u)|((c->mode==LAB_REAL_NAVIGATION&&!b->chassis.ops)?8u:0u);
 snprintf(r->view.reason,sizeof(r->view.reason),"%s",r->view.missing_modules?"WAITING_FOR_MODULES":"WAITING_FOR_DATA");*out=r;return LAB_OK;
}
static lab_status sample_reject(lab_real_runtime*r,const lab_live_sample*e,uint64_t now,lab_status status,const char*reason,const real_stream*old){
 snprintf(r->sample_reject_reason,sizeof(r->sample_reject_reason),"SENSOR_REJECT_%s",reason);
 const lab_real_meta*m=&e->meta;
 if(!old)for(unsigned i=0;i<16;i++)if(r->streams[i].used&&r->streams[i].provider==m->provider&&r->streams[i].id==m->stream_id){old=r->streams+i;break;}
 fprintf(stderr,"SENSOR_SAMPLE_REJECT status=%d reason=%s now_us=%llu object_mode=%u sample_type=%u sample_reserved=%u expected_context=%u,%u,%u,%u context=%u,%u,%u,%u provider=%u stream=%u session=%u sequence=%u flags=0x%x meta_reserved=%u source_us=%llu received_us=%llu uncertainty_us=%llu parameter_version=%llu expected_parameter_version=%llu last_received_us=%llu prior_stream=%u prior_session=%u prior_sequence=%u prior_source_us=%llu\n",(int)status,reason,(unsigned long long)now,r->view.object_mode,e->type,e->reserved,r->view.context.source,r->view.context.clock,r->view.context.timeline_id,r->view.context.origin_epoch,m->context.source,m->context.clock,m->context.timeline_id,m->context.origin_epoch,m->provider,m->stream_id,m->stream_session,m->sequence,m->flags,m->reserved,(unsigned long long)m->time_us,(unsigned long long)m->received_us,(unsigned long long)m->time_uncertainty_us,(unsigned long long)m->parameter_version,(unsigned long long)r->parameter_version,(unsigned long long)(r->have_rx?r->received_us:0),old?old->used:0,old?old->session:0,old?old->sequence:0,(unsigned long long)(old?old->time_us:0));
 switch(e->type){
 case LAB_EVENT_SCAN:fprintf(stderr,"SENSOR_SAMPLE_PAYLOAD type=SCAN count=%u source=%u flags=0x%x geometry=%u session=%u sequence=%u timestamp_us=%llu period_us=%u received_ms=%llu\n",e->data.scan.count,e->data.scan.source,e->data.scan.flags,e->data.scan.geometry_kind,e->data.scan.session,e->data.scan.sequence,(unsigned long long)e->data.scan.timestamp_us,e->data.scan.period_us,(unsigned long long)e->data.scan.received_ms);break;
 case LAB_EVENT_POSE:fprintf(stderr,"SENSOR_SAMPLE_PAYLOAD type=POSE x=%.17g y=%.17g yaw=%.17g distance=%.17g uncertainty=%.17g quality=0x%x calibrated=%u\n",e->data.odometry.x_m,e->data.odometry.y_m,e->data.odometry.yaw_rad,e->data.odometry.distance_m,e->data.odometry.uncertainty_m,e->data.odometry.quality,e->data.odometry.calibrated);break;
 case LAB_EVENT_ENCODER:fprintf(stderr,"SENSOR_SAMPLE_PAYLOAD type=ENCODER mask=0x%x cpr=%u counts=%lld,%lld,%lld,%lld\n",e->data.encoder.valid_mask,e->data.encoder.counts_per_revolution,(long long)e->data.encoder.counts[0],(long long)e->data.encoder.counts[1],(long long)e->data.encoder.counts[2],(long long)e->data.encoder.counts[3]);break;
 case LAB_EVENT_IMU:fprintf(stderr,"SENSOR_SAMPLE_PAYLOAD type=IMU yaw=%.17g rate=%.17g flags=0x%x\n",e->data.imu.yaw_rad,e->data.imu.yaw_rate_rad_s,e->data.imu.valid_flags);break;
 case LAB_EVENT_RESET:fprintf(stderr,"SENSOR_SAMPLE_PAYLOAD type=RESET reason=%u next_epoch=%u\n",e->data.reset.reason,e->data.reset.next_origin_epoch);break;
 case LAB_EVENT_GAP:fprintf(stderr,"SENSOR_SAMPLE_PAYLOAD type=GAP reason=%u count=%llu affected=%u\n",e->data.gap.reason,(unsigned long long)e->data.gap.dropped_events,e->data.gap.affected_event_type);break;
 default:break;
 }
 return status;
}
static lab_status accept(lab_real_runtime *r,const lab_live_sample *e,uint64_t now){
 r->sample_reject_reason[0]=0;
 if(!lab_live_sample_valid(e))return sample_reject(r,e,now,LAB_ERR_ARGUMENT,lab_real_sample_invalid_reason(e),NULL);
 if(!lab_real_context_equal(&r->view.context,&e->meta.context))return sample_reject(r,e,now,LAB_ERR_CONTEXT,"CONTEXT_MISMATCH",NULL);
 if(!(e->meta.flags&LAB_REAL_TIME_VALID))return sample_reject(r,e,now,LAB_ERR_TIME,"TIME_VALID_FLAG_MISSING",NULL);
 /* 显式无效输入不替换可信测量，也不刷新其年龄；持续无效输入仍按真实期限过期。
 */
 if((e->type==LAB_EVENT_POSE||e->type==LAB_EVENT_SCAN)&&!(e->meta.flags&LAB_REAL_DATA_VALID))return sample_reject(r,e,now,LAB_ERR_ARGUMENT,"MEASUREMENT_INVALID_DROP_ONE",NULL);
 if(e->type==LAB_EVENT_POSE&&(!(e->data.odometry.quality&LAB_POSE_VALID)||(e->data.odometry.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))))return sample_reject(r,e,now,LAB_ERR_ARGUMENT,"POSE_INVALID_DROP_ONE",NULL);
 if(e->meta.received_us>now)return sample_reject(r,e,now,LAB_ERR_TIME,"RECEIPT_AFTER_TICK",NULL);
 if(r->have_rx&&e->meta.received_us<r->received_us)return sample_reject(r,e,now,LAB_ERR_TIME,"RECEIPT_ORDER_ROLLBACK",NULL);
 if(r->parameter_version&&r->parameter_version!=e->meta.parameter_version)return sample_reject(r,e,now,LAB_ERR_CONTEXT,"PARAMETER_VERSION_CHANGED",NULL);
 r->parameter_version=e->meta.parameter_version;
 real_stream *slot=NULL;for(size_t i=0;i<16;++i){real_stream *p=&r->streams[i];if(p->used&&p->provider==e->meta.provider&&p->id==e->meta.stream_id){slot=p;break;}if(!slot&&!p->used)slot=p;}
 if(!slot)return sample_reject(r,e,now,LAB_ERR_LIMIT,"STREAM_CAPACITY_EXHAUSTED",NULL);
 if(slot->used){uint32_t d=e->meta.sequence-slot->sequence;if(slot->session!=e->meta.stream_session)return sample_reject(r,e,now,LAB_ERR_CONTEXT,"STREAM_SESSION_CHANGED",slot);if(!d)return sample_reject(r,e,now,LAB_ERR_TIME,"STREAM_SEQUENCE_DUPLICATE",slot);if(d>=UINT32_C(0x80000000))return sample_reject(r,e,now,LAB_ERR_TIME,"STREAM_SEQUENCE_ROLLBACK",slot);if(e->meta.time_us<slot->time_us)return sample_reject(r,e,now,LAB_ERR_TIME,"STREAM_SOURCE_TIME_ROLLBACK",slot);}
 *slot=(real_stream){1,e->meta.provider,e->meta.stream_id,e->meta.stream_session,e->meta.sequence,e->meta.time_us};r->received_us=e->meta.received_us;r->have_rx=1;
 if(e->type==LAB_EVENT_RESET&&!(e->data.reset.reason==LAB_RESET_START&&e->data.reset.next_origin_epoch==r->view.context.origin_epoch&&!r->view.accepted))return sample_reject(r,e,now,LAB_ERR_CONTEXT,"RESET_REQUIRES_NEW_CONTEXT",slot);
 if(e->type==LAB_EVENT_SCAN){*r->scan=*e;r->view.has_scan=(e->meta.flags&3)==3;r->view.scan_time_us=e->meta.time_us;r->scan_uncertainty=e->meta.time_uncertainty_us;}
 if(e->type==LAB_EVENT_POSE){r->view.raw_odometry=e->data.odometry;r->view.has_odometry=(e->meta.flags&3)==3&&(e->data.odometry.quality&LAB_POSE_VALID)&&!(e->data.odometry.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP));r->view.odometry_time_us=e->meta.time_us;r->odometry_uncertainty=e->meta.time_uncertainty_us;}
 if(e->type==LAB_EVENT_GAP){if(!e->data.gap.affected_event_type||e->data.gap.affected_event_type==LAB_EVENT_SCAN)r->view.has_scan=0;if(!e->data.gap.affected_event_type||e->data.gap.affected_event_type==LAB_EVENT_POSE)r->view.has_odometry=0;}
 /* 每个获准源样本在SLAM或地图匹配拒绝前，先交给独立近场观测器；该观测器不输出命令。
 */
 if(r->bindings.navigation.observe_sample)r->bindings.navigation.observe_sample(r->bindings.navigation.user,e,now);
 return LAB_OK;
}
static int navigable(const lab_localization_view*v){return v->state==LAB_REAL_LOC_TRACKING||(v->state==LAB_REAL_LOC_DEGRADED&&v->odometry_fallback==1);}
static int fresh(uint32_t have,uint64_t stamp,uint64_t now,uint64_t max){return have&&stamp<=now&&now-stamp<=max;}
static int revoke_check(lab_real_runtime*r,uint64_t now){
#ifdef LAB_REAL_POSIX_SOURCE
 const char*dir=getenv("ACG_PHONE_DIR");if(r->view.context.source==LAB_REAL_HARDWARE&&dir&&*dir){uint64_t generation=0;int rc=phone_control_fence_read(dir,&generation);
 if(rc<0||(rc==1&&r->seen_revoke)||generation<r->seen_revoke){r->view.manual_revoked=1;lab_real_runtime_cancel(r,now);return 0;}
 if(generation!=r->seen_revoke){r->seen_revoke=generation;r->view.manual_revoked=1;lab_real_runtime_cancel(r,now);return 0;}}
#else
 (void)r;(void)now;
#endif
 return 1;
}

static lab_status source_restart(lab_real_runtime*r,uint64_t now){
 lab_status s;
   const lab_live_sample*e=r->sample;
   if(!lab_live_sample_valid(e)||!lab_real_context_equal(&r->view.context,&e->meta.context)||e->meta.received_us>now||r->view.context.origin_epoch==UINT32_MAX||e->data.reset.next_origin_epoch!=r->view.context.origin_epoch+1)return finish(r,LAB_ERR_CONTEXT,"INVALID_SOURCE_RESET_EVENT",now);
   lab_real_context next=r->view.context;next.origin_epoch=e->data.reset.next_origin_epoch;
   unsigned object_mode=r->view.object_mode,manual_revoked=r->view.manual_revoked,requires_start=r->view.resume_requires_start;
   int rearm_intent=(r->view.enabled||r->view.pending_enable||r->view.object_soft_rearm_pending)&&!manual_revoked&&!requires_start;
   int retain=r->view.has_goal&&r->view.has_localization&&r->view.localization.has_pose&&!manual_revoked;
   lab_real_goal g=r->view.goal;lab_pose old=r->view.localization.pose;
   double dx=g.x_m-old.x_m,dy=g.y_m-old.y_m,cs=cos(old.yaw_rad),sn=sin(old.yaw_rad);
   if(r->observer&&(s=r->observer(r->observer_user,e))!=LAB_OK)return finish(r,s,"RECORDER_FAILED",now);
   s=lab_real_runtime_reset(r,&next);if(s!=LAB_OK)return finish(r,s,"SOURCE_CONTEXT_REBUILD_FAILED",now);
   if(object_mode){s=lab_real_runtime_object_mode(r,(int)object_mode,now);if(s!=LAB_OK)return finish(r,s,"SOURCE_OBJECT_MODE_RESTORE_FAILED",now);r->view.object_soft_rearm_pending=(unsigned)rearm_intent;}
   r->view.manual_revoked=manual_revoked;r->view.resume_requires_start=requires_start;
   if(retain){r->recovery_goal=g;r->recovery_goal_x=cs*dx+sn*dy;r->recovery_goal_y=-sn*dx+cs*dy;r->recovery_goal_yaw=remainder(g.yaw_rad-old.yaw_rad,6.2831853071795864769);r->recovery_goal_pending=1;}
   fprintf(stderr,"SOURCE_CONTEXT_RECOVERED epoch=%u retained_goal=%d object_mode=%u old_command_replayed=0 message_zh=数据源重启已自动重建，等待真实新数据后重新规划\n",next.origin_epoch,retain,object_mode);
   r->have_tick=1;r->view.time_us=now;return finish(r,LAB_AGAIN,"SOURCE_CONTEXT_REBUILT_WAIT_NEW_DATA",now);
  
}

/* 单个格式错误样本丢弃且不刷新源时钟，真实数据持续缺失时原时效规则仍停止输出。
 */
static int sample_soft_error(lab_status s){return s==LAB_ERR_ARGUMENT||s==LAB_ERR_FORMAT||s==LAB_ERR_CRC||s==LAB_ERR_TRUNCATED||s==LAB_ERR_VERSION||s==LAB_ERR_TIME||s==LAB_ERR_CONTEXT;}
#ifdef LAB_REAL_POSIX_SOURCE
static lab_status refresh_for_publication(lab_real_runtime*r,uint64_t now){
 for(unsigned i=0;i<32;i++){lab_status s=r->bindings.sensors.ops->poll(r->bindings.sensors.user,now,r->sample);if(s==LAB_AGAIN||s==LAB_EOF)break;if(sample_soft_error(s)){++r->view.rejected;continue;}if(s!=LAB_OK)return s;
  if(r->sample->type==LAB_EVENT_RESET&&r->sample->data.reset.reason==LAB_RESET_SOURCE_CHANGE)return source_restart(r,now);
  s=accept(r,r->sample,now);if(sample_soft_error(s)){++r->view.rejected;continue;}if(s!=LAB_OK)return s;++r->view.accepted;
  if(r->observer&&(s=r->observer(r->observer_user,r->sample))!=LAB_OK)return s;
  if(r->sample->type==LAB_EVENT_SCAN){if(r->deferred_count==4){memmove(r->deferred_scans,r->deferred_scans+1,3*sizeof(*r->sample));--r->deferred_count;}r->deferred_scans[r->deferred_count++]=*r->sample;}
  else if(r->sample->type==LAB_EVENT_POSE){
   if(r->bindings.localization.ops->push_control_pose)s=r->bindings.localization.ops->push_control_pose(r->bindings.localization.user,r->sample);else s=r->bindings.localization.ops->push(r->bindings.localization.user,r->sample);
   if(s<0&&s!=LAB_ERR_UNAVAILABLE)return s;r->last_forwarded_pose_us=r->sample->meta.time_us;r->have_forwarded_pose=1;
  }
 }
 lab_status display=r->bindings.localization.ops->snapshot(r->bindings.localization.user,now,&r->view.localization);return r->view.object_mode&&(display==LAB_AGAIN||display==LAB_ERR_UNAVAILABLE)?LAB_OK:display;
}
#endif
static int fresh_bound(uint32_t have,uint64_t stamp,uint64_t uncertainty,uint64_t now,uint64_t max){return fresh(have,stamp,now,max)&&uncertainty<=max-(now-stamp);}
/* 地图导航和实物局部意图都经过相同的控制权、反馈、ARM后命令、保护及500ms发布通路。
 */
static lab_status submit_command(lab_real_runtime*r,uint64_t now,uint64_t pose_time,int ordinary_wait){lab_status s=LAB_OK;
 if(r->start_deadline_us&&!r->first_apply_us){
  if(now>=r->start_deadline_us)return finish(r,LAB_ERR_TIME,"START_REQUEST_EXPIRED",now);
  if(r->view.request.expires_us>r->start_deadline_us)r->view.request.expires_us=r->start_deadline_us;
 }
 r->view.request.sequence=++r->command_sequence;
 if(!lab_real_context_equal(&r->view.context,&r->view.request.context)||!lab_real_command_valid(&r->view.request,now)||r->view.request.expires_us-r->view.request.generated_us>r->config.command_valid_us||r->view.request.linear_m_s<-.200000001||
 fabs(r->view.request.linear_m_s)>r->config.max_linear_m_s||fabs(r->view.request.angular_rad_s)>r->config.max_angular_rad_s)return finish(r,LAB_ERR_ARGUMENT,"INVALID_NAVIGATION_COMMAND",now);
 r->prepared_candidate=!ordinary_wait&&(fabs(r->view.request.linear_m_s)>1e-9||fabs(r->view.request.angular_rad_s)>1e-9);r->prepared_generation=r->goal_generation;
 if(r->config.mode==LAB_REAL_SHADOW){
  if(!r->bindings.chassis.ops)return finish(r,LAB_OK,"SHADOW_UNPROTECTED_NO_MOTOR_OUTPUT",now);
  lab_chassis_health h={r->view.context,now,r->view.scan_time_us,pose_time,now,1,1,1,0,0,0};
  r->view.request.control_session=r->config.control_session;
  s=r->bindings.chassis.ops->protect(r->bindings.chassis.user,&h,&r->view.request,&r->view.protected_output);
  if(s==LAB_OK&&(!lab_real_context_equal(&r->view.context,&r->view.protected_output.command.context)||!lab_real_command_valid(&r->view.protected_output.command,now)||r->view.protected_output.command.expires_us>r->view.request.expires_us))s=LAB_ERR_ARGUMENT;
  for(int i=0;i<4;++i)if(!isfinite(r->view.protected_output.wheel_rpm[i]))s=LAB_ERR_ARGUMENT;
  return finish(r,s,s==LAB_OK?"SHADOW_PROTECTED_NO_MOTOR_OUTPUT":"SHADOW_PROTECTION_STOP",now);
 }
 if(!r->view.enabled){
  if(!r->view.pending_enable){r->view.last_status=LAB_AGAIN;snprintf(r->view.reason,sizeof(r->view.reason),"HARDWARE_DISABLED");return LAB_AGAIN;}
  if(now<r->enable_started_us||now-r->enable_started_us>=CONT_HANDSHAKE_US)return finish(r,LAB_ERR_TIME,"ENABLE_HANDSHAKE_TIMEOUT",now);
  ++r->view.hardware_calls;s=r->bindings.chassis.ops->set_enabled(r->bindings.chassis.user,1,r->config.control_session,now);
  if(s==LAB_AGAIN){zero(r,now);r->view.last_status=LAB_AGAIN;snprintf(r->view.reason,sizeof(r->view.reason),"ENABLE_HANDSHAKE_PENDING");return LAB_AGAIN;}
  if(s!=LAB_OK)return finish(r,s,"ENABLE_HANDSHAKE_FAILED",now);r->view.enabled=1;r->view.pending_enable=0;r->view.object_soft_rearm_pending=0;
 }
 lab_chassis_feedback feedback={0};s=r->bindings.chassis.ops->feedback(r->bindings.chassis.user,now,&feedback);
 if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"FEEDBACK_CLOCK_ROLLBACK",now);
 if(s==LAB_OK)r->view.feedback=feedback;
 if(s!=LAB_OK||!lab_real_context_equal(&feedback.context,&r->view.context)||feedback.backend!=LAB_CHASSIS_HARDWARE||feedback.time_us>now||now-feedback.time_us>runtime_feedback_age(r)||feedback.faults){
  fprintf(stderr,"CHASSIS_FEEDBACK_FAILURE status=%d context_ok=%d backend=%u age_us=%llu faults=%u armed=%u capabilities=0x%x reason=%s now_us=%llu\n",(int)s,lab_real_context_equal(&feedback.context,&r->view.context),feedback.backend,(unsigned long long)(now>=feedback.time_us?now-feedback.time_us:UINT64_MAX),feedback.faults,feedback.armed,feedback.capabilities,feedback.reason,(unsigned long long)now);
  if(feedback.faults&15u)r->view.resume_requires_start=1;
  return finish(r,LAB_AGAIN,"INVALID_CHASSIS_FEEDBACK",now);
 }
 lab_chassis_health h={r->view.context,now,r->view.scan_time_us,pose_time,feedback.time_us,1,1,1,0,feedback.faults,0};
 r->view.request.control_session=r->config.control_session;r->view.request.enabled=1;
 s=r->bindings.chassis.ops->protect(r->bindings.chassis.user,&h,&r->view.request,&r->view.protected_output);
 if(s==LAB_AGAIN&&r->view.protected_output.fresh_command_required){
  /* 保留严格ARM后边界，维持控制权，丢弃旧候选，由下一次真实tick重新计算命令。
 */
  if(r->bindings.navigation.invalidate_command)r->bindings.navigation.invalidate_command(r->bindings.navigation.user);
  zero(r,now);r->view.protected_output.fresh_command_required=1;r->view.last_status=LAB_AGAIN;snprintf(r->view.reason,sizeof(r->view.reason),"POST_ARM_RECOMPUTE_PENDING");return LAB_AGAIN;
 }
 if(s!=LAB_OK){fprintf(stderr,"CHASSIS_PROTECTION_FAILURE status=%d reason=%s bits=0x%x armed=%u now_us=%llu\n",(int)s,r->view.protected_output.reason,r->view.protected_output.reasons,r->view.protected_output.armed,(unsigned long long)now);return finish(r,s,"PROTECTION_STOP",now);}
 if(r->view.protected_output.backend!=LAB_CHASSIS_HARDWARE||r->view.protected_output.armed!=1||r->view.protected_output.reasons||r->view.protected_output.command.enabled!=1){fprintf(stderr,"CHASSIS_PROTECTION_FAILURE status=DISABLED reason=%s bits=0x%x armed=%u backend=%u command_enabled=%u now_us=%llu\n",r->view.protected_output.reason,r->view.protected_output.reasons,r->view.protected_output.armed,r->view.protected_output.backend,r->view.protected_output.command.enabled,(unsigned long long)now);return finish(r,LAB_AGAIN,"PROTECTION_OUTPUT_DISABLED",now);}
 const lab_real_command *cmd=&r->view.protected_output.command;
 int context_ok=lab_real_context_equal(&cmd->context,&r->view.context),valid=lab_real_command_valid(cmd,now),lease_ok=cmd->lease!=0,session_ok=cmd->control_session==r->config.control_session,expiry_ok=cmd->expires_us<=r->view.request.expires_us;
 if(!context_ok||!valid||!lease_ok||!session_ok||!expiry_ok){fprintf(stderr,"PROTECTED_COMMAND_DIAG context_ok=%d valid=%d lease_ok=%d session_ok=%d expiry_ok=%d now_us=%llu generated_us=%llu expires_us=%llu request_expires_us=%llu enabled=%u linear=%.6f angular=%.6f\n",context_ok,valid,lease_ok,session_ok,expiry_ok,(unsigned long long)now,(unsigned long long)cmd->generated_us,(unsigned long long)cmd->expires_us,(unsigned long long)r->view.request.expires_us,cmd->enabled,cmd->linear_m_s,cmd->angular_rad_s);return finish(r,LAB_ERR_ARGUMENT,"INVALID_PROTECTED_COMMAND",now);}
 for(int i=0;i<4;++i)if(!isfinite(r->view.protected_output.wheel_rpm[i]))return finish(r,LAB_ERR_ARGUMENT,"INVALID_PROTECTED_RPM",now);
 if(r->view.request.linear_m_s==0&&r->view.request.angular_rad_s==0&&r->last_drive_motion){
  fprintf(stderr,"MOTION_ZERO_EVENT reason=NAVIGATION_REQUEST_ZERO goal_id=%u nav_state=%u nav_phase=%u nav_reason=%s blocking=%s fpga_reason=%s now_us=%llu\n",r->view.goal.id,r->view.navigation.state,r->view.navigation.phase,r->view.navigation.reason,r->view.navigation.blocking_reason,r->view.feedback.reason,(unsigned long long)now);
  const s3_nearfield_view*nf=&r->view.navigation.nearfield;
  fprintf(stderr,"MOTION_ZERO_GEOMETRY checked_us=%llu nf_code=%u nf_reason=%s source=%u source_us=%llu v=%.4f w=%.4f horizon=%.3f hit=%.3f,%.3f ttc=%.3f cells=%u/%u pending=%u unknown=%u scan_age_us=%llu\n",
   (unsigned long long)nf->query_time_us,nf->query_code,nf->query_reason,nf->query_source,(unsigned long long)nf->query_source_time_us,nf->query_v_m_s,nf->query_w_rad_s,nf->query_horizon_s,nf->query_x_m,nf->query_y_m,nf->query_ttc_s,nf->history_cells,nf->history_capacity,nf->history_pending,nf->history_unknown,(unsigned long long)nf->age_us);
 }
 if(!revoke_check(r,now))return finish(r,LAB_OK,"MANUAL_REVOKED_NAVIGATION",now);
 ++r->view.hardware_calls;s=r->bindings.chassis.ops->apply(r->bindings.chassis.user,&r->view.protected_output,now);
 if(s==LAB_OK){r->last_drive_motion=(r->view.request.linear_m_s!=0||r->view.request.angular_rad_s!=0);int moving_target=0;for(unsigned i=0;i<4;i++)if(fabs(r->view.protected_output.wheel_rpm[i])>=.01)moving_target=1;if(!r->first_apply_us&&moving_target){r->first_apply_us=now;r->start_deadline_us=0;}}
 else fprintf(stderr,"CHASSIS_APPLY_FAILURE status=%d goal_id=%u request_seq=%u request_expires_us=%llu protected_expires_us=%llu fpga_reason=%s now_us=%llu\n",(int)s,r->view.goal.id,r->view.request.sequence,(unsigned long long)r->view.request.expires_us,(unsigned long long)r->view.protected_output.command.expires_us,r->view.feedback.reason,(unsigned long long)now);
 return finish(r,s,s==LAB_OK?(ordinary_wait?"NAVIGATION_WAITING":"COMMAND_SUBMITTED_WAIT_ACK"):"CHASSIS_APPLY_FAILED",now);
}
static lab_status object_tick(lab_real_runtime*r,uint64_t now){
 if(!r->object_controller||!r->bindings.navigation.local_step||!r->bindings.navigation.local_publish)return finish(r,LAB_ERR_UNAVAILABLE,"OBJECT_LOCAL_MODULE_UNAVAILABLE",now);
 /* 本模式允许SLAM暂时不可用，其快照仅用于显示。 */
 if(r->bindings.localization.ops){lab_status display=r->bindings.localization.ops->snapshot(r->bindings.localization.user,now,&r->view.localization);r->view.has_localization=display==LAB_OK&&r->view.localization.has_pose;}
 if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"OBJECT_CLOCK_ROLLBACK",now);
 if(!fresh_bound(r->view.has_scan,r->view.scan_time_us,r->scan_uncertainty,now,runtime_scan_age(r))||!fresh_bound(r->view.has_odometry,r->view.odometry_time_us,r->odometry_uncertainty,now,runtime_pose_age(r)))return finish(r,LAB_AGAIN,"OBJECT_WAIT_FRESH_RAW_SCAN_ODOMETRY",now);
 lab_real_command desired={0};int active=r->object_controller(r->object_user,&r->view,now,&desired);
 if(!active){halt(r,now,"OBJECT_TASK_INACTIVE");return finish(r,LAB_OK,"OBJECT_TASK_INACTIVE",now);}
 if(!lab_real_context_equal(&desired.context,&r->view.context)||!lab_real_command_valid(&desired,now)||desired.expires_us-desired.generated_us>r->config.command_valid_us||fabs(desired.linear_m_s)>.250000001||fabs(desired.angular_rad_s)>r->config.max_angular_rad_s)return finish(r,LAB_ERR_ARGUMENT,"OBJECT_INTENT_INVALID_CONTEXT_OR_LIMIT",now);
 if(r->bindings.navigation.chassis_hint){lab_chassis_feedback hint=r->view.feedback;hint.armed=hint.armed&&r->view.enabled;r->bindings.navigation.chassis_hint(r->bindings.navigation.user,&hint);}
 /* 控制器的保持状态优先，缓存路线不能把近障停车、丢方向保持或到达复核改回运动。
 */
 if(desired.linear_m_s==0&&desired.angular_rad_s==0){
  obj_turn_observe(&r->object_turn.motion,r->view.odometry_time_us,r->view.raw_odometry.yaw_rad);obj_turn_shape(&r->object_turn,now,0,1);
  r->view.object_turn_waiting=0;r->view.object_turn_selected_w=r->view.object_turn_applied_w=0;snprintf(r->view.object_turn_reason,sizeof r->view.object_turn_reason,"STOP_PRIORITY");
  r->view.request=desired;
  snprintf(r->view.navigation.reason,sizeof(r->view.navigation.reason),"OBJECT_CONTROLLER_HOLD_PRIORITY");
  return submit_command(r,now,r->view.odometry_time_us,0);
 }
 double cap=desired.linear_m_s>0?fmin(.25,desired.linear_m_s):.25;
 lab_status result=r->bindings.navigation.local_step(r->bindings.navigation.user,now,&r->view.raw_odometry,r->view.odometry_time_us,r->scan,&desired,cap,&r->view.request);
 if(r->bindings.navigation.view)r->bindings.navigation.view(r->bindings.navigation.user,&r->view.navigation);
 if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"OBJECT_CLOCK_ROLLBACK",now);
#ifdef LAB_REAL_POSIX_SOURCE
 if(result==LAB_OK){uint32_t epoch_before=r->view.context.origin_epoch;lab_status refreshed=refresh_for_publication(r,now);if(epoch_before!=r->view.context.origin_epoch)return finish(r,LAB_AGAIN,"SOURCE_CONTEXT_REBUILT_WAIT_NEW_DATA",now);if(refreshed<0)return finish(r,refreshed,"OBJECT_PUBLICATION_REFRESH_FAILED",now);}
 if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"OBJECT_CLOCK_ROLLBACK",now);
#endif
 if(!fresh_bound(r->view.has_scan,r->view.scan_time_us,r->scan_uncertainty,now,runtime_scan_age(r))||!fresh_bound(r->view.has_odometry,r->view.odometry_time_us,r->odometry_uncertainty,now,runtime_pose_age(r)))return finish(r,LAB_AGAIN,"OBJECT_PUBLICATION_RAW_SOURCE_STALE",now);
 if(result==LAB_OK){
  double selected=r->view.request.angular_rad_s;
  double cap=fabs(desired.linear_m_s)<1e-9?fmin(OBJ_ALIGN_YAW_CAP,fabs(desired.angular_rad_s)):OBJ_ALIGN_YAW_CAP;
  double target=obj_turn_clip(selected,-cap,cap);
  /* 规划等待输出零速请求；该请求可用于停车确认。 */
  int hold=r->view.request.linear_m_s==0&&selected==0;
  obj_turn_observe(&r->object_turn.motion,r->view.odometry_time_us,r->view.raw_odometry.yaw_rad);
  r->view.request.angular_rad_s=obj_turn_shape(&r->object_turn,now,target,hold);
  r->view.object_turn_waiting=r->object_turn.waiting;
  r->view.object_turn_selected_w=selected;r->view.object_turn_applied_w=r->view.request.angular_rad_s;
  snprintf(r->view.object_turn_reason,sizeof r->view.object_turn_reason,"%s",r->object_turn.reason);
  if(!r->object_turn.log_us||now-r->object_turn.log_us>=250000){
   fprintf(stderr,"OBJECT_TURN_SMOOTH now_us=%llu desired_w=%.5f route_w=%.5f capped_w=%.5f output_w=%.5f measured_valid=%d measured_w=%.5f reversal_wait=%d reason=%s fpga_start_assist=UNCHANGED message_zh=仅RK平滑转向_原FPGA起步助力仍存在\n",(unsigned long long)now,desired.angular_rad_s,selected,target,r->view.request.angular_rad_s,obj_turn_measured(&r->object_turn.motion,now),r->object_turn.motion.rate,r->object_turn.waiting,r->object_turn.reason);r->object_turn.log_us=now;
  }
  /* 已有local_publish核对实际整形后的命令及所有时钟。 */
  result=r->bindings.navigation.local_publish(r->bindings.navigation.user,now,&r->view.raw_odometry,r->view.odometry_time_us,&r->view.request);
 }
 if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"OBJECT_CLOCK_ROLLBACK",now);
 int waiting=result==LAB_AGAIN;if(result<0)return finish(r,result,"OBJECT_R8_LOCAL_ACTION_FAILURE",now);
 if(waiting){obj_turn_shape(&r->object_turn,now,0,1);r->view.object_turn_selected_w=r->view.object_turn_applied_w=0;r->view.object_turn_waiting=0;snprintf(r->view.object_turn_reason,sizeof r->view.object_turn_reason,"PLANNER_WAIT_ZERO_PRIORITY");r->view.request=(lab_real_command){.context=r->view.context,.generated_us=now,.expires_us=now+r->config.command_valid_us};}
 return submit_command(r,now,r->view.odometry_time_us,waiting);
}
lab_status lab_real_runtime_tick(lab_real_runtime *r,uint64_t now){
 if(!r)return LAB_ERR_ARGUMENT;
 if(r->stop_fault)return finish(r,LAB_ERR_IO,"STOP_TRANSPORT_FAILURE",now);
 if(r->context_fault)return finish(r,LAB_ERR_CONTEXT,"CONTEXT_CHANGE_RESET_REQUIRED",now);
 if(r->clock_fault||(r->have_tick&&now<=r->view.time_us)){r->clock_fault=1;return finish(r,LAB_ERR_TIME,"CLOCK_FAULT_RESET_REQUIRED",now);}
 if(!revoke_check(r,now))return finish(r,LAB_OK,"MANUAL_REVOKED_NAVIGATION",now);
 r->have_tick=1;r->view.time_us=now;++r->view.ticks;zero(r,now);
 if(r->view.missing_modules)return finish(r,LAB_ERR_UNAVAILABLE,"WAITING_FOR_MODULES",now);
 /* 尚无目标或ARM时也接收只读状态，不发送CLAIM、PROBE或DRIVE，不使能硬件；真实反馈与假设预览输出分开。

 */
 if(r->config.allow_hardware_output&&r->bindings.chassis.ops){
  lab_chassis_feedback idle_feedback={0};lab_status fs=r->bindings.chassis.ops->feedback(r->bindings.chassis.user,now,&idle_feedback);
  if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"FEEDBACK_CLOCK_ROLLBACK",now);
  if(fs==LAB_OK){r->view.feedback=idle_feedback;if(idle_feedback.faults&15u)r->view.resume_requires_start=1;}
  else if(idle_feedback.reason[0])snprintf(r->view.feedback.reason,sizeof(r->view.feedback.reason),"%s",idle_feedback.reason);
 }
 for(unsigned i=0;i<r->deferred_count;i++){lab_status ds=r->bindings.localization.ops->push(r->bindings.localization.user,&r->deferred_scans[i]);if(ds<0&&ds!=LAB_ERR_UNAVAILABLE){r->deferred_count=0;return finish(r,ds,"DEFERRED_SCAN_REJECTED",now);}}r->deferred_count=0;
 for(unsigned i=0;i<64;++i){lab_status s=r->bindings.sensors.ops->poll(r->bindings.sensors.user,now,r->sample);if(s==LAB_AGAIN||s==LAB_EOF)break;if(sample_soft_error(s)){++r->view.rejected;continue;}if(s!=LAB_OK){r->bindings.sensors.ops->health(r->bindings.sensors.user,now,&r->view.sensor_health);return finish(r,s,"SENSOR_POLL_FAILED",now);}
  /* 数据源重启使坐标失效，但保留任务意图。停止旧输出，重建消费者并等待新的真实测量。
 */
  if(r->sample->type==LAB_EVENT_RESET&&r->sample->data.reset.reason==LAB_RESET_SOURCE_CHANGE)return source_restart(r,now);
  s=accept(r,r->sample,now);if(sample_soft_error(s)){++r->view.rejected;continue;}if(s!=LAB_OK){if(s==LAB_ERR_CONTEXT)r->context_fault=1;++r->view.rejected;return finish(r,s,r->sample_reject_reason[0]?r->sample_reject_reason:"SENSOR_CONTEXT_OR_TIME_REJECTED",now);}++r->view.accepted;
   if(r->observer&&(s=r->observer(r->observer_user,r->sample))!=LAB_OK)return finish(r,s,"RECORDER_FAILED",now);
   if(r->bindings.localization.ops){
    int alias=r->sample->type==LAB_EVENT_POSE&&r->have_forwarded_pose&&r->sample->meta.time_us==r->last_forwarded_pose_us;
    if(alias){++r->view.coalesced_pose_aliases;continue;}
    s=r->bindings.localization.ops->push(r->bindings.localization.user,r->sample);
    if(r->sample->type==LAB_EVENT_POSE&&s!=LAB_ERR_CONTEXT&&s!=LAB_ERR_VERSION&&s!=LAB_ERR_ARGUMENT&&s!=LAB_ERR_TIME){r->last_forwarded_pose_us=r->sample->meta.time_us;r->have_forwarded_pose=1;}
    if(s==LAB_ERR_CONTEXT)r->context_fault=1;
    if(s==LAB_AGAIN||s==LAB_ERR_UNAVAILABLE){++r->view.localizer_waits;continue;}
    if(s!=LAB_OK){fprintf(stderr,"LOCALIZER_PUSH_DETAIL status=%d event_type=%u sequence=%u time_us=%llu received_us=%llu now_us=%llu\n",(int)s,(unsigned)r->sample->type,r->sample->meta.sequence,(unsigned long long)r->sample->meta.time_us,(unsigned long long)r->sample->meta.received_us,(unsigned long long)now);return finish(r,s,"LOCALIZER_PUSH_FAILED",now);}}}
  lab_status hs=r->bindings.sensors.ops->health(r->bindings.sensors.user,now,&r->view.sensor_health);
  if(hs<0)return finish(r,hs,"SOURCE_HEALTH_FAILED",now);
  r->view.scan_uncertainty_us=r->scan_uncertainty;r->view.odometry_uncertainty_us=r->odometry_uncertainty;
  r->view.latest_scan=r->view.has_scan?&r->scan->data.scan:NULL;
 if(r->config.mode==LAB_REAL_READONLY)return finish(r,r->view.accepted?LAB_OK:LAB_AGAIN,r->view.accepted?"READONLY_NO_MOTOR_OUTPUT":"WAITING_FOR_REAL_DATA",now);
 if(r->view.object_mode)return object_tick(r,now);
 lab_status s=r->bindings.localization.ops->snapshot(r->bindings.localization.user,now,&r->view.localization);
 if(s!=LAB_OK)return finish(r,s,"WAITING_FOR_LOCALIZATION",now);
 const lab_localization_view *v=&r->view.localization;r->view.has_localization=v->has_pose;
  if(!v->has_pose)return finish(r,LAB_AGAIN,"WAITING_FOR_LOCALIZATION",now);
 if(!lab_real_context_equal(&r->view.context,&v->context)||!lab_real_context_equal(&r->view.context,&v->map.context)||!lab_real_grid_valid(&v->map)||
 v->has_pose>1||v->state>LAB_REAL_LOC_LOST||v->parameter_version!=r->parameter_version||!isfinite(v->pose.x_m)||!isfinite(v->pose.y_m)||
 !isfinite(v->pose.yaw_rad)||!isfinite(v->pose.distance_m)||v->pose.distance_m<0||!isfinite(v->pose.uncertainty_m)||v->pose.uncertainty_m<0||v->pose.calibrated>1||v->odometry_fallback>1||v->reserved)return finish(r,LAB_ERR_CONTEXT,"INVALID_LOCALIZATION_SNAPSHOT",now);
 if(r->recovery_goal_pending&&navigable(v)&&r->view.has_odometry&&fresh(1,r->view.odometry_time_us,now,r->config.max_pose_age_us)){
  lab_real_goal g=r->recovery_goal;double cs=cos(v->pose.yaw_rad),sn=sin(v->pose.yaw_rad);
  g.map_id=v->map.map_id;g.x_m=v->pose.x_m+cs*r->recovery_goal_x-sn*r->recovery_goal_y;g.y_m=v->pose.y_m+sn*r->recovery_goal_x+cs*r->recovery_goal_y;g.yaw_rad=remainder(v->pose.yaw_rad+r->recovery_goal_yaw,6.2831853071795864769);
  /* 这是保留的相对任务估计，不是实测全局位姿；不复用旧路线或命令，取消操作清除待处理目标。
 */
  lab_status gs=lab_real_runtime_set_goal(r,&g,now);if(gs==LAB_OK){r->recovery_goal_pending=0;fprintf(stderr,"GOAL_CONTEXT_REPROJECTED estimated=1 goal_id=%u map_id=%llu x=%.6f y=%.6f message_zh=已按重启前相对目标重建任务，重新规划路线\n",g.id,(unsigned long long)g.map_id,g.x_m,g.y_m);}
 }
 if(r->view.has_goal&&v->map.map_id!=r->view.goal.map_id){halt(r,now,"MAP_REBUILD_WAIT_REPLAN");r->view.goal.map_id=v->map.map_id;lab_status gs=r->bindings.navigation.goal(r->bindings.navigation.user,&r->view.goal,now);return finish(r,gs==LAB_OK?LAB_AGAIN:gs,"MAP_ID_CHANGED_REPLANNING",now);}
 if(v->map.map_id==r->map_id&&(v->map.map_version<r->map_version||v->map.correction_version<r->correction_version))return finish(r,LAB_ERR_CONTEXT,"MAP_VERSION_REGRESSION",now);
 r->map_id=v->map.map_id;r->map_version=v->map.map_version;r->correction_version=v->map.correction_version;
#ifdef LAB_REAL_POSIX_SOURCE
 /* 先扫描匹配和复制快照，再导航；驾驶时效检查与命令生成使用实际单调时刻，避免将前置耗时误记为规划慢。

 */
 if(r->config.allow_hardware_output&&r->view.context.source==LAB_REAL_HARDWARE){
  struct timespec wall;if(clock_gettime(CLOCK_MONOTONIC,&wall)!=0)return finish(r,LAB_ERR_IO,"COMMAND_CLOCK_READ_FAILED",now);
  uint64_t actual=(uint64_t)wall.tv_sec*1000000u+(uint64_t)wall.tv_nsec/1000u;
  if(actual<now)return finish(r,LAB_ERR_TIME,"COMMAND_CLOCK_ROLLBACK",now);
  if(actual-now>CONT_CONTROL_US)fprintf(stderr,"CONTROL_TICK_REBASE preprocessing_us=%llu old_us=%llu now_us=%llu\n",(unsigned long long)(actual-now),(unsigned long long)now,(unsigned long long)actual);
  now=actual;r->view.time_us=now;
 }
#endif
 if(r->config.mode<LAB_REAL_SHADOW)return finish(r,LAB_OK,"ESTIMATION_ONLY_NO_MOTOR_OUTPUT",now);
 int scan_ok=fresh_bound(r->view.has_scan,r->view.scan_time_us,r->scan_uncertainty,now,r->config.max_scan_age_us);
 int odom_ok=fresh_bound(r->view.has_odometry,r->view.odometry_time_us,r->odometry_uncertainty,now,r->config.max_pose_age_us);
 int pose_ok=fresh(v->has_pose,v->pose_time_us,now,r->config.max_pose_age_us);
 int match_ok=(v->odometry_fallback==1&&v->state==LAB_REAL_LOC_DEGRADED)||(v->last_matched_scan_us<=now&&now-v->last_matched_scan_us<=CONT_MATCH_US);
 int quality_ok=navigable(v)&&(v->pose.quality&LAB_POSE_VALID)&&!(v->pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP));
 if(!scan_ok||!odom_ok||!pose_ok||!match_ok||!quality_ok){
  if(r->view.enabled||r->view.pending_enable||!r->last_stale_log_us||now-r->last_stale_log_us>=1000000){r->last_stale_log_us=now;fprintf(stderr,"LOCALIZATION_GATE_DIAG scan_ok=%d odom_ok=%d pose_ok=%d match_ok=%d quality_ok=%d scan_age_us=%llu scan_unc_us=%llu odom_age_us=%llu odom_unc_us=%llu pose_age_us=%llu match_age_us=%llu scan_limit_us=%llu pose_limit_us=%llu match_limit_us=%llu has_scan=%u has_odom=%u has_pose=%u state=%u quality=0x%x odom_quality=0x%x\n",
   scan_ok,odom_ok,pose_ok,match_ok,quality_ok,
   (unsigned long long)(now>=r->view.scan_time_us?now-r->view.scan_time_us:UINT64_MAX),(unsigned long long)r->scan_uncertainty,
   (unsigned long long)(now>=r->view.odometry_time_us?now-r->view.odometry_time_us:UINT64_MAX),(unsigned long long)r->odometry_uncertainty,
   (unsigned long long)(now>=v->pose_time_us?now-v->pose_time_us:UINT64_MAX),
   (unsigned long long)(now>=v->last_matched_scan_us?now-v->last_matched_scan_us:UINT64_MAX),
   (unsigned long long)r->config.max_scan_age_us,(unsigned long long)r->config.max_pose_age_us,(unsigned long long)CONT_MATCH_US,
   r->view.has_scan,r->view.has_odometry,v->has_pose,v->state,v->pose.quality,r->view.raw_odometry.quality);}
  return finish(r,LAB_AGAIN,"STALE_OR_INVALID_LOCALIZATION",now);
 }
 if(!r->view.has_goal)return finish(r,LAB_AGAIN,"NO_GOAL",now);
 if(r->bindings.navigation.chassis_hint&&r->bindings.chassis.ops&&r->config.allow_hardware_output){
  lab_chassis_feedback hint={0};lab_status hint_status=r->bindings.chassis.ops->feedback(r->bindings.chassis.user,now,&hint);
  if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"FEEDBACK_CLOCK_ROLLBACK",now);
  if(hint_status==LAB_OK)r->view.feedback=hint;
  hint.armed=hint.armed&&r->view.enabled; /* 执行许可归本运行对象所有，不由其他已使能端点授予。 */
  r->bindings.navigation.chassis_hint(r->bindings.navigation.user,&hint);
 }
 uint64_t generation=r->goal_generation;
#ifdef LAB_REAL_POSIX_SOURCE
 uint64_t computation_started=now;
#endif
 s=r->bindings.navigation.step(r->bindings.navigation.user,now,v,r->scan,&r->view.request);
 if(generation!=r->goal_generation)return finish(r,LAB_OK,"CONTROL_GENERATION_REVOKED",now);
 if(r->bindings.navigation.view)r->bindings.navigation.view(r->bindings.navigation.user,&r->view.navigation);
#ifdef LAB_REAL_POSIX_SOURCE
 /* 按实际计算耗时调整负载。 */
 if(r->config.allow_hardware_output&&r->view.context.source==LAB_REAL_HARDWARE){
  struct timespec wall;if(clock_gettime(CLOCK_MONOTONIC,&wall)!=0)return finish(r,LAB_ERR_IO,"COMMAND_CLOCK_READ_FAILED",now);
  uint64_t actual=(uint64_t)wall.tv_sec*1000000u+(uint64_t)wall.tv_nsec/1000u;if(actual<now)return finish(r,LAB_ERR_TIME,"COMMAND_CLOCK_ROLLBACK",now);
  now=actual;r->view.time_us=now;if(!revoke_check(r,now))return finish(r,LAB_OK,"MANUAL_REVOKED_NAVIGATION",now);
  if(s==LAB_OK&&r->bindings.navigation.publish){uint32_t epoch_before=r->view.context.origin_epoch;lab_status rs=refresh_for_publication(r,now);if(epoch_before!=r->view.context.origin_epoch)return finish(r,LAB_AGAIN,"SOURCE_CONTEXT_REBUILT_WAIT_NEW_DATA",now);if(rs<0)return finish(r,rs,"PUBLICATION_REFRESH_FAILED",now);
   v=&r->view.localization;r->view.has_localization=v->has_pose;r->view.scan_uncertainty_us=r->scan_uncertainty;r->view.odometry_uncertainty_us=r->odometry_uncertainty;

   lab_chassis_feedback updated={0};lab_status updated_status=r->bindings.chassis.ops->feedback(r->bindings.chassis.user,now,&updated);if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"FEEDBACK_CLOCK_ROLLBACK",now);if(updated_status==LAB_OK){r->view.feedback=updated;if(r->bindings.navigation.chassis_hint){updated.armed=updated.armed&&r->view.enabled;r->bindings.navigation.chassis_hint(r->bindings.navigation.user,&updated);}}
   int inputs=fresh_bound(r->view.has_scan,r->view.scan_time_us,r->scan_uncertainty,now,r->config.max_scan_age_us)&&fresh_bound(r->view.has_odometry,r->view.odometry_time_us,r->odometry_uncertainty,now,r->config.max_pose_age_us)&&v->has_pose&&navigable(v)&&fresh(1,v->pose_time_us,now,r->config.max_pose_age_us);
   if(inputs)rs=r->bindings.navigation.publish(r->bindings.navigation.user,now,v,&r->view.request);else rs=LAB_AGAIN;
   if(rs<0)return finish(r,rs,"PUBLICATION_REVALIDATION_FAILED",now);
   if(rs==LAB_AGAIN)r->view.request=(lab_real_command){.context=r->view.context,.generated_us=now,.expires_us=now+r->config.command_valid_us};
  }
  if(clock_gettime(CLOCK_MONOTONIC,&wall)!=0)return finish(r,LAB_ERR_IO,"COMMAND_CLOCK_READ_FAILED",now);actual=(uint64_t)wall.tv_sec*1000000u+(uint64_t)wall.tv_nsec/1000u;if(actual<now)return finish(r,LAB_ERR_TIME,"COMMAND_CLOCK_ROLLBACK",now);now=actual;r->view.time_us=now;
  uint64_t elapsed=actual-computation_started;lab_real_runtime_note_cost(r,elapsed);
  if(elapsed>CONT_CONTROL_WARN_US)fprintf(stderr,"CONTROL_COST warn=1 elapsed_us=%llu soft_streak=%u load_reduced=%u owner_retained=1 now_us=%llu\n",(unsigned long long)elapsed,r->budget.streak,r->budget.reduced,(unsigned long long)actual);

 }

#endif
 if(s==LAB_EOF){halt(r,now,"GOAL_REACHED");r->view.has_goal=0;return finish(r,LAB_OK,"GOAL_REACHED",now);}
 int ordinary_wait=s==LAB_AGAIN;
 if(ordinary_wait){
  /* AGAIN表示导航接口的正常等待，保留控制权与目标并发送有效零命令，原因文字仅供诊断。
 */
  r->view.request=(lab_real_command){.context=r->view.context,.generated_us=now,.expires_us=now+r->config.command_valid_us};
  if(r->last_drive_motion){snprintf(r->view.last_stop_reason,sizeof(r->view.last_stop_reason),"NAVIGATION_WAIT_OR_FAILURE");snprintf(r->view.last_stop_detail,sizeof(r->view.last_stop_detail),"%s",r->view.navigation.reason);r->view.last_stop_us=now;fprintf(stderr,"SOFT_WAIT_EVENT reason=%s session_retained=1 now_us=%llu\n",r->view.navigation.reason,(unsigned long long)now);}
  s=LAB_OK;
 }
 if(s!=LAB_OK){
  fprintf(stderr,"NAVIGATION_WAIT_DIAG status=%d nav_state=%u nav_phase=%u nav_reason=%s blocking=%s path_points=%u pose_state=%u pose_reason=%s scan_age_us=%llu odom_age_us=%llu goal_id=%u map_version=%llu enabled=%u pending=%u\n",
   (int)s,r->view.navigation.state,r->view.navigation.phase,r->view.navigation.reason,r->view.navigation.blocking_reason,r->view.navigation.path_count,
   v->state,v->reason,(unsigned long long)(now>=r->view.scan_time_us?now-r->view.scan_time_us:UINT64_MAX),
   (unsigned long long)(now>=r->view.odometry_time_us?now-r->view.odometry_time_us:UINT64_MAX),r->view.goal.id,
   (unsigned long long)v->map.map_version,r->view.enabled,r->view.pending_enable);
  return finish(r,s,"NAVIGATION_WAIT_OR_FAILURE",now);
 }
 if(r->start_deadline_us&&!r->first_apply_us){
  if(now>=r->start_deadline_us)return finish(r,LAB_ERR_TIME,"START_REQUEST_EXPIRED",now);
  if(r->view.request.expires_us>r->start_deadline_us)r->view.request.expires_us=r->start_deadline_us;
 }
 r->view.request.sequence=++r->command_sequence;
 if(!lab_real_context_equal(&r->view.context,&r->view.request.context)||!lab_real_command_valid(&r->view.request,now)||r->view.request.expires_us-r->view.request.generated_us>r->config.command_valid_us||r->view.request.linear_m_s<-.200000001||
 fabs(r->view.request.linear_m_s)>r->config.max_linear_m_s||fabs(r->view.request.angular_rad_s)>r->config.max_angular_rad_s)return finish(r,LAB_ERR_ARGUMENT,"INVALID_NAVIGATION_COMMAND",now);
 r->prepared_candidate=!ordinary_wait&&(fabs(r->view.request.linear_m_s)>1e-9||fabs(r->view.request.angular_rad_s)>1e-9);r->prepared_generation=r->goal_generation;
 if(r->config.mode==LAB_REAL_SHADOW){
  if(!r->bindings.chassis.ops)return finish(r,LAB_OK,"SHADOW_UNPROTECTED_NO_MOTOR_OUTPUT",now);
  lab_chassis_health h={r->view.context,now,r->view.scan_time_us,v->pose_time_us,now,1,1,1,0,0,0};
  r->view.request.control_session=r->config.control_session;
  s=r->bindings.chassis.ops->protect(r->bindings.chassis.user,&h,&r->view.request,&r->view.protected_output);
  if(s==LAB_OK&&(!lab_real_context_equal(&r->view.context,&r->view.protected_output.command.context)||!lab_real_command_valid(&r->view.protected_output.command,now)||r->view.protected_output.command.expires_us>r->view.request.expires_us))s=LAB_ERR_ARGUMENT;
  for(int i=0;i<4;++i)if(!isfinite(r->view.protected_output.wheel_rpm[i]))s=LAB_ERR_ARGUMENT;
  return finish(r,s,s==LAB_OK?"SHADOW_PROTECTED_NO_MOTOR_OUTPUT":"SHADOW_PROTECTION_STOP",now);
 }
 if(!r->view.enabled){
  if(!r->view.pending_enable){r->view.last_status=LAB_AGAIN;snprintf(r->view.reason,sizeof(r->view.reason),"HARDWARE_DISABLED");return LAB_AGAIN;}
  if(now<r->enable_started_us||now-r->enable_started_us>=CONT_HANDSHAKE_US)return finish(r,LAB_ERR_TIME,"ENABLE_HANDSHAKE_TIMEOUT",now);
  ++r->view.hardware_calls;s=r->bindings.chassis.ops->set_enabled(r->bindings.chassis.user,1,r->config.control_session,now);
  if(s==LAB_AGAIN){zero(r,now);r->view.last_status=LAB_AGAIN;snprintf(r->view.reason,sizeof(r->view.reason),"ENABLE_HANDSHAKE_PENDING");return LAB_AGAIN;}
  if(s!=LAB_OK)return finish(r,s,"ENABLE_HANDSHAKE_FAILED",now);r->view.enabled=1;r->view.pending_enable=0;
 }
 lab_chassis_feedback feedback={0};s=r->bindings.chassis.ops->feedback(r->bindings.chassis.user,now,&feedback);
 if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"FEEDBACK_CLOCK_ROLLBACK",now);
 if(s==LAB_OK)r->view.feedback=feedback;
 if(s!=LAB_OK||!lab_real_context_equal(&feedback.context,&r->view.context)||feedback.backend!=LAB_CHASSIS_HARDWARE||feedback.time_us>now||now-feedback.time_us>CONT_FEEDBACK_US||feedback.faults){
  fprintf(stderr,"CHASSIS_FEEDBACK_FAILURE status=%d context_ok=%d backend=%u age_us=%llu faults=%u armed=%u capabilities=0x%x reason=%s now_us=%llu\n",(int)s,lab_real_context_equal(&feedback.context,&r->view.context),feedback.backend,(unsigned long long)(now>=feedback.time_us?now-feedback.time_us:UINT64_MAX),feedback.faults,feedback.armed,feedback.capabilities,feedback.reason,(unsigned long long)now);
  if(feedback.faults&15u)r->view.resume_requires_start=1;
  return finish(r,LAB_AGAIN,"INVALID_CHASSIS_FEEDBACK",now);
 }
 lab_chassis_health h={r->view.context,now,r->view.scan_time_us,v->pose_time_us,feedback.time_us,1,1,1,0,feedback.faults,0};
 r->view.request.control_session=r->config.control_session;r->view.request.enabled=1;
 s=r->bindings.chassis.ops->protect(r->bindings.chassis.user,&h,&r->view.request,&r->view.protected_output);
 if(s==LAB_AGAIN&&r->view.protected_output.fresh_command_required){
  /* 保留严格ARM后边界，维持控制权，丢弃旧候选，由下一次真实tick重新计算命令。
 */
  if(r->bindings.navigation.invalidate_command)r->bindings.navigation.invalidate_command(r->bindings.navigation.user);
  zero(r,now);r->view.protected_output.fresh_command_required=1;r->view.last_status=LAB_AGAIN;snprintf(r->view.reason,sizeof(r->view.reason),"POST_ARM_RECOMPUTE_PENDING");return LAB_AGAIN;
 }
 if(s!=LAB_OK){fprintf(stderr,"CHASSIS_PROTECTION_FAILURE status=%d reason=%s bits=0x%x armed=%u now_us=%llu\n",(int)s,r->view.protected_output.reason,r->view.protected_output.reasons,r->view.protected_output.armed,(unsigned long long)now);return finish(r,s,"PROTECTION_STOP",now);}
 if(r->view.protected_output.backend!=LAB_CHASSIS_HARDWARE||r->view.protected_output.armed!=1||r->view.protected_output.reasons||r->view.protected_output.command.enabled!=1){fprintf(stderr,"CHASSIS_PROTECTION_FAILURE status=DISABLED reason=%s bits=0x%x armed=%u backend=%u command_enabled=%u now_us=%llu\n",r->view.protected_output.reason,r->view.protected_output.reasons,r->view.protected_output.armed,r->view.protected_output.backend,r->view.protected_output.command.enabled,(unsigned long long)now);return finish(r,LAB_AGAIN,"PROTECTION_OUTPUT_DISABLED",now);}
 const lab_real_command *cmd=&r->view.protected_output.command;
 int context_ok=lab_real_context_equal(&cmd->context,&r->view.context),valid=lab_real_command_valid(cmd,now),lease_ok=cmd->lease!=0,session_ok=cmd->control_session==r->config.control_session,expiry_ok=cmd->expires_us<=r->view.request.expires_us;
 if(!context_ok||!valid||!lease_ok||!session_ok||!expiry_ok){fprintf(stderr,"PROTECTED_COMMAND_DIAG context_ok=%d valid=%d lease_ok=%d session_ok=%d expiry_ok=%d now_us=%llu generated_us=%llu expires_us=%llu request_expires_us=%llu enabled=%u linear=%.6f angular=%.6f\n",context_ok,valid,lease_ok,session_ok,expiry_ok,(unsigned long long)now,(unsigned long long)cmd->generated_us,(unsigned long long)cmd->expires_us,(unsigned long long)r->view.request.expires_us,cmd->enabled,cmd->linear_m_s,cmd->angular_rad_s);return finish(r,LAB_ERR_ARGUMENT,"INVALID_PROTECTED_COMMAND",now);}
 for(int i=0;i<4;++i)if(!isfinite(r->view.protected_output.wheel_rpm[i]))return finish(r,LAB_ERR_ARGUMENT,"INVALID_PROTECTED_RPM",now);
 if(r->view.request.linear_m_s==0&&r->view.request.angular_rad_s==0&&r->last_drive_motion){
  fprintf(stderr,"MOTION_ZERO_EVENT reason=NAVIGATION_REQUEST_ZERO goal_id=%u nav_state=%u nav_phase=%u nav_reason=%s blocking=%s fpga_reason=%s now_us=%llu\n",r->view.goal.id,r->view.navigation.state,r->view.navigation.phase,r->view.navigation.reason,r->view.navigation.blocking_reason,r->view.feedback.reason,(unsigned long long)now);
  const s3_nearfield_view*nf=&r->view.navigation.nearfield;
  fprintf(stderr,"MOTION_ZERO_GEOMETRY checked_us=%llu nf_code=%u nf_reason=%s source=%u source_us=%llu v=%.4f w=%.4f horizon=%.3f hit=%.3f,%.3f ttc=%.3f cells=%u/%u pending=%u unknown=%u scan_age_us=%llu\n",
   (unsigned long long)nf->query_time_us,nf->query_code,nf->query_reason,nf->query_source,(unsigned long long)nf->query_source_time_us,nf->query_v_m_s,nf->query_w_rad_s,nf->query_horizon_s,nf->query_x_m,nf->query_y_m,nf->query_ttc_s,nf->history_cells,nf->history_capacity,nf->history_pending,nf->history_unknown,(unsigned long long)nf->age_us);
 }
 if(!revoke_check(r,now))return finish(r,LAB_OK,"MANUAL_REVOKED_NAVIGATION",now);
 ++r->view.hardware_calls;s=r->bindings.chassis.ops->apply(r->bindings.chassis.user,&r->view.protected_output,now);
 if(s==LAB_OK){r->last_drive_motion=(r->view.request.linear_m_s!=0||r->view.request.angular_rad_s!=0);int moving_target=0;for(unsigned i=0;i<4;i++)if(fabs(r->view.protected_output.wheel_rpm[i])>=.01)moving_target=1;if(!r->first_apply_us&&moving_target){r->first_apply_us=now;r->start_deadline_us=0;}}
 else fprintf(stderr,"CHASSIS_APPLY_FAILURE status=%d goal_id=%u request_seq=%u request_expires_us=%llu protected_expires_us=%llu fpga_reason=%s now_us=%llu\n",(int)s,r->view.goal.id,r->view.request.sequence,(unsigned long long)r->view.request.expires_us,(unsigned long long)r->view.protected_output.command.expires_us,r->view.feedback.reason,(unsigned long long)now);
 return finish(r,s,s==LAB_OK?(ordinary_wait?"NAVIGATION_WAITING":"COMMAND_SUBMITTED_WAIT_ACK"):"CHASSIS_APPLY_FAILED",now);
}
lab_status lab_real_runtime_set_goal(lab_real_runtime *r,const lab_real_goal *g,uint64_t now){
 if(!r||!g||!g->id||!g->map_id||g->has_yaw>1||!isfinite(g->x_m)||!isfinite(g->y_m)||!isfinite(g->yaw_rad))return LAB_ERR_ARGUMENT;
 if(!r->have_tick||now<r->view.time_us)return LAB_ERR_TIME;
 if(r->config.mode<LAB_REAL_SHADOW||!r->bindings.navigation.goal)return LAB_ERR_READONLY;
 if(!r->view.has_localization||g->map_id!=r->view.localization.map.map_id)return LAB_ERR_CONTEXT;
 ++r->goal_generation;halt(r,now,"NEW_GOAL");lab_status s=r->bindings.navigation.goal(r->bindings.navigation.user,g,now);r->view.has_goal=s==LAB_OK;if(s==LAB_OK)r->view.goal=*g;return s;
}
lab_status lab_real_runtime_cancel(lab_real_runtime *r,uint64_t now){if(!r)return LAB_ERR_ARGUMENT;++r->goal_generation;r->recovery_goal_pending=0;halt(r,now,"USER_OR_PHONE_CANCEL");r->view.has_goal=0;if(r->stop_fault)return finish(r,LAB_ERR_IO,"STOP_TRANSPORT_FAILURE",now);if(r->bindings.navigation.cancel)return r->bindings.navigation.cancel(r->bindings.navigation.user,now);return LAB_OK;}
lab_status lab_real_runtime_start_deadline(lab_real_runtime*r,uint64_t expires){if(!r||!expires)return LAB_ERR_ARGUMENT;if(r->view.enabled||r->view.pending_enable)return LAB_ERR_READONLY;r->start_deadline_us=expires;r->first_apply_us=0;return LAB_OK;}
uint64_t lab_real_runtime_first_apply_us(const lab_real_runtime*r){return r?r->first_apply_us:0;}
lab_status lab_real_runtime_start_ready(const lab_real_runtime*r,uint64_t now){
 if(!r)return LAB_ERR_ARGUMENT;
 if(r->stop_fault)return LAB_ERR_IO;if(r->context_fault||r->clock_fault)return LAB_ERR_CONTEXT;
 if(r->start_deadline_us&&now>=r->start_deadline_us)return LAB_ERR_TIME;
 if(r->view.object_mode){
  if(!r->object_controller||!r->prepared_candidate||r->prepared_generation!=r->goal_generation||!r->have_tick||now<r->view.time_us||now-r->view.time_us>CONT_CONTROL_US||r->view.missing_modules||!lab_real_context_equal(&r->view.request.context,&r->view.context)||!lab_real_command_valid(&r->view.request,now))return LAB_AGAIN;
  if(!fresh_bound(r->view.has_scan,r->view.scan_time_us,r->scan_uncertainty,now,runtime_scan_age(r))||!fresh_bound(r->view.has_odometry,r->view.odometry_time_us,r->odometry_uncertainty,now,runtime_pose_age(r)))return LAB_AGAIN;
  if(r->config.geometry_verification_required&&!r->config.geometry_verified)return LAB_AGAIN;return LAB_OK;
 }
 if(!r->prepared_candidate||r->prepared_generation!=r->goal_generation||!r->have_tick||now<r->view.time_us||now-r->view.time_us>CONT_CONTROL_US||r->view.missing_modules||!r->view.has_goal||!r->view.has_localization)return LAB_AGAIN;
 if(!lab_real_context_equal(&r->view.request.context,&r->view.context)||!lab_real_command_valid(&r->view.request,now)||!navigable(&r->view.localization))return LAB_AGAIN;
 if(!fresh_bound(r->view.has_scan,r->view.scan_time_us,r->scan_uncertainty,now,runtime_scan_age(r))||!fresh_bound(r->view.has_odometry,r->view.odometry_time_us,r->odometry_uncertainty,now,runtime_pose_age(r))||!fresh(r->view.localization.has_pose,r->view.localization.pose_time_us,now,runtime_pose_age(r)))return LAB_AGAIN;
 if(r->config.geometry_verification_required&&!r->config.geometry_verified)return LAB_AGAIN;
 return LAB_OK;
}
lab_status lab_real_runtime_enable(lab_real_runtime *r,uint32_t enabled,uint64_t now){
 if(!r||enabled>1)return LAB_ERR_ARGUMENT;if(!enabled){halt(r,now,"EXPLICIT_DISABLE");return r->stop_fault?LAB_ERR_IO:LAB_OK;}
 if(r->stop_fault)return LAB_ERR_IO;
 if(r->view.resume_requires_start){snprintf(r->view.reason,sizeof(r->view.reason),"FINAL_MOTOR_FAULT_EXPLICIT_START_REQUIRED");return LAB_AGAIN;}
 if(!r->config.allow_hardware_output||r->config.mode!=LAB_REAL_NAVIGATION||r->view.context.source!=LAB_REAL_HARDWARE||r->view.context.clock!=LAB_REAL_MONOTONIC)return LAB_ERR_READONLY;
 if(r->view.manual_revoked||!revoke_check(r,now))return LAB_AGAIN;
 if(r->view.enabled)return LAB_OK;
 if(r->view.pending_enable)return LAB_AGAIN;
 if(r->context_fault||r->clock_fault)return LAB_ERR_CONTEXT;
 if(r->config.geometry_verification_required&&!r->config.geometry_verified){snprintf(r->view.reason,sizeof(r->view.reason),"LIDAR_OFFSET_NOT_VERIFIED");return LAB_AGAIN;}
 /* 标定状态照常上报，不单独作为 ARM 的拒绝条件。 */
 lab_status prepared=lab_real_runtime_start_ready(r,now);if(prepared!=LAB_OK)return prepared;
 if(!r->view.object_mode){
 if(!r->have_tick||now<r->view.time_us||now-r->view.time_us>CONT_CONTROL_US||r->view.missing_modules||r->clock_fault||!r->view.has_goal||!r->view.has_localization)return LAB_AGAIN;
 if(!navigable(&r->view.localization)||!fresh_bound(r->view.has_scan,r->view.scan_time_us,r->scan_uncertainty,now,runtime_scan_age(r))||!fresh_bound(r->view.has_odometry,r->view.odometry_time_us,r->odometry_uncertainty,now,runtime_pose_age(r))||!fresh(r->view.localization.has_pose,r->view.localization.pose_time_us,now,runtime_pose_age(r)))return LAB_AGAIN;
 }
 lab_chassis_feedback f;lab_status s=r->bindings.chassis.ops->feedback(r->bindings.chassis.user,now,&f);
 if(!s5_after_feedback_clock(r,&now))return finish(r,LAB_ERR_TIME,"FEEDBACK_CLOCK_ROLLBACK",now);
 if(s==LAB_OK)r->view.feedback=f;
 if(s!=LAB_OK||!lab_real_context_equal(&f.context,&r->view.context)||f.backend!=LAB_CHASSIS_HARDWARE||(f.capabilities&7)!=7||f.faults||f.time_us>now||now-f.time_us>runtime_feedback_age(r))return LAB_ERR_UNAVAILABLE;
 ++r->view.hardware_calls;s=r->bindings.chassis.ops->set_enabled(r->bindings.chassis.user,1,r->config.control_session,now);r->view.enabled=s==LAB_OK;if(s==LAB_OK||s==LAB_AGAIN)r->view.object_soft_rearm_pending=0;
 if(s==LAB_AGAIN){r->view.pending_enable=1;r->enable_started_us=now;}return s;
}
lab_status lab_real_runtime_acknowledge_start(lab_real_runtime*r,uint64_t now){
 if(!r)return LAB_ERR_ARGUMENT;
 if(!revoke_check(r,now))return LAB_AGAIN;r->view.manual_revoked=0;if(!r->view.resume_requires_start)return LAB_OK;
 if(r->view.enabled||r->view.pending_enable||!r->view.feedback.time_us||now<r->view.feedback.time_us||now-r->view.feedback.time_us>runtime_feedback_age(r)||r->view.feedback.faults)return LAB_AGAIN;
 r->view.resume_requires_start=0;return LAB_OK;
}
lab_status lab_real_runtime_reset(lab_real_runtime *r,const lab_real_context *c){
 if(!r||!lab_real_context_valid(c)||c->clock==LAB_REAL_UNMAPPED_CLOCK)return LAB_ERR_ARGUMENT;
 if(r->stop_fault)return LAB_ERR_IO;
 if(c->source!=r->view.context.source||c->clock!=r->view.context.clock||lab_real_context_equal(c,&r->view.context))return LAB_ERR_CONTEXT;
 halt(r,r->view.time_us,"CONTEXT_RESET");if(r->stop_fault)return LAB_ERR_IO;r->clock_fault=1;r->view.has_goal=0;r->recovery_goal_pending=0;r->deferred_count=0;lab_status s;
 if(r->bindings.sensors.ops&&(s=r->bindings.sensors.ops->reset(r->bindings.sensors.user,c))!=LAB_OK)return s;
 if(r->bindings.localization.ops&&(s=r->bindings.localization.ops->reset(r->bindings.localization.user,c))!=LAB_OK)return s;
 if(r->bindings.chassis.ops&&(s=r->bindings.chassis.ops->reset(r->bindings.chassis.user,c))!=LAB_OK)return s;
 if(r->bindings.navigation.reset&&(s=r->bindings.navigation.reset(r->bindings.navigation.user,c))!=LAB_OK)return s;
 if(r->bindings.navigation.cancel)r->bindings.navigation.cancel(r->bindings.navigation.user,r->view.time_us);
 uint32_t missing=r->view.missing_modules;memset(&r->view,0,sizeof(r->view));r->view.context=*c;r->view.mode=r->config.mode;r->view.missing_modules=missing;memset(r->streams,0,sizeof(r->streams));r->have_rx=r->have_tick=r->clock_fault=r->context_fault=r->have_forwarded_pose=0;r->parameter_version=r->map_id=r->map_version=r->correction_version=r->scan_uncertainty=r->odometry_uncertainty=r->last_forwarded_pose_us=0;return LAB_OK;
}
lab_status lab_real_runtime_view_get(const lab_real_runtime *r,lab_real_runtime_view *out){if(!r||!out)return LAB_ERR_ARGUMENT;*out=r->view;return LAB_OK;}
void lab_real_runtime_close(lab_real_runtime *r){if(!r)return;halt(r,r->view.time_us,"SESSION_CLOSE");free(r->sample);free(r->scan);free(r);}
lab_status lab_real_runtime_observe(lab_real_runtime *r,lab_real_sample_observer cb,void *user){if(!r)return LAB_ERR_ARGUMENT;if(r->have_tick)return LAB_ERR_READONLY;r->observer=cb;r->observer_user=user;return LAB_OK;}

lab_status lab_real_runtime_set_cruise(lab_real_runtime*r,uint32_t cruise){if(!r||cruise>1)return LAB_ERR_ARGUMENT;if(r->view.enabled||r->view.pending_enable)return LAB_ERR_READONLY;if(!r->bindings.navigation.set_cruise)return LAB_ERR_UNAVAILABLE;return r->bindings.navigation.set_cruise(r->bindings.navigation.user,cruise);}

lab_status lab_real_runtime_object_controller(lab_real_runtime*r,lab_real_object_controller cb,void*user){if(!r)return LAB_ERR_ARGUMENT;if(r->view.enabled||r->view.pending_enable)return LAB_ERR_READONLY;r->object_controller=cb;r->object_user=user;return LAB_OK;}
lab_status lab_real_runtime_object_mode(lab_real_runtime*r,int enabled,uint64_t now){
 if(!r||(enabled<0||enabled>2))return LAB_ERR_ARGUMENT;
 if(enabled&&(!r->object_controller||!r->bindings.navigation.local_step||!r->bindings.navigation.local_publish||r->config.mode<LAB_REAL_SHADOW))return LAB_ERR_UNAVAILABLE;
 if(r->view.object_mode==(uint32_t)enabled)return LAB_OK;
 lab_status result=lab_real_runtime_cancel(r,now);if(result!=LAB_OK)return result;if(r->bindings.chassis.ops&&r->bindings.chassis.ops->object_profile){result=r->bindings.chassis.ops->object_profile(r->bindings.chassis.user,(uint32_t)enabled);if(result!=LAB_OK)return result;}if(r->bindings.navigation.local_mode){result=r->bindings.navigation.local_mode(r->bindings.navigation.user,(uint32_t)enabled);if(result!=LAB_OK)return result;}r->view.object_mode=(uint32_t)enabled;++r->goal_generation;r->prepared_candidate=0;return LAB_OK;
}

lab_status lab_real_runtime_object_rearm_ready(const lab_real_runtime*r,uint64_t now){
 if(!r)return LAB_ERR_ARGUMENT;
 if(!r->view.object_mode||!r->config.allow_hardware_output||r->config.mode!=LAB_REAL_NAVIGATION||r->view.context.source!=LAB_REAL_HARDWARE||r->view.context.clock!=LAB_REAL_MONOTONIC)return LAB_ERR_READONLY;
 if(r->stop_fault)return LAB_ERR_IO;if(r->context_fault||r->clock_fault)return LAB_ERR_CONTEXT;
 if(!r->view.object_soft_rearm_pending||r->view.enabled||r->view.pending_enable||r->view.manual_revoked||r->view.resume_requires_start||strcmp(r->view.reason,"HARDWARE_DISABLED"))return LAB_AGAIN;
 const lab_chassis_feedback*f=&r->view.feedback;
 if(!lab_real_context_equal(&f->context,&r->view.context)||f->backend!=LAB_CHASSIS_HARDWARE||(f->capabilities&7)!=7||f->faults||(f->local_stop_sources&0x3eu)||!f->time_us||f->time_us>now||now-f->time_us>runtime_feedback_age(r))return LAB_AGAIN;
 return lab_real_runtime_start_ready(r,now);
}

lab_status lab_real_runtime_object_target_v2(lab_real_runtime*r,const object_route_target_v2*t){
 if(!r||!t||!lab_real_context_equal(&r->view.context,&t->context))return LAB_ERR_CONTEXT;
 if(!r->view.object_mode||!r->bindings.navigation.local_target)return LAB_ERR_UNAVAILABLE;
 if(t->mode!=r->view.object_mode)return LAB_ERR_CONTEXT;
 return r->bindings.navigation.local_target(r->bindings.navigation.user,t);
}
