#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "lab_real_navigation.h"
#include "s2_navigation_limits.h"
#include "lab_real_nav_math.h"
#include <math.h>
#include "continuous_policy.h"
#include "object_time_policy.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef LAB_REAL_POSIX_SOURCE
#include <time.h>
static uint64_t diagnostic_clock_us(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;return (uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;}
#endif
typedef struct real_nav {
 lab_real_context context;lab_localization_binding localization;nav_config config;
 nav_engine *engine,*object_engine;int local_active,local_result_fresh;unsigned object_policy;uint64_t local_scan_us;
 struct{uint64_t time,error;}raw_dates[256];unsigned raw_date_count;uint64_t map_id,map_version,correction,revision;
 lab_real_goal goal;int pending_goal;
 int live_mapping;char waiting_reason[64];
 uint64_t last_step_us;int have_step;
 int cruise,have_map_odom;double map_odom_x,map_odom_y,map_odom_yaw;
 lab_real_command last_valid_command;int alignment_wait,alignment_holding,execution_ready,last_command_executing;
 lab_live_sample *pending_scans;unsigned pending_count;uint64_t queued_scan_time;
 lab_live_sample *aligned_scan;lab_real_meta aligned_meta;lab_pose aligned_pose;int have_aligned,result_fresh,scan_reused,fresh_required;uint64_t pending_dropped;
 uint32_t output_sequence; /* 不依赖重新启动的私有规划器。 */
 lab_event *pose,*scan,*scan_pose,*near_sample;lab_real_xy path[LAB_REAL_PATH_MAX],local_path[34];
} real_nav;
int w1_real_carrier_context(const lab_context *c){return c&&c->mode==W1_REAL_CARRIER&&c->timeline_id&&c->origin_epoch&&(c->clock_domain==LAB_CLOCK_HOST_MONOTONIC||c->clock_domain==LAB_CLOCK_REPLAY);}
int w1_real_carrier_event(const lab_event *e){
 if(!e)return 0;lab_context c={e->meta.mode,e->meta.clock_domain,e->meta.timeline_id,e->meta.origin_epoch};
 if(!w1_real_carrier_context(&c)||!e->meta.stream_session||!e->meta.stream_id||e->meta.received_us<e->meta.time_us||e->meta.flags&~3u)return 0;
 if(e->type==LAB_EVENT_POSE)return e->meta.provider==LAB_PROVIDER_ODOMETRY&&isfinite(e->data.pose.x_m)&&isfinite(e->data.pose.y_m)&&isfinite(e->data.pose.yaw_rad);
 return e->type==LAB_EVENT_SCAN&&(e->meta.provider==LAB_PROVIDER_USB||e->meta.provider==LAB_PROVIDER_FPGA)&&e->data.scan.count&&e->data.scan.count<=SCAN_MAX_POINTS&&e->data.scan.session==e->meta.stream_session&&e->data.scan.sequence==e->meta.sequence;
}
static lab_context carrier(lab_real_context c){return (lab_context){W1_REAL_CARRIER,c.clock==LAB_REAL_REPLAY_CLOCK?LAB_CLOCK_REPLAY:LAB_CLOCK_HOST_MONOTONIC,c.timeline_id,c.origin_epoch};}
static lab_meta meta(real_nav *n,const lab_real_meta *m,uint32_t provider){lab_context c=carrier(n->context);return (lab_meta){.mode=c.mode,.clock_domain=c.clock_domain,.timeline_id=c.timeline_id,.origin_epoch=c.origin_epoch,.provider=provider,.stream_id=m->stream_id,.stream_session=m->stream_session,.sequence=m->sequence,.flags=m->flags&3u,.time_us=m->time_us,.received_us=m->received_us};}
static void observe_sample(void*u,const lab_live_sample*s,uint64_t now){
 real_nav*n=u;if(!n||!n->engine||!n->near_sample||!s||!lab_real_context_equal(&n->context,&s->meta.context)||(s->meta.flags&3u)!=3u)return;
 lab_event*e=n->near_sample;
 if(s->type==LAB_EVENT_POSE){
  unsigned k=n->raw_date_count;if(k&&n->raw_dates[k-1].time==s->meta.time_us){n->raw_dates[k-1].error=s->meta.time_uncertainty_us;}else{if(k==256){memmove(n->raw_dates,n->raw_dates+1,255*sizeof(*n->raw_dates));k=255;}n->raw_dates[k].time=s->meta.time_us;n->raw_dates[k].error=s->meta.time_uncertainty_us;n->raw_date_count=k+1;}
  e->type=LAB_EVENT_POSE;e->meta=meta(n,&s->meta,LAB_PROVIDER_ODOMETRY);e->data.pose=s->data.odometry;}
 else if(s->type==LAB_EVENT_SCAN){e->type=LAB_EVENT_SCAN;e->meta=meta(n,&s->meta,s->meta.provider==LAB_REAL_USB?LAB_PROVIDER_USB:LAB_PROVIDER_FPGA);e->data.scan=s->data.scan;}
 else return;
 nav_observe_nearfield(n->engine,e,now);if(n->object_engine)nav_observe_nearfield(n->object_engine,e,now);
}
static lab_status open_engine(real_nav *n){lab_context c=carrier(n->context);lab_status s=nav_open(&n->engine,&n->config,&c);if(s==LAB_OK){s=nav_open(&n->object_engine,&n->config,&c);if(s!=LAB_OK){nav_close(n->engine);n->engine=NULL;return s;}nav_object_time_profile(n->object_engine);nav_set_cruise(n->object_engine,0);if(n->context.source==LAB_REAL_HARDWARE)nav_set_execution_hint(n->object_engine,0);nav_set_cruise(n->engine,n->cruise);if(n->context.source==LAB_REAL_HARDWARE)nav_set_execution_hint(n->engine,0);}return s;}
static lab_status goal(void *u,const lab_real_goal *g,uint64_t now){real_nav *n=u;if(!n||!g||!g->map_id||!g->id||g->has_yaw>1||!isfinite(g->x_m)||!isfinite(g->y_m)||!isfinite(g->yaw_rad))return LAB_ERR_ARGUMENT;nav_cancel(n->engine,now);n->waiting_reason[0]=0;n->last_valid_command=(lab_real_command){0};n->result_fresh=0;n->alignment_wait=n->alignment_holding=0;n->goal=*g;n->pending_goal=1;return LAB_OK;}
static lab_status cancel(void *u,uint64_t now){real_nav *n=u;if(!n)return LAB_ERR_ARGUMENT;n->pending_goal=0;n->waiting_reason[0]=0;n->last_valid_command=(lab_real_command){0};n->result_fresh=0;n->alignment_wait=n->alignment_holding=0;memset(&n->goal,0,sizeof(n->goal));nav_cancel(n->engine,now);if(n->object_engine)nav_cancel(n->object_engine,now);n->local_active=n->local_result_fresh=0;return LAB_OK;}
static lab_status alignment_wait_command(real_nav*n,uint64_t now,lab_real_command*out){
 n->alignment_wait=1;n->alignment_holding=0;n->result_fresh=0;
 if(n->last_valid_command.expires_us>now&&n->last_valid_command.generated_us<=now&&lab_real_context_equal(&n->last_valid_command.context,&n->context)&&
    (n->context.source!=LAB_REAL_HARDWARE||(n->execution_ready&&n->last_command_executing))&&
    nav_direction_allows(n->engine,now,n->last_valid_command.linear_m_s,n->last_valid_command.angular_rad_s)){
  *out=n->last_valid_command;n->alignment_holding=1; /* 沿用观测原时间和截止时间。 */
 }else *out=(lab_real_command){.context=n->context,.generated_us=now,.expires_us=now+n->config.command_valid_us};
 out->sequence=++n->output_sequence;return LAB_OK;
}
/* 未知或地图外格只作路线建议，实际执行依据同上下文当前扫描及原近距和线上保护。
 */
static lab_status step(void *u,uint64_t now,const lab_localization_view *v,const lab_live_sample *s,lab_real_command *out){
 real_nav *n=u;if(!n||!v||!s||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));out->context=n->context;
 if(!lab_real_context_equal(&v->context,&n->context)||!lab_real_context_equal(&s->meta.context,&n->context)||!lab_real_grid_valid(&v->map)||s->type!=LAB_EVENT_SCAN)return LAB_ERR_CONTEXT;
 if(v->map.map_id!=n->goal.map_id)return LAB_ERR_CONTEXT;
 if(n->map_id&&n->map_id!=v->map.map_id)return LAB_ERR_CONTEXT;
 if(n->have_step&&now<=n->last_step_us)return LAB_ERR_TIME;
 if(n->have_step&&now-n->last_step_us>250000){
  /* 数据过期暂停时保留上下文和显式目标，不重置恢复预算或缓存，不丢失刚到的底盘提示，也不隐式建立新脱困过程；下方仍重新检查实际时效。

 */
  lab_status resumed=nav_resume_after_pause(n->engine,now);if(resumed!=LAB_OK)return resumed;n->have_step=0;
 }
 if(n->have_map_odom){
  double a=remainder(v->map_odom_yaw_rad-n->map_odom_yaw,6.283185307179586);
  double dx=v->map_odom_x_m-(cos(a)*n->map_odom_x-sin(a)*n->map_odom_y),dy=v->map_odom_y_m-(sin(a)*n->map_odom_x+cos(a)*n->map_odom_y);
  if(hypot(dx,dy)>=.005||fabs(a)>=.003){nav_reframe_local(n->engine,dx,dy,a);if(n->have_aligned){double x=n->aligned_pose.x_m,y=n->aligned_pose.y_m;n->aligned_pose.x_m=dx+cos(a)*x-sin(a)*y;n->aligned_pose.y_m=dy+sin(a)*x+cos(a)*y;n->aligned_pose.yaw_rad=remainder(n->aligned_pose.yaw_rad+a,6.283185307179586);}n->map_odom_x=v->map_odom_x_m;n->map_odom_y=v->map_odom_y_m;n->map_odom_yaw=v->map_odom_yaw_rad;}
 }else{n->have_map_odom=1;n->map_odom_x=v->map_odom_x_m;n->map_odom_y=v->map_odom_y_m;n->map_odom_yaw=v->map_odom_yaw_rad;}
 if(n->map_id!=v->map.map_id||n->map_version!=v->map.map_version){
  nav_view prior;nav_get_view(n->engine,&prior);
  lab_map_view map={.context=carrier(n->context),.width=v->map.width,.height=v->map.height,.resolution_m=v->map.resolution_m,.origin_x_m=v->map.origin_x_m,.origin_y_m=v->map.origin_y_m,.cells=v->map.cells};
  lab_status r=nav_set_map(n->engine,&map,++n->revision);if(r!=LAB_OK)return r;n->map_id=v->map.map_id;n->map_version=v->map.map_version;n->correction=v->map.correction_version;
  if(n->live_mapping&&n->goal.id&&(prior.state==NAV_FAILED||prior.state==NAV_UNREACHABLE))n->pending_goal=1;
 }
 /* 未知和地图外只影响路线建议，当前扫描保护执行。 */
 n->waiting_reason[0]=0;
 if(n->pending_goal){nav_goal g={n->goal.id,n->goal.has_yaw,n->goal.x_m,n->goal.y_m,n->goal.yaw_rad};lab_status r=nav_set_goal(n->engine,&g,now);if(r!=LAB_OK)return r;n->pending_goal=0;}
 /* 最多排队4个真实未对齐扫描，新原始位姿重试同一扫描；后来的待处理扫描不替换上一次成功对齐结果。
 */
 if(s->meta.time_us>n->queued_scan_time){if(n->pending_count==4){memmove(n->pending_scans,n->pending_scans+1,3*sizeof(*s));--n->pending_count;++n->pending_dropped;}n->pending_scans[n->pending_count++]=*s;n->queued_scan_time=s->meta.time_us;}
 unsigned retained=0;lab_status r=LAB_AGAIN;
 for(unsigned i=0;i<n->pending_count;i++){const lab_live_sample*q=&n->pending_scans[i];
  if(q->meta.time_us>now)return LAB_ERR_TIME;
  if(now-q->meta.time_us>CONT_SCAN_US){++n->pending_dropped;continue;}
  lab_real_meta am;lab_pose ap;r=n->localization.ops->pose_at(n->localization.user,q->meta.time_us,&am,&ap);
  if(r==LAB_AGAIN){n->pending_scans[retained++]=*q;continue;}if(r!=LAB_OK)return r;
  if(!lab_real_context_equal(&am.context,&n->context))return LAB_ERR_CONTEXT;
  if(am.time_us!=q->meta.time_us||am.received_us>now)return LAB_ERR_TIME;
  if(am.time_uncertainty_us>n->config.max_pose_age_us||now-am.time_us>n->config.max_pose_age_us-am.time_uncertainty_us){n->pending_scans[retained++]=*q;continue;}
  if(!n->have_aligned||q->meta.time_us>n->aligned_scan->meta.time_us){*n->aligned_scan=*q;n->aligned_meta=am;n->aligned_pose=ap;n->have_aligned=1;}
 }
 n->pending_count=retained;
 if(!n->have_aligned||now<n->aligned_scan->meta.time_us||now-n->aligned_scan->meta.time_us>CONT_SCAN_US){fprintf(stderr,"NAV_PENDING_WAIT now_us=%llu queued=%u dropped=%llu have_aligned=%d\n",(unsigned long long)now,n->pending_count,(unsigned long long)n->pending_dropped,n->have_aligned);return alignment_wait_command(n,now,out);}
 n->scan_reused=n->pending_count!=0;s=n->aligned_scan;lab_real_meta aligned=n->aligned_meta;lab_pose aligned_pose=n->aligned_pose;
 memset(n->pose,0,sizeof(*n->pose));memset(n->scan,0,sizeof(*n->scan));memset(n->scan_pose,0,sizeof(*n->scan_pose));
 n->alignment_wait=n->alignment_holding=0;
 lab_real_meta current=aligned;current.time_us=v->pose_time_us;current.received_us=now;
 n->pose->type=LAB_EVENT_POSE;n->pose->meta=meta(n,&current,LAB_PROVIDER_ODOMETRY);n->pose->data.pose=v->pose;
 n->scan->type=LAB_EVENT_SCAN;n->scan->meta=meta(n,&s->meta,s->meta.provider==LAB_REAL_USB?LAB_PROVIDER_USB:LAB_PROVIDER_FPGA);n->scan->data.scan=s->data.scan;
 n->scan_pose->type=LAB_EVENT_POSE;n->scan_pose->meta=meta(n,&aligned,LAB_PROVIDER_ODOMETRY);n->scan_pose->data.pose=aligned_pose;
 nav_set_time_uncertainty(n->engine,(s->meta.time_uncertainty_us+aligned.time_uncertainty_us)/1000000.0);
 nav_command c;
#ifdef LAB_REAL_POSIX_SOURCE
 nav_view before;nav_get_view(n->engine,&before);uint64_t started=diagnostic_clock_us();
#endif
 r=nav_step(n->engine,now,n->pose,n->scan,n->scan_pose,&c);nav_profile_log(n->engine,now);n->last_step_us=now;n->have_step=1;nav_view view;nav_get_view(n->engine,&view);
#ifdef LAB_REAL_POSIX_SOURCE
 uint64_t ended=diagnostic_clock_us();
 if(view.plans!=before.plans || (started && ended>=started && ended-started>50000))
  fprintf(stderr,"NAV_SEARCH_DIAG elapsed_us=%llu plans_total=%llu expanded_delta=%llu expanded_total=%llu status=%d reason=%s geometry=%s path_points=%u\n",(unsigned long long)(ended>=started?ended-started:0),(unsigned long long)view.plans,(unsigned long long)(view.expanded_nodes>=before.expanded_nodes?view.expanded_nodes-before.expanded_nodes:0),(unsigned long long)view.expanded_nodes,(int)r,nav_reason_text(view.reason),passage_reason_text((int)view.passage_reason),view.path_count);
#endif
 if(view.state==NAV_REACHED)return LAB_EOF;int retained_wait=n->config.passage.continuous&&view.state==NAV_WAITING&&r==LAB_OK&&c.expires_us>now;
 if(r!=LAB_OK||(view.state!=NAV_TRACKING&&!retained_wait)){
  fprintf(stderr,"NAV_STEP_DIAG status=%d state=%u phase=%u reason=%s blocking=%s geometry=%s path_points=%u map_version=%llu plans=%llu\n",(int)r,view.state,view.phase,nav_reason_text(view.reason),nav_reason_text(view.blocking_reason),passage_reason_text((int)view.passage_reason),view.path_count,(unsigned long long)v->map.map_version,(unsigned long long)view.plans);
  return r!=LAB_OK?r:LAB_AGAIN;
 }
 out->motion_intent=c.motion_intent;out->linear_m_s=c.linear_m_s;out->angular_rad_s=c.angular_rad_s;out->generated_us=c.time_us;out->expires_us=c.expires_us;out->sequence=++n->output_sequence;n->result_fresh=1;n->fresh_required=0;if(n->context.source!=LAB_REAL_HARDWARE){n->last_valid_command=*out;n->last_command_executing=n->execution_ready;}return LAB_OK;
}
static lab_status publish(void*u,uint64_t now,const lab_localization_view*v,lab_real_command*out){
 real_nav*n=u;if(!n||!v||!out||!lab_real_context_equal(&v->context,&n->context)||v->map.map_id!=n->goal.map_id)return LAB_ERR_CONTEXT;
 if(n->fresh_required)return LAB_AGAIN;
 if(!n->result_fresh){if(out->expires_us<=now)*out=(lab_real_command){.context=n->context,.generated_us=now,.expires_us=now+CONT_COMMAND_US};return LAB_OK;}
 n->result_fresh=0;
 if(!v->has_pose||v->pose_time_us>now||now-v->pose_time_us>CONT_ODOM_US||!(v->pose.quality&LAB_POSE_VALID)||
    !nav_action_validate(n->engine,now,&v->pose,v->pose_time_us,out->linear_m_s,out->angular_rad_s)){
  *out=(lab_real_command){.context=n->context,.generated_us=now,.expires_us=now+CONT_COMMAND_US};return LAB_AGAIN;
 }
 #ifdef LAB_REAL_POSIX_SOURCE
 if(n->context.source==LAB_REAL_HARDWARE){uint64_t end=diagnostic_clock_us();if(end<now)return LAB_ERR_TIME;now=end;}
#endif
 if(now<v->pose_time_us||now-v->pose_time_us>CONT_ODOM_US||now<n->aligned_scan->meta.time_us||now-n->aligned_scan->meta.time_us>CONT_SCAN_US)return LAB_AGAIN;
 out->motion_intent=nav_action_intent(n->engine,out->linear_m_s,out->angular_rad_s);out->generated_us=now;out->expires_us=now+CONT_COMMAND_US;n->last_valid_command=*out;n->last_command_executing=n->execution_ready;
 nav_command_published(n->engine,now,out->expires_us);return LAB_OK;
}
static lab_status local_target(void*u,const object_route_target_v2*t){real_nav*n=u;if(!n||!t||!lab_real_context_equal(&n->context,&t->context))return LAB_ERR_CONTEXT;return nav_external_target_v2(n->object_engine,t);}
static lab_status local_mode(void*u,uint32_t mode){real_nav*n=u;if(!n||mode>2)return LAB_ERR_ARGUMENT;n->object_policy=mode?mode:1;nav_external_profile(n->object_engine,n->object_policy);return LAB_OK;}
static lab_status local_step(void*u,uint64_t now,const lab_pose*raw,uint64_t stamp,const lab_live_sample*scan,const lab_real_command*desired,double cap,lab_real_command*out){
 real_nav*n=u;if(!n||!raw||!scan||!desired||!out||!lab_real_context_equal(&desired->context,&n->context)||!lab_real_context_equal(&scan->meta.context,&n->context)||scan->type!=LAB_EVENT_SCAN)return LAB_ERR_CONTEXT;
 *out=(lab_real_command){.context=n->context,.generated_us=now,.expires_us=now+CONT_COMMAND_US};n->local_active=1;n->local_result_fresh=0;n->waiting_reason[0]=0;
 lab_pose at;if(!nav_raw_pose_at(n->object_engine,scan->meta.time_us,&at)){snprintf(n->waiting_reason,sizeof(n->waiting_reason),"LOCAL_SCAN_RAW_POSE_ALIGNMENT_WAIT");return LAB_AGAIN;}
 lab_real_meta pm=scan->meta;pm.time_us=stamp;pm.received_us=now;
 n->pose->type=LAB_EVENT_POSE;n->pose->meta=meta(n,&pm,LAB_PROVIDER_ODOMETRY);n->pose->data.pose=*raw;
 n->scan->type=LAB_EVENT_SCAN;n->scan->meta=meta(n,&scan->meta,scan->meta.provider==LAB_REAL_USB?LAB_PROVIDER_USB:LAB_PROVIDER_FPGA);n->scan->data.scan=scan->data.scan;
 pm.time_us=scan->meta.time_us;n->scan_pose->type=LAB_EVENT_POSE;n->scan_pose->meta=meta(n,&pm,LAB_PROVIDER_ODOMETRY);n->scan_pose->data.pose=at;
 uint64_t aligned_error=UINT64_MAX,current_error=n->raw_date_count?n->raw_dates[n->raw_date_count-1].error:UINT64_MAX;
 for(unsigned k=0;k<n->raw_date_count;k++){if(n->raw_dates[k].time==scan->meta.time_us){aligned_error=n->raw_dates[k].error;break;}if(k&&n->raw_dates[k].time>scan->meta.time_us){aligned_error=n->raw_dates[k].error>n->raw_dates[k-1].error?n->raw_dates[k].error:n->raw_dates[k-1].error;break;}}
 if(aligned_error>OBJECT_CONTROL_ALIGNMENT_US||current_error>OBJECT_CONTROL_POSE_US||scan->meta.time_us>now||now-scan->meta.time_us>OBJECT_CONTROL_ALIGNMENT_US-aligned_error){snprintf(n->waiting_reason,sizeof(n->waiting_reason),"LOCAL_SCAN_RAW_POSE_TIME_BOUND");return LAB_AGAIN;}
 nav_set_time_uncertainty(n->object_engine,(scan->meta.time_uncertainty_us+fmax(aligned_error,current_error))/1e6);
 nav_external_profile(n->object_engine,n->object_policy==2?2:1);nav_command command;lab_status rc=nav_external_step(n->object_engine,now,n->pose,n->scan,n->scan_pose,desired->linear_m_s,desired->angular_rad_s,cap,&command);nav_profile_log(n->object_engine,now);
 if(rc!=LAB_OK)return rc;out->linear_m_s=command.linear_m_s;out->angular_rad_s=command.angular_rad_s;out->motion_intent=command.motion_intent;out->generated_us=command.time_us;out->expires_us=command.expires_us;out->sequence=++n->output_sequence;n->local_scan_us=scan->meta.time_us;n->local_result_fresh=1;return LAB_OK;
}
static lab_status local_publish(void*u,uint64_t now,const lab_pose*raw,uint64_t stamp,lab_real_command*out){
 real_nav*n=u;if(!n||!raw||!out||!n->local_active)return LAB_ERR_ARGUMENT;
 if(!n->local_result_fresh)return LAB_AGAIN;n->local_result_fresh=0;
 /* 保持始终请求零速度，不触发R8脱困动作。 */
 if(fabs(out->linear_m_s)<1e-9&&fabs(out->angular_rad_s)<1e-9){out->generated_us=now;out->expires_us=now+CONT_COMMAND_US;return LAB_OK;}
 if(stamp>now||now-stamp>OBJECT_CONTROL_POSE_US||n->local_scan_us>now||now-n->local_scan_us>OBJECT_CONTROL_SCAN_US||!nav_action_validate(n->object_engine,now,raw,stamp,out->linear_m_s,out->angular_rad_s)){
  *out=(lab_real_command){.context=n->context,.generated_us=now,.expires_us=now+CONT_COMMAND_US};return LAB_AGAIN;
 }
 out->motion_intent=nav_action_intent(n->object_engine,out->linear_m_s,out->angular_rad_s);out->generated_us=now;out->expires_us=now+CONT_COMMAND_US;nav_command_published(n->object_engine,now,out->expires_us);return LAB_OK;
}
static void reduce_load(void*u,uint32_t reduced){real_nav*n=u;if(n){nav_reduce_load(n->engine,reduced);nav_reduce_load(n->object_engine,reduced);}}
static void engine_chassis_hint(nav_engine*engine,const lab_chassis_feedback*f){nav_set_chassis_hint_s1(engine,f->time_us,f->sonar_mm,f->sonar_valid,f->sonar_fresh,f->sonar_near,f->sonar_degraded,f->sonar_front_stable,f->sonar_pending);nav_set_chassis_samples_s5(engine,f->time_us,f->sonar_front_sequence,f->sonar_front_sequence_valid,f->sonar_raw_zero_mask);nav_set_chassis_raw_s5(engine,f->sonar_diagnostic_time_us,f->sonar_source_uptime_ms,f->sonar_raw_mm,f->sonar_nonzero_sequence,f->sonar_raw_age_us,f->sonar_raw_available);nav_set_execution_feedback(engine,f->armed&&f->time_us!=0,f->actual_rpm);}
static void chassis_hint(void*u,const lab_chassis_feedback*f){real_nav*n=u;if(n&&f){n->execution_ready=f->armed&&f->time_us!=0;engine_chassis_hint(n->engine,f);engine_chassis_hint(n->object_engine,f);}}
static lab_status view(void *u,lab_real_navigation_view *out){real_nav *n=u;if(!n||!out)return LAB_ERR_ARGUMENT;nav_view v;lab_status r=nav_get_view(n->local_active?n->object_engine:n->engine,&v);if(r!=LAB_OK)return r;memset(out,0,sizeof(*out));out->state=v.state;out->phase=v.phase;out->recovery_distance_m=v.backup_progress_m;out->recovery_target_m=v.backup_target_m>0?v.backup_target_m:.38;out->extended_reverse_attempts=v.extended_reverse_attempts;out->extended_reverse_active=v.extended_reverse_active;out->recovery_attempts=v.reverse_attempts;out->plans=v.plans;out->map_version=n->map_version;out->path_count=v.path_count;out->narrow_passage=v.narrow_passage;out->lidar_offset_verified=v.lidar_offset_verified;out->passage_reason=v.passage_reason;out->body_length_m=v.vehicle_length_m;out->body_width_m=v.vehicle_width_m;out->margin_m=v.side_margin_m;out->local_resolution_m=v.local_resolution_m;
 out->route_complete=v.route_complete;out->route_partial=v.route_partial;out->route_search_budget=v.route_search_budget;out->route_kind=v.route_kind;out->candidate_budget_rejections=v.candidate_budget_rejections;snprintf(out->route_reason,sizeof(out->route_reason),"%s",v.route_reason);
 if(n->local_active){snprintf(out->local_path_frame,sizeof(out->local_path_frame),"ODOM");out->local_path_generation=v.object_generation;out->local_path_source_us=v.object_source_us;out->has_object_target=v.has_object_target;out->actual_target_x=v.object_target_x;out->actual_target_y=v.object_target_y;out->actual_goal_x=v.object_goal_x;out->actual_goal_y=v.object_goal_y;}
 out->nearfield=v.nearfield;out->avoidance=v.avoidance;
#ifdef LAB_REAL_POSIX_SOURCE
 if(n->context.source==LAB_REAL_HARDWARE){uint64_t current=diagnostic_clock_us();if(current)nav_nearfield_view(n->local_active?n->object_engine:n->engine,current,&out->nearfield);}
#endif
 out->cruise_mode=v.cruise_mode;out->local_probe=v.local_probe;out->can_start_local=v.can_start_local;out->backup_requests=v.backup_requests;
 out->provisional_route=v.provisional_route;out->candidate_sonar_rejections=v.candidate_sonar_rejections;out->candidate_occupied_rejections=v.candidate_occupied_rejections;out->candidate_unknown_rejections=v.candidate_unknown_rejections;out->candidate_accepted=v.candidate_accepted;out->observed_fraction=v.observed_fraction;snprintf(out->rear_reason,sizeof(out->rear_reason),"%s",v.rear_reason);
 if(n->waiting_reason[0]){out->state=NAV_WAITING;out->phase=NAV_PHASE_WAIT;out->path_count=0;snprintf(out->reason,sizeof(out->reason),"%s",n->waiting_reason);snprintf(out->blocking_reason,sizeof(out->blocking_reason),"%s",n->waiting_reason);snprintf(out->phase_text,sizeof(out->phase_text),"SAFE_WAIT");return LAB_OK;}
 if(v.path_count>LAB_REAL_PATH_MAX)return LAB_ERR_LIMIT;for(uint32_t i=0;i<v.path_count;++i)n->path[i]=(lab_real_xy){v.path[i].x_m,v.path[i].y_m};out->path=n->path;out->local_path_count=v.local_path_count;if(v.local_path_count>34)return LAB_ERR_LIMIT;for(uint32_t k=0;k<v.local_path_count;k++)n->local_path[k]=(lab_real_xy){v.local_path[k].x_m,v.local_path[k].y_m};out->local_path=n->local_path;snprintf(out->reason,sizeof(out->reason),"%s",nav_reason_text(v.reason));snprintf(out->blocking_reason,sizeof(out->blocking_reason),"%s",nav_reason_text(v.blocking_reason));snprintf(out->phase_text,sizeof(out->phase_text),"%s",nav_phase_text(v.phase));
 if(n->alignment_wait){snprintf(out->reason,sizeof(out->reason),"%s",n->alignment_holding?"ALIGNMENT_WAIT_HOLD":"ALIGNMENT_WAIT_ZERO");snprintf(out->phase_text,sizeof(out->phase_text),"%s",n->alignment_holding?"ALIGNMENT_HOLD":"ALIGNMENT_WAIT");}
 return LAB_OK;}
static lab_status reset(void *u,const lab_real_context *c){real_nav *n=u;if(!n||!lab_real_context_valid(c))return LAB_ERR_ARGUMENT;nav_engine *old=n->engine,*old_object=n->object_engine;n->engine=n->object_engine=NULL;lab_real_context prev=n->context;n->context=*c;lab_status r=open_engine(n);if(r!=LAB_OK){n->context=prev;n->engine=old;n->object_engine=old_object;return r;}nav_close(old);nav_close(old_object);n->local_active=n->local_result_fresh=0;n->raw_date_count=0;n->map_id=n->map_version=n->correction=n->revision=0;n->pending_goal=0;n->pending_count=0;n->have_aligned=0;n->queued_scan_time=0;n->result_fresh=0;n->waiting_reason[0]=0;n->last_step_us=0;n->have_step=0;memset(&n->goal,0,sizeof(n->goal));return LAB_OK;}
static void close_nav(void *u){real_nav *n=u;if(!n)return;nav_close(n->engine);nav_close(n->object_engine);free(n->pose);free(n->scan);free(n->scan_pose);free(n->near_sample);free(n->pending_scans);free(n->aligned_scan);free(n);}
static void invalidate_command(void*u){real_nav*n=u;if(!n)return;n->last_valid_command=(lab_real_command){0};n->result_fresh=0;n->fresh_required=1;n->alignment_holding=0;n->last_command_executing=0;nav_invalidate_command(n->engine);nav_invalidate_command(n->object_engine);n->local_result_fresh=0;}
static lab_status set_cruise(void*u,uint32_t enabled){real_nav*n=u;if(!n||enabled>1)return LAB_ERR_ARGUMENT;n->cruise=(int)enabled;nav_set_cruise(n->engine,enabled);return LAB_OK;}
lab_status lab_real_navigation_create_profile(lab_real_navigation_ops *out,const lab_real_context *c,const lab_localization_binding *l,const lab_localization_config *g,const passage_config*profile){
 if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));if(!lab_real_context_valid(c)||!l||!l->ops||!l->ops->pose_at||!g||!passage_config_valid(profile))return LAB_ERR_ARGUMENT;
 real_nav *n=calloc(1,sizeof(*n));if(!n)return LAB_ERR_NOMEM;n->context=*c;n->localization=*l;n->live_mapping=g->mode==LAB_REAL_BUILD_MAP;nav_config_default(&n->config);n->config.max_pose_age_us=CONT_ODOM_US; /* 当前控制位姿年龄独立于最近扫描匹配成功年龄。 */n->config.max_scan_age_us=CONT_SCAN_US; /* 与lab_real_session.c使用相同实时扫描边界。 */ n->config.max_speed_m_s=.25;n->config.max_yaw_rate_rad_s=S2_MAX_YAW_RAD_S;n->config.enable_recovery=0;n->config.mount_x_m=g->mount_x_m;n->config.mount_y_m=g->mount_y_m;n->config.mount_yaw_rad=g->mount_yaw_rad;
 n->config.rectangular_footprint=1;n->config.passage=*profile;n->config.passage.continuous=c->source==LAB_REAL_HARDWARE||profile->continuous;n->config.passage.mount_x_m=g->mount_x_m;n->config.passage.mount_y_m=g->mount_y_m;n->config.passage.mount_yaw_rad=g->mount_yaw_rad;
 n->config.vehicle_width_m=profile->width_m;n->config.vehicle_length_m=profile->length_m;n->config.safety_margin_m=profile->margin_m;n->config.max_speed_m_s=profile->normal_speed_m_s;n->config.lidar_to_rear_m=profile->length_m*.5+g->mount_x_m;
 /* 通过共享运行对象显式选择模式，不依赖隐式环境状态。 */
 n->cruise=0;
 n->config.command_valid_us=CONT_COMMAND_US;n->pending_scans=calloc(4,sizeof(*n->pending_scans));n->aligned_scan=calloc(1,sizeof(*n->aligned_scan));
 n->pose=calloc(1,sizeof(*n->pose));n->scan=calloc(1,sizeof(*n->scan));n->scan_pose=calloc(1,sizeof(*n->scan_pose));n->near_sample=calloc(1,sizeof(*n->near_sample));if(!n->pose||!n->scan||!n->scan_pose||!n->near_sample||!n->pending_scans||!n->aligned_scan){close_nav(n);return LAB_ERR_NOMEM;}lab_status r=open_engine(n);if(r!=LAB_OK){close_nav(n);return r;}
 *out=(lab_real_navigation_ops){n,goal,cancel,step,reset,view,close_nav,chassis_hint,publish,reduce_load,set_cruise,invalidate_command,observe_sample,local_step,local_publish,local_mode,local_target};return LAB_OK;
}
lab_status lab_real_navigation_create(lab_real_navigation_ops*out,const lab_real_context*c,const lab_localization_binding*l,const lab_localization_config*g){passage_config p;passage_config_default(&p);return lab_real_navigation_create_profile(out,c,l,g,&p);}
