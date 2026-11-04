#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "lab_real_nav_math.h"
#include "nav_private.h"
#include "s1_motion_policy.h"
#include "s2_navigation_limits.h"
#include "nearfield_s3.h"
#include "object_time_policy.h"
#include "object_follow_policy_v6.h"
#include "s6_planning_types.h"
#include "r4_scene_types.h"
#include "nav_near_scene.inc"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>
#include <stdio.h>
#ifdef LAB_REAL_POSIX_SOURCE
#include <pthread.h>
#include <time.h>
#include <stdatomic.h>
typedef struct async_plan {
 pthread_t thread;pthread_mutex_t mutex;int active,done,result,orphaned;uint64_t revision,generation,elapsed_us;
 passage_world world;passage_local *local;int8_t *cells;passage_pose start;passage_xy goal;
 int has_yaw;double goal_yaw;passage_xy path[PASSAGE_PATH_MAX];unsigned count,expanded;
 atomic_uint cancelled;passage_plan_report report;
} async_plan;
#endif
#define PI 3.14159265358979323846
struct nav_engine {
    /* 实物入口元数据，下方由R8.1决定动作。 */
    uint32_t object_time_profile,external_active,external_profile;
    object_route_target_v2 object_target;uint64_t object_plan_us,object_grid_source;
    int8_t object_grid[PASSAGE_LOCAL_CELLS]; /* 实际局部射线投影，不依赖SLAM。 */
    nav_config c; lab_context context; nav_goal goal; nav_command cmd;
    nav_state state; nav_reason reason,blocking_reason;
    uint32_t w,h,n,path_n,segment,heap_n,have_goal,dirty,have_tick,waiting;
    double res,ox,oy,radius,last_distance;
    int8_t *cells; uint8_t *blocked,*closed; uint16_t *dist;
    double *cost; int32_t *parent,*heap_pos,*heap;
    nav_xy path[NAV_PATH_MAX],raw[NAV_PATH_MAX];
    uint64_t revision,plans,expanded,stops,goal_started,last_tick,last_plan,wait_started;
    size_t allocated;
    nav_phase phase;passage_plan_report route_report;double corner_distance,corner_angle;
    uint32_t reverse_attempts,clear_scans,scan_sequence,have_scan_sequence,still_samples;
    int turn_dir,episode,have_motion_pose;
    uint64_t phase_started,episode_started,motion_time,scan_time;
    double rear_clearance,backup_progress,backup_total,turn_total,turn_start_yaw;
    double measured_v,measured_w;
    nf_state nearfield;uint32_t nf_external;uint64_t nf_now;
    r4_scene_config r4_config;r4_scene_state r4_state;r4_scene_input r4_input;r4_scene_output r4_output;
    uint32_t r4_initialized,r4_active,r4_checking,r4_last_logged_scene,r4_last_logged_stage;uint64_t r4_last_log;
    double r6_desired_v,r6_desired_w;int32_t r6_preferred_side,r6_check_result;
    uint64_t r6_check_time,r6_cycle_deadline,r6_step_time,r6_saved_scene_time,r6_route_log_us;char r6_route_change_reason[64];r4_scene_state r6_saved_scene;
    /* 内部有界主动观察，不持有线上控制或目标权限。 */
    struct {uint32_t active,mission,cursor,kind,issuing;int sign;uint64_t scan_us,pose_us,active_us;
      double x,y,yaw,last_yaw,angle,v,w,target_m;} r8_observe;
    cm_state *s6_cm;s6_plan *s6_plan;s6_config s6_config;
    uint32_t s6_disabled,s6_active,s6_have_profile,s6_rank,s6_vetoed,s6_validating;
    uint64_t s6_generated,s6_tick,s6_log_time,s6_mismatch_since,s6_mismatch_log,s6_cm_source;
    double s6_v_error,s6_w_error;char s6_reason[64],s6_logged_reason[64];
    nf_check_result last_nf_check;uint64_t nf_query_log_us,last_nf_query_time;nf_check_code nf_logged_code;
    double last_nf_query_v,last_nf_query_w,last_nf_query_horizon;
    nav_reason flow_nf_reason;
    /* 每任务保存有界策略状态，不访问设备，不接管其他传感器。 */
    uint16_t s5_raw_mm[3];uint32_t s5_raw_seq[3],s5_raw_age_us[3],s5_raw_available,s5_raw_new_mask;
    uint64_t s5_raw_source_us[3],s5_diag_time,s5_policy_now;uint32_t s5_source_uptime_ms;
    uint32_t s5_emergency,s5_emergency_confirmed,s5_emergency_hits,s5_glitch_clears;
    uint32_t s5_sample_seen,s5_sequence_valid,s5_sample_new,s5_front_sequence,s5_raw_zero_mask;
    uint64_t s5_sample_time,s5_previous_sample_time,s5_emergency_since,s5_emergency_sample_time;
    uint32_t s5_side_state[2],s5_front_approach,s5_turn_hold,s5_hold_clear,s5_recover_clear,s5_resume_limited;
    uint32_t s5_brake_reselect,s5_after_backup,s5_force_corner,s5_allow_hazard_switch;
    uint64_t s5_hold_scan,s5_recover_scan,s5_corridor_scan,s5_opposite_since,s5_path_mismatch_since,s5_policy_log_time;
    double s5_front_clearance,s5_corridor_clearance,s5_side_clearance[2],s5_corridor_scores[2];
    int s5_opposite_direction;double s5_rejected_v,s5_rejected_w;char s5_rejected_reason[64];
    nav_xy s5_command_path[34];uint32_t s5_command_path_n;
    s3_avoidance_view s3_action;double s3_action_yaw,s3_action_last_yaw,s3_action_v,s3_action_w;
    uint64_t s3_action_pose_time,s3_exit_scan;uint32_t s3_exit_confirmations;
    uint64_t flow_hint_time,flow_direction_until,flow_blocked_since,flow_progress_time,flow_diag_time;
    uint16_t flow_sonar[3];uint8_t flow_valid,flow_fresh,flow_near,flow_degraded,flow_front_stable,flow_pending;
    int flow_have_hint,flow_have_progress,flow_pivot,flow_soft;nav_xy flow_progress_pose;double flow_progress_yaw;
    uint32_t flow_exec_ready,flow_exec_known,flow_wheel_reverse,flow_backup_counted,flow_backup_requests,flow_cruise,flow_local_probe,flow_can_start;
    uint64_t flow_backup_execution_us,flow_exhausted_scan;
    double flow_progress_goal,flow_backup_yaw,flow_backup_target;
    double flow_forward_progress;nav_xy flow_forward_origin;
    uint32_t flow_alignment_grace_used;
    uint32_t flow_final_yaw,flow_extended_attempts,flow_backup_extended;
    nav_reason flow_rear_reason;uint32_t flow_reject_sonar,flow_reject_occupied,flow_reject_unknown,flow_reject_budget,flow_accept,flow_provisional;
    double flow_observed_fraction;uint64_t flow_candidate_diag_us;
    struct {nav_xy p;double yaw;uint64_t expires;} flow_failed[8];unsigned flow_failed_next;
    uint64_t flow_spin_started,flow_spin_blocked[2],flow_stall_mark_us,flow_spin_last_us,flow_spin_spent_us[2];int flow_spin_sign;nav_xy flow_spin_origin,flow_spin_block_origin;
    uint64_t cost_scan_us,cost_candidates_us,cost_plan_us,cost_route_us,cost_narrow_us,cost_log_at;unsigned cost_candidate_count;
    uint32_t flow_load_reduced,flow_candidate_cursor;uint64_t flow_full_pool_us,query_deadline_us;
    /* 场景分类与状态机状态有界，由单一管理者持有。 */
    uint32_t r4_scene,r4_candidate_scene,r4_side_class,r4_calibrated,r4_side_low_hits[2];
    uint64_t r4_scene_candidate_since,r4_scene_good_since,r4_band_up_since,r4_explore_start,r4_explore_progress_time;
    uint64_t r4_side_low_since[2],r4_last_scan_seen;
    double r4_band_cap,r4_explore_side_deg[2],r4_explore_origin_x,r4_explore_origin_y;
    char r4_scene_reason[64];
    nav_xy motion_pose,backup_start,episode_origin;
    double motion_yaw;
    struct {nav_xy p;uint64_t expires;} obstacles[32];
    struct {nav_xy p;uint64_t expires;} failures[8];
    unsigned obstacle_n,failure_next;
    passage_local*local;passage_xy rectangle_path[PASSAGE_PATH_MAX];passage_result passage_collision;uint32_t narrow;double time_uncertainty_s;
#ifdef LAB_REAL_POSIX_SOURCE
    async_plan *async;uint64_t goal_generation;
#endif
};
static void s5_command_path(nav_engine*,uint64_t,double,double);
static double clamp(double x,double a,double b){return x<a?a:x>b?b:x;}
void nav_set_time_uncertainty(nav_engine*n,double seconds){if(n)n->time_uncertainty_s=isfinite(seconds)?clamp(seconds,0,1):1;}
static double wrap(double a){return remainder(a,2*PI);}
static double approach(double x,double target,double delta){return x+clamp(target-x,-delta,delta);}
static int context_equal(lab_context a,lab_context b){
    return a.mode==b.mode&&a.clock_domain==b.clock_domain&&a.timeline_id==b.timeline_id&&a.origin_epoch==b.origin_epoch;
}
static int context_event(const nav_engine *n,const lab_event *e){return e&&lab_context_matches(&n->context,&e->meta);}
static int pose_valid(const lab_event *e){
    if(!e||e->type!=LAB_EVENT_POSE||e->meta.provider==LAB_PROVIDER_TRUTH||!w1_real_carrier_event(e))return 0;
    return (e->meta.flags&(LAB_TIME_VALID|LAB_DATA_VALID))==(LAB_TIME_VALID|LAB_DATA_VALID)&&
      (e->data.pose.quality&LAB_POSE_VALID)&&!(e->data.pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))&&
      isfinite(e->data.pose.x_m)&&isfinite(e->data.pose.y_m)&&isfinite(e->data.pose.yaw_rad);
}
/* 反馈为独立数据源，实物配置使用自身较宽年龄上限。 */
static uint64_t nav_feedback_age_limit(const nav_engine*n){return n->object_time_profile?OBJECT_CONTROL_FEEDBACK_US:CONT_FEEDBACK_US;}
void nav_config_default(nav_config *c){
    if(!c)return;memset(c,0,sizeof(*c));
    c->vehicle_width_m=.36;c->vehicle_length_m=.42;c->safety_margin_m=.04;
    c->max_speed_m_s=.40;c->max_yaw_rate_rad_s=S2_MAX_YAW_RAD_S;c->acceleration_m_s2=.30;
    c->angular_acceleration_rad_s2=2.5;c->brake_m_s2=.45;
    c->lookahead_m=.35;c->goal_tolerance_m=.10;c->yaw_tolerance_rad=PI/18;
    c->max_pose_age_us=200000;c->max_scan_age_us=250000;c->command_valid_us=CONT_COMMAND_US;
    c->replan_interval_us=250000;c->wait_timeout_us=3000000;c->goal_timeout_us=180000000;
    c->occupancy_threshold=65;c->max_expansions=150000;c->require_scan=1;
    c->lidar_to_rear_m=.21;c->enable_recovery=1;
    passage_config_default(&c->passage);
}
static int config_valid(const nav_config *c){
    if(!c)return 0;
    if(c->rectangular_footprint>1||(c->rectangular_footprint&&(!passage_config_valid(&c->passage)||c->enable_recovery)))return 0;
    const double a[]={c->vehicle_width_m,c->vehicle_length_m,c->safety_margin_m,c->max_speed_m_s,
      c->max_yaw_rate_rad_s,c->acceleration_m_s2,c->angular_acceleration_rad_s2,c->brake_m_s2,
      c->lookahead_m,c->goal_tolerance_m,c->yaw_tolerance_rad,c->mount_x_m,c->mount_y_m,c->mount_yaw_rad,c->lidar_to_rear_m};
    for(size_t i=0;i<sizeof(a)/sizeof(a[0]);++i)if(!isfinite(a[i]))return 0;
    return c->vehicle_width_m>0&&c->vehicle_width_m<=2&&c->vehicle_length_m>0&&c->vehicle_length_m<=2&&
      c->safety_margin_m>=0&&c->safety_margin_m<=.5&&c->max_speed_m_s>0&&c->max_speed_m_s<=1&&
      c->max_yaw_rate_rad_s>0&&c->max_yaw_rate_rad_s<=3&&c->acceleration_m_s2>0&&c->acceleration_m_s2<=5&&
      c->angular_acceleration_rad_s2>0&&c->angular_acceleration_rad_s2<=10&&c->brake_m_s2>0&&c->brake_m_s2<=5&&
      c->lookahead_m>=.05&&c->lookahead_m<=2&&c->goal_tolerance_m>=.02&&c->goal_tolerance_m<=.5&&
      c->yaw_tolerance_rad>=.01&&c->yaw_tolerance_rad<=PI&&c->max_pose_age_us>0&&c->max_pose_age_us<=OBJECT_CONTROL_POSE_US&&
      c->max_scan_age_us>0&&c->max_scan_age_us<=OBJECT_CONTROL_SCAN_US&&c->command_valid_us>0&&c->command_valid_us<=500000&&
      c->replan_interval_us>0&&c->wait_timeout_us>=c->max_pose_age_us&&c->wait_timeout_us<=30000000&&
      c->goal_timeout_us>0&&c->goal_timeout_us<=600000000&&c->occupancy_threshold>0&&c->occupancy_threshold<=100&&
      c->max_expansions>0&&c->max_expansions<=NAV_GRID_MAX&&c->require_scan<=1&&
      fabs(c->mount_x_m)<=2&&fabs(c->mount_y_m)<=2&&c->enable_recovery<=1&&
      c->lidar_to_rear_m-c->mount_x_m>0&&c->lidar_to_rear_m-c->mount_x_m<c->vehicle_length_m;
}
lab_status nav_open(nav_engine **out,const nav_config *c,const lab_context *ctx){
    if(!out)return LAB_ERR_ARGUMENT;*out=NULL;
    if(!config_valid(c)||!ctx||!ctx->timeline_id||!ctx->origin_epoch||!w1_real_carrier_context(ctx))return LAB_ERR_ARGUMENT;
    nav_engine *n=calloc(1,sizeof(*n));if(!n)return LAB_ERR_NOMEM;
    n->s5_front_clearance=n->s5_corridor_clearance=n->s5_side_clearance[0]=n->s5_side_clearance[1]=-1;
    n->c=*c;n->context=*ctx;nf_init(&n->nearfield,&c->passage);n->nearfield.pose_max_age_us=c->max_pose_age_us;n->nearfield.scan_max_age_us=c->max_scan_age_us;
    double rear=c->lidar_to_rear_m-c->mount_x_m;
    n->radius=hypot(c->vehicle_width_m/2,fmax(rear,c->vehicle_length_m-rear))+c->safety_margin_m;
    n->rear_clearance=-1;
    if(c->rectangular_footprint){n->local=calloc(1,sizeof(*n->local));n->s6_cm=calloc(1,sizeof(*n->s6_cm));n->s6_plan=calloc(1,sizeof(*n->s6_plan));if(!n->local||!n->s6_cm||!n->s6_plan){free(n->local);free(n->s6_cm);free(n->s6_plan);free(n);return LAB_ERR_NOMEM;}n->allocated+=sizeof(*n->local)+sizeof(*n->s6_cm)+sizeof(*n->s6_plan);}
#ifdef NAV_S6_DISABLE
    n->s6_disabled=1;
#endif
    n->state=NAV_IDLE;n->reason=NAV_NO_GOAL;n->allocated+=sizeof(*n);*out=n;return LAB_OK;
}
static uint64_t nav_geometry_clock(void){
#ifdef LAB_NAV_TEST_CLOCK
 extern uint64_t lab_nav_test_clock(void);return lab_nav_test_clock();
#elif defined(LAB_REAL_POSIX_SOURCE)
 struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t)==0)return (uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;
#endif
 return 0;
}
static int nav_geometry_cancel(void*u){nav_engine*n=u;return n->query_deadline_us&&nav_geometry_clock()>n->query_deadline_us;}
static passage_world rectangle_world(const nav_engine*n,passage_pose initial){
    passage_world w={.local_only=n->external_active&&!n->n,.config=n->c.passage,.global={n->w,n->h,n->res,n->ox,n->oy,n->cells},.local=n->local,.now_us=n->last_tick,.initial=initial,.allow_initial_unknown=1,.confirmed_only=n->c.passage.continuous!=0};if(n->external_active&&n->object_grid_source&&n->local)w.global=(passage_grid){PASSAGE_LOCAL_SIDE,PASSAGE_LOCAL_SIDE,n->local->resolution,n->local->origin_x+n->local->map_from_local_x,n->local->origin_y+n->local->map_from_local_y,n->object_grid};if(n->query_deadline_us){w.cancelled=nav_geometry_cancel;w.cancel_user=(void*)n;}return w;
}
static nav_xy reframe_xy(nav_xy p,double dx,double dy,double a){return (nav_xy){dx+cos(a)*p.x_m-sin(a)*p.y_m,dy+sin(a)*p.x_m+cos(a)*p.y_m};}
void nav_reframe_local(nav_engine*n,double dx,double dy,double yaw){
 if(!n||!isfinite(dx)||!isfinite(dy)||!isfinite(yaw))return;
 n->s6_have_profile=0;n->s6_cm_source=0;
 if(n->local)passage_local_reframe(n->local,&n->c.passage,dx,dy,yaw);
 /* 地图与里程计校正不是轮行程，同步变换短期基准，避免把定位跳变当作后退或脱困成功；显式世界目标和路线保持不动。

 */
 if(n->r4_initialized){
  nav_xy q=reframe_xy((nav_xy){n->r4_state.phase_x,n->r4_state.phase_y},dx,dy,yaw);n->r4_state.phase_x=q.x_m;n->r4_state.phase_y=q.y_m;
  q=reframe_xy((nav_xy){n->r4_state.last_x,n->r4_state.last_y},dx,dy,yaw);n->r4_state.last_x=q.x_m;n->r4_state.last_y=q.y_m;
  q=reframe_xy((nav_xy){n->r4_state.explore_x,n->r4_state.explore_y},dx,dy,yaw);n->r4_state.explore_x=q.x_m;n->r4_state.explore_y=q.y_m;
  q=reframe_xy((nav_xy){n->r4_state.doorway_x,n->r4_state.doorway_y},dx,dy,yaw);n->r4_state.doorway_x=q.x_m;n->r4_state.doorway_y=q.y_m;
  n->r4_state.phase_yaw=wrap(n->r4_state.phase_yaw+yaw);n->r4_state.last_yaw=wrap(n->r4_state.last_yaw+yaw);
 }
 if(n->r8_observe.active){nav_xy q=reframe_xy((nav_xy){n->r8_observe.x,n->r8_observe.y},dx,dy,yaw);n->r8_observe.x=q.x_m;n->r8_observe.y=q.y_m;n->r8_observe.yaw=wrap(n->r8_observe.yaw+yaw);n->r8_observe.last_yaw=wrap(n->r8_observe.last_yaw+yaw);}
 n->motion_pose=reframe_xy(n->motion_pose,dx,dy,yaw);n->motion_yaw=wrap(n->motion_yaw+yaw);
 n->s3_action_yaw=wrap(n->s3_action_yaw+yaw);n->s3_action_last_yaw=wrap(n->s3_action_last_yaw+yaw);
 n->backup_start=reframe_xy(n->backup_start,dx,dy,yaw);n->flow_backup_yaw=wrap(n->flow_backup_yaw+yaw);
 n->flow_progress_pose=reframe_xy(n->flow_progress_pose,dx,dy,yaw);n->flow_progress_yaw=wrap(n->flow_progress_yaw+yaw);
 n->flow_progress_goal=hypot(n->goal.x_m-n->flow_progress_pose.x_m,n->goal.y_m-n->flow_progress_pose.y_m);
 n->episode_origin=reframe_xy(n->episode_origin,dx,dy,yaw);n->flow_forward_origin=reframe_xy(n->flow_forward_origin,dx,dy,yaw);
 n->flow_spin_origin=reframe_xy(n->flow_spin_origin,dx,dy,yaw);n->flow_spin_block_origin=reframe_xy(n->flow_spin_block_origin,dx,dy,yaw);
 for(unsigned i=0;i<8;i++)if(n->flow_failed[i].expires){n->flow_failed[i].p=reframe_xy(n->flow_failed[i].p,dx,dy,yaw);n->flow_failed[i].yaw=wrap(n->flow_failed[i].yaw+yaw);}
}
void nav_set_cruise(nav_engine*n,uint32_t enabled){if(n)n->flow_cruise=enabled!=0;}
void nav_observe_nearfield(nav_engine*n,const lab_event*e,uint64_t now){if(!n||!e)return;n->nf_external=1;n->nf_now=now;nf_observe(&n->nearfield,e,now);}
/* 预测时域等效于完成一次受控停车：反应覆盖原命令租约、传感器年龄及50ms调度余量，恒速下半停车时间覆盖实际制动位移或角度，不任意缩短预测时间。


 */
static double nav_progressive_horizon(const nav_engine*n,uint64_t now,double v,double w){
 double scan_age=n->scan_time&&n->scan_time<=now?(now-n->scan_time)/1e6:0;
 double pose_age=n->motion_time&&n->motion_time<=now?(now-n->motion_time)/1e6:0;
 if(n->c.passage.r4_enabled){
  unsigned mask=v<0?14u:1u;
  if(fabs(w)>.001){int same_forward=v>0&&v>=fabs(w)*.145;mask|=same_forward?(w>0?2u:4u):6u;}
  for(unsigned side=0;side<4;side++){uint64_t source=n->r4_input.direction[side].source_us;if((mask&(1u<<side))&&source&&source<=now)scan_age=fmax(scan_age,(now-source)/1e6);}
 }
 double brake=fmax(fabs(v)/(2*fmax(.01,n->c.brake_m_s2)),fabs(w)/(2*fmax(.01,n->c.angular_acceleration_rad_s2)));
 return fmax(.20,n->c.command_valid_us/1e6)+fmax(scan_age,pose_age)+n->time_uncertainty_s+.05+brake;
}
static nav_reason nav_nf_reason(nf_check_code code){
 switch(code){case NF_CLEAR:return NAV_REASON_NONE;case NF_COLLISION:return NAV_OBSTACLE;case NF_NEED_OBSERVATION:return NAV_NEARFIELD_OBSERVATION;case NF_DATA_STALE:return NAV_NEARFIELD_STALE;case NF_POSE_INVALID:return NAV_NEARFIELD_POSE;default:return NAV_NEARFIELD_INVALID;}
}
static nf_check_result nav_nearfield_check(nav_engine*n,uint64_t now,double v,double w,double horizon){
 nf_check_result result={0};if(!n){result.code=NF_INVALID_ACTION;return result;}
 if(n->c.passage.continuous&&n->c.require_scan)result=nf_check_current(&n->nearfield,now,v,w);
 n->last_nf_check=result;n->last_nf_query_time=now;n->last_nf_query_v=v;n->last_nf_query_w=w;n->last_nf_query_horizon=horizon;
 if(result.code!=NF_CLEAR&&(!n->nf_query_log_us||(now>=n->nf_query_log_us&&now-n->nf_query_log_us>=250000))){
  n->nf_query_log_us=now;n->nf_logged_code=result.code;
  fprintf(stderr,"NEARFIELD_QUERY policy=CURRENT_MEASURED prediction=ADVISORY_ONLY 当前实测防护_预测仅建议 now_us=%llu result=%s source=%u source_us=%llu x=%.4f y=%.4f ttc=%.3f unknown=%u v=%.4f w=%.4f horizon=%.3f\n",(unsigned long long)now,nf_check_reason(result.code),result.source_kind,(unsigned long long)result.source_time,result.first_collision_x,result.first_collision_y,result.ttc,result.unknown_cells,v,w,horizon);
 }
 return result;
}
void nav_nearfield_view(const nav_engine*n,uint64_t now,s3_nearfield_view*out){
 if(!out)return;memset(out,0,sizeof(*out));out->front_m=out->left_m=out->right_m=out->rear_m=-1;if(!n)return;
 nf_view(&n->nearfield,now,out);out->query_code=n->last_nf_check.code;out->query_source=n->last_nf_check.source_kind;out->query_source_time_us=n->last_nf_check.source_time;
 out->query_x_m=n->last_nf_check.first_collision_x;out->query_y_m=n->last_nf_check.first_collision_y;out->query_ttc_s=n->last_nf_check.ttc;
 out->query_time_us=n->last_nf_query_time;out->query_v_m_s=n->last_nf_query_v;out->query_w_rad_s=n->last_nf_query_w;out->query_horizon_s=n->last_nf_query_horizon;
 snprintf(out->query_reason,sizeof(out->query_reason),"%s",nf_check_reason(n->last_nf_check.code));
}
int nav_nearfield_allows(nav_engine*n,uint64_t now,double v,double w){return n&&nav_nearfield_check(n,now,v,w,nav_progressive_horizon(n,now,v,w)).code==NF_CLEAR;}
void nav_set_execution_hint(nav_engine*n,uint32_t armed){if(n){n->flow_exec_known=1;n->flow_exec_ready=armed!=0;}}
void nav_set_execution_feedback(nav_engine*n,uint32_t armed,const double rpm[4]){
 nav_set_execution_hint(n,armed);if(!n)return;n->flow_wheel_reverse=0;
 if(armed&&rpm)for(unsigned i=0;i<4;i++)if(isfinite(rpm[i])&&rpm[i]<-1.0)n->flow_wheel_reverse=1;
}
lab_status nav_resume_after_pause(nav_engine*n,uint64_t now){
 if(!n||!now)return LAB_ERR_ARGUMENT;if(n->have_tick&&now<n->last_tick)return LAB_ERR_TIME;
 /* 同上下文的正向时间恢复不新建目标或脱困过程；旧输出过期，仅重启tick积分和进展采样，保留源年龄、实际后退距离时间、尝试预算及提示。

 */
 n->have_tick=0;n->waiting=0;n->flow_have_progress=0;n->cmd.linear_m_s=n->cmd.angular_rad_s=0;n->cmd.time_us=n->cmd.expires_us=now;
 return LAB_OK;
}
static void free_grid(nav_engine *n){
    free(n->cells);free(n->blocked);free(n->closed);free(n->dist);free(n->cost);free(n->parent);free(n->heap_pos);free(n->heap);
    n->cells=NULL;n->blocked=NULL;n->closed=NULL;n->dist=NULL;n->cost=NULL;n->parent=NULL;n->heap_pos=NULL;n->heap=NULL;
}
static int grid_allocate(nav_engine *n,uint32_t count){
    n->cells=malloc(count);n->blocked=malloc(count);n->closed=malloc(count);
    n->dist=malloc(count*sizeof(uint16_t));n->cost=malloc(count*sizeof(double));
    n->parent=malloc(count*sizeof(int32_t));n->heap_pos=malloc(count*sizeof(int32_t));n->heap=malloc(count*sizeof(int32_t));
    return n->cells&&n->blocked&&n->closed&&n->dist&&n->cost&&n->parent&&n->heap_pos&&n->heap;
}
/* 保守正方形切比雪夫膨胀，工作量O(格数)，内存有界；加入栅格对角余量，避免量化缩小圆形车体包络。
 */
static void inflate(nav_engine *n){
    if(n->c.rectangular_footprint){for(unsigned k=0;k<n->n;k++){n->blocked[k]=(uint8_t)(n->cells[k]<0||n->cells[k]>=(int)n->c.occupancy_threshold);n->dist[k]=n->blocked[k]?0:1;}return;}
    const unsigned inf=32760;unsigned w=n->w,h=n->h;
    for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){
        unsigned k=y*w+x;int v=n->cells[k];
        unsigned edge=x+1;if(w-x<edge)edge=w-x;if(y+1<edge)edge=y+1;if(h-y<edge)edge=h-y;
        n->dist[k]=(uint16_t)((v<0||v>=(int)n->c.occupancy_threshold)?0:(edge<inf?edge:inf));
    }
    for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){
        unsigned k=y*w+x,d=n->dist[k];
        if(x&&n->dist[k-1]+1u<d)d=n->dist[k-1]+1u;
        if(y){for(int dx=-1;dx<=1;++dx){int xx=(int)x+dx;if(xx>=0&&xx<(int)w&&n->dist[k-w+dx]+1u<d)d=n->dist[k-w+dx]+1u;}}
        n->dist[k]=(uint16_t)d;
    }
    for(int y=(int)h-1;y>=0;--y)for(int x=(int)w-1;x>=0;--x){
        unsigned k=(unsigned)y*w+(unsigned)x,d=n->dist[k];
        if(x+1<(int)w&&n->dist[k+1]+1u<d)d=n->dist[k+1]+1u;
        if(y+1<(int)h){for(int dx=-1;dx<=1;++dx){int xx=x+dx;if(xx>=0&&xx<(int)w&&n->dist[k+w+dx]+1u<d)d=n->dist[k+w+dx]+1u;}}
        n->dist[k]=(uint16_t)d;
    }
    unsigned cells=(unsigned)ceil(n->radius/n->res+.7071067811865476);
    for(unsigned k=0;k<n->n;++k)n->blocked[k]=(uint8_t)(n->c.rectangular_footprint?(n->cells[k]<0||n->cells[k]>=(int)n->c.occupancy_threshold):n->dist[k]<=cells);
}
lab_status nav_set_map(nav_engine *n,const lab_map_view *m,uint64_t rev){
    if(!n||!m||!m->cells||!rev)return LAB_ERR_ARGUMENT;
    if(!context_equal(n->context,m->context))return LAB_ERR_CONTEXT;
    if(rev<=n->revision)return LAB_ERR_TIME;
    if(!m->width||!m->height||m->width>NAV_SIDE_MAX||m->height>NAV_SIDE_MAX||(uint64_t)m->width*m->height>NAV_GRID_MAX)return LAB_ERR_LIMIT;
    if(!isfinite(m->resolution_m)||m->resolution_m<.01||m->resolution_m>1||!isfinite(m->origin_x_m)||
       !isfinite(m->origin_y_m)||fabs(m->origin_x_m)>1000000||fabs(m->origin_y_m)>1000000)return LAB_ERR_ARGUMENT;
    uint32_t count=m->width*m->height;for(uint32_t k=0;k<count;++k)if(m->cells[k]<-1||m->cells[k]>100)return LAB_ERR_FORMAT;
    if(n->n!=count){
        nav_engine tmp={0};if(!grid_allocate(&tmp,count)){free_grid(&tmp);return LAB_ERR_NOMEM;}
        free_grid(n);n->cells=tmp.cells;n->blocked=tmp.blocked;n->closed=tmp.closed;n->dist=tmp.dist;
        n->cost=tmp.cost;n->parent=tmp.parent;n->heap_pos=tmp.heap_pos;n->heap=tmp.heap;
    }
    n->n=count;n->w=m->width;n->h=m->height;n->res=m->resolution_m;n->ox=m->origin_x_m;n->oy=m->origin_y_m;
    memcpy(n->cells,m->cells,count);inflate(n);n->revision=rev;
    if(n->c.rectangular_footprint&&n->path_n>=2)n->dirty=2;else{n->dirty=1;n->path_n=0;n->segment=0;}
    if(n->have_goal&&n->state==NAV_UNREACHABLE){n->state=NAV_WAITING;n->reason=NAV_MAP_CHANGED;n->waiting=0;}
    n->allocated=sizeof(*n)+(size_t)count*(3+sizeof(uint16_t)+sizeof(double)+3*sizeof(int32_t))+(n->local?sizeof(*n->local):0)+(n->s6_cm?sizeof(*n->s6_cm):0)+(n->s6_plan?sizeof(*n->s6_plan):0);
    return LAB_OK;
}
static int index_at(const nav_engine *n,double x,double y){
    if(!isfinite(x)||!isfinite(y)||x<n->ox||y<n->oy||x>=n->ox+n->w*n->res||y>=n->oy+n->h*n->res)return -1;
    return (int)((unsigned)floor((y-n->oy)/n->res)*n->w+(unsigned)floor((x-n->ox)/n->res));
}
static int free_at(const nav_engine *n,double x,double y){int k=index_at(n,x,y);return k>=0&&!n->blocked[k];}
static nav_xy cell_xy(const nav_engine *n,int k){nav_xy p={n->ox+(k%(int)n->w+.5)*n->res,n->oy+(k/(int)n->w+.5)*n->res};return p;}
static int line_clear(const nav_engine *n,nav_xy a,nav_xy b){
    unsigned steps=(unsigned)ceil(hypot(b.x_m-a.x_m,b.y_m-a.y_m)/(n->res*.3))+1;
    for(unsigned i=0;i<=steps;++i){double t=(double)i/steps;if(!free_at(n,a.x_m+t*(b.x_m-a.x_m),a.y_m+t*(b.y_m-a.y_m)))return 0;}
    return 1;
}
lab_status nav_set_goal(nav_engine *n,const nav_goal *g,uint64_t now){
    if(!n||!g||!g->id||g->has_yaw>1||!isfinite(g->x_m)||!isfinite(g->y_m)||!isfinite(g->yaw_rad))return LAB_ERR_ARGUMENT;
    n->s6_have_profile=n->s6_active=0;
    if(n->have_tick&&now<n->last_tick)return LAB_ERR_TIME;
#ifdef LAB_REAL_POSIX_SOURCE
    ++n->goal_generation;if(n->async)atomic_store(&n->async->cancelled,1);
#endif
    memset(&n->r8_observe,0,sizeof(n->r8_observe));
    n->goal=*g;n->goal.yaw_rad=wrap(g->yaw_rad);n->goal_started=now;n->have_goal=1;n->dirty=1;n->path_n=0;memset(&n->route_report,0,sizeof(n->route_report));
    r4_scene_init(&n->r4_state);memset(&n->r4_output,0,sizeof(n->r4_output));n->r4_initialized=0;n->r4_active=n->r4_checking=0;
    memset(&n->s3_action,0,sizeof(n->s3_action));n->s3_action_pose_time=0;n->s3_exit_confirmations=0;n->s5_turn_hold=n->s5_after_backup=n->s5_resume_limited=n->s5_brake_reselect=n->s5_force_corner=0;n->s5_command_path_n=0;
    n->phase=NAV_PHASE_TRACK;n->episode=0;n->flow_extended_attempts=n->flow_backup_extended=0;n->reverse_attempts=0;n->backup_total=0;n->backup_progress=0;
    n->turn_total=0;n->clear_scans=0;n->turn_dir=0;n->flow_blocked_since=n->flow_direction_until=0;n->flow_have_progress=n->flow_pivot=0;n->flow_can_start=n->flow_backup_counted=n->flow_backup_requests=n->flow_local_probe=n->flow_alignment_grace_used=0;n->flow_spin_started=n->flow_stall_mark_us=0;n->flow_spin_sign=0;memset(n->flow_spin_blocked,0,sizeof(n->flow_spin_blocked));memset(n->failures,0,sizeof(n->failures));memset(n->flow_failed,0,sizeof(n->flow_failed));
    n->flow_spin_last_us=now;memset(n->flow_spin_spent_us,0,sizeof(n->flow_spin_spent_us));n->state=NAV_WAITING;n->reason=NAV_MAP_CHANGED;n->blocking_reason=NAV_REASON_NONE;n->waiting=0;n->cmd.linear_m_s=0;n->cmd.angular_rad_s=0;return LAB_OK;
}
void nav_cancel(nav_engine *n,uint64_t now){
    if(!n)return;n->external_active=0;memset(&n->object_target,0,sizeof(n->object_target));memset(&n->r8_observe,0,sizeof(n->r8_observe));n->s6_have_profile=n->s6_active=0;n->have_goal=0;n->path_n=0;memset(&n->route_report,0,sizeof(n->route_report));n->waiting=0;n->state=NAV_CANCELLED;n->reason=NAV_USER_CANCEL;
    r4_scene_init(&n->r4_state);memset(&n->r4_output,0,sizeof(n->r4_output));n->r4_initialized=0;n->r4_active=n->r4_checking=0;
    if(n->s3_action.active)++n->s3_action.interrupted_actions;n->s3_action.active=0;n->s3_action.stage=0;snprintf(n->s3_action.reason,sizeof(n->s3_action.reason),"USER_CANCEL");
#ifdef LAB_REAL_POSIX_SOURCE
    ++n->goal_generation;if(n->async)atomic_store(&n->async->cancelled,1);
#endif
    n->phase=NAV_PHASE_TRACK;n->episode=0;n->flow_extended_attempts=n->flow_backup_extended=0;n->clear_scans=0;n->reverse_attempts=0;n->backup_progress=n->backup_total=0;
    n->flow_backup_counted=n->flow_backup_requests=n->flow_can_start=n->flow_local_probe=0;n->flow_have_progress=0;
    memset(n->flow_failed,0,sizeof(n->flow_failed));
    n->cmd.linear_m_s=0;n->cmd.angular_rad_s=0;n->cmd.time_us=now;n->cmd.expires_us=now;n->cmd.context=n->context;
}
static double heuristic(const nav_engine *n,int a,int b){
    double dx=abs(a%(int)n->w-b%(int)n->w),dy=abs(a/(int)n->w-b/(int)n->w);
    return fmax(dx,dy)+.4142135623730951*fmin(dx,dy);
}
static int heap_less(const nav_engine *n,int a,int b,int goal){
    double fa=n->cost[a]+heuristic(n,a,goal),fb=n->cost[b]+heuristic(n,b,goal);
    return fa<fb-1e-12||(fabs(fa-fb)<=1e-12&&a<b);
}
static void heap_swap(nav_engine *n,unsigned a,unsigned b){
    int t=n->heap[a];n->heap[a]=n->heap[b];n->heap[b]=t;n->heap_pos[n->heap[a]]=(int)a;n->heap_pos[n->heap[b]]=(int)b;
}
static void heap_up(nav_engine *n,unsigned p,int goal){while(p){unsigned q=(p-1)/2;if(!heap_less(n,n->heap[p],n->heap[q],goal))break;heap_swap(n,p,q);p=q;}}
static void heap_add(nav_engine *n,int k,int goal){
    if(n->heap_pos[k]>=0){heap_up(n,(unsigned)n->heap_pos[k],goal);return;}
    unsigned p=n->heap_n++;n->heap[p]=k;n->heap_pos[k]=(int)p;heap_up(n,p,goal);
}
static int heap_pop(nav_engine *n,int goal){
    int k=n->heap[0];n->heap_pos[k]=-1;--n->heap_n;if(!n->heap_n)return k;
    n->heap[0]=n->heap[n->heap_n];n->heap_pos[n->heap[0]]=0;unsigned p=0;
    for(;;){unsigned q=p*2+1;if(q>=n->heap_n)break;if(q+1<n->heap_n&&heap_less(n,n->heap[q+1],n->heap[q],goal))++q;
        if(!heap_less(n,n->heap[q],n->heap[p],goal))break;heap_swap(n,p,q);p=q;}
    return k;
}
#ifdef LAB_REAL_POSIX_SOURCE
static int rectangle_route_clear(nav_engine*n,const lab_pose*pose);
static int search_cancelled(void*u){return atomic_load(&((async_plan*)u)->cancelled)!=0;}
static uint64_t plan_clock(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;return (uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;}
static void *plan_worker(void *arg){
 async_plan *a=arg;uint64_t t=plan_clock();
 int result=passage_plan_ex(&a->world,a->start,a->goal,a->has_yaw,a->goal_yaw,a->path,PASSAGE_PATH_MAX,&a->count,&a->expanded,&a->report);
 uint64_t end=plan_clock();pthread_mutex_lock(&a->mutex);a->result=result;a->elapsed_us=end>=t?end-t:0;a->done=1;int orphaned=a->orphaned;pthread_mutex_unlock(&a->mutex);
 if(orphaned){free(a->cells);free(a->local);pthread_mutex_destroy(&a->mutex);free(a);}return NULL;
}
/* 只处理地图副本，结果归原地图和目标；地图、目标或车辆起点变化后丢弃结果。
 */
static nav_reason plan_async(nav_engine *n,passage_pose p){
 async_plan *a=n->async;
 if(!a){a=calloc(1,sizeof(*a));if(!a||pthread_mutex_init(&a->mutex,NULL)){free(a);return NAV_SEARCH_LIMIT;}n->async=a;}
 if(a->active){
  pthread_mutex_lock(&a->mutex);int done=a->done;pthread_mutex_unlock(&a->mutex);if(!done)return NAV_REPLANNING;
  pthread_join(a->thread,NULL);a->active=0;
  fprintf(stderr,"NAV_ASYNC_PLAN elapsed_us=%llu expanded=%u result=%s points=%u revision=%llu start=%.3f,%.3f,%.2f goal=%.3f,%.3f goal_yaw=%d,%.2f route_kind=%u complete=%u partial=%u budget=%u corridor_expanded=%u pose_expanded=%u\n",(unsigned long long)a->elapsed_us,a->expanded,passage_reason_text(a->result),a->count,(unsigned long long)a->revision,a->start.x,a->start.y,a->start.yaw,a->goal.x,a->goal.y,a->has_yaw,a->goal_yaw,a->report.kind,a->report.complete,a->report.partial,a->report.budget,a->report.corridor_expanded,a->report.pose_expanded);
  n->expanded+=a->expanded;
  int stale=a->generation!=n->goal_generation;
  if(!stale&&(a->result==PASSAGE_CLEAR||a->result==PASSAGE_PARTIAL)){
   passage_world current=rectangle_world(n,p);current.planning_unknown_soft=0;passage_result hit;passage_pose q=p;n->flow_provisional=0;double checked_length=0;
   for(unsigned i=1;i<a->count&&!stale;i++){double angle=atan2(a->path[i].y-q.y,a->path[i].x-q.x);
    if(!passage_rotate_clear(&current,q,angle,&hit)){stale=1;break;}q.yaw=angle;
    passage_xy end=a->path[i];double length=hypot(end.x-q.x,end.y-q.y);if(n->c.passage.continuous&&checked_length+length>.5){double ratio=(.5-checked_length)/fmax(length,1e-9);end=(passage_xy){q.x+ratio*(end.x-q.x),q.y+ratio*(end.y-q.y)};}
    if(!passage_translate_clear(&current,q,end,n->c.passage.width_m,&hit)){stale=1;break;}if(hit.unknown_cells)n->flow_provisional=1;checked_length+=hypot(end.x-q.x,end.y-q.y);q.x=end.x;q.y=end.y;if(n->c.passage.continuous&&checked_length>=.499){n->flow_provisional=1;break;}
   }
   if(!stale&&a->has_yaw&&hypot(q.x-a->goal.x,q.y-a->goal.y)<1e-5&&!passage_rotate_clear(&current,q,a->goal_yaw,&hit))stale=1;
   if(!stale)a->path[0]=(passage_xy){p.x,p.y};
  }else if(a->revision!=n->revision)stale=1;
  if(!stale){n->passage_collision.reason=a->result;if(a->result==PASSAGE_CLEAR||a->result==PASSAGE_PARTIAL){n->route_report=a->report;for(unsigned i=0;i<a->count;i++)n->path[i]=(nav_xy){a->path[i].x,a->path[i].y};n->path_n=a->count;n->segment=0;free(a->cells);free(a->local);a->cells=NULL;a->local=NULL;return NAV_REASON_NONE;}}
  free(a->cells);free(a->local);a->cells=NULL;a->local=NULL;
  if(!stale){n->route_report=a->report;return a->result==PASSAGE_SEARCH_LIMIT?NAV_SEARCH_LIMIT:NAV_NO_PATH;}
 }
 a->cells=malloc(n->n);a->local=malloc(sizeof(*a->local));if(!a->cells||!a->local){free(a->cells);free(a->local);a->cells=NULL;a->local=NULL;return NAV_SEARCH_LIMIT;}
 memcpy(a->cells,n->cells,n->n);memcpy(a->local,n->local,sizeof(*a->local));a->world=rectangle_world(n,p);a->world.global.cells=a->cells;a->world.local=a->local;
 a->world.planning_unknown_soft=n->c.passage.continuous!=0;
 a->start=p;a->goal=(passage_xy){n->goal.x_m,n->goal.y_m};a->has_yaw=(int)n->goal.has_yaw;a->goal_yaw=n->goal.yaw_rad;a->revision=n->revision;a->generation=n->goal_generation;a->count=a->expanded=0;a->done=0;
 atomic_store(&a->cancelled,0);a->world.cancelled=search_cancelled;a->world.cancel_user=a;
 if(pthread_create(&a->thread,NULL,plan_worker,a)){free(a->cells);free(a->local);a->cells=NULL;a->local=NULL;return NAV_SEARCH_LIMIT;}
 a->active=1;++n->plans;if(!n->c.passage.continuous){n->path_n=0;n->segment=0;}return NAV_REPLANNING;
}
#endif
static nav_reason object_adapter_plan(nav_engine*,nav_xy);
static nav_reason plan(nav_engine *n,nav_xy start){
    if(n->external_active)return object_adapter_plan(n,start);
    if(n->c.rectangular_footprint){
#ifdef LAB_REAL_POSIX_SOURCE
        return plan_async(n,(passage_pose){start.x_m,start.y_m,n->motion_yaw});
#else
        passage_pose p={start.x_m,start.y_m,n->motion_yaw};passage_world world=rectangle_world(n,p);unsigned count=0,expanded=0;
        world.planning_unknown_soft=n->c.passage.continuous!=0;
        ++n->plans;n->path_n=0;n->segment=0;int r=passage_plan_ex(&world,p,(passage_xy){n->goal.x_m,n->goal.y_m},(int)n->goal.has_yaw,n->goal.yaw_rad,n->rectangle_path,PASSAGE_PATH_MAX,&count,&expanded,&n->route_report);n->expanded+=expanded;n->passage_collision.reason=r;
        if(r!=PASSAGE_CLEAR&&r!=PASSAGE_PARTIAL){passage_result hit;if(!passage_pose_clear(&world,p,n->c.passage.width_m,&hit)){n->passage_collision=hit;return NAV_START_BLOCKED;}return r==PASSAGE_SEARCH_LIMIT?NAV_SEARCH_LIMIT:NAV_NO_PATH;}
        n->flow_provisional=0;passage_pose q=p;for(unsigned i=0;i<count;i++){n->path[i]=(nav_xy){n->rectangle_path[i].x,n->rectangle_path[i].y};if(i){passage_result h;q.yaw=atan2(n->rectangle_path[i].y-q.y,n->rectangle_path[i].x-q.x);if(passage_translate_clear(&world,q,n->rectangle_path[i],world.config.width_m,&h)&&h.unknown_cells)n->flow_provisional=1;q.x=n->rectangle_path[i].x;q.y=n->rectangle_path[i].y;}}n->path_n=count;return NAV_REASON_NONE;
#endif
    }
    n->path_n=0;n->segment=0;++n->plans;int s=index_at(n,start.x_m,start.y_m),g=index_at(n,n->goal.x_m,n->goal.y_m);
    if(g<0)return NAV_GOAL_OUTSIDE;if(n->blocked[g])return NAV_GOAL_BLOCKED;if(s<0||(n->blocked[s]&1))return NAV_START_BLOCKED;
    for(unsigned i=0;i<n->n;++i){n->cost[i]=DBL_MAX;n->parent[i]=-1;n->heap_pos[i]=-1;n->closed[i]=0;}
    n->heap_n=0;n->cost[s]=0;heap_add(n,s,g);unsigned expanded=0;int found=0;
    while(n->heap_n){
        int k=heap_pop(n,g);if(k==g){found=1;break;}n->closed[k]=1;
        if(++expanded>n->c.max_expansions){n->expanded+=expanded;return NAV_SEARCH_LIMIT;}
        int x=k%(int)n->w,y=k/(int)n->w;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
            int xx=x+dx,yy=y+dy;if((!dx&&!dy)||xx<0||yy<0||xx>=(int)n->w||yy>=(int)n->h)continue;
            int j=yy*(int)n->w+xx;if(n->blocked[j]||n->closed[j])continue;
            if(dx&&dy&&(n->blocked[y*(int)n->w+xx]||n->blocked[yy*(int)n->w+x]))continue;
            double step=dx&&dy?1.4142135623730951:1,extra=0;
            if(n->c.enable_recovery){
                double pad=ceil(n->radius/n->res+.7071067811865476);
                extra=1.5/fmax(1,n->dist[j]-pad);
                nav_xy at=cell_xy(n,j);
                for(unsigned z=0;z<8;++z)if(n->failures[z].expires>n->last_tick){
                    double d=hypot(at.x_m-n->failures[z].p.x_m,at.y_m-n->failures[z].p.y_m);
                    if(d<.45)extra+=3*(1-d/.45);
                }
            }
            double cost=n->cost[k]+step*(1+extra);
            if(cost<n->cost[j]){n->cost[j]=cost;n->parent[j]=k;heap_add(n,j,g);}
        }
    }
    n->expanded+=expanded;if(!found)return NAV_NO_PATH;
    unsigned count=0;for(int k=g;k>=0;k=n->parent[k]){if(count>=NAV_PATH_MAX-2)return NAV_PATH_LIMIT;n->raw[count++]=cell_xy(n,k);if(k==s)break;}
    for(unsigned i=0;i<count/2;++i){nav_xy t=n->raw[i];n->raw[i]=n->raw[count-1-i];n->raw[count-1-i]=t;}
    n->raw[0]=start;n->raw[count-1]=(nav_xy){n->goal.x_m,n->goal.y_m};
    if(count==1){n->path[0]=start;n->path[1]=n->raw[0];n->path_n=2;return NAV_REASON_NONE;}
    n->path[n->path_n++]=start;unsigned anchor=0;
    while(anchor<count-1){
        unsigned last=anchor+1;for(unsigned j=anchor+2;j<count;++j){if(!line_clear(n,n->raw[anchor],n->raw[j]))break;last=j;}
        n->path[n->path_n++]=n->raw[last];anchor=last;
    }
    return NAV_REASON_NONE;
}
void nav_invalidate_command(nav_engine*n){if(n){n->cmd.linear_m_s=n->cmd.angular_rad_s=0;n->cmd.time_us=n->cmd.expires_us=0;}}
static void command(nav_engine *n,uint64_t now,double v,double w,nav_command *out){
    /* 路线、场景或规划器成功命令决定下一段，只有观察发送器可保留活动观察基准；零命令等待或预算不足保留已实际执行的观察事实。

 */
    if((fabs(v)>1e-9||fabs(w)>1e-9)&&!n->r8_observe.issuing)n->r8_observe.active=0;
    if(n->r4_active&&n->have_goal&&(!n->r6_route_log_us||now<n->r6_route_log_us||now-n->r6_route_log_us>=500000||w*n->cmd.angular_rad_s<-.0004)){
     fprintf(stderr,"R6_ROUTE_ACTION policy_revision=9 cfg=%08x now_us=%llu expected_v=%.4f expected_w=%.4f actual_v=%.4f actual_w=%.4f expected_side=%d actual_side=%d front_m=%.4f cap_m_s=%.4f measured_v=%.4f measured_w=%.4f check=%d check_us=%llu decision=%s rejected=%s state_reason=%s\n",(unsigned)R6_CONFIG_ID,(unsigned long long)now,n->r6_desired_v,n->r6_desired_w,v,w,n->r6_preferred_side,w>.02?1:w<-.02?-1:0,n->r4_input.direction[0].clearance_m,n->r4_output.speed_cap,n->measured_v,n->measured_w,n->r6_check_result,(unsigned long long)n->r6_check_time,n->r6_route_change_reason[0]?n->r6_route_change_reason:"NO_DIRECTION_CHANGE",n->s5_rejected_reason[0]?n->s5_rejected_reason:"NONE",nav_reason_text(n->reason));
     n->r6_route_log_us=now;
    }
    n->cmd.linear_m_s=v;n->cmd.angular_rad_s=w;n->cmd.time_us=now;n->cmd.expires_us=now+n->c.command_valid_us;
    s5_command_path(n,now,v,w);n->cmd.context=n->context;n->cmd.motion_intent=nav_action_intent(n,v,w);++n->cmd.sequence;*out=n->cmd;
}
static lab_status stop(nav_engine *n,uint64_t now,nav_state state,nav_reason reason,lab_status result,nav_command *out){
    if(n->cmd.linear_m_s!=0||n->cmd.angular_rad_s!=0)++n->stops;
    if(n->s3_action.active&&(state==NAV_REACHED||state==NAV_CANCELLED||state==NAV_FAILED||state==NAV_UNREACHABLE)){n->s3_action.active=0;n->s3_action.stage=0;snprintf(n->s3_action.reason,sizeof(n->s3_action.reason),"NAVIGATION_TERMINATED");}
    if(state==NAV_FAILED)n->phase=NAV_PHASE_BLOCKED;
    if(state==NAV_REACHED){n->phase=NAV_PHASE_TRACK;n->blocking_reason=NAV_REASON_NONE;}
    n->state=state;n->reason=reason;command(n,now,0,0,out);return result;
}
static lab_status wait_for(nav_engine *n,uint64_t now,nav_reason why,lab_status result,nav_command *out){
    n->blocking_reason=why;
    n->clear_scans=0;n->still_samples=0;
    if(!n->waiting){n->waiting=1;n->wait_started=now;}
    if(!n->c.passage.continuous&&now>=n->wait_started&&now-n->wait_started>=n->c.wait_timeout_us)
        return stop(n,now,NAV_FAILED,NAV_WAIT_TIMEOUT,result,out);
    return stop(n,now,NAV_WAITING,why,result,out);
}
static nav_xy lookahead(nav_engine *n,nav_xy p){
    n->corner_distance=INFINITY;n->corner_angle=0;unsigned s=n->segment;
    while(s+2<n->path_n){nav_xy a=n->path[s],b=n->path[s+1];double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,den=dx*dx+dy*dy;
        double t=den?((p.x_m-a.x_m)*dx+(p.y_m-a.y_m)*dy)/den:1;
        double tolerance=n->c.rectangular_footprint?.008:.035;
        int passed=hypot(p.x_m-b.x_m,p.y_m-b.y_m)<tolerance||(t>=1&&hypot(p.x_m-b.x_m,p.y_m-b.y_m)<tolerance*1.4);
        if(!passed&&n->c.passage.continuous){
          nav_xy next=n->path[s+2];double nx=next.x_m-b.x_m,ny=next.y_m-b.y_m,nd=nx*nx+ny*ny;
          double nt=nd?((p.x_m-b.x_m)*nx+(p.y_m-b.y_m)*ny)/nd:0,clamped=clamp(nt,0,1);
          double near_next=hypot(p.x_m-b.x_m-clamped*nx,p.y_m-b.y_m-clamped*ny),near_vertex=hypot(p.x_m-b.x_m,p.y_m-b.y_m);
          /* 实际绕行可能在路点旁数厘米通过，无需触碰8mm圆。按最近路径段单调推进，避免追已过拐角；只改变目标，实际动作仍检查完整车体扫掠。


 */
          passed=nt>0&&near_next<=n->c.lookahead_m&&((t>=1&&near_next+.01<near_vertex)||(t>=.5&&near_next+.05<near_vertex));
        }
        if(!passed&&n->c.rectangular_footprint&&hypot(p.x_m-b.x_m,p.y_m-b.y_m)<.10){
          nav_xy next=n->path[s+2];double heading=atan2(next.y_m-b.y_m,next.x_m-b.x_m);passage_pose pose={p.x_m,p.y_m,n->motion_yaw};passage_world w=rectangle_world(n,pose);w.planning_unknown_soft=0;passage_result hit;
          double d=hypot(next.x_m-b.x_m,next.y_m-b.y_m),t=fmin(1,.15/fmax(d,1e-9));passage_xy q={b.x_m+t*(next.x_m-b.x_m),b.y_m+t*(next.y_m-b.y_m)};
          if(passage_rotate_clear(&w,pose,heading,&hit)){pose.yaw=heading;if(passage_translate_clear(&w,pose,q,n->c.passage.width_m,&hit))passed=1;}
        }
        if(passed)++s;else break;}
    n->segment=s;nav_xy a=n->path[s],b=n->path[s+1];double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,den=dx*dx+dy*dy;
    double t=den?clamp(((p.x_m-a.x_m)*dx+(p.y_m-a.y_m)*dy)/den,0,1):1;
    nav_xy q={a.x_m+t*dx,a.y_m+t*dy};double d=hypot(b.x_m-q.x_m,b.y_m-q.y_m);
    if(s+2<n->path_n){nav_xy next=n->path[s+2];n->corner_angle=fabs(wrap(atan2(next.y_m-b.y_m,next.x_m-b.x_m)-atan2(dy,dx)));if(n->corner_angle>S2_ALIGN_ENTER_RAD)n->corner_distance=hypot(p.x_m-b.x_m,p.y_m-b.y_m);}
    /* 不跨越急转角直接前视，否则纯追踪会切入膨胀障碍；先到当前顶点再换段。
 */
    if(d>n->c.lookahead_m)return (nav_xy){q.x_m+(b.x_m-q.x_m)*n->c.lookahead_m/d,q.y_m+(b.y_m-q.y_m)*n->c.lookahead_m/d};
    return b;
}
static void advance(double *x,double *y,double *yaw,double v,double w,double dt){
    if(fabs(w)<1e-9){*x+=v*dt*cos(*yaw);*y+=v*dt*sin(*yaw);}
    else{*x+=v/w*(sin(*yaw+w*dt)-sin(*yaw));*y+=v/w*(cos(*yaw)-cos(*yaw+w*dt));}
    *yaw=wrap(*yaw+w*dt);
}
static nav_reason sweep_reason(nav_engine*n,const lab_pose*pose,double v,double w,const nav_xy*points,unsigned count){
 (void)pose;(void)points;(void)count;
 /* 历史、全局及未来多边形只作路线提示，不直接否决执行。 */
 if(!n->c.require_scan)return NAV_REASON_NONE;
 return nav_nf_reason(nav_nearfield_check(n,n->nf_now,v,w,0).code);
}
#include "nav_recovery.inc"
#include "nav_directional_flow.inc"
int nav_direction_allows(const nav_engine*n,uint64_t now,double v,double w){return n&&flow_sonar_allows(n,now,v,w)&&(!n->c.passage.continuous||!n->c.require_scan||nf_check_current(&n->nearfield,now,v,w).code==NF_CLEAR);}
static int rectangle_route_clear(nav_engine*n,const lab_pose*pose){
    if(n->path_n<2)return 0;passage_pose p={pose->x_m,pose->y_m,pose->yaw_rad};passage_world world=rectangle_world(n,p);passage_result hit;
    if(n->c.passage.continuous)world.planning_unknown_soft=0;
    nav_xy current={p.x,p.y};(void)lookahead(n,current);
    double checked=0;for(unsigned i=n->segment+1;i<n->path_n;i++){
        passage_xy end={n->path[i].x_m,n->path[i].y_m};if(hypot(end.x-p.x,end.y-p.y)<.000001)continue;
        double yaw=atan2(end.y-p.y,end.x-p.x);if(!passage_rotate_clear(&world,p,yaw,&hit))return hit.reason==PASSAGE_SEARCH_LIMIT?-1:0;p.yaw=yaw;
        double length=hypot(end.x-p.x,end.y-p.y);if(n->c.passage.continuous&&checked+length>.5){double t=(.5-checked)/fmax(length,.000001);end.x=p.x+(end.x-p.x)*t;end.y=p.y+(end.y-p.y)*t;}
        if(!passage_translate_clear(&world,p,end,n->c.passage.width_m,&hit))return hit.reason==PASSAGE_SEARCH_LIMIT?-1:0;checked+=hypot(end.x-p.x,end.y-p.y);p.x=end.x;p.y=end.y;if(n->c.passage.continuous&&checked>=.499)return 1;
    }
    return !n->goal.has_yaw||passage_rotate_clear(&world,p,n->goal.yaw_rad,&hit);
}
static void object_adapter_grid(nav_engine*,uint64_t);
lab_status nav_step(nav_engine *n,uint64_t now,const lab_event *p,const lab_event *scan,const lab_event *sp,nav_command *out){
    if(!n||!out)return LAB_ERR_ARGUMENT;
    uint64_t cost_start=nav_geometry_clock();n->r6_cycle_deadline=cost_start?cost_start+R6_CYCLE_GEOMETRY_BUDGET_US:0;n->query_deadline_us=n->r6_cycle_deadline;n->r6_check_result=R6_CHECK_CLEAR;n->r6_check_time=0;n->r6_step_time=now;n->r6_desired_v=n->r6_desired_w=0;n->r6_preferred_side=0;n->r6_route_change_reason[0]=0;n->cost_scan_us=n->cost_candidates_us=n->cost_plan_us=n->cost_route_us=n->cost_narrow_us=0;n->cost_candidate_count=0;
    if(now>UINT64_MAX-n->c.command_valid_us)return stop(n,0,NAV_FAILED,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    double dt=n->have_tick?(double)(now>=n->last_tick?now-n->last_tick:0)/1e6:.05;
    if(n->have_tick&&now<=n->last_tick)return stop(n,now,NAV_FAILED,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    n->last_tick=now;n->have_tick=1;
    n->nf_now=now;
    if(!n->have_goal)return stop(n,now,n->state==NAV_CANCELLED?NAV_CANCELLED:NAV_IDLE,n->state==NAV_CANCELLED?NAV_USER_CANCEL:NAV_NO_GOAL,LAB_OK,out);
    if((n->state==NAV_FAILED||n->state==NAV_UNREACHABLE)&&now>=n->last_plan&&now-n->last_plan>=2000000u&&
       n->c.passage.continuous&&
       (n->reason==NAV_SEARCH_LIMIT||n->reason==NAV_NO_PATH||n->reason==NAV_START_BLOCKED||n->reason==NAV_GOAL_BLOCKED||n->reason==NAV_WAIT_TIMEOUT)){
        n->state=NAV_WAITING;n->reason=NAV_REPLANNING;n->dirty=1;n->waiting=0;n->goal_started=now;n->phase=NAV_PHASE_TRACK;
    }
    if(n->state==NAV_FAILED||n->state==NAV_UNREACHABLE||n->state==NAV_REACHED)return stop(n,now,n->state,n->reason,LAB_OK,out);
    if(now<n->goal_started)return stop(n,now,NAV_FAILED,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    if(now-n->goal_started>=n->c.goal_timeout_us){
      if(n->c.passage.continuous)n->goal_started=now; /* 控制权仍由手机租约决定。 */
      else return stop(n,now,NAV_FAILED,NAV_GOAL_TIMEOUT,LAB_OK,out);
    }
    if(dt>.25)return wait_for(n,now,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    if(!n->n&&!n->external_active)return wait_for(n,now,NAV_NO_MAP,LAB_AGAIN,out);
    if(!context_event(n,p))return wait_for(n,now,NAV_BAD_CONTEXT,LAB_ERR_CONTEXT,out);
    if(!pose_valid(p))return wait_for(n,now,NAV_INVALID_POSE,LAB_ERR_ARGUMENT,out);
    if(p->meta.time_us>now||p->meta.received_us>now||now-p->meta.time_us>n->c.max_pose_age_us)
        return wait_for(n,now,NAV_POSE_STALE,LAB_AGAIN,out);
    /* 栈使用限制128KiB，仅在匹配位姿下转换有效回波。 */
    nav_xy points[SCAN_MAX_POINTS];unsigned point_n=0;
    if(n->c.require_scan){
        if(!scan||!sp)return wait_for(n,now,NAV_SCAN_STALE,LAB_AGAIN,out);
        if(!context_event(n,scan)||!context_event(n,sp))return wait_for(n,now,NAV_BAD_CONTEXT,LAB_ERR_CONTEXT,out);
        if(scan->type!=LAB_EVENT_SCAN||scan->meta.provider==LAB_PROVIDER_TRUTH||!w1_real_carrier_event(scan)||!pose_valid(sp)||
          (scan->meta.flags&(LAB_TIME_VALID|LAB_DATA_VALID))!=(LAB_TIME_VALID|LAB_DATA_VALID))
            return wait_for(n,now,NAV_SCAN_INVALID,LAB_ERR_ARGUMENT,out);
        if(scan->meta.time_us>now||scan->meta.received_us>now||sp->meta.received_us>now||scan->meta.time_us!=sp->meta.time_us||
           now-scan->meta.time_us>n->c.max_scan_age_us)return wait_for(n,now,NAV_SCAN_STALE,LAB_AGAIN,out);
        if(!n->nf_external){nf_observe(&n->nearfield,sp,now);nf_observe(&n->nearfield,scan,now);nf_observe(&n->nearfield,p,now);}
        double ca=cos(sp->data.pose.yaw_rad),sa=sin(sp->data.pose.yaw_rad);
        for(unsigned j=0;j<scan->data.scan.count;++j){
            const struct scan_point *q=&scan->data.scan.points[j];if(!q->quality||!scan_return_in_range(&scan->data.scan,j,n->c.rectangular_footprint?n->c.passage.lidar_min_m:.001,n->c.rectangular_footprint?n->c.passage.lidar_max_m:30))continue;
            /* LS01节点标志仅作诊断。 */
            double r=q->distance_q2/4000.0;if(r>30||(n->c.rectangular_footprint&&(r<n->c.passage.lidar_min_m||r>n->c.passage.lidar_max_m)))continue;
            double a=-(double)q->angle_q14*(PI/32768.0)+n->c.mount_yaw_rad;
            double bx=n->c.mount_x_m+r*cos(a),by=n->c.mount_y_m+r*sin(a);
            points[point_n++]=(nav_xy){sp->data.pose.x_m+ca*bx-sa*by,sp->data.pose.y_m+sa*bx+ca*by};
        }
        if(!point_n)return wait_for(n,now,NAV_SCAN_INVALID,LAB_AGAIN,out);
    }
    int new_scan=scan&&(!n->have_scan_sequence||(scan->meta.time_us>n->scan_time&&
        (int32_t)(scan->meta.sequence-n->scan_sequence)>0));
    if(new_scan){n->have_scan_sequence=1;n->scan_sequence=scan->meta.sequence;n->scan_time=scan->meta.time_us;}
    uint64_t scan_started=nav_geometry_clock();
    if(new_scan&&n->c.rectangular_footprint&&n->local){
        passage_pose base={p->data.pose.x_m,p->data.pose.y_m,p->data.pose.yaw_rad},at_scan={sp->data.pose.x_m,sp->data.pose.y_m,sp->data.pose.yaw_rad};
        if(passage_local_begin(n->local,&n->c.passage,base,scan->meta.sequence,scan->meta.time_us))for(unsigned j=0;j<scan->data.scan.count;j++){
            const struct scan_point*q=&scan->data.scan.points[j];passage_local_ray(n->local,&n->c.passage,at_scan,-(double)q->angle_q14*(PI/32768.0),q->distance_q2/4000.0,q->quality,scan->meta.sequence,scan->meta.time_us);
        }
        passage_local_scan_finish(n->local,&n->c.passage,scan->meta.sequence,scan->meta.time_us);
    }
    n->cost_scan_us=nav_geometry_clock()-scan_started;
    observe_motion(n,p,now);rear_observed(n,&p->data.pose,points,point_n);
    dynamic_layer(n,&p->data.pose,points,point_n,now,new_scan);
    if(n->external_active)object_adapter_grid(n,now);
    nav_xy pos={p->data.pose.x_m,p->data.pose.y_m};n->last_distance=hypot(n->goal.x_m-pos.x_m,n->goal.y_m-pos.y_m);
    n->flow_final_yaw=0;
    if(flow_enabled(n)&&!n->flow_cruise&&n->last_distance<=n->c.goal_tolerance_m){
        double yaw_error=wrap(n->goal.yaw_rad-p->data.pose.yaw_rad);
        if(!n->goal.has_yaw||fabs(yaw_error)<=n->c.yaw_tolerance_rad){n->waiting=0;return stop(n,now,NAV_REACHED,NAV_GOAL_REACHED,LAB_OK,out);}
        n->flow_final_yaw=1;nav_xy facing={pos.x_m+cos(n->goal.yaw_rad),pos.y_m+sin(n->goal.yaw_rad)};
        return flow_step(n,now,&p->data.pose,facing,0,clamp(2*yaw_error,-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s),points,point_n,new_scan,out);
    }
    if(n->route_report.partial&&n->path_n>=2&&hypot(pos.x_m-n->path[n->path_n-1].x_m,pos.y_m-n->path[n->path_n-1].y_m)<.12){n->dirty=1;n->last_plan=0;n->path_n=0;n->segment=0;}
    if(n->flow_cruise){nav_xy forward={pos.x_m+cos(p->data.pose.yaw_rad),pos.y_m+sin(p->data.pose.yaw_rad)};n->path_n=0;return flow_step(n,now,&p->data.pose,forward,n->c.max_speed_m_s,0,points,point_n,new_scan,out);}
    uint64_t route_started=nav_geometry_clock();
    if(n->c.rectangular_footprint&&n->dirty==2){int route_clear=rectangle_route_clear(n,&p->data.pose);if(route_clear>0)n->dirty=0;else if(route_clear==0){n->dirty=1;n->path_n=0;n->segment=0;}}
    n->cost_route_us=nav_geometry_clock()-route_started;
    if(n->dirty&&(n->phase==NAV_PHASE_TRACK||n->phase==NAV_PHASE_AVOID||n->phase==NAV_PHASE_RECOVER||(flow_enabled(n)&&n->phase!=NAV_PHASE_BACKUP&&n->phase!=NAV_PHASE_REJOIN))){
        if(n->plans&&now-n->last_plan<n->c.replan_interval_us){if(!flow_enabled(n))return wait_for(n,now,NAV_MAP_CHANGED,LAB_AGAIN,out);}else{
        uint64_t plan_started=nav_geometry_clock();n->last_plan=now;nav_reason why=plan(n,pos);n->cost_plan_us=nav_geometry_clock()-plan_started;n->dirty=0;
#ifdef LAB_REAL_POSIX_SOURCE
        if(why==NAV_REPLANNING){n->dirty=1;n->last_plan=0;n->waiting=0;
  if(!n->c.passage.continuous)return stop(n,now,NAV_WAITING,why,LAB_AGAIN,out);
  if(n->path_n<2||!rectangle_route_clear(n,&p->data.pose)){n->path_n=0;nav_xy t={n->goal.x_m,n->goal.y_m};return flow_step(n,now,&p->data.pose,t,n->c.max_speed_m_s,0,points,point_n,new_scan,out);}
  why=NAV_REASON_NONE;n->reason=NAV_REPLANNING;
 }
#endif
        if(why!=NAV_REASON_NONE){
            if(flow_enabled(n)){n->dirty=1;flow_episode(n,&p->data.pose,now,why);nav_xy t={n->goal.x_m,n->goal.y_m};return flow_step(n,now,&p->data.pose,t,n->c.max_speed_m_s,0,points,point_n,new_scan,out);}
            else if(n->c.enable_recovery&&why==NAV_NO_PATH&&(n->obstacle_n||n->episode)){
                if(!n->episode)begin_episode(n,&p->data.pose,now,why);
                phase_to(n,NAV_PHASE_BACK_CHECK,now);
            }
            else return stop(n,now,NAV_UNREACHABLE,why,LAB_OK,out);
        }
    }
    }
    if(flow_enabled(n)&&!n->c.passage.r4_enabled&&(n->phase==NAV_PHASE_BACKUP||n->phase==NAV_PHASE_REJOIN)){nav_xy t=n->path_n>=2?lookahead(n,pos):(nav_xy){n->goal.x_m,n->goal.y_m};return flow_step(n,now,&p->data.pose,t,0,0,points,point_n,new_scan,out);}
    if(!flow_enabled(n)&&n->c.enable_recovery&&n->phase!=NAV_PHASE_TRACK){
        nav_xy target=n->path_n>=2?lookahead(n,pos):(nav_xy){n->goal.x_m,n->goal.y_m};
        return recovery_step(n,now,&p->data.pose,target,dt,points,point_n,new_scan,out);
    }
    double v=0,w=0;
    if(n->last_distance<=n->c.goal_tolerance_m){
        double error=wrap(n->goal.yaw_rad-p->data.pose.yaw_rad);
        if(!n->goal.has_yaw||fabs(error)<=n->c.yaw_tolerance_rad){n->waiting=0;return stop(n,now,NAV_REACHED,NAV_GOAL_REACHED,LAB_OK,out);}
        w=clamp(2*error,-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);n->reason=NAV_FINAL_YAW;
    }else{
        if(n->path_n<2){if(flow_enabled(n)){nav_xy t={n->goal.x_m,n->goal.y_m};return flow_step(n,now,&p->data.pose,t,n->c.max_speed_m_s,0,points,point_n,new_scan,out);}return stop(n,now,NAV_UNREACHABLE,NAV_NO_PATH,LAB_OK,out);}
        nav_xy target=lookahead(n,pos);double d=hypot(target.x_m-pos.x_m,target.y_m-pos.y_m);
        double alpha=wrap(atan2(target.y_m-pos.y_m,target.x_m-pos.x_m)-p->data.pose.yaw_rad);
        if(fabs(alpha)>.65){v=0;w=clamp(2*alpha,-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);}
        else{v=n->c.max_speed_m_s*clamp(1-fabs(alpha)/.9,.20,1);
            v=fmin(v,sqrt(2*n->c.brake_m_s2*fmax(n->last_distance-n->c.goal_tolerance_m,0)));
            if(n->segment+2<n->path_n)v=fmin(v,sqrt(2*n->c.brake_m_s2*fmax(d-.02,0)));
            w=clamp(2*v*sin(alpha)/fmax(d,.05),-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);}
        n->reason=NAV_REASON_NONE;
    }
    uint64_t narrow_started=nav_geometry_clock();
    double passage_speed_limit=n->c.max_speed_m_s;n->narrow=0;
    if(n->c.rectangular_footprint){
        passage_pose pose={p->data.pose.x_m,p->data.pose.y_m,p->data.pose.yaw_rad};passage_world world=rectangle_world(n,pose);passage_result hit;
        if(flow_enabled(n)&&n->c.require_scan&&n->local&&n->local->initialized)world.local_only=1;
        double ahead=fmax(.15,fabs(v)*.5+.05),wide=n->c.passage.narrow_width_m-2*n->c.passage.margin_m;
        if(!passage_translate_clear(&world,pose,(passage_xy){pose.x+ahead*cos(pose.yaw),pose.y+ahead*sin(pose.yaw)},wide,&hit)&&hit.reason==PASSAGE_OCCUPIED){n->narrow=1;passage_speed_limit=n->c.passage.narrow_speed_m_s;}
        v=fmin(v,passage_speed_limit);
    }
    n->cost_narrow_us=nav_geometry_clock()-narrow_started;
    if(!flow_enabled(n))v=approach(n->cmd.linear_m_s,v,n->c.acceleration_m_s2*dt);
    if(n->c.rectangular_footprint)v=fmin(v,passage_speed_limit);
    if(!flow_enabled(n))w=approach(n->cmd.angular_rad_s,w,n->c.angular_acceleration_rad_s2*dt);
    if(flow_enabled(n)){nav_xy t=n->path_n>=2?lookahead(n,pos):(nav_xy){n->goal.x_m,n->goal.y_m};if(n->last_distance<=n->c.goal_tolerance_m&&n->goal.has_yaw)t=(nav_xy){pos.x_m+cos(n->goal.yaw_rad),pos.y_m+sin(n->goal.yaw_rad)};return flow_step(n,now,&p->data.pose,t,v,w,points,point_n,new_scan,out);}
    nav_reason unsafe=sweep_reason(n,&p->data.pose,v,w,points,point_n);
    if(unsafe!=NAV_REASON_NONE&&n->c.rectangular_footprint&&n->c.passage.continuous){
        /* 只有硬、软检查均通过才选相近曲率，保留方向和前进速度，不进行未观测后退或盲目脱困。
 */
        double candidates[3]={w*.5,0,clamp(w+(w>=0?-.15:.15),-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s)};
        for(unsigned i=0;i<3;i++)if(fabs(candidates[i]-n->cmd.angular_rad_s)<=n->c.angular_acceleration_rad_s2*dt+1e-9&&sweep_reason(n,&p->data.pose,v,candidates[i],points,point_n)==NAV_REASON_NONE){
          w=candidates[i];unsafe=NAV_REASON_NONE;n->reason=NAV_LOCAL_AVOID;break;
        }
    }
    if(unsafe!=NAV_REASON_NONE){
        if(!n->c.enable_recovery)return wait_for(n,now,unsafe,LAB_AGAIN,out);
        begin_episode(n,&p->data.pose,now,unsafe);
        nav_xy target=n->path_n>=2?lookahead(n,pos):(nav_xy){n->goal.x_m,n->goal.y_m};
        return recovery_step(n,now,&p->data.pose,target,dt,points,point_n,new_scan,out);
    }
    n->waiting=0;n->state=NAV_TRACKING;command(n,now,v,w,out);return LAB_OK;
}
int nav_action_validate(nav_engine*n,uint64_t now,const lab_pose*p,uint64_t pose_time,double v,double w){
 int semantic_static=n&&n->external_active&&n->object_target.mode==1&&(n->object_target.static_search_left||n->object_target.static_bearing_approach);
 if(!n||!p||!nf_pose_valid(p)||(!n->have_goal&&!semantic_static)||pose_time>now||now-pose_time>n->c.max_pose_age_us||n->scan_time>now||now-n->scan_time>n->c.max_scan_age_us||!flow_sonar_allows(n,now,v,w))return 0;
 if(n->c.require_scan&&(!n->nearfield.have_context||!context_equal(n->nearfield.context,n->context)))return 0;
 uint64_t tick=n->last_tick;n->last_tick=now;
 int ok=(!n->c.require_scan||nav_nearfield_check(n,now,v,w,0).code==NF_CLEAR)&&flow_sonar_geometry(n,v,w,nav_action_intent(n,v,w)==S1_ESCAPE);
 n->last_tick=tick;return ok;
}
void nav_command_published(nav_engine*n,uint64_t now,uint64_t expiry){if(n&&n->have_goal){n->cmd.time_us=now;n->cmd.expires_us=expiry;}}
void nav_reduce_load(nav_engine*n,uint32_t reduced){if(n)n->flow_load_reduced=reduced!=0;}
void nav_profile_log(nav_engine*n,uint64_t now){if(!n||!nav_geometry_clock()||(n->cost_log_at&&now>=n->cost_log_at&&now-n->cost_log_at<1000000))return;n->cost_log_at=now;fprintf(stderr,"NAV_STAGE_COST policy=CURRENT_MEASURED prediction=ADVISORY_ONLY 当前实测防护_预测仅建议 now_us=%llu scan_us=%llu plan_us=%llu route_us=%llu narrow_us=%llu candidates_us=%llu candidate_count=%u load_reduced=%u\n",(unsigned long long)now,(unsigned long long)n->cost_scan_us,(unsigned long long)n->cost_plan_us,(unsigned long long)n->cost_route_us,(unsigned long long)n->cost_narrow_us,(unsigned long long)n->cost_candidates_us,n->cost_candidate_count,n->flow_load_reduced);}
lab_status nav_get_view(const nav_engine *n,nav_view *out){
    if(!n||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));out->has_object_target=n->object_target.valid;out->object_generation=n->object_target.generation;out->object_source_us=n->object_target.source_us;out->object_target_x=n->object_target.target_x_m;out->object_target_y=n->object_target.target_y_m;out->object_goal_x=n->object_target.goal_x_m;out->object_goal_y=n->object_target.goal_y_m;out->state=n->state;out->reason=n->reason;out->blocking_reason=n->blocking_reason;out->goal=n->goal;
    nav_nearfield_view(n,n->nf_now>n->last_tick?n->nf_now:n->last_tick,&out->nearfield);out->avoidance=n->s3_action;out->avoidance.policy_revision=S5_POLICY_REVISION;out->avoidance.front_approach=n->s5_front_approach;out->avoidance.nf_emergency=n->s5_emergency;out->avoidance.turn_hold=n->s5_turn_hold;out->avoidance.front_clearance_m=n->s5_front_clearance;out->avoidance.front_corridor_clearance_m=n->s5_corridor_clearance;out->avoidance.reverse_target_m=n->flow_backup_target;for(unsigned side=0;side<2;side++){out->avoidance.side_state[side]=n->s5_side_state[side];out->avoidance.side_clearance_m[side]=n->s5_side_clearance[side];}out->avoidance.rejected_v_m_s=n->s5_rejected_v;out->avoidance.rejected_w_rad_s=n->s5_rejected_w;snprintf(out->avoidance.rejected_reason,sizeof(out->avoidance.rejected_reason),"%s",n->s5_rejected_reason);
    out->avoidance.scene_id=n->r4_output.scene;out->avoidance.side_class=n->r4_side_class;out->avoidance.calibrated=n->r4_calibrated;out->avoidance.scene_owner=n->r4_output.owner;out->avoidance.scene_stage=n->r4_output.stage;out->avoidance.explore_round=n->r4_output.explore_round;out->avoidance.reverse_attempts=n->r4_output.reverse_attempts;out->avoidance.reverse_extended_attempts=n->r4_output.reverse_extended_attempts;out->avoidance.scene_speed_cap=n->r4_output.speed_cap;snprintf(out->avoidance.scene,sizeof(out->avoidance.scene),"%s",r4_scene_name(n->r4_output.scene));snprintf(out->avoidance.scene_reason,sizeof(out->avoidance.scene_reason),"%s",n->r4_scene_reason);
    if(n->r4_active){out->avoidance.nf_emergency=n->r4_output.emergency;out->avoidance.turn_hold=n->r4_output.stage==R4_TURN_HOLD;out->avoidance.reverse_target_m=n->r4_state.fitted_target_m;out->avoidance.backup_target_m=n->r4_state.fitted_target_m;out->avoidance.backup_progress_m=n->r4_state.reverse_progress;}
    out->avoidance.scene_pending=n->r4_state.candidate_scene!=n->r4_state.scene;
    if(n->r4_active){
     uint32_t stage=n->r4_state.stage;int direction=n->r4_state.turn_side?-1:1;
     out->avoidance.scene_stage=stage;
     out->avoidance.active=n->have_goal&&(stage!=R4_STAGE_IDLE||n->r4_output.emergency);
     out->avoidance.stage=stage==R4_PROBE_TURN?2:stage==R4_REVERSE?3:stage==R4_EXIT_TURN?4:stage==R4_STAGE_IDLE&&!n->r4_output.emergency?0:5;
     out->avoidance.direction=stage==R4_PROBE_TURN?direction:0;
     out->avoidance.event_id=n->goal.id;
     out->avoidance.turn_target_rad=stage==R4_PROBE_TURN?direction*n->r4_config.explore_step_rad:0;
     out->avoidance.turn_progress_rad=stage==R4_PROBE_TURN?direction*n->r4_state.turn_progress:0;
     snprintf(out->avoidance.reason,sizeof(out->avoidance.reason),"%.47s",n->r4_scene_reason);
    }
    for(unsigned j=0;j<4;j++){const r4_direction*d=&n->r4_input.direction[j];uint64_t now=n->r4_input.now_us;out->avoidance.direction_age_us[j]=d->source_us&&d->source_us<=now?now-d->source_us:UINT64_MAX;if(r4_direction_usable(d,now,n->c.max_scan_age_us))out->avoidance.usable_direction_mask|=1u<<j;}
    out->avoidance.policy_revision=9;out->avoidance.planner_active=n->s6_active;out->avoidance.planner_rank=n->s6_rank;out->avoidance.planner_vetoed=n->s6_vetoed;out->avoidance.planner_samples=n->s6_plan?n->s6_plan->sampled:0;out->avoidance.planner_generated_us=n->s6_generated;out->avoidance.costmap_age_ms=n->s6_cm?cm_age_ms(n->s6_cm,n->last_tick):UINT64_MAX;out->avoidance.planner_v_error=n->s6_v_error;out->avoidance.planner_w_error=n->s6_w_error;out->avoidance.traj_mismatch_ms=n->s6_mismatch_since&&n->last_tick>=n->s6_mismatch_since?(n->last_tick-n->s6_mismatch_since)/1000:0;snprintf(out->avoidance.planner_veto_reason,sizeof(out->avoidance.planner_veto_reason),"%s",n->s6_reason);
    if(n->s6_active&&n->s6_plan&&n->s6_rank<n->s6_plan->count){const s6_trajectory*t=&n->s6_plan->ranked[n->s6_rank];out->avoidance.planner_v_m=n->cmd.linear_m_s;out->avoidance.planner_w_rad_s=n->cmd.angular_rad_s;out->avoidance.planner_score=t->score;out->avoidance.planner_clear_min=t->clear_min;out->avoidance.traj_count=t->count;for(unsigned k=0;k<t->count;k++){const s6_point*q=&t->points[k];out->avoidance.traj_points[k]=(s3_trajectory_point){q->x,q->y,q->yaw,q->v,q->w,q->t};}}
    out->avoidance.route_expected_v_m_s=n->r6_desired_v;out->avoidance.route_expected_w_rad_s=n->r6_desired_w;
    out->avoidance.route_actual_v_m_s=n->cmd.linear_m_s;out->avoidance.route_actual_w_rad_s=n->cmd.angular_rad_s;
    out->avoidance.route_expected_side=n->r6_preferred_side;out->avoidance.route_actual_side=r6_side_of(n->cmd.angular_rad_s);
    out->avoidance.route_side_changed=r6_route_opposite(n,n->cmd.angular_rad_s);
    out->avoidance.motion_check_result=n->r6_check_result;out->avoidance.motion_check_time_us=n->r6_check_time;
    snprintf(out->avoidance.route_change_reason,sizeof(out->avoidance.route_change_reason),"%s",n->r6_route_change_reason);
    out->route_complete=n->route_report.complete;out->route_partial=n->route_report.partial;out->route_search_budget=n->route_report.budget;out->route_kind=n->route_report.kind;snprintf(out->route_reason,sizeof(out->route_reason),"%s",passage_reason_text(n->route_report.reason));
    out->extended_reverse_attempts=n->flow_extended_attempts;out->extended_reverse_active=n->flow_backup_extended;out->backup_target_m=n->flow_backup_target;out->command=n->cmd;out->map_revision=n->revision;out->plans=n->plans;out->expanded_nodes=n->expanded;out->emergency_stops=n->stops;
    out->goal_distance_m=n->last_distance;out->footprint_radius_m=n->radius;out->path_count=n->path_n;out->path=n->path;out->local_path_count=n->s5_command_path_n;out->local_path=n->s5_command_path;
    out->blocked=n->blocked;out->width=n->w;out->height=n->h;out->resolution_m=n->res;out->origin_x_m=n->ox;out->origin_y_m=n->oy;
    out->allocated_bytes=n->allocated;out->phase=n->phase;out->reverse_attempts=n->reverse_attempts;
    out->clear_scans=n->clear_scans;out->rear_clearance_m=n->rear_clearance;
    out->backup_progress_m=n->backup_progress;out->backup_total_m=n->backup_total;
    out->dynamic_count=n->obstacle_n;for(unsigned i=0;i<n->obstacle_n;++i)out->dynamic_obstacles[i]=n->obstacles[i].p;
    out->narrow_passage=n->narrow;out->passage_reason=(uint32_t)n->passage_collision.reason;out->lidar_offset_verified=n->c.passage.mount_verified;
    out->vehicle_width_m=n->c.rectangular_footprint?n->c.passage.width_m:n->c.vehicle_width_m;out->vehicle_length_m=n->c.rectangular_footprint?n->c.passage.length_m:n->c.vehicle_length_m;out->side_margin_m=n->c.rectangular_footprint?n->c.passage.margin_m:n->c.safety_margin_m;out->local_resolution_m=n->c.rectangular_footprint?n->c.passage.local_resolution_m:0;
    out->cruise_mode=n->flow_cruise;out->local_probe=n->flow_local_probe;out->can_start_local=n->flow_can_start;out->backup_requests=n->flow_backup_requests;
    out->provisional_route=n->flow_provisional;out->candidate_sonar_rejections=n->flow_reject_sonar;out->candidate_occupied_rejections=n->flow_reject_occupied;out->candidate_unknown_rejections=n->flow_reject_unknown;out->candidate_accepted=n->flow_accept;out->candidate_budget_rejections=n->flow_reject_budget;out->observed_fraction=n->flow_observed_fraction;snprintf(out->rear_reason,sizeof(out->rear_reason),"%s",nav_reason_text(n->flow_rear_reason));
    return LAB_OK;
}
const char *nav_state_text(nav_state s){const char *a[]={"IDLE","TRACKING","WAITING","REACHED","CANCELLED","UNREACHABLE","FAILED"};return s>=0&&s<=NAV_FAILED?a[s]:"UNKNOWN";}
const char *nav_reason_text(nav_reason s){
    const char *a[]={"NONE","NO_GOAL","GOAL_REACHED","USER_CANCEL","NO_MAP","BAD_CONTEXT","INVALID_POSE","POSE_STALE","SCAN_STALE",
      "SCAN_INVALID","GOAL_OUTSIDE","GOAL_BLOCKED","START_BLOCKED","NO_PATH","SEARCH_LIMIT","PATH_LIMIT","MAP_CHANGED","SWEEP_BLOCKED",
      "OBSTACLE","WAIT_TIMEOUT","GOAL_TIMEOUT","CLOCK_ERROR","INVALID_INPUT","FINAL_YAW",
      "LOCAL_AVOID","REPLANNING","TURN_RECOVERY","BACK_CHECK","BACKING","LOW_RECOVERY",
      "REAR_BLOCKED","REAR_UNOBSERVED","RECOVERY_EXHAUSTED","LOCAL_PROBE","CRUISE","RECOVERY_RETRY_WAIT","REAR_SCAN_STALE","REAR_COVERAGE","REAR_SONAR_STALE","REAR_BODY_OCCUPIED","REAR_GEOMETRY_UNKNOWN","REAR_CLEARANCE","CONTROL_GEOMETRY_BUDGET","NEARFIELD_OBSERVATION_WAIT","NEARFIELD_DATA_STALE","NEARFIELD_POSE_INVALID","NEARFIELD_INVALID_ACTION","ALL_ACTIONS_UNAVAILABLE","FRONT_APPROACH","NF_EMERGENCY","NEAR_AVOID","TURN_HOLD","RECOVERY_WAIT","SIDE_ACTION_REJECT","PATH_ACTION_MISMATCH"};
    return s>=0&&(unsigned)s<sizeof(a)/sizeof(a[0])?a[s]:"UNKNOWN";
}
void nav_close(nav_engine *n){if(n){
#ifdef LAB_REAL_POSIX_SOURCE
 if(n->async){async_plan*a=n->async;int orphaned=0;if(a->active){pthread_mutex_lock(&a->mutex);if(!a->done){atomic_store(&a->cancelled,1);a->orphaned=1;orphaned=1;pthread_detach(a->thread);}pthread_mutex_unlock(&a->mutex);if(!orphaned)pthread_join(a->thread,NULL);}if(!orphaned){free(a->cells);free(a->local);pthread_mutex_destroy(&a->mutex);free(a);}}
#endif
 free_grid(n);free(n->local);free(n->s6_cm);free(n->s6_plan);free(n);}}

/* 附加ODOM任务接口，与参考决策分开。 */
#include "nav_object_adapter.inc"
