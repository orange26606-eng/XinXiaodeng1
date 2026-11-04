#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "phone_nav_snapshot.h"
#include "passage.h"
#include "navigation_status.h"
#include "motion_preview.h"
#include "s5_status_text.h"
#include "r4_status_text.h"
#include "s1_motion_policy.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#if defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#endif

typedef struct buffer {char *p;size_t cap,used;int ok;} buffer;
static void add(buffer*b,const char*fmt,...){
 if(!b->ok)return;va_list a;va_start(a,fmt);int n=vsnprintf(b->p+b->used,b->cap-b->used,fmt,a);va_end(a);
 if(n<0||(size_t)n>=b->cap-b->used){b->ok=0;return;}b->used+=(size_t)n;
}
static void bytes64(buffer*b,const int8_t*p,size_t n){
 static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
 for(size_t i=0;i<n&&b->ok;i+=3){unsigned a=(unsigned char)p[i],c=i+1<n?(unsigned char)p[i+1]:0,d=i+2<n?(unsigned char)p[i+2]:0;
  if(b->cap-b->used<=4){b->ok=0;break;}b->p[b->used++]=alphabet[a>>2];b->p[b->used++]=alphabet[((a&3)<<4)|(c>>4)];b->p[b->used++]=i+1<n?alphabet[((c&15)<<2)|(d>>6)]:'=';b->p[b->used++]=i+2<n?alphabet[d&63]:'=';b->p[b->used]='\0';}
}
static const char*safe_reason(const char*s){for(const char*p=s;*p;p++)if(!((*p>='A'&&*p<='Z')||(*p>='0'&&*p<='9')||*p=='_'))return "UNKNOWN";return s;}
static void s3_distance(buffer*b,int known,double value){if(known&&isfinite(value)&&value>=0)add(b,"%.6g",value);else add(b,"null");}
static void s6_diagnostics(buffer*b,const s3_avoidance_view*a){
 add(b,",\"planner_active\":%s,\"planner_v_m\":%.9g,\"planner_w_rad_s\":%.9g,\"planner_veto_reason\":\"%s\",\"planner_diagnostics\":{\"software_policy\":9,\"fpga_policy\":5,\"generated_us\":\"%llu\",\"costmap_age_ms\":%llu,\"rank\":%u,\"sampled\":%u,\"vetoed\":%u,\"score\":%.9g,\"clear_min_m\":%.9g,\"v_error\":%.9g,\"w_error\":%.9g,\"mismatch_ms\":%llu,\"sonar_time_mode\":\"RX_MINUS_SOURCE_AGE_UNSYNCHRONIZED\"},\"traj_points\":[",a->planner_active?"true":"false",isfinite(a->planner_v_m)?a->planner_v_m:0,isfinite(a->planner_w_rad_s)?a->planner_w_rad_s:0,safe_reason(a->planner_veto_reason),(unsigned long long)a->planner_generated_us,(unsigned long long)a->costmap_age_ms,a->planner_rank,a->planner_samples,a->planner_vetoed,isfinite(a->planner_score)?a->planner_score:0,isfinite(a->planner_clear_min)?a->planner_clear_min:0,isfinite(a->planner_v_error)?a->planner_v_error:0,isfinite(a->planner_w_error)?a->planner_w_error:0,(unsigned long long)a->traj_mismatch_ms);
 uint32_t count=a->traj_count<=30?a->traj_count:0;
 for(uint32_t i=0;i<count;i++){const s3_trajectory_point*q=&a->traj_points[i];if(!isfinite(q->x)||!isfinite(q->y)||!isfinite(q->yaw)||!isfinite(q->v)||!isfinite(q->w)||!isfinite(q->t))break;add(b,"%s[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]",i?",":"",q->x,q->y,q->yaw,q->v,q->w,q->t);}add(b,"]");
}
static void r4_diagnostics(buffer*b,const s3_avoidance_view*a){
 add(b,",\"scene_id\":%u,\"scene\":\"%s\",\"scene_zh\":\"%s\",\"scene_reason\":\"%s\",\"scene_reason_zh\":\"%s\",\"scene_owner\":%u,\"scene_owner_zh\":\"%s\",\"scene_stage\":%u,\"scene_stage_zh\":\"%s\",\"side_class\":%u,\"scene_calibrated\":%s,\"scene_speed_cap\":%.6g,\"explore_round\":%u,\"reverse_attempts\":%u,\"reverse_extended_attempts\":%u,\"max_rpm\":120,\"pivot_rpm\":78,\"continuous_exploration\":true",
 a->scene_id,safe_reason(a->scene),a->scene[0]?r4_scene_zh(a->scene_id):"等待本轮场景",safe_reason(a->scene_reason),r4_reason_zh(a->scene_reason),a->scene_owner,r4_owner_zh(a->scene_owner),a->scene_stage,r4_stage_zh(a->scene_stage),a->side_class,a->calibrated?"true":"false",isfinite(a->scene_speed_cap)&&a->scene_speed_cap>=0?a->scene_speed_cap:0,a->explore_round,a->reverse_attempts,a->reverse_extended_attempts);
 add(b,",\"usable_direction_mask\":%u,\"scene_pending\":%s,\"direction_age_ms\":[",a->usable_direction_mask&15u,a->scene_pending?"true":"false");
 for(unsigned i=0;i<4;i++){if(i)add(b,",");if(a->direction_age_us[i]==UINT64_MAX)add(b,"null");else add(b,"%.3f",(double)a->direction_age_us[i]/1000.0);}add(b,"]");
}
static void r6_diagnostics(buffer*b,const s3_avoidance_view*a,uint64_t now,int task_active){
 int historical=a->motion_check_time_us&&a->motion_check_time_us<=now;
 int checked=task_active&&historical&&now-a->motion_check_time_us<=300000;
 add(b,",\"route_policy\":{\"expected_side\":%d,\"actual_side\":%d,\"expected_v\":%.6g,\"expected_w\":%.6g,\"actual_v\":%.6g,\"actual_w\":%.6g,\"side_changed\":%s,\"change_reason\":\"%s\",\"change_reason_zh\":\"%s\"}",a->route_expected_side,a->route_actual_side,isfinite(a->route_expected_v_m_s)?a->route_expected_v_m_s:0,isfinite(a->route_expected_w_rad_s)?a->route_expected_w_rad_s:0,isfinite(a->route_actual_v_m_s)?a->route_actual_v_m_s:0,isfinite(a->route_actual_w_rad_s)?a->route_actual_w_rad_s:0,a->route_side_changed?"true":"false",safe_reason(a->route_change_reason),a->route_change_reason[0]?r4_reason_zh(a->route_change_reason):"");
 add(b,",\"action_check\":{\"available\":%s,\"result\":",checked?"true":"false");if(checked)add(b,"%d",a->motion_check_result);else add(b,"null");
 add(b,",\"reason_zh\":\"%s\",\"checked_us\":\"%llu\",\"age_ms\":%.3f}",checked?r6_check_zh(a->motion_check_result):"尚无本轮动作检查",(unsigned long long)a->motion_check_time_us,checked?(double)(now-a->motion_check_time_us)/1000.0:-1.0);
 add(b,",\"front_policy\":{\"observe_mm\":%u,\"near_mm\":%u,\"forward_stop_mm\":%u,\"emergency_mm\":%u,\"release_mm\":%u,\"release_samples\":%u}",R6_FRONT_OBSERVE_MM,R6_FRONT_NEAR_MM,R6_FRONT_FORWARD_STOP_MM,R6_FRONT_EMERGENCY_MM,R6_FRONT_RELEASE_MM,R6_FRONT_RELEASE_SAMPLES);
}
static void s5_diagnostics(buffer*b,const lab_real_runtime_view*v,uint64_t now){
 const s3_avoidance_view*a=&v->navigation.avoidance;const lab_chassis_feedback*f=&v->feedback;
 int raw=f->sonar_diagnostic_time_us&&now>=f->sonar_diagnostic_time_us&&now-f->sonar_diagnostic_time_us<CONT_FEEDBACK_US;
 add(b,",\"policy_revision\":9,\"expected_config_id\":\"0x%08X\",\"status_revision\":5,\"command_version\":3,\"fpga_policy\":{\"config_id\":%u,\"status_revision\":%u,\"ttl_ms\":%u},\"state_zh\":\"%s\",\"front_approach\":%s,\"nf_emergency\":%s,\"turn_hold\":%s,\"front_clearance_m\":",(unsigned)R6_CONFIG_ID,f->fpga_config_id,f->fpga_status_revision,f->fpga_command_ttl_ms,s5_reason_zh(v->navigation.reason),a->front_approach?"true":"false",a->nf_emergency?"true":"false",a->turn_hold?"true":"false");
 s3_distance(b,a->policy_revision>=5&&a->policy_revision<=9,a->front_clearance_m);add(b,",\"reverse_target_m\":%.6g,\"side_state\":[%u,%u],\"side_clearance_m\":[",isfinite(a->reverse_target_m)?a->reverse_target_m:0,a->side_state[0],a->side_state[1]);s3_distance(b,a->policy_revision>=5&&a->policy_revision<=9,a->side_clearance_m[0]);add(b,",");s3_distance(b,a->policy_revision>=5&&a->policy_revision<=9,a->side_clearance_m[1]);add(b,"],\"front_corridor_clearance_m\":");s3_distance(b,a->policy_revision>=5&&a->policy_revision<=9,a->front_corridor_clearance_m);
 add(b,",\"rejected_action\":{\"reason\":\"%s\",\"v\":%.6g,\"w\":%.6g},\"raw_diagnostics\":{\"available\":%s,\"source\":\"NCD5_FPGA_RAW\",\"received_us\":\"%llu\",\"fpga_uptime_ms\":%u,\"raw_mm\":[",safe_reason(a->rejected_reason),isfinite(a->rejected_v_m_s)?a->rejected_v_m_s:0,isfinite(a->rejected_w_rad_s)?a->rejected_w_rad_s:0,raw?"true":"false",(unsigned long long)f->sonar_diagnostic_time_us,f->sonar_source_uptime_ms);
 for(unsigned i=0;i<3;i++){if(i)add(b,",");if(raw&&(f->sonar_raw_available&(1u<<i)))add(b,"%u",f->sonar_raw_mm[i]);else add(b,"null");}
 add(b,"],\"nonzero_seq\":[%u,%u,%u],\"sample_age_us\":[%u,%u,%u],\"observation_block\":%u,\"flags\":%u,\"effective_pwm\":[",f->sonar_nonzero_sequence[0],f->sonar_nonzero_sequence[1],f->sonar_nonzero_sequence[2],f->sonar_raw_age_us[0],f->sonar_raw_age_us[1],f->sonar_raw_age_us[2],f->sonar_observation_block,f->sonar_diagnostic_flags);
 for(unsigned i=0;i<4;i++){if(i)add(b,",");if(raw)add(b,"%u",f->effective_pwm[i]);else add(b,"null");}add(b,"],\"ncd_zero_mask\":%u,\"ncd_fresh\":%u,\"ncd_near\":%u,\"ncd_pending\":%u,\"sampling_kind\":\"FPGA_RAW_SOURCE_AGE_AND_RK_USERSPACE_DEQUEUE_TIME\"",f->sonar_diagnostic_zero_mask,f->sonar_diagnostic_fresh,f->sonar_diagnostic_near,f->sonar_diagnostic_pending);add(b,",\"pwm_kind\":\"POST_GATE_COMPARE_THRESHOLD_NOT_ANALOG_MEASUREMENT\",\"pwm_limit\":1750,\"pwm_period\":2500},\"local_path\":[");
 uint32_t count=v->navigation.local_path_count;if(count>33||!v->navigation.local_path)count=0;
 for(uint32_t i=0;i<count;i++){const lab_real_xy*q=&v->navigation.local_path[i];if(!isfinite(q->x_m)||!isfinite(q->y_m))break;add(b,"%s[%.9g,%.9g]",i?",":"",q->x_m,q->y_m);}add(b,"]");
}
static void s3_diagnostics(buffer*b,const lab_real_runtime_view*v,uint64_t now){
 s5_diagnostics(b,v,now);s6_diagnostics(b,&v->navigation.avoidance);r4_diagnostics(b,&v->navigation.avoidance);r6_diagnostics(b,&v->navigation.avoidance,now,v->has_goal||v->enabled||v->pending_enable||v->navigation.cruise_mode);
 const s3_nearfield_view*n=&v->navigation.nearfield;const s3_avoidance_view*a=&v->navigation.avoidance;
 add(b,",\"task_state\":{\"active\":%s,\"wheels\":\"%s\"}",(v->has_goal||v->enabled||v->pending_enable||v->navigation.cruise_mode)?"true":"false",navigation_feedback_fresh(v)?(navigation_wheels_moving(v)?"MOVING":"STOPPED"):"UNKNOWN");
 int time_ok=n->scan_time_us&&n->scan_time_us<=now;long long age=time_ok?(long long)((now-n->scan_time_us)/1000):-1;
 add(b,",\"nearfield\":{\"version\":1,\"valid\":%s,\"stale\":%s,\"scan_time_us\":\"%llu\",\"age_ms\":%lld,\"points\":%u,\"focus_points\":%u,\"raw_points\":%u,\"held_points\":%u,\"updates\":%llu,\"rejected_scans\":%llu,\"front_m\":",n->valid&&time_ok&&age<=300?"true":"false",n->stale||!time_ok||age>300?"true":"false",(unsigned long long)n->scan_time_us,age,n->point_count,n->focus_points,n->raw_points,n->held_points,(unsigned long long)n->updates,(unsigned long long)n->rejected_scans);
 s3_distance(b,time_ok,n->front_m);add(b,",\"left_m\":");s3_distance(b,time_ok,n->left_m);add(b,",\"right_m\":");s3_distance(b,time_ok,n->right_m);add(b,",\"rear_m\":");s3_distance(b,time_ok,n->rear_m);
 add(b,",\"forward_returns\":%u,\"forward_bins\":%u,\"forward_corridor_returns\":%u,\"forward_self_returns\":%u,\"forward_reason\":\"%s\",\"forward_return_m\":",n->forward_returns,n->forward_bins,n->forward_corridor_returns,n->forward_self_returns,safe_reason(n->forward_reason));
 s3_distance(b,time_ok&&age<=300,n->forward_return_m);add(b,",\"forward_corridor_m\":");s3_distance(b,time_ok&&age<=300,n->forward_corridor_m);
 add(b,",\"bracket_cleared\":\"%llu\",\"blocked\":[%u,%u,%u,%u],\"reason\":\"%s\",\"hazards\":[",(unsigned long long)n->bracket_cleared,!!n->front_blocked,!!n->left_blocked,!!n->right_blocked,!!n->rear_blocked,safe_reason(n->reason));
 unsigned written=0;for(unsigned i=0;i<n->hazard_count&&i<S3_HAZARD_VIEW_MAX;i++){const s3_hazard_point*p=&n->hazards[i];if(!isfinite(p->x_m)||!isfinite(p->y_m)||fabs(p->x_m)>100||fabs(p->y_m)>100||!p->time_us||p->time_us>now)continue;add(b,"%s[%.6g,%.6g,\"%llu\"]",written++?",":"",p->x_m,p->y_m,(unsigned long long)p->time_us);}
 add(b,"],\"storage\":{\"cells\":%u,\"recent\":%u,\"pending\":%u,\"unknown\":%u,\"capacity\":%u,\"bytes\":%u,\"expired\":\"%llu\",\"ray_cleared\":\"%llu\",\"window_evicted\":\"%llu\"},\"query\":{\"code\":%u,\"reason\":\"%s\",\"source\":%u,\"checked_us\":\"%llu\",\"source_us\":\"%llu\",\"v\":%.6g,\"w\":%.6g,\"horizon_s\":%.6g,\"point\":[%.6g,%.6g],\"ttc_s\":%.6g}",
 n->history_cells,n->history_recent,n->history_pending,n->history_unknown,n->history_capacity,n->storage_bytes,
 (unsigned long long)n->history_expired,(unsigned long long)n->ray_cleared,(unsigned long long)n->window_evicted,
 n->query_code,safe_reason(n->query_reason),n->query_source,(unsigned long long)n->query_time_us,(unsigned long long)n->query_source_time_us,
 isfinite(n->query_v_m_s)?n->query_v_m_s:0,isfinite(n->query_w_rad_s)?n->query_w_rad_s:0,isfinite(n->query_horizon_s)?n->query_horizon_s:0,
 isfinite(n->query_x_m)?n->query_x_m:0,isfinite(n->query_y_m)?n->query_y_m:0,isfinite(n->query_ttc_s)?n->query_ttc_s:0);
 add(b,"},\"avoidance\":{\"version\":1,\"active\":%s,\"stage\":%u,\"event_id\":%u,\"direction\":%d,\"turn_target_rad\":%.6g,\"turn_progress_rad\":%.6g,\"backup_target_m\":%.6g,\"backup_progress_m\":%.6g,\"completed_actions\":%u,\"interrupted_actions\":%u,\"reason\":\"%s\"}",a->active?"true":"false",a->stage,a->event_id,a->direction,isfinite(a->turn_target_rad)?a->turn_target_rad:0,isfinite(a->turn_progress_rad)?a->turn_progress_rad:0,isfinite(a->backup_target_m)?a->backup_target_m:0,isfinite(a->backup_progress_m)?a->backup_progress_m:0,a->completed_actions,a->interrupted_actions,safe_reason(a->reason));
 int body_pose_ok=navigation_pose_valid(v)&&v->localization.pose_time_us<=now&&now-v->localization.pose_time_us<=CONT_ODOM_US;
 add(b,",\"body_pose\":");if(body_pose_ok)add(b,"[%.9g,%.9g,%.9g]",v->localization.pose.x_m,v->localization.pose.y_m,v->localization.pose.yaw_rad);else add(b,"null");
 /* 动作流5Hz、完整地图1Hz。明确源位姿年龄，显示端不再把完整地图的接收时刻当作最新位姿时刻。 */
 add(b,",\"body_pose_age_ms\":%.3f",body_pose_ok?(double)(now-v->localization.pose_time_us)/1000.0:-1.0);
}
static int snapshot_json(const lab_real_runtime_view*v,const lab_pose*display_pose,uint64_t display_time,int pose_valid,char*out,size_t cap,size_t*written){
 if(written)*written=0;if(!out||!cap||!v)return 0;out[0]='\0';
 const lab_real_grid_view*m=&v->localization.map;
 if(!lab_real_grid_valid(m))return 0;
 if(m->width>1000||m->height>1000||m->trace_count>LAB_TRACE_MAX||v->navigation.path_count>LAB_REAL_PATH_MAX)return 0;
 buffer b={out,cap,0,1};
 const char*reason=v->reason;for(const char*p=reason;*p;p++)if(!((*p>='A'&&*p<='Z')||(*p>='0'&&*p<='9')||*p=='_')){reason="UNKNOWN";break;}
 if(v->has_goal&&(!isfinite(v->goal.x_m)||!isfinite(v->goal.y_m)))return 0;
 add(&b,"{\"schema\":1,\"test_match_age_ms\":1000,\"map_id\":\"%llu\",\"map_version\":\"%llu\",\"width\":%u,\"height\":%u,\"resolution\":%.9g,\"origin_x\":%.9g,\"origin_y\":%.9g,\"state\":%u,\"enabled\":%s,\"reason\":\"%s\",\"has_goal\":%s,\"goal\":[%.9g,%.9g],\"pose\":",(unsigned long long)m->map_id,(unsigned long long)m->map_version,m->width,m->height,m->resolution_m,m->origin_x_m,m->origin_y_m,v->localization.state,v->enabled?"true":"false",reason,v->has_goal?"true":"false",v->goal.x_m,v->goal.y_m);
 if(display_pose)add(&b,"[%.9g,%.9g,%.9g]",display_pose->x_m,display_pose->y_m,display_pose->yaw_rad);else add(&b,"null");
 long long pose_age=display_pose&&display_time&&v->time_us>=display_time?(long long)((v->time_us-display_time)/1000):-1;
 add(&b,",\"pose_valid\":%s,\"pose_retained\":%s,\"pose_time_us\":\"%llu\",\"pose_age_ms\":%lld,\"map_retained\":%s,\"recovery_state\":\"%s\",\"cells\":\"",pose_valid?"true":"false",display_pose&&!pose_valid?"true":"false",(unsigned long long)(display_pose?display_time:0),pose_age,pose_valid?"false":"true",safe_reason(v->localization.reason));
 bytes64(&b,m->cells,(size_t)m->width*m->height);add(&b,"\",\"hardware_calls\":%llu,\"nav_reason\":\"%s\",\"nav_blocking\":\"%s\",\"localizer_reason\":\"%s\",\"trace\":[",(unsigned long long)v->hardware_calls,safe_reason(v->navigation.reason),safe_reason(v->navigation.blocking_reason),safe_reason(v->localization.reason));
 if(m->trace_count&&!m->trace)b.ok=0;
 for(uint32_t i=0;i<m->trace_count&&b.ok;i++){if(!isfinite(m->trace[i].x_m)||!isfinite(m->trace[i].y_m)){b.ok=0;break;}add(&b,"%s[%.9g,%.9g]",i?",":"",m->trace[i].x_m,m->trace[i].y_m);}
 add(&b,"],\"geometry\":{\"length_m\":%.6g,\"width_m\":%.6g,\"margin_m\":%.6g,\"local_resolution_m\":%.6g,\"lidar_offset_verified\":%s,\"narrow\":%s,\"collision\":%u,\"collision_text\":\"%s\"},\"path\":[",v->navigation.body_length_m,v->navigation.body_width_m,v->navigation.margin_m,v->navigation.local_resolution_m,v->navigation.lidar_offset_verified?"true":"false",v->navigation.narrow_passage?"true":"false",v->navigation.passage_reason,passage_reason_text((int)v->navigation.passage_reason));
 /* 地图内容修订不改变世界坐标，重规划时保留旧路线显示，具体运动由控制器重新核对。
 */
 uint32_t paths=v->has_goal?v->navigation.path_count:0;
 if(paths&&!v->navigation.path)b.ok=0;
 for(uint32_t i=0;i<paths&&b.ok;i++){if(!isfinite(v->navigation.path[i].x_m)||!isfinite(v->navigation.path[i].y_m)){b.ok=0;break;}add(&b,"%s[%.9g,%.9g]",i?",":"",v->navigation.path[i].x_m,v->navigation.path[i].y_m);}
 int fresh=navigation_feedback_fresh(v);
 long long age=v->feedback.time_us&&v->time_us>=v->feedback.time_us?(long long)((v->time_us-v->feedback.time_us)/1000):-1;
 long long stop_age=v->last_stop_us&&v->time_us>=v->last_stop_us?(long long)((v->time_us-v->last_stop_us)/1000):-1;
 add(&b,"],\"path_pending_recheck\":%s,\"vehicle_status\":\"%s\",\"phase_text\":\"%s\",\"published_us\":\"%llu\",\"last_stop_reason\":\"%s\",\"last_stop_detail\":\"%s\",\"last_stop_us\":%llu,\"last_stop_age_ms\":%lld,\"last_stop_recovered\":%s,\"feedback_fresh\":%s,\"feedback_age_ms\":%lld,\"feedback_faults\":%u,\"feedback_reason\":\"%s\",\"wheel_rpm\":[",paths&&v->navigation.map_version!=m->map_version?"true":"false",navigation_vehicle_status(v),safe_reason(v->navigation.phase_text),(unsigned long long)v->time_us,safe_reason(v->last_stop_reason),safe_reason(v->last_stop_detail),(unsigned long long)v->last_stop_us,stop_age,v->last_stop_us&&v->enabled&&navigation_wheels_moving(v)?"true":"false",fresh?"true":"false",age,v->feedback.faults,safe_reason(v->feedback.reason));
 for(unsigned i=0;i<4;i++){if(i)add(&b,",");if(fresh&&isfinite(v->feedback.actual_rpm[i]))add(&b,"%.3f",v->feedback.actual_rpm[i]);else add(&b,"null");}
 add(&b,"],\"last_wheel_rpm\":[");
 for(unsigned i=0;i<4;i++)add(&b,"%s%.3f",i?",":"",isfinite(v->feedback.actual_rpm[i])?v->feedback.actual_rpm[i]:0);
 add(&b,"],\"target_wheel_rpm\":[");
 for(unsigned i=0;i<4;i++)add(&b,"%s%.3f",i?",":"",isfinite(v->protected_output.wheel_rpm[i])?v->protected_output.wheel_rpm[i]:0);
 add(&b,"],\"requested_speed\":[%.4f,%.4f],\"recovery_distance_m\":%.4f,\"recovery_target_m\":%.4f,\"recovery_attempts\":%u,\"cruise_mode\":%s,\"local_probe\":%s,\"can_start_local\":%s,\"backup_requests\":%u,",v->request.linear_m_s,v->request.angular_rad_s,v->navigation.recovery_distance_m,v->navigation.recovery_target_m,v->navigation.recovery_attempts,v->navigation.cruise_mode?"true":"false",v->navigation.local_probe?"true":"false",pose_valid&&v->navigation.can_start_local?"true":"false",v->navigation.backup_requests);
 const lab_localization_view*l=&v->localization;
 add(&b,"\"recovery_diagnostics\":{\"state\":%u,\"attempts\":%u,\"trials\":%u,\"total_trials\":%u,\"ambiguous\":%u,\"candidate_failures\":%u,\"best_score\":%.6g,\"second_score\":%.6g,\"search_m\":%.6g,\"search_rad\":%.6g},",l->recovery_state,l->recovery_attempts,l->recovery_trials,l->recovery_total_trials,l->recovery_ambiguous,l->recovery_candidate_failures,isfinite(l->recovery_best_score)?l->recovery_best_score:0,isfinite(l->recovery_second_score)?l->recovery_second_score:0,isfinite(l->recovery_search_m)?l->recovery_search_m:0,isfinite(l->recovery_search_rad)?l->recovery_search_rad:0);
 add(&b,"\"provisional_route\":%s,\"rear_reason\":\"%s\",\"candidate_diagnostics\":{\"sonar_rejections\":%u,\"occupied_rejections\":%u,\"unknown_rejections\":%u,\"accepted\":%u,\"observed_fraction\":%.6g,\"budget_rejections\":%u},",v->navigation.provisional_route?"true":"false",safe_reason(v->navigation.rear_reason),v->navigation.candidate_sonar_rejections,v->navigation.candidate_occupied_rejections,v->navigation.candidate_unknown_rejections,v->navigation.candidate_accepted,isfinite(v->navigation.observed_fraction)?v->navigation.observed_fraction:0,v->navigation.candidate_budget_rejections);
 add(&b,"\"resume_requires_start\":%s,",v->resume_requires_start?"true":"false");
 add(&b,"\"map_reference_version\":\"%llu\",\"map_reference_pending\":%s,",(unsigned long long)l->map_reference_version,l->map_reference_pending?"true":"false");
 add(&b,"\"match_diagnostics\":{\"input_points\":%u,\"inside_points\":%u,\"outside_points\":%u,\"reference_points\":%u,\"close_points\":%u,\"sector_count\":%u,\"rmse_m\":%.6g,\"robust_rmse_m\":%.6g,\"ratio\":%.6g,\"coverage\":%.6g,\"map_expansions\":%llu,\"map_growth_limited\":%llu,\"map_geometry_version\":%llu,\"map_allocated_bytes\":%llu},\"sonar_mm\":[",l->match_input_points,l->match_inside_points,l->match_outside_points,l->match_reference_points,l->match_close_points,l->match_sector_count,isfinite(l->match_rmse_m)?l->match_rmse_m:0,isfinite(l->match_robust_rmse_m)?l->match_robust_rmse_m:0,isfinite(l->match_ratio)?l->match_ratio:0,isfinite(l->match_coverage)?l->match_coverage:0,(unsigned long long)l->map_expansions,(unsigned long long)l->map_growth_limited,(unsigned long long)l->map_geometry_version,(unsigned long long)l->map_allocated_bytes);
 for(unsigned i=0;i<3;i++){if(i)add(&b,",");if(fresh&&(v->feedback.sonar_valid&(1u<<i)))add(&b,"%u",v->feedback.sonar_mm[i]);else add(&b,"null");}
 add(&b,"],\"sonar_samples\":[");
 for(unsigned i=0;i<3;i++){
  int known=fresh&&(v->feedback.sonar_valid&(1u<<i));unsigned age=v->feedback.sonar_age_bucket[i]*200;
  add(&b,"%s{\"age_known\":%s,\"nonzero_age_min_ms\":",i?",":"",known?"true":"false");if(known)add(&b,"%u",age);else add(&b,"null");
  add(&b,",\"nonzero_age_max_ms\":");if(known&&v->feedback.sonar_age_bucket[i]<15)add(&b,"%u",age+200);else add(&b,"null");
  add(&b,",\"raw_zero_ignored\":%s}",fresh&&(v->feedback.sonar_raw_zero_mask&(1u<<i))?"true":"false");
 }
 unsigned motor_phase=0,motor_retries=0,motor_stop=0;int motor_known=navigation_shared_motor_state(v->time_us,&motor_phase,&motor_retries,&motor_stop);
 add(&b,"],\"sonar_valid\":%u,\"sonar_fresh\":%u,\"sonar_near\":%u,\"sonar_degraded\":%u,\"sonar_front_stable\":%s,\"local_stop_sources\":%u,\"pwm_limit_percent\":70,\"actual_pwm_percent\":null,\"motor_recovery_known\":%s,\"motor_recovery_phase\":",fresh?v->feedback.sonar_valid:0,fresh?v->feedback.sonar_fresh:0,fresh?v->feedback.sonar_near:0,v->feedback.sonar_degraded,fresh&&v->feedback.sonar_front_stable?"true":"false",v->feedback.local_stop_sources,motor_known?"true":"false");
 if(motor_known)add(&b,"%u,\"motor_recovery_retries\":%u,\"motor_stop_reason\":%u",motor_phase,motor_retries,motor_stop);else add(&b,"null,\"motor_recovery_retries\":null,\"motor_stop_reason\":null");
 add(&b,",\"route_complete\":%s,\"route_partial\":%s,\"route_search_budget\":%u,\"route_kind\":%u,\"route_reason\":\"%s\"",v->navigation.route_complete?"true":"false",v->navigation.route_partial?"true":"false",v->navigation.route_search_budget,v->navigation.route_kind,safe_reason(v->navigation.route_reason));
 char motion[8192];size_t motion_bytes=0;
 if(phone_nav_motion_json(v,v->time_us,motion,sizeof(motion),&motion_bytes))add(&b,",\"motion_preview\":%s",motion);
 s3_diagnostics(&b,v,v->time_us);
 add(&b,"}");
 if(!b.ok){out[0]='\0';return 0;}if(written)*written=b.used;return 1;
}
int phone_nav_snapshot_json_cached(phone_nav_snapshot_state*s,const lab_real_runtime_view*v,char*out,size_t cap,size_t*written){
 if(!s||!v)return phone_nav_snapshot_json(v,out,cap,written);
 const lab_real_context*c=&v->context;
 if(s->map_id!=v->localization.map.map_id||s->context.source!=c->source||s->context.clock!=c->clock||s->context.timeline_id!=c->timeline_id||s->context.origin_epoch!=c->origin_epoch){memset(s,0,sizeof(*s));s->context=*c;s->map_id=v->localization.map.map_id;}
 int valid=navigation_pose_valid(v);
 if(valid){s->pose=v->localization.pose;s->pose_time_us=v->localization.pose_time_us;s->have_pose=1;}
 if(s->have_pose&&s->pose_time_us>v->time_us){s->have_pose=0;s->pose_time_us=0;}
 return snapshot_json(v,s->have_pose?&s->pose:NULL,s->pose_time_us,valid,out,cap,written);
}
int phone_nav_snapshot_json(const lab_real_runtime_view*v,char*out,size_t cap,size_t*written){
 if(!v)return snapshot_json(v,NULL,0,0,out,cap,written);
 int valid=navigation_pose_valid(v);const lab_pose*p=&v->localization.pose;
 int available=v->localization.pose_time_us&&v->time_us>=v->localization.pose_time_us&&isfinite(p->x_m)&&isfinite(p->y_m)&&isfinite(p->yaw_rad);
 return snapshot_json(v,available?p:NULL,available?v->localization.pose_time_us:0,valid,out,cap,written);
}
int phone_nav_motion_json(const lab_real_runtime_view*v,uint64_t now,char*out,size_t cap,size_t*written){
 if(written)*written=0;if(!v||!out||!cap)return 0;out[0]=0;s2_motion_preview p;s2_motion_preview_make(v,now,&p);buffer b={out,cap,0,1};
 unsigned remaining=p.valid&&p.expires_us>now?(unsigned)((p.expires_us-now)/1000):0;
 add(&b,"{\"schema\":1,\"map_id\":\"%llu\",\"map_version\":\"%llu\",\"published_us\":\"%llu\",\"valid\":%s,\"source\":\"%s\",\"reason\":\"%s\",\"command_generated_us\":\"%llu\",\"command_expires_us\":\"%llu\",\"pose_time_us\":\"%llu\",\"remaining_ms\":%u,\"v\":%.9g,\"w\":%.9g,\"horizon_s\":%.3f,\"points\":[",(unsigned long long)v->localization.map.map_id,(unsigned long long)v->localization.map.map_version,(unsigned long long)now,p.valid?"true":"false",p.source,p.reason,(unsigned long long)p.generated_us,(unsigned long long)p.expires_us,(unsigned long long)p.pose_us,remaining,p.v,p.w,p.horizon);
 for(unsigned i=0;i<p.count;i++)add(&b,"%s[%.9g,%.9g,%.9g]",i?",":"",p.points[i].x,p.points[i].y,p.points[i].yaw);
 add(&b,"]");s3_diagnostics(&b,v,now);
 /* 5Hz动作诊断携带真实反馈源年龄，避免1Hz地图后半秒出现假过期。
  * 只显示，不修改任何运动/反馈门；新失效反馈不借旧地图复活。 */
 int feedback_ok=v->feedback.time_us&&v->feedback.time_us<=now&&now-v->feedback.time_us<=CONT_FEEDBACK_US;
 add(&b,",\"live_feedback\":{\"version\":1,\"fresh\":%s,\"age_ms\":%.3f,\"wheel_rpm\":[",feedback_ok?"true":"false",v->feedback.time_us&&v->feedback.time_us<=now?(double)(now-v->feedback.time_us)/1000.0:-1.0);
 for(unsigned i=0;i<4;i++){if(i)add(&b,",");if(feedback_ok&&isfinite(v->feedback.actual_rpm[i]))add(&b,"%.3f",v->feedback.actual_rpm[i]);else add(&b,"null");}
 add(&b,"],\"target_wheel_rpm\":[");for(unsigned i=0;i<4;i++)add(&b,"%s%.3f",i?",":"",isfinite(v->protected_output.wheel_rpm[i])?v->protected_output.wheel_rpm[i]:0);add(&b,"]}}");
 if(!b.ok){out[0]=0;return 0;}if(written)*written=b.used;return 1;
}
int phone_nav_motion_export(const lab_real_runtime_view*v,const char*directory,uint64_t now){
#if defined(__linux__)
 static uint64_t last_us,last_map;struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;
 uint64_t actual=(uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;if(actual<now)return 0;now=actual;
 if(!v||!directory||directory[0]!='/'||(last_map==v->localization.map.map_id&&last_us&&now>=last_us&&now-last_us<100000))return 0;
 char data[16384],temporary[80];size_t bytes=0;if(!phone_nav_motion_json(v,now,data,sizeof(data),&bytes))return 0;
 int dirfd=open(directory,O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW);if(dirfd<0)return 0;struct stat st;
 if(fstat(dirfd,&st)||st.st_uid!=geteuid()||(st.st_mode&0077)){close(dirfd);return 0;}
 snprintf(temporary,sizeof(temporary),"navigation-motion.%lu.tmp",(unsigned long)getpid());
 int fd=openat(dirfd,temporary,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0){close(dirfd);return 0;}
 size_t done=0;while(done<bytes){ssize_t n=write(fd,data+done,bytes-done);if(n<=0)break;done+=(size_t)n;}
 int closed=close(fd);int ok=done==bytes&&!closed&&!renameat(dirfd,temporary,dirfd,"navigation-motion.json");if(!ok)unlinkat(dirfd,temporary,0);close(dirfd);
 if(ok){last_us=now;last_map=v->localization.map.map_id;}return ok;
#else
 (void)v;(void)directory;(void)now;return 0;
#endif
}
