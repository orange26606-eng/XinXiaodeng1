/* 路线优先、带类型几何及短退，不访问设备。 */
#ifndef ACG_R4_NEAR_SCENE_INC
#define ACG_R4_NEAR_SCENE_INC
#include "nav_scene_classifier.inc"
static int r4_task_active(const r4_scene_input*i){return i->goal_active||i->cruise_active;}
static int r4_near_stopped(const r4_scene_input*i){return fabs(i->measured_v)<=.025&&fabs(i->measured_w)<=.1;}
static int r4_same_forward_arc(double v,double w){return v>0&&fabs(w)>1e-9&&v+1e-9>=fabs(w)*.145;}
static int r4_away_from_side(double v,double w,unsigned side){return r4_same_forward_arc(v,w)&&(side?w>0:w<0);}
static int r4_result(const r4_scene_input*i){return i->check_result?*i->check_result:R6_CHECK_INVALID;}
static int r4_record_result(const r4_scene_input*i,int value){if(i->check_result){if(*i->check_result==R6_CHECK_BUDGET)return R6_CHECK_BUDGET;*i->check_result=value;}return value;}
static int r4_check(const r4_scene_input*i,double v,double w,double horizon){
 if(r4_result(i)==R6_CHECK_BUDGET)return R6_CHECK_BUDGET;
 if(!isfinite(v)||!isfinite(w)||!isfinite(horizon)||horizon<=0)return r4_record_result(i,R6_CHECK_INVALID);
 if(v>0){if(!r4_direction_usable(&i->direction[0],i->now_us,CONT_SCAN_US))return r4_record_result(i,R6_CHECK_STALE);if(i->direction[0].clearance_m<R6_FRONT_FORWARD_STOP_MM/1000.)return r4_record_result(i,R6_CHECK_WIRE_REJECT);}
 if(v<0){for(unsigned j=1;j<4;j++)if(!r4_direction_usable(&i->direction[j],i->now_us,CONT_SCAN_US))return r4_record_result(i,R6_CHECK_STALE);}
 else if(fabs(w)>1e-9){
  if(r4_same_forward_arc(v,w)){unsigned side=w>0?1u:2u;if(!r4_direction_usable(&i->direction[side],i->now_us,CONT_SCAN_US))return r4_record_result(i,R6_CHECK_STALE);if(i->direction[side].clearance_m<R6_SIDE_TURN_MM/1000.)return r4_record_result(i,R6_CHECK_WIRE_REJECT);}
  else for(unsigned j=0;j<3;j++){if(!r4_direction_usable(&i->direction[j],i->now_us,CONT_SCAN_US))return r4_record_result(i,R6_CHECK_STALE);if(i->direction[j].clearance_m<R6_SIDE_TURN_MM/1000.)return r4_record_result(i,R6_CHECK_WIRE_REJECT);}
 }
 if(!i->motion_check)return r4_record_result(i,R6_CHECK_INVALID);
 int result=i->motion_check(i->motion_context,v,w,horizon);if(result>R6_CHECK_CLEAR||result<R6_CHECK_INVALID)result=R6_CHECK_INVALID;return r4_record_result(i,result);
}
static double r4_action_speed_cap(const r4_scene_state*s,const r4_scene_input*i,double front_cap,double v,double w){
 if(v<=0)return INFINITY;double cap=isfinite(front_cap)?fmax(0,front_cap):0;if(i->have_speed_limit)cap=r4_min(cap,isfinite(i->speed_limit_m_s)?fmax(0,i->speed_limit_m_s):0);
 for(unsigned side=0;side<2;side++){const r4_direction*d=&i->direction[side+1];if(!r4_direction_usable(d,i->now_us,CONT_SCAN_US)||r4_away_from_side(v,w,side))continue;if(d->clearance_m<=.020)cap=r4_min(cap,d->clearance_m<=.015?.12:.25);if(s&&s->side_hits[side]==1)cap=r4_min(cap,.12);}return cap;
}
static void r4_stage_enter(r4_scene_state*s,const r4_scene_input*i,uint32_t stage){s->stage=stage;s->phase_since=i->now_us;s->execution_since=0;s->phase_x=i->x_m;s->phase_y=i->y_m;s->phase_yaw=i->yaw_rad;s->last_x=i->x_m;s->last_y=i->y_m;s->turn_progress=0;s->reverse_progress=0;s->reverse_counted=0;s->phase_entered=1;s->fitted_target_m=0;}
/* 只消费真实新姿态。总计角单调保存，供内部及root最终预算回滚幂等地保留已执行事实。 */
static void r4_observe_executed_turn(r4_scene_state*s,const r4_scene_input*i){
 if(!i->pose_valid||!isfinite(i->yaw_rad)||!i->pose_source_us||i->pose_source_us>i->now_us||i->pose_source_us<=s->last_motion_source)return;
 if(s->last_motion_source&&i->execution_ready&&(s->stage==R4_PROBE_TURN||s->stage==R4_TURN_HOLD)&&s->turn_seen&&s->turn_side<2){double angle=fabs(r4_wrap(i->yaw_rad-s->last_yaw));s->turn_progress+=angle;s->side_angle[s->turn_side]+=angle;s->executed_turn_angle[s->turn_side]+=angle;}
 s->last_yaw=i->yaw_rad;s->last_motion_source=i->pose_source_us;
}
static void r4_owned(r4_scene_output*o,double v,double w,const char*reason){o->owns_motion=1;o->owner=R4_OWNER_NEAR;o->v=v;o->w=w;o->planner_allowed=0;o->reason=reason;}
static void r4_wait(r4_scene_output*o,const char*reason){r4_owned(o,0,0,reason);o->owner=R4_OWNER_WAIT;}
static const char*r4_check_reason(int result){switch(result){case R6_CHECK_BUDGET:return "R6_COMPUTE_BUDGET_WAIT";case R6_CHECK_UNOBSERVED:return "R6_ACTION_UNOBSERVED";case R6_CHECK_STALE:return "R6_ACTION_SOURCE_STALE";case R6_CHECK_POSE_INVALID:return "R6_ACTION_POSE_INVALID";case R6_CHECK_WIRE_REJECT:return "R6_ACTION_HARD_GATE_REJECTED";case R6_CHECK_INVALID:return "R6_ACTION_CHECK_INVALID";default:return "R6_ACTION_OCCUPIED";}}
static int r4_checked(r4_scene_output*o,const r4_scene_input*i,double v,double w,const char*reason){
 if(v>0){double cap=r4_action_speed_cap(NULL,i,o->speed_cap,v,w);if(cap<=0){r4_wait(o,"R6_BRAKING_SPEED_LIMIT_WAIT");return 0;}if(v>cap){double scale=cap/v;v*=scale;w*=scale;}}
 int result=r4_check(i,v,w,.8);if(result!=R6_CHECK_CLEAR){r4_wait(o,r4_check_reason(result));return 0;}r4_owned(o,v,w,reason);return 1;
}
static double r4_current_front_cap(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i){if(!r4_direction_usable(&i->direction[0],i->now_us,c->direction_max_age_us))return 0;double cap=r4_speed_band(s,c,i);if(i->have_speed_limit)cap=r4_min(cap,isfinite(i->speed_limit_m_s)?fmax(0,i->speed_limit_m_s):0);return cap;}
static void r4_observed_hold(r4_scene_state*s,const r4_scene_input*i,r4_scene_output*o,const char*reason){s->stage=R4_HOLD;s->hold_retry_us=i->now_us;s->hold_retry_source=0;for(unsigned j=0;j<4;j++)if(i->direction[j].source_us>s->hold_retry_source)s->hold_retry_source=i->direction[j].source_us;o->stage=s->stage;r4_wait(o,reason);}
static int r4_hold_has_new_data(const r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i){if(i->now_us<s->hold_retry_us||i->now_us-s->hold_retry_us<c->switch_hold_us)return 0;for(unsigned j=0;j<4;j++)if(i->direction[j].source_us>s->hold_retry_source)return 1;return 0;}
static int r4_route_side(const r4_scene_input*i,const r4_scene_state*s){if(fabs(i->desired_w)>1e-6)return i->desired_w>0?0:1;if(fabs(i->goal_bearing_rad)>1e-6)return i->goal_bearing_rad>0?0:1;if(s->turn_seen)return (int)s->turn_side;return i->direction[1].clearance_m>=i->direction[2].clearance_m?0:1;}
static int r4_side_select(const r4_scene_input*i,const r4_scene_config*c,const r4_scene_state*s,double minimum){unsigned preferred=(unsigned)r4_route_side(i,s);for(unsigned k=0;k<2;k++){unsigned side=k?1-preferred:preferred;if(s->side_angle[side]+1e-9>=c->explore_side_budget_rad)continue;const r4_direction*d=&i->direction[side+1];if(!r4_direction_usable(d,i->now_us,c->direction_max_age_us)||d->clearance_m<minimum)continue;int result=r4_check(i,0,side?-c->pivot_w:c->pivot_w,.8);if(result==R6_CHECK_CLEAR)return (int)side;if(result==R6_CHECK_BUDGET)return -1;}return -1;}
static int r4_prepare_turn(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,unsigned side,r4_scene_output*o){
 if(!(s->stage==R4_PROBE_TURN&&s->turn_side==side)&&!r4_near_stopped(i)){r4_owned(o,0,0,"R6_BRAKE_BEFORE_DIRECTION_CHANGE");o->transition=1;return 0;}
 if(s->stage!=R4_PROBE_TURN||s->turn_side!=side){r4_stage_enter(s,i,R4_PROBE_TURN);s->turn_side=side;s->turn_seen=1;s->side_selected_us=i->now_us;}o->stage=s->stage;return r4_checked(o,i,0,side?-c->pivot_w:c->pivot_w,"R6_ROUTE_SIDE_TURN");
}
static void r4_prepare_retreat(r4_scene_state*s,const r4_scene_input*i,r4_scene_output*o,const char*reason){if(s->stage!=R4_CHECK_REAR)r4_stage_enter(s,i,R4_CHECK_REAR);o->stage=s->stage;r4_owned(o,0,0,reason);}
static void r4_explore_start(r4_scene_state*s,const r4_scene_input*i){s->explore_started=1;s->round_counted=0;s->explore_since=i->now_us;s->explore_active_us=0;s->explore_x=i->x_m;s->explore_y=i->y_m;s->side_angle[0]=s->side_angle[1]=0;s->attempted_sides=0;s->open_since=0;s->hold_clears=0;s->hold_source=0;s->hold_seq=0;r4_stage_enter(s,i,R4_GAP_MEASURE);}
static void r4_brakes(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,r4_scene_output*o){
 const r4_direction*f=&i->direction[0];int front_ok=r4_direction_usable(f,i->now_us,c->direction_max_age_us);
 if(front_ok&&f->clearance_m<=c->front_brake_m){if(!s->front_brake)s->front_brake_since=i->now_us;s->front_brake=1;s->front_clears=0;}
 if(front_ok&&r4_new_sample(f,&s->front_source,&s->front_seq)){
  if(f->clearance_m>=R6_FRONT_RELEASE_MM/1000.){if(s->front_clears<R6_FRONT_RELEASE_SAMPLES)s->front_clears++;if(s->front_clears>=R6_FRONT_RELEASE_SAMPLES&&s->front_brake){s->front_brake=0;s->resume_limited=0;s->speed_cap=r4_front_band(f->clearance_m);s->initialized=1;s->band_since=0;}}else s->front_clears=0;
 }
 int turn=fabs(i->measured_w)>c->w_straight,forward_arc=r4_same_forward_arc(i->measured_v,i->measured_w);unsigned turn_mask=forward_arc?(i->measured_w>0?1u:2u):3u;
 for(unsigned side=0;side<2;side++){
  const r4_direction*d=&i->direction[side+1];if(!r4_direction_usable(d,i->now_us,c->direction_max_age_us))continue;
  if(turn&&(turn_mask&(1u<<side))&&d->clearance_m<c->side_turn_m){if(!s->side_brake[side])s->side_brake_since[side]=i->now_us;s->side_brake[side]=1;}
  if(r4_new_sample(d,&s->side_last_source[side],&s->side_seq[side])){
   if((!turn||forward_arc)&&d->clearance_m<c->side_straight_m){if(s->side_hits[side]&&d->source_us>s->side_first_us[side]&&d->source_us-s->side_first_us[side]<=100000)s->side_hits[side]++;else{s->side_hits[side]=1;s->side_first_us[side]=d->source_us;}if(s->side_hits[side]>=2){if(!s->side_brake[side])s->side_brake_since[side]=i->now_us;s->side_brake[side]=1;}}
   else if(d->clearance_m>=c->side_release_m){s->side_hits[side]=0;s->side_brake[side]=0;}
   else if(!turn&&d->clearance_m>=.020){s->side_hits[side]=0;s->side_brake[side]=0;}
   else if(d->clearance_m>=c->side_straight_m)s->side_hits[side]=0;
  }
 }
 if(s->front_brake||s->side_brake[0]||s->side_brake[1]){o->owner=R4_OWNER_BRAKE;o->owns_motion=1;o->v=o->w=0;o->emergency=1;o->planner_allowed=0;o->reason=s->front_brake?"FRONT_20MM_BRAKE":s->side_brake[0]?"LEFT_EMERGENCY":"RIGHT_EMERGENCY";}
}
static int r4_away_from_latched_sides(const r4_scene_state*s,double v,double w){if(!r4_same_forward_arc(v,w))return 0;for(unsigned side=0;side<2;side++)if(s->side_brake[side]&&!r4_away_from_side(v,w,side))return 0;return 1;}
static int r4_try_side_away_escape(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,r4_scene_output*o,uint64_t brake_since){
 if(s->front_brake||(!s->side_brake[0]&&!s->side_brake[1])||!r4_task_active(i)||!i->pose_valid||i->now_us<=brake_since){s->side_escape_active=0;return 0;}
 if(!r4_direction_usable(&i->direction[0],i->now_us,c->direction_max_age_us))return 0;
 if(s->side_escape_active&&s->side_escape_brake_us!=brake_since)s->side_escape_active=0;
 if(!r4_near_stopped(i)&&(!s->side_escape_active||i->measured_v<0||(fabs(i->measured_w)>c->w_straight&&!r4_away_from_latched_sides(s,i->measured_v,i->measured_w))))return 0;
 o->speed_cap=r4_current_front_cap(s,c,i);double sign=s->side_brake[0]?-1:1;const double v[3]={fmax(0,i->desired_v),.12,.12},w[3]={i->desired_w,sign*.1,sign*.3};
 for(unsigned k=0;k<3;k++){if(r4_away_from_latched_sides(s,v[k],w[k])&&r4_checked(o,i,v[k],w[k],"SIDE_AWAY_CHECKED_ESCAPE")){if(!s->side_escape_active)r4_stage_enter(s,i,R4_STAGE_IDLE);s->side_escape_active=1;s->side_escape_brake_us=brake_since;s->explore_started=0;o->stage=s->stage;o->emergency=1;return 1;}if(r4_result(i)==R6_CHECK_BUDGET)return 0;}s->side_escape_active=0;return 0;
}
static int r4_route_candidate(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,r4_scene_output*o){
 double v=i->desired_v,w=i->desired_w;if(v<0)v=0;if(v<=0&&fabs(w)<1e-6){if(fabs(i->goal_bearing_rad)>.1){v=0;w=copysign(c->pivot_w,i->goal_bearing_rad);}else v=c->explore_fast_m_s;}
 if(v>0){double cap=r4_action_speed_cap(s,i,o->speed_cap,v,w);if(cap<=0)return r4_record_result(i,R6_CHECK_WIRE_REJECT);if(v>cap){double k=cap/v;v*=k;w*=k;}}return r4_check(i,v,w,.8);
}
static int r4_recovered_route(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,r4_scene_output*o){
 const r4_direction*f=&i->direction[0];if(!r4_direction_usable(f,i->now_us,c->direction_max_age_us)||f->clearance_m<R6_FRONT_RELEASE_MM/1000.||s->front_brake){s->hold_clears=0;return 0;}
 /* 恢复可行性不沿用旧阻挡的零速度带，仅作候选，需两笔新样本及实测静止后才执行。
 */
 r4_scene_output probe=*o;probe.speed_cap=r4_front_band(f->clearance_m);
 if(i->have_speed_limit)probe.speed_cap=r4_min(probe.speed_cap,isfinite(i->speed_limit_m_s)?fmax(0,i->speed_limit_m_s):0);
 int result=r4_route_candidate(s,c,i,&probe);if(result==R6_CHECK_BUDGET){r4_wait(o,r4_check_reason(result));return 1;}if(result!=R6_CHECK_CLEAR){s->hold_clears=0;return 0;}
 if(r4_new_sample(f,&s->hold_source,&s->hold_seq)&&s->hold_clears<R6_FRONT_RELEASE_SAMPLES)s->hold_clears++;
 if(s->hold_clears<R6_FRONT_RELEASE_SAMPLES){if(s->stage==R4_REVERSE)return 0;r4_wait(o,"R6_ROUTE_CLEAR_WAIT_TWO_NEW");return 1;}
 if(!r4_near_stopped(i)){s->stage=R4_EXIT_TURN;o->stage=s->stage;r4_owned(o,0,0,"R6_ROUTE_CLEAR_BRAKE_TO_REJOIN");o->transition=1;return 1;}
 r4_stage_enter(s,i,R4_STAGE_IDLE);s->explore_started=0;s->post_reverse=0;s->turn_seen=0;s->hold_clears=0;
 s->speed_cap=probe.speed_cap;s->initialized=1;s->band_target=probe.speed_cap;s->band_since=0;s->band_last_sample=f->source_us;o->speed_cap=probe.speed_cap;
 o->stage=s->stage;o->owner=R4_OWNER_ROUTE;o->owns_motion=0;o->planner_allowed=1;o->reason="R6_ROUTE_RECOVERED_NO_FIXED_TURN";return 1;
}
static double r4_fit_retreat_target(const r4_scene_config*c,const r4_scene_input*i,double desired,double minimum){
 const r4_direction*b=&i->direction[3];if(!r4_direction_usable(b,i->now_us,c->direction_max_age_us))return 0;
 double speed=fmax(c->reverse_speed_m_s,fabs(i->measured_v)),reserve=speed*speed/(2*R6_BRAKE_M_S2)+speed*.05+R6_BODY_BUFFER_MM/1000.;double distance=fmin(desired,b->clearance_m-reserve);if(distance+1e-9<minimum)return 0;
 for(unsigned k=0;k<16;k++){int result=r4_check(i,-c->reverse_speed_m_s,0,distance/c->reverse_speed_m_s+.25);if(result==R6_CHECK_CLEAR)return distance;if(result<0)return 0;if(distance<=minimum+1e-9)break;distance=fmax(minimum,distance-.025);}return 0;
}
static int r4_retreat(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,r4_scene_output*o){
 if(s->stage!=R4_CHECK_REAR&&s->stage!=R4_REVERSE)return 0;
 if(s->stage==R4_CHECK_REAR&&s->phase_since<i->now_us&&r4_recovered_route(s,c,i,o))return 1;
 if(s->stage==R4_REVERSE){
  double along=-(i->x_m-s->phase_x)*cos(s->phase_yaw)-(i->y_m-s->phase_y)*sin(s->phase_yaw);if(i->execution_ready&&isfinite(along)&&along>s->reverse_progress)s->reverse_progress=along;if(i->execution_ready&&!s->execution_since)s->execution_since=i->now_us;
  double speed=fmax(.02,fabs(i->measured_v)),stop=speed*speed/(2*R6_BRAKE_M_S2)+speed*.05+R6_BODY_BUFFER_MM/1000.;
  if(s->reverse_progress>=c->reverse_hard_m||s->reverse_progress+stop>=s->fitted_target_m||(s->execution_since&&i->now_us>=s->execution_since&&i->now_us-s->execution_since>=c->reverse_max_us)){int moved=s->reverse_progress>=.003;r4_stage_enter(s,i,R4_EXIT_TURN);s->post_reverse=1;if(moved)s->side_angle[0]=s->side_angle[1]=0;o->stage=s->stage;r4_owned(o,0,0,"R6_RETREAT_SEGMENT_DONE_REASSESS_ROUTE");return 1;}
  if(r4_recovered_route(s,c,i,o))return 1;
 }
 if(s->stage==R4_CHECK_REAR){
  if(!r4_near_stopped(i)){r4_owned(o,0,0,"R6_BRAKE_BEFORE_SHORT_RETREAT");o->transition=1;return 1;}
  double distance=r4_fit_retreat_target(c,i,c->reverse_target_m,R6_REVERSE_MIN_FIT_M);if(distance<=0){if(r4_result(i)==R6_CHECK_BUDGET){r4_wait(o,r4_check_reason(r4_result(i)));return 1;}r4_observed_hold(s,i,o,r4_result(i)<0?r4_check_reason(r4_result(i)):"R6_REAR_PATH_NOT_AVAILABLE");return 1;}
  r4_stage_enter(s,i,R4_REVERSE);s->fitted_target_m=distance;s->extended=0;s->hold_clears=0;s->hold_source=0;s->hold_seq=0;o->stage=s->stage;
 }
 int result=r4_check(i,-c->reverse_speed_m_s,0,.8);if(result!=R6_CHECK_CLEAR){if(result==R6_CHECK_BUDGET){r4_wait(o,r4_check_reason(result));return 1;}r4_stage_enter(s,i,R4_EXIT_TURN);s->post_reverse=1;o->stage=s->stage;r4_wait(o,r4_check_reason(result));return 1;}
 if(i->execution_ready&&!s->reverse_counted&&s->reverse_progress>=.003&&i->measured_v<-.005){s->reverse_counted=1;if(s->reverse_attempts<UINT32_MAX)s->reverse_attempts++;}o->scene=R4_RETREAT;r4_owned(o,-c->reverse_speed_m_s,0,"R6_SHORT_RETREAT_012M");return 1;
}
static void r4_recovery(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,r4_scene_output*o){
 if(r4_recovered_route(s,c,i,o))return;if(r4_result(i)==R6_CHECK_BUDGET){r4_wait(o,r4_check_reason(r4_result(i)));return;}
 if(s->stage==R4_PROBE_TURN&&(s->turn_progress+1e-9>=c->explore_step_rad||s->side_angle[s->turn_side]+1e-9>=c->explore_side_budget_rad)){s->stage=R4_TURN_HOLD;o->stage=s->stage;r4_owned(o,0,0,"R6_TURN_TIME_REASSESS_WAIT_NEW");return;}
 if(s->stage==R4_TURN_HOLD){if(!r4_near_stopped(i)){r4_owned(o,0,0,"R6_TURN_TIME_REASSESS_WAIT_NEW");return;}r4_stage_enter(s,i,R4_GAP_MEASURE);}
 if(s->stage==R4_PROBE_TURN&&i->now_us>=s->phase_since&&i->now_us-s->phase_since>=c->explore_window_us){s->stage=R4_TURN_HOLD;o->stage=s->stage;r4_owned(o,0,0,"R6_TURN_TIME_REASSESS_WAIT_NEW");return;}
 int side=r4_side_select(i,c,s,c->side_turn_m);if(side>=0){r4_prepare_turn(s,c,i,(unsigned)side,o);return;}if(r4_result(i)==R6_CHECK_BUDGET){r4_wait(o,r4_check_reason(r4_result(i)));return;}
 if(r4_direction_usable(&i->direction[3],i->now_us,c->direction_max_age_us)){r4_prepare_retreat(s,i,o,"R6_FORWARD_TURNS_UNAVAILABLE_CHECK_SHORT_REAR");r4_retreat(s,c,i,o);return;}r4_observed_hold(s,i,o,r4_result(i)<0?r4_check_reason(r4_result(i)):"R6_NO_VERIFIED_ACTION_TARGET_PRESERVED");
}
static void r4_scene_step_impl(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*i,r4_scene_output*o){
 memset(o,0,sizeof(*o));o->scene=s->scene;o->stage=s->stage;o->owner=R4_OWNER_ROUTE;o->speed_cap=R6_MAX_LINEAR_M_S;o->reason="SCENE_ROUTE_ALLOWED";
 if(i->reset_estop)s->estop_latched=0;if(i->estop)s->estop_latched=1;if(s->estop_latched){o->owner=R4_OWNER_ESTOP;o->owns_motion=1;o->reason="EXPLICIT_STOP_OR_FAULT";return;}
 if(i->manual){s->last_step_us=i->now_us;s->last_motion_source=0;o->reason="MANUAL_UNCHANGED_BYPASS";return;}
 if(!i->now_us||!isfinite(i->measured_v)||!isfinite(i->measured_w)||!isfinite(i->desired_v)||!isfinite(i->desired_w)){r4_wait(o,"SCENE_INPUT_INVALID");return;}if(s->last_step_us&&i->now_us<s->last_step_us){r4_wait(o,"SCENE_TIME_REVERSED");return;}s->last_step_us=i->now_us;
 if(i->mission_id!=s->mission_id){s->mission_id=i->mission_id;s->stage=R4_STAGE_IDLE;s->explore_started=0;s->hold_clears=0;s->hold_source=0;s->hold_seq=0;s->turn_seen=0;s->last_motion_source=0;s->side_angle[0]=s->side_angle[1]=0;}
 r4_observe_executed_turn(s,i);
 r4_brakes(s,c,i,o);if(!i->pose_valid){if(!o->emergency)r4_wait(o,"SCENE_POSE_INVALID_WAIT_NEW_DATA");return;}
 uint32_t proposed=r4_scene_classify(i,c,s);r4_scene_confirm(s,c,i,proposed);o->scene=s->scene;o->corner_observed=s->corner_observed;o->speed_cap=r4_current_front_cap(s,c,i);
 if(o->emergency){
  uint64_t since=s->front_brake?s->front_brake_since:0;for(unsigned j=0;j<2;j++)if(s->side_brake[j]&&s->side_brake_since[j]>since)since=s->side_brake_since[j];if(i->now_us<=since||!r4_task_active(i))return;
  if(r4_try_side_away_escape(s,c,i,o,since))return;if(r4_result(i)==R6_CHECK_BUDGET)return;if(s->stage==R4_HOLD&&!r4_hold_has_new_data(s,c,i))return;
  if(s->stage!=R4_CHECK_REAR&&s->stage!=R4_REVERSE)r4_stage_enter(s,i,R4_CHECK_REAR);r4_retreat(s,c,i,o);o->emergency=1;return;
 }
 s->side_escape_active=0;if(!r4_task_active(i)){if(s->stage!=R4_STAGE_IDLE)r4_stage_enter(s,i,R4_STAGE_IDLE);o->reason="R6_NO_ACTIVE_TASK_NO_RECOVERY";return;}
 if(r4_retreat(s,c,i,o))return;if(s->stage==R4_HOLD){if(!r4_hold_has_new_data(s,c,i)){r4_wait(o,"PARTIAL_DIRECTION_WAIT_NEW_OBSERVATION");return;}if(s->side_angle[0]+1e-9>=c->explore_side_budget_rad&&s->side_angle[1]+1e-9>=c->explore_side_budget_rad&&r4_near_stopped(i))s->side_angle[0]=s->side_angle[1]=0;r4_stage_enter(s,i,R4_GAP_MEASURE);}
 if(s->stage!=R4_STAGE_IDLE||s->explore_started){r4_recovery(s,c,i,o);return;}
 const r4_direction*f=&i->direction[0];if(!r4_direction_usable(f,i->now_us,c->direction_max_age_us)||f->clearance_m<R6_FRONT_FORWARD_STOP_MM/1000.){r4_explore_start(s,i);r4_recovery(s,c,i,o);return;}
 /* 30..99mm仍先执行路线方向的可制动候选，观察/近障标签不是强制探转。 */
 o->planner_allowed=1;o->straight_model=0;if(proposed!=s->scene)o->reason="SCENE_CONFIRMING_UNIFIED_RULES";else o->reason=proposed==R4_FRONT_NEAR?"R6_NEAR_ROUTE_FIRST":f->clearance_m<R6_FRONT_OBSERVE_MM/1000.?"R6_OBSERVE_ROUTE_FIRST":"GENERAL_OBSERVED_ENVIRONMENT";
}
static void r4_budget_restore(r4_scene_state*s,const r4_scene_state*before){
 r4_scene_state observed=*s;*s=*before;s->last_step_us=observed.last_step_us;s->estop_latched=observed.estop_latched;s->front_brake=observed.front_brake;s->front_brake_since=observed.front_brake_since;s->front_clears=observed.front_clears;s->front_source=observed.front_source;s->front_seq=observed.front_seq;
 s->speed_cap=observed.speed_cap;s->initialized=observed.initialized;s->band_since=observed.band_since;s->band_target=observed.band_target;s->band_last_sample=observed.band_last_sample;
 s->last_motion_source=observed.last_motion_source;s->last_yaw=observed.last_yaw;
 for(unsigned j=0;j<2;j++){double delta=fmax(0,observed.executed_turn_angle[j]-before->executed_turn_angle[j]);s->executed_turn_angle[j]=observed.executed_turn_angle[j];s->side_angle[j]=before->side_angle[j]+delta;if((before->stage==R4_PROBE_TURN||before->stage==R4_TURN_HOLD)&&before->turn_side==j)s->turn_progress=before->turn_progress+delta;}
 for(unsigned j=0;j<2;j++){s->side_brake[j]=observed.side_brake[j];s->side_brake_since[j]=observed.side_brake_since[j];s->side_hits[j]=observed.side_hits[j];s->side_first_us[j]=observed.side_first_us[j];s->side_last_source[j]=observed.side_last_source[j];s->side_seq[j]=observed.side_seq[j];}
}
static void r4_scene_step(r4_scene_state*s,const r4_scene_config*c,const r4_scene_input*original,r4_scene_output*o){
 int local_result=R6_CHECK_CLEAR;r4_scene_input input=*original;if(!input.check_result)input.check_result=&local_result;r4_scene_state before=*s;r4_scene_step_impl(s,c,&input,o);
 if(r4_result(&input)==R6_CHECK_BUDGET&&o->owner!=R4_OWNER_ESTOP&&!input.manual){r4_budget_restore(s,&before);r4_wait(o,"R6_COMPUTE_BUDGET_WAIT");o->emergency=s->front_brake||s->side_brake[0]||s->side_brake[1];}
 else if(s->explore_started&&!s->round_counted&&input.execution_ready&&o->owns_motion&&(fabs(o->v)>.001||fabs(o->w)>.001)&&(fabs(input.measured_v)>.005||fabs(input.measured_w)>.01)){s->round_counted=1;if(s->explore_round<UINT32_MAX)s->explore_round++;}
 o->stage=s->stage;o->explore_round=s->explore_round;o->reverse_attempts=s->reverse_attempts;o->reverse_extended_attempts=s->reverse_extended_attempts;
}
#endif
