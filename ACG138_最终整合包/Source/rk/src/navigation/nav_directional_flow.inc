/* 自适应实时控制，路线失败仍保留目标，实际近场检查继续拒绝不合适动作，不访问设备，也不新增控制持有者。
 */
static int flow_enabled(const nav_engine*n){return n->c.rectangular_footprint&&n->c.passage.continuous;}
void nav_set_chassis_hint_s1(nav_engine*n,uint64_t stamp,const uint16_t mm[3],uint8_t valid,uint8_t fresh,uint8_t near,uint8_t degraded,uint8_t stable,uint8_t pending){
 if(!n||!mm)return;n->flow_hint_time=stamp;memcpy(n->flow_sonar,mm,sizeof(n->flow_sonar));n->flow_valid=valid&7;n->flow_fresh=fresh&7;n->flow_near=near&7;n->flow_degraded=degraded!=0;n->flow_front_stable=stable!=0;n->flow_have_hint=1;n->flow_pending=pending&7;
}
void nav_set_chassis_hint(nav_engine*n,uint64_t stamp,const uint16_t mm[3],uint8_t valid,uint8_t fresh,uint8_t near,uint8_t degraded,uint8_t stable){nav_set_chassis_hint_s1(n,stamp,mm,valid,fresh,near,degraded,stable,0);}
/* 统一线上兼容分类器使用最终量化机械轮速。 */
static void flow_lr(double v,double w,int*l,int*r){double scale=6000/(PI*.0675);*l=(int)((v-w*.29*.5)*scale);*r=(int)((v+w*.29*.5)*scale);}
#define s5_speed_cap s5_speed_cap_legacy
#define s5_candidate_policy s5_candidate_policy_legacy
#define s5_sonar_geometry s5_sonar_geometry_legacy
#include "nav_s5_policy.inc"
#undef s5_speed_cap
#undef s5_candidate_policy
#undef s5_sonar_geometry
#include "nav_r4_policy.inc"
unsigned nav_action_intent(const nav_engine*n,double v,double w){int l,r;flow_lr(v,w,&l,&r);unsigned shape=s1_classify_lr_centi(l,r);if(shape==S1_FORWARD&&l!=r&&n&&n->flow_have_hint&&((l<0||r<0)||n->flow_sonar[0]<R6_FRONT_FORWARD_STOP_MM||(n->flow_near&1)))return S1_ESCAPE;return shape;}
static const char*flow_sonar_reject_reason(const nav_engine*n,uint64_t now,double v,double w){
 if(!n->flow_have_hint)return n->c.passage.r4_enabled?"FPGA_STATUS_MISSING":NULL;
 if(now<n->flow_hint_time||now-n->flow_hint_time>nav_feedback_age_limit(n))return "FPGA_STATUS_AGE_EXCEEDS_PROFILE";
 int l,r;flow_lr(v,w,&l,&r);unsigned intent=nav_action_intent(n,v,w);
 if(s1_direction_allows(intent,l,r,n->flow_sonar,n->flow_valid,n->flow_fresh,n->flow_near,n->flow_pending))return NULL;
 unsigned ok=n->flow_valid&n->flow_fresh&7u,blocked=n->flow_near&n->flow_pending&7u;
 if(!s1_intent_matches(intent,l,r))return "FPGA_INTENT_WHEEL_SHAPE_MISMATCH";
 if(intent==S1_REVERSE){if(!(ok&2))return "FPGA_REVERSE_LEFT_SONAR_UNAVAILABLE";if(!(ok&4))return "FPGA_REVERSE_RIGHT_SONAR_UNAVAILABLE";}
 if(intent==S1_PIVOT){
  if(!(ok&1))return "FPGA_PIVOT_FRONT_SONAR_UNAVAILABLE";
  if(!(ok&2))return "FPGA_PIVOT_LEFT_SONAR_UNAVAILABLE";
  if(!(ok&4))return "FPGA_PIVOT_RIGHT_SONAR_UNAVAILABLE";
  if(blocked)return "FPGA_PIVOT_NEAR_CONFIRMATION_PENDING";
  if(n->flow_sonar[0]<30)return "FPGA_PIVOT_FRONT_UNDER_30MM";
  if(n->flow_sonar[1]<30)return "FPGA_PIVOT_LEFT_UNDER_30MM";
  if(n->flow_sonar[2]<30)return "FPGA_PIVOT_RIGHT_UNDER_30MM";
 }
 if(!(ok&1))return "FPGA_FORWARD_FRONT_SONAR_UNAVAILABLE";
 if(blocked&1)return "FPGA_FRONT_NEAR_CONFIRMATION_PENDING";
 if(n->flow_sonar[0]<R6_FRONT_FORWARD_STOP_MM)return "FPGA_FRONT_UNDER_30MM";
 if(intent==S1_FORWARD&&(n->flow_near&1))return "FPGA_FRONT_NEAR_LATCH_RELEASE_40MM";
 if(l!=r){unsigned side=r>l?1u:2u,bit=1u<<side;
  if(!(ok&bit))return side==1?"FPGA_TURN_LEFT_SONAR_UNAVAILABLE":"FPGA_TURN_RIGHT_SONAR_UNAVAILABLE";
  if(blocked&bit)return side==1?"FPGA_TURN_LEFT_NEAR_PENDING":"FPGA_TURN_RIGHT_NEAR_PENDING";
  if(n->flow_sonar[side]<30)return side==1?"FPGA_TURN_LEFT_UNDER_30MM":"FPGA_TURN_RIGHT_UNDER_30MM";
  if(l<0||r<0)return "FPGA_MIXED_TURN_NEEDS_BOTH_SIDES_30MM";
 }
 return "FPGA_DIRECTION_GATE_REJECTED";
}
static int flow_sonar_allows(const nav_engine*n,uint64_t now,double v,double w){
 return flow_sonar_reject_reason(n,now,v,w)==NULL;
}
/* 超声波近距回波按车身边缘模型转换成障碍点，实际安装偏移需实测。该点模型不覆盖完整声束；雷达仍独立检查矩形车身扫掠，弧线经过障碍点时判为不可行。 */
static int flow_sonar_geometry(nav_engine*n,double v,double w,int escape){
 (void)escape;return s5_sonar_geometry(n,v,w,.8);
}
static double flow_sonar_cost(const nav_engine*n,double v,double w){
 if(!n->flow_have_hint)return 0;double left=(n->flow_valid&2)?clamp((80.-n->flow_sonar[1])/80.,0,1):0;
 double right=(n->flow_valid&4)?clamp((80.-n->flow_sonar[2])/80.,0,1):0;
 /* 侧向预警用于引导远离，不否决整车；侧距例如93mm时仍可允许平行运动或已核对后方的退行。
 */
 double cost=.50*(left*fmax(w,0)+right*fmax(-w,0))-.10*(left*fmax(-w,0)+right*fmax(w,0));
 if(v>0&&(n->flow_valid&1)&&n->flow_sonar[0]<150)cost+=.15*(150.-n->flow_sonar[0])/150.;return cost;
}
/* 窄道或恢复限速时保留命令曲率。 */
static void flow_linear_cap(double*v,double*w,double cap){if(*v>cap&&*v>0){double s=cap/ *v;*v=cap;*w*=s;}}
static void flow_limit(double*v,double*w){
 const double wheel=.0675*PI*80/60,track=.29;double peak=fmax(fabs(*v-*w*track*.5),fabs(*v+*w*track*.5));
 if(peak>wheel){double s=wheel/peak;*v*=s;*w*=s;}
}
static void r4_flow_limit(nav_engine*n,double*v,double*w){
 if(!r4_policy_active(n)){flow_limit(v,w);return;}
 double peak=fmax(fabs(*v-*w*.145),fabs(*v+*w*.145)),wheel=.0675*PI*120/60;
 if(peak>wheel){double scale=wheel/peak;*v*=scale;*w*=scale;}
 if(fabs(*v)<1e-9&&fabs(*w)>S2_PIVOT_YAW)*w=copysign(S2_PIVOT_YAW,*w);
}
static void r4_action_limit(nav_engine*n,double*v,double*w){
 if(r4_policy_active(n)&&*v>0){double cap=r4_action_speed_cap(&n->r4_state,&n->r4_input,n->r4_output.speed_cap,*v,*w);flow_linear_cap(v,w,cap);}
}
static void flow_episode(nav_engine*n,const lab_pose*p,uint64_t now,nav_reason why){
 if(!n->episode){n->episode=1;n->episode_started=now;n->episode_origin=(nav_xy){p->x_m,p->y_m};n->reverse_attempts=0;n->flow_extended_attempts=n->flow_backup_extended=0;n->backup_total=0;n->turn_total=0;n->flow_forward_progress=0;n->flow_forward_origin=n->episode_origin;}
 n->blocking_reason=why;
}
static passage_world flow_local_world(const nav_engine*n,passage_pose p){
 passage_world world=rectangle_world(n,p);world.confirmed_only=1;
 /* 越过旧全局占用进行探索时，需当前明确观测的自由几何；局部扫描缺失不能开放通行。
 */
 if(n->c.require_scan&&n->local&&n->local->initialized){world.local_only=1;world.observed_probe=1;world.probe_unknown_fraction=.20;world.probe_unknown_margin_m=.05;}
 return world;
}
#include "nav_r6_motion.inc"
static int flow_reverse_clear(nav_engine*n,const lab_pose*p,const nav_xy*points,unsigned count,uint64_t now,double distance){
 if(!n->c.require_scan||n->scan_time>now||(double)(now-n->scan_time)+n->time_uncertainty_s*1000000>n->c.max_scan_age_us){n->flow_rear_reason=NAV_REAR_SCAN_STALE;return 0;}
 if(!rear_observed(n,p,points,count)){n->flow_rear_reason=NAV_REAR_COVERAGE;return 0;}
 if(!flow_sonar_allows(n,now,-.2,0)||!flow_sonar_geometry(n,-.2,0,0)){n->flow_rear_reason=NAV_REAR_SONAR_STALE;return 0;}
 nf_check_result rear=nav_nearfield_check(n,now,-.2,0,nav_progressive_horizon(n,now,-.2,0));
 if(rear.code!=NF_CLEAR){n->flow_rear_reason=rear.code==NF_COLLISION?NAV_REAR_BODY_OCCUPIED:nav_nf_reason(rear.code);return 0;}
 double age=(now-n->scan_time)/1000000.0+n->time_uncertainty_s;
 double reserve=.2*.2/(2*n->c.brake_m_s2)+.2*(age+.05)+.01;
 if(n->rear_clearance<distance+reserve){n->flow_rear_reason=NAV_REAR_CLEARANCE;return 0;}
 passage_pose start={p->x_m,p->y_m,p->yaw_rad};passage_world world=flow_local_world(n,start);passage_result hit;
 passage_xy end={start.x-(distance+reserve)*cos(start.yaw),start.y-(distance+reserve)*sin(start.yaw)};
 int clear=passage_translate_clear(&world,start,end,world.config.width_m,&hit);
 n->flow_rear_reason=clear?NAV_REASON_NONE:hit.reason==PASSAGE_OCCUPIED?NAV_REAR_BODY_OCCUPIED:NAV_REAR_GEOMETRY_UNKNOWN;return clear;
}
static double flow_failure_cost(const nav_engine*n,const lab_pose*p,double yaw,uint64_t now){
 double cost=0;for(unsigned i=0;i<8;i++)if(n->flow_failed[i].expires>now&&hypot(p->x_m-n->flow_failed[i].p.x_m,p->y_m-n->flow_failed[i].p.y_m)<.5&&fabs(wrap(yaw-n->flow_failed[i].yaw))<.45)cost+=1.;return cost;
}
static void flow_remember(nav_engine*n,const lab_pose*p,uint64_t now){
 unsigned i=n->flow_failed_next++%8;n->flow_failed[i].p=(nav_xy){p->x_m,p->y_m};n->flow_failed[i].yaw=p->yaw_rad;n->flow_failed[i].expires=now+8000000;
}
/* 采用VFH式极坐标空隙偏好，不绕过占用地图。有效回波提供角域覆盖，缺失回波不形成空闲桶；各候选仍由完整矩形扫掠核对。

 */
static int flow_gap_target(nav_engine*n,const lab_pose*p,nav_xy wanted,const nav_xy*points,unsigned count,uint64_t now,nav_xy*out){
 double ranges[72]={0},limits[72];for(unsigned i=0;i<72;i++)limits[i]=30;
 for(unsigned j=0;j<count;j++){
  double dx=points[j].x_m-p->x_m,dy=points[j].y_m-p->y_m,d=hypot(dx,dy);if(d<.03)continue;
  double a=wrap(atan2(dy,dx)-p->yaw_rad);int b=(int)floor((a+PI)*72/(2*PI));if(b<0)b=0;if(b>71)b=71;
  if(!ranges[b]||d<ranges[b])ranges[b]=d;
  double spread=asin(fmin(1,(n->c.passage.width_m*.5+.025)/d));int pad=(int)ceil(spread*72/(2*PI));
  for(int k=-pad;k<=pad;k++){unsigned q=(unsigned)(b+k+144)%72;if(d<limits[q])limits[q]=d;}
 }
 double bearing=wrap(atan2(wanted.y_m-p->y_m,wanted.x_m-p->x_m)-p->yaw_rad),best=DBL_MAX;int found=0;double side_best[2]={DBL_MAX,DBL_MAX};nav_xy side_target[2]={{0},{0}};
 for(unsigned b=0;b<72;b++){
  double a=-PI+(b+.5)*2*PI/72,clear=fmin(ranges[b],limits[b]);if(fabs(wrap(a-bearing))<=PI/72)a=bearing;if(!ranges[b]||clear<.40||fabs(a)>150*PI/180)continue;
  unsigned wide=0;for(int k=-3;k<=3;k++){unsigned q=(unsigned)((int)b+k+144)%72;if(ranges[q]&&fmin(ranges[q],limits[q])>=.40)wide++;}
  unsigned near_width=0,far_width=0;for(int k=-4;k<=4;k++){unsigned q=(unsigned)((int)b+k+144)%72;if(ranges[q]&&fmin(ranges[q],limits[q])>=.6)near_width++;if(ranges[q]&&fmin(ranges[q],limits[q])>=1.2)far_width++;}
  double score=(n->flow_cruise?.65:1.4)*fabs(wrap(a-bearing))+.35*fabs(a)-.10*fmin(clear,3)-.035*wide-.012*near_width-.010*far_width+(n->path_n>=2?.08*path_distance(n,p->x_m+.6*cos(p->yaw_rad+a),p->y_m+.6*sin(p->yaw_rad+a)):0)+flow_failure_cost(n,p,p->yaw_rad+a,now);
  if(n->turn_dir&&now<n->flow_direction_until&&a*n->turn_dir<-.1)score+=.8;
  double length=fmin(.9,fmax(.25,clear-.25));nav_xy candidate={p->x_m+length*cos(p->yaw_rad+a),p->y_m+length*sin(p->yaw_rad+a)};unsigned side=a>=0?0:1;
  if(score<side_best[side]){side_best[side]=score;side_target[side]=candidate;}
  if(score<best){best=score;*out=candidate;found=1;}
 }
 if(found&&n->s5_front_approach){
  unsigned preferred=side_best[0]<=side_best[1]?0:1;
  if(!n->turn_dir)n->turn_dir=preferred? -1:1;
  unsigned held=n->turn_dir>0?0:1,other=1-held;
  if(side_best[held]<DBL_MAX){
   double held_quality=exp(-clamp(side_best[held],-8,8));double other_quality=side_best[other]<DBL_MAX?exp(-clamp(side_best[other],-8,8)):0;
   if(other_quality>held_quality*1.30){
    int direction=other?-1:1;
    if(n->s5_opposite_direction!=direction){n->s5_opposite_direction=direction;n->s5_opposite_since=now;}
    else if(now>=n->s5_opposite_since&&now-n->s5_opposite_since>=500000){n->turn_dir=direction;held=other;n->s5_opposite_since=0;}
   }else{n->s5_opposite_direction=0;n->s5_opposite_since=0;}
   *out=side_target[held];
  }
 }
 return found;
}
static int s3_full_target_clear(nav_engine*,const lab_pose*,uint64_t,double,double,double);
static int flow_candidate_eval(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,double v,double w,double*score){
 if(r4_policy_active(n)){
  if(n->r6_check_result==R6_CHECK_BUDGET)return 0;
  double lv=v,lw=w;r4_flow_limit(n,&lv,&lw);
  if(fabs(lv-v)>1e-7||fabs(lw-w)>1e-7){r6_record_check(n,R6_CHECK_WIRE_REJECT,"WHEEL_TARGET_LIMIT",v,w);++n->flow_reject_sonar;return 0;}
  if(!s5_candidate_policy(n,v,w)){r6_record_check(n,R6_CHECK_WIRE_REJECT,n->s5_rejected_reason,v,w);++n->flow_reject_sonar;return 0;}
  int result=r6_geometric_check(n,p,now,v,w,nav_progressive_horizon(n,now,v,w),1);
  if(result!=R6_CHECK_CLEAR){
   if(result==R6_CHECK_BUDGET)++n->flow_reject_budget;
   else if(result==R6_CHECK_OCCUPIED)++n->flow_reject_occupied;
   else if(result==R6_CHECK_WIRE_REJECT)++n->flow_reject_sonar;
   else{++n->flow_reject_unknown;n->flow_nf_reason=r6_wait_reason(n);}
   return 0;
  }
  ++n->flow_accept;*score=0;return 1;
 }
 /* R5方向稳定由新场景机管理。旧方向锁/自旋禁用不能抢先拒绝另一条安全动作。 */
 if(!r4_policy_active(n)&&n->s5_front_approach&&n->turn_dir&&w*n->turn_dir<-.05&&!n->s5_allow_hazard_switch){s5_reject(n,v,w,"FRONT_APPROACH_DIRECTION_HOLD");return 0;}
 if(!r4_policy_active(n)&&fabs(v)<.001&&fabs(w)>.001&&n->flow_spin_blocked[w>0?1:0]>now&&hypot(p->x_m-n->flow_spin_block_origin.x_m,p->y_m-n->flow_spin_block_origin.y_m)<.12)return 0;
 if(!r4_policy_active(n)&&v==0&&fabs(w)>.01&&n->s5_front_clearance>=.05&&n->s5_front_clearance<.10&&!s3_full_target_clear(n,p,now,v,w,copysign(70*PI/180,w))){s5_reject(n,v,w,"NEAR_PIVOT_FULL_SWEEP_REQUIRED");++n->flow_reject_sonar;return 0;}
 double limited_v=v,limited_w=w;r4_flow_limit(n,&limited_v,&limited_w);if(fabs(limited_v-v)>1e-7||fabs(limited_w-w)>1e-7){s5_reject(n,v,w,"WHEEL_TARGET_LIMIT");return 0;}if(!s5_candidate_policy(n,v,w)){++n->flow_reject_sonar;return 0;}if(!r4_policy_active(n)&&v>0&&n->s5_front_approach&&nav_nearfield_check(n,now,v,w,.8).code!=NF_CLEAR){++n->flow_reject_occupied;s5_reject(n,v,w,"FRONT_CORRIDOR_SWEEP");return 0;}if(!nav_nearfield_allows(n,now,v,w)){
  if(n->last_nf_check.code==NF_COLLISION)++n->flow_reject_occupied;else{++n->flow_reject_unknown;n->flow_nf_reason=nav_nf_reason(n->last_nf_check.code);}s5_reject(n,v,w,nav_reason_text(nav_nf_reason(n->last_nf_check.code)));return 0;}if(!flow_sonar_geometry(n,v,w,nav_action_intent(n,v,w)==S1_ESCAPE)){++n->flow_reject_sonar;return 0;}const char*wire_reason=flow_sonar_reject_reason(n,now,v,w);if(wire_reason){s5_reject(n,v,w,wire_reason);++n->flow_reject_sonar;return 0;}
 nav_reason unsafe=sweep_reason(n,p,v,w,points,count);if(unsafe!=NAV_REASON_NONE){if(unsafe==NAV_COMPUTE_WAIT)++n->flow_reject_budget;else if(unsafe==NAV_OBSTACLE)++n->flow_reject_occupied;else ++n->flow_reject_unknown;s5_reject(n,v,w,nav_reason_text(unsafe));return 0;}++n->flow_accept;
 if(n->s6_validating){*score=0;return 1;} /* 排序由规划器负责，不再增加第二套S5速度奖励。 */
 double x=p->x_m,y=p->y_m,a=p->yaw_rad;advance(&x,&y,&a,v,w,.8);
 double d=hypot(target.x_m-x,target.y_m-y),heading=fabs(wrap(atan2(target.y_m-y,target.x_m-x)-a));
 double goal_term=n->flow_cruise||n->path_n>=2?0:.5*hypot(n->goal.x_m-x,n->goal.y_m-y);
 *score=4*d+goal_term+(n->flow_local_probe?0:1.2*path_distance(n,x,y))+.30*heading+.06*fabs(w-n->cmd.angular_rad_s)-(n->s5_front_approach||(n->s5_front_clearance>=0&&n->s5_front_clearance<=.6)?.02:.35)*v+.35*n->flow_soft+.4*flow_failure_cost(n,p,a,now)+flow_sonar_cost(n,v,w);return 1;
}
static int flow_candidate(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,double v,double w,double*score){uint64_t started=nav_geometry_clock();++n->cost_candidate_count;int ok=flow_candidate_eval(n,p,target,points,count,now,v,w,score);n->cost_candidates_us+=nav_geometry_clock()-started;return ok;}
static lab_status flow_wait(nav_engine*n,uint64_t now,nav_reason why,nav_command*out){n->flow_can_start=0;n->state=NAV_WAITING;n->reason=why;n->blocking_reason=why;
 if(n->r4_active&&n->r6_check_result==R6_CHECK_BUDGET&&n->r6_saved_scene_time==now){
  r4_budget_restore(&n->r4_state,&n->r6_saved_scene);why=n->reason=n->blocking_reason=NAV_COMPUTE_WAIT;
  n->r4_output.stage=n->r4_state.stage;n->r4_output.owner=R4_OWNER_WAIT;n->r4_output.owns_motion=1;n->r4_output.planner_allowed=0;n->r4_output.v=n->r4_output.w=0;
  n->r4_output.reverse_attempts=n->r4_state.reverse_attempts;n->r4_output.reverse_extended_attempts=n->r4_state.reverse_extended_attempts;
  snprintf(n->r4_scene_reason,sizeof(n->r4_scene_reason),"R6_COMPUTE_BUDGET_WAIT");
 }
 if(!n->flow_candidate_diag_us||now-n->flow_candidate_diag_us>=1000000){n->flow_candidate_diag_us=now;fprintf(stderr,"DIRECTIONAL_WAIT now_us=%llu goal=%u controller=%s reason=%s scene_reason=%s rejected_reason=%s rejected_v=%.4f rejected_w=%.4f accepted=%u sonar_rejected=%u occupied_rejected=%u coverage_rejected=%u rear_reason=%s rear_clearance=%.3f attempts=%u requests=%u sonar_mm=%u,%u,%u valid=%u fresh=%u near=%u\n",(unsigned long long)now,n->goal.id,n->c.passage.r4_enabled?"R6_UNIFIED":"EXPLICIT_LEGACY_CONFIG",nav_reason_text(why),n->r4_scene_reason,n->s5_rejected_reason,n->s5_rejected_v,n->s5_rejected_w,n->flow_accept,n->flow_reject_sonar,n->flow_reject_occupied,n->flow_reject_unknown,nav_reason_text(n->flow_rear_reason),n->rear_clearance,n->reverse_attempts,n->flow_backup_requests,n->flow_sonar[0],n->flow_sonar[1],n->flow_sonar[2],n->flow_valid,n->flow_fresh,n->flow_near);}
 command(n,now,0,0,out);return LAB_OK;}
#include "nav_s3_avoidance.inc"
#include "nav_s6_integration.inc"
#include "nav_r4_adapter.inc"
#include "nav_r8_observation.inc"
static lab_status flow_step(nav_engine*n,uint64_t now,const lab_pose*p,nav_xy target,double preferred_v,double preferred_w,const nav_xy*points,unsigned count,int new_scan,nav_command*out){
 int s5_retreat_checked=0;const double pivot=S2_PIVOT_YAW;n->flow_can_start=0;n->flow_local_probe=n->path_n<2||n->flow_cruise||n->route_report.partial;if(n->path_n<2||n->route_report.partial)n->flow_provisional=1;n->flow_reject_sonar=n->flow_reject_occupied=n->flow_reject_unknown=n->flow_reject_budget=n->flow_accept=0;n->flow_nf_reason=NAV_REASON_NONE;
 s3_nearfield_view nf;nav_nearfield_view(n,now,&nf);
 if(!n->c.passage.r4_enabled){s3_progress(n,p,now);s5_update(n,p,points,count,now,new_scan,&nf);}
 if(n->c.passage.r4_enabled){
  /* 进入统一控制器时终止旧承诺，不让COMMITTED穿透到旧5+1预算和80RPM路径。
   * 原始样本setter与S1硬门独立保留，急刹与恢复由新场景机负责。 */
  if(n->s3_action.active)s3_action_end(n,0,"R5_UNIFIED_ACTION_RESELECT");
  n->s5_policy_now=now;n->s5_emergency=n->s5_emergency_confirmed=n->s5_brake_reselect=0;
  n->s5_turn_hold=n->s5_resume_limited=n->s5_force_corner=n->s5_after_backup=0;
  n->s5_raw_new_mask=n->s5_sample_new=0;
  double alpha=wrap(atan2(target.y_m-p->y_m,target.x_m-p->x_m)-p->yaw_rad);
  double route_v=preferred_v,route_w=preferred_w;
  if(fabs(alpha)>S2_ALIGN_ENTER_RAD){route_v=0;route_w=copysign(S2_PIVOT_YAW,alpha);}
  else if(n->flow_local_probe){route_v=n->c.max_speed_m_s;route_w=clamp(2*route_v*sin(alpha)/fmax(.35,hypot(target.x_m-p->x_m,target.y_m-p->y_m)),-2,2);}
  if(n->flow_final_yaw){route_v=0;route_w=clamp(2*alpha,-S2_PIVOT_YAW,S2_PIVOT_YAW);}
  n->r6_desired_v=route_v;n->r6_desired_w=route_w;n->r6_preferred_side=r6_side_of(route_w);
  n->r6_route_change_reason[0]=0;
  uint64_t geometry_start=nav_geometry_clock();
  n->query_deadline_us=geometry_start?geometry_start+R6_ACTION_GEOMETRY_BUDGET_US:0;
  if(n->r6_cycle_deadline&&n->query_deadline_us>n->r6_cycle_deadline)n->query_deadline_us=n->r6_cycle_deadline;
  r4_scene_state before_scene=n->r4_state;n->r6_saved_scene=before_scene;n->r6_saved_scene_time=now;
  r4_prepare(n,p,target,now,route_v,route_w);
  if(n->r4_active){
   if(n->r6_check_result==R6_CHECK_BUDGET){r4_budget_restore(&n->r4_state,&before_scene);return flow_wait(n,now,NAV_COMPUTE_WAIT,out);}
   /* 降低目标前检查实际惯性，不假设瞬间停车。 */
   int braking=r6_current_braking_check(n,p,now);
   if(braking!=R6_CHECK_CLEAR){
    snprintf(n->r6_route_change_reason,sizeof(n->r6_route_change_reason),"%s",braking==R6_CHECK_OCCUPIED?"MEASURED_MOTION_BRAKE_ROUTE_RETAINED":"CURRENT_BRAKING_CHECK_WAIT");
    return flow_wait(n,now,r6_wait_reason(n),out);
   }
   s6_tick_begin(n,now);
   if(n->r4_output.owns_motion){
    if(n->r6_check_result==R6_CHECK_UNOBSERVED&&!n->r4_output.emergency&&r8_observation_step(n,p,target,points,count,now,out))return LAB_OK;
    if(n->s3_action.active)s3_action_end(n,0,"SCENE_PREEMPT");
    double v=n->r4_output.v,w=n->r4_output.w;r4_action_limit(n,&v,&w);r4_flow_limit(n,&v,&w);
    if(fabs(v)<1e-9&&fabs(w)<1e-9)return flow_wait(n,now,n->r4_output.emergency?NAV_NF_EMERGENCY:n->r4_output.stage==R4_TURN_HOLD?NAV_TURN_HOLD:NAV_RECOVERY_WAIT,out);
    double score=0;if(!flow_candidate(n,p,target,points,count,now,v,w,&score))return flow_wait(n,now,r6_wait_reason(n),out);
    if(r6_route_opposite(n,w))snprintf(n->r6_route_change_reason,sizeof(n->r6_route_change_reason),"ROUTE_BLOCKED_SCENE_ESCAPE_CHECKED");
    n->flow_can_start=1;n->phase=v<0?NAV_PHASE_BACKUP:fabs(w)>.1?NAV_PHASE_ALIGN:NAV_PHASE_PROBE;
    return recovering_command(n,now,v<0?NAV_BACKING:NAV_LOCAL_AVOID,v,w,out);
   }
   double v=route_v,w=route_w;
   if(n->r4_output.straight_model&&fabs(w)<=.1){v=fmin(v,n->r4_output.speed_cap);w=n->r4_output.w;}
   else flow_linear_cap(&v,&w,n->r4_output.speed_cap);
   r4_action_limit(n,&v,&w);r4_flow_limit(n,&v,&w);
   double score=0;
   /* 路线候选按顺序尝试，先同曲率降速，再改变几何。 */
   const double factors[]={1,.75,.5,.25};
   int route_result=R6_CHECK_CLEAR;char route_failure[64]="";
   for(unsigned k=0;k<4;k++){
    double cv=v*factors[k],cw=w*factors[k];
    if(fabs(cv)<1e-5&&fabs(cw)<1e-5)continue;
    if(flow_candidate(n,p,target,points,count,now,cv,cw,&score)){
     n->r8_observe.active=0;n->r8_observe.cursor=0;n->flow_can_start=1;n->phase=fabs(cw)>.1?NAV_PHASE_ALIGN:NAV_PHASE_TRACK;
     snprintf(n->r6_route_change_reason,sizeof(n->r6_route_change_reason),"%s",k?"ROUTE_CURVATURE_PRESERVED_SLOWER":"ROUTE_PREFERRED_CLEAR");
     return recovering_command(n,now,NAV_REASON_NONE,cv,cw,out);
    }
    route_result=n->r6_check_result;snprintf(route_failure,sizeof(route_failure),"%s",n->s5_rejected_reason);
    if(route_result==R6_CHECK_BUDGET||route_result==R6_CHECK_STALE||route_result==R6_CHECK_POSE_INVALID||route_result==R6_CHECK_INVALID)return flow_wait(n,now,r6_wait_reason(n),out);
    if(k==0&&n->r8_observe.active&&route_result==R6_CHECK_UNOBSERVED&&r8_observation_step(n,p,target,points,count,now,out))return LAB_OK;
   }
   /* 原路线及降速版本都缺少覆盖时，大范围局部规划也无法补出观测；先为有界观察保留候选槽，再消费剩余计算预算。

 */
   if(route_result==R6_CHECK_UNOBSERVED&&r8_observation_step(n,p,target,points,count,now,out))return LAB_OK;
   if(n->r4_output.planner_allowed&&s6_try_command(n,p,target,points,count,now,v,w,out))return LAB_OK;
   if(n->r6_check_result==R6_CHECK_BUDGET)return flow_wait(n,now,NAV_COMPUTE_WAIT,out);
   /* 先试路线侧弧线或原地转，仅路线实际拒绝后再尝试另一侧。 */
   int sign=n->r6_preferred_side?n->r6_preferred_side:(alpha>=0?1:-1);
   for(unsigned side_pass=0;side_pass<2;side_pass++){
    int direction=side_pass?-sign:sign;
    if(side_pass&&route_result!=R6_CHECK_OCCUPIED&&route_result!=R6_CHECK_WIRE_REJECT)break;
    const double rates[]={.1,.3,1.0,S2_PIVOT_YAW};
    for(unsigned k=0;k<4;k++){
     double cv=k==3?0:fmin(n->r4_output.speed_cap,.20),cw=direction*rates[k];
     r4_action_limit(n,&cv,&cw);r4_flow_limit(n,&cv,&cw);
     if(flow_candidate(n,p,target,points,count,now,cv,cw,&score)){
      n->r8_observe.active=0;n->flow_can_start=1;n->phase=cw>0?NAV_PHASE_DETOUR_LEFT:NAV_PHASE_DETOUR_RIGHT;
      snprintf(n->r6_route_change_reason,sizeof(n->r6_route_change_reason),"%s",side_pass?"ROUTE_SIDE_REJECTED_OTHER_SIDE_CLEAR":"ROUTE_SIDE_ALTERNATIVE_CLEAR");
      fprintf(stderr,"ROUTE_RESELECT policy_revision=9 now_us=%llu expected_side=%d actual_side=%d route_check=%d route_reason=%s v=%.4f w=%.4f reason=%s\n",(unsigned long long)now,n->r6_preferred_side,r6_side_of(cw),route_result,route_failure,cv,cw,n->r6_route_change_reason);
      return recovering_command(n,now,NAV_LOCAL_AVOID,cv,cw,out);
     }
     if(n->r6_check_result==R6_CHECK_BUDGET||n->r6_check_result==R6_CHECK_STALE||n->r6_check_result==R6_CHECK_POSE_INVALID||n->r6_check_result==R6_CHECK_INVALID)return flow_wait(n,now,r6_wait_reason(n),out);
    }
   }
   /* 覆盖缺失属于观测不足，逐个核对小动作、另一侧或后方动作来主动获取新视角。
 */
   if((route_result==R6_CHECK_UNOBSERVED||n->r6_check_result==R6_CHECK_UNOBSERVED)&&
      r8_observation_step(n,p,target,points,count,now,out))return LAB_OK;
   if(n->have_goal&&(route_result==R6_CHECK_OCCUPIED||route_result==R6_CHECK_WIRE_REJECT))r4_explore_start(&n->r4_state,&n->r4_input);
   return flow_wait(n,now,r6_wait_reason(n),out);
  }
  /* 统一推进当前导航状态；等待期间接收新数据。 */
  return flow_wait(n,now,NAV_RECOVERY_WAIT,out);
 }else n->r4_active=0;
 s6_tick_begin(n,now);
 if(n->s5_brake_reselect){if(fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,NAV_RECOVERY_WAIT,out);n->s5_brake_reselect=0;n->s5_allow_hazard_switch=1;}
 if(n->s5_emergency&&!n->s5_emergency_confirmed)return flow_wait(n,now,NAV_NF_EMERGENCY,out);
 if(n->s5_emergency&&(n->measured_v>.025||fabs(n->measured_w)>.1))return flow_wait(n,now,NAV_NF_EMERGENCY,out);
 double current_goal=hypot(n->goal.x_m-p->x_m,n->goal.y_m-p->y_m);
 /* 有限实际转向可完成180度对准，持续轮空转不算进展；局部禁用重复旋转符号，改试反向或平移，再观察重试。预览不消费预算。

 */
 if(hypot(p->x_m-n->flow_spin_block_origin.x_m,p->y_m-n->flow_spin_block_origin.y_m)>=.12){memset(n->flow_spin_blocked,0,sizeof(n->flow_spin_blocked));memset(n->flow_spin_spent_us,0,sizeof(n->flow_spin_spent_us));n->flow_spin_block_origin=(nav_xy){p->x_m,p->y_m};}
 if((!n->flow_exec_known||n->flow_exec_ready)&&fabs(n->cmd.linear_m_s)<.001&&fabs(n->cmd.angular_rad_s)>.1){
  int sign=n->cmd.angular_rad_s>0?1:-1;
  uint64_t executed_until=now<n->cmd.expires_us?now:n->cmd.expires_us;
  if(n->flow_spin_last_us&&executed_until>=n->flow_spin_last_us&&now>=n->flow_spin_last_us&&now-n->flow_spin_last_us<=250000)n->flow_spin_spent_us[sign>0?1:0]+=executed_until-n->flow_spin_last_us;
  if(!n->flow_spin_started||sign!=n->flow_spin_sign){n->flow_spin_started=now;n->flow_spin_sign=sign;n->flow_spin_origin=(nav_xy){p->x_m,p->y_m};}
  if(n->flow_spin_spent_us[sign>0?1:0]>=4000000&&hypot(p->x_m-n->flow_spin_origin.x_m,p->y_m-n->flow_spin_origin.y_m)<.12){n->flow_spin_block_origin=(nav_xy){p->x_m,p->y_m};n->flow_spin_blocked[sign>0?1:0]=now+8000000;n->flow_spin_spent_us[sign>0?1:0]=0;n->flow_spin_started=0;flow_remember(n,p,now);}
 }else n->flow_spin_started=0;
 n->flow_spin_last_us=now;
 if(!n->flow_have_progress){n->flow_have_progress=1;n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_yaw=p->yaw_rad;n->flow_progress_goal=current_goal;}
 double progress=hypot(p->x_m-n->flow_progress_pose.x_m,p->y_m-n->flow_progress_pose.y_m);
 /* 仅旋转不算平移脱困，横向绕行需有明显位移，沿目标方向的小幅前进可更早计入进展。
 */
 if(progress>.12||(progress>.06&&n->flow_progress_goal-current_goal>.025)){n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_goal=current_goal;}
 int stalled=now>=n->flow_progress_time&&now-n->flow_progress_time>=3000000;
 if(stalled&&new_scan&&(!n->flow_stall_mark_us||now-n->flow_stall_mark_us>=2000000)){flow_remember(n,p,now);n->flow_stall_mark_us=now;flow_episode(n,p,now,NAV_LOCAL_AVOID);}
 int s5_preserve_checked_pivot=n->s3_action.active&&n->s3_action.stage==2&&!n->s5_emergency&&
  s3_full_target_clear(n,p,now,0,n->s3_action_w,n->s3_action.turn_target_rad-n->s3_action.turn_progress_rad);
 if(n->phase!=NAV_PHASE_BACKUP&&n->phase!=NAV_PHASE_REJOIN&&!s5_preserve_checked_pivot&&((n->s5_front_clearance>=0&&n->s5_front_clearance<=.150000001)||n->s5_emergency_confirmed)){
  if(n->s3_action.active)s3_action_end(n,0,"NEAR_AVOID_PREEMPT");
  flow_episode(n,p,now,n->s5_emergency?NAV_NF_EMERGENCY:NAV_NEAR_AVOID);
  if(!n->flow_blocked_since)n->flow_blocked_since=now;
  if(now-n->flow_blocked_since<50000||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,n->s5_emergency?NAV_NF_EMERGENCY:NAV_NEAR_AVOID,out);
  /* 后退预算耗尽仅移除倒退，不移除其他通过检查的原地转和弧线。 */
  if(n->reverse_attempts>=5&&n->flow_extended_attempts>=1)goto s5_select_actions;
  goto s5_retreat;
 }
 if(n->phase==NAV_PHASE_BACKUP){
  double moved=-(p->x_m-n->backup_start.x_m)*cos(n->flow_backup_yaw)-(p->y_m-n->backup_start.y_m)*sin(n->flow_backup_yaw);n->backup_progress=fmax(0,moved);
  n->s3_action.backup_progress_m=n->backup_progress;n->s3_action.backup_target_m=n->flow_backup_target;
  int executing=!n->flow_exec_known||n->flow_exec_ready;
  if(executing&&!n->flow_backup_execution_us)n->flow_backup_execution_us=now;
  if(!executing)n->flow_backup_execution_us=0; /* 预览不消费时间或尝试预算。 */
  if(executing&&!n->flow_backup_counted&&((moved>=.003&&n->measured_v<-.005)||(n->flow_wheel_reverse&&n->cmd.linear_m_s<-.001))){if(n->flow_backup_extended)++n->flow_extended_attempts;else ++n->reverse_attempts;n->flow_backup_counted=1;}
  double speed=fmax(.02,fabs(n->measured_v)),age=(now>=n->scan_time?(now-n->scan_time)/1000000.0:1)+n->time_uncertainty_s;
  double stopping=speed*speed/(2*n->c.brake_m_s2)+speed*(age+.05)+.01;
  if(moved>=(n->flow_backup_extended?.60:.40)||(n->flow_backup_execution_us&&now-n->flow_backup_execution_us>=(n->flow_backup_extended?4000000u:3000000u))||moved+stopping>=n->flow_backup_target){n->s3_action.stage=4;snprintf(n->s3_action.reason,sizeof(n->s3_action.reason),"BRAKE_TO_MEASURED_BACKUP_TARGET");phase_to(n,NAV_PHASE_REJOIN,now);return flow_wait(n,now,NAV_LOW_RECOVERY,out);}
  /* 旧的后退超过6cm且一帧前进可行就退出会提前终止后退，改为完成按净空确定的实际目标。
 */
  if(flow_reverse_clear(n,p,points,count,now,fmin(n->flow_backup_target-fmax(0,moved),.025))){n->flow_can_start=1;return recovering_command(n,now,NAV_BACKING,-.20,0,out);}
  /* 后方侵入只终止后退，本tick继续选前进、远离或转向；FPGA仍执行机械换向间隔。真正缺失源数据时按独立原因等待。

 */
  phase_to(n,NAV_PHASE_WAIT,now);
  if(n->s3_action.active)s3_action_end(n,0,n->flow_rear_reason==NAV_REAR_BODY_OCCUPIED?"REAR_HAZARD_RESELECT":"REAR_OBSERVATION_RESELECT");
  return flow_wait(n,now,n->flow_rear_reason==NAV_REASON_NONE?NAV_RECOVERY_WAIT:n->flow_rear_reason,out);
 }
 if(n->phase==NAV_PHASE_REJOIN){
  if(now-n->phase_started<50000||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,NAV_LOW_RECOVERY,out);
  if(n->s3_action.active&&n->s3_action.stage==4){n->s3_action.backup_progress_m=n->backup_progress;s3_action_end(n,1,"MEASURED_BACKUP_COMPLETED");}
  n->dirty=1;n->path_n=0;n->phase=NAV_PHASE_TRACK;n->flow_pivot=1;n->flow_progress_time=now;if(n->flow_backup_extended){flow_remember(n,p,now);n->turn_dir=n->turn_dir?-n->turn_dir:1;n->flow_direction_until=now+600000;}
  /* 后退用于腾出转向空间，不立即直行回到导致后退的障碍处。
 */
  if(n->s5_front_clearance>=0&&n->s5_front_clearance<.65){n->s5_force_corner=1;n->s5_after_backup=1;
   int direction=n->turn_dir?n->turn_dir:(nf.left_m>nf.right_m?1:-1),exit_selected=0;
   for(unsigned attempt=0;attempt<2;attempt++,direction=-direction)
    if(s3_action_begin(n,p,now,0,direction*pivot,&nf)){snprintf(n->s3_action.reason,sizeof(n->s3_action.reason),"BACKUP_THEN_MEASURED_EXIT_TURN");exit_selected=1;break;}
   if(!exit_selected){
    n->phase=NAV_PHASE_REJOIN;s5_reject(n,0,direction*pivot,"REJOIN_EXIT_SWEEP_UNAVAILABLE");
    if(now>=n->phase_started&&now-n->phase_started>=1000000&&(n->reverse_attempts<5||n->flow_extended_attempts<1)){n->flow_blocked_since=now-50000;goto s5_retreat;}
    return flow_wait(n,now,NAV_RECOVERY_WAIT,out);
   }
  }
 }
s5_select_actions:
 if(s5_turn_hold_wait(n,new_scan))return flow_wait(n,now,NAV_TURN_HOLD,out);
 if(s3_committed_step(n,p,target,points,count,now,new_scan,out))return LAB_OK;
 if(!n->flow_final_yaw&&(n->flow_cruise||n->path_n<2||n->s5_front_approach)){nav_xy gap;if(flow_gap_target(n,p,target,points,count,now,&gap))target=gap;}
 double alpha=wrap(atan2(target.y_m-p->y_m,target.x_m-p->x_m)-p->yaw_rad);
 if(n->flow_pivot&&fabs(alpha)<S2_ALIGN_EXIT_RAD){n->flow_pivot=0;
  /* 180度初始对准可能占用3秒进展窗口的大部分时间，每个显式目标只给一次切换宽限，不因持续旋转无限重置。

 */
  if(!n->flow_alignment_grace_used&&now>=n->goal_started&&now-n->goal_started<8000000){n->flow_alignment_grace_used=1;n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_goal=current_goal;}
 }if(fabs(alpha)>S2_ALIGN_ENTER_RAD)n->flow_pivot=1;
 if(n->flow_pivot){preferred_v=0;preferred_w=copysign(fmin(pivot,n->c.max_yaw_rate_rad_s),alpha);}
 else if(fabs(alpha)>15*PI/180){preferred_v=fmin(fabs(alpha)>25*PI/180?S2_TIGHT_LINEAR:S2_STRONG_LINEAR,n->c.max_speed_m_s);preferred_w=copysign(fmin(fabs(alpha)>25*PI/180?S2_TIGHT_YAW:S2_STRONG_YAW,n->c.max_yaw_rate_rad_s),alpha);}
 else if(n->flow_local_probe){preferred_v=n->c.max_speed_m_s;preferred_w=clamp(2*preferred_v*sin(alpha)/fmax(.35,hypot(target.x_m-p->x_m,target.y_m-p->y_m)),-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);}
 if(n->flow_final_yaw){preferred_v=0;preferred_w=clamp(2*alpha,-fmin(pivot,n->c.max_yaw_rate_rad_s),fmin(pivot,n->c.max_yaw_rate_rad_s));n->flow_pivot=1;}
 if(!n->flow_cruise&&n->path_n>=2&&isfinite(n->corner_distance)&&n->corner_distance<.25&&preferred_v>0)flow_linear_cap(&preferred_v,&preferred_w,fmax(.04,1.5*n->corner_distance));
 if(n->narrow)flow_linear_cap(&preferred_v,&preferred_w,n->c.passage.narrow_speed_m_s);
 if(n->flow_have_hint&&!n->flow_front_stable)flow_linear_cap(&preferred_v,&preferred_w,.12);flow_linear_cap(&preferred_v,&preferred_w,s5_speed_cap(n));flow_limit(&preferred_v,&preferred_w);
 if(s6_try_command(n,p,target,points,count,now,preferred_v,preferred_w,out))return LAB_OK;
 double best=DBL_MAX,bv=0,bw=0,score=0,best_observed=0;int selected=0,preferred=0,best_soft=0;passage_result best_hit={0};nf_check_result best_nf={0};
 if((fabs(preferred_v)>.001||fabs(preferred_w)>.001)&&flow_candidate(n,p,target,points,count,now,preferred_v,preferred_w,&score)){best=score-.05;bv=preferred_v;bw=preferred_w;selected=1;preferred=1;best_hit=n->passage_collision;best_observed=n->flow_observed_fraction;best_soft=n->flow_soft;best_nf=n->last_nf_check;}
 if(!selected&&(n->flow_reject_sonar||n->flow_reject_occupied)&&(fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)){
  n->s5_brake_reselect=1;return flow_wait(n,now,NAV_RECOVERY_WAIT,out);
 }
 if(!selected&&(n->flow_reject_sonar||n->flow_reject_occupied)&&fabs(n->measured_v)<=.025&&fabs(n->measured_w)<=.1)n->s5_allow_hazard_switch=1;
 if(!selected||((n->flow_soft||(n->episode&&stalled))&&(!n->flow_full_pool_us||now-n->flow_full_pool_us>=200000))){
  n->flow_full_pool_us=now;double evaluated[56][2];unsigned evaluated_n=0,tries=0,max_tries=n->flow_load_reduced?8:12;
  const double anchors[10][2]={{S2_ESCAPE_LINEAR,S2_ESCAPE_YAW},{S2_ESCAPE_LINEAR,-S2_ESCAPE_YAW},{.06,0},{.12,0},{0,S2_PIVOT_YAW},{0,-S2_PIVOT_YAW},{S2_TIGHT_LINEAR,S2_TIGHT_YAW},{S2_TIGHT_LINEAR,-S2_TIGHT_YAW},{.12,.35},{.12,-.35}};
  for(unsigned ai=0;ai<10;ai++){
   double av=anchors[ai][0],aw=anchors[ai][1];if(n->flow_final_yaw&&av>0)continue;
   av=fmin(av,n->narrow?n->c.passage.narrow_speed_m_s:n->c.max_speed_m_s);if(n->flow_have_hint&&!n->flow_front_stable&&av>.12)av=.12;aw=clamp(aw,-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);flow_limit(&av,&aw);
   evaluated[evaluated_n][0]=av;evaluated[evaluated_n++][1]=aw;++tries;
   if(flow_candidate(n,p,target,points,count,now,av,aw,&score)&&score<best){best=score;bv=av;bw=aw;selected=1;preferred=0;best_hit=n->passage_collision;best_observed=n->flow_observed_fraction;best_soft=n->flow_soft;best_nf=n->last_nf_check;}
   if(nav_geometry_cancel(n))break;
  }
  const double speeds[]={.25,S2_TIGHT_LINEAR,.12,.06,0,.04},rates[]={0,.35,-.35,S2_TIGHT_YAW,-S2_TIGHT_YAW,S2_PIVOT_YAW,-S2_PIVOT_YAW};
  for(unsigned pass=0;pass<2;pass++){
   for(unsigned i=0;i<6;i++)for(unsigned j=0;j<7;j++){
    unsigned candidate=(i*7+j+n->flow_candidate_cursor)%42,si=candidate/7,rj=candidate%7;double v=fmin(speeds[si],n->narrow?n->c.passage.narrow_speed_m_s:n->c.max_speed_m_s),w=clamp(rates[rj],-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);
    if(v==0&&w==0)continue;if(n->flow_pivot&&fabs(w)<.7)continue;if(n->flow_final_yaw&&v>0)continue;if(n->flow_pivot&&v>0&&pass==0&&!stalled&&preferred)continue;
    if(pass==0&&now<n->flow_direction_until&&n->turn_dir&&w*n->turn_dir<-.05)continue;
    if(n->flow_have_hint&&!n->flow_front_stable&&v>.12)v=.12;flow_limit(&v,&w);int duplicate=0;for(unsigned z=0;z<evaluated_n;z++)if(fabs(v-evaluated[z][0])<1e-8&&fabs(w-evaluated[z][1])<1e-8)duplicate=1;if(duplicate)continue;evaluated[evaluated_n][0]=v;evaluated[evaluated_n++][1]=w;if(tries++>=max_tries)break;if(flow_candidate(n,p,target,points,count,now,v,w,&score)&&score<best){best=score;bv=v;bw=w;selected=1;preferred=0;best_hit=n->passage_collision;best_observed=n->flow_observed_fraction;best_soft=n->flow_soft;best_nf=n->last_nf_check;}
   }
   if(selected)break;
  }
  n->flow_candidate_cursor=(n->flow_candidate_cursor+max_tries)%42;
 }
 /* 受阻只改变方向代价，不否决全部合法候选；后方净空或后退预算不覆盖有效前进、弧线和转向，独立旋转记录防止无限同向转动。

 */
 int try_retreat=selected&&!n->flow_final_yaw&&stalled&&fabs(bv)<.001&&(n->reverse_attempts<5||n->flow_extended_attempts<1)&&n->flow_spin_spent_us[0]+n->flow_spin_spent_us[1]>=2000000&&flow_reverse_clear(n,p,points,count,now,.10);
 if(selected&&!try_retreat){
  n->passage_collision=best_hit;n->flow_observed_fraction=best_observed;n->flow_soft=best_soft;n->last_nf_check=best_nf;n->last_nf_query_time=now;n->last_nf_query_v=bv;n->last_nf_query_w=bw;n->last_nf_query_horizon=nav_progressive_horizon(n,now,bv,bw);
  n->s5_allow_hazard_switch=0;n->flow_blocked_since=0;n->flow_can_start=fabs(bv)>.001||fabs(bw)>.001;
  if(!n->flow_final_yaw&&!n->s3_action.active&&fabs(bw)>.2&&nf.valid&&nf.front_m>=0&&nf.front_m<.45&&(!preferred||n->flow_local_probe))
   s3_action_begin(n,p,now,bv,bw,&nf);
  if(!preferred){flow_episode(n,p,now,NAV_OBSTACLE);if(fabs(bw)>.1){int sign=bw>0?1:-1;if(sign!=n->turn_dir){n->turn_dir=sign;n->flow_direction_until=now+600000;}}n->phase=bv==0?NAV_PHASE_ALIGN:bw>0?NAV_PHASE_DETOUR_LEFT:NAV_PHASE_DETOUR_RIGHT;}
  else if(n->flow_pivot)n->phase=NAV_PHASE_ALIGN;
  else {int clear=!n->flow_have_hint||(n->flow_front_stable&&(n->flow_fresh&1));if(n->flow_have_hint)n->clear_scans=clear?2:0;else if(new_scan&&n->clear_scans<2)++n->clear_scans;if(n->clear_scans>=2){n->phase=n->flow_local_probe?NAV_PHASE_PROBE:NAV_PHASE_TRACK;if(!n->s5_front_approach)n->turn_dir=0;n->blocking_reason=NAV_REASON_NONE;}}
  nav_reason why=n->s5_front_approach?NAV_FRONT_APPROACH:n->flow_cruise?NAV_CRUISE:n->flow_local_probe?NAV_LOCAL_PROBE:n->phase==NAV_PHASE_TRACK?NAV_REASON_NONE:NAV_LOCAL_AVOID;
  if(!n->flow_candidate_diag_us||now-n->flow_candidate_diag_us>=1000000){n->flow_candidate_diag_us=now;fprintf(stderr,"DIRECTIONAL_CANDIDATES now_us=%llu goal=%u selected_v=%.4f selected_w=%.4f accepted=%u sonar_rejected=%u occupied_rejected=%u coverage_rejected=%u observed_fraction=%.3f rear_reason=%s attempts=%u requests=%u\n",(unsigned long long)now,n->goal.id,bv,bw,n->flow_accept,n->flow_reject_sonar,n->flow_reject_occupied,n->flow_reject_unknown,n->flow_observed_fraction,nav_reason_text(n->flow_rear_reason),n->reverse_attempts,n->flow_backup_requests);}
  return recovering_command(n,now,why,bv,bw,out);
 }
 if(n->flow_reject_budget||nav_geometry_cancel(n))return flow_wait(n,now,NAV_COMPUTE_WAIT,out);
 if(n->flow_nf_reason==NAV_NEARFIELD_STALE||n->flow_nf_reason==NAV_NEARFIELD_POSE||n->flow_nf_reason==NAV_NEARFIELD_INVALID)return flow_wait(n,now,n->flow_nf_reason,out);
s5_retreat:
 if(s5_retreat_checked)return flow_wait(n,now,NAV_NO_EXECUTABLE_ACTION,out);s5_retreat_checked=1;
 flow_episode(n,p,now,n->s5_emergency?NAV_NF_EMERGENCY:n->s5_front_clearance>=0&&n->s5_front_clearance<.10?NAV_NEAR_AVOID:NAV_OBSTACLE);
 if(!n->flow_blocked_since){n->flow_blocked_since=now;flow_remember(n,p,now);}
 n->phase=NAV_PHASE_BACK_CHECK;
 if(now-n->flow_blocked_since<50000||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,NAV_BACK_CHECK,out);
 if(n->reverse_attempts>=5&&n->flow_extended_attempts>=1){n->phase=NAV_PHASE_WAIT;return flow_wait(n,now,NAV_RECOVERY_EXHAUSTED,out);}
 /* 按前方危险几何选择0.20至0.40m或0.40至0.60m目标，再缩减至已观测后方空间，不强制最小距离。
 */
 int extended=n->reverse_attempts>=5;double distance=s3_backup_distance(n,&nf,extended),minimum=extended?.40:.20;while(distance>=minimum&&!flow_reverse_clear(n,p,points,count,now,distance))distance-=.025;
 if(distance<minimum){
  n->phase=NAV_PHASE_WAIT;
  if(n->s5_front_clearance>=.05&&n->s5_front_clearance<=.150000001&&!n->s5_emergency){n->s5_allow_hazard_switch=1;goto s5_select_actions;}
  /* 只有前进、弧线、原地转及适配后退全部失败才形成综合结果；后方诊断只作子原因，不显示成后方占用禁止前进。

 */
  if(n->flow_rear_reason==NAV_NEARFIELD_STALE||n->flow_rear_reason==NAV_NEARFIELD_POSE||n->flow_rear_reason==NAV_NEARFIELD_INVALID)return flow_wait(n,now,n->flow_rear_reason,out);
  return flow_wait(n,now,NAV_NO_EXECUTABLE_ACTION,out);
 }
 ++n->flow_backup_requests;n->flow_backup_counted=0;n->flow_backup_execution_us=0;n->flow_backup_extended=extended;n->flow_backup_target=fmin(extended?.60:.40,distance);n->flow_backup_yaw=p->yaw_rad;
 n->backup_start=(nav_xy){p->x_m,p->y_m};n->backup_progress=0;n->flow_blocked_since=0;
 if(n->s3_action.active)s3_action_end(n,0,"RESELECT_BACKUP");
 n->s3_action.active=1;n->s3_action.stage=3;++n->s3_action.event_id;n->s3_action.direction=0;n->s3_action.backup_target_m=n->flow_backup_target;n->s3_action.backup_progress_m=0;
 snprintf(n->s3_action.reason,sizeof(n->s3_action.reason),"GEOMETRY_FITTED_BACKUP");
 phase_to(n,NAV_PHASE_BACKUP,now);return flow_wait(n,now,NAV_BACK_CHECK,out);
}
