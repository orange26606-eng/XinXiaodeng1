/* S6只接入原近场与超声波接收器，不授予运动许可；每个插值候选仍通过原S5动作评估和最终发布检查。

 */
#include "nav_costmap.inc"
#include "nav_local_planner.inc"
void nav_s6_set_enabled(nav_engine*n,uint32_t enabled){if(n){n->s6_disabled=!enabled;n->s6_have_profile=0;n->s6_active=0;}}
static void s6_fallback(nav_engine*n,uint64_t now,const char*why){
 n->s6_active=0;n->s6_have_profile=0;snprintf(n->s6_reason,sizeof(n->s6_reason),"%s",why);
 if(strcmp(why,n->s6_logged_reason)||!n->s6_log_time||now-n->s6_log_time>=1000000){
  fprintf(stderr,"PLANNER_RULE_HANDOFF policy_revision=9 now_us=%llu reason=%s ranked=%u vetoed=%u rule_output=%s\n",(unsigned long long)now,why,n->s6_plan?n->s6_plan->count:0,n->s6_vetoed,n->c.passage.r4_enabled?"R6_UNIFIED":"EXPLICIT_LEGACY_CONFIG");
  snprintf(n->s6_logged_reason,sizeof(n->s6_logged_reason),"%s",why);n->s6_log_time=now;
 }
}
static void s6_tick_begin(nav_engine*n,uint64_t now){
 n->s6_active=0;n->s6_v_error=n->cmd.linear_m_s-n->measured_v;n->s6_w_error=n->cmd.angular_rad_s-n->measured_w;
 if(n->s6_have_profile&&(fabs(n->s6_v_error)>.05||fabs(n->s6_w_error)>.30)){
  if(!n->s6_mismatch_since)n->s6_mismatch_since=now;
  if(now>=n->s6_mismatch_since&&now-n->s6_mismatch_since>=300000&&(!n->s6_mismatch_log||now-n->s6_mismatch_log>=1000000)){
   fprintf(stderr,"TRAJ_MISMATCH policy_revision=9 now_us=%llu v_err=%.6f w_err=%.6f ms=%llu action=DIAGNOSTIC_ONLY\n",(unsigned long long)now,n->s6_v_error,n->s6_w_error,(unsigned long long)((now-n->s6_mismatch_since)/1000));n->s6_mismatch_log=now;
  }
 }else n->s6_mismatch_since=0;
 const char*gate;
 if(n->c.passage.r4_enabled)gate=n->s6_disabled?"S6_DISABLED":n->r4_output.emergency?"R5_COLLISION_RECOVERY_PRIORITY":n->r4_output.owns_motion?"R5_RULE_ACTION_PRIORITY":!n->r4_output.planner_allowed?"R5_DIRECTIONAL_RULE_PRIORITY":NULL;
 else gate=n->s6_disabled?"S6_DISABLED":n->s5_emergency?"NF_EMERGENCY_PRIORITY":n->s5_brake_reselect?"BRAKE_RESELECT_PRIORITY":n->phase==NAV_PHASE_BACKUP?"BACKUP_PRIORITY":n->phase==NAV_PHASE_REJOIN?"REJOIN_PRIORITY":n->s5_turn_hold?"TURN_HOLD_PRIORITY":n->s3_action.active?"COMMITTED_ACTION_PRIORITY":n->s5_front_clearance>=0&&n->s5_front_clearance<=.150000001?"NEAR_AVOID_PRIORITY":NULL;
 if(gate)s6_fallback(n,now,gate);
}
static s6_pose s6_to_map(const lab_pose*raw_current,const lab_pose*map_current,double x,double y,double yaw){
 double bx,by;nf_body_point(raw_current,x,y,&bx,&by);double c=cos(map_current->yaw_rad),s=sin(map_current->yaw_rad);
 return(s6_pose){map_current->x_m+c*bx-s*by,map_current->y_m+s*bx+c*by,wrap(yaw-raw_current->yaw_rad+map_current->yaw_rad)};
}
static void s6_configure(nav_engine*n){
 s6_config_default(&n->s6_config);const passage_config*p=&n->c.passage;s6_config*c=&n->s6_config;
 c->r4_speed_bands=n->c.passage.r4_enabled!=0;c->max_rpm=120;c->scene_geometry_enabled=n->r4_active;c->scene_straight_mode=n->r4_active&&n->r4_output.straight_model;c->body_length_m=p->length_m;c->body_width_m=p->width_m;c->body_buffer_m=.01;c->front_lethal_from_edge_m=p->front_lethal_from_edge_m;
 c->cycle_s=p->planner_cycle_ms/1000.;c->horizon_s=p->planner_horizon_s;c->dt_s=p->planner_dt_ms/1000.;c->lethal_m=p->costmap_lethal_m;c->inflation_m=p->costmap_inflation_m;
 c->w_heading=p->planner_w_heading;c->w_path=p->planner_w_path;c->w_clearance=p->planner_w_clearance;c->w_smoothness=p->planner_w_smoothness;c->w_velocity=p->planner_w_velocity;
 c->a_lin=p->a_lin_m_s2;c->a_yaw=p->a_yaw_rad_s2;c->brake=fmin(n->c.brake_m_s2,c->a_lin);c->allow_window_boundary_samples=0;
}
static int s6_build_costmap(nav_engine*n,const lab_pose*p,uint64_t now){
 nf_state*nf=&n->nearfield;lab_pose current,at;
 if(!nf->have_scan||nf->scan_time>now||now-nf->scan_time>n->c.max_scan_age_us||!nf->raw_points||!nf_current(nf,now,&current)||!nf_pose_at(nf,nf->scan_time,&at))return 0;
 cm_state*cm=n->s6_cm;cm_begin(cm,(s6_pose){p->x_m,p->y_m,p->yaw_rad},nf->scan_time,n->s6_config.lethal_m,n->s6_config.inflation_m);
 /* 全局源只读，未知保持255；采样到评分视图不清除原地图，也不生成时间戳。
 */
 for(unsigned y=0;y<S6_CM_SIDE;y++)for(unsigned x=0;x<S6_CM_SIDE;x++){
  double wx=cm->origin_x+(x+.5)*.02,wy=cm->origin_y+(y+.5)*.02;int k=index_at(n,wx,wy);uint8_t state=k<0||n->cells[k]<0?255:n->cells[k]>=65?254:0;cm_write_static(cm,wx,wy,state,0);
 }
 double c=cos(at.yaw_rad),s=sin(at.yaw_rad),sx=at.x_m+c*nf->c.mount_x_m-s*nf->c.mount_y_m,sy=at.y_m+s*nf->c.mount_x_m+c*nf->c.mount_y_m;
 s6_pose origin=s6_to_map(&current,p,sx,sy,at.yaw_rad);double ranges[NF_RAYS]={0},angles[NF_RAYS]={0};
 for(unsigned i=0;i<nf->latest_count;i++){
  double dx=nf->latest[i].x-sx,dy=nf->latest[i].y-sy,d=hypot(dx,dy),a=atan2(dy,dx);unsigned bin=nf_bin(a-at.yaw_rad-nf->c.mount_yaw_rad);
  if(!ranges[bin]||d<ranges[bin]){ranges[bin]=d;angles[bin]=a;}
 }
 /* 仅有实际回波的角桶清除射线，原点和角度使用扫描历史位姿，当前位姿仅用于坐标系映射。
 */
 for(unsigned b=0;b<NF_RAYS;b++)if(ranges[b]>0){
  double a=wrap(angles[b]-current.yaw_rad+p->yaw_rad),d=fmin(ranges[b]-.025,5.7);unsigned steps=d>0?(unsigned)ceil(d/.01):0;
  for(unsigned i=0;i<steps;i++){double along=i*.01;cm_write_obstacle(cm,origin.x+along*cos(a),origin.y+along*sin(a),0,nf->scan_time);}
 }
 /* 先写完全部自由射线，再写端点和近期近场证据，避免相邻擦边射线擦除实际障碍。
 */
 for(unsigned i=0;i<NF_CELLS;i++){
  const nf_cell*q=&nf->cells[i];if(q->state!=NF_CELL_HIT||q->time>now||now-q->time>NF_HOLD_US)continue;
  s6_pose m=s6_to_map(&current,p,q->x,q->y,0);cm_write_obstacle(cm,m.x,m.y,254,q->time);
 }
 for(unsigned i=0;i<nf->latest_count;i++){s6_pose m=s6_to_map(&current,p,nf->latest[i].x,nf->latest[i].y,0);cm_write_obstacle(cm,m.x,m.y,254,nf->scan_time);}
 /* 仅用于评分的时刻估计为RK接收时刻减源年龄，标记未同步；不覆盖扫描source_us，也不作为运动授权。
 */
 for(unsigned j=0;j<3;j++)if((n->s5_raw_available&n->flow_valid&n->flow_fresh&(1u<<j))&&!(n->flow_pending&(1u<<j))&&!(n->s5_raw_zero_mask&(1u<<j))&&n->s5_raw_mm[j]>0&&n->s5_raw_mm[j]<=4500&&n->s5_raw_mm[j]==n->flow_sonar[j]&&n->s5_diag_time<=now&&now-n->s5_diag_time<=250000&&n->s5_raw_age_us[j]<=250000-(now-n->s5_diag_time)&&n->s5_raw_age_us[j]<=n->s5_diag_time){
  uint64_t estimate=n->s5_diag_time-n->s5_raw_age_us[j];lab_pose at_sonar;
  if(nf_pose_at(nf,estimate,&at_sonar)){s6_pose m=s6_to_map(&current,p,at_sonar.x_m,at_sonar.y_m,at_sonar.yaw_rad);cm_project_sonar(cm,m,j,n->s5_raw_mm[j]/1000.,j?n->c.passage.sonar_side_offset_m:n->c.passage.sonar_front_offset_m,n->c.passage.sonar_beam_margin_m,estimate);}
 }
 cm_finish(cm);n->s6_cm_source=nf->scan_time;return 1;
}
static nav_xy s6_target(nav_engine*n,const lab_pose*p,nav_xy fallback){
 if(n->s5_front_approach||n->flow_final_yaw||n->flow_cruise||n->path_n<2)return fallback;
 double left=.5;nav_xy from={p->x_m,p->y_m};
 for(unsigned i=n->segment+1;i<n->path_n;i++){nav_xy to=n->path[i];double d=hypot(to.x_m-from.x_m,to.y_m-from.y_m);if(d>=left&&d>0)return(nav_xy){from.x_m+(to.x_m-from.x_m)*left/d,from.y_m+(to.y_m-from.y_m)*left/d};left-=d;from=to;}
 return n->path[n->path_n-1];
}
static int s6_speed_gap(const nav_engine*n,uint64_t now,double desired_v,double desired_w){
 if(n->s5_front_approach||n->narrow||n->s5_front_clearance<=.6||desired_v<=0||fabs(desired_w)>.05||!n->flow_have_hint||!(n->flow_valid&n->flow_fresh&1)||(n->flow_pending&1)||(n->s5_raw_zero_mask&1)||!n->flow_sonar[0]||n->flow_hint_time>now||now-n->flow_hint_time>nav_feedback_age_limit(n))return 0;
 const double vs[]={.06,.12,.20,.25,.35,.40},ws[]={0,.3,-.3,1,-1,1.9,-1.9};double maximum=-1;
 for(unsigned i=0;i<6;i++)for(unsigned j=0;j<7;j++)if(fabs(vs[i]-n->measured_v)<=n->s6_config.a_lin*n->s6_config.cycle_s+1e-9&&fabs(ws[j]-n->measured_w)<=n->s6_config.a_yaw*n->s6_config.cycle_s+1e-9&&s6_rpm_ok(&n->s6_config,vs[i],ws[j]))maximum=fmax(maximum,vs[i]);
 return maximum>=0&&desired_v>maximum+1e-9&&fabs(desired_v-n->measured_v)>n->s6_config.a_lin*n->s6_config.cycle_s+1e-9;
}
static int s6_yaw_gap(const nav_engine*n,double desired_w){
 if(fabs(desired_w-n->measured_w)<=.05)return 0;
 const double vs[]={.06,.12,.20,.25,.35,.40},ws[]={0,.3,-.3,1,-1,1.9,-1.9};double best=DBL_MAX;
 for(unsigned i=0;i<6;i++)for(unsigned j=0;j<7;j++)if(fabs(vs[i]-n->measured_v)<=n->s6_config.a_lin*n->s6_config.cycle_s+1e-9&&fabs(ws[j]-n->measured_w)<=n->s6_config.a_yaw*n->s6_config.cycle_s+1e-9&&s6_rpm_ok(&n->s6_config,vs[i],ws[j]))best=fmin(best,fabs(desired_w-ws[j]));
 return best<DBL_MAX&&best>=fabs(desired_w-n->measured_w)-1e-9;
}
static int s6_try_command_impl(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,double desired_v,double desired_w,nav_command*out){
 if((n->r4_active&&!n->r4_output.planner_allowed)||n->s6_disabled||!n->s6_cm||!n->s6_plan){s6_fallback(n,now,"S6_DISABLED_OR_STORAGE");return 0;}
 if(n->s5_front_clearance<0){s6_fallback(n,now,"FRONT_UNKNOWN_S5_FALLBACK");return 0;}
 if((!n->r4_active&&(n->s5_emergency||n->s5_brake_reselect||n->s5_turn_hold||n->s3_action.active||n->phase==NAV_PHASE_BACKUP||n->phase==NAV_PHASE_REJOIN))||(n->r4_active?n->s5_front_clearance<R6_FRONT_NEAR_MM/1000.:n->s5_front_clearance<=.150000001)){s6_fallback(n,now,n->r4_active?"R5_NEAR_ACTION_PRIORITY":"S5_PRIORITY_ACTIVE");return 0;}
 if(!nf_pose_valid(p)||!n->have_motion_pose||n->motion_time>now||now-n->motion_time>n->c.max_pose_age_us){s6_fallback(n,now,"POSE_INVALID_OR_STALE");return 0;}
 s6_configure(n);if(!s6_config_valid(&n->s6_config)){s6_fallback(n,now,"PLANNER_CONFIG_INVALID");return 0;}
 if(s6_yaw_gap(n,desired_w)){s6_fallback(n,now,"DYNAMIC_WINDOW_YAW_GAP");return 0;}
 if(s6_speed_gap(n,now,desired_v,desired_w)){s6_fallback(n,now,"DYNAMIC_WINDOW_SPEED_GAP");return 0;}
 int recompute=!n->s6_have_profile||now<n->s6_generated||now-n->s6_generated>=(uint64_t)llround(n->s6_config.cycle_s*1e6);
 double dt=n->cmd.time_us&&now>n->cmd.time_us?fmin(.25,(now-n->cmd.time_us)/1e6):.02;
 if(recompute){
  if(!s6_build_costmap(n,p,now)){s6_fallback(n,now,"COSTMAP_SOURCE_STALE_OR_POSE_UNALIGNED");return 0;}
  target=s6_target(n,p,target);s6_xy path[NAV_PATH_MAX];for(unsigned i=0;i<n->path_n;i++)path[i]=(s6_xy){n->path[i].x_m,n->path[i].y_m};
  s6_plan_input input={now,{p->x_m,p->y_m,p->yaw_rad},n->measured_v,n->measured_w,atan2(target.y_m-p->y_m,target.x_m-p->x_m),n->s5_front_clearance,path,n->path_n};
  s6_plan_compute(n->s6_cm,&n->s6_config,&input,n->s6_plan);n->s6_generated=now;n->s6_rank=0;n->s6_vetoed=0;
  if(!n->s6_plan->count){s6_fallback(n,now,n->s6_plan->reason);return 0;}
 }
 if(cm_age_ms(n->s6_cm,now)>n->c.max_scan_age_us/1000||now<n->s6_cm->source_us||now-n->s6_cm->source_us>n->c.max_scan_age_us){s6_fallback(n,now,"COSTMAP_STALE");return 0;}
 char why[64]="ALL_RANKED_S5_VETO";double v=0,w=0;
 for(unsigned rank=n->s6_rank;rank<n->s6_plan->count;rank++){
  s6_trajectory*tr=&n->s6_plan->ranked[rank];double elapsed=(now-n->s6_generated)/1e6+dt;
  if(!s6_profile_at(tr,elapsed,&v,&w))continue;
  /* 使用同一凸插值比例，保持轮速菱形域及两轴加速度边界，包括普通规划减速。
 */
  double dv=v-n->cmd.linear_m_s,dw=w-n->cmd.angular_rad_s,scale=1;
  if(fabs(dv)>n->s6_config.a_lin*dt)scale=fmin(scale,n->s6_config.a_lin*dt/fabs(dv));if(fabs(dw)>n->s6_config.a_yaw*dt)scale=fmin(scale,n->s6_config.a_yaw*dt/fabs(dw));
  v=n->cmd.linear_m_s+scale*dv;w=n->cmd.angular_rad_s+scale*dw;
  if(n->r4_active&&r6_route_opposite(n,w))continue;
  if(n->r4_active){r4_action_limit(n,&v,&w);r4_flow_limit(n,&v,&w);}
  double discarded_score=0;n->s5_rejected_reason[0]=0;n->s6_validating=1;int ok=flow_candidate(n,p,target,points,count,now,v,w,&discarded_score);n->s6_validating=0;
  if(!ok){if(n->r4_active&&n->r6_check_result==R6_CHECK_BUDGET){s6_fallback(n,now,"COMPUTE_BUDGET_RETRY_SAME_ROUTE");return 0;}n->s6_vetoed++;snprintf(why,sizeof(why),"%s",n->s5_rejected_reason[0]?n->s5_rejected_reason:n->flow_reject_sonar?"SONAR_FINAL_VETO":n->flow_reject_occupied?"S5_SWEEP_OCCUPIED":n->flow_reject_unknown?"S5_OBSERVATION_REQUIRED":"S5_FINAL_VETO");continue;}
  n->s6_rank=rank;n->s6_have_profile=n->s6_active=1;snprintf(n->s6_reason,sizeof(n->s6_reason),"%s",rank?"RANKED_RETRY_ACCEPTED":"PLANNER_ACCEPTED");
  n->flow_can_start=fabs(v)>.001||fabs(w)>.001;n->phase=fabs(w)<.1?NAV_PHASE_TRACK:w>0?NAV_PHASE_DETOUR_LEFT:NAV_PHASE_DETOUR_RIGHT;
  if(recompute)fprintf(stderr,"PLANNER_CHOICE policy_revision=9 now_us=%llu rank=%u ranked=%u vetoed=%u v=%.6f w=%.6f score=%.6f horizon=%.3f clear_min=%.6f costmap_age_ms=%llu source_scan_us=%llu sonar_time_mode=RX_MINUS_SOURCE_AGE_UNSYNCHRONIZED\n",(unsigned long long)now,rank,n->s6_plan->count,n->s6_vetoed,v,w,tr->score,n->s6_config.horizon_s,tr->clear_min,(unsigned long long)cm_age_ms(n->s6_cm,now),(unsigned long long)n->s6_cm->source_us);
  recovering_command(n,now,n->s5_front_approach?NAV_FRONT_APPROACH:NAV_REASON_NONE,v,w,out);return 1;
 }
 s6_fallback(n,now,why);return 0;
}
static int s6_try_command(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,double desired_v,double desired_w,nav_command*out){
 uint32_t sonar=n->flow_reject_sonar,occupied=n->flow_reject_occupied,unknown=n->flow_reject_unknown,budget=n->flow_reject_budget,accepted=n->flow_accept;nav_reason nf_reason=n->flow_nf_reason;
 int result=s6_try_command_impl(n,p,target,points,count,now,desired_v,desired_w,out);
 if(!result&&n->r6_check_result!=R6_CHECK_BUDGET){n->flow_reject_sonar=sonar;n->flow_reject_occupied=occupied;n->flow_reject_unknown=unknown;n->flow_reject_budget=budget;n->flow_accept=accepted;n->flow_nf_reason=nf_reason;}
 return result;
}
