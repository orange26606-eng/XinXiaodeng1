/* R4生产接线：原始观测不改时间，场景只有一个输出所有者。 */
typedef struct r4_nav_check_context {nav_engine*n;const lab_pose*p;uint64_t now;} r4_nav_check_context;
static double r4_sonar_clear(const nav_engine*n,unsigned side,uint64_t now){
 if(n->s5_diag_time<=now&&now-n->s5_diag_time<=nav_feedback_age_limit(n)&&(n->s5_raw_available&(1u<<side))&&n->s5_raw_age_us[side]<=nav_feedback_age_limit(n)-(now-n->s5_diag_time)&&n->s5_raw_mm[side]>0)
  return n->s5_raw_mm[side]/1000.+(side?n->c.passage.sonar_side_offset_m:n->c.passage.sonar_front_offset_m);
 /* 原始0不清空有效距离，也不刷新非零样本年龄。 */
 if(n->flow_have_hint&&n->flow_hint_time<=now&&now-n->flow_hint_time<=nav_feedback_age_limit(n)&&(n->flow_valid&n->flow_fresh&(1u<<side))&&n->flow_sonar[side]>0)
  return n->flow_sonar[side]/1000.+(side?n->c.passage.sonar_side_offset_m:n->c.passage.sonar_front_offset_m);
 return -1;
}
static void r4_fuse(nav_engine*n,const lab_pose*p,uint64_t now,r4_scene_input*in){
 for(unsigned j=0;j<4;j++)in->direction[j]=(r4_direction){.clearance_m=-1,.quality=R4_INVALID};
 nf_state*nf=&n->nearfield;lab_pose raw;
 if(nf->have_scan&&nf->scan_time<=now&&now-nf->scan_time<=n->c.max_scan_age_us&&nf_current(nf,now,&raw)){
  const double hx=n->c.passage.length_m*.5,hy=n->c.passage.width_m*.5;
  for(unsigned k=0;k<nf->latest_count;k++){
   double x,y;nf_body_point(&raw,nf->latest[k].x,nf->latest[k].y,&x,&y);
   /* 仅屏蔽自车外廓；该过滤不代表探头已经实车标定。 */
   if(fabs(x)<=hx+.01&&fabs(y)<=hy+.01)continue;
   double a=atan2(y,x),d[4]={-1,-1,-1,-1};
   /* 前方速度档只看实际前进走廊。斜侧点仍留在NF完整扫掠中，
    * 不能因为扇区重叠，把车身左外侧的墙投影成正前方近障。 */
   if(fabs(a)<=R4_PI/4&&fabs(y)<=hy+.01)d[0]=fmax(0,x-hx);
   if(a>=R4_PI/6&&a<=5*R4_PI/6&&fabs(x)<=hx+.01)d[1]=fmax(0,y-hy);
   if(a<=-R4_PI/6&&a>=-5*R4_PI/6&&fabs(x)<=hx+.01)d[2]=fmax(0,-y-hy);
   if(fabs(a)>=3*R4_PI/4&&fabs(y)<=hy+.01)d[3]=fmax(0,-x-hx);
   for(unsigned j=0;j<4;j++)if(d[j]>=0&&(in->direction[j].clearance_m<0||d[j]<in->direction[j].clearance_m))
    in->direction[j]=(r4_direction){d[j],nf->scan_time,(uint32_t)(nf->scan_time&0xffffffffu),R4_FRESH|R4_SINGLE_SOURCE,n->c.max_scan_age_us};
  }
 }
 double lidar_front=in->direction[0].clearance_m;
 for(unsigned j=0;j<3;j++){
  double d=r4_sonar_clear(n,j,now);if(d<=0)continue;
  int raw=n->s5_raw_mm[j]>0&&(n->s5_raw_available&(1u<<j))&&n->s5_raw_source_us[j];
  uint64_t age=raw?n->s5_raw_age_us[j]:0;
  uint64_t stamp=n->s5_diag_time>=age?n->s5_diag_time-age:0;
  if(!raw){stamp=n->flow_hint_time;age=n->s5_raw_age_us[j];if(age<=stamp)stamp-=age;}
  /* 超声波保持值保留原采集时间；当前雷达方向优先使用新观测。 */

  if(!stamp||stamp>now||now-stamp>nav_feedback_age_limit(n))continue;
  /* 收到 0 值后沿用最后非零距离及其样本身份。NCS 状态和 0 值帧不计入两笔解除或三笔清障确认；缺少样本身份时不确认。 */
  uint32_t seq=(n->s5_raw_available&(1u<<j))?n->s5_raw_seq[j]:j==0&&n->s5_sequence_valid?n->s5_front_sequence:0;
  /* 同一方向取近值，沿用选中来源的真实样本身份。不能给融合值盖本周期时间。 */
  r4_direction*o=&in->direction[j];
  if(o->clearance_m<0||d<o->clearance_m){uint32_t quality=R4_FRESH|(o->clearance_m<0?R4_SINGLE_SOURCE:0);*o=(r4_direction){d,stamp,seq,quality,nav_feedback_age_limit(n)};}
  else o->quality&=~R4_SINGLE_SOURCE;
 }
 /* 场景标签只表达传感器几何现象，不宣称识别出了物体材质。 */
 double sonar_front=r4_sonar_clear(n,0,now);
 in->low_obstacle=sonar_front>0&&sonar_front<.15&&lidar_front>.30;
 in->glass=sonar_front>0&&sonar_front<.15&&lidar_front<0&&nf->have_scan&&nf->raw_points>20&&nf->scan_time<=now&&now-nf->scan_time<=n->c.max_scan_age_us;
 /* 连续点簇中的窄前向结构：只能作为“桌腿/悬空候选”保守整片处理。 */
 if(nf->have_scan&&nf->scan_time<=now&&now-nf->scan_time<=n->c.max_scan_age_us&&nf_current(nf,now,&raw)){
  double min_y=1e6,max_y=-1e6;unsigned narrow=0,group=0;double gx0=0,gy0=0,lastx=0,lasty=0;
  for(unsigned k=0;k<=nf->latest_count;k++){
   double x=0,y=0;if(k<nf->latest_count)nf_body_point(&raw,nf->latest[k].x,nf->latest[k].y,&x,&y);
   int eligible=k<nf->latest_count&&x>n->c.passage.length_m*.5+.15&&x<1.5&&fabs(y)<.65;
   if(group&&(!eligible||hypot(x-lastx,y-lasty)>.10)){
    if(group>=2&&hypot(lastx-gx0,lasty-gy0)<.08){narrow++;min_y=fmin(min_y,gy0);max_y=fmax(max_y,gy0);}group=0;
   }
   if(eligible){if(!group){gx0=x;gy0=y;}group++;lastx=x;lasty=y;}
  }
  in->overhang=narrow>=2&&max_y-min_y>.20;
 }
 (void)p;
}
static int r4_nav_motion_check(void*user,double v,double w,double horizon){
 r4_nav_check_context*q=user;nav_engine*n=q->n;uint64_t now=q->now;
 if(!isfinite(v)||!isfinite(w)||horizon<=0||horizon>4)return r6_record_check(n,R6_CHECK_INVALID,"ACTION_INPUT_INVALID",v,w);
 /* 普通候选检查完整反应+制动段，不强迫每次转完0.8秒；拟合后退的全距离保留。 */
 if(v>=0||horizon<=.800001)horizon=fmin(horizon,nav_progressive_horizon(n,now,v,w));
 if(v>0&&!r4_direction_usable(&n->r4_input.direction[0],now,n->c.max_scan_age_us))return r6_record_check(n,R6_CHECK_UNOBSERVED,"FRONT_DIRECTION_UNAVAILABLE",v,w);
 if(v>0&&n->r4_input.direction[0].clearance_m<R6_FRONT_FORWARD_STOP_MM/1000.)return r6_record_check(n,R6_CHECK_WIRE_REJECT,"FRONT_UNDER_30_REVERSE_OR_TURN_ONLY",v,w);
 return r6_geometric_check(n,q->p,now,v,w,horizon,1);
}
static void r4_configure(nav_engine*n){
 r4_scene_config_default(&n->r4_config);const passage_config*p=&n->c.passage;r4_scene_config*c=&n->r4_config;
 c->direction_max_age_us=n->c.max_scan_age_us;c->body_length_m=p->length_m;c->body_width_m=p->width_m;c->body_pad_m=.01;
 c->side_straight_m=p->side_straight_emergency_mm/1000.;c->w_straight=p->w_thresh_straight_rad_s;
 c->band_up_hyst=p->band_up_hyst;c->band_up_hold_us=(uint64_t)(p->band_up_hold_ms*1000);
 c->scene_confirm_us=(uint64_t)(p->scene_confirm_ms*1000);c->scene_exit_us=(uint64_t)(p->scene_exit_ms*1000);
 c->doorway_length_m=p->doorway_len_max_m;c->doorway_max_us=(uint64_t)(p->doorway_time_max_ms*1000);
 c->explore_step_rad=p->explore_turn_step_deg*PI/180;c->explore_side_budget_rad=p->explore_side_budget_deg*PI/180;
 c->explore_fast_m_s=p->explore_probe_fast_m_s;c->explore_crawl_m_s=p->explore_probe_crawl_m_s;
 c->explore_progress_m=p->explore_progress_mm/1000.;c->explore_window_us=(uint64_t)(p->explore_progress_s*1e6);
 c->explore_rounds=(uint32_t)p->explore_rounds;c->reverse_attempts=(uint32_t)p->reverse_attempts;c->reverse_extended_attempts=(uint32_t)p->extended_reverse_attempts;
 c->continuous_exploration=1;
}
static void r4_prepare(nav_engine*n,const lab_pose*p,nav_xy target,uint64_t now,double v,double w){
 if(!n->r4_initialized){r4_scene_init(&n->r4_state);n->r4_initialized=1;}
 r4_configure(n);r4_scene_input*i=&n->r4_input;memset(i,0,sizeof(*i));
 i->now_us=now;i->mission_id=n->goal.id;i->x_m=p->x_m;i->y_m=p->y_m;i->yaw_rad=p->yaw_rad;
 i->pose_source_us=n->motion_time;
 i->measured_v=n->measured_v;i->measured_w=n->measured_w;i->desired_v=v;i->desired_w=w;
 i->goal_bearing_rad=wrap(atan2(target.y_m-p->y_m,target.x_m-p->x_m)-p->yaw_rad);
 i->goal_active=n->have_goal&&!n->flow_cruise;i->cruise_active=n->have_goal&&n->flow_cruise;i->pose_valid=n->have_motion_pose&&n->motion_time<=now&&now-n->motion_time<=n->c.max_pose_age_us;
 i->execution_ready=n->flow_exec_known&&n->flow_exec_ready;i->committed_action=n->s3_action.active;
 r4_fuse(n,p,now,i);
 if(n->r6_step_time!=now){n->r6_check_result=R6_CHECK_CLEAR;n->r6_check_time=0;n->r6_step_time=now;}
 i->have_speed_limit=1;i->speed_limit_m_s=r6_dynamic_forward_limit(n,now);
 flow_linear_cap(&i->desired_v,&i->desired_w,i->speed_limit_m_s);
 /* 不因任意一侧近障提前清掉全部转向。每个拟议动作由场景机、
  * 完整扫掠及最终FPGA意图门分别检查，安全的离开侧弧线仍有机会。 */
 r4_nav_check_context context={n,p,now};i->motion_check=r4_nav_motion_check;i->motion_context=&context;i->check_result=&n->r6_check_result;
 n->r4_checking=1;r4_scene_step(&n->r4_state,&n->r4_config,i,&n->r4_output);n->r4_checking=0;
 if(n->r6_check_result!=R6_CHECK_CLEAR&&n->r6_check_time!=now){
  int result=n->r6_check_result;n->r6_check_result=R6_CHECK_CLEAR;
  r6_record_check(n,result,r4_check_reason(result),i->desired_v,i->desired_w);
 }
 i->motion_context=NULL;i->motion_check=NULL;i->check_result=NULL;
 n->r4_output.speed_cap=fmin(n->r4_output.speed_cap,i->speed_limit_m_s);
 /* R5默认控制器不向旧规则交权。异常输出在此封闭为可恢复等待。 */
 if(n->r4_output.owner==R4_OWNER_FALLBACK){
  n->r4_output.owner=R4_OWNER_WAIT;n->r4_output.owns_motion=1;
  n->r4_output.v=n->r4_output.w=0;n->r4_output.planner_allowed=0;
  n->r4_output.reason="UNEXPECTED_LEGACY_OWNER_WAIT";
 }
 if(n->r4_output.owner==R4_OWNER_COMMITTED)n->r4_output.owner=R4_OWNER_ROUTE;
 n->r4_active=1;
 n->r4_scene=n->r4_output.scene;n->r4_side_class=fabs(n->r4_output.w)>.1;
 n->r4_calibrated=(uint32_t)n->c.passage.calibrated;
 snprintf(n->r4_scene_reason,sizeof(n->r4_scene_reason),"%s",n->r4_output.reason?n->r4_output.reason:"SCENE_NO_REASON");
 n->s5_front_clearance=i->direction[0].clearance_m;n->s5_corridor_clearance=i->direction[0].clearance_m;
 n->s5_front_approach=r4_direction_usable(&i->direction[0],now,n->c.max_scan_age_us)&&i->direction[0].clearance_m<R6_FRONT_OBSERVE_MM/1000.;
 for(unsigned j=0;j<2;j++){
  const r4_direction*d=&i->direction[j+1];n->s5_side_clearance[j]=d->clearance_m;
  n->s5_side_state[j]=!r4_direction_usable(d,now,n->c.max_scan_age_us)?0:d->clearance_m<.015?3:d->clearance_m<.020?2:d->clearance_m<=.120?1:0;
 }
 if(n->r4_last_logged_scene!=n->r4_output.scene||n->r4_last_logged_stage!=n->r4_output.stage||!n->r4_last_log||now-n->r4_last_log>=1000000){
  unsigned usable=0;uint64_t ages[4];for(unsigned j=0;j<4;j++){const r4_direction*d=&i->direction[j];ages[j]=d->source_us&&d->source_us<=now?now-d->source_us:UINT64_MAX;if(r4_direction_usable(d,now,n->c.max_scan_age_us))usable|=1u<<j;}
  fprintf(stderr,"SCENE_TRANSIT policy_revision=9 controller=R6_UNIFIED legacy_fallback=0 now_us=%llu scene=%u candidate=%u pending=%u owner=%u stage=%u reason=%s usable_mask=%x F_age_us=%llu L_age_us=%llu R_age_us=%llu B_age_us=%llu v=%.4f w=%.4f cap=%.4f front=%.4f left=%.4f right=%.4f rear=%.4f attempts=%u/%u exploration=%u continuous=1\n",(unsigned long long)now,n->r4_output.scene,n->r4_state.candidate_scene,n->r4_state.candidate_scene!=n->r4_state.scene,n->r4_output.owner,n->r4_output.stage,n->r4_scene_reason,usable,(unsigned long long)ages[0],(unsigned long long)ages[1],(unsigned long long)ages[2],(unsigned long long)ages[3],n->r4_output.v,n->r4_output.w,n->r4_output.speed_cap,i->direction[0].clearance_m,i->direction[1].clearance_m,i->direction[2].clearance_m,i->direction[3].clearance_m,n->r4_output.reverse_attempts,n->r4_output.reverse_extended_attempts,n->r4_output.explore_round);
  n->r4_last_logged_scene=n->r4_output.scene;n->r4_last_logged_stage=n->r4_output.stage;n->r4_last_log=now;
 }
}
