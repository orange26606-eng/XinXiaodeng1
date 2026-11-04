/* 带类型证据及保持路线的限速，不访问设备。 */
static double r6_reaction_seconds(const nav_engine*n,uint64_t now){
 double age=0;
 if(n->scan_time&&n->scan_time<=now)age=(now-n->scan_time)/1e6;
 if(n->motion_time&&n->motion_time<=now)age=fmax(age,(now-n->motion_time)/1e6);
 const r4_direction*f=&n->r4_input.direction[0];
 if(f->source_us&&f->source_us<=now)age=fmax(age,(now-f->source_us)/1e6);
 return fmax(.20,n->c.command_valid_us/1e6)+age+fmax(0,n->time_uncertainty_s)+.05;
}
static double r6_dynamic_forward_limit(const nav_engine*n,uint64_t now){
 const r4_direction*f=&n->r4_input.direction[0];
 if(!r4_direction_usable(f,now,CONT_SCAN_US)||f->clearance_m<R6_FRONT_FORWARD_STOP_MM/1000.)return 0;
 double space=fmax(0,f->clearance_m-R6_BODY_BUFFER_MM/1000.);
 double reaction=r6_reaction_seconds(n,now),a=fmax(.01,n->c.brake_m_s2);
 /* 稳定求解v×T+v²/(2a)=space的正根，源年龄只计算一次。 */
 double maximum=2*space/(sqrt(reaction*reaction+2*space/a)+reaction);
 return fmin(n->c.max_speed_m_s,maximum);
}
static int r6_record_check(nav_engine*n,int result,const char*reason,double v,double w){
 if(n->r6_check_result==R6_CHECK_BUDGET){if(!n->r6_check_time)n->r6_check_time=n->last_tick;if(!n->s5_rejected_reason[0])s5_reject(n,v,w,"COMPUTE_BUDGET_RETRY_SAME_ROUTE");return R6_CHECK_BUDGET;}
 n->r6_check_result=result;n->r6_check_time=n->last_tick;
 if(result!=R6_CHECK_CLEAR){if(reason==n->s5_rejected_reason){n->s5_rejected_v=v;n->s5_rejected_w=w;}else s5_reject(n,v,w,reason);}
 else{n->s5_rejected_reason[0]=0;n->s5_rejected_v=n->s5_rejected_w=0;}
 return result;
}
static int r6_nf_result(nf_check_code code){
 switch(code){case NF_CLEAR:return R6_CHECK_CLEAR;case NF_COLLISION:return R6_CHECK_OCCUPIED;
 case NF_NEED_OBSERVATION:return R6_CHECK_UNOBSERVED;case NF_DATA_STALE:return R6_CHECK_STALE;
 case NF_POSE_INVALID:return R6_CHECK_POSE_INVALID;default:return R6_CHECK_INVALID;}
}
static int r6_geometric_check(nav_engine*n,const lab_pose*p,uint64_t now,double v,double w,double horizon,int check_wire){
 if(n->r6_check_result==R6_CHECK_BUDGET||nav_geometry_cancel(n))return r6_record_check(n,R6_CHECK_BUDGET,"COMPUTE_BUDGET_RETRY_SAME_ROUTE",v,w);
 if(!isfinite(v)||!isfinite(w)||!isfinite(horizon)||horizon<=0||horizon>4)return r6_record_check(n,R6_CHECK_INVALID,"ACTION_INPUT_INVALID",v,w);
 if(check_wire){
  const char*why=flow_sonar_reject_reason(n,now,v,w);
  if(why)return r6_record_check(n,R6_CHECK_WIRE_REJECT,why,v,w);
 }
 if(!s5_sonar_geometry(n,v,w,horizon))return r6_record_check(n,R6_CHECK_OCCUPIED,n->s5_rejected_reason[0]?n->s5_rejected_reason:"SONAR_BODY_SWEEP",v,w);
 if(n->c.require_scan){
  if(!n->nearfield.have_scan)return r6_record_check(n,R6_CHECK_STALE,"NEARFIELD_SCAN_UNAVAILABLE",v,w);
  nf_check_result hit=nav_nearfield_check(n,now,v,w,horizon);
  if(hit.code!=NF_CLEAR)return r6_record_check(n,r6_nf_result(hit.code),nf_check_reason(hit.code),v,w);
 }
 passage_pose start={p->x_m,p->y_m,p->yaw_rad};passage_world world=flow_local_world(n,start);passage_result hit={0};int clear;
 if(fabs(w)<1e-9){passage_xy end={start.x+v*horizon*cos(start.yaw),start.y+v*horizon*sin(start.yaw)};clear=passage_translate_clear(&world,start,end,world.config.width_m,&hit);}
 else clear=passage_arc_clear(&world,start,v,w,horizon,&hit);
 if(nav_geometry_cancel(n)||(!clear&&hit.reason==PASSAGE_SEARCH_LIMIT))return r6_record_check(n,R6_CHECK_BUDGET,"COMPUTE_BUDGET_RETRY_SAME_ROUTE",v,w);
 if(!clear)return r6_record_check(n,hit.reason==PASSAGE_OCCUPIED?R6_CHECK_OCCUPIED:hit.reason==PASSAGE_UNKNOWN||hit.reason==PASSAGE_OUTSIDE?R6_CHECK_UNOBSERVED:R6_CHECK_INVALID,passage_reason_text(hit.reason),v,w);
 return r6_record_check(n,R6_CHECK_CLEAR,"CLEAR",v,w);
}
static int r6_current_braking_check(nav_engine*n,const lab_pose*p,uint64_t now){
 if(fabs(n->measured_v)<.01&&fabs(n->measured_w)<.04)return R6_CHECK_CLEAR;
 /* 较低请求目标不能抹去实际惯性或启动助力，批准新运动前按实测反馈确认完全停止。
 */
 double horizon=nav_progressive_horizon(n,now,n->measured_v,n->measured_w);
 return r6_geometric_check(n,p,now,n->measured_v,n->measured_w,fmin(4,horizon),0);
}
static nav_reason r6_wait_reason(const nav_engine*n){
 switch(n->r6_check_result){case R6_CHECK_BUDGET:return NAV_COMPUTE_WAIT;
 case R6_CHECK_UNOBSERVED:return NAV_NEARFIELD_OBSERVATION;case R6_CHECK_STALE:return NAV_NEARFIELD_STALE;
 case R6_CHECK_POSE_INVALID:return NAV_NEARFIELD_POSE;case R6_CHECK_WIRE_REJECT:return NAV_NEAR_AVOID;
 case R6_CHECK_INVALID:return NAV_NEARFIELD_INVALID;default:return NAV_OBSTACLE;}
}
static int r6_side_of(double w){return w>.02?1:w<-.02?-1:0;}
static int r6_route_opposite(const nav_engine*n,double w){return !n->flow_cruise&&n->r6_preferred_side&&r6_side_of(w)&&r6_side_of(w)!=n->r6_preferred_side;}
