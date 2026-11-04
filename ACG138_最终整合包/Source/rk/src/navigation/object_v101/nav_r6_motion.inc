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
 (void)horizon;
 if(!p||!nf_pose_valid(p)||!isfinite(v)||!isfinite(w))return r6_record_check(n,R6_CHECK_INVALID,"CURRENT_ACTION_INPUT_INVALID",v,w);
 if(check_wire){const char*why=flow_sonar_reject_reason(n,now,v,w);if(why)return r6_record_check(n,R6_CHECK_WIRE_REJECT,why,v,w);}
 if(!s5_sonar_geometry(n,v,w,0))return r6_record_check(n,R6_CHECK_OCCUPIED,n->s5_rejected_reason,v,w);
 if(n->c.require_scan){
  if(!n->nearfield.have_context||!context_equal(n->nearfield.context,n->context))return r6_record_check(n,R6_CHECK_INVALID,"CURRENT_SCAN_CONTEXT_MISMATCH",v,w);
  nf_check_result hit=nav_nearfield_check(n,now,v,w,0);
  if(hit.code!=NF_CLEAR)return r6_record_check(n,r6_nf_result(hit.code),nf_check_reason(hit.code),v,w);
 }
 return r6_record_check(n,R6_CHECK_CLEAR,"CURRENT_MEASURED_GUARD_PREDICTION_ADVISORY",v,w);
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
