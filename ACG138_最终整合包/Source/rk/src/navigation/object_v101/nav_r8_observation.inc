/* 仅收到带类型UNOBSERVED后进入主动观察，不把未知当自由，不修改重定位；使用当前保护。每段有界，任务本身无尝试次数上限。

 */
static void r8_observation_reason(nav_engine*n,const char*reason){
 snprintf(n->r6_route_change_reason,sizeof(n->r6_route_change_reason),"%s",reason);
 snprintf(n->r4_scene_reason,sizeof(n->r4_scene_reason),"%s",reason);
}
static int r8_observation_wait(nav_engine*n,uint64_t now,const char*reason,nav_reason why,nav_command*out){
 r8_observation_reason(n,reason);flow_wait(n,now,why,out);return 1;
}
static int r8_observation_fatal_wait(nav_engine*n,uint64_t now,nav_command*out){
 if(n->r6_check_result==R6_CHECK_BUDGET)return r8_observation_wait(n,now,"R8_OBSERVE_COMPUTE_RETRY",NAV_COMPUTE_WAIT,out);
 if(n->r6_check_result==R6_CHECK_STALE||n->r6_check_result==R6_CHECK_POSE_INVALID||n->r6_check_result==R6_CHECK_INVALID)
  return r8_observation_wait(n,now,"R8_OBSERVE_WAIT_FRESH_SOURCES",r6_wait_reason(n),out);
 return 0;
}
static int r8_observation_query(nav_engine*n,const lab_pose*p,uint64_t now,double v,double w,double horizon){
 if(n->r6_check_result==R6_CHECK_BUDGET)return R6_CHECK_BUDGET;
 uint64_t clock=nav_geometry_clock();
 if(n->r6_cycle_deadline&&clock>=n->r6_cycle_deadline)return r6_record_check(n,R6_CHECK_BUDGET,"COMPUTE_BUDGET_RETRY_SAME_ROUTE",v,w);
 n->query_deadline_us=clock?clock+R6_ACTION_GEOMETRY_BUDGET_US:0;
 if(n->r6_cycle_deadline&&n->query_deadline_us>n->r6_cycle_deadline)n->query_deadline_us=n->r6_cycle_deadline;
 if(!s5_candidate_policy(n,v,w))return r6_record_check(n,R6_CHECK_WIRE_REJECT,n->s5_rejected_reason,v,w);
 return r6_geometric_check(n,p,now,v,w,horizon,1);
}
static int r8_observation_command(nav_engine*n,uint64_t now,nav_command*out){
 const char*reason=n->r8_observe.kind==3?"R8_OBSERVE_SHORT_REVERSE":
  n->r8_observe.kind==2?(n->r8_observe.sign==n->r6_preferred_side?"R8_OBSERVE_ROUTE_SIDE_ARC":"R8_OBSERVE_OTHER_SIDE_ARC"):
  n->r8_observe.sign==n->r6_preferred_side?"R8_OBSERVE_ROUTE_SIDE_TURN":"R8_OBSERVE_OTHER_SIDE_TURN";
 r8_observation_reason(n,reason);n->flow_can_start=1;
 n->phase=n->r8_observe.kind==3?NAV_PHASE_BACKUP:n->r8_observe.kind==2?NAV_PHASE_PROBE:NAV_PHASE_ALIGN;
 n->r8_observe.issuing=1;recovering_command(n,now,n->r8_observe.kind==3?NAV_BACKING:NAV_LOCAL_AVOID,n->r8_observe.v,n->r8_observe.w,out);n->r8_observe.issuing=0;return 1;
}
static int r8_observation_step(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,nav_command*out){
 (void)target;
 if(!n->have_goal)return 0;
 if(n->r8_observe.mission!=n->goal.id){memset(&n->r8_observe,0,sizeof(n->r8_observe));n->r8_observe.mission=n->goal.id;}
 if(r8_observation_fatal_wait(n,now,out))return 1;
 if(!n->c.require_scan||!n->nearfield.have_scan||n->scan_time>now||now-n->scan_time>NF_FRESH_US||
    !n->have_motion_pose||n->motion_time>now||now-n->motion_time>NF_FRESH_US)
  return r8_observation_wait(n,now,"R8_OBSERVE_WAIT_FRESH_SOURCES",NAV_NEARFIELD_STALE,out);
 /* 旧路线或场景输出可能仍有惯性，新候选不能把更低请求速度当作实际速度取得许可。
 */
 if(!n->r8_observe.active&&(fabs(n->measured_v)>.025||fabs(n->measured_w)>.10))
  return r8_observation_wait(n,now,"R8_OBSERVE_BRAKE_BEFORE_CHANGE",NAV_RECOVERY_WAIT,out);
 if(n->r8_observe.active){
  if(n->flow_exec_known&&n->flow_exec_ready&&n->motion_time>n->r8_observe.pose_us){
   uint64_t dt=n->motion_time-n->r8_observe.pose_us;
   double da=wrap(p->yaw_rad-n->r8_observe.last_yaw);
   if(dt<=250000&&fabs(da)<=2*n->c.max_yaw_rate_rad_s*dt/1e6+.03){
    n->r8_observe.active_us+=dt;
    if(n->r8_observe.kind==1)n->r8_observe.angle+=fmax(0,n->r8_observe.sign*da);
   }
  }
  n->r8_observe.pose_us=n->motion_time;n->r8_observe.last_yaw=p->yaw_rad;
  double moved=(p->x_m-n->r8_observe.x)*cos(n->r8_observe.yaw)+(p->y_m-n->r8_observe.y)*sin(n->r8_observe.yaw);
  if(n->r8_observe.kind==3)moved=-moved;
  double speed=fabs(n->measured_v),stop=speed*speed/(2*fmax(.01,n->c.brake_m_s2))+speed*.05+.01;
  double angular_stop=n->measured_w*n->measured_w/(2*fmax(.01,n->c.angular_acceleration_rad_s2))+fabs(n->measured_w)*.05;
  int executing=n->flow_exec_known&&n->flow_exec_ready;
  int done=executing&&(n->r8_observe.active_us>=R6_REVERSE_MAX_MS*1000u||
   (n->r8_observe.kind==1?(n->r8_observe.angle>=20*PI/180||n->r8_observe.angle+angular_stop>=20*PI/180):
    moved>=n->r8_observe.target_m||moved+stop>=n->r8_observe.target_m||moved>=(n->r8_observe.kind==3?R6_REVERSE_HARD_M:.12)));
  if(done){
   fprintf(stderr,"OBSERVATION_ACTION policy=CURRENT_MEASURED prediction=ADVISORY_ONLY now_us=%llu goal=%u stage=SEGMENT_DONE kind=%u angle_rad=%.4f moved_m=%.4f active_us=%llu target_m=%.4f scan_us=%llu\n",(unsigned long long)now,n->goal.id,n->r8_observe.kind,n->r8_observe.angle,moved,(unsigned long long)n->r8_observe.active_us,n->r8_observe.target_m,(unsigned long long)n->scan_time);
   n->r8_observe.active=0;n->r8_observe.scan_us=n->scan_time;
   return r8_observation_wait(n,now,"R8_OBSERVE_SEGMENT_DONE_RESCAN",NAV_NEARFIELD_OBSERVATION,out);
  }
  double remaining=n->r8_observe.kind==1?fmax(0,20*PI/180-n->r8_observe.angle)/fabs(n->r8_observe.w):
   fmax(0,n->r8_observe.target_m-fmax(0,moved))/fabs(n->r8_observe.v);
  if(n->r8_observe.kind==3&&!flow_reverse_clear(n,p,points,count,now,fmin(.025,fmax(0,n->r8_observe.target_m-fmax(0,moved))))){
   if(r8_observation_fatal_wait(n,now,out))return 1;
   n->r8_observe.active=0;return r8_observation_wait(n,now,"R8_OBSERVE_BRAKE_BEFORE_CHANGE",NAV_RECOVERY_WAIT,out);
  }
  double horizon=fmax(nav_progressive_horizon(n,now,n->r8_observe.v,n->r8_observe.w),remaining);
  int check=r8_observation_query(n,p,now,n->r8_observe.v,n->r8_observe.w,horizon);
  if(check==R6_CHECK_CLEAR)return r8_observation_command(n,now,out);
  if(r8_observation_fatal_wait(n,now,out))return 1;
  n->r8_observe.active=0;
  return r8_observation_wait(n,now,"R8_OBSERVE_BRAKE_BEFORE_CHANGE",NAV_RECOVERY_WAIT,out);
 }
 /* 一轮有界候选耗尽不取消任务，等待不同扫描后再开始下一轮，避免对同一缺失证据空转；优选路线恢复时立即中断观察段。


 */
 if(n->r8_observe.scan_us&&n->scan_time<=n->r8_observe.scan_us)
  return r8_observation_wait(n,now,"R8_OBSERVE_RECHECK_NEW_SCAN",NAV_NEARFIELD_OBSERVATION,out);
 int sign=n->r6_preferred_side?n->r6_preferred_side:(n->r4_input.goal_bearing_rad>=0?1:-1);
 unsigned checked=0;
 while(checked++<3){
  unsigned at=n->r8_observe.cursor%7;n->r8_observe.cursor=(at+1)%7;
  double v=0,w=0;unsigned kind=1;
  if(at==0)w=sign*.70;else if(at==1)w=sign*.35;
  else if(at==2){v=.12;w=sign*.35;kind=2;}
  else if(at==3)w=-sign*.70;else if(at==4)w=-sign*.35;
  else if(at==5){v=.12;w=-sign*.35;kind=2;}
  else{v=-R6_REVERSE_SPEED_M_S;kind=3;}
  r4_action_limit(n,&v,&w);r4_flow_limit(n,&v,&w);
  if(fabs(v)<1e-9&&fabs(w)<1e-9)continue;
  double target_m=kind==3?R6_REVERSE_TARGET_M:kind==2?.10:0;
  if(kind==3){
   int fitted=0;for(unsigned fit=0;fit<2;fit++){
    target_m=fit?R6_REVERSE_MIN_FIT_M:R6_REVERSE_TARGET_M;
    if(flow_reverse_clear(n,p,points,count,now,target_m)){fitted=1;break;}
    if(n->r6_check_result==R6_CHECK_BUDGET)break;
   }
   if(!fitted){
   if(r8_observation_fatal_wait(n,now,out))return 1;
   if(at==6){n->r8_observe.scan_us=n->scan_time;return r8_observation_wait(n,now,"R8_OBSERVE_NO_VERIFIED_ACTION",NAV_NEARFIELD_OBSERVATION,out);}continue;
   }
  }
  double segment=kind==1?(20*PI/180)/fabs(w):target_m/fabs(v);
  double horizon=fmax(nav_progressive_horizon(n,now,v,w),segment);
  int check=r8_observation_query(n,p,now,v,w,horizon);
  if(check==R6_CHECK_CLEAR){
   n->r8_observe.active=1;n->r8_observe.kind=kind;n->r8_observe.sign=w>=0?1:-1;
   n->r8_observe.x=p->x_m;n->r8_observe.y=p->y_m;n->r8_observe.yaw=n->r8_observe.last_yaw=p->yaw_rad;
   n->r8_observe.pose_us=n->motion_time;n->r8_observe.active_us=0;n->r8_observe.angle=0;
   n->r8_observe.v=v;n->r8_observe.w=w;n->r8_observe.target_m=target_m;
   fprintf(stderr,"OBSERVATION_ACTION policy=CURRENT_MEASURED prediction=ADVISORY_ONLY now_us=%llu goal=%u stage=SELECT kind=%u candidate=%u v=%.4f w=%.4f horizon_s=%.3f scan_us=%llu original_route_side=%d\n",(unsigned long long)now,n->goal.id,kind,at,v,w,horizon,(unsigned long long)n->scan_time,n->r6_preferred_side);
   return r8_observation_command(n,now,out);
  }
  if(n->r6_check_result==R6_CHECK_BUDGET)n->r8_observe.cursor=at;
  if(r8_observation_fatal_wait(n,now,out))return 1;
  if(at==6){n->r8_observe.scan_us=n->scan_time;return r8_observation_wait(n,now,"R8_OBSERVE_NO_VERIFIED_ACTION",NAV_NEARFIELD_OBSERVATION,out);}
 }
 return r8_observation_wait(n,now,"R8_OBSERVE_RECHECK_NEW_SCAN",NAV_NEARFIELD_OBSERVATION,out);
}
