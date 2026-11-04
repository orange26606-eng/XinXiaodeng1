/* 按实测局部动作执行承诺，目标由几何选择；原年龄传感器及完整车体扫掠可随时中断。
 */
static void s3_progress(nav_engine*n,const lab_pose*p,uint64_t now){
 uint64_t stamp=n->motion_time?n->motion_time:now;
 if(!n->s3_action.active||!(n->s3_action.stage==1||n->s3_action.stage==2))return;
 if(!n->s3_action_pose_time){n->s3_action_pose_time=stamp;n->s3_action_last_yaw=p->yaw_rad;return;}
 if(stamp<=n->s3_action_pose_time)return;
 double dt=(stamp-n->s3_action_pose_time)/1e6,delta=wrap(p->yaw_rad-n->s3_action_last_yaw);
 if((!n->flow_exec_known||n->flow_exec_ready)&&fabs(delta)<=2*n->c.max_yaw_rate_rad_s*dt+.03)
  n->s3_action.turn_progress_rad+=delta;
 n->s3_action_pose_time=stamp;n->s3_action_last_yaw=p->yaw_rad;
}
static void s3_action_end(nav_engine*n,int completed,const char*why){
 fprintf(stderr,"S3_ACTION_END event=%u stage=%u completed=%d angle_target=%.3f angle_done=%.3f backup_target=%.3f backup_done=%.3f reason=%s\n",n->s3_action.event_id,n->s3_action.stage,completed,n->s3_action.turn_target_rad,n->s3_action.turn_progress_rad,n->s3_action.backup_target_m,n->s3_action.backup_progress_m,why);
 if(n->s3_action.active){if(completed)++n->s3_action.completed_actions;else ++n->s3_action.interrupted_actions;}
 n->s3_action.active=0;n->s3_action.stage=completed?4:5;
 snprintf(n->s3_action.reason,sizeof(n->s3_action.reason),"%s",why);
 n->s3_exit_confirmations=0;n->s3_action_pose_time=0;
}
static double s3_angle_target(const nav_engine*n,const s3_nearfield_view*nf,int sign,int arc){
 double lo=(arc?20:40)*PI/180,hi=(arc?40:70)*PI/180;
 if(!arc&&(n->s5_force_corner||(nf->front_m>=0&&nf->front_m<.15))){lo=70*PI/180;hi=100*PI/180;}
 double edge=0,hx=n->c.passage.length_m*.5,hy=n->c.passage.width_m*.5;
 for(unsigned i=0;i<nf->hazard_count&&i<S3_HAZARD_VIEW_MAX;i++){
  double x=nf->hazards[i].x_m,y=sign*nf->hazards[i].y_m;
  if(x<hx-.03||x>hx+1.0||y<-.65||y>.65)continue;
  double a=atan2(fmax(.01,y+hy+.025),fmax(.04,x));if(a>edge)edge=a;
 }
 return sign*clamp(edge+8*PI/180,lo,hi);
}
static int s3_full_target_clear(nav_engine*n,const lab_pose*p,uint64_t now,double v,double w,double angle){
 if(fabs(w)<.02)return 0;if(!s5_sonar_geometry(n,v,w,fmax(.8,fabs(angle/w))))return 0;if(n->s5_front_clearance>=.05&&n->s5_front_clearance<.10&&fabs(v)>.001)return 0;double duration=fabs(angle/w);if(!isfinite(duration)||duration>6)return 0;
 if(n->s5_front_clearance>=.05&&n->s5_front_clearance<.10&&n->c.require_scan&&!nf_rotation_clear(&n->nearfield,now,angle))return 0;
 if(n->c.require_scan&&!nf_motion_clear(&n->nearfield,now,v,w,duration))return 0;
 passage_pose at={p->x_m,p->y_m,p->yaw_rad};passage_world world=flow_local_world(n,at);passage_result hit;
 if(fabs(v)<.001)return passage_rotate_clear(&world,at,wrap(at.yaw+angle),&hit);
 unsigned steps=(unsigned)ceil(fmax(fabs(angle)/.035,fabs(v)*duration/.015));if(!steps)steps=1;if(steps>128)return 0;
 double dt=duration/steps;
 for(unsigned k=0;k<steps;k++){
  double x=at.x,y=at.y,a=at.yaw;advance(&x,&y,&a,v,w,dt);
  if(!passage_rotate_clear(&world,at,a,&hit))return 0;at.yaw=a;
  if(!passage_translate_clear(&world,at,(passage_xy){x,y},world.config.width_m,&hit))return 0;at.x=x;at.y=y;
 }
 return 1;
}
static int s3_action_begin(nav_engine*n,const lab_pose*p,uint64_t now,double v,double w,const s3_nearfield_view*nf){
 int sign=w>0?1:-1;unsigned stage=fabs(v)>.01?1:2;
 double angle=s3_angle_target(n,nf,sign,stage==1),minimum=(stage==1?20:(nf->front_m>=0&&nf->front_m<.15)?70:40)*PI/180;
 /* 目标转角描述机动目的；每次发布前检查反应与制动所需的可执行段，完整目标用于提前评估。 */
 (void)minimum;
 if(!flow_sonar_allows(n,now,v,w)||!flow_sonar_geometry(n,v,w,nav_action_intent(n,v,w)==S1_ESCAPE)||!s5_candidate_policy(n,v,w)||!nav_nearfield_allows(n,now,v,w)||sweep_reason(n,p,v,w,NULL,0)!=NAV_REASON_NONE)return 0;
 int full_clear=s3_full_target_clear(n,p,now,v,w,angle);
 uint32_t event=n->s3_action.event_id+1,done=n->s3_action.completed_actions,interrupted=n->s3_action.interrupted_actions;
 memset(&n->s3_action,0,sizeof(n->s3_action));n->s3_action.active=1;n->s3_action.stage=stage;n->s3_action.event_id=event;n->s3_action.direction=sign;
 n->s3_action.completed_actions=done;n->s3_action.interrupted_actions=interrupted;
 n->s3_action.turn_target_rad=angle;
 n->s3_action_v=v;n->s3_action_w=w;
 n->s3_action_yaw=n->s3_action_last_yaw=p->yaw_rad;n->s3_action_pose_time=n->motion_time?n->motion_time:now;
 n->s3_exit_scan=0;n->s3_exit_confirmations=0;n->turn_dir=sign;n->flow_direction_until=now+600000;
 snprintf(n->s3_action.reason,sizeof(n->s3_action.reason),"%s",full_clear?(stage==1?"LIDAR_EARLY_ARC":"LIDAR_MEASURED_TURN"):(stage==1?"PROGRESSIVE_EARLY_ARC":"PROGRESSIVE_MEASURED_TURN"));
 fprintf(stderr,"S3_ACTION_BEGIN event=%u stage=%u direction=%d target_rad=%.3f front_m=%.3f\n",event,stage,sign,n->s3_action.turn_target_rad,nf->front_m);
 return 1;
}
/* 完成条件核对整个旋转车体及后续0.25m走廊，不只看某条中心射线变远。
 */
static int s3_exit_clear(nav_engine*n,const lab_pose*p,uint64_t now,double yaw){
 passage_pose start={p->x_m,p->y_m,p->yaw_rad};passage_world world=flow_local_world(n,start);passage_result hit;
 if(!passage_rotate_clear(&world,start,yaw,&hit))return 0;
 start.yaw=yaw;passage_xy end={start.x+.25*cos(yaw),start.y+.25*sin(yaw)};
 return passage_translate_clear(&world,start,end,world.config.width_m,&hit)&&
        nav_nearfield_allows(n,now,.12,0)&&flow_sonar_allows(n,now,.12,0)&&flow_sonar_geometry(n,.12,0,0);
}
static int s3_committed_step(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,int new_scan,nav_command*out){
 if(!n->s3_action.active||(n->s3_action.stage!=1&&n->s3_action.stage!=2))return 0;
 s3_progress(n,p,now);int sign=n->s3_action.direction;
 double progress=sign*n->s3_action.turn_progress_rad,wanted=fabs(n->s3_action.turn_target_rad);
 if(new_scan&&n->scan_time!=n->s3_exit_scan){n->s3_exit_scan=n->scan_time;
  if(progress>=wanted-5*PI/180&&s3_exit_clear(n,p,now,p->yaw_rad)){if(n->s3_exit_confirmations<2)++n->s3_exit_confirmations;}
  else n->s3_exit_confirmations=0;
 }
 if(n->s3_exit_confirmations>=2||progress>=wanted){s3_action_end(n,1,"MEASURED_ANGLE_TURN_HOLD");n->flow_pivot=0;n->flow_blocked_since=0;n->s5_turn_hold=1;n->s5_hold_clear=0;n->s5_hold_scan=0;n->s5_resume_limited=1;n->s3_action.stage=6;flow_wait(n,now,NAV_TURN_HOLD,out);return 1;}
 /* 接近目标方位时维持已核对低速前进，等待两次真实新扫描，避免额外完整原地转。
 */
 /* 保留承诺时已通过整车检查的动作原语；下一tick若改为更快速度或不同曲率，可能引入新冲突并导致细碎重试。

 */
 double v=n->s3_action_v,w=n->s3_action_w;
 if(progress>=wanted){v=0;w=0;}
 if(n->narrow)flow_linear_cap(&v,&w,n->c.passage.narrow_speed_m_s);
 if(n->flow_have_hint&&!n->flow_front_stable)flow_linear_cap(&v,&w,.12);flow_linear_cap(&v,&w,s5_speed_cap(n));flow_limit(&v,&w);
 double score=0;
 if(!flow_candidate(n,p,target,points,count,now,v,w,&score)){
  if(n->flow_reject_budget||nav_geometry_cancel(n)){flow_wait(n,now,NAV_COMPUTE_WAIT,out);return 1;}
  /* 缺少观测不等于新测到墙，保留已测目标和方向，由外层检查另一实际观测动作；只有实际碰撞才记该几何方向失败。

 */
  if(n->last_nf_check.code!=NF_CLEAR&&n->last_nf_check.code!=NF_COLLISION){
   if(n->last_nf_check.code==NF_NEED_OBSERVATION){s3_action_end(n,0,"OBSERVATION_RESELECT");n->flow_pivot=0;return 0;}
   flow_wait(n,now,nav_nf_reason(n->last_nf_check.code),out);return 1;
  }
  if(n->flow_reject_unknown&&!n->flow_reject_occupied&&!n->flow_reject_sonar){s3_action_end(n,0,"OBSERVATION_RESELECT");n->flow_pivot=0;return 0;}
  s3_action_end(n,0,"NEW_HAZARD_RESELECT");flow_remember(n,p,now);n->flow_pivot=0;n->s5_brake_reselect=1;flow_wait(n,now,NAV_RECOVERY_WAIT,out);return 1;
 }
 n->phase=v==0?NAV_PHASE_ALIGN:sign>0?NAV_PHASE_DETOUR_LEFT:NAV_PHASE_DETOUR_RIGHT;n->flow_can_start=1;
 recovering_command(n,now,NAV_LOCAL_AVOID,v,w,out);return 1;
}
static double s3_backup_distance(const nav_engine*n,const s3_nearfield_view*nf,int extended){
 double base=extended?.55:.38,hard=extended?.60:.40,front=n->s5_front_clearance;
 if(front<0&&nf->valid&&nf->front_m>=0)front=nf->front_m;
 if(front<0)return base;
 double radius=hypot(n->c.passage.length_m*.5,n->c.passage.width_m*.5);
 double extra=fmax(0,(radius-n->c.passage.length_m*.5)+.10-front);
 return clamp(base+extra,base,hard);
}
