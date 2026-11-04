/* 实物入口适配器使用参考路线和当前保护，不另建评分器、轨迹选择器或恢复状态机。实际ODOM目标转为nav_goal，通道规划、导航、流程、场景、近场及S6建议共用当前执行检查。


 */
void nav_object_time_profile(nav_engine*n){if(n){n->object_time_profile=1;n->c.max_pose_age_us=OBJECT_CONTROL_POSE_US;n->c.max_scan_age_us=OBJECT_CONTROL_SCAN_US;n->nearfield.pose_max_age_us=n->c.max_pose_age_us;n->nearfield.scan_max_age_us=n->c.max_scan_age_us;}}
void nav_external_profile(nav_engine*n,unsigned mode){if(n&&(mode==1||mode==2))n->external_profile=mode;}
int nav_raw_pose_at(const nav_engine*n,uint64_t stamp,lab_pose*out){return n&&out&&nf_pose_at(&n->nearfield,stamp,out);}
lab_status nav_external_target_v2(nav_engine*n,const object_route_target_v2*t){
 if(!n||!t||t->mode<1||t->mode>2||t->trail_count>OBJECT_ROUTE_V2_MAX||t->context.timeline_id!=n->context.timeline_id||t->context.origin_epoch!=n->context.origin_epoch)return LAB_ERR_CONTEXT;
 if(t->static_search_left>1||t->static_bearing_approach>1||(t->static_search_left&&t->static_bearing_approach))return LAB_ERR_ARGUMENT;
 if((t->static_search_left||t->static_bearing_approach)&&(t->mode!=1||t->valid||t->goal_valid||t->trail_count))return LAB_ERR_ARGUMENT;
 if(t->static_search_left&&(t->epoch||t->track_id))return LAB_ERR_ARGUMENT;
 if(t->static_bearing_approach&&(!t->epoch||t->track_id<1))return LAB_ERR_ARGUMENT;
 if(t->valid&&(!t->epoch||t->track_id<1||!t->source_us||t->source_us>t->published_us||!isfinite(t->target_x_m)||!isfinite(t->target_y_m)||!isfinite(t->goal_x_m)||!isfinite(t->goal_y_m)))return LAB_ERR_ARGUMENT;
 if(t->visual_bearing_valid&&(!t->visual_source_us||t->visual_source_us>t->published_us||!isfinite(t->visual_bearing_rad)||fabs(t->visual_bearing_rad)>PI))return LAB_ERR_ARGUMENT;
 if(t->control_bearing_valid&&(!t->control_source_us||t->control_source_us>t->published_us||!isfinite(t->control_bearing_rad)||fabs(t->control_bearing_rad)>PI||t->control_turning>1))return LAB_ERR_ARGUMENT;
 for(unsigned k=0;k<t->trail_count;k++)if(!isfinite(t->trail[k].x_m)||!isfinite(t->trail[k].y_m)||!t->trail[k].source_us||t->trail[k].source_us>t->published_us||(k&&t->trail[k].source_us<=t->trail[k-1].source_us))return LAB_ERR_ARGUMENT;
 int identity=n->object_target.epoch==t->epoch&&n->object_target.track_id==t->track_id&&n->object_target.generation==t->generation&&n->object_target.mode==t->mode;
 if(identity&&t->published_us<n->object_target.published_us)return LAB_ERR_TIME;
 if(!identity){nav_cancel(n,t->published_us);n->object_plan_us=0;}
 else if(t->goal_valid&&hypot(t->goal_x_m-n->object_target.goal_x_m,t->goal_y_m-n->object_target.goal_y_m)>.05)n->dirty=1;
 n->object_target=*t;return LAB_OK;
}
/* 用局部实际射线证据建立有界栅格视图，未知保持-1，不生成全局SLAM图。ODOM引擎不被定位重构坐标，遇到异常旋转坐标时拒绝执行，不重标坐标或构造格。


 */
static void object_adapter_grid(nav_engine*n,uint64_t now){
 const passage_local*l=n->local;
 if(!l||!l->initialized||fabs(l->map_from_local_yaw)>1e-12||!n->scan_time){n->object_grid_source=0;return;}
 if(n->object_grid_source==n->scan_time)return;
 passage_world world=rectangle_world(n,(passage_pose){0});world.local_only=1;world.confirmed_only=0;world.now_us=now;
 for(unsigned y=0;y<PASSAGE_LOCAL_SIDE;y++)for(unsigned x=0;x<PASSAGE_LOCAL_SIDE;x++){
  double wx=l->origin_x+l->map_from_local_x+(x+.5)*l->resolution,wy=l->origin_y+l->map_from_local_y+(y+.5)*l->resolution;
  int value=passage_planning_sample(&world,wx,wy);n->object_grid[y*PASSAGE_LOCAL_SIDE+x]=(int8_t)(value<0?-1:value);
 }
 n->object_grid_source=n->scan_time;
}
/* 参考规划器支持local_only实际射线世界，同步有界规划不建立合成全局自由图。
 */
static nav_reason object_adapter_plan(nav_engine*n,nav_xy start){
 passage_pose p={start.x_m,start.y_m,n->motion_yaw};passage_world world=rectangle_world(n,p);unsigned count=0,expanded=0;
 world.local_only=1;world.planning_unknown_soft=0;++n->plans;n->path_n=n->segment=0;
 int rc=passage_plan_ex(&world,p,(passage_xy){n->goal.x_m,n->goal.y_m},n->goal.has_yaw,n->goal.yaw_rad,n->rectangle_path,PASSAGE_PATH_MAX,&count,&expanded,&n->route_report);
 n->expanded+=expanded;n->passage_collision.reason=rc;
 if(rc!=PASSAGE_CLEAR&&rc!=PASSAGE_PARTIAL)return rc==PASSAGE_SEARCH_LIMIT?NAV_SEARCH_LIMIT:NAV_NO_PATH;
 for(unsigned k=0;k<count;k++)n->path[k]=(nav_xy){n->rectangle_path[k].x,n->rectangle_path[k].y};n->path_n=count;return NAV_REASON_NONE;
}
/* 静止语义初始搜索无实测身份，绑定后只有方向时不构造距离或目标；仍检查真实来源、上下文、时钟、当前车体近障及线上报文。

 */
static lab_status nav_static_intent_step(nav_engine*n,uint64_t now,const lab_event*p,const lab_event*scan,const lab_event*sp,double v,double w,double cap,nav_command*out){
 const object_route_target_v2*t=&n->object_target;
 if(t->mode!=1||n->external_profile==2||t->valid||t->goal_valid)return stop(n,now,NAV_WAITING,NAV_INVALID_INPUT,LAB_ERR_ARGUMENT,out);
 if(t->static_search_left&&(t->epoch||t->track_id||fabs(v)>1e-9||w<0||w>.20+1e-9))return stop(n,now,NAV_WAITING,NAV_INVALID_INPUT,LAB_ERR_ARGUMENT,out);
 if(t->static_bearing_approach&&(!t->epoch||t->track_id<1||v<0||v>.25||fabs(w)>.30+1e-9))return stop(n,now,NAV_WAITING,NAV_INVALID_INPUT,LAB_ERR_ARGUMENT,out);
 if(t->static_bearing_approach&&v>0&&(!t->control_bearing_valid||!t->control_source_us||t->control_source_us>now||now-t->control_source_us>3000000))return stop(n,now,NAV_WAITING,NAV_INVALID_INPUT,LAB_AGAIN,out);
 if(now>UINT64_MAX-n->c.command_valid_us||(n->have_tick&&now<=n->last_tick))return stop(n,now,NAV_FAILED,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
 if(!context_event(n,p)||!context_event(n,scan)||!context_event(n,sp))return stop(n,now,NAV_WAITING,NAV_BAD_CONTEXT,LAB_ERR_CONTEXT,out);
 if(!pose_valid(p)||!pose_valid(sp)||!nf_pose_valid(&p->data.pose)||!nf_pose_valid(&sp->data.pose))return stop(n,now,NAV_WAITING,NAV_INVALID_POSE,LAB_AGAIN,out);
 if(p->meta.time_us>now||p->meta.received_us>now||sp->meta.received_us>now||now-p->meta.time_us>n->c.max_pose_age_us)return stop(n,now,NAV_WAITING,NAV_POSE_STALE,LAB_AGAIN,out);
 if(scan->type!=LAB_EVENT_SCAN||!w1_real_carrier_event(scan)||!scan_geometry_valid(&scan->data.scan)||(scan->meta.flags&3)!=3||scan->meta.time_us!=sp->meta.time_us||scan->meta.time_us>now||scan->meta.received_us>now||now-scan->meta.time_us>n->c.max_scan_age_us)return stop(n,now,NAV_WAITING,NAV_SCAN_STALE,LAB_AGAIN,out);
 if(!n->nf_external){nf_observe(&n->nearfield,sp,now);nf_observe(&n->nearfield,scan,now);nf_observe(&n->nearfield,p,now);}
 n->last_tick=now;n->have_tick=1;n->nf_now=now;n->scan_time=scan->meta.time_us;n->scan_sequence=scan->meta.sequence;n->have_scan_sequence=1;
 observe_motion(n,p,now);n->external_active=1;n->have_goal=0;memset(&n->goal,0,sizeof(n->goal));n->path_n=n->segment=0;
 v=fmin(cap,v);int check=r6_geometric_check(n,&p->data.pose,now,v,w,0,1);
 if(check!=R6_CHECK_CLEAR)return flow_wait(n,now,r6_wait_reason(n),out);
 n->state=NAV_TRACKING;n->reason=NAV_REASON_NONE;n->blocking_reason=NAV_REASON_NONE;n->phase=fabs(v)<1e-9?NAV_PHASE_ALIGN:NAV_PHASE_TRACK;
 snprintf(n->r6_route_change_reason,sizeof(n->r6_route_change_reason),"%s",t->static_search_left?"STATIC_INITIAL_LEFT_CURRENT_GUARD":"STATIC_BEARING_CURRENT_GUARD_NO_RANGE");
 command(n,now,v,w,out);return LAB_OK;
}
lab_status nav_external_step(nav_engine*n,uint64_t now,const lab_event*p,const lab_event*scan,const lab_event*sp,double desired_v,double desired_w,double cap,nav_command*out){
 if(!n||!out||!isfinite(desired_v)||!isfinite(desired_w)||!isfinite(cap)||cap<=0||cap>.25)return LAB_ERR_ARGUMENT;
 /* STOP或保持状态由任务决定，参考恢复不能重新启动保持任务。 */
 if(n->object_target.zero_required||(fabs(desired_v)<1e-9&&fabs(desired_w)<1e-9))return stop(n,now,NAV_WAITING,NAV_RECOVERY_WAIT,LAB_OK,out);
 if(n->object_target.static_search_left||n->object_target.static_bearing_approach)return nav_static_intent_step(n,now,p,scan,sp,desired_v,desired_w,cap,out);
 if(!context_event(n,p)||!context_event(n,scan)||!context_event(n,sp))return stop(n,now,NAV_WAITING,NAV_BAD_CONTEXT,LAB_ERR_CONTEXT,out);
 int turn_only=fabs(desired_v)<1e-9&&fabs(desired_w)>1e-9;
 if(!turn_only&&(!n->object_target.valid||!n->object_target.goal_valid))return stop(n,now,NAV_WAITING,NAV_NO_GOAL,LAB_AGAIN,out);
 nav_goal g={.id=(uint32_t)(n->object_target.generation?n->object_target.generation:1),.x_m=n->object_target.goal_x_m,.y_m=n->object_target.goal_y_m};
 if(turn_only){g.has_yaw=1;g.x_m=p->data.pose.x_m;g.y_m=p->data.pose.y_m;g.yaw_rad=wrap(p->data.pose.yaw_rad+copysign(.60,desired_w));}
 int first=!n->external_active||!n->have_goal;
 if(first){lab_status rc=nav_set_goal(n,&g,now);if(rc!=LAB_OK)return rc;}
 else if(g.has_yaw!=n->goal.has_yaw||hypot(g.x_m-n->goal.x_m,g.y_m-n->goal.y_m)>.05){n->dirty=1;n->state=NAV_WAITING;n->waiting=0;}
 n->goal=g;n->have_goal=1;n->external_active=1;
 if(n->state==NAV_REACHED){n->state=NAV_WAITING;n->dirty=1;}
 double saved_v=n->c.max_speed_m_s,saved_w=n->c.max_yaw_rate_rad_s;
 double yaw_cap=n->external_profile==2?(turn_only&&fabs(desired_w)>OBJECT_FOLLOW_NORMAL_YAW_V6?OBJECT_FOLLOW_DIRECTED_YAW_V6:OBJECT_FOLLOW_NORMAL_YAW_V6):.30;
 if(turn_only)yaw_cap=fmin(yaw_cap,fabs(desired_w));n->c.max_speed_m_s=fmin(saved_v,cap);n->c.max_yaw_rate_rad_s=fmin(saved_w,yaw_cap);
 lab_status rc=nav_step(n,now,p,scan,sp,out);
 /* 兼容速度上限作用于实际执行曲率，并由当前实测保护复核，未来几何仍仅供建议。
 */
 if(rc==LAB_OK){double v=out->linear_m_s,w=out->angular_rad_s,scale=1;
  if(fabs(w)>yaw_cap)scale=fmin(scale,yaw_cap/fabs(w));if(v>cap)scale=fmin(scale,cap/v);
  if(turn_only&&(fabs(v)>1e-9||w*desired_w<0)){rc=flow_wait(n,now,NAV_RECOVERY_WAIT,out);}
  else if(scale<1){v*=scale;w*=scale;int checked=r6_geometric_check(n,&p->data.pose,now,v,w,nav_progressive_horizon(n,now,v,w),1);
   if(checked!=R6_CHECK_CLEAR)rc=flow_wait(n,now,r6_wait_reason(n),out);else{nav_reason why=n->reason;command(n,now,v,w,out);n->reason=why;}
  }
 }
 n->c.max_speed_m_s=saved_v;n->c.max_yaw_rate_rad_s=saved_w;return rc;
}
