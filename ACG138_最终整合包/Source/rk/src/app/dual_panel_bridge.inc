/* 主程序内桥接，不独立持有传感器、套接字或电机。 */
#include "object_memory_v4.h"
#include "object_time_v4.h"
#include "object_exit_time_v5.h"
#include "object_body_target_v9.h"
#include "semantic_observation.h"
static int dual_export_is_current(const obj_frame*current,const obj_frame*next){return !current->generated_us||next->generated_us>=current->generated_us;}
/* 检测编号来自语义准入，不来自图像选择回执。仅在私有帧副本调用，不重写源字段。
 */
static void dual_semantic_select_frame(const object_dual_task*t,obj_frame*f){
 if(!t||!f||!((t->static_search&&t->search_bound)||(t->semantic_direct&&t->prepared))||f->epoch!=t->epoch)return;
 for(int i=0;i<f->count;i++)if(f->objects[i].track_id==t->track_id){f->selected=t->track_id;return;}
}
/* 复用一个有界临时帧读取原始状态及检测脉冲，不修改公开文件，也不改预测标记或源时间。
 */
static int dual_semantic_read_attributes(const char*dir,obj_frame*frame,uint64_t now,uint64_t epoch,int track,const char*category,const char*color,const char*size){
 if(!dir||!frame)return 0;
 int have=frame->epoch!=0;obj_frame*scratch=malloc(sizeof(*scratch));if(!scratch)return have;
 char path[640];
 if(snprintf(path,sizeof(path),"%s/objects-state.json",dir)<(int)sizeof(path)&&obj_frame_load(path,scratch)){*frame=*scratch;have=1;}
 int have_detector=snprintf(path,sizeof(path),"%s/objects-detections.json",dir)<(int)sizeof(path)&&obj_frame_load(path,scratch);
 const obj_frame*chosen=semantic_observation_pick_attributes(have?frame:NULL,have_detector?scratch:NULL,now,epoch,track,category,color,size);
 if(chosen){if(chosen!=frame)*frame=*chosen;have=1;}
 free(scratch);return have;
}
static int dual_semantic_read_frame(const char*dir,obj_frame*frame,uint64_t now,uint64_t epoch,int track,const char*category){return dual_semantic_read_attributes(dir,frame,now,epoch,track,category,NULL,NULL);}
/* 当前雷达观测可跨过一次缺失相机导出，但不能跨越不同任务准备、传感器上下文或选中身份。
 */
static int dual_export_for_task(const object_dual_task*t,const lab_real_runtime_view*v,const obj_frame*current,const obj_frame*next,const obj_scene*s,uint64_t prepared,uint64_t published){
 if(prepared!=published)return 0;
 if(t->radar_only)return !t->prepared||(s->epoch==t->epoch&&s->track_id==t->track_id&&lab_real_context_equal(&s->context,&v->context));
 /* 较新的跟踪发布可能只含预测，不能按发布时间顺序丢弃较早但仍有效的真实检测帧。
 */
 if(t->semantic_direct||t->static_search){uint64_t epoch=t->static_search&&!t->search_bound?0:t->epoch;int track=t->static_search&&!t->search_bound?0:t->track_id;const char*category=t->static_search?t->search_category:NULL;if((!current->epoch||current->epoch==next->epoch)&&semantic_observation_matches_attributes(next,v->time_us,epoch,track,category,t->static_search&&!t->search_bound?t->search_color:NULL,t->static_search&&!t->search_bound?t->search_size:NULL))return 1;}
 if(dual_export_is_current(current,next))return 1;
 const object_target_observation_v2*m=&s->measurement_v2;
 return m->valid&&(m->kind==OBJECT_MEASURE_RADAR_VISUAL||m->kind==OBJECT_MEASURE_RADAR_STEREO||m->kind==OBJECT_MEASURE_RADAR_ONLY)&&m->scan_us&&m->source_us&&m->source_us<=v->time_us&&v->time_us-m->source_us<=OBJECT_CONTROL_MEASUREMENT_US&&s->epoch==t->epoch&&s->track_id==t->track_id&&m->epoch==t->epoch&&m->track_id==t->track_id&&lab_real_context_equal(&s->context,&v->context)&&lab_real_context_equal(&m->context,&v->context);
}
static lab_status dual_sample_observer(void*u,const lab_live_sample*e){panel*p=u;r8_deskew_observe(&p->dual_history,e);return LAB_OK;}
static int dual_history_pose(const panel*p,uint64_t time,lab_pose*out){uint64_t error=0;return p->dual_history.have_context&&lab_real_context_equal(&p->dual_history.context,&p->session.view.context)&&r8_deskew_at(&p->dual_history,time,out,&error)==R8_DESKEW_APPLIED&&error<=500000;}
static int dual_exit_yaw(void*u,uint64_t time,double*yaw){lab_pose pose={0};if(!dual_history_pose((const panel*)u,time,&pose))return 0;*yaw=pose.yaw_rad;return 1;}
/* 绘图使用静止控制目标的准确副本，融合缓存独立；绘制记忆目标不产生新样本。
 */
static void dual_display_target(panel*p,obj_scene*s,uint64_t now){
 const object_dual_task*t=&p->dual_task;const lab_real_runtime_view*v=&p->session.view;
 if(!t->prepared||!t->have_target||!lab_real_context_equal(&t->context,&v->context))return;
 s->target_odom_valid=1;s->target_odom_x=t->target_x;s->target_odom_y=t->target_y;s->source_us=t->last_source_us;
 s->historical=!t->last_source_us||now<t->last_source_us||now-t->last_source_us>800000;s->standoff_m=t->standoff_m;
 snprintf(s->source,sizeof(s->source),"%s",t->source);s->confirmed=t->arrived;s->target_valid=s->dock_valid=0;s->path_count=0;
 snprintf(s->reason,sizeof(s->reason),"%s",t->reason);snprintf(s->message_zh,sizeof(s->message_zh),"%s",t->message_zh);
 const lab_localization_view*l=&v->localization;
 if(!v->has_localization||!l->has_pose||!l->map.map_id||!lab_real_context_equal(&l->context,&v->context))return;
 double c=cos(l->map_odom_yaw_rad),sn=sin(l->map_odom_yaw_rad);
 s->target_x=l->map_odom_x_m+c*t->target_x-sn*t->target_y;s->target_y=l->map_odom_y_m+sn*t->target_x+c*t->target_y;s->target_valid=1;
 double rx=l->map_odom_x_m+c*v->raw_odometry.x_m-sn*v->raw_odometry.y_m,ry=l->map_odom_y_m+sn*v->raw_odometry.x_m+c*v->raw_odometry.y_m;
 double dx=s->target_x-rx,dy=s->target_y-ry,d=hypot(dx,dy),offset=p->session.options.passage.length_m*.5+t->standoff_m;s->range_m=d;
 if(d>offset+.01){s->dock_x=s->target_x-offset*dx/d;s->dock_y=s->target_y-offset*dy/d;s->dock_yaw=atan2(dy,dx);s->dock_valid=1;}
 else{s->dock_x=rx;s->dock_y=ry;s->dock_yaw=l->map_odom_yaw_rad+v->raw_odometry.yaw_rad;}
 if(t->active&&v->navigation.has_object_target&&!strcmp(v->navigation.local_path_frame,"ODOM")){double gx=v->navigation.actual_goal_x,gy=v->navigation.actual_goal_y;if(isfinite(gx)&&isfinite(gy)){s->dock_x=l->map_odom_x_m+c*gx-sn*gy;s->dock_y=l->map_odom_y_m+sn*gx+c*gy;s->dock_yaw=atan2(s->target_y-s->dock_y,s->target_x-s->dock_x);s->dock_valid=1;}}
}
static void dual_scene_worker(panel*j,uint64_t now){const char*dir=getenv("ACG_PHONE_DIR");if(!dir)return;char path[640];if(snprintf(path,sizeof(path),"%s/objects-state.json",dir)>=(int)sizeof(path))return;
 obj_frame frame={0};const object_dual_task*input_task=&j->dual_task;
 int semantic=input_task->semantic_direct||input_task->static_search;
 uint64_t wanted_epoch=input_task->static_search&&!input_task->search_bound?0:input_task->epoch;
 int wanted_track=input_task->static_search&&!input_task->search_bound?0:input_task->track_id;
 int have=semantic?dual_semantic_read_attributes(dir,&frame,now,wanted_epoch,wanted_track,input_task->static_search?input_task->search_category:NULL,input_task->static_search&&!input_task->search_bound?input_task->search_color:NULL,input_task->static_search&&!input_task->search_bound?input_task->search_size:NULL):obj_frame_load(path,&frame);
 uint64_t header_time=frame.frame_us,header_id=frame.frame_id;
 /* 语义任务已绑定该真实检测编号，感知选择回执独立，使用私有副本关联距离。
 */
 if(have)dual_semantic_select_frame(&j->dual_task,&frame);
 obj_image_source_v4 image_source=obj_selected_image_source_v4(have?&frame:NULL);if(image_source.valid){frame.frame_us=image_source.time_us;frame.frame_id=image_source.frame_id;}
 approach_observation o={0};o.context=j->session.view.context;o.context_valid=1;
 if(have){o.epoch=frame.epoch;o.track_id=frame.selected;o.frame_us=frame.frame_us;o.frame_id=frame.frame_id;lab_pose pose={0};if(dual_history_pose(j,frame.frame_us,&pose)){o.frame_pose_valid=1;o.frame_pose_x=pose.x_m;o.frame_pose_y=pose.y_m;o.frame_pose_yaw=pose.yaw_rad;}
  for(int i=0;i<frame.count;i++)if(frame.objects[i].track_id==frame.selected){const obj_detection*d=&frame.objects[i];if(d->depth_time_us&&now>=d->depth_time_us&&now-d->depth_time_us<=2000000&&dual_history_pose(j,d->depth_time_us,&pose)){o.stereo_pose_valid=1;o.stereo_us=d->depth_time_us;o.stereo_frame_id=d->depth_frame_id;o.stereo_pose_x=pose.x_m;o.stereo_pose_y=pose.y_m;o.stereo_pose_yaw=pose.yaw_rad;o.stereo_bearing_valid=d->depth_bearing_valid;o.stereo_bearing_rad=d->depth_bearing;}}
 }
 o.scan_us=j->session.view.scan_time_us;lab_pose actual_scan_pose={0};uint64_t scan_pose_error=UINT64_MAX;
 if(j->dual_history.have_context&&lab_real_context_equal(&j->dual_history.context,&j->session.view.context)&&r8_deskew_at(&j->dual_history,o.scan_us,&actual_scan_pose,&scan_pose_error)==R8_DESKEW_APPLIED&&scan_pose_error<=OBJECT_CONTROL_ALIGNMENT_US){o.scan_pose_valid=1;o.scan_pose_x=actual_scan_pose.x_m;o.scan_pose_y=actual_scan_pose.y_m;o.scan_pose_yaw=actual_scan_pose.yaw_rad;}
 obj_scene_config cfg;obj_scene_config_default(&cfg);const char*mount_path=getenv("ACG_OBJECTS_CONFIG");if(mount_path&&!obj_scene_config_load_mount(&cfg,mount_path))fprintf(stderr,"OBJECT_MOUNT_CONFIG_INVALID path=%s fallback=APPROXIMATE_DEFAULT\n",mount_path);cfg.standoff_m=j->dual_task.prepared?j->dual_task.standoff_m:.20;cfg.mode=j->dual_task.mode==OBJECT_MODE_FOLLOW?OBJECT_MODE_FOLLOW:OBJECT_MODE_STATIC;
 int radar_wanted=j->dual_task.prepared&&j->dual_task.mode==OBJECT_MODE_FOLLOW;
 obj_radar_follow_hint hint={0};const object_dual_task*t=&j->dual_task;hint.independent=t->radar_only;
 if(t->have_context&&lab_real_context_equal(&t->context,&j->session.view.context)){
  hint.position_valid=t->have_target&&t->last_source_us;hint.odom_x=t->target_x;hint.odom_y=t->target_y;hint.position_source_us=t->last_source_us;hint.error=t->observation_v2.valid?t->observation_v2.error_m:.15;
  hint.bearing_valid=t->direction_initialized&&t->direction_source_us;hint.world_yaw=t->direction_world_yaw;hint.bearing_source_us=t->direction_source_us;
 }
 if(o.scan_pose_valid){hint.scan_pose_valid=1;hint.scan_pose_time_us=o.scan_us;hint.scan_pose_x=actual_scan_pose.x_m;hint.scan_pose_y=actual_scan_pose.y_m;hint.scan_pose_yaw=actual_scan_pose.yaw_rad;hint.scan_pose_error_m=actual_scan_pose.uncertainty_m;hint.scan_pose_error_us=scan_pose_error;}
 int radar_current=radar_wanted?obj_radar_follow_update(&j->radar_follow,&j->dual_scene,&cfg,&j->session.view,&j->session.options.passage,now,t->epoch,t->track_id,t->binding_generation,&hint):0;
 if(j->dual_task.radar_only&&!radar_current){
  /* 命名空间属于任务元数据，不作为构造的雷达回波或相机观测。 */
  j->dual_scene.context=j->session.view.context;j->dual_scene.context_valid=1;j->dual_scene.epoch=t->epoch;j->dual_scene.track_id=t->track_id;j->dual_scene.binding_generation=t->binding_generation;j->dual_scene.measurement_v2.valid=0;j->dual_scene.historical=1;
 }
 if(!radar_current&&!j->dual_task.radar_only)obj_scene_fuse(&j->dual_scene,&cfg,have?&frame:NULL,&o,NULL,&j->session.view,&j->session.options.passage,now);
 if(!radar_wanted&&j->dual_task.radar_only){j->dual_scene.measurement_v2.valid=0;snprintf(j->dual_scene.reason,sizeof(j->dual_scene.reason),"RADAR_TASK_NOT_ACTIVE");snprintf(j->dual_scene.message_zh,sizeof(j->dual_scene.message_zh),"雷达任务未启用，不自行前往障碍物");}
 obj_scene_cloud(&j->dual_scene,&j->session.view,&j->session.options.passage,now);frame.frame_us=header_time;frame.frame_id=header_id;j->dual_frame=have?frame:(obj_frame){0};
 obj_scene previous=j->dual_display_scene;j->dual_display_scene=j->dual_scene;dual_display_target(j,&j->dual_display_scene,now);
 if(j->dual_task.active){
  obj_scene*s=&j->dual_display_scene;const lab_real_runtime_view*v=&j->session.view;const lab_real_navigation_view*n=&v->navigation;const lab_localization_view*l=&v->localization;
  s->path_count=0;snprintf(s->preview_kind,sizeof(s->preview_kind),"NONE");snprintf(s->preview_source_frame,sizeof(s->preview_source_frame),"ODOM");
  int map=v->has_localization&&l->has_pose&&l->map.map_id&&lab_real_context_equal(&l->context,&v->context);
  if(map&&!strcmp(n->local_path_frame,"ODOM")){
   const lab_real_xy*path=n->path;unsigned count=n->path_count;const char*kind="EXECUTING_REFERENCE_ROUTE";
   if(!path||!count){path=n->local_path;count=n->local_path_count;kind="VALIDATED_ACTION";}
   if(path&&count<=OBJ_SCENE_PATH_MAX){double c=cos(l->map_odom_yaw_rad),sn=sin(l->map_odom_yaw_rad);for(unsigned i=0;i<count;i++){if(!isfinite(path[i].x_m)||!isfinite(path[i].y_m)){s->path_count=0;break;}s->path[i][0]=l->map_odom_x_m+c*path[i].x_m-sn*path[i].y_m;s->path[i][1]=l->map_odom_y_m+sn*path[i].x_m+c*path[i].y_m;s->path_count++;}if(s->path_count)snprintf(s->preview_kind,sizeof(s->preview_kind),"%s",kind);}
  }
  s->plan_us=n->local_path_source_us;snprintf(s->preview_status,sizeof(s->preview_status),"%s",s->path_count?((fabs(j->dual_task.requested_v)<1e-9&&fabs(j->dual_task.requested_w)<1e-9)||!v->enabled?"HELD":"ACTIVE"):"WAIT");snprintf(s->preview_reason,sizeof(s->preview_reason),"%s",!map?"EXECUTING_ODOM_MAP_TRANSFORM_UNAVAILABLE":n->reason);
 }
 else if(!j->dual_last_plan||now-j->dual_last_plan>=500000){obj_scene_plan(&j->dual_display_scene,&j->session.view,&j->session.options.passage,now);j->dual_last_plan=now;}
 else if(previous.map_id==j->dual_display_scene.map_id&&previous.track_id==j->dual_display_scene.track_id&&previous.dock_valid&&j->dual_display_scene.dock_valid&&hypot(previous.dock_x-j->dual_display_scene.dock_x,previous.dock_y-j->dual_display_scene.dock_y)<.05){
  j->dual_display_scene.path_count=previous.path_count;memcpy(j->dual_display_scene.path,previous.path,sizeof(previous.path));j->dual_display_scene.plan_us=previous.plan_us;
  snprintf(j->dual_display_scene.preview_status,sizeof(j->dual_display_scene.preview_status),"%s",previous.preview_status);snprintf(j->dual_display_scene.preview_reason,sizeof(j->dual_display_scene.preview_reason),"%s",previous.preview_reason);
 }else{j->dual_display_scene.path_count=0;snprintf(j->dual_display_scene.preview_status,sizeof(j->dual_display_scene.preview_status),"WAIT");snprintf(j->dual_display_scene.preview_reason,sizeof(j->dual_display_scene.preview_reason),"TARGET_UPDATED_WAIT_NEXT_PREVIEW");}
 char*json=malloc(OBJ_SCENE_JSON_MAX);size_t n;
 if(json&&obj_scene_json(&j->dual_display_scene,json,OBJ_SCENE_JSON_MAX,&n)){
  char body[768];if(object_body_target_v9_json(&j->dual_scene,&j->dual_task,&j->session.view,now,body,sizeof(body))){int z=snprintf(json+n-1,OBJ_SCENE_JSON_MAX-n+1,",%s}",body);if(z>0&&(size_t)z<OBJ_SCENE_JSON_MAX-n+1)n=n-1+(size_t)z;}
  if(snprintf(path,sizeof(path),"%s/objects-scene.json",dir)<(int)sizeof(path))obj_atomic_write(path,json,n);
 }free(json);
}
static int dual_intent(void*u,const lab_real_runtime_view*v,uint64_t now,lab_real_command*out){panel*p=u;
 if(!p->dual_task.static_search&&p->dual_task.mode!=OBJECT_MODE_FOLLOW&&p->dual_task.active&&p->dual_frame.epoch&&p->dual_frame.selected!=p->dual_task.track_id)object_dual_task_cancel(&p->dual_task,"TARGET_SELECTION_CHANGED");
 /* 图像射线保留原时间，仅将控制副本由采集时车体航向旋转到当前车体；融合与显示仍独立使用原采集位姿。

 */
 obj_scene_config mount;obj_scene_config_default(&mount);const char*mount_path=getenv("ACG_OBJECTS_CONFIG");if(mount_path)obj_scene_config_load_mount(&mount,mount_path);
 obj_frame control=p->dual_frame;
 if(p->dual_task.semantic_direct||p->dual_task.static_search){const char*dir=getenv("ACG_PHONE_DIR");uint64_t epoch=p->dual_task.static_search&&!p->dual_task.search_bound?0:p->dual_task.epoch;int track=p->dual_task.static_search&&!p->dual_task.search_bound?0:p->dual_task.track_id;dual_semantic_read_attributes(dir,&control,now,epoch,track,p->dual_task.static_search?p->dual_task.search_category:NULL,p->dual_task.static_search&&!p->dual_task.search_bound?p->dual_task.search_color:NULL,p->dual_task.static_search&&!p->dual_task.search_bound?p->dual_task.search_size:NULL);}
 dual_semantic_select_frame(&p->dual_task,&control);
 obj_image_source_v4 image_source=obj_selected_image_source_v4(&control);uint64_t image_time=image_source.valid?image_source.time_us:control.frame_us;lab_pose capture={0};int have=dual_history_pose(p,image_time,&capture);
 obj_direction_time_v4 timing=obj_direction_time_v4_resolve(image_time,now,have,capture.yaw_rad,v->raw_odometry.yaw_rad,p->dual_measured_w_valid,p->dual_yaw_us,p->dual_measured_w);
 p->dual_camera_compensation=timing.kind;p->dual_camera_error=timing.error_rad;p->dual_camera_source_us=image_time;snprintf(p->dual_camera_reason,sizeof(p->dual_camera_reason),"%s",timing.reason);
 for(int i=0;i<control.count;i++){obj_detection*d=&control.objects[i];if(d->track_id!=control.selected&&!(p->dual_task.static_search&&!p->dual_task.search_bound))continue;
  obj_exit_compensate_v5(&d->follow_exit,&mount,now,v->raw_odometry.yaw_rad,dual_exit_yaw,p);
  if(d->bearing_valid){if(d->ray_direction_valid&&d->geometry_eye==0){double vector[3];obj_scene_camera_vector(&mount,d->ray_direction,vector);if(hypot(vector[0],vector[1])>1e-6)d->bearing=atan2(vector[1],vector[0]);else d->bearing_valid=0;}else if(mount.mount_configured)d->bearing=remainder(d->bearing+mount.camera.yaw,6.283185307179586);}
  if(timing.valid){if(d->bearing_valid)d->bearing=remainder(d->bearing+timing.delta_rad,6.283185307179586);}else if(!p->dual_task.static_search)d->bearing_valid=0;
  if(d->reacquire_candidate_bearing_valid){int found=dual_history_pose(p,d->reacquire_candidate_time_us,&capture);obj_direction_time_v4 candidate_timing=obj_direction_time_v4_resolve(d->reacquire_candidate_time_us,now,found,capture.yaw_rad,v->raw_odometry.yaw_rad,p->dual_measured_w_valid,p->dual_yaw_us,p->dual_measured_w);if(candidate_timing.valid)d->reacquire_candidate_bearing_rad=remainder(d->reacquire_candidate_bearing_rad+candidate_timing.delta_rad,6.283185307179586);else d->reacquire_candidate_bearing_valid=0;}
 }
 p->dual_task.front_m=p->session.options.passage.length_m*.5;
 int active=object_dual_task_tick(&p->dual_task,&p->dual_scene,&control,v,now,out);
 if(active&&p->dual_task.mode==OBJECT_MODE_FOLLOW&&!p->dual_task.have_target&&!strcmp(p->dual_task.reason,"FOLLOW_WAIT_DISTANCE_MEASUREMENT")&&!strncmp(p->dual_scene.follow_reason,"RADAR_",6)){
  snprintf(p->dual_task.reason,sizeof(p->dual_task.reason),"%s",p->dual_scene.follow_reason);snprintf(p->dual_task.message_zh,sizeof(p->dual_task.message_zh),"%s",p->dual_scene.message_zh);
 }
 object_route_target_v2 route={0};object_dual_task_route_v2(&p->dual_task,v,now,&route);
 if(p->dual_task.mode==OBJECT_MODE_STATIC&&(p->dual_task.near_hold||p->dual_task.arrived)){
  out->linear_m_s=out->angular_rad_s=0;p->dual_task.requested_v=p->dual_task.requested_w=0;
  route.zero_required=1;route.goal_valid=0;route.trail_count=0;route.static_search_left=route.static_bearing_approach=0;
 }

 if(p->session.runtime&&(active||v->object_mode)){lab_status status=lab_real_runtime_object_target_v2(p->session.runtime,&route);if(status!=LAB_OK){out->linear_m_s=out->angular_rad_s=0;p->dual_task.requested_v=p->dual_task.requested_w=0;snprintf(p->dual_task.reason,sizeof(p->dual_task.reason),"TARGET_HANDOFF_REJECTED_%d",status);snprintf(p->dual_task.message_zh,sizeof(p->dual_task.message_zh),"目标控制交接被拒绝：状态%d；等待当前任务、里程计会话和导航模式一致后恢复",status);}}
 return active;
}
/* 配置的光学到车体RzRyRx安装变换的逆变换。方位为正始终表示光学左侧，与显示眼无关。
 */
static double dual_optical_bearing(const obj_scene_config*c,double bx,double by){
 double yaw=c->mount_configured?c->camera.yaw:0,pitch=c->mount_configured?c->camera.pitch:0,roll=c->mount_configured?c->camera.roll:0;
 double x=cos(yaw)*bx+sin(yaw)*by,y=-sin(yaw)*bx+cos(yaw)*by,z=0;
 double xx=cos(pitch)*x-sin(pitch)*z,zz=sin(pitch)*x+cos(pitch)*z;
 double yy=cos(roll)*y+sin(roll)*zz;return atan2(yy,xx);
}
static size_t dual_memory_candidates(panel*p,const char*dir,const obj_scene_config*cfg,uint64_t now,obj_scene_candidate*out){
 static obj_scene_candidate cache[16];static size_t count;static uint64_t last,epoch;static int selected=-1;static uint32_t generation;static lab_real_context context;
 const object_dual_task*t=&p->dual_task;const lab_real_runtime_view*v=&p->session.view;
 if(memcmp(&context,&v->context,sizeof(context))||epoch!=t->epoch||selected!=t->track_id||generation!=t->binding_generation){count=0;last=0;context=v->context;epoch=t->epoch;selected=t->track_id;generation=t->binding_generation;}
 if(!t->prepared||t->mode!=OBJECT_MODE_STATIC)return 0;
 if(count&&(!cache[0].image_time_us||cache[0].image_time_us>now||now-cache[0].image_time_us>2000000))count=0;
 if(!last||now<last||now-last>=250000){
  last=now;char path[640];obj_frame*f=malloc(sizeof(*f));
  if(f&&snprintf(path,sizeof(path),"%s/objects-detections.json",dir)<(int)sizeof(path)&&obj_frame_load(path,f)&&f->epoch==t->epoch){
   approach_observation o={0};o.context=v->context;o.context_valid=1;o.epoch=f->epoch;o.track_id=f->selected;o.frame_us=f->frame_us;o.frame_id=f->frame_id;o.scan_us=v->scan_time_us;lab_pose q={0};
   if(dual_history_pose(p,f->frame_us,&q)){o.frame_pose_valid=1;o.frame_pose_x=q.x_m;o.frame_pose_y=q.y_m;o.frame_pose_yaw=q.yaw_rad;}
   if(dual_history_pose(p,o.scan_us,&q)){o.scan_pose_valid=1;o.scan_pose_x=q.x_m;o.scan_pose_y=q.y_m;o.scan_pose_yaw=q.yaw_rad;}
   count=obj_scene_candidates(cfg,f,&o,v,&p->session.options.passage,now,cache,16);
  }else count=0;free(f);
 }
 if(count)memcpy(out,cache,count*sizeof(*out));return count;
}
static void dual_memory_export(panel*p,uint64_t now){
 if(p->dual_last_export&&now-p->dual_last_export<100000)return;p->dual_last_export=now;
 const char*dir=getenv("ACG_PHONE_DIR");if(!dir)return;const object_dual_task*t=&p->dual_task;const lab_real_runtime_view*v=&p->session.view;const obj_scene*scene=&p->dual_scene;
 int locked=t->prepared&&t->have_target&&lab_real_context_equal(&t->context,&v->context);
 int task_identity=t->prepared&&t->epoch&&t->track_id>0;
 uint64_t epoch=task_identity?t->epoch:p->dual_frame.epoch;int id=task_identity?t->track_id:p->dual_frame.selected;
 int scene_current=scene->epoch==epoch&&scene->track_id==id&&lab_real_context_equal(&scene->context,&v->context)&&(!task_identity||(scene->binding_generation==t->binding_generation&&scene->association_revision==t->association_revision));
 uint64_t source=locked?t->last_source_us:scene_current?scene->source_us:0;
 double tx=locked?t->target_x:scene->target_odom_x,ty=locked?t->target_y:scene->target_odom_y;
 double age=source&&source<=now?(now-source)/1e6:0;
 if(locked&&t->mode==OBJECT_MODE_FOLLOW&&age<=1.){tx+=t->target_vx*age;ty+=t->target_vy*age;}
 int valid=(locked||(scene_current&&scene->target_odom_valid))&&source&&epoch&&id>0&&v->has_odometry&&v->odometry_time_us&&v->odometry_time_us<=now&&now-v->odometry_time_us<=1200000&&isfinite(tx)&&isfinite(ty);
 if(locked&&t->mode==OBJECT_MODE_FOLLOW&&age>5.)valid=0;
 double c=cos(v->raw_odometry.yaw_rad),sn=sin(v->raw_odometry.yaw_rad),dx=tx-v->raw_odometry.x_m,dy=ty-v->raw_odometry.y_m;
 obj_scene_config cfg;obj_scene_config_default(&cfg);const char*mp=getenv("ACG_OBJECTS_CONFIG");if(mp)obj_scene_config_load_mount(&cfg,mp);double origin[3];obj_scene_camera_origin(&cfg,&p->session.options.passage,origin);
 double bx=c*dx+sn*dy-origin[0],by=-sn*dx+c*dy-origin[1],range=hypot(bx,by),bearing=dual_optical_bearing(&cfg,bx,by);
 double width=fmin(1.0,fmax(.18,atan2(fmax(.12,scene->error_m)+fmin(.6,age*.08),fmax(.10,range))+.12));
 int lidar=valid&&scene_current&&scene->epoch==epoch&&scene->track_id==id&&scene->lidar_valid&&scene->lidar_source_us&&scene->lidar_source_us<=now&&now-scene->lidar_source_us<=800000;
 double lx=scene->lidar_x-v->raw_odometry.x_m,ly=scene->lidar_y-v->raw_odometry.y_m;
 double lb=dual_optical_bearing(&cfg,c*lx+sn*ly-origin[0],-sn*lx+c*ly-origin[1]);
 char path[640],json[16384];int n=snprintf(json,sizeof(json),"{\"version\":1,\"epoch\":\"%llu\",\"track_id\":%d,\"mode\":\"%s\",\"published_us\":\"%llu\",\"source_us\":\"%llu\",\"target_source_us\":\"%llu\",\"context\":{\"source\":%u,\"clock\":%u,\"timeline_id\":%u,\"origin_epoch\":%u},\"valid\":%s,\"center_bearing_rad\":%.9g,\"half_width_rad\":%.9g,\"lidar_support_valid\":%s,\"lidar_source_us\":\"%llu\",\"lidar_bearing_rad\":%.9g,\"lidar_half_width_rad\":%.9g}",(unsigned long long)epoch,id,t->prepared&&t->mode==OBJECT_MODE_FOLLOW?"follow":"static",(unsigned long long)now,(unsigned long long)v->odometry_time_us,(unsigned long long)source,v->context.source,v->context.clock,v->context.timeline_id,v->context.origin_epoch,valid?"true":"false",isfinite(bearing)?bearing:0,width,lidar?"true":"false",(unsigned long long)scene->lidar_source_us,isfinite(lb)?lb:0,fmin(.7,width));
 if(n>0&&(size_t)n<sizeof(json)){
  int task_class=t->task_class_id;for(int i=0;task_class<0&&i<p->dual_frame.count;i++)if(p->dual_frame.objects[i].track_id==id)task_class=p->dual_frame.objects[i].task_class_id;
  obj_scene_candidate candidates[16];size_t count=dual_memory_candidates(p,dir,&cfg,now,candidates);
  obj_memory_extra_v4 extra={valid,task_class,t->binding_generation,v->odometry_time_us,tx,ty,range,fmax(.12,scene->error_m),candidates,count};
  size_t length=obj_memory_v4_append(json,sizeof(json),(size_t)n,&extra);
  if(length&&snprintf(path,sizeof(path),"%s/objects-target-memory.json",dir)<(int)sizeof(path))obj_atomic_write(path,json,length);
 }
}
static void dual_reason_safe(char*dest,size_t cap,const char*source){size_t n=0;for(;source&&*source&&n+1<cap;source++){unsigned char c=(unsigned char)*source;dest[n++]=(c<32||c=='"'||c=='\\')?'_':(char)c;}dest[n]=0;}
static void dual_number(char*out,size_t cap,int valid,double value){if(valid&&isfinite(value))snprintf(out,cap,"%.9g",value);else snprintf(out,cap,"null");}
static void dual_publish_task(panel*p,uint64_t now){const char*dir=getenv("ACG_PHONE_DIR");if(!dir)return;char path[640],json[OBJECT_TASK_JSON_MAX];size_t n;
 dual_memory_export(p,now);const lab_real_runtime_view*v=&p->session.view;
 if(v->has_odometry&&v->odometry_time_us&&v->odometry_time_us!=p->dual_yaw_us){uint64_t stamp=v->odometry_time_us;double dt=stamp>p->dual_yaw_us?(stamp-p->dual_yaw_us)/1e6:0;
  p->dual_measured_w_valid=p->dual_yaw_us&&dt>=.005&&dt<=.5;if(p->dual_measured_w_valid){p->dual_measured_w=remainder(v->raw_odometry.yaw_rad-p->dual_last_yaw,6.283185307179586)/dt;if(!isfinite(p->dual_measured_w)||fabs(p->dual_measured_w)>20)p->dual_measured_w_valid=0;}p->dual_yaw_us=stamp;p->dual_last_yaw=v->raw_odometry.yaw_rad;
 }
 if(!object_dual_task_json(&p->dual_task,now,json,sizeof(json),&n))return;
 int fresh=v->feedback.backend==LAB_CHASSIS_HARDWARE&&(v->feedback.capabilities&LAB_CHASSIS_FEEDBACK)&&v->feedback.time_us&&v->feedback.time_us<=now&&now-v->feedback.time_us<=(v->object_mode?OBJECT_CONTROL_FEEDBACK_US:250000);char wheel[160]="null",measured[40]="null",depth_age[40]="null",candidate_reason[96]="NONE";int candidate=-1;
 if(fresh)snprintf(wheel,sizeof(wheel),"[%.3f,%.3f,%.3f,%.3f]",v->feedback.actual_rpm[0],v->feedback.actual_rpm[1],v->feedback.actual_rpm[2],v->feedback.actual_rpm[3]);
 if(p->dual_measured_w_valid&&p->dual_yaw_us<=now&&now-p->dual_yaw_us<=500000)snprintf(measured,sizeof(measured),"%.6f",p->dual_measured_w);
 for(int i=0;i<p->dual_frame.count;i++)if(p->dual_frame.objects[i].track_id==p->dual_frame.selected){const obj_detection*d=&p->dual_frame.objects[i];if(d->depth_time_us&&d->depth_time_us<=now)snprintf(depth_age,sizeof(depth_age),"%llu",(unsigned long long)((now-d->depth_time_us)/1000));candidate=d->reacquire_candidate_track_id>0?d->reacquire_candidate_track_id:-1;dual_reason_safe(candidate_reason,sizeof(candidate_reason),d->reacquire_candidate_reason[0]?d->reacquire_candidate_reason:d->reason);}
 int owner=p->lease.claimed&&p->lease.motion_active&&p->lease.last_seen_us<=now&&now-p->lease.last_seen_us<CONT_PHONE_US;
 int hard=v->manual_revoked||strstr(v->reason,"CLOCK")||strstr(v->reason,"CONTEXT")||strstr(v->reason,"TRANSPORT");
 int restart_required=p->dual_task.active&&v->resume_requires_start&&!v->enabled&&!v->pending_enable;
 int can_restart=restart_required&&p->session.options.enable_drive&&owner&&!hard&&fresh&&!v->feedback.faults&&p->session.runtime&&lab_real_runtime_start_ready(p->session.runtime,now)==LAB_OK;
 const char*restart_reason=!p->session.options.enable_drive?"PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE":hard?"EXPLICIT_RESTART_BLOCKED_BY_CLOCK_CONTEXT_OR_TRANSPORT":!owner?"WAIT_CURRENT_PHONE_OWNER":v->feedback.faults?"WAIT_FEEDBACK_FAULT_CLEAR":restart_required?"FINAL_FAULT_REQUIRES_EXPLICIT_RESTART":p->dual_task.active&&!v->enabled?"WAIT_AUTOMATIC_SOFT_RECOVERY":"NO_RESTART_REQUIRED";
 const lab_real_command*actual=&v->protected_output.command;int applied=v->enabled&&lab_real_command_valid(actual,now);double pv=applied?actual->linear_m_s:0,pw=applied?actual->angular_rad_s:0;
 int z=snprintf(json+n-1,sizeof(json)-n+1,",\"drive_capable\":%s,\"runtime_reason\":\"%s\",\"navigation_reason\":\"%s\",\"enabled\":%s,\"pending_enable\":%s,\"feedback_faults\":%u,\"feedback_fresh\":%s,\"feedback_age_ms\":%llu,\"protected_v\":%.5f,\"protected_w\":%.5f,\"measured_w_rad_s\":%s,\"measured_w_source\":\"RAW_ODOMETRY_YAW_DIFFERENCE\",\"candidate_track_id\":%d,\"candidate_reason\":\"%s\",\"depth_age_ms\":%s,\"actual_wheel_rpm\":%s,\"resume_requires_start\":%s,\"can_restart\":%s,\"restart_required\":%s,\"restart_reason\":\"%s\",\"task_request\":{\"id\":\"%llu\",\"state\":\"%s\",\"reason\":\"%s\"}}",p->session.options.enable_drive?"true":"false",v->reason,v->navigation.reason,v->enabled?"true":"false",v->pending_enable?"true":"false",v->feedback.faults,fresh?"true":"false",(unsigned long long)(v->feedback.time_us&&now>=v->feedback.time_us?(now-v->feedback.time_us)/1000:UINT64_MAX),pv,pw,measured,candidate,candidate_reason,depth_age,wheel,v->resume_requires_start?"true":"false",can_restart?"true":"false",restart_required?"true":"false",restart_reason,(unsigned long long)p->request_id,p->request_state,p->request_reason);
 if(z>0&&(size_t)z<sizeof(json)-n+1){size_t length=n-1+(size_t)z;
  const object_dual_task*t=&p->dual_task;const object_target_observation_v2*m=&t->observation_v2;
  int obs=m->valid&&m->source_us&&m->source_us<=now&&now-m->source_us<=800000&&m->epoch==t->epoch&&m->track_id==t->track_id&&m->binding_generation==t->binding_generation&&m->association_revision==t->association_revision&&lab_real_context_equal(&m->context,&v->context);
  int estimate=t->have_target&&lab_real_context_equal(&t->context,&v->context);char gap[40],error[40],x[40],y[40],egap[40],age[40];
  dual_number(gap,sizeof(gap),obs,m->surface_gap_m);dual_number(error,sizeof(error),obs,m->error_m);double prediction=t->mode==OBJECT_MODE_FOLLOW&&t->last_source_us&&t->last_source_us<=now&&now-t->last_source_us<=OBJECT_FOLLOW_PREDICT_US?(now-t->last_source_us)/1e6:0;dual_number(x,sizeof(x),estimate,t->target_x+t->target_vx*prediction);dual_number(y,sizeof(y),estimate,t->target_y+t->target_vy*prediction);dual_number(egap,sizeof(egap),estimate,t->range_m);
  if(estimate&&t->last_source_us&&t->last_source_us<=now)snprintf(age,sizeof(age),"%llu",(unsigned long long)((now-t->last_source_us)/1000));else snprintf(age,sizeof(age),"null");
  const char*kind=!estimate?"NONE":prediction>0&&(fabs(t->target_vx)+fabs(t->target_vy)>1e-9)?"PREDICTED":t->last_source_us&&t->last_source_us<=now&&now-t->last_source_us<=800000?"MEASURED_FILTERED":"MEMORY";
  const char*layer=!t->active?"TASK":!p->session.options.enable_drive?"PREVIEW_AUTH":!owner?"PHONE_OWNER":!fresh||v->feedback.faults?"CHASSIS_FEEDBACK":!v->enabled?"RUNTIME":t->requested_v==0&&t->requested_w==0?"TARGET_TASK":fabs(pv-t->requested_v)>.001||fabs(pw-t->requested_w)>.001?(v->object_turn_reason[0]&&!strcmp(v->navigation.reason,"NONE")?"OBJECT_TURN_SMOOTH":"LOCAL_GEOMETRY_OR_COMMAND_GUARD"):"NONE";
  const char*recovery=!strcmp(layer,"OBJECT_TURN_SMOOTH")?(v->object_turn_waiting?"转向正在减速换向，航向减稳后自动继续；直行速度不变":"实物转向正在平滑执行，避免突然猛转；原FPGA起步助力仍保留"):!strcmp(layer,"PREVIEW_AUTH")?"以 sh begin drive 启动后重新明确开始任务":!strcmp(layer,"PHONE_OWNER")?"保持手机连接并确认当前控制权":!strcmp(layer,"CHASSIS_FEEDBACK")?"等待当前轮速反馈和底盘故障解除":!strcmp(layer,"RUNTIME")?"查看 runtime_reason 与 restart_reason；满足恢复条件后再运行":!strcmp(layer,"TARGET_TASK")?t->message_zh:!strcmp(layer,"TASK")?"双击选择目标开始；已到达则保持停车":"局部通路恢复后继续；查看 navigation_reason";
  char ms[48],mr[96],rr[96],nr[64];dual_reason_safe(ms,sizeof(ms),m->source);dual_reason_safe(mr,sizeof(mr),m->reason);dual_reason_safe(rr,sizeof(rr),v->reason);dual_reason_safe(nr,sizeof(nr),v->navigation.reason);
  z=snprintf(json+length-1,sizeof(json)-length+1,",\"target_observation\":{\"valid\":%s,\"source_us\":\"%llu\",\"image_us\":\"%llu\",\"scan_us\":\"%llu\",\"surface_gap_m\":%s,\"error_m\":%s,\"source\":\"%s\",\"reason\":\"%s\",\"mount_verified\":%s},\"target_estimate\":{\"x_m\":%s,\"y_m\":%s,\"gap_m\":%s,\"age_ms\":%s,\"kind\":\"%s\"},\"execution\":{\"requested_v\":%.6g,\"requested_w\":%.6g,\"actual_v\":%.6g,\"actual_w\":%.6g,\"reason\":\"%s\",\"navigation_reason\":\"%s\",\"route_frame\":\"%s\",\"route_kind\":\"%s\",\"blocked_layer\":\"%s\",\"recovery_condition_zh\":\"%s\"}}",obs?"true":"false",(unsigned long long)m->source_us,(unsigned long long)m->image_us,(unsigned long long)m->scan_us,gap,error,ms,mr,m->mount_verified?"true":"false",x,y,egap,age,kind,t->requested_v,t->requested_w,pv,pw,rr,nr,v->navigation.local_path_frame,v->navigation.path_count?"EXECUTING_REFERENCE_ROUTE":v->navigation.local_path_count?"VALIDATED_ACTION":"NONE",layer,recovery);
  if(z<0||(size_t)z>=sizeof(json)-length+1)return;length=length-1+(size_t)z;
z=snprintf(json+length-1,sizeof(json)-length+1,",\"image_timing\":{\"frame_id\":\"%llu\",\"source_us\":\"%llu\",\"compensation_kind\":%d,\"reason\":\"%s\",\"angle_error_rad\":%.6g}}",(unsigned long long)p->dual_frame.frame_id,(unsigned long long)p->dual_camera_source_us,p->dual_camera_compensation,p->dual_camera_reason,p->dual_camera_error);
 if(z<0||(size_t)z>=sizeof(json)-length+1)return;length=length-1+(size_t)z;
 char turn_reason[64];dual_reason_safe(turn_reason,sizeof turn_reason,v->object_turn_reason);
 z=snprintf(json+length-1,sizeof(json)-length+1,",\"turn_smoothing\":{\"enabled\":true,\"selected_w\":%.6g,\"output_w\":%.6g,\"waiting_reversal\":%s,\"reason\":\"%s\",\"fpga_start_assist_changed\":false}}",v->object_turn_selected_w,v->object_turn_applied_w,v->object_turn_waiting?"true":"false",turn_reason);
 if(z<0||(size_t)z>=sizeof(json)-length+1)return;length=length-1+(size_t)z;
 char radar_reason[96],radar_zh[256];dual_reason_safe(radar_reason,sizeof(radar_reason),p->dual_scene.follow_reason);dual_reason_safe(radar_zh,sizeof(radar_zh),p->dual_scene.message_zh);
 z=snprintf(json+length-1,sizeof(json)-length+1,",\"radar_search\":{\"reason\":\"%s\",\"message_zh\":\"%s\",\"source_us\":\"%llu\",\"support_points\":%u,\"sparse\":%s}}",radar_reason,radar_zh,(unsigned long long)p->dual_scene.lidar_source_us,p->dual_scene.follow_support_points,p->dual_scene.follow_sparse?"true":"false");
 if(z<0||(size_t)z>=sizeof(json)-length+1)return;length=length-1+(size_t)z;
 if(snprintf(path,sizeof(path),"%s/objects-task.json",dir)<(int)sizeof(path))obj_atomic_write(path,json,length);if(!p->last_log||now-p->last_log>=1000000)fprintf(stderr,"OBJECT_TASK %.*s\n",(int)length,json);}
}
