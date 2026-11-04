#define _POSIX_C_SOURCE 200809L
#include "object_scene.h"
#include "scan_core.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
static uint64_t scene_clock(void){
#ifdef _WIN32
 return (uint64_t)((double)clock()*1000000/CLOCKS_PER_SEC);
#else
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;
#endif
}
static int expired(void*u){return scene_clock()>=*(uint64_t*)u;}
void obj_scene_plan(obj_scene*s,const lab_real_runtime_view*v,const passage_config*c,uint64_t now){
 if(!s||!v||!c)return;
 s->path_count=0;snprintf(s->preview_kind,sizeof(s->preview_kind),"READONLY_PREVIEW");s->preview_source_frame[0]=0;snprintf(s->preview_status,sizeof(s->preview_status),"WAIT_TARGET");snprintf(s->preview_reason,sizeof(s->preview_reason),"NO_METRIC_TARGET");
 if(!s->target_valid||!s->dock_valid)return;
 const lab_localization_view*l=&v->localization;
 if(!v->has_localization||!l->has_pose||!lab_real_grid_valid(&l->map)||s->map_id!=l->map.map_id){snprintf(s->preview_reason,sizeof(s->preview_reason),"NO_MATCHING_MAP_POSE");return;}
 if(!l->pose_time_us||l->pose_time_us>now||now-l->pose_time_us>1500000){snprintf(s->preview_reason,sizeof(s->preview_reason),"MAP_POSE_OLDER_THAN_1500MS");return;}
 if(s->historical){snprintf(s->preview_reason,sizeof(s->preview_reason),"HISTORICAL_TARGET_NOT_NEW_PLAN");return;}
 passage_world w={0};w.config=*c;w.global=(passage_grid){l->map.width,l->map.height,l->map.resolution_m,l->map.origin_x_m,l->map.origin_y_m,l->map.cells};w.now_us=now;w.initial=(passage_pose){l->pose.x_m,l->pose.y_m,l->pose.yaw_rad};w.allow_initial_unknown=1;
 uint64_t begin=scene_clock(),deadline=begin+150000;w.cancelled=expired;w.cancel_user=&deadline;
 passage_xy path[OBJ_SCENE_PATH_MAX];unsigned count=0,expanded=0;passage_plan_report report={0};
 int result=passage_plan_ex(&w,w.initial,(passage_xy){s->dock_x,s->dock_y},1,s->dock_yaw,path,OBJ_SCENE_PATH_MAX,&count,&expanded,&report);
 s->plan_us=now;s->plan_elapsed_us=scene_clock()-begin;s->plan_expanded=expanded;s->plan_version=l->map.map_version;s->plan_goal_x=s->dock_x;s->plan_goal_y=s->dock_y;
 if(result==PASSAGE_CLEAR&&count>=2&&count<=OBJ_SCENE_PATH_MAX&&report.complete){for(unsigned i=0;i<count;i++){if(!isfinite(path[i].x)||!isfinite(path[i].y)){snprintf(s->preview_reason,sizeof(s->preview_reason),"PLANNER_NONFINITE_POINT");return;}s->path[i][0]=path[i].x;s->path[i][1]=path[i].y;}s->path_count=count;snprintf(s->preview_status,sizeof(s->preview_status),"READY");snprintf(s->preview_reason,sizeof(s->preview_reason),"BODY_CHECKED_READONLY_ROUTE");}
 else {snprintf(s->preview_status,sizeof(s->preview_status),"NO_PATH");snprintf(s->preview_reason,sizeof(s->preview_reason),"%s%s",passage_reason_text(result),report.partial?"_PARTIAL_NOT_REACHABLE":"");}
}
void obj_scene_cloud(obj_scene*s,const lab_real_runtime_view*v,const passage_config*c,uint64_t now){
 if(!s||!v||!c)return;s->cloud_count=0;s->obstacle_source_us=0;s->obstacle_raw_scan_points=s->obstacle_front_points=s->obstacle_left_points=s->obstacle_right_points=s->obstacle_rear_points=0;
 int current_pose=v->has_odometry&&(v->raw_odometry.quality&LAB_POSE_VALID)&&!(v->raw_odometry.quality&(LAB_POSE_GAP|LAB_POSE_STALE))&&v->odometry_time_us&&v->odometry_time_us<=now&&now-v->odometry_time_us<=300000&&isfinite(v->raw_odometry.x_m)&&isfinite(v->raw_odometry.y_m)&&isfinite(v->raw_odometry.yaw_rad);
 const lab_localization_view*l=&v->localization;s->cloud_pose_valid=current_pose&&v->has_localization&&l->has_pose&&isfinite(l->map_odom_x_m)&&isfinite(l->map_odom_y_m)&&isfinite(l->map_odom_yaw_rad);
 if(s->cloud_pose_valid){double a=l->map_odom_yaw_rad;s->cloud_pose[0]=l->map_odom_x_m+cos(a)*v->raw_odometry.x_m-sin(a)*v->raw_odometry.y_m;s->cloud_pose[1]=l->map_odom_y_m+sin(a)*v->raw_odometry.x_m+cos(a)*v->raw_odometry.y_m;s->cloud_pose[2]=a+v->raw_odometry.yaw_rad;}
 const struct scan_frame*f=v->latest_scan;if(!f||f->count>SCAN_MAX_POINTS||!v->has_scan||!v->scan_time_us||v->scan_time_us>now||now-v->scan_time_us>800000)return;
 s->obstacle_source_us=v->scan_time_us;s->obstacle_raw_scan_points=f->count;
 for(unsigned i=0;i<f->count;i++){if(!scan_return_in_range(f,i,c->lidar_min_m,fmin(12,c->lidar_max_m)))continue;const struct scan_point*q=f->points+i;double d=q->distance_q2/4000.,a=-(double)q->angle_q14*3.14159265358979323846/32768+c->mount_yaw_rad,x=c->mount_x_m+d*cos(a),y=c->mount_y_m+d*sin(a),angle=fabs(atan2(y,x));
  if(angle<=.7853981634){if(hypot(x,y)<=10)s->obstacle_front_points++;}else if(angle<=2.3561944902){if(hypot(x,y)<=8){if(y>=0)s->obstacle_left_points++;else s->obstacle_right_points++;}}else if(hypot(x,y)<=4)s->obstacle_rear_points++;
 }
 int existing=0;for(unsigned i=0;i<3;i++)if(s->cloud_history[i].time_us&&s->cloud_history[i].session==f->session&&s->cloud_history[i].sequence==f->sequence)existing=1;
 if(!existing){unsigned k=s->cloud_next++%3;s->cloud_history[k].time_us=v->scan_time_us;s->cloud_history[k].session=f->session;s->cloud_history[k].sequence=f->sequence;s->cloud_history[k].count=0;
  int pose=current_pose;
  /* 导出器不新增历史位姿管理者，缺少同步里程计时只画当前扫描；历史显示支持不送入融合。
 */
  uint64_t delta=v->odometry_time_us>v->scan_time_us?v->odometry_time_us-v->scan_time_us:v->scan_time_us-v->odometry_time_us;pose=pose&&delta<=100000;
  s->cloud_history[k].pose_valid=pose;s->cloud_history[k].odom_x=v->raw_odometry.x_m;s->cloud_history[k].odom_y=v->raw_odometry.y_m;s->cloud_history[k].odom_yaw=v->raw_odometry.yaw_rad;
  /* 显示角度桶保留最近真实回波，避免按点序跳采漏掉孤立前景或侧面点；控制端仍接收最多8192个原始回波。

 */
  int chosen[512];double nearest[512];for(unsigned i=0;i<512;i++){chosen[i]=-1;nearest[i]=1e9;}
  for(unsigned i=0;i<f->count;i++){const struct scan_point*q=&f->points[i];if(!scan_return_in_range(f,i,c->lidar_min_m,fmin(12,c->lidar_max_m)))continue;double d=q->distance_q2/4000.,a=-(double)q->angle_q14*3.14159265358979323846/32768+c->mount_yaw_rad,x=c->mount_x_m+d*cos(a),y=c->mount_y_m+d*sin(a),angle=atan2(y,x),pi=3.14159265358979323846;unsigned bin;
   if(angle>=-pi/4&&angle<=pi/4)bin=(unsigned)fmin(191,floor((angle+pi/4)/(pi/2)*192));
   else if(angle>pi/4&&angle<=3*pi/4)bin=192+(unsigned)fmin(127,floor((angle-pi/4)/(pi/2)*128));
   else if(angle>=-3*pi/4&&angle< -pi/4)bin=320+(unsigned)fmin(127,floor((angle+3*pi/4)/(pi/2)*128));
   else{double aa=angle<0?angle+2*pi:angle;bin=448+(unsigned)fmin(63,floor((aa-3*pi/4)/(pi/2)*64));}
   double range=hypot(x,y);if(range<nearest[bin]){nearest[bin]=range;chosen[bin]=(int)i;}
  }
  for(unsigned bin=0;bin<512;bin++){if(chosen[bin]<0)continue;const struct scan_point*q=&f->points[chosen[bin]];double d=q->distance_q2/4000.,a=-(double)q->angle_q14*3.14159265358979323846/32768+c->mount_yaw_rad;unsigned n=s->cloud_history[k].count++;s->cloud_history[k].points[n][0]=c->mount_x_m+d*cos(a);s->cloud_history[k].points[n][1]=c->mount_y_m+d*sin(a);}

 }
 double yaw=v->raw_odometry.yaw_rad,cc=cos(yaw),ss=sin(yaw);
 for(unsigned k=0;k<3;k++){uint64_t t=s->cloud_history[k].time_us;if(!t||t>now||now-t>300000)continue;int same=s->cloud_history[k].session==f->session&&s->cloud_history[k].sequence==f->sequence;
  if(!same&&(!s->cloud_history[k].pose_valid||!current_pose))continue;
  for(unsigned i=0;i<s->cloud_history[k].count&&s->cloud_count<OBJ_SCENE_CLOUD_MAX;i++){double x=s->cloud_history[k].points[i][0],y=s->cloud_history[k].points[i][1];
   if(s->cloud_history[k].pose_valid&&current_pose){double a=s->cloud_history[k].odom_yaw,wx=s->cloud_history[k].odom_x+cos(a)*x-sin(a)*y,wy=s->cloud_history[k].odom_y+sin(a)*x+cos(a)*y;double dx=wx-v->raw_odometry.x_m,dy=wy-v->raw_odometry.y_m;x=cc*dx+ss*dy;y=-ss*dx+cc*dy;}
   if(hypot(x,y)>12)continue;unsigned n=s->cloud_count++;s->cloud[n][0]=x;s->cloud[n][1]=y;s->cloud[n][2]=(now-t)/1000.;
  }
 }s->cloud_sequence=f->sequence;
}
typedef struct scene_writer {char*p;size_t cap,n;int ok;} scene_writer;
static void append(scene_writer*w,const char*f,...){if(!w->ok)return;va_list a;va_start(a,f);int n=vsnprintf(w->p+w->n,w->cap-w->n,f,a);va_end(a);if(n<0||(size_t)n>=w->cap-w->n){w->ok=0;return;}w->n+=(size_t)n;}
static void quoted(scene_writer*w,const char*s){append(w,"\"");for(const unsigned char*p=(const unsigned char*)s;*p;p++){if(*p<32)append(w," ");else if(*p=='\"'||*p=='\\')append(w,"\\%c",*p);else append(w,"%c",*p);}append(w,"\"");}
int obj_scene_json(const obj_scene*s,char*out,size_t cap,size_t*written){if(!s||!out||!written||cap>OBJ_SCENE_JSON_MAX||cap<2)return 0;scene_writer w={out,cap,0,1};
 append(&w,"{\"version\":1,\"published_us\":\"%llu\",\"epoch\":\"%llu\",\"track_id\":%d,\"map_id\":\"%llu\",\"map_version\":\"%llu\",\"reason\":",(unsigned long long)s->published_us,(unsigned long long)s->epoch,s->track_id,(unsigned long long)s->map_id,(unsigned long long)s->map_version);quoted(&w,s->reason);append(&w,",\"message_zh\":");quoted(&w,s->message_zh);append(&w,",\"mount_quality\":\"%s\",\"source\":",s->approximate?"APPROXIMATE_CONFIGURED_MOUNT":"VERIFIED_CONFIGURED_MOUNT");quoted(&w,s->source);
 append(&w,",\"scale_prior\":{\"valid\":%s,\"range_m\":",s->scale_valid?"true":"false");if(s->scale_valid&&isfinite(s->scale_range_m))append(&w,"%.9g",s->scale_range_m);else append(&w,"null");append(&w,",\"reference_us\":\"%llu\",\"quality\":\"PREDICTION_ONLY_NOT_MEASURED\"}",(unsigned long long)s->scale_reference_us);
 append(&w,",\"binding_generation\":%u",s->binding_generation);
 append(&w,",\"bearing_valid\":%s,\"bearing_rad\":",s->bearing_valid?"true":"false");if(s->bearing_valid&&isfinite(s->bearing_rad))append(&w,"%.9g",s->bearing_rad);else append(&w,"null");
 append(&w,",\"fusion_diagnostics\":{\"lidar_track_valid\":%s,\"lidar_source_us\":\"%llu\",\"last_visual_us\":\"%llu\",\"lidar_updates\":%u,\"lidar_candidates\":%u,\"stereo_prior_only\":%s,\"stereo_conflict\":%s}",s->lidar_valid?"true":"false",(unsigned long long)s->lidar_source_us,(unsigned long long)s->lidar_visual_us,s->lidar_updates,s->lidar_candidates,s->stereo_prior_only?"true":"false",s->stereo_conflict?"true":"false");
 append(&w,",\"association_revision\":%u,\"follow_wall_score\":%.6g,\"follow_motion_score\":%.6g,\"follow_isolation_score\":%.6g,\"follow_candidates\":%u,\"follow_wall_rejected\":%u,\"follow_motion_support\":%u,\"follow_reason\":",s->association_revision,s->follow_wall_score,s->follow_motion_score,s->follow_isolation_score,s->follow_candidates,s->follow_wall_rejected,s->follow_motion_support);quoted(&w,s->follow_reason);
 append(&w,",\"follow_sparse\":%s,\"follow_support_points\":%u,\"follow_search_half_rad\":%.6g,\"obstacle_sensitivity\":{\"front_range_m\":10,\"side_range_m\":8,\"rear_range_m\":4,\"source_us\":\"%llu\",\"raw_scan_points\":%u,\"front_points\":%u,\"left_points\":%u,\"right_points\":%u,\"rear_points\":%u}",s->follow_sparse?"true":"false",s->follow_support_points,s->follow_search_half_rad,(unsigned long long)s->obstacle_source_us,s->obstacle_raw_scan_points,s->obstacle_front_points,s->obstacle_left_points,s->obstacle_right_points,s->obstacle_rear_points);
 const object_target_observation_v2*m=&s->measurement_v2;
 append(&w,",\"target_observation\":{\"valid\":%s,\"source_us\":\"%llu\",\"image_us\":\"%llu\",\"scan_us\":\"%llu\",\"surface_gap_m\":",m->valid?"true":"false",(unsigned long long)m->source_us,(unsigned long long)m->image_us,(unsigned long long)m->scan_us);
 if(m->valid&&isfinite(m->surface_gap_m))append(&w,"%.9g",m->surface_gap_m);else append(&w,"null");append(&w,",\"error_m\":");if(m->valid&&isfinite(m->error_m))append(&w,"%.9g",m->error_m);else append(&w,"null");append(&w,",\"source\":");quoted(&w,m->source);append(&w,",\"reason\":");quoted(&w,m->reason);append(&w,",\"mount_verified\":%s}",m->mount_verified?"true":"false");
 int valid=s->target_valid&&isfinite(s->target_x)&&isfinite(s->target_y)&&isfinite(s->range_m)&&isfinite(s->error_m);
 append(&w,",\"target\":{\"valid\":%s,\"confirmed\":%s,\"historical\":%s,\"map_x_m\":",valid?"true":"false",s->confirmed?"true":"false",s->historical?"true":"false");if(valid)append(&w,"%.9g,\"map_y_m\":%.9g,\"range_m\":%.9g,\"error_m\":%.9g",s->target_x,s->target_y,s->range_m,s->error_m);else append(&w,"null,\"map_y_m\":null,\"range_m\":null,\"error_m\":null");append(&w,",\"source_us\":\"%llu\",\"frame_id\":\"%llu\",\"scan_us\":\"%llu\",\"support\":%u}",(unsigned long long)s->source_us,(unsigned long long)s->frame_id,(unsigned long long)s->scan_us,s->support);
 int dock=valid&&s->dock_valid&&isfinite(s->dock_x)&&isfinite(s->dock_y)&&isfinite(s->dock_yaw);append(&w,",\"dock\":{\"valid\":%s,\"map_x_m\":",dock?"true":"false");if(dock)append(&w,"%.9g,\"map_y_m\":%.9g,\"yaw_rad\":%.9g",s->dock_x,s->dock_y,s->dock_yaw);else append(&w,"null,\"map_y_m\":null,\"yaw_rad\":null");append(&w,",\"standoff_m\":%.6g},\"preview\":{\"status\":",s->standoff_m);quoted(&w,s->preview_status);append(&w,",\"kind\":");quoted(&w,s->preview_kind[0]?s->preview_kind:"NONE");append(&w,",\"frame\":\"MAP\",\"source_frame\":");quoted(&w,s->preview_source_frame[0]?s->preview_source_frame:"MAP");append(&w,",\"reason\":");quoted(&w,s->preview_reason);append(&w,",\"elapsed_us\":\"%llu\",\"expanded\":%u,\"readonly\":%s,\"path\":[",(unsigned long long)s->plan_elapsed_us,s->plan_expanded,!strcmp(s->preview_kind,"EXECUTING_REFERENCE_ROUTE")||!strcmp(s->preview_kind,"VALIDATED_ACTION")?"false":"true");
 for(unsigned i=0;i<s->path_count&&i<OBJ_SCENE_PATH_MAX;i++)append(&w,"%s[%.9g,%.9g]",i?",":"",s->path[i][0],s->path[i][1]);append(&w,"]},\"front_cloud\":{\"range_m\":12,\"max_range_m\":12,\"sector_deg\":360,\"history_max_ms\":300,\"pose_valid\":%s,\"pose\":[%.9g,%.9g,%.9g],\"points\":[",s->cloud_pose_valid?"true":"false",s->cloud_pose[0],s->cloud_pose[1],s->cloud_pose[2]);
 for(unsigned i=0;i<s->cloud_count&&i<OBJ_SCENE_CLOUD_MAX;i++)append(&w,"%s[%.7g,%.7g,%.1f]",i?",":"",s->cloud[i][0],s->cloud[i][1],s->cloud[i][2]);append(&w,"]}}");if(!w.ok){out[0]=0;*written=0;return 0;}*written=w.n;return 1;
}
