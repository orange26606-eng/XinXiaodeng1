#include "object_navigation.h"
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>
static void state(object_navigation*t,const char*s,const char*r){snprintf(t->state,sizeof(t->state),"%s",s);snprintf(t->reason,sizeof(t->reason),"%s",r);}
void object_navigation_cancel(object_navigation*t,const char*r){if(!t)return;t->active=0;t->blocked=1;state(t,"CANCELLED",r);}
void object_navigation_reject(object_navigation*t,const char*r){t->active=0;t->blocked=1;state(t,"REJECTED",r);}
static int fresh(const obj_lidar_target*a,const lab_real_runtime_view*v,uint64_t now){
 return a&&a->valid&&!strcmp(a->status,"CONFIRMED")&&a->association_id[0]&&a->frame_us&&a->scan_us&&a->published_us&&a->frame_us<=now&&a->scan_us<=now&&a->published_us<=now&&now-a->frame_us<=500000&&now-a->scan_us<=800000&&now-a->published_us<=500000&&isfinite(a->map_x)&&isfinite(a->map_y)&&isfinite(a->error_m)&&a->error_m>=0&&a->error_m<=.5&&a->map_id==v->localization.map.map_id&&a->map_version==v->localization.map.map_version&&a->context.source==v->context.source&&a->context.clock==v->context.clock&&a->context.timeline_id==v->context.timeline_id&&a->context.origin_epoch==v->context.origin_epoch&&v->has_localization&&(v->localization.state==LAB_REAL_LOC_TRACKING||v->localization.odometry_fallback);
}
int object_navigation_matches(const object_navigation*t,const obj_lidar_target*a,const lab_real_runtime_view*v,uint64_t now){return t&&t->active&&fresh(a,v,now)&&a->epoch==t->epoch&&a->track_id==t->track_id&&a->map_id==t->map_id&&!strcmp(a->association_id,t->association_id)&&hypot(a->map_x-t->target_x,a->map_y-t->target_y)<=.20;}
int object_navigation_preview(object_navigation*t,const phone_goal_message*m,const obj_lidar_target*a,const lab_real_runtime_view*v,double front,uint64_t now){
 *t=(object_navigation){.epoch=m->object_epoch,.track_id=m->object_track_id,.map_id=m->map_id,.map_version=m->map_version,.request_us=m->sent_us,.created_us=now,.standoff_m=m->x_m,.front_m=front};snprintf(t->association_id,sizeof(t->association_id),"%s",m->association_id);
 for(size_t i=0;m->association_id[i];i++){unsigned char c=(unsigned char)m->association_id[i];if(!isalnum(c)&&c!='-'&&c!='_'&&c!='.'&&c!=':'){object_navigation_reject(t,"OBJECT_ASSOCIATION_ID_INVALID");return 0;}}
 if(!isfinite(m->x_m)||m->x_m<.2||m->x_m>3||!isfinite(front)||front<=0||front>2||!m->association_id[0]||!fresh(a,v,now)||a->epoch!=m->object_epoch||a->track_id!=m->object_track_id||a->map_id!=m->map_id||(!m->map_version||m->map_version>a->map_version)||strcmp(a->association_id,m->association_id)){object_navigation_reject(t,"OBJECT_TARGET_IDENTITY_CONTEXT_OR_FRESHNESS_INVALID");return 0;}
 double dx=a->map_x-v->localization.pose.x_m,dy=a->map_y-v->localization.pose.y_m,d=hypot(dx,dy),offset=m->x_m+front;
 if(!isfinite(d)||d<offset+.05){object_navigation_reject(t,"OBJECT_INSUFFICIENT_APPROACH_DISTANCE");return 0;}
 t->active=1;t->frame_us=a->frame_us;t->target_x=a->map_x;t->target_y=a->map_y;t->goal_yaw=atan2(dy,dx);t->goal_x=a->map_x-offset*dx/d;t->goal_y=a->map_y-offset*dy/d;state(t,"PREVIEW_PENDING","OBJECT_ROUTE_PENDING_EXPLICIT_START_REQUIRED");return 1;
}
int object_navigation_can_start(const object_navigation*t,const obj_lidar_target*a,const lab_real_runtime_view*v,uint64_t now){return t&&!t->blocked&&!strcmp(t->state,"PREVIEW_READY")&&object_navigation_matches(t,a,v,now)&&v->has_goal&&v->navigation.path_count&&v->goal.map_id==t->map_id&&fabs(v->goal.x_m-t->goal_x)<1e-6&&fabs(v->goal.y_m-t->goal_y)<1e-6;}
int object_navigation_tick(object_navigation*t,const obj_lidar_target*a,const lab_real_runtime_view*v,int same_goal,uint64_t now){
 if(!t->active||t->blocked)return 0;
 if(!object_navigation_matches(t,a,v,now)){t->blocked=1;state(t,"WAIT_TARGET","OBJECT_ASSOCIATION_LOST_STALE_OR_CHANGED_REPREVIEW_REQUIRED");return 1;}
 if(!strcmp(t->state,"PREVIEW_PENDING")){if(same_goal&&v->has_goal&&v->navigation.path_count)state(t,"PREVIEW_READY","OBJECT_ROUTE_AVAILABLE_EXPLICIT_START_REQUIRED");else if(now-t->created_us>3000000){object_navigation_reject(t,"OBJECT_ROUTE_UNREACHABLE_OR_TIMEOUT");return 1;}}
 if(!strcmp(t->state,"PREVIEW_READY")&&(!same_goal||!v->has_goal||!v->navigation.path_count)){object_navigation_reject(t,"OBJECT_ROUTE_NO_LONGER_AVAILABLE");return 1;}
 if(!strcmp(t->state,"NAVIGATING")&&!strcmp(v->reason,"GOAL_REACHED")){t->reached_us=now;state(t,"WAIT_REOBSERVE","OBJECT_WAIT_NEW_IMAGE_AND_SCAN_AFTER_STOP");return 1;}
 if(!strcmp(t->state,"WAIT_REOBSERVE")){
  if(a->frame_us<=t->reached_us||a->scan_us<=t->reached_us)return 0;
  double dx=a->map_x-v->localization.pose.x_m,dy=a->map_y-v->localization.pose.y_m,yaw=atan2(dy,dx)-v->localization.pose.yaw_rad;yaw=atan2(sin(yaw),cos(yaw));
  if(!v->enabled&&!v->pending_enable&&fabs(hypot(dx,dy)-(t->standoff_m+t->front_m))<=.2&&fabs(yaw)<=.35){t->active=0;state(t,"ARRIVED","OBJECT_POSTSTOP_REOBSERVATION_CONFIRMED");}
  else if(now-t->reached_us>5000000)snprintf(t->reason,sizeof(t->reason),"OBJECT_ARRIVAL_UNCONFIRMED_REPREVIEW_REQUIRED");
 }
 return 0;
}
