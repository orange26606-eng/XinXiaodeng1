#ifndef ACG_S2_MOTION_PREVIEW_H
#define ACG_S2_MOTION_PREVIEW_H
#include "lab_real_runtime.h"
#include "navigation_status.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#define S2_PREVIEW_MAX 33u
/* 0.8秒条件几何投影不代表同等时长电机授权，显示时效仍遵守原命令不超过500ms的期限。
 */
typedef struct s2_preview_point {double x,y,yaw;} s2_preview_point;
typedef struct s2_motion_preview {
 unsigned valid,commanded,count;uint64_t generated_us,expires_us,pose_us;
 double v,w,horizon;const char*source;const char*reason;
 s2_preview_point points[S2_PREVIEW_MAX];
} s2_motion_preview;
static inline int s2_preview_command_fresh(const lab_real_command*c,uint64_t now){
 return c&&c->generated_us&&c->generated_us<=now&&c->expires_us>now&&c->expires_us>c->generated_us&&c->expires_us-c->generated_us<=CONT_COMMAND_US&&isfinite(c->linear_m_s)&&isfinite(c->angular_rad_s);
}
static inline int s2_motion_preview_make(const lab_real_runtime_view*v,uint64_t now,s2_motion_preview*out){
 if(!out)return 0;memset(out,0,sizeof(*out));out->source="NONE";out->reason="NO_ACTIVE_ACTION";
 if(!v||!v->localization.map.map_id||!navigation_pose_valid(v)||!v->localization.pose_time_us||v->localization.pose_time_us>now||now-v->localization.pose_time_us>CONT_ODOM_US){out->reason="WAIT_CURRENT_POSE";return 0;}
 const lab_real_command*c=NULL;
 if(v->enabled){
  if(!v->protected_output.fresh_command_required&&v->protected_output.armed&&v->protected_output.command.enabled&&!v->protected_output.reasons&&s2_preview_command_fresh(&v->protected_output.command,now)){
   c=&v->protected_output.command;out->commanded=1;out->source="PROTECTED_COMMAND";
   /* 预览使用底盘限幅与反向处理后的最终机械轮速。 */

   double l=(v->protected_output.wheel_rpm[0]+v->protected_output.wheel_rpm[2])*.5;
   double rr=(v->protected_output.wheel_rpm[1]+v->protected_output.wheel_rpm[3])*.5;
   if(!isfinite(l)||!isfinite(rr)||fabs(l)>120.001||fabs(rr)>120.001){out->reason="INVALID_PROTECTED_TARGETS";return 0;}
   const double metres_per_rpm=3.14159265358979323846*.0675/60;
   out->v=(l+rr)*.5*metres_per_rpm;out->w=(rr-l)*metres_per_rpm/.29;
  }else{out->reason="WAIT_CURRENT_PROTECTED_COMMAND";return 0;}
 }else if(v->has_goal&&!v->pending_enable&&s2_preview_command_fresh(&v->request,now)){
  c=&v->request;out->source="PREVIEW";out->v=c->linear_m_s;out->w=c->angular_rad_s;
 }else return 0;
 out->generated_us=c->generated_us;out->expires_us=c->expires_us;out->pose_us=v->localization.pose_time_us;
 if(fabs(out->v)<1e-9&&fabs(out->w)<1e-9){out->reason="CURRENT_ZERO_ACTION";return 0;}
 double x=v->localization.pose.x_m,y=v->localization.pose.y_m,a=v->localization.pose.yaw_rad;
 if(!isfinite(x)||!isfinite(y)||!isfinite(a)){out->reason="INVALID_POSE";return 0;}
 out->horizon=.8;out->count=S2_PREVIEW_MAX;
 for(unsigned i=0;i<S2_PREVIEW_MAX;i++){
  out->points[i]=(s2_preview_point){x,y,a};if(i+1==S2_PREVIEW_MAX)break;
  double dt=out->horizon/(S2_PREVIEW_MAX-1),next=a+out->w*dt;
  if(fabs(out->w)<1e-9){x+=out->v*dt*cos(a);y+=out->v*dt*sin(a);}
  else{x+=out->v/out->w*(sin(next)-sin(a));y+=out->v/out->w*(cos(a)-cos(next));}
  a=next;
 }
 out->valid=1;out->reason=out->commanded?"CONDITIONAL_CURRENT_WHEEL_PROJECTION":"ACTION_PREVIEW_NOT_EXECUTING";return 1;
}
#endif
