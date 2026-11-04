#ifndef ACG_OBJECT_BODY_TARGET_V9_H
#define ACG_OBJECT_BODY_TARGET_V9_H
#include "object_dual_task.h"
#include <stdio.h>
#include <math.h>
/* 将真实测量重表达到当前原始ODOM车体系，不使用地图变换、单距离构造、预测或时间刷新。
 */
static inline int object_body_target_v9_json(const obj_scene*s,const object_dual_task*t,const lab_real_runtime_view*v,uint64_t now,char*out,size_t cap){
 const object_target_observation_v2*m=&s->measurement_v2;
 int valid=m->valid&&m->epoch==s->epoch&&m->track_id==s->track_id&&lab_real_context_equal(&m->context,&v->context)&&
  (!t->prepared||(m->epoch==t->epoch&&m->track_id==t->track_id))&&m->source_us&&m->source_us<=now&&now-m->source_us<=OBJECT_CONTROL_MEASUREMENT_US&&
  v->has_odometry&&(v->raw_odometry.quality&LAB_POSE_VALID)&&v->odometry_time_us&&v->odometry_time_us<=now&&now-v->odometry_time_us<=OBJECT_CONTROL_POSE_US&&
  isfinite(m->odom_x_m)&&isfinite(m->odom_y_m)&&isfinite(v->raw_odometry.x_m)&&isfinite(v->raw_odometry.y_m)&&isfinite(v->raw_odometry.yaw_rad);
 char x[48]="null",y[48]="null",source[48];unsigned i=0;
 for(;m->source[i]&&i<sizeof(source)-1;i++){unsigned char c=(unsigned char)m->source[i];source[i]=(c<32||c=='"'||c=='\\')?'_':(char)c;}source[i]=0;
 if(valid){double dx=m->odom_x_m-v->raw_odometry.x_m,dy=m->odom_y_m-v->raw_odometry.y_m,c=cos(v->raw_odometry.yaw_rad),sn=sin(v->raw_odometry.yaw_rad);snprintf(x,sizeof(x),"%.9g",c*dx+sn*dy);snprintf(y,sizeof(y),"%.9g",-sn*dx+c*dy);}
 int n=snprintf(out,cap,"\"body_target\":{\"valid\":%s,\"x_m\":%s,\"y_m\":%s,\"source_us\":\"%llu\",\"body_pose_us\":\"%llu\",\"source\":\"%s\",\"historical\":false}",valid?"true":"false",x,y,(unsigned long long)m->source_us,(unsigned long long)v->odometry_time_us,source);
 return n>0&&(size_t)n<cap;
}
#endif
