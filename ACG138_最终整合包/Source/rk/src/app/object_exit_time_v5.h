#ifndef ACG_OBJECT_EXIT_TIME_V5_H
#define ACG_OBJECT_EXIT_TIME_V5_H
#include "object_scene.h"
#include <math.h>
#include <stdio.h>
typedef int (*obj_exit_yaw_lookup_v5)(void*,uint64_t,double*);
/* 仅修改私有控制副本，将最后真实射线转换到当前车体，任务只外推一次世界角速度，不刷新源时间。
 */
static inline int obj_exit_compensate_v5(object_follow_exit_v5*e,const obj_scene_config*m,uint64_t now,double current_yaw,obj_exit_yaw_lookup_v5 lookup,void*u){
 if(!e)return 0;e->compensated=0;
 if(!e->valid||(e->kind!=1&&e->kind!=2)||!e->source_us||e->source_us>now||now-e->source_us>2000000||e->count<2||e->count>OBJECT_FOLLOW_KEYFRAMES_V5_MAX||!isfinite(current_yaw)||!lookup)return 0;
 double times[8],angles[8],last=0;unsigned n=0;int have_last=0;
 uint64_t previous_time=0,previous_frame=0;
 for(unsigned i=0;i<e->count;i++){
  const object_follow_keyframe_v5*k=e->keyframes+i;
  if(!k->source_us||k->source_us>e->source_us||!k->frame_id||k->frame_id>e->frame_id||k->source_us<=previous_time||k->frame_id<=previous_frame)return 0;
  previous_time=k->source_us;previous_frame=k->frame_id;
  if(e->source_us-k->source_us>1500000)continue;
  double yaw=0,ray[3],body[3];if(!lookup(u,k->source_us,&yaw)||!isfinite(yaw))continue;
  if(k->ray_valid){for(unsigned z=0;z<3;z++)ray[z]=k->ray_direction[z];}
  else if(k->bearing_valid&&k->eye==0&&isfinite(k->bearing_rad)){ray[0]=-sin(k->bearing_rad);ray[1]=0;ray[2]=cos(k->bearing_rad);}
  else continue;
  obj_scene_camera_vector(m,ray,body);if(!isfinite(body[0])||!isfinite(body[1])||hypot(body[0],body[1])<1e-6)continue;
  double angle=yaw+atan2(body[1],body[0]);if(n)angle=last+remainder(angle-last,6.283185307179586);last=angle;
  times[n]=-(double)(e->source_us-k->source_us)/1e6;angles[n++]=angle;
  if(k->source_us==e->source_us&&k->frame_id==e->frame_id)have_last=1;
 }
 if(!have_last||n<2||times[n-1]-times[0]<.03)return 0;
 double mt=0,ma=0,den=0,num=0;for(unsigned i=0;i<n;i++){mt+=times[i];ma+=angles[i];}mt/=n;ma/=n;
 for(unsigned i=0;i<n;i++){den+=(times[i]-mt)*(times[i]-mt);num+=(times[i]-mt)*(angles[i]-ma);}
 if(den<1e-6)return 0;
 e->angular_velocity_rad_s=fmax(-1,fmin(1,num/den));e->bearing_rad=remainder(angles[n-1]-current_yaw,6.283185307179586);e->bearing_valid=1;e->compensated=1;
 snprintf(e->reason,sizeof(e->reason),"KEYFRAMES_CAPTURE_POSES_MOUNT_COMPENSATED");return 1;
}
#endif
