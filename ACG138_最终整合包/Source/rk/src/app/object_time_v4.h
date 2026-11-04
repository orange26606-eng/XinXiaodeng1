#ifndef ACG_OBJECT_TIME_V4_H
#define ACG_OBJECT_TIME_V4_H
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "object_readonly.h"
typedef struct obj_image_source_v4 {int valid;uint64_t time_us,frame_id;} obj_image_source_v4;
static inline obj_image_source_v4 obj_selected_image_source_v4(const obj_frame*f){
 obj_image_source_v4 r={0,0,0};if(!f)return r;
 for(int i=0;i<f->count&&i<OBJ_MAX;i++){const obj_detection*d=&f->objects[i];if(d->track_id!=f->selected)continue;
  if(!d->observed||!strcmp(d->tracking,"PREDICTED")||!strcmp(d->tracking,"LOST"))return r;
  r.time_us=d->observation_time_us?d->observation_time_us:f->frame_us;r.frame_id=d->observation_frame_id?d->observation_frame_id:f->frame_id;r.valid=r.time_us&&r.frame_id&&r.time_us<=f->frame_us&&r.frame_id<=f->frame_id;return r;
 }return r;
}
typedef struct obj_direction_time_v4 {int valid,kind;double delta_rad,error_rad;const char*reason;} obj_direction_time_v4;
/* 旋转仅作用于控制副本。优先采用历史位姿，模型及当前位姿近似均标记误差和有效期。 */


static inline obj_direction_time_v4 obj_direction_time_v4_resolve(uint64_t image,uint64_t now,int history,double capture_yaw,double current_yaw,int rate_valid,uint64_t rate_time,double rate){
 obj_direction_time_v4 r={0,0,0,0,"CAPTURE_POSE_UNAVAILABLE"};
 if(!image||image>now||!isfinite(current_yaw)){r.reason=image>now?"IMAGE_TIME_IN_FUTURE":"IMAGE_TIME_OR_POSE_INVALID";return r;}
 if(history&&isfinite(capture_yaw)){r.valid=1;r.kind=1;r.delta_rad=remainder(capture_yaw-current_yaw,6.283185307179586);r.reason="CAPTURE_POSE_HISTORY";return r;}
 uint64_t age=now-image;
 if(age<=500000&&rate_valid&&rate_time<=now&&now-rate_time<=200000&&isfinite(rate)&&fabs(rate)<=2){r.valid=1;r.kind=2;r.delta_rad=-rate*(age/1e6);r.error_rad=.03+.30*(age/1e6);r.reason="BOUNDED_YAW_RATE_MODEL";return r;}
 if(age<=200000){r.valid=1;r.kind=3;r.error_rad=.30*(age/1e6);r.reason="FRESH_CURRENT_POSE_APPROXIMATION";return r;}
 return r;
}
#endif
