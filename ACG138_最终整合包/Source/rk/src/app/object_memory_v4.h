#ifndef ACG_OBJECT_MEMORY_V4_H
#define ACG_OBJECT_MEMORY_V4_H
#include "object_scene.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef struct obj_memory_extra_v4 {int valid,task_class_id;uint32_t binding_generation;uint64_t pose_time_us;double x,y,range,error;const obj_scene_candidate*candidates;size_t count;} obj_memory_extra_v4;
typedef struct om4_writer {char*text;size_t capacity,used;int ok;} om4_writer;
static inline void om4_write(om4_writer*w,const char*fmt,...){if(!w->ok)return;va_list a;va_start(a,fmt);int n=vsnprintf(w->text+w->used,w->capacity-w->used,fmt,a);va_end(a);if(n<0||(size_t)n>=w->capacity-w->used)w->ok=0;else w->used+=(size_t)n;}
/* 追加到完整v1 JSON对象，本模块不生成源时刻；各候选图像与扫描源时钟保持独立。
 */
static inline size_t obj_memory_v4_append(char*json,size_t capacity,size_t length,const obj_memory_extra_v4*e){
 if(!json||!e||length<2||length>=capacity||json[length-1]!='}'||e->count>16||(e->count&&!e->candidates))return 0;
 om4_writer w={json,capacity,length-1,1};int valid=e->valid&&isfinite(e->x)&&isfinite(e->y)&&isfinite(e->range)&&e->range>=0&&isfinite(e->error)&&e->error>=0;
 om4_write(&w,",\"task_class_id\":%d,\"binding_generation\":%u,\"target_frame\":\"ODOM\",\"pose_time_us\":\"%llu\",\"camera_time_compensated\":%s,\"target_odom_x_m\":",e->task_class_id,e->binding_generation,(unsigned long long)e->pose_time_us,valid?"true":"false");
 if(valid)om4_write(&w,"%.9g,\"target_odom_y_m\":%.9g,\"target_range_m\":%.9g,\"target_error_m\":%.9g",e->x,e->y,e->range,e->error);else om4_write(&w,"null,\"target_odom_y_m\":null,\"target_range_m\":null,\"target_error_m\":null");
 om4_write(&w,",\"spatial_candidates\":[");
 for(size_t i=0;i<e->count;i++){
  const obj_scene_candidate*c=e->candidates+i;
  if(c->track_id<1||c->class_id<0||c->class_id>80||c->eye<0||c->eye>1||!c->frame_id||!c->image_time_us||!c->source_us||!c->scan_us||!isfinite(c->bearing_rad)||!isfinite(c->range_m)||c->range_m<=0||!isfinite(c->odom_x_m)||!isfinite(c->odom_y_m)||!isfinite(c->error_m)||c->error_m<0)return 0;
  for(int k=0;k<4;k++)if(!isfinite(c->bbox[k]))return 0;
  om4_write(&w,"%s{\"frame_id\":\"%llu\",\"image_time_us\":\"%llu\",\"source_us\":\"%llu\",\"scan_us\":\"%llu\",\"track_id\":%d,\"class_id\":%d,\"eye\":%d,\"bbox\":[%.9g,%.9g,%.9g,%.9g],\"bearing_rad\":%.9g,\"range_m\":%.9g,\"odom_x_m\":%.9g,\"odom_y_m\":%.9g,\"error_m\":%.9g}",i?",":"",(unsigned long long)c->frame_id,(unsigned long long)c->image_time_us,(unsigned long long)c->source_us,(unsigned long long)c->scan_us,c->track_id,c->class_id,c->eye,c->bbox[0],c->bbox[1],c->bbox[2],c->bbox[3],c->bearing_rad,c->range_m,c->odom_x_m,c->odom_y_m,c->error_m);
 }
 om4_write(&w,"]}");return w.ok?w.used:0;
}
#endif
