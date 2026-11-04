#ifndef ACG_SEMANTIC_OBSERVATION_H
#define ACG_SEMANTIC_OBSERVATION_H
#include "object_readonly.h"
#include <string.h>
/* 选择完整原始帧，不把observed=true复制到预测帧，也不以发布时间替换观测时刻。
 */
static inline int semantic_observation_category(const char*a,const char*b){
 return !a||!*a||!strcmp(a,b)||(!strcmp(a,"cup")&&!strcmp(b,"glass"))||
  ((!strcmp(a,"cell phone")||!strcmp(a,"cell_phone"))&&(!strcmp(b,"cell phone")||!strcmp(b,"cell_phone")));
}
static inline int semantic_observation_matches_attributes(const obj_frame*f,uint64_t now,uint64_t epoch,int track,const char*category,const char*color,const char*size){
 if(!f||!f->epoch||!f->frame_us||f->frame_us>now||now-f->frame_us>UINT64_C(2000000)||(epoch&&f->epoch!=epoch)||f->count<0||f->count>OBJ_MAX)return 0;
 for(int i=0;i<f->count;i++){
  const obj_detection*d=&f->objects[i];
  if(d->track_id<1||!d->observed||(track>0&&d->track_id!=track)||!semantic_observation_category(category,d->label))continue;
  if(!obj_attribute_color_match(&d->attributes,color))continue;
  if(color&&*color&&d->attributes.color_source_us&&(d->attributes.color_source_us>now||now-d->attributes.color_source_us>UINT64_C(2000000)))continue;
  /* 大小可用性由候选选择判断，不由帧读取判断；大小未知时仍允许真实颜色及类别候选被观测，历史米制尺寸不获得新时间。

 */
  (void)size;
  if(d->observation_time_us&&(d->observation_time_us>now||now-d->observation_time_us>UINT64_C(2000000)))continue;
  return 1;
 }
 return 0;
}
static inline uint64_t semantic_observation_attribute_time(const obj_frame*f,uint64_t now,uint64_t epoch,int track,const char*category,const char*color,const char*size){
 if(!semantic_observation_matches_attributes(f,now,epoch,track,category,color,size))return 0;
 uint64_t best=0;
 for(int i=0;i<f->count;i++){
  const obj_detection*d=f->objects+i;
  if(!d->observed||d->track_id<1||(track>0&&d->track_id!=track)||!semantic_observation_category(category,d->label)||!obj_attribute_color_match(&d->attributes,color))continue;
  uint64_t stamp=d->observation_time_us?d->observation_time_us:f->frame_us;
  if(!stamp||stamp>now||now-stamp>UINT64_C(2000000))continue;
  if(color&&*color&&d->attributes.color_source_us){uint64_t c=d->attributes.color_source_us;if(c>now||now-c>UINT64_C(2000000))continue;if(c<stamp)stamp=c;}
  if(size&&*size&&d->attributes.size_valid&&d->attributes.size_source_us&&d->attributes.size_source_us<stamp)stamp=d->attributes.size_source_us;
  if(stamp>best)best=stamp;
 }
 return best;
}
static inline const obj_frame*semantic_observation_pick_attributes(const obj_frame*state,const obj_frame*detector,uint64_t now,uint64_t epoch,int track,const char*category,const char*color,const char*size){
 int actual_state=semantic_observation_matches_attributes(state,now,epoch,track,category,color,size);
 int actual_detector=semantic_observation_matches_attributes(detector,now,epoch,track,category,color,size)&&(!state||!state->epoch||detector->epoch==state->epoch);
 int attributed=(color&&*color)||(size&&*size);
 if(actual_detector&&(!actual_state||(attributed?semantic_observation_attribute_time(detector,now,epoch,track,category,color,size)>=semantic_observation_attribute_time(state,now,epoch,track,category,color,size):detector->frame_us>state->frame_us)))return detector;
 return state;
}
static inline int semantic_observation_matches(const obj_frame*f,uint64_t now,uint64_t epoch,int track,const char*category){return semantic_observation_matches_attributes(f,now,epoch,track,category,NULL,NULL);}
static inline const obj_frame*semantic_observation_pick(const obj_frame*state,const obj_frame*detector,uint64_t now,uint64_t epoch,int track,const char*category){return semantic_observation_pick_attributes(state,detector,now,epoch,track,category,NULL,NULL);}
#endif
