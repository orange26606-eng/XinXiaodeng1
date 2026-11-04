#ifndef ACG_VOICE_PERCEPTION_H
#define ACG_VOICE_PERCEPTION_H
#include "../rk/src/app/semantic_observation.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 高速原图或预测发布不能覆盖尚未消费的真实检测。检测伴随文件仅在实际推理帧上原子发布，并保留原epoch、帧编号和源时间。

 */
typedef struct voice_perception_diag {
 const char *source,*reason;
 uint64_t detector_frame,detector_us,tracking_frame,tracking_us;
 int detector_objects,tracking_objects;
} voice_perception_diag;
static inline int voice_perception_load_attributes(const char *dir,uint64_t now,obj_frame *out,voice_perception_diag *diag,const char *category,const char *color,const char *size){
 char path[1024];voice_perception_diag d={.source="NONE",.reason="PERCEPTION_UNAVAILABLE"};
 if(!dir||!out){if(diag)*diag=d;return 0;}
 int tracking=snprintf(path,sizeof path,"%s/objects-state.json",dir)<(int)sizeof path&&obj_frame_load(path,out);
 if(tracking){d.tracking_frame=out->frame_id;d.tracking_us=out->frame_us;d.tracking_objects=out->count;d.source="TRACKING_STATE";d.reason="REAL_DETECTION_UNAVAILABLE_FALLBACK";}
 obj_frame *real=calloc(1,sizeof *real);
 if(real&&snprintf(path,sizeof path,"%s/objects-detections.json",dir)<(int)sizeof path&&obj_frame_load(path,real)){
  d.detector_frame=real->frame_id;d.detector_us=real->frame_us;d.detector_objects=real->count;
  if(!real->epoch||!real->frame_id||!real->frame_us||real->frame_us>now)d.reason="DETECTION_SOURCE_TIME_INVALID";
  else if(now-real->frame_us>UINT64_C(2000000))d.reason="DETECTION_SOURCE_OLDER_THAN_2S";
  else if(tracking&&real->epoch!=out->epoch)d.reason="DETECTION_EPOCH_CHANGED_WAIT_PUBLICATION";
  else if(semantic_observation_pick_attributes(tracking?out:NULL,real,now,0,0,category,color,size)==real){*out=*real;d.source="REAL_DETECTIONS";d.reason=(color&&*color)||(size&&*size)?"RECENT_REAL_ATTRIBUTE_MATCH_NOT_PREDICTION":"RECENT_REAL_DETECTION_NOT_PREDICTION";tracking=1;}
  else d.reason="NO_NEWER_REAL_MATCH_KEEP_TRACKING";
 }
 free(real);if(diag)*diag=d;return tracking;
}
static inline int voice_perception_load(const char *dir,uint64_t now,obj_frame *out,voice_perception_diag *diag,const char *category){return voice_perception_load_attributes(dir,now,out,diag,category,NULL,NULL);}
#endif
