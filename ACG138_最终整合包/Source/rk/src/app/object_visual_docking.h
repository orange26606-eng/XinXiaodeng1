#ifndef ACG_OBJECT_VISUAL_DOCKING_H
#define ACG_OBJECT_VISUAL_DOCKING_H
/* 静止目标的当前帧证据计算。 */
#include <stdint.h>
#define OBJECT_VISUAL_DOCK_SAMPLES 16u
typedef struct object_visual_geometry {
 int loaded,configured,verified,mono_valid[2],extrinsics_valid;
 double x,y,z,roll,pitch,yaw,fx[2],fy[2],cx[2],cy[2],distortion[2][5];
 double stereo_R[9],stereo_T[3];char reason[64];
} object_visual_geometry;
typedef struct object_visual_dock_sample {uint64_t frame_id,source_us;double width,height,scale,x,y,yaw,bearing;int clipped,pose_valid;} object_visual_dock_sample;
typedef struct object_visual_dock {
 uint64_t epoch,frame_id,source_us,metric_source_us,confirmation_source_us,hold_source_us,last_log_us,far_source_us,surface_source_us;
 uint32_t binding;int track_id,eye,initialized,count,next,hold,metric,confirmed,far_count;
 unsigned confirmations;
 double growth_ratio,advance_m,gap_m,gap_error_m,camera_range_m,metric_world_x,metric_world_y;
 int gap_valid,metric_world_valid,clipped;
 object_visual_dock_sample samples[OBJECT_VISUAL_DOCK_SAMPLES];
 char source[48],reason[96];
} object_visual_dock;
#endif
