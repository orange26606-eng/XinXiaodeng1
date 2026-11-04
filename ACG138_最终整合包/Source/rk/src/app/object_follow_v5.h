#ifndef ACG_OBJECT_FOLLOW_V5_H
#define ACG_OBJECT_FOLLOW_V5_H
#include <stdint.h>
#define OBJECT_FOLLOW_KEYFRAMES_V5_MAX 8u
typedef struct object_follow_keyframe_v5 {uint64_t frame_id,source_us;int eye,bearing_valid,ray_valid;double bbox[4],bearing_rad,ray_direction[3];} object_follow_keyframe_v5;
typedef struct object_follow_exit_v5 {
 int valid,bearing_valid,direction,kind,eye,compensated;uint32_t count;
 uint64_t source_us,frame_id;double bearing_rad,angular_velocity_rad_s,confidence;
 char reason[96];object_follow_keyframe_v5 keyframes[OBJECT_FOLLOW_KEYFRAMES_V5_MAX];
} object_follow_exit_v5;
#endif
