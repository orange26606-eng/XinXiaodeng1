#ifndef ACG_OBJECT_SCENE_INPUT_H
#define ACG_OBJECT_SCENE_INPUT_H
#include "lab_real_runtime.h"
#include "passage.h"
#include "approach_observation.h"
#define OBJ_RADAR_EVENT_MAX 16u
typedef struct obj_radar_event {uint64_t time_us;unsigned cluster_id;int pass,update;double maha,sigma;char reason[64];}obj_radar_event;
typedef struct obj_radar_anchor_view {int valid;double odom_x,odom_y,range_m;uint64_t observed_us;unsigned candidate_support,event_count;obj_radar_event events[OBJ_RADAR_EVENT_MAX];double initial_x,initial_y;uint64_t initial_us;}obj_radar_anchor_view;

#endif
