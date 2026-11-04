#ifndef ACG_OBJECT_TARGET_V2_H
#define ACG_OBJECT_TARGET_V2_H
#include "lab_live.h"
#include <stdint.h>
#define OBJECT_TARGET_V2_VERSION 1u
#define OBJECT_ROUTE_V2_MAX 128u
enum object_measurement_kind_v2 { OBJECT_MEASURE_NONE=0, OBJECT_MEASURE_RADAR_VISUAL=1, OBJECT_MEASURE_RADAR_STEREO=2, OBJECT_MEASURE_STEREO_COARSE=3, OBJECT_MEASURE_RADAR_ONLY=4 };
/* 当前配对测量不等于保留或绘制估计，所有x、y使用连续原始ODOM，body_gap相对实际车头边缘。
 */
typedef struct object_target_observation_v2 {
 uint32_t valid,confirmed,surface_valid,mount_verified,kind;
 lab_real_context context;
 uint64_t epoch,source_us,image_us,scan_us,frame_id;
 int track_id;
 double odom_x_m,odom_y_m,body_x_m,body_y_m,surface_gap_m,error_m,bearing_rad;
 char source[48],reason[96];
 uint32_t binding_generation;
 uint32_t association_revision;
 uint32_t support_points,sparse;
} object_target_observation_v2;
typedef struct object_route_point_v2 { double x_m,y_m;uint64_t source_us; } object_route_point_v2;
/* 内部目标传递结构，不作为线路目标报文或地图位姿。valid=0 表示仅有方向；zero_required 在保持状态下禁止路线绕行输出。 */
typedef struct object_route_target_v2 {
 uint32_t valid,mode,goal_valid,zero_required,trail_count;
 lab_real_context context;
 uint64_t epoch,generation,source_us,published_us;
 int track_id;
 double target_x_m,target_y_m,goal_x_m,goal_y_m,standoff_m,error_m;
 object_route_point_v2 trail[OBJECT_ROUTE_V2_MAX];
 uint32_t visual_bearing_valid;
 double visual_bearing_rad;
 uint64_t visual_source_us;
 /* 持续任务方向可来自视觉、雷达或ODOM记忆，但不能标作新视觉观测。
 */
 uint32_t control_bearing_valid,control_turning;
 double control_bearing_rad;uint64_t control_source_us;
 /* 仅表示静止任务意图，两标记都不证明已测得目标坐标。 */
 uint32_t static_search_left,static_bearing_approach;
} object_route_target_v2;
#endif
