#ifndef ACG_APPROACH_OBSERVATION_H
#define ACG_APPROACH_OBSERVATION_H
#include <stdint.h>
#include "lab_live.h"
typedef struct approach_observation {
 uint64_t epoch,frame_id,frame_us,published_us,scan_us,odom_us;
 int track_id,class_id,observed,identity_valid,direction_valid,config_error,correction_sign_error;
 int memory_valid; /* 方向和距离可使用同一锁定目标记忆，但不作为新相机观测。 */
 double bearing_rad;
 int frame_pose_valid,scan_pose_valid,lidar_bearing_valid;
 double frame_pose_x,frame_pose_y,frame_pose_yaw,scan_pose_x,scan_pose_y,scan_pose_yaw;
 double lidar_bearing_rad,lidar_side_offset_m;
 int stereo_valid,lidar_valid,lidar_confirmed,offset_verified;
 double stereo_range_m,stereo_error_m,lidar_range_m,lidar_error_m,camera_front_offset_m,lidar_front_offset_m;
 char label[64],sensor_reason[128],geometry_reason[128],stereo_reason[96],lidar_reason[96];
 uint32_t valid_lidar_points,candidate_count;
 int anchor_valid; /* 当前选择和上下文中已有确认的雷达位置。 */
 int context_valid;lab_real_context context; /* 原始连续里程计上下文。 */
 /* 异步深度保留原采集几何，零时间戳按旧格式同帧处理。 */
 uint64_t stereo_us,stereo_frame_id;int stereo_pose_valid,stereo_bearing_valid;
 double stereo_pose_x,stereo_pose_y,stereo_pose_yaw,stereo_bearing_rad;
} approach_observation;
#endif
