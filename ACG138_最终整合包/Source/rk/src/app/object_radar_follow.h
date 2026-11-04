#ifndef ACG_OBJECT_RADAR_FOLLOW_H
#define ACG_OBJECT_RADAR_FOLLOW_H
#include "object_scene.h"
#define OBJ_RADAR_FOLLOW_BYTES 32768u
typedef union {uint64_t alignment;unsigned char bytes[OBJ_RADAR_FOLLOW_BYTES];} obj_radar_follow;
/* ODOM提示归调用方已有任务身份，源日期保持历史值，提示不当作新距离；independent=1表示独立全周雷达任务，0表示选中视觉身份。

 */
typedef struct obj_radar_follow_hint {
 int independent,position_valid;double odom_x,odom_y,error;
 uint64_t position_source_us;
 int bearing_valid;double world_yaw;uint64_t bearing_source_us;
 /* 可选扫描时刻ODOM由原始位姿观测器给出两端包围插值，要求准确扫描时间，不外推。
 */
 int scan_pose_valid;uint64_t scan_pose_time_us;
 double scan_pose_x,scan_pose_y,scan_pose_yaw,scan_pose_error_m;
 uint64_t scan_pose_error_us;
} obj_radar_follow_hint;
/* 纯感知，不依赖相机身份或地图，不调用执行器；只有新接受的真实扫描观测返回1。
 */
int obj_radar_follow_update(obj_radar_follow*,obj_scene*,const obj_scene_config*,const lab_real_runtime_view*,const passage_config*,uint64_t now,uint64_t task_epoch,int canonical_track,uint32_t binding_generation,const obj_radar_follow_hint*);
void obj_radar_follow_reset(obj_radar_follow*);
#endif
