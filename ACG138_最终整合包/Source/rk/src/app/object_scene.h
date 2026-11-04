#ifndef ACG_OBJECT_SCENE_H
#define ACG_OBJECT_SCENE_H
#include "object_readonly.h"
#include "object_scene_input.h"
#include "lab_real_runtime.h"
#include "object_target_v2.h"
#define OBJ_SCENE_PATH_MAX 512u
#define OBJ_SCENE_CLOUD_MAX 2048u
#define OBJ_SCENE_JSON_MAX (256u*1024u)
typedef struct obj_scene_config {double camera_ahead_lidar_m,standoff_m,range_max_m;uint64_t coarse_max_age_us;int mount_approximate;obj_mount camera;int mount_configured;unsigned mode;} obj_scene_config;
typedef struct obj_scene {
 uint64_t epoch,map_id,map_version,published_us,source_us,frame_id,scan_us;
 int track_id,target_valid,dock_valid,approximate,confirmed,bearing_valid,historical;
 double target_x,target_y,range_m,error_m,bearing_rad,dock_x,dock_y,dock_yaw,standoff_m;
 char source[48],reason[96],message_zh[256],preview_status[24],preview_reason[96];
 uint32_t path_count,cloud_count;double path[OBJ_SCENE_PATH_MAX][2],cloud[OBJ_SCENE_CLOUD_MAX][3];
 uint64_t plan_us,plan_version,plan_elapsed_us;unsigned plan_expanded;double plan_goal_x,plan_goal_y;
 /* 融合记忆保留来源身份和时间，重复读取只返回已有支持。 */
 lab_real_context context;int context_valid,target_odom_valid;double target_odom_x,target_odom_y;
 int cloud_pose_valid;double cloud_pose[3];uint64_t cloud_sequence;
 struct {uint64_t time_us;uint32_t session,sequence,count;double odom_x,odom_y,odom_yaw,points[512][2];int pose_valid;} cloud_history[3];unsigned cloud_next;
 int scale_valid,scale_reference_eye;double scale_range_m,scale_reference_range,scale_reference_area,scale_reference_aspect,scale_reference_yaw;uint64_t scale_reference_us;
 uint64_t support_source_us;unsigned support;double previous_x,previous_y;
 object_target_observation_v2 measurement_v2;
 /* 每个规范目标保存有界雷达轨迹，仅在新扫描到达时更新。 */
 uint64_t lidar_source_us,lidar_visual_us;
 int lidar_valid,lidar_support_valid,stereo_prior_only,stereo_conflict;
 unsigned lidar_updates,lidar_candidates;
 double lidar_x,lidar_y,lidar_vx,lidar_vy,lidar_error_m;
 char preview_kind[40],preview_source_frame[8];
 uint32_t binding_generation;
 uint64_t lidar_visual_frame;
 uint32_t association_revision;
 double follow_wall_score,follow_motion_score,follow_isolation_score;
 unsigned follow_candidates,follow_wall_rejected,follow_motion_support;
 char follow_reason[96];
 unsigned follow_sparse,follow_support_points;double follow_search_half_rad;
 uint64_t obstacle_source_us;unsigned obstacle_raw_scan_points,obstacle_front_points,obstacle_left_points,obstacle_right_points,obstacle_rear_points;
 /* 主程序持有的不透明有界分类与轨迹历史，随场景重置。 */
 union {uint64_t alignment;unsigned char bytes[24576];} follow_state_v5;
} obj_scene;
typedef struct obj_scene_candidate {uint64_t frame_id,image_time_us,source_us,scan_us;int track_id,class_id,eye;double bbox[4],bearing_rad,range_m,odom_x_m,odom_y_m,error_m;} obj_scene_candidate;
void obj_scene_config_default(obj_scene_config*);
int obj_scene_config_load_mount(obj_scene_config*,const char*);
void obj_scene_camera_origin(const obj_scene_config*,const passage_config*,double[3]);
void obj_scene_camera_vector(const obj_scene_config*,const double[3],double[3]);
void obj_scene_fuse(obj_scene*,const obj_scene_config*,const obj_frame*,const approach_observation*,const obj_radar_anchor_view*,const lab_real_runtime_view*,const passage_config*,uint64_t);
void obj_scene_plan(obj_scene*,const lab_real_runtime_view*,const passage_config*,uint64_t);
void obj_scene_cloud(obj_scene*,const lab_real_runtime_view*,const passage_config*,uint64_t);
int obj_scene_json(const obj_scene*,char*,size_t,size_t*);
/* 仅提供历史关联证据，各候选分别保留真实相机和扫描来源，不构造当前观测。
 */
size_t obj_scene_candidates(const obj_scene_config*,const obj_frame*,const approach_observation*,const lab_real_runtime_view*,const passage_config*,uint64_t,obj_scene_candidate*,size_t);
#endif
