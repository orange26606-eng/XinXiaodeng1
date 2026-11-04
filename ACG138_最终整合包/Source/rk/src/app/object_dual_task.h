#ifndef ACG_OBJECT_DUAL_TASK_H
#define ACG_OBJECT_DUAL_TASK_H
#include "object_scene.h"
#include "object_target_v2.h"
#include "object_mode_contract.h"
#include "object_time_policy.h"
#include "object_follow_policy_v6.h"
#include "object_visual_docking.h"
#include "object_turn_smooth.h"
#define OBJECT_STATIC_SEARCH_FIRST_DEG 24.0
#define OBJECT_STATIC_SEARCH_MAX_DEG 90.0
#define OBJECT_STATIC_SEARCH_STAGES 6u
#define OBJECT_STATIC_SEARCH_MAX_RATE .20
#define OBJECT_NORMAL_TURN_MAX_RATE .30
#define OBJECT_TURN_DEADZONE_RAD .05
#define OBJECT_DIRECTION_ENTER_RAD (6.0*3.14159265358979323846/180.0)
#define OBJECT_DIRECTION_EXIT_RAD (3.0*3.14159265358979323846/180.0)
#define OBJECT_DIRECTION_TAU_S .12
#define OBJECT_DIRECTION_STRONG_RAD (15.0*3.14159265358979323846/180.0)
#define OBJECT_FOLLOW_DEFAULT_STANDOFF_M .20
#define OBJECT_FOLLOW_HOLD_HALF_M .10
#define OBJECT_FOLLOW_TRACE_MAX 128u
#define OBJECT_FOLLOW_TRACE_TTL_US UINT64_C(20000000)
#define OBJECT_FOLLOW_PREDICT_US UINT64_C(1000000)
#define OBJECT_FOLLOW_LOST_STOP_US UINT64_C(5000000)
#define OBJECT_FOLLOW_NORMAL_W OBJECT_FOLLOW_NORMAL_YAW_V6
#define OBJECT_FOLLOW_DIRECTED_W OBJECT_FOLLOW_DIRECTED_YAW_V6
#define OBJECT_FOLLOW_SEARCH_W .20
typedef struct object_dual_trace {double x,y;uint64_t source_us;} object_dual_trace;
typedef struct object_dual_task {
 int task_class_id;uint32_t binding_generation,association_revision;
 unsigned association_corrections;uint64_t association_source_us;char association_reason[96];
 object_follow_exit_v5 follow_exit_hint;double follow_exit_world_yaw;int exit_hint_used;char exit_hint_status[32];
 int directed_active,directed_sign;uint64_t directed_source_us,directed_frame_id,directed_started_us,directed_ended_us,directed_consumed_source_us;
 double directed_start_yaw,directed_goal_yaw,directed_last_yaw,directed_progress_rad,directed_world_hint;char directed_status[32];
 unsigned mode;int radar_only;int semantic_direct;int static_search,search_bound,static_bearing_estimated;char search_category[64],search_color[16],search_size[12];uint64_t static_range_revoked_us;int prepared,active,explicit_start,arrived,track_id,range_valid,bearing_valid,have_target,have_context,waiting_distance;
 uint64_t epoch,request_id,started_us,last_visual_us,last_source_us,last_tick_us,arrival_source_us,generation;
 obj_turn_measure turn_measure;uint64_t static_endpoint_hold_us,fallback_hold_us;int fallback_active,fallback_leg,fallback_sign;double fallback_center_yaw;
 int static_lost;uint64_t static_lost_us,static_lost_visual_us;double static_search_yaw;unsigned static_search_stage,static_search_leg,static_search_rounds;int static_search_sign;
 uint64_t last_hint_us;double hint_world_yaw;
 unsigned static_relocation_support;uint64_t static_relocation_us;double static_relocation_x,static_relocation_y,visual_world_yaw;
 object_dual_trace follow_trace[OBJECT_FOLLOW_TRACE_MAX];unsigned follow_trace_count,follow_trace_cursor;
 int current_visual_valid;uint64_t current_visual_us;
 double follow_v,follow_goal_x,follow_goal_y;uint64_t follow_control_us;
 object_target_observation_v2 observation_v2;
 uint64_t observation_seen_us,near_latch_us;unsigned near_latch,near_clear_count;
 int probe_active,probe_current,probe_track_id,probe_eye;uint64_t probe_epoch,probe_frame_id,probe_source_us;double probe_bearing_rad,probe_world_yaw;char probe_reason[96];
 unsigned search_class_candidates,search_color_candidates,search_metric_candidates;
 int near_hold,near_metric;char near_reason[96];
 object_visual_geometry visual_geometry;object_visual_dock visual_dock;
 unsigned arrival_count;lab_real_context context;
 double standoff_m,front_m,range_m,bearing_rad,target_x,target_y,target_vx,target_vy,previous_x,previous_y,last_visual_yaw;
 int direction_initialized,direction_turning,hint_suppressed_for_memory,velocity_jump_suppressed;
 unsigned velocity_jump_count;uint64_t direction_time_us,direction_source_us,direction_visual_us,direction_log_us,velocity_anchor_us;
 double direction_world_yaw,direction_raw_rad,direction_filtered_rad,velocity_anchor_x,velocity_anchor_y;
 char direction_source[32];
 double requested_v,requested_w,search_center_yaw;char state[32],reason[96],message_zh[256],source[48];
} object_dual_task;
void object_dual_task_init(object_dual_task*);
int object_dual_task_prepare(object_dual_task*,unsigned mode,uint64_t epoch,int track_id,double standoff_m,uint64_t request_id);
int object_dual_task_prepare_search(object_dual_task*,const char*category,double standoff_m,uint64_t request_id);
int object_dual_task_prepare_search_attributes(object_dual_task*,const char*category,const char*color,const char*size,double standoff_m,uint64_t request_id);
/* 仅加载实测标定，无效或未核验几何明确标为不可用。 */
int object_dual_task_geometry_load(object_dual_task*,const char*objects_config);
int object_dual_task_prepare_radar(object_dual_task*,uint64_t radar_epoch,double standoff_m,uint64_t request_id);
int object_dual_task_start(object_dual_task*,unsigned mode,uint64_t epoch,int track_id,uint64_t now_us);
void object_dual_task_pause(object_dual_task*,const char*reason);
void object_dual_task_cancel(object_dual_task*,const char*reason);
/* 仅显式活动任务返回1，包括零距离保持。此处只提出期望运动，最终几何和租约条件由R8运行对象管理。
 */
int object_dual_task_tick(object_dual_task*,const obj_scene*,const obj_frame*,const lab_real_runtime_view*,uint64_t,lab_real_command*);
int object_dual_task_route_v2(const object_dual_task*,const lab_real_runtime_view*,uint64_t,object_route_target_v2*);
int object_dual_task_json(const object_dual_task*,uint64_t,char*,size_t,size_t*);
#endif
