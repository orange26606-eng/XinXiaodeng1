#ifndef ACG_S3_AVOIDANCE_TYPES_H
#define ACG_S3_AVOIDANCE_TYPES_H
#include <stdint.h>
#define S3_HAZARD_VIEW_MAX 32u
/* 当前车体坐标，单位米，前方+X、左侧+Y；-1表示净空未知，时间为原始观测的单调微秒。 */

typedef struct s3_hazard_point { double x_m,y_m;uint64_t time_us; } s3_hazard_point;
typedef struct s3_nearfield_view {
 uint32_t valid,stale,point_count,focus_points,raw_points,held_points;
 uint64_t scan_time_us,age_us,updates,rejected_scans;
 double front_m,left_m,right_m,rear_m;
 double obstacle_x_m,obstacle_y_m;
 uint32_t front_blocked,left_blocked,right_blocked,rear_blocked;
 char reason[48];
 uint32_t hazard_count;s3_hazard_point hazards[S3_HAZARD_VIEW_MAX];
 /* 有界内存及查询诊断，原因码0表示CLEAR；字段保存实际查询结果，不由界面计数推算。 */
 uint32_t history_cells,history_recent,history_pending,history_unknown;
 uint32_t history_capacity,storage_bytes,query_code,query_source;
 uint64_t history_expired,ray_cleared,window_evicted,query_source_time_us;
 uint64_t query_time_us;
 double query_x_m,query_y_m,query_ttc_s;
 double query_v_m_s,query_w_rad_s,query_horizon_s;
 char query_reason[48];
 /* 最新有效回波端点不代表自由空间许可，上方历史front_m只覆盖车体前方1.8m走廊。 */
 uint32_t forward_returns,forward_bins,forward_corridor_returns,forward_self_returns;
 double forward_return_m,forward_corridor_m;
 uint64_t bracket_cleared;
 char forward_reason[48];
} s3_nearfield_view;
/* 阶段0空闲、1弧线、2转向、3后退、4回归、5等待。角度用有符号弧度，进度来自测量。 */

typedef struct s3_trajectory_point {double x,y,yaw,v,w,t;} s3_trajectory_point;
typedef struct s3_avoidance_view {
 uint32_t active,stage,event_id;int32_t direction;
 double turn_target_rad,turn_progress_rad,backup_target_m,backup_progress_m;
 uint32_t completed_actions,interrupted_actions;
 char reason[48];
 /* 补充策略诊断，side_state取0畅通、1较近、2很近、3紧急。 */
 uint32_t policy_revision,front_approach,nf_emergency,turn_hold,side_state[2];
 double front_clearance_m,reverse_target_m,front_corridor_clearance_m;
 double side_clearance_m[2],rejected_v_m_s,rejected_w_rad_s;
 char rejected_reason[64];
 /* 内部候选诊断字段，公开S5字段单独保留。 */
 uint32_t planner_active,traj_count,planner_rank,planner_samples,planner_vetoed;
 double planner_v_m,planner_w_rad_s,planner_score,planner_clear_min;
 double planner_v_error,planner_w_error;uint64_t costmap_age_ms,planner_generated_us,traj_mismatch_ms;
 char planner_veto_reason[64];s3_trajectory_point traj_points[30];
 /* 场景状态机的可选附加诊断字段。 */
 uint32_t scene_id,side_class,calibrated;
 uint32_t scene_owner,scene_stage,explore_round,reverse_attempts,reverse_extended_attempts;
 double scene_speed_cap;
 char scene[24],scene_reason[64];
 /* 按方向诊断，部分方向掩码不等于全局停车。 */
 uint32_t usable_direction_mask,scene_pending;
 uint64_t direction_age_us[4]; /* 前、左、右、后顺序，UINT64_MAX表示未知。 */
 /* 显示请求路线与实际发布命令的差异。 */
 double route_expected_v_m_s,route_expected_w_rad_s,route_actual_v_m_s,route_actual_w_rad_s;
 int32_t route_expected_side,route_actual_side;
 uint32_t route_side_changed;
 int32_t motion_check_result;uint64_t motion_check_time_us;
 char route_change_reason[64];
} s3_avoidance_view;
#endif
