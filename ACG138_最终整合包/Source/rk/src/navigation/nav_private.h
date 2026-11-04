#ifndef ACG_NAV_PRIVATE_H
#define ACG_NAV_PRIVATE_H
/* 导航模块内部接口，公开 ABI 见 include/，沿用 v1 数据格式。此层不直接访问串口、套接字、FPGA 或电机。 */
#include "lab_mapping.h"
#include "passage.h"
#include "s3_avoidance_types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define NAV_SIDE_MAX LAB_MAP_SIDE_MAX
#define NAV_GRID_MAX (NAV_SIDE_MAX * NAV_SIDE_MAX)
#define NAV_PATH_MAX 4096u
typedef struct nav_engine nav_engine;
void nav_s6_set_enabled(nav_engine*,uint32_t enabled); /* 回归和显式集成控制接口，不具电机权限。 */
void nav_set_time_uncertainty(nav_engine *n,double seconds);
void nav_set_chassis_hint(nav_engine*n,uint64_t stamp,const uint16_t mm[3],uint8_t valid,uint8_t fresh,uint8_t near,uint8_t degraded,uint8_t stable);
void nav_set_chassis_hint_s1(nav_engine*n,uint64_t stamp,const uint16_t mm[3],uint8_t valid,uint8_t fresh,uint8_t near,uint8_t degraded,uint8_t stable,uint8_t pending);
void nav_set_chassis_raw_s5(nav_engine*,uint64_t diagnostic_rx_us,uint32_t fpga_uptime_ms,const uint16_t raw_mm[3],const uint32_t nonzero_seq[3],const uint32_t raw_age_us[3],uint8_t available_mask);
void nav_set_chassis_samples_s5(nav_engine*,uint64_t stamp,uint8_t front_sequence,uint8_t sequence_valid,uint8_t raw_zero_mask);
void nav_set_execution_hint(nav_engine*n,uint32_t armed);
void nav_set_execution_feedback(nav_engine*n,uint32_t armed,const double rpm[4]);
void nav_reframe_local(nav_engine*n,double dx,double dy,double yaw);
void nav_set_cruise(nav_engine*n,uint32_t enabled);
void nav_observe_nearfield(nav_engine*,const lab_event*,uint64_t);
void nav_nearfield_view(const nav_engine*,uint64_t,s3_nearfield_view*);
int nav_nearfield_allows(nav_engine*,uint64_t,double,double);
lab_status nav_resume_after_pause(nav_engine*n,uint64_t now_us);
int nav_direction_allows(const nav_engine*n,uint64_t now_us,double v,double w);
typedef struct nav_xy { double x_m, y_m; } nav_xy;
typedef enum nav_state {
    NAV_IDLE=0, NAV_TRACKING, NAV_WAITING, NAV_REACHED, NAV_CANCELLED,
    NAV_UNREACHABLE, NAV_FAILED
} nav_state;
typedef enum nav_reason {
    NAV_REASON_NONE=0, NAV_NO_GOAL, NAV_GOAL_REACHED, NAV_USER_CANCEL,
    NAV_NO_MAP, NAV_BAD_CONTEXT, NAV_INVALID_POSE, NAV_POSE_STALE,
    NAV_SCAN_STALE, NAV_SCAN_INVALID, NAV_GOAL_OUTSIDE, NAV_GOAL_BLOCKED,
    NAV_START_BLOCKED, NAV_NO_PATH, NAV_SEARCH_LIMIT, NAV_PATH_LIMIT,
    NAV_MAP_CHANGED, NAV_SWEEP_BLOCKED, NAV_OBSTACLE, NAV_WAIT_TIMEOUT,
    NAV_GOAL_TIMEOUT, NAV_CLOCK_ERROR, NAV_INVALID_INPUT, NAV_FINAL_YAW,
    NAV_LOCAL_AVOID, NAV_REPLANNING, NAV_TURN_RECOVERY, NAV_BACK_CHECK,
    NAV_BACKING, NAV_LOW_RECOVERY, NAV_REAR_BLOCKED, NAV_REAR_UNOBSERVED,
    NAV_RECOVERY_EXHAUSTED,NAV_LOCAL_PROBE,NAV_CRUISE,NAV_RECOVERY_RETRY_WAIT,
    NAV_REAR_SCAN_STALE,NAV_REAR_COVERAGE,NAV_REAR_SONAR_STALE,NAV_REAR_BODY_OCCUPIED,NAV_REAR_GEOMETRY_UNKNOWN,NAV_REAR_CLEARANCE,NAV_COMPUTE_WAIT,
    NAV_NEARFIELD_OBSERVATION,NAV_NEARFIELD_STALE,NAV_NEARFIELD_POSE,NAV_NEARFIELD_INVALID,NAV_NO_EXECUTABLE_ACTION,NAV_FRONT_APPROACH,NAV_NF_EMERGENCY,NAV_NEAR_AVOID,NAV_TURN_HOLD,NAV_RECOVERY_WAIT,NAV_SIDE_ACTION_REJECT,NAV_PATH_ACTION_MISMATCH
} nav_reason;

typedef enum nav_phase {
    NAV_PHASE_TRACK=0,NAV_PHASE_AVOID,NAV_PHASE_REPLAN,NAV_PHASE_TURN,
    NAV_PHASE_BACK_CHECK,NAV_PHASE_BACKUP,NAV_PHASE_RECOVER,
    NAV_PHASE_WAIT,NAV_PHASE_BLOCKED,NAV_PHASE_DETOUR_LEFT,NAV_PHASE_DETOUR_RIGHT,NAV_PHASE_ALIGN,NAV_PHASE_REJOIN,NAV_PHASE_PROBE
} nav_phase;
typedef struct nav_config {
    double vehicle_width_m, vehicle_length_m, safety_margin_m;
    double max_speed_m_s, max_yaw_rate_rad_s, acceleration_m_s2,
           angular_acceleration_rad_s2, brake_m_s2;
    double lookahead_m, goal_tolerance_m, yaw_tolerance_rad;
    double mount_x_m, mount_y_m, mount_yaw_rad;
    uint64_t max_pose_age_us, max_scan_age_us, command_valid_us,
             replan_interval_us, wait_timeout_us, goal_timeout_us;
    uint32_t occupancy_threshold, max_expansions, require_scan;
    double lidar_to_rear_m; /* 沿车体X轴，雷达中心到实际后边缘的距离。 */
    uint32_t enable_recovery;
    uint32_t rectangular_footprint;
    passage_config passage;
} nav_config;
typedef struct nav_goal {
    uint32_t id, has_yaw;
    double x_m, y_m, yaw_rad;
} nav_goal;
typedef struct nav_command {
    double linear_m_s, angular_rad_s;
    uint64_t time_us, expires_us;
    uint32_t sequence;
    lab_context context;
    uint32_t motion_intent;
} nav_command;
typedef struct nav_view {
    nav_state state;
    nav_reason reason;
    nav_reason blocking_reason; /* WAIT_TIMEOUT保留最初原因。 */
    nav_goal goal;
    nav_command command;
    uint64_t map_revision, plans, expanded_nodes, emergency_stops;
    double goal_distance_m, footprint_radius_m;
    uint32_t path_count;
    uint32_t local_path_count;const nav_xy*local_path; /* 实际选定动作，不是参考路线。 */
    const nav_xy *path; /* 借用内存在下次修改前有效。 */
    const uint8_t *blocked; /* 膨胀图与已接受地图尺寸相同。 */
    uint32_t width, height;
    double resolution_m, origin_x_m, origin_y_m;
    size_t allocated_bytes;
    uint32_t phase,reverse_attempts,clear_scans;
    double rear_clearance_m,backup_progress_m,backup_total_m;
    uint32_t dynamic_count;
    nav_xy dynamic_obstacles[32];
    uint32_t narrow_passage,passage_reason,lidar_offset_verified;
    double vehicle_width_m,vehicle_length_m,side_margin_m,local_resolution_m;
    uint32_t cruise_mode,local_probe,can_start_local,backup_requests;
    uint32_t provisional_route,candidate_sonar_rejections,candidate_occupied_rejections,candidate_unknown_rejections,candidate_accepted;
    double observed_fraction;char rear_reason[32];
    uint32_t extended_reverse_attempts,extended_reverse_active;double backup_target_m;
    uint32_t route_complete,route_partial,route_search_budget,route_kind;char route_reason[48];uint32_t candidate_budget_rejections;
 s3_nearfield_view nearfield;
 s3_avoidance_view avoidance;
} nav_view;
const char *nav_phase_text(uint32_t phase);
void nav_config_default(nav_config *out);
lab_status nav_open(nav_engine **out,const nav_config *config,const lab_context *context);
/* 复制栅格，内容变化时修订号需增加，拒绝相同修订；未知及地图外按阻挡处理。
 */
lab_status nav_set_map(nav_engine *nav,const lab_map_view *map,uint64_t revision);
lab_status nav_set_goal(nav_engine *nav,const nav_goal *goal,uint64_t now_us);
void nav_cancel(nav_engine *nav,uint64_t now_us);
/* scan_pose须为同上下文中扫描时刻的插值位姿，避免旧射线套用当前位姿。输入同步读取，不保留指针，失败输出零命令；require_scan=0只供地图测试。


 */
lab_status nav_step(nav_engine *nav,uint64_t now_us,const lab_event *current_pose,
                    const lab_event *scan,const lab_event *scan_pose,nav_command *out);
lab_status nav_get_view(const nav_engine *nav,nav_view *out);
/* 发布前按当前真实位姿和扫描年龄重新核对新决策，缓存命令保留原日期，不推进目标或控制状态。
 */
int nav_action_validate(nav_engine*,uint64_t,const lab_pose*,uint64_t,double,double);
unsigned nav_action_intent(const nav_engine*,double,double);
void nav_invalidate_command(nav_engine*);
void nav_command_published(nav_engine*,uint64_t,uint64_t);
void nav_reduce_load(nav_engine*,uint32_t);
void nav_profile_log(nav_engine*,uint64_t);
const char *nav_state_text(nav_state state);
const char *nav_reason_text(nav_reason reason);
void nav_close(nav_engine *nav);
#ifdef __cplusplus
}
#endif
#endif
