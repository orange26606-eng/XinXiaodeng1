#ifndef ACG_PASSAGE_H
#define ACG_PASSAGE_H
#include <stdint.h>
#include <stddef.h>
#define PASSAGE_LOCAL_SIDE 240u
#define PASSAGE_LOCAL_CELLS (PASSAGE_LOCAL_SIDE*PASSAGE_LOCAL_SIDE)
#define PASSAGE_PATH_MAX 4096u
#define PASSAGE_WEDGE_RAYS_MAX 2048u
typedef struct passage_config {
 double length_m,width_m,margin_m,normal_speed_m_s,narrow_speed_m_s,narrow_width_m;
 double local_resolution_m,mount_x_m,mount_y_m,mount_yaw_rad;
 double lidar_min_m,lidar_max_m;
 uint32_t mount_verified;uint64_t local_ttl_us;
 uint32_t continuous;
 /* 带符号车体净空偏置沿用既定实现。 */
 double sonar_front_offset_m,sonar_side_offset_m,sonar_beam_margin_m,wheel_outer_margin_m;
 double front_approach_m,front_corridor_m;
 uint32_t sonar_offsets_verified,wheel_outer_margin_verified;
 /* S6规划参数，加减速度仍为未标定假设。 */
 double planner_cycle_ms,planner_horizon_s,planner_dt_ms,costmap_lethal_m,costmap_inflation_m;
 double planner_w_heading,planner_w_path,planner_w_clearance,planner_w_smoothness,planner_w_velocity;
 double a_lin_m_s2,a_yaw_rad_s2;
 /* 标定标记不参与运动门控；不启用卡死、打滑检测参数。 */
 double r4_enabled,side_straight_emergency_mm,side_straight_crawl_mm,side_straight_observe_mm,w_thresh_straight_rad_s,band_up_hyst,band_up_hold_ms,scene_confirm_ms,scene_exit_ms,wall_band_min_mm,wall_band_max_mm,corridor_max_mm,doorway_len_max_m,doorway_time_max_ms,explore_turn_step_deg,explore_side_budget_deg,explore_fwd_min_mm,explore_probe_fast_m_s,explore_probe_crawl_m_s,explore_progress_mm,explore_progress_s,explore_rounds,reverse_attempts,extended_reverse_attempts,calibrated,front_lethal_from_edge_m;
} passage_config;
typedef struct passage_pose {double x,y,yaw;} passage_pose;
typedef struct passage_xy {double x,y;} passage_xy;
typedef struct passage_grid {unsigned width,height;double resolution,origin_x,origin_y;const int8_t*cells;} passage_grid;
typedef struct passage_local {
 int8_t score[PASSAGE_LOCAL_CELLS];uint64_t stamp[PASSAGE_LOCAL_CELLS];uint32_t sequence[PASSAGE_LOCAL_CELLS];
 double origin_x,origin_y,resolution;uint64_t last_scan;uint32_t scan_sequence;int initialized;
 uint64_t rays,hits,invalid;
 uint8_t hit_count[PASSAGE_LOCAL_CELLS];uint32_t hit_sequence[PASSAGE_LOCAL_CELLS];uint64_t hit_stamp[PASSAGE_LOCAL_CELLS];
 uint8_t hit_history[PASSAGE_LOCAL_CELLS];
 uint64_t rolls,roll_cleared,reframes,reframe_resets;
 uint64_t clear_stamp[PASSAGE_LOCAL_CELLS];
 /* 端点连续表示，避免微小坐标校正被反复取整。 */
 float hit_point_x[PASSAGE_LOCAL_CELLS],hit_point_y[PASSAGE_LOCAL_CELLS];
 /* 连续局部坐标转换到校正地图坐标，栅格本身不旋转。 */
 double map_from_local_x,map_from_local_y,map_from_local_yaw;
 double beam_prev_sx,beam_prev_sy,beam_prev_angle,beam_prev_range;
 double beam_first_sx,beam_first_sy,beam_first_angle,beam_first_range;
 uint32_t beam_entries,beam_prev_valid,beam_first_valid,scan_finished;
 uint32_t scan_wedge_rays;
 uint64_t wedges,wedge_rays;
} passage_local;
enum passage_reason {PASSAGE_CLEAR=0,PASSAGE_OCCUPIED=1,PASSAGE_UNKNOWN=2,PASSAGE_OUTSIDE=3,PASSAGE_INVALID=4,PASSAGE_SEARCH_LIMIT=5,PASSAGE_NO_PATH=6,PASSAGE_OBSERVED_PROBE=7,PASSAGE_TENTATIVE=8,PASSAGE_PARTIAL=9};
typedef struct passage_world {
 passage_config config;passage_grid global;const passage_local*local;uint64_t now_us;
 passage_pose initial;int allow_initial_unknown;
 int confirmed_only;
 int (*cancelled)(void*);void*cancel_user;
 /* 仅使用当前局部感知，未知或过期时不退回旧地图。 */
 uint32_t local_only;
 /* 暂定路线不授予未观测空间的实际运动许可。 */
 uint32_t observed_probe,planning_unknown_soft;
 double probe_unknown_fraction,probe_unknown_margin_m;
} passage_world;
typedef struct passage_result {int reason,used_initial_unknown;unsigned cells_checked;double x,y;unsigned observed_cells,unknown_cells,occupied_cells,stale_cells;double observed_fraction;uint32_t used_probe;} passage_result;
void passage_config_default(passage_config*c);
int passage_config_valid(const passage_config*c);
int passage_config_load(passage_config*c,const char*path,char*error,size_t cap);
void passage_local_reset(passage_local*l);
/* 只组合map_from_local，耗时O(1)，不改证据或时间戳。 */
int passage_local_reframe(passage_local*l,const passage_config*c,double dx,double dy,double yaw);
/* 仅完整有效新扫描调用begin，重复序号不清格。 */
int passage_local_begin(passage_local*l,const passage_config*c,passage_pose base,uint32_t sequence,uint64_t time_us);
void passage_local_ray(passage_local*l,const passage_config*c,passage_pose scan_base,double angle_rad,double distance_m,unsigned quality,uint32_t sequence,uint64_t time_us);
void passage_local_scan_finish(passage_local*l,const passage_config*c,uint32_t sequence,uint64_t time_us);
int passage_pose_clear(const passage_world*w,passage_pose p,double width_m,passage_result*out);
int passage_translate_clear(const passage_world*w,passage_pose a,passage_xy b,double width_m,passage_result*out);
int passage_rotate_clear(const passage_world*w,passage_pose a,double target_yaw,passage_result*out);
int passage_arc_clear(const passage_world*w,passage_pose a,double v,double omega,double horizon_s,passage_result*out);
/* 粗搜索样本仅作提示，最终路径段和弯道使用精确车体检查。 */
int passage_planning_sample(const passage_world*,double x,double y);
enum passage_route_kind {PASSAGE_ROUTE_NONE=0,PASSAGE_ROUTE_DIRECT=1,PASSAGE_ROUTE_CORRIDOR=2,PASSAGE_ROUTE_LOCAL_POSE=3,PASSAGE_ROUTE_PARTIAL=4};
typedef struct passage_plan_report {unsigned kind,complete,partial,budget;int reason;unsigned corridor_expanded,pose_expanded;} passage_plan_report;
int passage_plan_ex(const passage_world*,passage_pose,passage_xy,int,double,passage_xy*,unsigned,unsigned*,unsigned*,passage_plan_report*);
/* 有界带朝向回退搜索，不访问电机或传感器。 */
int passage_plan(const passage_world*w,passage_pose start,passage_xy goal,int has_goal_yaw,double goal_yaw,passage_xy*out,unsigned cap,unsigned*count,unsigned*expanded);
const char*passage_reason_text(int reason);
#endif
