#ifndef ACG_R4_SCENE_TYPES_H
#define ACG_R4_SCENE_TYPES_H
#include <stdint.h>
#include "r6_motion_policy.h"
#include "continuous_policy.h"

/* 距离统一为车身边缘净空（米），不是雷达/探头量程。
 * 原始量程0必须由适配器忽略或标RAW_ZERO；几何净空0且FRESH则是真实接触证据。 */
enum r4_direction_index {R4_FRONT=0,R4_LEFT=1,R4_RIGHT=2,R4_REAR=3};
enum r4_quality {R4_FRESH=1u,R4_CONTRADICTORY=2u,R4_SINGLE_SOURCE=4u,R4_INVALID=8u,R4_RAW_ZERO=16u};
enum r4_scene {R4_OPEN=0,R4_FRONT_NEAR=1,R4_WALL_LEFT=2,R4_CORRIDOR=3,
 R4_CORNER=4,R4_DEAD_END=5,R4_DOORWAY=6,R4_LOW_OBSTACLE=7,R4_GLASS=8,
 R4_OVERHANG=9,R4_RETREAT=10,R4_WALL_RIGHT=12,R4_FALLBACK=13,R4_GENERAL=14};
/* 不启用STUCK场景及卡死、打滑恢复链。 */
enum r4_owner {R4_OWNER_ROUTE=0,R4_OWNER_ESTOP=1,R4_OWNER_BRAKE=2,
 R4_OWNER_NEAR=3,R4_OWNER_COMMITTED=4,R4_OWNER_FALLBACK=5,R4_OWNER_WAIT=6};
enum r4_stage {R4_STAGE_IDLE=0,R4_CHECK_REAR=1,R4_REVERSE=2,R4_EXIT_TURN=3,
 R4_TURN_HOLD=4,R4_GAP_MEASURE=11,R4_PROBE_TURN=12,R4_PROBE_FORWARD=13,
 R4_EXPLORE_SUCCESS=14,R4_EXPLORE_GIVE_UP=15,R4_HOLD=16};
typedef struct r4_direction {double clearance_m;uint64_t source_us;uint32_t sequence,quality;uint64_t max_age_us;} r4_direction;
/* Typed r6_check_result。必须明确==R6_CHECK_CLEAR，任何负值都不是成功。 */
typedef int (*r4_motion_check)(void*context,double v,double w,double horizon_s);
typedef struct r4_scene_config {
 uint64_t direction_max_age_us,scene_confirm_us,scene_exit_us,band_up_hold_us;
 double band_up_hyst,w_straight,body_length_m,body_width_m,body_pad_m;
 double front_brake_m,side_straight_m,side_turn_m,side_release_m;
 double reverse_speed_m_s,reverse_target_m,reverse_hard_m,reverse_extended_m,reverse_extended_hard_m;
 uint64_t reverse_max_us,reverse_extended_max_us;
 uint32_t reverse_attempts,reverse_extended_attempts,explore_rounds;
 double pivot_w,explore_step_rad,explore_side_budget_rad,explore_fast_m_s,explore_crawl_m_s;
 uint64_t explore_window_us,switch_hold_us;double explore_progress_m,switch_score_ratio;
 double doorway_length_m;uint64_t doorway_max_us;uint32_t continuous_exploration;
} r4_scene_config;
typedef struct r4_scene_input {
 uint64_t now_us,mission_id;r4_direction direction[4];
 double x_m,y_m,yaw_rad,measured_v,measured_w,desired_v,desired_w,goal_bearing_rad;
 uint32_t pose_valid,manual,estop,reset_estop,goal_active,execution_ready;
 uint32_t low_obstacle,glass,overhang,glass_clear_confirmed,committed_action;
 r4_motion_check motion_check;void*motion_context;uint32_t cruise_active;
 int*check_result;uint32_t have_speed_limit;double speed_limit_m_s;
 uint64_t pose_source_us; /* 姿态原始时间戳，保留采集时间，不使用控制循环的 now。 */
} r4_scene_input;
typedef struct r4_scene_output {
 uint32_t scene,owner,stage,owns_motion,planner_allowed,straight_model,emergency,transition;
 double v,w,speed_cap;uint32_t explore_round,reverse_attempts,reverse_extended_attempts;
 const char*reason;uint32_t corner_observed;
} r4_scene_output;
typedef struct r4_scene_state {
 uint32_t initialized,scene,candidate_scene,stage,estop_latched,front_brake,front_clears;
 uint32_t side_hits[2],side_brake[2],hold_clears,hold_seen;
 uint64_t side_first_us[2],side_last_source[2],front_source,hold_source,front_brake_since;
 uint32_t side_seq[2],front_seq,hold_seq;
 uint64_t scene_since,candidate_since,band_since,band_last_sample,doorway_since,mission_id;
 double speed_cap,band_target,doorway_x,doorway_y;
 uint32_t doorway_seen,explore_round,reverse_attempts,reverse_extended_attempts,reverse_counted;
 uint32_t turn_side,attempted_sides,post_reverse,extended,phase_entered,turn_seen,explore_started;
 double phase_x,phase_y,last_x,last_y,last_yaw,phase_yaw,turn_progress,side_angle[2],reverse_progress;
 double explore_x,explore_y,previous_front;
 uint64_t phase_since,execution_since,last_step_us,explore_since,side_selected_us,open_since;
 uint32_t resume_limited,recover_clears,recover_seq;uint64_t recover_source,hold_retry_source,hold_retry_us;
 uint64_t explore_active_us,execution_step_us;uint32_t round_counted;uint64_t side_brake_since[2];
 uint64_t front_trend_source;uint32_t front_trend_seq,front_approach_count,corner_observed;double fitted_target_m;
 uint32_t side_escape_active;uint64_t side_escape_brake_us;
 uint64_t last_motion_source;double executed_turn_angle[2];
} r4_scene_state;
static void r4_scene_config_default(r4_scene_config*);
static void r4_scene_init(r4_scene_state*);
static int r4_direction_usable(const r4_direction*,uint64_t,uint64_t);
static uint32_t r4_scene_classify(const r4_scene_input*,const r4_scene_config*,r4_scene_state*);
static void r4_scene_step(r4_scene_state*,const r4_scene_config*,const r4_scene_input*,r4_scene_output*);
static double r4_action_speed_cap(const r4_scene_state*,const r4_scene_input*,double,double,double);
static const char*r4_scene_name(uint32_t);
#endif
