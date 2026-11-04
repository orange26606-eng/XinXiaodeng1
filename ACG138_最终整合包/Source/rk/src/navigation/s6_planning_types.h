#ifndef ACG_S6_PLANNING_TYPES_H
#define ACG_S6_PLANNING_TYPES_H
#include <stdint.h>
#include <stddef.h>
#define S6_CM_SIDE 200u
#define S6_CM_CELLS (S6_CM_SIDE*S6_CM_SIDE)
#define S6_TRAJ_MAX 30u
#define S6_SAMPLES_MAX 64u
typedef struct s6_pose {double x,y,yaw;} s6_pose;
typedef struct s6_xy {double x,y;} s6_xy;
typedef struct s6_point {double x,y,yaw,v,w,t;} s6_point;
typedef struct s6_config {
 double cycle_s,horizon_s,dt_s,lethal_m,inflation_m;
 double w_heading,w_path,w_clearance,w_smoothness,w_velocity;
 double a_lin,a_yaw,brake,terminal_margin,wheel_radius,track,max_rpm;
 uint32_t allow_window_boundary_samples;
 /* R4 显式开关；清零后保留旧 S6 几何与速度档，便于独立回归。 */
 uint32_t r4_speed_bands,scene_geometry_enabled,scene_straight_mode;
 double body_length_m,body_width_m,body_buffer_m,front_lethal_from_edge_m,straight_w_threshold;
} s6_config;
typedef struct cm_cell {uint64_t last_obs_us;uint8_t occupancy,layer_bits;uint16_t reserved;} cm_cell;
typedef struct cm_state {
 cm_cell cells[S6_CM_CELLS];uint8_t static_layer[S6_CM_CELLS],obstacle_layer[S6_CM_CELLS],cost[S6_CM_CELLS];
 float distance_m[S6_CM_CELLS];uint32_t work[S6_CM_CELLS];
 double origin_x,origin_y;uint64_t source_us;uint32_t initialized;double lethal_m,inflation_m;
} cm_state;
typedef struct s6_trajectory {
 s6_point initial; /* t=0为实测状态，不计入最多30个未来点。 */
 s6_point points[S6_TRAJ_MAX];uint32_t count;double target_v,target_w,score,clear_min;
 double heading_score,path_score,clearance_score,smoothness_score,velocity_score;
 uint32_t terminal_stopped;char reason[64];
} s6_trajectory;
typedef struct s6_plan_input {
 uint64_t now_us;s6_pose pose;double current_v,current_w,goal_heading,front_clearance_m;
 const s6_xy*path;uint32_t path_count;
} s6_plan_input;
typedef struct s6_plan {
 s6_trajectory ranked[S6_SAMPLES_MAX];uint32_t count,sampled,dynamic_rejected,rpm_rejected,envelope_rejected,lethal_rejected;
 uint64_t generated_us;char reason[64];
} s6_plan;
/* 实现在nav_costmap.inc。 */
static void cm_begin(cm_state*,s6_pose,uint64_t,double,double);
static int cm_write_static(cm_state*,double,double,uint8_t,uint64_t);
static int cm_write_obstacle(cm_state*,double,double,uint8_t,uint64_t);
static int cm_project_sonar(cm_state*,s6_pose,unsigned,double,double,double,uint64_t);
static void cm_finish(cm_state*);
static uint8_t cm_cost_at(const cm_state*,double,double);
static uint8_t cm_occupancy_at(const cm_state*,double,double);
static double cm_clearance_at(const cm_state*,double,double);
static double cm_min_clearance_along(const cm_state*,const s6_point*,uint32_t);
static uint64_t cm_age_ms(const cm_state*,uint64_t);
/* 实现在nav_local_planner.inc，不新增电机后端或修改S5规则。 */
static void s6_config_default(s6_config*);
static int s6_config_valid(const s6_config*);
static void s6_plan_compute(const cm_state*,const s6_config*,const s6_plan_input*,s6_plan*);
static int s6_profile_at(const s6_trajectory*,double,double*,double*);
#endif
