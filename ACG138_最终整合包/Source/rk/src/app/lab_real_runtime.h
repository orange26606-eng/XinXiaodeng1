#ifndef ACG_LAB_REAL_RUNTIME_H
#define ACG_LAB_REAL_RUNTIME_H
#include "lab_localization.h"
#include "lab_chassis.h"
#include "object_target_v2.h"
#include "s3_avoidance_types.h"
/* 运行时协调器，通过工厂注入传感器、定位和底盘模块。模块不可用时返回 UNAVAILABLE，资源由调用方管理。 */
typedef struct lab_real_runtime lab_real_runtime;
typedef struct lab_real_goal {uint64_t map_id;uint32_t id,has_yaw;double x_m,y_m,yaw_rad;} lab_real_goal;
typedef struct lab_real_navigation_view {
 uint32_t state,phase,path_count,narrow_passage,lidar_offset_verified,passage_reason;
 double body_length_m,body_width_m,margin_m,local_resolution_m;
 double recovery_distance_m,recovery_target_m;uint32_t recovery_attempts;
 uint64_t plans,map_version;
 const lab_real_xy *path;
 uint32_t local_path_count;const lab_real_xy *local_path;
 char reason[64],blocking_reason[64],phase_text[32];
 uint32_t cruise_mode,local_probe,can_start_local,backup_requests;
 uint32_t extended_reverse_attempts,extended_reverse_active;
 uint32_t provisional_route,candidate_sonar_rejections,candidate_occupied_rejections,candidate_unknown_rejections,candidate_accepted;
 double observed_fraction;char rear_reason[32];
 uint32_t route_complete,route_partial,route_search_budget,route_kind;char route_reason[48];uint32_t candidate_budget_rejections;
 char local_path_frame[8];uint64_t local_path_generation,local_path_source_us;
 uint32_t has_object_target;double actual_target_x,actual_target_y,actual_goal_x,actual_goal_y;
 s3_nearfield_view nearfield;
 s3_avoidance_view avoidance;
} lab_real_navigation_view;
typedef struct lab_real_navigation_ops {
 void *user;
 lab_status (*goal)(void *,const lab_real_goal *,uint64_t);
 lab_status (*cancel)(void *,uint64_t);
 /* OK为请求，AGAIN为等待，EOF为目标到达或停止，负值为失败。 */
 lab_status (*step)(void *,uint64_t,const lab_localization_view *,const lab_live_sample *,lab_real_command *);
 lab_status (*reset)(void *,const lab_real_context *);
 lab_status (*view)(void *,lab_real_navigation_view *);
 void (*close)(void *);
 void (*chassis_hint)(void *,const lab_chassis_feedback *);
 /* 刷新定位后、新计算命令发布前调用，保持旧命令原日期并拒绝过期结果。
 */
 lab_status (*publish)(void*,uint64_t,const lab_localization_view*,lab_real_command*);
 void (*reduce_load)(void*,uint32_t);
 lab_status (*set_cruise)(void*,uint32_t);
 /* 仅丢弃ARM前缓存动作，保留路线、目标和脱困预算。 */
 void (*invalidate_command)(void*);
 /* 全局定位前只读通知已有样本，不执行运动。 */
 void (*observe_sample)(void*,const lab_live_sample*,uint64_t);
 lab_status (*local_step)(void*,uint64_t,const lab_pose*,uint64_t,const lab_live_sample*,const lab_real_command*,double,lab_real_command*);
 lab_status (*local_publish)(void*,uint64_t,const lab_pose*,uint64_t,lab_real_command*);
 lab_status (*local_mode)(void*,uint32_t); /* 1为静止目标，2为跟随，0结束局部任务。 */
 lab_status (*local_target)(void*,const object_route_target_v2*);
} lab_real_navigation_ops;
typedef struct lab_real_bindings {
 lab_live_source_binding sensors;lab_localization_binding localization;lab_chassis_binding chassis;
 lab_real_navigation_ops navigation;
} lab_real_bindings;
typedef struct lab_real_runtime_config {
 uint32_t mode,allow_hardware_output,geometry_verification_required,geometry_verified;
 double max_linear_m_s,max_angular_rad_s;
 uint64_t max_scan_age_us,max_pose_age_us,command_valid_us,control_session;
} lab_real_runtime_config;
typedef struct lab_real_runtime_view {
 lab_real_context context;
 uint32_t mode,enabled,pending_enable,missing_modules,has_scan,has_odometry,has_localization,has_goal;
 uint64_t time_us,scan_time_us,odometry_time_us,accepted,rejected,ticks,hardware_calls;
 lab_status last_status;
 lab_localization_view localization;
 lab_real_goal goal;
 lab_real_command request;
 lab_chassis_output protected_output;
 lab_chassis_feedback feedback;
 lab_live_health sensor_health;
 lab_real_navigation_view navigation;
 uint32_t experimental_timing;
 lab_pose raw_odometry;
 const struct scan_frame *latest_scan; /* 运行时借用对象，生命周期随本次调用。 */
 uint64_t localizer_waits,coalesced_pose_aliases,scan_uncertainty_us,odometry_uncertainty_us;
 char reason[64];
 char last_stop_reason[64],last_stop_detail[64];uint64_t last_stop_us;
 uint32_t resume_requires_start;
 uint64_t control_elapsed_us,control_warns,control_soft_events;
 uint32_t control_soft_streak,control_load_reduced,manual_revoked;
 uint32_t object_soft_rearm_pending;
 uint32_t object_mode; /* 本地实物任务目标。 */
 uint32_t object_turn_waiting;double object_turn_selected_w,object_turn_applied_w;char object_turn_reason[48];
} lab_real_runtime_view;
typedef int (*lab_real_object_controller)(void*,const lab_real_runtime_view*,uint64_t,lab_real_command*);
lab_status lab_real_runtime_object_target_v2(lab_real_runtime*,const object_route_target_v2*);
lab_status lab_real_runtime_object_controller(lab_real_runtime*,lab_real_object_controller,void*);
lab_status lab_real_runtime_object_mode(lab_real_runtime*,int,uint64_t);
/* 只读就绪检查，调用方仍需同一显式活动任务、认证手机租约及当前屏障；此处不确认清除最终故障。
 */
lab_status lab_real_runtime_object_rearm_ready(const lab_real_runtime*,uint64_t);
void lab_real_runtime_config_default(lab_real_runtime_config *out);
lab_status lab_real_runtime_open(lab_real_runtime **out,const lab_real_runtime_config *config,const lab_real_context *context,const lab_real_bindings *bindings);
lab_status lab_real_runtime_tick(lab_real_runtime *run,uint64_t now_us);
lab_status lab_real_runtime_set_goal(lab_real_runtime *run,const lab_real_goal *goal,uint64_t now_us);
lab_status lab_real_runtime_cancel(lab_real_runtime *run,uint64_t now_us);
lab_status lab_real_runtime_enable(lab_real_runtime *run,uint32_t enabled,uint64_t now_us);
lab_status lab_real_runtime_acknowledge_start(lab_real_runtime *run,uint64_t now_us);
lab_status lab_real_runtime_reset(lab_real_runtime *run,const lab_real_context *new_context);
lab_status lab_real_runtime_view_get(const lab_real_runtime *run,lab_real_runtime_view *out);
void lab_real_runtime_close(lab_real_runtime *run);
/* 记录实际控制耗时，仅用于选择较轻计算流程，不授予控制权。 */
lab_status lab_real_runtime_set_cruise(lab_real_runtime*,uint32_t);
void lab_real_runtime_note_cost(lab_real_runtime*,uint64_t);
/* 可选观测器接收每个获准样本，包括时间戳别名；返回失败可停止处理，不保留借用指针。
 */
typedef lab_status (*lab_real_sample_observer)(void *,const lab_live_sample *);
lab_status lab_real_runtime_observe(lab_real_runtime *run,lab_real_sample_observer callback,void *user);
#endif
