#ifndef ACG_LAB_AUTONOMY_H
#define ACG_LAB_AUTONOMY_H
/* 仿真接口扩展。沿用 LAB_API_VERSION 和 v1 数据格式，输出不连接硬件。 */
#include "lab_mapping.h"
#ifdef __cplusplus
extern "C" {
#endif
#define LAB_AUTONOMY_API_VERSION UINT32_C(0x00010100)
#define LAB_AUTO_PATH_MAX 4096u
typedef struct lab_autonomy lab_autonomy;
enum lab_auto_mode {LAB_AUTO_NAV_IDEAL=1,LAB_AUTO_SLAM_NAV=2};
enum lab_auto_state {LAB_AUTO_IDLE=0,LAB_AUTO_TRACKING,LAB_AUTO_WAITING,
 LAB_AUTO_REACHED,LAB_AUTO_CANCELLED,LAB_AUTO_UNREACHABLE,LAB_AUTO_FAILED};
typedef struct lab_auto_config {
 uint32_t mode,enable_loop;
 lab_mapping_config map;
 double vehicle_width_m,vehicle_length_m,safety_margin_m,max_speed_m_s;
 uint64_t max_localization_age_us,command_valid_us;
 double lidar_to_rear_m; /* 暂定安装几何，默认0.21m。 */
} lab_auto_config;
typedef struct lab_auto_goal {uint32_t id,has_yaw;double x_m,y_m,yaw_rad;} lab_auto_goal;
typedef struct lab_auto_command {
 double linear_m_s,angular_rad_s;
 uint64_t time_us,expires_us;
 uint32_t sequence;
 lab_context context;
} lab_auto_command;
typedef struct lab_auto_xy {double x_m,y_m;} lab_auto_xy;
typedef struct lab_auto_view {
 uint32_t state,slam_reason,has_pose,path_count;
 uint64_t map_version,correction_version,scans,matched,rejected,plans,pose_time_us;
 uint64_t truth_rejected,context_rejected,alignment_waits;
 lab_pose pose;
 lab_auto_goal goal;
 lab_auto_command command;
 const lab_auto_xy *path;
 lab_map_view map;
 char state_text[32],reason[64],blocking_reason[64],localization_reason[64];
 size_t integration_bytes,slam_bytes,nav_bytes;
 char recovery_phase[32];
 uint32_t reverse_attempts,clear_scans;
 double rear_clearance_m,backup_progress_m,backup_total_m;
 uint32_t dynamic_count;
 lab_auto_xy dynamic_obstacles[32]; /* 实测临时障碍，不使用仿真真值。 */
} lab_auto_view;
void lab_auto_config_default(lab_auto_config *config);
lab_status lab_auto_open(lab_autonomy **out,const lab_auto_config *config,const lab_context *context);
/* POSE为原始里程计，SCAN为LS01，时间戳单调不减。编码器与IMU可录制，定位使用传入的融合位姿；坐标重置经reset()执行。 */


lab_status lab_auto_push(lab_autonomy *auto_run,const lab_event *event);
/* 仅NAV_IDEAL接受已知地图，SLAM_NAV自行维护估计地图。 */
lab_status lab_auto_set_known_map(lab_autonomy *auto_run,const lab_map_view *map,uint64_t revision);
lab_status lab_auto_set_goal(lab_autonomy *auto_run,const lab_auto_goal *goal,uint64_t now_us);
lab_status lab_auto_cancel(lab_autonomy *auto_run,uint64_t now_us);
/* 控制时刻严格递增，且不早于所有已交付事件的received_us。失败返回零命令，调用方另行实施期限检查。
 */
lab_status lab_auto_tick(lab_autonomy *auto_run,uint64_t now_us,lab_auto_command *out);
/* 停止并取消旧目标，清空全部历史，需使用新时间线或原点。 */
lab_status lab_auto_reset(lab_autonomy *auto_run,const lab_context *new_context);
/* 借用的地图及路径只在下次修改调用前有效，由单线程持有。 */
lab_status lab_auto_get_view(lab_autonomy *auto_run,lab_auto_view *out);
void lab_auto_close(lab_autonomy *auto_run);
#ifdef __cplusplus
}
#endif
#endif
