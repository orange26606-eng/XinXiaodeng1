#ifndef ACG_MAPPING_LAB_SIM_H
#define ACG_MAPPING_LAB_SIM_H
#include "lab_types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct lab_sim lab_sim;
enum lab_sim_path {LAB_PATH_STATIONARY=1,LAB_PATH_STRAIGHT=2,LAB_PATH_ROTATE=3,LAB_PATH_CIRCLE=4,LAB_PATH_RECTANGLE=5};
typedef struct lab_sim_config {
 uint32_t seed,mode,path,timeline_id;
 uint64_t duration_us;
 uint32_t scan_hz,pose_hz,encoder_hz,imu_hz,scan_points,counts_per_revolution;
 double room_width_m,room_height_m,speed_m_s,yaw_rate_rad_s;
 double wheel_diameter_m,track_m,mount_x_m,mount_y_m,mount_yaw_rad;
 double scan_noise_std_m,wheel_noise_std_counts,imu_noise_std_rad,imu_bias_rad_s;
 double wheel_scale[4];
 int32_t encoder_signs[4];
 uint32_t left_mask,drop_scan_every,drop_pose_every;
 uint64_t pose_delivery_delay_us;
} lab_sim_config;
typedef struct lab_sim_metrics {
 uint64_t generated_events,generated_scans,estimated_poses,truth_poses,dropped_scans,dropped_poses;
 double position_rmse_m,yaw_rmse_rad,final_position_error_m,final_yaw_error_rad;
} lab_sim_metrics;
void lab_sim_config_default(lab_sim_config *config);
lab_status lab_sim_open(lab_sim **out,const lab_sim_config *config);
/* 仿真采用确定性事件顺序，同一时刻先POSE后SCAN。真值单独作为LAB_EVENT_TRUTH输出。 */

lab_status lab_sim_next(lab_sim *sim,lab_event *out);
void lab_sim_get_metrics(const lab_sim *sim,lab_sim_metrics *out);
void lab_sim_close(lab_sim *sim);
#ifdef __cplusplus
}
#endif
#endif
