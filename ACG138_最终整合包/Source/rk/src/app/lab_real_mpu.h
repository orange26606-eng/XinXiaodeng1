#ifndef W1_REAL_MPU_H
#define W1_REAL_MPU_H
#include "lab_live.h"
#ifdef __cplusplus
extern "C" {
#endif
/* I9E1-v2 接收适配器。只读取数据，沿用原共享文件 ABI。 */
typedef struct lab_real_mpu lab_real_mpu;
typedef struct lab_real_mpu_config {
 char bind_ip[64],peer_ip[64],odom_config[256],imu_config[256],timing_profile[256];
 uint32_t port,experimental,drift_ppm;
 uint64_t delay_min_us,delay_max_us,lidar_error_us,imu_error_us,declared_version;double mount[3];
} lab_real_mpu_config;
typedef struct lab_real_mpu_stats {
 uint64_t packets,crc_errors,duplicates,gaps,scan_frames,timing_rejected,queue_drops;
 uint64_t pose_time_us,imu_time_us,scan_time_us,uncertainty_us;
 uint32_t mpu_id,imu_flags,i2c_error,bias_ready,counts_mask,have_profile,clock_fault,pose_valid,scan_valid;
 double x_m,y_m,yaw_rad,distance_m,roll_rad,pitch_rad,rpm[4];int32_t counts[4];
 char reason[64];
} lab_real_mpu_stats;
void lab_real_mpu_config_default(lab_real_mpu_config *out);
uint64_t lab_real_mpu_identity(const lab_real_mpu_config *config);
lab_status lab_real_mpu_profile(lab_real_mpu_config *out,const char *path);
lab_status lab_real_mpu_core_open(lab_real_mpu **out,const lab_real_mpu_config *config,const lab_real_context *context,uint64_t parameter_version);
lab_status lab_real_mpu_feed(lab_real_mpu *source,const uint8_t *packet,size_t bytes,uint64_t observed_us);
/* LT01时间包络中保留原LS01报文。 */
#define LAB_REAL_TIMED_HEADER 64u
size_t lab_real_timed_encode(uint8_t *out,size_t capacity,const uint8_t *ls01,size_t bytes,uint64_t first_us,uint64_t returned_us,uint32_t span_us);
lab_status lab_real_mpu_scan(lab_real_mpu *source,const uint8_t *packet,size_t bytes,uint64_t observed_us);
lab_status lab_real_mpu_take(lab_real_mpu *source,uint64_t now_us,lab_live_sample *out);
void lab_real_mpu_get_stats(const lab_real_mpu *source,lab_real_mpu_stats *out);
void lab_real_mpu_core_close(lab_real_mpu *source);
lab_status lab_real_mpu_source_create(lab_live_source_binding *out,const lab_real_mpu_config *config,const lab_live_source_config *scan,const lab_real_context *context);
#ifdef __cplusplus
}
#endif
#endif
