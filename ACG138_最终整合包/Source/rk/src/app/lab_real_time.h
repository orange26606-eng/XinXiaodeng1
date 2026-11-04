#ifndef W1_REAL_TIME_H
#define W1_REAL_TIME_H
#include "lab_live.h"
#include "odometry.h"
#ifdef __cplusplus
extern "C" {
#endif
#define LAB_SCAN_TIME_BYTES 112u
#define LAB_ODOM_TIME_BYTES 256u
typedef struct lab_real_time_profile {
 uint64_t parameter_version,fpga_delay_min_us,fpga_delay_max_us,sdk_time_error_us;
 uint32_t counts_per_revolution,rate_error_ppm,warmup_samples;
 uint32_t fpga_delay_declared,fpga_stamp_declared,sdk_time_declared,profile_crc32;
 uint32_t lidar_model,lidar_firmware,odometry_config_crc32;
} lab_real_time_profile;
typedef struct lab_real_scan_time {
 uint32_t session,sequence,count,sample_period_ns,model,firmware;
 uint64_t legacy_end_us,first_us,last_us,fetched_us;
} lab_real_scan_time;
typedef struct lab_real_odom_time {
 uint64_t producer_id,parameter_version,received_us,published_us;
 uint32_t session,accepted_sequence,counts_per_revolution,config_crc32,generation;
 uint8_t raw_od01[64];lab_pose pose;
} lab_real_odom_time;
typedef struct lab_real_time_clock {
 lab_real_time_profile profile;
 uint32_t have,sequence,remote_ms,epoch,imu_token,samples,failed;
 uint64_t unwrapped_ms,received_us,lower_us,upper_us,last_estimate_us;
 char reason[64];
} lab_real_time_clock;
void lab_real_time_profile_default(lab_real_time_profile *out);
int lab_real_time_profile_valid(const lab_real_time_profile *profile);
lab_status lab_real_time_profile_load(const char *path,lab_real_time_profile *out);
void lab_real_clock_init(lab_real_time_clock *clock,const lab_real_time_profile *profile);
/* 时间界限依赖显式声明的单向时延及采样语义，不把单向回归结果称为时钟同步。
 */
lab_status lab_real_clock_observe(lab_real_time_clock *clock,const uint8_t od01[64],uint64_t received_us,uint64_t *measurement_us,uint64_t *uncertainty_us);
lab_status lab_real_scan_time_encode(uint8_t out[LAB_SCAN_TIME_BYTES],const lab_real_scan_time *time);
lab_status lab_real_scan_time_decode(lab_real_scan_time *out,const uint8_t *bytes,size_t size);
lab_status lab_real_scan_time_apply(const lab_real_scan_time *time,const lab_real_time_profile *profile,lab_live_sample *scan);
lab_status lab_real_odom_time_encode(uint8_t out[LAB_ODOM_TIME_BYTES],const lab_real_odom_time *time);
lab_status lab_real_odom_time_decode(lab_real_odom_time *out,const uint8_t *bytes,size_t size);
int lab_real_od01_valid(const uint8_t *bytes,size_t size);
#ifdef __cplusplus
}
#endif
#endif
