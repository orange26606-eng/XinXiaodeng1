#ifndef W1_REAL_TIME_SOURCE_H
#define W1_REAL_TIME_SOURCE_H
#include "lab_real_time.h"
typedef struct lab_real_timed_source_config {
 lab_live_source_config legacy;
 lab_real_time_profile profile;
 char timed_odometry_path[256];uint32_t scan_time_port;
} lab_real_timed_source_config;
/* 可选时间适配层。复用现有 LS01 接收器，不绑定 OD01/5010。 */
lab_status lab_real_timed_source_create(lab_live_source_binding *out,const lab_real_timed_source_config *config,const lab_real_context *context);
/* POSIX数据源和集成测试共用的可移植转换。 */
typedef struct lab_real_timed_odom_state {
 lab_real_time_clock clock;uint64_t producer_id,last_imu_time_us;uint32_t session,config_crc,accepted,have_imu,imu_sequence;
} lab_real_timed_odom_state;
void lab_real_timed_odom_init(lab_real_timed_odom_state *state,const lab_real_time_profile *profile);
lab_status lab_real_timed_odom_convert(lab_real_timed_odom_state *state,const lab_real_odom_time *cache,const lab_real_context *context,uint64_t visible_us,lab_live_sample out[3],uint32_t *count);
#endif
