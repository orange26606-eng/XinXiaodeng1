#ifndef ACG_LAB_LIVE_H
#define ACG_LAB_LIVE_H
/* 独立实测接口约定，采用C ABI和单一持有者，不使用lab_event/v1线上格式。 */
#include "lab_types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define LAB_REAL_API_VERSION UINT32_C(0x00010000)
#define LAB_REAL_PATH_MAX 4096u
#define LAB_REAL_HISTORY_MAX 256u
enum lab_real_source {LAB_REAL_HARDWARE=1,LAB_REAL_RECORDING=2,LAB_REAL_TEST_FIXTURE=3};
enum lab_real_clock {LAB_REAL_MONOTONIC=1,LAB_REAL_REPLAY_CLOCK=2,LAB_REAL_UNMAPPED_CLOCK=3};
enum lab_real_mode {LAB_REAL_READONLY=1,LAB_REAL_MAPPING=2,LAB_REAL_LOCALIZATION=3,LAB_REAL_SHADOW=4,LAB_REAL_NAVIGATION=5};
enum lab_real_flags {LAB_REAL_TIME_VALID=1u,LAB_REAL_DATA_VALID=2u,LAB_REAL_TIME_ESTIMATED=4u,LAB_REAL_CALIBRATED=8u};
enum lab_real_provider {LAB_REAL_USB=1,LAB_REAL_FPGA=2,LAB_REAL_ODOMETRY=3,LAB_REAL_LOCALIZER=4};
typedef struct lab_real_context {uint32_t source,clock,timeline_id,origin_epoch;} lab_real_context;
typedef struct lab_real_meta {
 lab_real_context context;
 uint32_t provider,stream_id,stream_session,sequence,flags,reserved;
 uint64_t time_us,received_us,time_uncertainty_us,parameter_version;
} lab_real_meta;
/* 载荷单位及扫描Q14/Q2量化沿用公开结构；此元数据结构与ACGLAB v1编码分开。 */

typedef struct lab_live_sample {
 lab_real_meta meta;
 uint32_t type,reserved;
 union {struct scan_frame scan;lab_pose odometry;lab_encoder encoder;lab_imu imu;lab_reset reset;lab_gap gap;} data;
} lab_live_sample;
typedef struct lab_live_source_config {
 char lidar_device[256],odometry_path[256],calibration_path[256],allowed_source_ip[64];
 uint32_t ls01_port,read_existing_odometry;
 uint64_t max_scan_age_us,max_odometry_age_us,parameter_version;
} lab_live_source_config;
typedef struct lab_live_health {
 lab_real_context context;
 uint64_t time_us,scan_time_us,odometry_time_us,encoder_time_us,imu_time_us;
 uint64_t accepted,rejected,gaps,reconnects;
 uint32_t scan_valid,odometry_valid,encoder_valid_mask,imu_valid_flags;
 char reason[64];
} lab_live_health;
typedef struct lab_live_source_ops {
 uint32_t api_version,struct_bytes;
 /* 非阻塞读取，OK表示一个归调用方持有的输出，AGAIN表示暂无数据。按received_us排序，返回前复制载荷，不保留调用方指针。
 */
 lab_status (*poll)(void *user,uint64_t now_us,lab_live_sample *out);
 lab_status (*health)(void *user,uint64_t now_us,lab_live_health *out);
 /* 调用方分配新上下文，并丢弃旧队列数据。 */
 lab_status (*reset)(void *user,const lab_real_context *context);
 void (*close)(void *user);
} lab_live_source_ops;
typedef struct lab_live_source_binding {void *user;const lab_live_source_ops *ops;} lab_live_source_binding;
/* 创建传感器接收实例；创建失败时不回退到模拟输入。调用方持有绑定，并负责调用一次 ops.close。 */
lab_status lab_live_source_create(lab_live_source_binding *out,const lab_live_source_config *config,const lab_real_context *context);
void lab_live_source_config_default(lab_live_source_config *out);
int lab_real_context_valid(const lab_real_context *context);
int lab_real_context_equal(const lab_real_context *a,const lab_real_context *b);
int lab_real_meta_valid(const lab_real_meta *meta);
int lab_live_sample_valid(const lab_live_sample *sample);
const char *lab_real_source_text(uint32_t source);
const char *lab_real_mode_text(uint32_t mode);
#ifdef __cplusplus
}
#endif
#endif
