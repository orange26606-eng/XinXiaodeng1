#ifndef ACG_LIVE_SENSOR_CANDIDATE_H
#define ACG_LIVE_SENSOR_CANDIDATE_H
/* 传感器内部数据处理接口，不管理套接字、线程或电机输出，也不回退到模拟输入。除远端时间字段外，时间单位为 RK CLOCK_MONOTONIC 微秒；公开 ABI 和报文格式保持不变。 */
#include "lab_types.h"
#include "odometry.h"
#ifdef __cplusplus
extern "C" {
#endif
#define LI_CANDIDATE_VERSION UINT32_C(0x00010000)
#define LI_TIME_ERROR_UNKNOWN UINT64_MAX

typedef struct li_source li_source;
enum li_input { LI_INPUT_SHARED=1, LI_INPUT_OD01_CALLBACK=2 };
enum li_run_kind { LI_REAL_READONLY=1, LI_OFFLINE_TEST=2 };
enum li_time_basis { LI_HOST_RECEIPT_PROXY=1, LI_HOST_SCAN_END=2, LI_REMOTE_UNMAPPED=3 };
enum li_flags {
 LI_HOST_TIME=1u, LI_ESTIMATED_MEASUREMENT_TIME=2u,
 LI_COUNTS_MASK_INFERRED=4u, LI_SYNTHETIC_TEST=8u
};
enum li_capability { LI_CAP_POSE=1u, LI_CAP_COUNTS=2u, LI_CAP_RAW_IMU_YAW=4u, LI_CAP_SCAN=8u };
typedef struct li_context {
 uint32_t timeline_id,origin_epoch,run_kind;
 uint64_t odom_instance,scan_instance; /* 集成层持有的生产者代次。 */
} li_context;
typedef struct li_meta {
 li_context context;
 uint32_t producer_session,sequence,time_basis,flags;
 uint64_t time_us,received_us,time_error_us;
} li_meta;
typedef struct li_pose_sample {
 li_meta meta;
 lab_pose pose;
 int32_t counts[4];
 uint32_t counts_valid_mask,counts_per_revolution,legacy_quality;
} li_pose_sample;
typedef struct li_imu_sample {
 li_meta meta;
 double yaw_rad;
 uint32_t valid_flags,remote_fpga_ms,age_ms,calibration_token;
 /* OD01提供航向，角速度标为不可用。 */
} li_imu_sample;
typedef struct li_scan_view {
 li_meta meta;
 const struct scan_frame *frame; /* 借用至下次修改、复位或关闭。 */
 uint64_t interval_start_us;
 uint32_t period_us;
} li_scan_view;
typedef struct li_config {
 uint32_t input,history_capacity,counts_per_revolution,allow_receipt_proxy;
 uint64_t max_pose_age_us,max_scan_age_us,max_interpolation_gap_us,receipt_time_error_us;
 struct odom_config fusion; /* 仅用于OD01回调模式。 */
} li_config;
typedef struct li_stats {
 uint64_t pose_samples,scan_samples,duplicates,rejected,context_changes,time_errors;
 uint64_t query_waits,query_rejected,resets,time_aliases;
 uint32_t pose_history,capabilities,reset_required;
 size_t allocated_bytes;
} li_stats;

void li_config_default(li_config *config);
lab_status li_open(li_source **out,const li_config *config,const li_context *context);
/* 只接受显式重置，集成层需丢弃旧代次排队回调。需新时间线或原点，各数据源代次独立。
 */
lab_status li_reset(li_source *source,const li_context *new_context);
/* 调用方提供读取一致的本机布局共享快照；该快照不等同于 IMU 原始数据。 */
lab_status li_push_shared(li_source *source,const struct odom_shared *snapshot,
                         uint64_t producer_instance,uint64_t now_us);
/* 由现有UDP5010接收器调用；传输层负责对端校验。 */

lab_status li_push_od01(li_source *source,const uint8_t *packet,size_t bytes,
                       uint64_t producer_instance,uint64_t received_us);
/* 输入选择完整扫描或独占接收器返回的LS01数据报。 */

lab_status li_push_scan(li_source *source,const struct scan_frame *frame,
                       uint64_t producer_instance,uint64_t now_us);
lab_status li_push_ls01(li_source *source,const uint8_t *packet,size_t bytes,
                       uint64_t producer_instance,uint64_t now_us);
lab_status li_latest_pose(li_source *source,uint64_t now_us,li_pose_sample *out);
lab_status li_latest_imu(li_source *source,uint64_t now_us,li_imu_sample *out);
lab_status li_latest_scan(li_source *source,uint64_t now_us,li_scan_view *out);
/* 仅接受准确时刻或有效两端插值，角度取最短路径。代理接收时间需allow_receipt_proxy=1且明确标为估计或未知；成功不代表测量时钟已标定或可直接驾驶。

 */
lab_status li_pose_at(li_source *source,uint64_t time_us,uint64_t now_us,li_pose_sample *out);
lab_status li_align_latest_scan(li_source *source,uint64_t now_us,li_scan_view *scan,li_pose_sample *pose);
void li_get_stats(const li_source *source,li_stats *out);
void li_close(li_source *source);
#ifdef __cplusplus
}
#endif
#endif
