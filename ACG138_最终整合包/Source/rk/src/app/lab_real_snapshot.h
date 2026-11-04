#ifndef W1_REAL_SNAPSHOT_H
#define W1_REAL_SNAPSHOT_H
#include "lab_real_mpu.h"
/* 读取传感器服务快照并复用其中的位姿。此适配层不绑定 UDP5016，也不重复运行姿态和里程计融合。 */
typedef struct lab_real_snapshot lab_real_snapshot;
typedef struct lab_real_snapshot_stats {
 uint64_t exports,duplicates,rejected,missed_exports,pose_samples,imu_samples,scans,queue_drops;
 uint64_t raw_received_us,pose_time_us,imu_time_us,scan_time_us,pose_error_us;
 uint32_t failed,pose_valid,imu_valid,scan_valid;
 /* 私有诊断，不改变UDP、FPGA或共享服务格式。 */
 uint64_t deskew_applied,deskew_skipped,deskew_replaced,deskew_waits;
 uint64_t deskew_first_us,deskew_reference_us,deskew_last_us,deskew_pose_error_us;
 uint32_t deskew_points,deskew_invalid_points,deskew_span_us,deskew_pending;
 double deskew_translation_m,deskew_yaw_rad,deskew_max_origin_m;
 uint64_t deskew_storage_bytes;
 char deskew_reason[96];
 char reason[80];
} lab_real_snapshot_stats;
uint64_t lab_real_snapshot_identity(const lab_real_mpu_config *config);
lab_status lab_real_snapshot_open(lab_real_snapshot **out,const lab_real_mpu_config *config,const lab_real_context *context,uint64_t version);
lab_status lab_real_snapshot_feed(lab_real_snapshot *source,const uint8_t *bytes,size_t size,uint64_t observed_us);
lab_status lab_real_snapshot_scan(lab_real_snapshot *source,const uint8_t *bytes,size_t size,uint64_t observed_us);
lab_status lab_real_snapshot_take(lab_real_snapshot *source,uint64_t now_us,lab_live_sample *out);
unsigned lab_real_snapshot_pending(const lab_real_snapshot *source);
void lab_real_snapshot_health(const lab_real_snapshot *source,uint64_t now_us,lab_live_health *out);
void lab_real_snapshot_stats_get(const lab_real_snapshot *source,lab_real_snapshot_stats *out);
void lab_real_snapshot_close(lab_real_snapshot *source);
lab_status lab_real_snapshot_source_create(lab_live_source_binding *out,const char *snapshot_path,const lab_real_mpu_config *config,const lab_live_source_config *scan,const lab_real_context *context);
#endif
