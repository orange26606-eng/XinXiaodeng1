#ifndef W1_MPUSVC_H
#define W1_MPUSVC_H
#include "lab_live.h"
#include "lab_real_mpusvc_vendor.h"
#ifdef __cplusplus
extern "C" {
#endif
#define MPUSVC_SNAPSHOT_BYTES 768u
#define MPUSVC_CAPTURE_HEADER 64u
#define MPUSVC_CAPTURE_RECORD 128u
#define MPUSVC_CAPTURE_FOOTER 40u
enum mpusvc_flags {MPUSVC_SEEN=1,MPUSVC_RUNNING=2,MPUSVC_STOPPED=4,MPUSVC_RECORD_ERROR=8};
typedef struct mpusvc_view {
 uint32_t source,flags,config_crc,session,pose_sequence,have_pose,epoch,counts_mask;
 uint32_t bias_ready,cal_samples,fusion_quality,counts_per_rev;int32_t axes[3];
 uint64_t instance,export_sequence,published_us,raw_received_us,raw_fpga_us;
 uint64_t time_uncertainty_us,parameter_version,pose_fpga_us,pose_received_us;
 uint64_t ag_fpga_us,ag_received_us,mag_fpga_us,mag_received_us;
 uint32_t ag_sequence,mag_sequence;
 uint32_t pose_imu_age_known;uint64_t pose_imu_age_us;
 uint8_t raw[100];
 uint64_t accepted,rejected,duplicates,missing,restarts,latest_skipped,record_drops,record_count,started_us;
 double q[4],rpy[3],accel[3],gyro[3],bias[3],mag[3],rpm[4],temperature;
 struct odom_state odometry;struct odom_config odom_config;double cal_seconds;
} mpusvc_view;
typedef struct mpusvc_core mpusvc_core;
lab_status mpusvc_open(mpusvc_core **out,uint32_t source,uint64_t instance,uint64_t parameter_version,const struct odom_config *odom,const struct i9_config *imu);
lab_status mpusvc_push(mpusvc_core *core,const uint8_t *raw,size_t bytes,uint64_t received_us);
void mpusvc_snapshot(const mpusvc_core *core,uint64_t now,mpusvc_view *out);
void mpusvc_close(mpusvc_core *core);
lab_status mpusvc_encode(uint8_t out[MPUSVC_SNAPSHOT_BYTES],const mpusvc_view *view);
/* 独立v2文件供建图使用，mpusvc_encode保持v1字节兼容。 */
lab_status mpusvc_encode_timed(uint8_t out[MPUSVC_SNAPSHOT_BYTES],const mpusvc_view *view);
lab_status mpusvc_decode(mpusvc_view *out,const uint8_t *data,size_t bytes);
lab_status mpusvc_read(const char *path,mpusvc_view *out);
lab_status mpusvc_legacy_read(const char *path,struct odom_shared *out);
lab_status mpusvc_write_new(const char *path,const void *data,size_t bytes);
/* 独立有界文件传输，旧接口及timed768 ABI不变。 */
#define MPUSVC_HISTORY_CAPACITY 256u
#define MPUSVC_HISTORY_HEADER 64u
typedef struct mpusvc_history_writer {void *file;uint64_t instance,first,last,parameter_version;uint32_t count,config_crc,session;} mpusvc_history_writer;
typedef struct mpusvc_history_cursor {uint64_t instance,sequence,rows,overwritten,latest_version_jumps,header_last,parameter_version,rejected_rows;uint32_t config_crc,session;} mpusvc_history_cursor;
lab_status mpusvc_history_create(mpusvc_history_writer *writer,const char *path);
lab_status mpusvc_history_append(mpusvc_history_writer *writer,const mpusvc_view *view);
void mpusvc_history_close(mpusvc_history_writer *writer);
/* 按文件头、行、文件头稳定读取，每行保留CRC及原时刻。CRC或格式错误行只推进可信头部序号并记拒绝，不作为数据返回，也不反复阻塞新行。

 */
lab_status mpusvc_history_next(const char *path,mpusvc_history_cursor *cursor,uint8_t bytes[768],mpusvc_view *view);
void mpusvc_imu_model(const mpusvc_view *view,uint64_t now_us,struct imu_model *out);
void mpusvc_print(const mpusvc_view *view,uint64_t now_us);
/* 仅录制规范原始数据。 */
void mpusvc_capture_header(uint8_t out[64],uint32_t source,uint64_t instance,uint64_t created_us,uint64_t version,uint32_t config_crc);
void mpusvc_capture_record(uint8_t out[128],const uint8_t raw[100],uint64_t received_us,uint64_t ordinal);
void mpusvc_capture_footer(uint8_t out[40],uint64_t count,uint64_t drops,uint32_t stream_crc,uint32_t flags);
lab_status mpusvc_reprocess(const char *capture,const struct odom_config *odom,const struct i9_config *imu,mpusvc_view *out);
#ifdef __cplusplus
}
#endif
#endif
