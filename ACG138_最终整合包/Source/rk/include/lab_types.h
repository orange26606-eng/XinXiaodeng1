#ifndef ACG_MAPPING_LAB_TYPES_H
#define ACG_MAPPING_LAB_TYPES_H
#include <stddef.h>
#include <stdint.h>
#include "scan_core.h"
#ifdef __cplusplus
extern "C" {
#endif
#define LAB_API_VERSION UINT32_C(0x00010000)
#define LAB_METADATA_MAX 32768u
#define LAB_RECORD_PAYLOAD_MAX 131072u
#define LAB_POSE_HISTORY_MAX 256u
#define LAB_TRACE_MAX 4096u
#define LAB_MAP_SIDE_MAX 1024u
#define LAB_REPLAY_LOOKAHEAD_MAX 128u
typedef enum lab_status {
 LAB_OK=0, LAB_AGAIN=1, LAB_EOF=2, LAB_ERR_ARGUMENT=-1, LAB_ERR_IO=-2,
 LAB_ERR_FORMAT=-3, LAB_ERR_CRC=-4, LAB_ERR_TRUNCATED=-5, LAB_ERR_VERSION=-6,
 LAB_ERR_LIMIT=-7, LAB_ERR_TIME=-8, LAB_ERR_CONTEXT=-9, LAB_ERR_READONLY=-10,
 LAB_ERR_NOMEM=-11, LAB_ERR_UNAVAILABLE=-12
} lab_status;
typedef enum lab_mode {LAB_LIVE_STATIC=1,LAB_REPLAY=2,LAB_SIM_IDEAL=3,LAB_SIM_NOISY=4} lab_mode;
typedef enum lab_provider {
 LAB_PROVIDER_USB=1,LAB_PROVIDER_FPGA=2,LAB_PROVIDER_ODOMETRY=3,LAB_PROVIDER_FIXED=4,
 LAB_PROVIDER_SIM_SENSOR=5,LAB_PROVIDER_SIM_ESTIMATE=6,LAB_PROVIDER_SIM_IDEAL=7,
 LAB_PROVIDER_TRUTH=8
} lab_provider;
typedef enum lab_clock {
 LAB_CLOCK_HOST_MONOTONIC=1,LAB_CLOCK_SIMULATION=2,LAB_CLOCK_REPLAY=3,LAB_CLOCK_REMOTE_UNMAPPED=4
} lab_clock;
enum lab_meta_flags {LAB_TIME_VALID=1u,LAB_DATA_VALID=2u};
enum lab_pose_quality {
 LAB_POSE_VALID=1u,LAB_POSE_UNCALIBRATED=2u,LAB_POSE_STALE=4u,LAB_POSE_GAP=8u,
 LAB_POSE_SLIP=16u,LAB_POSE_IMU_USED=32u
};
enum lab_imu_flags {LAB_IMU_YAW_VALID=1u,LAB_IMU_RATE_VALID=2u};
enum lab_event_type {
 LAB_EVENT_SCAN=1,LAB_EVENT_POSE=2,LAB_EVENT_ENCODER=3,LAB_EVENT_IMU=4,
 LAB_EVENT_TRUTH=5,LAB_EVENT_RESET=6,LAB_EVENT_GAP=7
};
enum lab_reset_reason {
 LAB_RESET_START=1,LAB_RESET_SOURCE_CHANGE=2,LAB_RESET_ORIGIN_CHANGE=3,
 LAB_RESET_REPLAY_RESTART=4,LAB_RESET_USER=5
};
typedef struct lab_context {
 uint32_t mode,clock_domain,timeline_id,origin_epoch;
} lab_context;
typedef struct lab_meta {
 uint32_t mode,provider,clock_domain,flags;
 uint32_t timeline_id,stream_id,stream_session,origin_epoch,sequence;
 uint64_t time_us,received_us;
} lab_meta;
typedef struct lab_pose {
 double x_m,y_m,yaw_rad,distance_m;
 uint32_t quality,calibrated;
 double uncertainty_m;
} lab_pose;
typedef struct lab_encoder {
 int32_t counts[4];
 uint32_t valid_mask,counts_per_revolution;
} lab_encoder;
typedef struct lab_imu {double yaw_rad,yaw_rate_rad_s;uint32_t valid_flags;} lab_imu;
typedef struct lab_truth {double x_m,y_m,yaw_rad,distance_m;} lab_truth;
typedef struct lab_reset {uint32_t reason,next_origin_epoch;} lab_reset;
typedef struct lab_gap {uint64_t dropped_events;uint32_t reason,affected_event_type;} lab_gap;
/* 事件值由调用方持有，约100KB，队列放在堆上，载荷无指针。各模块在push或append返回前完成复制或读取。
 */
typedef struct lab_event {
 lab_meta meta;
 uint32_t type;
 union {
  struct scan_frame scan;
  lab_pose pose;
  lab_encoder encoder;
  lab_imu imu;
  lab_truth truth;
  lab_reset reset;
  lab_gap gap;
 } data;
} lab_event;
int lab_meta_valid(const lab_meta *meta);
int lab_event_valid(const lab_event *event);
int lab_context_matches(const lab_context *context,const lab_meta *meta);
const char *lab_status_text(lab_status status);
const char *lab_mode_text(uint32_t mode);
#ifdef __cplusplus
}
#endif
#endif
