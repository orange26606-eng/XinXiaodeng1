/* 本地ABI，不访问硬件。 */
#ifndef PHONE_SHARED_H
#define PHONE_SHARED_H
#include <stdint.h>
#define PHONE_MAGIC UINT32_C(0x31485041)
#define PHONE_MAX_FRAME (16u*1024u*1024u)
struct phone_sensor_state {
    uint64_t stamp_ms, imu_ms, robot_ms;
    double roll, pitch, yaw;
    uint32_t imu_state, robot_seen, robot_bad;
    uint16_t mm[3], remote_sequence;
    uint8_t range_valid, error[3], auto_mode, actual, feedback, stop_reason;
    uint8_t remote_supported, remote_flags, nav_supported, nav_state, near_mask, feedback_supported, feedback_phase, feedback_retries;
};
struct phone_frame_meta {
    uint64_t stamp_ms;
    uint32_t width,height,stride,fourcc,matrix,range,bytes,reserved;
};
struct phone_shared {
    uint32_t magic,version,state_sequence,frame_sequence;
    struct phone_sensor_state state;
    struct phone_frame_meta frame;
    uint8_t pixels[PHONE_MAX_FRAME];
};
static inline uint32_t phone_read_seq(const uint32_t *p){return __atomic_load_n(p,__ATOMIC_ACQUIRE);}
static inline void phone_begin_write(uint32_t *p){__atomic_add_fetch(p,1,__ATOMIC_SEQ_CST);}
static inline void phone_end_write(uint32_t *p){__atomic_add_fetch(p,1,__ATOMIC_RELEASE);}
#endif
