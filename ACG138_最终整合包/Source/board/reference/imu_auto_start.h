#ifndef IMU_AUTO_START_H
#define IMU_AUTO_START_H
#include "imu_iic_control.h"
#include "imu_protocol.h"

#define IMU_AUTO_WAIT_MS 10000u
#define IMU_AUTO_DATA_TIMEOUT_MS 5000u
#define IMU_AUTO_GOOD_FRAMES 5u
#define IMU_AUTO_GOOD_SPAN_MS 80u
#define IMU_AUTO_MAX_GAP_MS 200u

/* WAIT_LED仅确认校准命令已接受，不表示校准完成。从首次回执开始静置等待，再检查新的有效角度帧。
 */
struct imu_auto_start {
    struct imu_control_session control;
    struct imu_model model;
    uint64_t wait_ms, ready_ms, good_since_ms, last_good_ms;
    unsigned consecutive_good;
    int waiting, ready, complete, cancelled, data_timed_out;
};
void imu_auto_start_init(struct imu_auto_start *start, uint32_t token,
                         int resume, uint64_t now_ms);
void imu_auto_start_tick(struct imu_auto_start *start, uint64_t now_ms);
int imu_auto_start_next_tx(struct imu_auto_start *start, uint64_t now_ms, uint8_t output[32]);
int imu_auto_start_angles(struct imu_auto_start *start, const uint8_t *packet,
                          size_t size, uint64_t now_ms);
int imu_auto_start_may_open(const struct imu_auto_start *start, uint64_t now_ms);
int imu_auto_start_failed(const struct imu_auto_start *start);
void imu_auto_start_cancel(struct imu_auto_start *start);
#endif
