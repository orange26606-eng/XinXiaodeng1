/* 独立IMU UDP v1解码和显示状态，不依赖板卡接口。 */
#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

#define IMU_UDP_PORT 5006
#define IMU_PACKET_SIZE 32
#define IMU_STALE_MS 1000

enum imu_decode_result {
    IMU_DECODE_OK = 0, IMU_BAD_LENGTH, IMU_BAD_MAGIC, IMU_BAD_VERSION,
    IMU_BAD_RESERVED, IMU_BAD_STATUS, IMU_BAD_CRC, IMU_BAD_FLOAT,
    IMU_BAD_ERROR_PAYLOAD
};
struct imu_frame { uint32_t sequence; uint8_t status; float radians[3]; };
uint32_t imu_crc32(const uint8_t *bytes, size_t length);
enum imu_decode_result imu_decode(const uint8_t *packet, size_t length,
                                 struct imu_frame *frame);
const char *imu_decode_name(enum imu_decode_result result);
const char *imu_error_name(uint8_t status);

enum imu_state { IMU_WAITING, IMU_LIVE, IMU_DEVICE_ERROR, IMU_INVALID,
                 IMU_OUT_OF_ORDER, IMU_STALE };
struct imu_model {
    int have_sequence, have_angles, seen_packet;
    int simulated; /* 仅用于主机绘制样例，实际接收器初始化为0。 */
    uint32_t sequence, good_sequence;
    uint8_t status;
    double degrees[3];
    enum imu_state state;
    enum imu_decode_result invalid_reason;
    uint64_t started_ms, last_packet_ms, last_good_ms;
    uint64_t good_count, error_count, invalid_count, missing_count, order_count;
};
void imu_model_init(struct imu_model *model, uint64_t now_ms);
int imu_model_push(struct imu_model *model, const uint8_t *packet,
                   size_t length, uint64_t now_ms);
enum imu_state imu_model_state(const struct imu_model *model, uint64_t now_ms);
#endif
