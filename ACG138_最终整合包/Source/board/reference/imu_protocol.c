#include "imu_protocol.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "IMU v1 requires IEEE-754 binary32 float");

static uint32_t little_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

uint32_t imu_crc32(const uint8_t *bytes, size_t length)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (UINT32_C(0xEDB88320) & (0u - (crc & 1u)));
    }
    return ~crc;
}

enum imu_decode_result imu_decode(const uint8_t *p, size_t length,
                                 struct imu_frame *frame)
{
    struct imu_frame decoded = {0};
    if (length != IMU_PACKET_SIZE) return IMU_BAD_LENGTH;
    if (memcmp(p, "IMU1", 4)) return IMU_BAD_MAGIC;
    if (p[4] != 1) return IMU_BAD_VERSION;
    if (p[6] || p[7] || p[24] || p[25] || p[26] || p[27])
        return IMU_BAD_RESERVED;
    if (p[5] > 5) return IMU_BAD_STATUS;
    if (imu_crc32(p, 28) != little_u32(p + 28)) return IMU_BAD_CRC;
    decoded.sequence = little_u32(p + 8);
    decoded.status = p[5];
    for (unsigned axis = 0; axis < 3; ++axis) {
        uint32_t bits = little_u32(p + 12 + axis * 4);
        if (decoded.status && bits) return IMU_BAD_ERROR_PAYLOAD;
        memcpy(&decoded.radians[axis], &bits, sizeof(bits));
        if (!decoded.status && !isfinite(decoded.radians[axis]))
            return IMU_BAD_FLOAT;
    }
    *frame = decoded;  /* 拒绝报文后不进行部分状态更新。 */
    return IMU_DECODE_OK;
}

const char *imu_decode_name(enum imu_decode_result result)
{
    static const char *const names[] = {
        "OK", "LENGTH", "MAGIC", "VERSION", "RESERVED", "STATUS",
        "CRC32", "NAN/INF", "NONZERO ERROR PAYLOAD"
    };
    return (unsigned)result < sizeof(names) / sizeof(names[0]) ?
           names[result] : "UNKNOWN";
}

const char *imu_error_name(uint8_t status)
{
    static const char *const names[] = {
        "OK", "NACK", "CLOCK TIMEOUT", "BUS BUSY", "BAD COMMAND", "DATA INVALID"
    };
    return status < sizeof(names) / sizeof(names[0]) ? names[status] : "UNKNOWN";
}

void imu_model_init(struct imu_model *model, uint64_t now_ms)
{
    memset(model, 0, sizeof(*model));
    model->started_ms = now_ms;
    model->state = IMU_WAITING;
}

int imu_model_push(struct imu_model *model, const uint8_t *packet,
                   size_t length, uint64_t now_ms)
{
    struct imu_frame frame;
    model->seen_packet = 1;
    model->last_packet_ms = now_ms;
    enum imu_decode_result result = imu_decode(packet, length, &frame);
    if (result != IMU_DECODE_OK) {
        ++model->invalid_count;
        model->invalid_reason = result;
        model->state = IMU_INVALID;
        return 0;
    }
    if (model->have_sequence) {
        uint32_t delta = frame.sequence - model->sequence;
        if (!delta || delta >= UINT32_C(0x80000000)) {
            ++model->order_count;
            model->state = IMU_OUT_OF_ORDER;
            return 0;
        }
        model->missing_count += delta - 1u;
    }
    model->have_sequence = 1;
    model->sequence = frame.sequence;
    model->status = frame.status;
    if (frame.status) {
        ++model->error_count;
        model->state = IMU_DEVICE_ERROR;
        return 0;
    }
    for (unsigned axis = 0; axis < 3; ++axis)
        model->degrees[axis] = (double)frame.radians[axis] * (180.0 / 3.14159265358979323846);
    model->have_angles = 1;
    model->good_sequence = frame.sequence;
    model->last_good_ms = now_ms;
    ++model->good_count;
    model->state = IMU_LIVE;
    return 1;
}

enum imu_state imu_model_state(const struct imu_model *model, uint64_t now_ms)
{
    /* 原始9250接收器在RK中校准，收到新的校准数据时界面继续等待，不复用旧模块READY握手。
 */
    if (model->seen_packet && model->state == IMU_WAITING &&
        now_ms >= model->last_packet_ms && now_ms-model->last_packet_ms < IMU_STALE_MS)
        return IMU_WAITING;
    if (!model->seen_packet)
        return now_ms - model->started_ms < 6000u ? IMU_WAITING : IMU_STALE;
    if (now_ms - model->last_packet_ms >= IMU_STALE_MS ||
        now_ms - (model->have_angles ? model->last_good_ms : model->started_ms) >= IMU_STALE_MS)
        return IMU_STALE;
    return model->state;
}
