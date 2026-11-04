/* 命令和回执状态处理，自动等待与数据就绪检查见imu_auto_start.c。 */
#include "imu_iic_control.h"
#include "imu_protocol.h"
#include <string.h>

static uint32_t get_le32(const uint8_t *p)
{ return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static void put_le32(uint8_t *p, uint32_t value)
{ for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8 * i)); }

int imu_control_encode_command(uint8_t operation, uint32_t token, uint8_t output[32])
{
    if (!output || (operation != IMU_CONTROL_START && operation != IMU_CONTROL_CONFIRM_LED_STOPPED)) return -1;
    memset(output, 0, 32); memcpy(output, "IMC1", 4);
    output[4] = 1; output[5] = operation;
    put_le32(output + 8, token); put_le32(output + 28, imu_crc32(output, 28));
    return 0;
}

enum imu_control_decode imu_control_decode_ack(const uint8_t *p, size_t length,
                                              struct imu_control_ack *ack)
{
    if (!p || !ack || length != 32) return IMU_CTRL_LENGTH;
    if (memcmp(p, "IMA1", 4)) return IMU_CTRL_MAGIC;
    if (p[4] != 1) return IMU_CTRL_VERSION;
    if (p[7]) return IMU_CTRL_RESERVED;
    for (unsigned i = 14; i < 28; ++i) if (p[i]) return IMU_CTRL_RESERVED;
    if (p[5] < 1 || p[5] > 4) return IMU_CTRL_STATE;
    if (p[6] > 6) return IMU_CTRL_ERROR;
    if (imu_crc32(p, 28) != get_le32(p + 28)) return IMU_CTRL_CRC;
    if (p[5] == IMU_ACK_WAIT_LED || p[5] == IMU_ACK_READY) {
        if (p[6]) return IMU_CTRL_ERROR;
        if (p[12] != 9 || p[13] != 50) return IMU_CTRL_CONFIG;
    } else {
        if (p[12] || p[13]) return IMU_CTRL_CONFIG;
        if ((p[5] == IMU_ACK_FAILED && !p[6]) || (p[5] == IMU_ACK_BUSY && p[6]))
            return IMU_CTRL_ERROR;
    }
    struct imu_control_ack decoded = {get_le32(p + 8), p[5], p[6], p[12], p[13]};
    *ack = decoded;
    return IMU_CTRL_OK;
}

void imu_control_session_init(struct imu_control_session *s, uint32_t token, uint64_t now_ms)
{
    memset(s, 0, sizeof(*s)); s->token = token; s->phase = IMU_PHASE_STARTING;
    s->operation = IMU_CONTROL_START; s->first_tx_ms = s->last_tx_ms = now_ms;
}

void imu_control_session_resume(struct imu_control_session *s, uint32_t token, uint64_t now_ms)
{ imu_control_session_init(s, token, now_ms); s->resume_existing = 1; }

int imu_control_session_next_tx(struct imu_control_session *s, uint64_t now_ms,
                               uint8_t output[32])
{
    if (s->phase != IMU_PHASE_STARTING && s->phase != IMU_PHASE_CONFIRMING) return 0;
    if (s->attempts >= IMU_CONTROL_ATTEMPTS) {
        if (now_ms - s->last_tx_ms >= IMU_CONTROL_BOOT_GRACE_MS) {
            s->timed_out = 1; s->phase = IMU_PHASE_FAILED;
        }
        return 0;
    }
    if (s->have_tx && now_ms - s->last_tx_ms < IMU_CONTROL_RETRY_MS) return 0;
    if (imu_control_encode_command(s->operation, s->token, output)) return 0;
    if (!s->have_tx) s->first_tx_ms = now_ms;
    s->have_tx = 1; s->last_tx_ms = now_ms; ++s->attempts;
    return 1;
}

int imu_control_session_confirm(struct imu_control_session *s, uint64_t now_ms)
{
    if (s->phase != IMU_PHASE_WAIT_LED) return 0;
    s->phase = IMU_PHASE_CONFIRMING; s->operation = IMU_CONTROL_CONFIRM_LED_STOPPED;
    s->attempts = 0; s->have_tx = 0; s->first_tx_ms = s->last_tx_ms = now_ms;
    return 1;
}

int imu_control_session_user_confirm(struct imu_control_session *s, uint64_t now_ms)
{ return imu_control_session_confirm(s, now_ms); }

int imu_control_session_receive(struct imu_control_session *s, const uint8_t *bytes, size_t length)
{
    struct imu_control_ack ack;
    if (imu_control_decode_ack(bytes, length, &ack) != IMU_CTRL_OK) { ++s->invalid_acks; return 0; }
    if (ack.token != s->token) { ++s->unrelated_acks; return 0; }
    if (s->phase == IMU_PHASE_STREAMING || s->phase == IMU_PHASE_FAILED) return 0;
    if (ack.state == IMU_ACK_FAILED || ack.state == IMU_ACK_BUSY) {
        s->phase = IMU_PHASE_FAILED; s->error = ack.error; s->busy = ack.state == IMU_ACK_BUSY;
        return 0;
    }
    if (s->phase == IMU_PHASE_STARTING && ack.state == IMU_ACK_WAIT_LED) {
        s->phase = IMU_PHASE_WAIT_LED; return 0;
    }
    if (s->phase == IMU_PHASE_STARTING && s->resume_existing && ack.state == IMU_ACK_READY) {
        s->phase = IMU_PHASE_STREAMING; return 1;
    }
    if (s->phase == IMU_PHASE_CONFIRMING && ack.state == IMU_ACK_READY) {
        s->phase = IMU_PHASE_STREAMING; return 1;
    }
    return 0; /* 阶段不符的READY不能绕过调用方的WAIT条件。 */
}

void imu_control_session_io_failed(struct imu_control_session *s)
{ s->phase = IMU_PHASE_FAILED; s->local_error = 1; }

size_t imu_control_build_ethernet(uint8_t output[74], const uint8_t source_mac[6],
    const uint8_t destination_mac[6], const uint8_t source_ipv4[4],
    const uint8_t destination_ipv4[4], const uint8_t payload[32])
{
    if (!output || !source_mac || !destination_mac || !source_ipv4 || !destination_ipv4 || !payload) return 0;
    memset(output, 0, 74); memcpy(output, destination_mac, 6); memcpy(output + 6, source_mac, 6);
    output[12] = 0x08; output[13] = 0x00;
    uint8_t *ip = output + 14;
    ip[0] = 0x45; ip[2] = 0; ip[3] = 60; ip[6] = 0x40; ip[8] = 64; ip[9] = 17;
    memcpy(ip + 12, source_ipv4, 4); memcpy(ip + 16, destination_ipv4, 4);
    unsigned sum = 0;
    for (unsigned i = 0; i < 20; i += 2) sum += ((unsigned)ip[i] << 8) | ip[i + 1];
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    uint16_t check = (uint16_t)~sum; ip[10] = (uint8_t)(check >> 8); ip[11] = (uint8_t)check;
    uint8_t *udp = ip + 20;
    udp[0] = udp[2] = (uint8_t)(IMU_CONTROL_PORT >> 8);
    udp[1] = udp[3] = (uint8_t)IMU_CONTROL_PORT;
    udp[5] = 40; /* UDP校验和保持0，IPv4允许该设置。 */
    memcpy(udp + 8, payload, 32);
    return 74;
}
