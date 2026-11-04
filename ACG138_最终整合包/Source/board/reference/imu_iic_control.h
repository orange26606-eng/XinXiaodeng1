/* 仅提供字节协议与会话逻辑，不访问设备。WAIT_LED表示命令确认，不代表传感器校准完成。
 */
#ifndef IMU_IIC_CONTROL_H
#define IMU_IIC_CONTROL_H
#include <stddef.h>
#include <stdint.h>
#define IMU_CONTROL_PORT 5007
#define IMU_CONTROL_BYTES 32
#define IMU_CONTROL_ATTEMPTS 10
#define IMU_CONTROL_RETRY_MS 1000
#define IMU_CONTROL_BOOT_GRACE_MS 5000
enum imu_control_op { IMU_CONTROL_START = 1, IMU_CONTROL_CONFIRM_LED_STOPPED = 2 };
enum imu_control_ack_state { IMU_ACK_WAIT_LED = 1, IMU_ACK_READY = 2,
                             IMU_ACK_FAILED = 3, IMU_ACK_BUSY = 4 };
struct imu_control_ack {
    uint32_t token;
    uint8_t state, error, algorithm, polling_hz;
};
enum imu_control_decode { IMU_CTRL_OK, IMU_CTRL_LENGTH, IMU_CTRL_MAGIC,
    IMU_CTRL_VERSION, IMU_CTRL_RESERVED, IMU_CTRL_STATE, IMU_CTRL_ERROR,
    IMU_CTRL_CONFIG, IMU_CTRL_CRC };
int imu_control_encode_command(uint8_t operation, uint32_t token, uint8_t output[32]);
enum imu_control_decode imu_control_decode_ack(const uint8_t *bytes, size_t length,
                                              struct imu_control_ack *ack);
enum imu_control_phase { IMU_PHASE_STARTING, IMU_PHASE_WAIT_LED,
    IMU_PHASE_CONFIRMING, IMU_PHASE_STREAMING, IMU_PHASE_FAILED };
struct imu_control_session {
    uint32_t token;
    enum imu_control_phase phase;
    uint8_t operation, error;
    unsigned attempts;
    uint64_t first_tx_ms, last_tx_ms;
    int have_tx, timed_out, local_error, busy;
    int resume_existing;
    uint64_t invalid_acks, unrelated_acks;
};
void imu_control_session_init(struct imu_control_session *session,
                              uint32_t token, uint64_t now_ms);
/* 旧令牌恢复需已有确认会话；新会话走正常准备流程。 */

void imu_control_session_resume(struct imu_control_session *session,
                                uint32_t token, uint64_t now_ms);
/* 需要重发同一令牌命令时返回1。含首次最多发送10次，最后再宽限5秒后超时。
 */
int imu_control_session_next_tx(struct imu_control_session *session,
                                uint64_t now_ms, uint8_t output[32]);
/* 调用方完成人工观察或自动静置等待后推进WAIT_LED。 */
int imu_control_session_confirm(struct imu_control_session *session, uint64_t now_ms);
/* 保留原手动流程的兼容入口。 */
int imu_control_session_user_confirm(struct imu_control_session *session,
                                    uint64_t now_ms);
/* 仅在转入STREAMING时返回1，调用方据此丢弃旧角度。 */
int imu_control_session_receive(struct imu_control_session *session,
                               const uint8_t *bytes, size_t length);
void imu_control_session_io_failed(struct imu_control_session *session);
/* 固定以太网、IPv4和UDP线上帧，不含MAC产生的FCS，不执行ARP。 */
#define IMU_CONTROL_ETH_BYTES 74
size_t imu_control_build_ethernet(uint8_t output[74], const uint8_t source_mac[6],
    const uint8_t destination_mac[6], const uint8_t source_ipv4[4],
    const uint8_t destination_ipv4[4], const uint8_t payload[32]);
#endif
