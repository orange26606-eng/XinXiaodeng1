/* 终端优先的纯流程实现，不访问设备。 */
#ifndef IMU_TERMINAL_FLOW_H
#define IMU_TERMINAL_FLOW_H
#include "imu_iic_control.h"
#include "imu_terminal_gate.h"
struct imu_terminal_flow {
    struct imu_control_session control;
    struct imu_terminal_gate gate;
    int stopped;
};
void imu_terminal_flow_init(struct imu_terminal_flow *flow, uint32_t token,
                             int resume, uint64_t now_ms);
int imu_terminal_flow_prompt_needed(const struct imu_terminal_flow *flow);
/* tcflush(TCIFLUSH)成功后、打印提示之前立即调用；操作系统刷新也会丢弃提前输入的不完整终端行。
 */
int imu_terminal_flow_after_input_flush(struct imu_terminal_flow *flow);
int imu_terminal_flow_input(struct imu_terminal_flow *flow, const char *bytes,
                            size_t length, uint64_t now_ms);
int imu_terminal_flow_receive(struct imu_terminal_flow *flow,
                              const uint8_t *bytes, size_t length);
int imu_terminal_flow_next_tx(struct imu_terminal_flow *flow, uint64_t now_ms,
                              uint8_t output[32]);
void imu_terminal_flow_disconnect(struct imu_terminal_flow *flow);
int imu_terminal_flow_may_open_window(const struct imu_terminal_flow *flow);
int imu_terminal_flow_failed(const struct imu_terminal_flow *flow);
#endif
