#ifndef IMU_TERMINAL_GATE_H
#define IMU_TERMINAL_GATE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 不访问设备，调用方传入标准输入字节，并报告EOF或断开。 */
struct imu_terminal_gate {
    int prompted;
    int confirmed;
    int disconnected;
    int discard_line;
    size_t used;
    char line[32];
};

void imu_terminal_gate_init(struct imu_terminal_gate *gate);

/* 打印提示后启用输入确认。提示前的不完整行一直丢弃到下一个LF，过长行同样处理；没有残行时清除discard_line。



 */
void imu_terminal_gate_prompt(struct imu_terminal_gate *gate);

/* 首次完整接收到y或Y加LF、或CRLF时返回1，拒绝空白及其他字节。不完整行不能确认；确认后锁存，后续输入返回0，空指针或空输入也返回0。




 */
int imu_terminal_gate_feed(struct imu_terminal_gate *gate,
                           const char *bytes, size_t count);

/* 锁存终端EOF或断开并返回1，空确认对象返回0。 */
int imu_terminal_gate_disconnect(struct imu_terminal_gate *gate);

#ifdef __cplusplus
}
#endif

#endif
