#ifndef ACG_CHASSIS_LINUX_H
#define ACG_CHASSIS_LINUX_H
#include "chassis_private.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct ch_linux ch_linux;
/* 只有此接口打开套接字，仅由主程序显式启用的真实后端调用，数据源须为CH_LIVE，命令行不直接调用。
 */
int ch_linux_open(ch_linux **out,const char *interface_name,const ch_context *context,uint32_t config_id);
int ch_linux_poll(ch_linux *transport,uint64_t host_monotonic_us,ch_wire_status *out);
/* 不重放缓存或自动心跳，调用方提供新的源期限，sendto前再次核对。
 */
int ch_linux_send(ch_linux *transport,const ch_wire_command *command,uint64_t source_deadline_us,uint64_t host_monotonic_us);
int ch_linux_diagnostics(ch_linux *transport,uint64_t now_us,ch_wire_diagnostic*out,uint64_t*received_us);
void ch_linux_close(ch_linux *transport);
#ifdef __cplusplus
}
#endif
#endif
