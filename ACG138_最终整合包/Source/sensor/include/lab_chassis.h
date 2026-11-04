#ifndef ACG_LAB_CHASSIS_H
#define ACG_LAB_CHASSIS_H
#include "lab_live.h"
#ifdef __cplusplus
extern "C" {
#endif
enum lab_chassis_backend {LAB_CHASSIS_DRY_RUN=1,LAB_CHASSIS_HARDWARE=2};
enum lab_chassis_transport {LAB_CHASSIS_TRANSPORT_NONE=0,LAB_CHASSIS_TRANSPORT_UDP=1,LAB_CHASSIS_TRANSPORT_UART=2,LAB_CHASSIS_TRANSPORT_IPC=3};
enum lab_chassis_capability {LAB_CHASSIS_TARGET_RPM=1u,LAB_CHASSIS_LOCAL_WATCHDOG=2u,LAB_CHASSIS_FEEDBACK=4u};
typedef struct lab_real_command {
 lab_real_context context;
 double linear_m_s,angular_rad_s;
 uint64_t generated_us,expires_us,control_session,lease;
 uint32_t sequence,enabled;
} lab_real_command;
typedef struct lab_chassis_config {
 uint32_t backend,left_mask,protocol_version,reserved;
 uint32_t transport,reserved_transport;
 char endpoint[256]; /* 空值或NONE用于无输出试运行，不猜测电机目标地址。 */
 double wheel_diameter_m[4],track_m,max_wheel_rpm,max_linear_m_s,max_angular_rad_s;
 int32_t drive_sign[4];
 uint64_t receive_watchdog_us,max_command_age_us;
} lab_chassis_config;
typedef struct lab_chassis_health {
 lab_real_context context;
 uint64_t now_us,scan_time_us,pose_time_us,link_time_us;
 uint32_t scan_valid,pose_valid,link_valid,obstacle_stop,motor_fault,estop;
} lab_chassis_health;
typedef struct lab_chassis_output {
 lab_real_command command;
 double wheel_rpm[4]; /* 逻辑前进为正，传输层只应用一次极性映射。 */
 uint32_t reasons,armed,backend,reserved;
 char reason[64];
} lab_chassis_output;
typedef struct lab_chassis_feedback {
 lab_real_context context;
 uint64_t time_us,control_session;
 uint32_t backend,capabilities,armed,faults,accepted_sequence,applied_sequence;
 double actual_rpm[4];
 char reason[64];
} lab_chassis_feedback;
typedef struct lab_chassis_ops {
 uint32_t api_version,struct_bytes;
 lab_status (*feedback)(void *user,uint64_t now_us,lab_chassis_feedback *out);
 /* 纯软件保护，不发送字节，不使能硬件或驱动电机。 */
 lab_status (*protect)(void *user,const lab_chassis_health *health,const lab_real_command *request,lab_chassis_output *out);
 /* 硬件状态转换仅供执行模式显式调用。 */
 lab_status (*set_enabled)(void *user,uint32_t enabled,uint64_t control_session,uint64_t now_us);
 /* 独立核对期限、数据源、会话、序号及设备能力。 */
 lab_status (*apply)(void *user,const lab_chassis_output *output,uint64_t now_us);
 lab_status (*stop)(void *user,uint64_t now_us);
 lab_status (*reset)(void *user,const lab_real_context *context);
 void (*close)(void *user);
} lab_chassis_ops;
typedef struct lab_chassis_binding {void *user;const lab_chassis_ops *ops;} lab_chassis_binding;
/* 创建底盘实例时保持禁用，不修改已有电机服务。 */
lab_status lab_chassis_create(lab_chassis_binding *out,const lab_chassis_config *config,const lab_real_context *context);
void lab_chassis_config_default(lab_chassis_config *out);
int lab_real_command_valid(const lab_real_command *command,uint64_t now_us);
#ifdef __cplusplus
}
#endif
#endif
