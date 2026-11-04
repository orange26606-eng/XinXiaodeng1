#ifndef ACG_CHASSIS_PRIVATE_H
#define ACG_CHASSIS_PRIVATE_H
/* C11 底盘内部接口，与公开 ABI 分开。单实例使用，输出数据由调用方持有；设备 I/O 通过显式调用执行。 */
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define CHASSIS_CANDIDATE_VERSION UINT32_C(0x00010000)
#define CH_COMMAND_BYTES 64u
#define CH_STATUS_BYTES 96u
#define CH_COMMAND_PORT 5012u /* 内部接口，接入前需注册。 */
#define CH_STATUS_PORT 5013u
enum ch_error {CH_OK=0,CH_WAIT=1,CH_ARGUMENT=-1,CH_CONTEXT=-2,CH_TIME=-3,
 CH_CRC=-4,CH_FORMAT=-5,CH_READONLY=-6,CH_LIMIT=-7,CH_IO=-8};
enum ch_source {CH_LIVE=1,CH_SIMULATION=2,CH_REPLAY=3};
enum ch_mode {CH_OFF=0,CH_MANUAL=1,CH_NAVIGATION=2};
enum ch_op {CH_PROBE=0,CH_CLAIM=1,CH_ARM=2,CH_DRIVE=3,CH_STOP=4,CH_RELEASE=5};
enum ch_receiver_state {CH_RX_IDLE=0,CH_RX_CLAIMED=1,CH_RX_ARMED=2,
 CH_RX_DRIVING=3,CH_RX_STOPPED=4,CH_RX_INHIBITED=5,CH_RX_EXPIRED=6};
enum ch_reason {CH_R_OFF=1u,CH_R_DISARMED=2u,CH_R_HEALTH=4u,CH_R_LINK=8u,
 CH_R_POSE=16u,CH_R_SCAN=32u,CH_R_OBSTACLE=64u,CH_R_MOTOR=128u,
 CH_R_EXPIRED=256u,CH_R_ESTOP=512u,CH_R_CLOCK=1024u,CH_R_NO_COMMAND=2048u};
enum ch_rx_reason {CH_RX_NONE=0,CH_RX_BAD_COMMAND=1,CH_RX_BAD_SESSION=2,
 CH_RX_BAD_LEASE=3,CH_RX_BAD_SEQUENCE=4,CH_RX_NOT_READY=5,CH_RX_LOCAL_STOP=6,
 CH_RX_FAULT=7,CH_RX_TIMEOUT=8,CH_RX_BAD_CONFIG=9,CH_RX_LEASE_EXHAUSTED=10,CH_RX_ULTRASONIC_BLOCKED=11};
typedef struct ch_context {uint64_t run_id;uint32_t origin_epoch,source;} ch_context;
typedef struct ch_config {
 double diameter_m[4],track_m,max_linear_m_s,max_angular_rad_s,max_rpm;
 double acceleration_m_s2,angular_acceleration_rad_s2;
 uint32_t left_mask;int motor_polarity[4];
 uint64_t command_age_us,command_ttl_us,health_age_us,link_age_us,pose_age_us,scan_age_us;
 uint64_t max_ramp_us;
 uint32_t config_id,direct_targets;
} ch_config;
typedef struct ch_health {
 ch_context context;uint64_t time_us,link_time_us,pose_time_us,scan_time_us;
 uint32_t link_ok,pose_valid,scan_valid,obstacle_stop,motor_fault;
} ch_health;
typedef struct ch_lease {ch_context context;uint64_t generation;uint32_t mode,session;} ch_lease;
typedef struct ch_request {
 ch_lease lease;uint64_t time_us,expires_us;uint32_t sequence;
 double linear_m_s,angular_rad_s;
} ch_request;
typedef struct ch_output {
 ch_context context;uint64_t time_us,expires_us,generation;
 uint32_t sequence,mode,session,reasons,armed;
 double linear_m_s,angular_rad_s,logical_rpm[4],electrical_rpm[4],limit_scale;
} ch_output;
typedef struct ch_guard ch_guard;
void ch_config_default(ch_config *out);
int ch_config_valid(const ch_config *c);
int ch_kinematics(const ch_config *c,double v,double w,ch_output *out);
int ch_guard_open(ch_guard **out,const ch_config *c,const ch_context *ctx);
int ch_guard_select(ch_guard *g,uint32_t mode,uint32_t session,uint64_t now);
int ch_guard_health(ch_guard *g,const ch_health *h,uint64_t now);
int ch_guard_arm(ch_guard *g,uint64_t now,ch_lease *out);
int ch_guard_get_lease(const ch_guard *g,ch_lease *out);
int ch_guard_submit(ch_guard *g,const ch_request *request,uint64_t now);
/* 输出指针非空时，所有错误都返回零输出，不刷新期限。 */
int ch_guard_tick(ch_guard *g,uint64_t now,ch_output *out);
int ch_guard_estop(ch_guard *g,int active,uint64_t now);
int ch_guard_clear_estop(ch_guard *g,uint64_t now);
int ch_guard_reset(ch_guard *g,const ch_context *ctx);
int ch_guard_view(const ch_guard *g,ch_output *out);
size_t ch_guard_allocated_bytes(const ch_guard *g);
void ch_guard_close(ch_guard *g);

/* 线上单位为有符号执行器RPM×100，主机最终施加时只应用一次drive_sign，固件不再重复旧电机异或；编码器方向仍独立配置。

 */
typedef struct ch_wire_command {
 uint8_t op,mode;uint32_t session,sequence;uint64_t lease;
 uint16_t ttl_ms;int16_t rpm_centi[4];uint32_t config_id;uint8_t intent;
} ch_wire_command;
typedef struct ch_wire_status {
 uint8_t state,capabilities,reason;uint32_t session,sequence;uint64_t lease;
 uint32_t uptime_ms,config_id;int16_t target_centi[4],measured_centi[4];
 int32_t counts[4];uint8_t valid_mask,fault_mask;
 uint16_t max_ttl_ms,max_rpm_centi,revision;
 uint32_t accepted,rejected,watchdogs;
 uint8_t mode;
 uint16_t sonar_mm[3];uint8_t sonar_valid,sonar_fresh,sonar_near,sonar_degraded,sonar_front_stable,local_stop_sources,sonar_pending;uint8_t sonar_age_bucket[3],sonar_raw_zero_mask;
 uint8_t sonar_front_sequence;
} ch_wire_status;

/* 只读伴随数据，不确认命令或刷新NCS时效。 */
typedef struct ch_wire_diagnostic {
 uint32_t config_id,uptime_ms,nonzero_sequence[3],raw_age_us[3];
 uint16_t raw_mm[3],effective_pwm[4];
 uint8_t available_mask,zero_mask,flags,fresh,near,pending,observation_block;
} ch_wire_diagnostic;
int ch_diagnostic_encode(uint8_t out[96],const ch_wire_diagnostic *s);
int ch_diagnostic_decode(ch_wire_diagnostic *out,const uint8_t *data,size_t size);
int ch_ethernet_diagnostic(ch_wire_diagnostic *out,const uint8_t *frame,size_t size);
uint32_t ch_crc32(const void *data,size_t size);
int ch_command_encode(uint8_t out[CH_COMMAND_BYTES],const ch_wire_command *c);
int ch_command_decode(ch_wire_command *out,const uint8_t *data,size_t size);
int ch_status_encode(uint8_t out[CH_STATUS_BYTES],const ch_wire_status *s);
int ch_status_decode(ch_wire_status *out,const uint8_t *data,size_t size);
int ch_output_command(const ch_output *o,uint64_t now,uint64_t receiver_lease,uint32_t config_id,ch_wire_command *out);
/* 以太网帧不含前导码和FCS，适用于AF_PACKET。板内IP固定192.168.0.3到192.168.0.2，与Wi-Fi或SSH地址无关。
 */
int ch_ethernet_command(uint8_t out[106],const uint8_t source_mac[6],const ch_wire_command *c);
int ch_ethernet_status(ch_wire_status *out,const uint8_t *frame,size_t size);

/* 固件接收模型独立持有值和时钟，不使用主机心跳；该模型最大TTL为200ms，相对计时不能保证原始源期限。
 */
typedef struct ch_receiver {
 ch_wire_status status;uint64_t now_ms,deadline_ms;uint32_t have_clock;
 uint32_t claimed,armed,have_sequence,ready,local_stop,fault,mode;
} ch_receiver;
void ch_receiver_init(ch_receiver *r,uint32_t config_id);
int ch_receiver_health(ch_receiver *r,int ready,int local_stop,int fault,uint64_t now_ms);
int ch_receiver_accept(ch_receiver *r,const ch_wire_command *c,uint64_t now_ms);
int ch_receiver_sample(ch_receiver *r,uint64_t now_ms,ch_wire_status *out);

#ifdef __cplusplus
}
#endif
#endif
