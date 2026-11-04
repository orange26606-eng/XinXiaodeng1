#ifndef ACG_PHONE_CONTROL_LINK_H
#define ACG_PHONE_CONTROL_LINK_H
#include "reference/robot_protocol.h"
#include "phone_timing.h"
/* 使用AF_PACKET被动复制数据，不绑定UDP，也不持有底盘命令控制权。 */
struct phone_control_link {
 struct robot_model robot;
 uint64_t status_ms,received,rejected;
 uint32_t uptime,config;
 uint16_t status_revision,max_rpm_centi,max_ttl_ms;uint8_t policy_mismatch;
 uint8_t have_status,encoder_mask,fault_mask;
 uint64_t diagnostic_ms;uint32_t diagnostic_uptime;uint32_t raw_sequence[3],raw_age_us[3];
 uint16_t raw_mm[3],effective_duty[4];
 uint8_t have_diagnostic,raw_available,diagnostic_zero,diagnostic_flags,diagnostic_fresh,diagnostic_near,diagnostic_pending,diagnostic_observation;
 uint8_t sonar_valid,sonar_age_bucket[3],sonar_raw_zero_mask,sonar_pending_mask,front_seq_mod4;
};
int phone_link_push(struct phone_control_link*,const uint8_t*,size_t,const uint8_t[6],uint64_t);
int phone_link_ready(const struct phone_control_link*,uint64_t);
#endif
