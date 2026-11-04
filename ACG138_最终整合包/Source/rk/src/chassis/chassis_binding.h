#ifndef ACG_CHASSIS_BINDING_H
#define ACG_CHASSIS_BINDING_H
/* 底盘接口适配层，公开接口见 include/。 */
#include "lab_chassis.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct chassis_hardware_options {
 char interface_name[64];
 uint32_t expected_config_id;
} chassis_hardware_options;
/* 可选一致性检查，无设备访问。公开配置提供ncv1://网口/192.168.0.2:5012?config_id=非零32位数，传输为UDP；选项须与工厂显式端点完全一致。

 */
lab_status chassis_binding_prepare(lab_chassis_binding *binding,const chassis_hardware_options *options);
/* 固定适配器与软件保护载荷，不包含分配器、套接字或传输对象。 */
size_t chassis_binding_private_bytes(const lab_chassis_binding *binding);
#ifdef __cplusplus
}
#endif
#endif
