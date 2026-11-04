#ifndef W1_MPUSVC_VENDOR_H
#define W1_MPUSVC_VENDOR_H
/* 只读 MPU9250 适配层，使用独立命名空间。 */
#define odom_default_config svc_odom_default_config
#define odom_load_config svc_odom_load_config
#define odom_init svc_odom_init
#define odom_push svc_odom_push
#define odom_quality_text svc_odom_quality_text
#define odom_json svc_odom_json
#define i9_crc32 svc_i9_crc32
#define i9_decode svc_i9_decode
#define i9_default_config svc_i9_default_config
#define i9_config_valid svc_i9_config_valid
#define i9_init svc_i9_init
#define i9_push svc_i9_push
#define i9_quality_name svc_i9_quality_name
#define mpu_config_load svc_mpu_config_load
#define mpu_adapter_init svc_mpu_adapter_init
#define mpu_adapter_push svc_mpu_adapter_push
#define mpu_adapter_push_select svc_mpu_adapter_push_select
#define mpu_adapter_json svc_mpu_adapter_json
#include "../../third_party/full_mpu9250/board/mpu9250/mpu_adapter.h"
#endif
