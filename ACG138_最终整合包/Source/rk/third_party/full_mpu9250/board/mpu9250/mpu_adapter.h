#ifndef MPU9250_FULL_ADAPTER_H
#define MPU9250_FULL_ADAPTER_H
#include "imu_fusion.h"
#include "../odometry.h"
#include "../reference/imu_protocol.h"
struct mpu_adapter {
 struct i9_state fusion;struct odom_model odom;struct imu_model imu;
 struct odom_config odom_config;struct i9_config fusion_config;
 uint32_t epoch,session;uint64_t received_ms,packet_errors,restarts;
 int have_epoch;
};
int mpu_config_load(struct i9_config *config,const char *path);
int mpu_adapter_init(struct mpu_adapter *a,const struct odom_config *odom,const struct i9_config *fusion);
int mpu_adapter_push(struct mpu_adapter *a,const uint8_t *data,size_t bytes,uint64_t now_ms);
size_t mpu_adapter_json(const struct mpu_adapter *a,char *out,size_t bytes,uint64_t now_ms);
#endif
