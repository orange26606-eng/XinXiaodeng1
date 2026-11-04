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
struct mpu_attitude_selection {unsigned valid,bias_ready,cal_samples,source;double rpy[3],q[4],bias[3];};
typedef int (*mpu_attitude_selector)(void*,const struct i9_packet*,const struct i9_state*,struct mpu_attitude_selection*);
int mpu_adapter_push_select(struct mpu_adapter*,const uint8_t*,size_t,uint64_t,mpu_attitude_selector,void*,struct mpu_attitude_selection*);
int mpu_adapter_push(struct mpu_adapter *a,const uint8_t *data,size_t bytes,uint64_t now_ms);
size_t mpu_adapter_json(const struct mpu_adapter *a,char *out,size_t bytes,uint64_t now_ms);
#endif
