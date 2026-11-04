#ifndef ACG_ODOMETRY_RUNTIME_H
#define ACG_ODOMETRY_RUNTIME_H
#include "odometry.h"
#include "reference/imu_protocol.h"
void odom_runtime_imu_snapshot(struct imu_model *model);
int odom_runtime_start(const char *bind_address,const char *peer);
void odom_runtime_stop(void);
void odom_runtime_snapshot(struct odom_state *s);
int odom_shared_read(const char *directory,struct odom_state *s);
#endif
