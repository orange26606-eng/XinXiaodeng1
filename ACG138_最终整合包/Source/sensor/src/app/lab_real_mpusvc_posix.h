#ifndef W1_MPUSVC_POSIX_H
#define W1_MPUSVC_POSIX_H
#include "lab_real_mpusvc.h"
typedef struct mpusvc_options {
 char state_dir[256],bind_ip[64],peer_ip[64],odom_path[256],imu_path[256],record_path[256];
 uint64_t duration_us,parameter_version;unsigned attitude_mode;
} mpusvc_options;
uint64_t mpusvc_now(void);
int mpusvc_run_posix(const mpusvc_options *options,const struct odom_config *odom,const struct i9_config *imu);
#endif
