#define _GNU_SOURCE
/* 替换原odometry_runtime接口，仅作为文件读取端，不创建套接字、传感器线程或接管服务。
 */
#include "lab_real_mpusvc_posix.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
static char state_dir[256]="/run/acg138-mpu-service";
static int read_view(mpusvc_view *v){char p[512];if(snprintf(p,sizeof(p),"%s/mpu9250.snapshot",state_dir)>=(int)sizeof(p))return -1;lab_status r=mpusvc_read(p,v);if(r!=LAB_OK||v->source!=LAB_REAL_HARDWARE)return -1;return 0;}
int odom_runtime_start(const char *bind_ip,const char *peer_ip){(void)bind_ip;(void)peer_ip;const char *d=getenv("ACG_PHONE_DIR");if(d&&*d){if(strlen(d)>=sizeof(state_dir))return -1;strcpy(state_dir,d);}fprintf(stderr,"MPU9250 FILE-ONLY panel client: %s; UDP5016 owned by independent service. Closing this panel does not stop sensors.\n",state_dir);return 0;}
void odom_runtime_stop(void){/* 此处不停止传感器服务。 */}
void odom_runtime_imu_snapshot(struct imu_model *out){mpusvc_view v;if(read_view(&v)){memset(out,0,sizeof(*out));out->state=IMU_WAITING;return;}mpusvc_imu_model(&v,mpusvc_now(),out);}
void odom_runtime_snapshot(struct odom_state *out){mpusvc_view v;memset(out,0,sizeof(*out));out->quality=ODOM_WAIT;if(read_view(&v))return;*out=v.odometry;uint64_t now=mpusvc_now();if(!v.have_pose||v.flags&MPUSVC_STOPPED||now<v.pose_received_us||now-v.pose_received_us>250000){out->quality=ODOM_STALE;return;}
 /* 旧轨迹只在与同一位姿快照匹配时附带。 */
 char p[512];snprintf(p,sizeof(p),"%s/odometry.shm",state_dir);struct odom_shared legacy;if(mpusvc_legacy_read(p,&legacy)==LAB_OK&&legacy.state.session==v.session&&legacy.state.accepted==v.odometry.accepted&&legacy.state.stamp_ms==v.odometry.stamp_ms)*out=legacy.state;
}
int odom_shared_read(const char *directory,struct odom_state *out){char path[512];if(!directory||!out||snprintf(path,sizeof(path),"%s/odometry.shm",directory)>=(int)sizeof(path))return -1;struct odom_shared v;if(mpusvc_legacy_read(path,&v)!=LAB_OK)return -1;*out=v.state;return 0;}
