#ifndef ACG_ODOMETRY_H
#define ACG_ODOMETRY_H
#include <stdint.h>
#include <stddef.h>
#define ODOM_POINTS 256
#define ODOM_MAGIC UINT32_C(0x314d444f)
enum odom_quality {ODOM_WAIT,ODOM_UNCALIBRATED,ODOM_FUSED,ODOM_WHEELS,ODOM_STALE,ODOM_GAP,ODOM_SLIP};
struct odom_config {
 double mm_per_count[4],track_mm,yaw_sign,yaw_offset,correction_tau;
 int signs[4],left_mask,calibrated;
};
struct odom_point { double x,y; };
struct odom_state {
 uint64_t stamp_ms;
 double x,y,heading,total_distance,origin_yaw,uncertainty_m;
 uint32_t quality,points,session,accepted,rejected,gaps,imu_rejected;
 int32_t raw_counts[4];
 uint32_t imu_used,slip,calibrated;
 struct odom_point trace[ODOM_POINTS];
};
struct odom_model {
 struct odom_config config;
 struct odom_state state;
 uint32_t previous[4],sequence,fpga_ms,epoch,imu_token,imu_sequence;
 double previous_imu_yaw;
 int have_packet,have_origin,have_imu;
};
struct odom_shared { uint32_t magic,version,bytes,sequence;struct odom_state state; };
void odom_default_config(struct odom_config *c);
int odom_load_config(struct odom_config *c,const char *path);
void odom_init(struct odom_model *m,const struct odom_config *c);
int odom_push(struct odom_model *m,const uint8_t *packet,size_t bytes,uint64_t now_ms);
const char *odom_quality_text(unsigned quality);
size_t odom_json(char *out,size_t bytes,const struct odom_state *s,uint64_t now_ms);
#endif
