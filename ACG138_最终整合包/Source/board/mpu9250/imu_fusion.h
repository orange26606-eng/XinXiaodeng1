#ifndef IMU9250_ENCODER_TEST_FUSION_H
#define IMU9250_ENCODER_TEST_FUSION_H
#include <stdint.h>
#include <stddef.h>
#define I9_PACKET_BYTES 96u
enum {I9_AG_VALID=1,I9_MAG_VALID=2,I9_MPU_READY=4,I9_MAG_READY=8,I9_ENCODERS_READY=16};
enum i9_result {I9_OK=0,I9_BAD_PACKET=-1,I9_OLD_PACKET=-2,I9_GAP=1};
enum i9_quality {I9_WAIT_IMU,I9_CALIBRATING,I9_FUSED,I9_WHEELS_ONLY,I9_WAIT_ENCODER,I9_DATA_GAP};
struct i9_packet {
 uint8_t flags,whoami,mag_id,asa[3],i2c_error,ab,initialized,phase;
 uint32_t sequence,snapshot_us,ag_us,mag_us,reset_epoch;
 int32_t counts[4];int16_t accel[3],temperature,gyro[3],mag[3];
 uint32_t errors,overflows,tx_drops,encoder_errors,ag_sequence,mag_sequence;
};
struct i9_config {
 double wheel_mm,track_mm,counts_per_rev,gyro_weight,cal_seconds;
 int signs[4],left_mask,axes[3]; /* 轴映射：±1为X、±2为Y、±3为Z，保持右手系。 */
};
struct i9_state {
 struct i9_config config;struct i9_packet packet;
 int have_packet,bias_ready,quality,have_ag,have_yaw_reference;
 uint32_t previous_sequence,previous_us,previous_ag_sequence;
 uint64_t time_us,last_ag_time_us;
 int32_t previous_counts[4];
 double q[4],bias[3],cal_sum[3],cal_squared[3];unsigned cal_count;
 double accel[3],gyro[3],mag[3],temperature_c;
 double roll,pitch,yaw,heading,x,y,distance,previous_yaw_reference;
 double rpm[4],rpm_counts[4],rpm_seconds;
 uint64_t accepted,rejected,missing,gaps,cal_resets,accel_rejected;
};
uint32_t i9_crc32(const uint8_t *p,size_t n);
int i9_decode(struct i9_packet *out,const uint8_t *p,size_t n);
void i9_default_config(struct i9_config *config);
int i9_config_valid(const struct i9_config *config);
int i9_init(struct i9_state *s,const struct i9_config *config);
int i9_push(struct i9_state *s,const struct i9_packet *packet);
const char *i9_quality_name(int quality);
#endif
