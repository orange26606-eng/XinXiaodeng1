#ifndef MPUSVC_FPGA_ATTITUDE_H
#define MPUSVC_FPGA_ATTITUDE_H
#include "lab_real_mpusvc_vendor.h"
#include <stddef.h>
#include <stdint.h>
#define F6_CACHE_CAPACITY 64u
#define F6_MAX_AGE_US UINT64_C(50000)
#define F6_MAX_RX_AGE_US UINT64_C(50000)
enum f6_mode {F6_MODE_RK=0,F6_MODE_AUTO=1,F6_MODE_FPGA=2};
enum f6_source {F6_SOURCE_UNKNOWN=0,F6_SOURCE_RK=1,F6_SOURCE_FPGA=2,F6_SOURCE_UNAVAILABLE=3};
typedef struct f6_attitude {
 uint32_t epoch,sequence,source_us,generated_us,raw_sequence,latest_source_us,age_us;
 uint32_t i2c_errors,math_errors,gaps,tx_drops;uint16_t cal_samples;
 uint8_t flags,status,whoami,i2c_error;double q[4],rpy[3],bias_lsb[3];
} f6_attitude;
typedef struct f6_cache_entry {f6_attitude sample;uint64_t received_us;unsigned used;} f6_cache_entry;
typedef struct f6_selector {
 f6_cache_entry cache[F6_CACHE_CAPACITY];unsigned cursor,mode,source;
 uint64_t accepted,rejected,crc_errors,duplicates,reorders,matches,misses,stale,source_switches,raw_wait_us;
 uint64_t last_received_us,paired_received_us;f6_attitude last,paired;
 uint32_t raw_epoch,raw_ag_sequence,raw_source_us;uint64_t raw_received_us;
 unsigned have_last,have_yaw,have_rk_yaw,selected_valid;double yaw_offset,last_yaw,previous_rk_yaw;
 double selected_rpy[3],selected_q[4];unsigned selected_cal,selected_bias;
 char reason[64];int axes[3];
} f6_selector;
int f6_decode(f6_attitude*,const uint8_t*,size_t);
void f6_selector_init(f6_selector*,unsigned,const int[3]);
void f6_selector_clear(f6_selector*);
int f6_selector_push(f6_selector*,const uint8_t*,size_t,uint64_t);
const f6_cache_entry*f6_selector_find(const f6_selector*,const struct i9_packet*,uint64_t);
int f6_selector_hook(void*,const struct i9_packet*,const struct i9_state*,struct mpu_attitude_selection*);
const char*f6_source_name(unsigned);
#endif
