#ifndef ACG_SIM_ENGINE_PRIVATE_H
#define ACG_SIM_ENGINE_PRIVATE_H
/* 仿真模块内部接口，不属于冻结的跨模块ABI或文件格式。 */
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define SIM_MAX_POINTS 8192u
#define SIM_OD01_BYTES 64u
#define SIM_MAX_SEGMENTS 32u
enum sim_path { SIM_STATIONARY, SIM_STRAIGHT, SIM_TURN, SIM_ARC, SIM_CIRCLE, SIM_RECTANGLE };
struct sim_config {
    enum sim_path path;
    uint32_t seed, period_us, scan_period_us, scan_points, start_ms;
    uint32_t scan_session, origin_epoch, imu_token;
    uint64_t duration_us;
    double speed_m_s, yaw_rate_rad_s, rectangle_leg_m, room_width_m, room_height_m;
    double wheel_diameter_m, track_m;
    unsigned counts_per_revolution, left_mask;
    uint32_t imu_period_us;
    int encoder_sign[4];
    uint32_t initial_counts[4];
    double mount_x_m, mount_y_m, mount_yaw_rad;
    double range_min_m, range_max_m;
    double encoder_scale[4], encoder_noise_counts;
    double imu_noise_rad, imu_drift_rad_per_s, imu_bias_rad;
    double range_noise_m, range_dropout_probability;
    double imu_dropout_start_s, imu_dropout_duration_s;
    int freeze_wheel;
    double wheel_freeze_start_s, wheel_freeze_duration_s;
    double imu_spike_start_s, imu_spike_duration_s, imu_spike_rad;
};
struct sim_truth {
    uint64_t elapsed_us;
    double x_m, y_m, yaw_rad, unwrapped_yaw_rad, distance_m;
    double wheel_travel_m[4];
};
struct sim_sample {
    uint64_t timestamp_us;
    uint32_t sequence, origin_epoch, imu_sequence, imu_token;
    uint32_t counts[4], imu_age_ms;
    float imu_yaw_rad;
    uint8_t imu_status;
    uint8_t od01[SIM_OD01_BYTES];
};
struct sim_scan_point {
    uint16_t angle_q14;
    uint32_t distance_q2;
    uint8_t quality, flags;
};
struct sim_scan {
    uint64_t timestamp_us;
    uint32_t sequence, session, period_us;
    uint16_t count;
    struct sim_scan_point points[SIM_MAX_POINTS];
};
struct sim_segment {double x0_m,y0_m,x1_m,y1_m;};
struct sim_engine {
    struct sim_config config;
    struct sim_truth truth;
    struct sim_sample sample;
    struct sim_segment segments[SIM_MAX_SEGMENTS];
    unsigned segment_count;
    uint32_t rng_encoder, rng_imu, rng_lidar, scan_sequence;
    uint64_t last_imu_us;
    double measured_wheel_counts[4];
    float last_imu_yaw;
    /* 可选内部世界查询，为空时保留预设线段场景。 */
    double (*world_raycast)(void *user,double x_m,double y_m,double heading_rad);
    void *world_user;
};
void sim_default_config(struct sim_config *config, enum sim_path path);
const char *sim_path_name(enum sim_path path);
double sim_duration_s(enum sim_path path);
int sim_validate_config(const struct sim_config *config);
/* init创建t=0样本，step有新样本返回1，结束返回0。 */
int sim_init(struct sim_engine *engine, const struct sim_config *config);
int sim_step(struct sim_engine *engine);
/* 内部调度先推进真实物理状态，再对指定传感器采样。 */
int sim_advance_to(struct sim_engine *engine, uint64_t elapsed_us);
/* 内部命令驱动积分支持轮行程与地面位移不同以模拟打滑，不采样传感器，也不改变预设轨迹调度。
 */
int sim_advance_wheels(struct sim_engine *engine, uint64_t elapsed_us,
    double left_wheel_m, double right_wheel_m,
    double left_ground_m, double right_ground_m);
int sim_observe(struct sim_engine *engine, int encoder_new, int imu_new);
/* 不推进时间，同一所需扫描时刻最多调用一次，避免反复重抽噪声。 */
int sim_scan_now(struct sim_engine *engine, struct sim_scan *scan);
double sim_raycast(const struct sim_engine *engine, double x_m, double y_m, double heading_rad);
double sim_wrap_rad(double value);
/* 沿用小端加CRC的OD01线上格式，不新增协议。 */
void sim_pack_od01(const struct sim_sample *sample, uint8_t packet[SIM_OD01_BYTES]);
#ifdef __cplusplus
}
#endif
#endif
