#ifndef ACG_W3_LOCALIZATION_CANDIDATE_H
#define ACG_W3_LOCALIZATION_CANDIDATE_H
/* 定位模块内部接口，与公开 ABI 和报文格式分开。此层不管理传输、设备、线程或电机输出，保留输入数据来源。 */
#include "lab_types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define W3_LOC_VERSION 1u
#define W3_LOC_POINTS_MAX 512u
#define W3_LOC_HISTORY_MAX 256u
#define W3_LOC_GRID_SIDE_MAX 1024u

enum w3_loc_mode { W3_LOC_REAL=1, W3_LOC_RECORDED_REAL=2, W3_LOC_SYNTHETIC_TEST=3 };
enum w3_loc_clock { W3_LOC_RK_MONOTONIC=1, W3_LOC_REPLAY_CLOCK=2, W3_LOC_TEST_CLOCK=3 };
enum w3_loc_reason {
    W3_LOC_UNINITIALIZED=1, W3_LOC_WAIT_POSE, W3_LOC_MATCHED,
    W3_LOC_TOO_FEW_POINTS, W3_LOC_MATCH_REJECTED, W3_LOC_DEGENERATE,
    W3_LOC_POSE_INVALID, W3_LOC_POSE_GAP, W3_LOC_LOST,
    W3_LOC_STALE, W3_LOC_CONTEXT_REJECTED, W3_LOC_TIME_REJECTED
};
typedef struct w3_loc_context {
    uint32_t mode, clock, timeline_id, origin_epoch;
} w3_loc_context;
typedef struct w3_loc_meta {
    w3_loc_context context;
    uint32_t provider, stream_id, session, sequence, flags;
    uint64_t time_us, received_us;
} w3_loc_meta;
typedef struct w3_loc_pose { double x_m, y_m, yaw_rad; } w3_loc_pose;
typedef struct w3_loc_map {
    uint32_t width, height;
    double resolution_m, origin_x_m, origin_y_m;
    uint64_t map_id, map_version;
    const int8_t *cells; /* 地图占用-1表示未知，0至100表示占用概率，打开时复制。 */
} w3_loc_map;
typedef struct w3_loc_config {
    uint32_t points_per_scan, iterations, min_points, occupancy_threshold;
    uint64_t max_pose_gap_us, max_scan_gap_us, max_pose_age_us;
    double mount_x_m, mount_y_m, mount_yaw_rad, min_range_m, max_range_m;
    double correspondence_m, huber_m, max_rmse_m, min_match_ratio;
    double initial_search_m, initial_search_rad;
    uint32_t allow_uncalibrated;
} w3_loc_config;
typedef struct w3_loc_result {
    w3_loc_pose pose, map_odom;
    lab_pose raw_odometry;
    w3_loc_meta scan_meta;
    uint64_t map_id, map_version, accepted, rejected, pending_replaced;
    uint64_t context_rejected, time_rejected;
    uint32_t has_pose, quality, reason, matched_points, input_points;
    double rmse_m, match_ratio;
    size_t allocated_bytes;
    uint32_t inside_points,outside_points,reference_points,close_points,sector_count;
    double robust_rmse_m,coverage;
} w3_loc_result;
typedef struct w3_localizer w3_localizer;

void w3_loc_config_default(w3_loc_config *out);
lab_status w3_loc_open(w3_localizer **out, const w3_loc_config *config,
                       const w3_loc_context *context, const w3_loc_map *map);
/* 手动地图位姿与原始里程计需对应相同初始时刻。初始化清空历史及旧解，直到reset前始终保持数据源身份屏障，包括手动重新初始化。该结果仅是先验，不是激光匹配成功；拒绝TRUTH及provider 8。


 */
lab_status w3_loc_initialize(w3_localizer *loc, const w3_loc_pose *map_pose,
                             const w3_loc_meta *odom_meta, const lab_pose *raw_odometry);
lab_status w3_loc_push_pose(w3_localizer *loc, const w3_loc_meta *meta, const lab_pose *pose);
/* LS01角度为顺时针Q14，距离为Q2毫米，点深复制，最多一个待处理扫描；帧会话与序号须与meta一致，不外推。
 */
lab_status w3_loc_push_scan(w3_localizer *loc, const w3_loc_meta *meta, const struct scan_frame *frame);
/* 年龄过期时清除返回质量的VALID，错误输出初始化为零。 */
lab_status w3_loc_get_result(const w3_localizer *loc, uint64_t now_us, w3_loc_result *out);
lab_status w3_loc_get_map(const w3_localizer *loc, w3_loc_map *out);
/* 替换不可变地图几何和内容时保留历史位姿及当前地图与里程计变换，完成新分配后再修改旧存储。
 */
lab_status w3_loc_update_map(w3_localizer *loc,const w3_loc_map *map);
/* 显式新时间线或原点清除历史、解和手动先验，保留地图几何和身份；换地图需新建对象。
 */
lab_status w3_loc_reset(w3_localizer *loc, const w3_loc_context *new_context);
const char *w3_loc_reason_text(uint32_t reason);
void w3_loc_close(w3_localizer *loc);
#ifdef __cplusplus
}
#endif
#endif
