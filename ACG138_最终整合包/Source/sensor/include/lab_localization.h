#ifndef ACG_LAB_LOCALIZATION_H
#define ACG_LAB_LOCALIZATION_H
#include "lab_live.h"
#ifdef __cplusplus
extern "C" {
#endif
enum lab_real_localization_state {LAB_REAL_LOC_WAITING=0,LAB_REAL_LOC_TRACKING=1,LAB_REAL_LOC_DEGRADED=2,LAB_REAL_LOC_LOST=3};
enum lab_real_localization_mode {LAB_REAL_BUILD_MAP=1,LAB_REAL_FIXED_MAP=2};
typedef struct lab_real_xy {double x_m,y_m;} lab_real_xy;
typedef struct lab_real_grid_view {
 lab_real_context context;
 uint64_t map_id,map_version,correction_version;
 uint32_t width,height,readonly,trace_count;
 double resolution_m,origin_x_m,origin_y_m;
 const int8_t *cells; /* 占用值-1为未知，0至100为概率，首行对应最小Y。 */
 const lab_real_xy *trace;
} lab_real_grid_view;
typedef struct lab_localization_config {
 uint32_t mode,width,height,allow_uncalibrated;
 double resolution_m,origin_x_m,origin_y_m;
 double mount_x_m,mount_y_m,mount_yaw_rad;
 uint64_t max_pose_age_us,max_interpolation_gap_us,parameter_version;
} lab_localization_config;
typedef struct lab_localization_view {
 lab_real_context context;
 uint32_t state,has_pose,quality,reserved;
 uint64_t pose_time_us,last_matched_scan_us,parameter_version;
 lab_pose pose;
 double map_odom_x_m,map_odom_y_m,map_odom_yaw_rad;
 lab_real_grid_view map;
 uint64_t scans,matched,rejected;
 char reason[64];
} lab_localization_view;
typedef struct lab_localization_ops {
 uint32_t api_version,struct_bytes;
 /* 定位用原始里程计与扫描修正，已融合的里程计保持单次融合。 */

 lab_status (*push)(void *user,const lab_live_sample *sample);
 /* 栅格和轨迹为借用内存，在下次修改调用前有效，仅单线程使用。 */
 lab_status (*snapshot)(void *user,uint64_t now_us,lab_localization_view *out);
 /* 返回与快照相同校正坐标系、准确测量时刻的位姿；仅在两端包围时插值，缺失或过期返回AGAIN。
 */
 lab_status (*pose_at)(void *user,uint64_t time_us,lab_real_meta *meta,lab_pose *pose);
 /* 固定地图与初始位姿通过两个独立显式操作设置。 */
 lab_status (*set_map)(void *user,const lab_real_grid_view *map);
 lab_status (*set_initial_pose)(void *user,const lab_pose *pose,uint64_t now_us);
 lab_status (*reset)(void *user,const lab_real_context *context);
 void (*close)(void *user);
} lab_localization_ops;
typedef struct lab_localization_binding {void *user;const lab_localization_ops *ops;} lab_localization_binding;
/* 创建定位实例，绑定资源由创建者释放。 */
lab_status lab_localization_create(lab_localization_binding *out,const lab_localization_config *config,const lab_real_context *context);
void lab_localization_config_default(lab_localization_config *out);
int lab_real_grid_valid(const lab_real_grid_view *view);
#ifdef __cplusplus
}
#endif
#endif
