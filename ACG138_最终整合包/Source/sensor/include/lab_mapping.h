#ifndef ACG_MAPPING_LAB_MAPPING_H
#define ACG_MAPPING_LAB_MAPPING_H
#include "lab_types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct lab_mapper lab_mapper;
typedef struct lab_mapping_config {
 double resolution_m,origin_x_m,origin_y_m;
 uint32_t width,height,pose_history_capacity,trace_capacity,allow_uncalibrated;
 uint64_t max_interpolation_gap_us,max_pose_age_us;
 double mount_x_m,mount_y_m,mount_yaw_rad,min_range_m,max_range_m;
} lab_mapping_config;
typedef struct lab_map_view {
 uint32_t width,height,trace_count,readonly,has_pose;
 double resolution_m,origin_x_m,origin_y_m;
 const int8_t *cells; /* 首行为最小Y，首列为最小X，-1表示未知，0至100表示占用概率。 */
 const struct lab_trace_point *trace;
 lab_pose pose;
 lab_context context;
} lab_map_view;
typedef struct lab_trace_point {double x_m,y_m;} lab_trace_point;
typedef struct lab_mapping_stats {
 uint64_t mapped_scans,accepted_poses,pending_replaced,skipped_time,skipped_quality,
 invalid_events,context_rejected,resets;
} lab_mapping_stats;
void lab_mapping_config_default(lab_mapping_config *config);
lab_status lab_mapper_open(lab_mapper **out,const lab_mapping_config *config,const lab_context *context);
/* 仅接收SCAN和POSE，TRUTH即使载荷类似位姿也被拒绝。后到的包围位姿可解锁待处理扫描；AGAIN表示等待位姿，实际建图扫描数见统计。

 */
lab_status lab_mapper_push(lab_mapper *mapper,const lab_event *event);
lab_status lab_mapper_reset(lab_mapper *mapper,const lab_context *new_context);
/* 借用指针在下次push、reset、load或close前有效，非线程安全。 */
lab_status lab_mapper_view(const lab_mapper *mapper,lab_map_view *out);
void lab_mapper_stats(const lab_mapper *mapper,lab_mapping_stats *out);
lab_status lab_mapper_save(const lab_mapper *mapper,const char *new_path);
/* 加载后进入只读预览，需显式reset清除加载数据后恢复写图。 */
lab_status lab_mapper_load(lab_mapper *mapper,const char *path);
lab_status lab_mapper_export_pgm(const lab_mapper *mapper,const char *new_pgm_path);
void lab_mapper_close(lab_mapper *mapper);
#ifdef __cplusplus
}
#endif
#endif
