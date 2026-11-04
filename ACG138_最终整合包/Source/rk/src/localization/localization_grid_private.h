#ifndef ACG_W3_LOCALIZATION_GRID_PRIVATE_H
#define ACG_W3_LOCALIZATION_GRID_PRIVATE_H
/* 定位模块的占用栅格计算。沿用射线、未知区域和命中优先规则，不转换元数据。 */
#include "localization_candidate.h"
typedef struct wl_grid {
    w3_loc_map view;
    int8_t *cells;
    int16_t *scores;
    uint8_t *marks;
    uint32_t *hit_epoch;
    uint8_t *hit_count;
    int8_t probability[2049];
    uint64_t integrated,invalid_points,outside_endpoints;
    uint64_t expansions,growth_limited,geometry_version;
    uint32_t last_scan_session,last_scan_sequence;int have_scan;
    size_t allocated_bytes;
} wl_grid;
lab_status wl_grid_open(wl_grid **out,uint32_t width,uint32_t height,double resolution,
                         double origin_x,double origin_y,uint64_t map_id);
lab_status wl_grid_insert(wl_grid *grid,const w3_loc_config *config,w3_loc_pose pose,const struct scan_frame *scan);
/* 每次最多扩展64格，保留原栅格、世界坐标及地图身份。 */
lab_status wl_grid_expand(wl_grid *grid,double min_x,double min_y,double max_x,double max_y);
void wl_grid_close(wl_grid *grid);
#endif
