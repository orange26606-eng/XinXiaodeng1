#ifndef ACG_W3_LOCALIZATION_INTERNAL_H
#define ACG_W3_LOCALIZATION_INTERNAL_H
#include "localization_candidate.h"
typedef struct wl_sample { w3_loc_meta meta; lab_pose pose; } wl_sample;
typedef struct wl_point { double x,y; } wl_point;
typedef struct wl_reference_job {
    w3_loc_map map;int8_t*cells;float*distance;
    double*line_in,*line_out,*breaks;int*sites;
    uint32_t phase,line;size_t allocated_bytes;
} wl_reference_job;
struct w3_localizer {
    w3_loc_config c; w3_loc_context context; w3_loc_map map;
    int8_t *cells; float *distance;
    double *line_in,*line_out,*breaks; int *sites;
    wl_sample history[W3_LOC_HISTORY_MAX]; uint32_t history_count;
    w3_loc_meta odom_stream,scan_stream,pending_meta;
    struct scan_frame *pending;
    w3_loc_pose initial_map,initial_odom,last_map,last_odom;
    uint64_t initial_time,last_success,received_us,last_scan_time;
    uint32_t last_scan_sequence;
    int initialized,have_odom,have_scan,have_pending,have_delivery,have_success,have_processed;
    w3_loc_result result;
    double correction_weight; /* 内部置信度，不属于公共ABI。 */
    wl_reference_job*reference_job;
};
lab_status wl_push_control_pose(w3_localizer*,const w3_loc_meta*,const lab_pose*);
double wl_wrap(double a);
w3_loc_pose wl_compose(w3_loc_pose a,w3_loc_pose b);
w3_loc_pose wl_relative(w3_loc_pose a,w3_loc_pose b);
void wl_build_field(w3_localizer *s);
/* 协作式建立不可变参考，每个回调处理有限行数。 */
lab_status wl_queue_reference(w3_localizer*s,const w3_loc_map*map);
int wl_reference_work(w3_localizer*s,unsigned line_budget);
void wl_reference_discard(w3_localizer*s);
/* 有界稳健场评分，仅产生候选，不直接接受位姿。 */
double wl_recovery_score(const w3_localizer*s,const wl_point*p,uint32_t n,w3_loc_pose pose);
int wl_match(w3_localizer *s,const wl_point *p,uint32_t n,w3_loc_pose initial,int first,
             w3_loc_pose *out,double *rmse,double *ratio,uint32_t *matches,int *degenerate);
#endif
