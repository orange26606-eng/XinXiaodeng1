#ifndef ACG_SCAN_CORE_H
#define ACG_SCAN_CORE_H
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif
#define SCAN_MAX_POINTS 8192
#define SCAN_CHUNK_POINTS 128
#define SCAN_HEADER 40
#define SCAN_PACKET_MAX (SCAN_HEADER + SCAN_CHUNK_POINTS * 8)
#define MAP_CELLS 240
struct scan_point { uint16_t angle_q14; uint32_t distance_q2; uint8_t quality, flags; };
enum scan_geometry_kind { SCAN_GEOMETRY_RAW=0, SCAN_GEOMETRY_DESKEW_ORIGINS=1 };
struct scan_frame {
    uint32_t session, sequence, period_us;
    uint64_t timestamp_us, received_ms;
    uint16_t count, flags;
    uint8_t source;
    struct scan_point points[SCAN_MAX_POINTS];
    /* 扫描补充数据仅在进程内使用。补偿端点与射线原点采用参考雷达坐标，单位四分之一毫米；无效回波原点为零。 */
    uint32_t geometry_kind;
    int16_t origins_q4[SCAN_MAX_POINTS][2];
    /* 采集量程按原始测量计算，补偿后的极径相对参考原点而非采集原点，允许合法为0。
 */
    uint32_t raw_distance_q2[SCAN_MAX_POINTS];
};
static inline int scan_geometry_valid(const struct scan_frame *f) {
    return f && f->geometry_kind <= SCAN_GEOMETRY_DESKEW_ORIGINS;
}
static inline void scan_ray_origin(const struct scan_frame *f,unsigned i,double *x,double *y) {
    *x=*y=0;
    if(f->geometry_kind==SCAN_GEOMETRY_DESKEW_ORIGINS&&i<f->count){*x=f->origins_q4[i][0]/4000.;*y=f->origins_q4[i][1]/4000.;}
}
/* 量程准入使用原始测量，几何使用校正端点。伴随数据保留最小和最大边界的准确量程判断；重建物理射线仅作带量化容差的一致性检查，不放宽采集阈值。


 */
static inline uint32_t scan_acquisition_distance_q2(const struct scan_frame*f,unsigned i){
    if(!scan_geometry_valid(f)||i>=f->count)return 0;
    return f->geometry_kind==SCAN_GEOMETRY_DESKEW_ORIGINS?f->raw_distance_q2[i]:f->points[i].distance_q2;
}
static inline double scan_acquisition_range_m(const struct scan_frame*f,unsigned i){
    return scan_acquisition_distance_q2(f,i)/4000.;
}
static inline double scan_physical_ray_length_m(const struct scan_frame*f,unsigned i){
    if(!scan_geometry_valid(f)||i>=f->count)return NAN;
    double a=-f->points[i].angle_q14*(6.2831853071795864769/65536.),r=f->points[i].distance_q2/4000.,ox,oy;
    scan_ray_origin(f,i,&ox,&oy);return hypot(r*cos(a)-ox,r*sin(a)-oy);
}
static inline int scan_return_in_range(const struct scan_frame*f,unsigned i,double minimum,double maximum){
    if(!scan_geometry_valid(f)||i>=f->count||!f->points[i].quality||!isfinite(minimum)||!isfinite(maximum)||minimum<=0||maximum<minimum)return 0;
    uint32_t raw=scan_acquisition_distance_q2(f,i);double range=raw/4000.;
    if(!raw||range<minimum||range>maximum)return 0;
    if(f->geometry_kind==SCAN_GEOMETRY_DESKEW_ORIGINS){
        double physical=scan_physical_ray_length_m(f,i);
        /* 端点半径误差±0.125mm，角度误差±pi/65536，原点X和Y各±0.125mm，按三角不等式估计射线长度误差。
 */
        double allowance=.000125*(1.+1.4142135623730951)+(f->points[i].distance_q2/4000.)*(3.14159265358979323846/65536.);
        if(!isfinite(physical)||fabs(physical-range)>allowance+1e-12)return 0;
    }
    return 1;
}
struct scan_slot {
    struct scan_frame frame;
    uint8_t seen[(SCAN_MAX_POINTS + 7) / 8];
    uint64_t started_ms;
    unsigned received;
    int active;
};
struct scan_receiver {
    struct scan_slot slots[2];
    struct scan_frame latest;
    uint32_t retired[4], current_session;
    unsigned retired_count, retired_index;
    uint64_t packets, completed, invalid, dropped, duplicates, expired;
    int have_latest, have_session;
};
struct map_config {
    double half_extent_m, mount_x_m, mount_y_m, yaw_deg;
    double vehicle_width_m, vehicle_length_m;
};
struct local_map {
    // 每次扫描重建：0未知、1观测自由射线、2占用端点。
    uint8_t cells[MAP_CELLS * MAP_CELLS];
    unsigned valid, invalid, zeros, low_quality;
    double nearest[3];
};
uint32_t scan_crc32(const uint8_t *packet, size_t length);
size_t scan_encode(uint8_t *out, size_t capacity, const struct scan_frame *frame,
                   uint16_t offset, uint16_t count);
int scan_receive(struct scan_receiver *state, const uint8_t *packet, size_t length, uint64_t now_ms);
void scan_expire(struct scan_receiver *state, uint64_t now_ms);
void map_point_xy(const struct scan_point *point, const struct map_config *config, double *x, double *y);
int map_cell_xy(double x, double y, const struct map_config *config, int *column, int *row);
int map_config_valid(const struct map_config *config);
void map_build(struct local_map *map, const struct scan_frame *frame, const struct map_config *config);
#ifdef __cplusplus
}
#endif
#endif
