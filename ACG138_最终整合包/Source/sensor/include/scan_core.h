#ifndef ACG_SCAN_CORE_H
#define ACG_SCAN_CORE_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define SCAN_MAX_POINTS 8192
#define SCAN_CHUNK_POINTS 128
#define SCAN_HEADER 40
#define SCAN_PACKET_MAX (SCAN_HEADER + SCAN_CHUNK_POINTS * 8)
#define MAP_CELLS 240
struct scan_point { uint16_t angle_q14; uint32_t distance_q2; uint8_t quality, flags; };
struct scan_frame {
    uint32_t session, sequence, period_us;
    uint64_t timestamp_us, received_ms;
    uint16_t count, flags;
    uint8_t source;
    struct scan_point points[SCAN_MAX_POINTS];
};
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
