#ifndef DASHBOARD_YOLO_FPS_H
#define DASHBOARD_YOLO_FPS_H
/* 1.1.0：统计实际完成的YOLO帧，与显示FPS分开。 */
#include <stddef.h>
#include <stdint.h>
struct dashboard_yolo_fps {
    uint64_t pid, timestamp_ms, total_frames;
    double fps;
    int seen;
};
int dashboard_yolo_fps_parse(const char *bytes,size_t count,struct dashboard_yolo_fps *out);
int dashboard_yolo_fps_read(const char *path,struct dashboard_yolo_fps *out);
int dashboard_yolo_fps_value(const struct dashboard_yolo_fps *sample,uint64_t now_ms,double *fps);
#endif
