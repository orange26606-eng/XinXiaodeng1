/* 每帧检测结果采用固定容量。 */
/* 1.1.1，2026-10-15：最近完整日志组的年龄，单位为单调时钟毫秒。 */
#ifndef DASHBOARD_DETECTIONS_H
#define DASHBOARD_DETECTIONS_H
#include <stdint.h>
#include <stddef.h>
#define DASHBOARD_OBJECTS 6
struct dashboard_object { char name[64]; double score; };
struct dashboard_detections {
    uint64_t timestamp_ms, frame;
    unsigned total, count;
    int seen;
    struct dashboard_object objects[DASHBOARD_OBJECTS];
};
struct dashboard_detection_parser {
    char line[4096]; size_t used; int overflow, active;
    unsigned expected, received;
    struct dashboard_detections pending, latest;
};
void dashboard_detections_init(struct dashboard_detection_parser *p);
int dashboard_detections_feed(struct dashboard_detection_parser *p,const char *bytes,size_t size,uint64_t now);
/* 两个本地程序共享快照文字，以原子重命名避免读取半写入文件。 */
int dashboard_detections_read(const char *path,struct dashboard_detections *d);
int dashboard_detections_write(const char *path,const struct dashboard_detections *d);
/* 结果缺失或时间明显超前时返回0；有效期由调用方判断。 */
int dashboard_detections_age_ms(const struct dashboard_detections *d,uint64_t now,uint64_t *age);
const char *dashboard_object_label(const char *name);
#endif
