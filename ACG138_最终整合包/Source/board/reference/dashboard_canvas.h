/* 固定相机与姿态布局，显示有效姿态更新率、最新检测列表，以及三角度和相对航向图。

 */
#ifndef DASHBOARD_CANVAS_H
#define DASHBOARD_CANVAS_H
#include "imu_protocol.h"
#include "dashboard_rate.h"
#include "dashboard_detections.h"
#include "dashboard_heading.h"
#include "robot_protocol.h"
#include "dashboard_yolo_fps.h"
#include "../odometry.h"
void dashboard_odometry_draw(uint32_t *pixels,const struct odom_state *state,uint64_t now_ms);
#define DASHBOARD_WIDTH 1280
#define DASHBOARD_HEIGHT 720
#define DASHBOARD_VIDEO_WIDTH 900
#define DASHBOARD_VIDEO_HEIGHT 506
void dashboard_canvas_draw(uint32_t *pixels, const struct imu_model *model,
    const uint32_t *video, uint64_t video_frames, uint64_t video_last_ms,
    uint64_t now_ms, const struct dashboard_rate *rate,
    const struct dashboard_detections *detections,
    const struct dashboard_heading *heading, const struct robot_model *robot,
    const struct dashboard_yolo_fps *yolo_fps);
#endif
