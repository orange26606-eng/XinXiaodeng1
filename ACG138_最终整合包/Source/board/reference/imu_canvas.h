#ifndef IMU_CANVAS_H
#define IMU_CANVAS_H
#include "imu_protocol.h"
#include "imu_iic_control.h"
#define IMU_CANVAS_WIDTH 1040
#define IMU_CANVAS_HEIGHT 560
/* 1.2.0，2026-10-12：中文光栅字形与浅色三卡片布局，绘制完整XRGB8888图像，运行时不依赖字体文件。
 */
void imu_canvas_set_connection(const char *interface_name, const char *bind,
                               const char *peer, unsigned port);
void imu_canvas_draw(uint32_t *pixels, const struct imu_model *model,
                     uint64_t now_ms);
void imu_canvas_draw_control(uint32_t *pixels, const struct imu_model *model,
                             const struct imu_control_session *control, uint64_t now_ms);
int imu_canvas_confirm_hit(double x, double y);
#endif
