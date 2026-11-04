/* 1.1.0：会话航向参考和二维方向独立于传感器校准。 */
#ifndef DASHBOARD_HEADING_H
#define DASHBOARD_HEADING_H
#include "imu_protocol.h"
struct dashboard_heading {
    int ready;
    double reference_degrees, relative_degrees;
    uint64_t last_good_count;
};
void dashboard_heading_init(struct dashboard_heading *heading);
/* FPGA进入READY且丢弃排队旧角度后再调用。第一笔新鲜、有效显示样本建立参考，仅在建立时返回1；故障或采集重启保留该参考。

 */
int dashboard_heading_update(struct dashboard_heading *heading,
                             const struct imu_model *model, uint64_t now_ms);
/* 右手坐标图，+X向右、+Y向上，正偏航逆时针；未建立参考时返回0，不推算距离或位置。
 */
int dashboard_heading_vector(const struct dashboard_heading *heading,
                             double *x, double *y);
#endif
