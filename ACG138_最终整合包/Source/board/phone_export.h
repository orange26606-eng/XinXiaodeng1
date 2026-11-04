#ifndef PHONE_EXPORT_H
#define PHONE_EXPORT_H
#include "reference/dashboard_video.h"
#include "reference/imu_protocol.h"
#include "reference/robot_protocol.h"
void phone_export_raw(const struct dashboard_video_layout*,const void*,size_t,uint64_t);
void phone_export_state(const struct imu_model*,const struct robot_model*,uint64_t);
#endif
