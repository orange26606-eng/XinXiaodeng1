#ifndef W2_LIVE_REAL_CONVERT_H
#define W2_LIVE_REAL_CONVERT_H
/* 内部数据转换接口；公开接口见 lab_live.h。 */
#include "lab_live.h"
#include "live_sensor.h"
lab_status li_real_pose_sample(const li_pose_sample *pose,const lab_real_context *context,
                              uint64_t parameter_version,uint32_t type,lab_live_sample *out);
lab_status li_real_scan_sample(const li_scan_view *scan,const lab_real_context *context,
                              uint64_t parameter_version,lab_live_sample *out);
#endif
