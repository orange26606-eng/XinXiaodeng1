#ifndef ACG_LOCALIZATION_RANGE_H
#define ACG_LOCALIZATION_RANGE_H
#include "lab_localization.h"
/* 在接收任何扫描或地图前调用的内部设置接口。 */
lab_status lab_real_localization_set_sensor_range(lab_localization_binding*b,double minimum_m,double maximum_m);
#endif
