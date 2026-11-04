/* License and attribution: LICENSES/source_notices/notice_015.txt */







/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#pragma once

#include "sl_lidar_driver.h"

#define SL_LIDAR_SDK_VERSION_MAJOR  2
#define SL_LIDAR_SDK_VERSION_MINOR  1
#define SL_LIDAR_SDK_VERSION_PATCH  0
#define SL_LIDAR_SDK_VERSION_SEQ    ((SL_LIDAR_SDK_VERSION_MAJOR << 16) | (SL_LIDAR_SDK_VERSION_MINOR << 8) | SL_LIDAR_SDK_VERSION_PATCH)


#define SL_LIDAR_SDK_VERSION_MK_STR_INDIR(x)  #x
#define SL_LIDAR_SDK_VERSION_MK_STR(x)        SL_LIDAR_SDK_VERSION_MK_STR_INDIR(x)

#define SL_LIDAR_SDK_VERSION        (SL_LIDAR_SDK_VERSION_MK_STR(SL_LIDAR_SDK_VERSION_MAJOR) "." SL_LIDAR_SDK_VERSION_MK_STR(SL_LIDAR_SDK_VERSION_MINOR) "." SL_LIDAR_SDK_VERSION_MK_STR(SL_LIDAR_SDK_VERSION_PATCH))
