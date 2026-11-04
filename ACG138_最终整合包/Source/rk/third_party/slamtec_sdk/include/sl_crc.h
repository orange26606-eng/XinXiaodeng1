/* License and attribution: LICENSES/source_notices/notice_014.txt */







/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#pragma once

#include "sl_lidar_cmd.h"

namespace sl {namespace crc32 {
    sl_u32 bitrev(sl_u32 input, sl_u16 bw);//reflect
    void init(sl_u32 poly); // table init
    sl_u32 cal(sl_u32 crc, void* input, sl_u16 len);
    sl_result getResult(sl_u8 *ptr, sl_u32 len);
}}
