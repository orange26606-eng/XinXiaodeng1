/* License and attribution: LICENSES/source_notices/notice_012.txt */








/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#pragma once

#include "hal/types.h"

#define delay(x)   ::Sleep(x)

namespace rp{ namespace arch{
    void HPtimer_reset();
    _u64 getHDTimer();
    _u64 getHDTimer_us();

}}

#define getms()   rp::arch::getHDTimer()
#define getus()   rp::arch::getHDTimer_us()
