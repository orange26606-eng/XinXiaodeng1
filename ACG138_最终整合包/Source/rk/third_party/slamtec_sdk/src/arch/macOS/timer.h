/* License and attribution: LICENSES/source_notices/notice_021.txt */








/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#pragma once

#include "rptypes.h"

#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

// TODO: the highest timer interface should be clock_gettime
namespace rp{ namespace arch{

_u64 rp_getus();
_u64 rp_getms();

}}

#define getms() rp::arch::rp_getms()
#define getus() rp::arch::rp_getus()
