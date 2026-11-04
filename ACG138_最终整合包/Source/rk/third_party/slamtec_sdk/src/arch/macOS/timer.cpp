/* License and attribution: LICENSES/source_notices/notice_021.txt */








/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#include "arch/macOS/arch_macOS.h"


namespace rp{ namespace arch{
_u64 rp_getus()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec*1000000LL + t.tv_nsec/1000;
}
_u64 rp_getms()
{
    struct timespec t;
    t.tv_sec = t.tv_nsec = 0;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec*1000L + t.tv_nsec/1000000L;
}

}}
