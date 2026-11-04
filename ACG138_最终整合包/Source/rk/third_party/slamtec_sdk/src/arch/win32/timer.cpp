/* License and attribution: LICENSES/source_notices/notice_012.txt */








/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#include "sdkcommon.h"
#include <mmsystem.h>
#pragma comment(lib, "Winmm.lib")

namespace rp{ namespace arch{

static LARGE_INTEGER _current_freq;

void HPtimer_reset()
{
    BOOL ans=QueryPerformanceFrequency(&_current_freq);
    _current_freq.QuadPart/=1000ULL;
    assert(ans);
}

_u64 getHDTimer_us()
{
    LARGE_INTEGER current;
    QueryPerformanceCounter(&current);

    return (_u64)(current.QuadPart / (_current_freq.QuadPart/1000ULL));

}

_u64 getHDTimer()
{
    LARGE_INTEGER current;
    QueryPerformanceCounter(&current);

    return (_u64)(current.QuadPart/_current_freq.QuadPart);
}

BEGIN_STATIC_CODE(timer_cailb)
{
    HPtimer_reset();
}END_STATIC_CODE(timer_cailb)

}}
