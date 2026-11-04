/* License and attribution: LICENSES/source_notices/notice_012.txt */








/* License and attribution: LICENSES/source_notices/notice_010.txt */
























#include "sdkcommon.h"
#include "hal/thread.h"

#if defined(_WIN32)
#include "arch/win32/winthread.hpp"
#elif defined(_MACOS)
#include "arch/macOS/thread.hpp"
#elif defined(__GNUC__)
#include "arch/linux/thread.hpp"
#else
#error no threading implemention found for this platform.
#endif


