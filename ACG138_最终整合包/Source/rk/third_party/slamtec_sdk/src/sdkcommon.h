/* License and attribution: LICENSES/source_notices/notice_012.txt */








/* License and attribution: LICENSES/source_notices/notice_010.txt */
























#if defined(_WIN32)

#include "arch/win32/arch_win32.h"
#elif defined(_MACOS)
#include "arch/macOS/arch_macOS.h"
#elif defined(__GNUC__)
#include "arch/linux/arch_linux.h"
#else
#error "unsupported target"
#endif

#include "hal/types.h"
#include "hal/assert.h"

#include "rplidar.h"

#include "hal/util.h"