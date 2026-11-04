/* License and attribution: LICENSES/source_notices/notice_021.txt */








/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#pragma once

// libc dep
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <stdarg.h>

// libc++ dep
#include <iostream>
#include <string>


// POSIX specific
extern "C" {
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <time.h>
}

#include "arch/macOS/timer.h"

