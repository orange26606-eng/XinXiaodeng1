/* License and attribution: LICENSES/source_notices/notice_025.txt */







 /*
  *  Sample Data Unpacker System
  *  External Reference and dependencies
  */

  /* License and attribution: LICENSES/source_notices/notice_026.txt */
























#pragma once

#include "sdkcommon.h"
#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/types.h"
#include "hal/assert.h"
#include "hal/locker.h"
#include "hal/socket.h"
#include "hal/event.h"
#include "hal/waiter.h"
#include "hal/byteorder.h"
#include "sl_lidar_driver.h"
#include "sl_crc.h" 
#include <algorithm>
#include <memory>

#define CONF_NO_BOOST_CRC_SUPPORT

#include "dataupacker_namespace.h"


