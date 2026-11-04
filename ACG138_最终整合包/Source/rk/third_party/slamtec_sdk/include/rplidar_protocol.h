/* License and attribution: LICENSES/source_notices/notice_012.txt */








/* License and attribution: LICENSES/source_notices/notice_010.txt */
























#pragma once
#include "sl_lidar_protocol.h"
// RP-Lidar Input Packets

#define RPLIDAR_CMD_SYNC_BYTE        SL_LIDAR_CMD_SYNC_BYTE
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  SL_LIDAR_CMDFLAG_HAS_PAYLOAD


#define RPLIDAR_ANS_SYNC_BYTE1       SL_LIDAR_ANS_SYNC_BYTE1
#define RPLIDAR_ANS_SYNC_BYTE2       SL_LIDAR_ANS_SYNC_BYTE2

#define RPLIDAR_ANS_PKTFLAG_LOOP     SL_LIDAR_ANS_PKTFLAG_LOOP

#define RPLIDAR_ANS_HEADER_SIZE_MASK        SL_LIDAR_ANS_HEADER_SIZE_MASK
#define RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT    SL_LIDAR_ANS_HEADER_SUBTYPE_SHIFT

#if defined(_WIN32)
#pragma pack(1)
#endif

typedef sl_lidar_cmd_packet_t rplidar_cmd_packet_t;
typedef sl_lidar_ans_header_t rplidar_ans_header_t;


#if defined(_WIN32)
#pragma pack()
#endif
