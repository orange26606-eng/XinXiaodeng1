/* License and attribution: LICENSES/source_notices/notice_019.txt */







/* License and attribution: LICENSES/source_notices/notice_013.txt */
























#pragma once


#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4200)
#endif

#include "sl_types.h"

#define SL_LIDAR_CMD_SYNC_BYTE              0xA5
#define SL_LIDAR_CMDFLAG_HAS_PAYLOAD        0x80

#define SL_LIDAR_ANS_SYNC_BYTE1             0xA5
#define SL_LIDAR_ANS_SYNC_BYTE2             0x5A

#define SL_LIDAR_ANS_PKTFLAG_LOOP           0x1

#define SL_LIDAR_ANS_HEADER_SIZE_MASK       0x3FFFFFFF
#define SL_LIDAR_ANS_HEADER_SUBTYPE_SHIFT   (30)

#if defined(_WIN32)
#pragma pack(1)
#endif



typedef struct sl_lidar_cmd_packet_t
{
    sl_u8 syncByte; //must be SL_LIDAR_CMD_SYNC_BYTE
    sl_u8 cmd_flag;
    sl_u8 size;
    sl_u8 data[0];
} __attribute__((packed)) sl_lidar_cmd_packet_t;


typedef struct sl_lidar_ans_header_t
{
    sl_u8  syncByte1; // must be SL_LIDAR_ANS_SYNC_BYTE1
    sl_u8  syncByte2; // must be SL_LIDAR_ANS_SYNC_BYTE2
    sl_u32 size_q30_subtype; // see _u32 size:30; _u32 subType:2;
    sl_u8  type;
} __attribute__((packed)) sl_lidar_ans_header_t;

#if defined(_WIN32)
#pragma pack()
#endif


#if defined(_MSC_VER)
#pragma warning(pop)
#endif