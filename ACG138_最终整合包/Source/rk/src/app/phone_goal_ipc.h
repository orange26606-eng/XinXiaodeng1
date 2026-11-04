#ifndef PHONE_GOAL_IPC_H
#define PHONE_GOAL_IPC_H
#include <stdint.h>
/* 仅本地AF_UNIX数据报，文件目录由root私有持有，GOAL消息不携带电机命令。
 */
#define PHONE_GOAL_MAGIC UINT32_C(0x31474e50)
typedef struct phone_goal_message {
 uint32_t magic,version,operation,reserved; /* 旧v3 TAG_GOAL将reserved设为tag_id+1，x_m为停靠间距，y_m为0。 */
 uint64_t sent_us,map_id,map_version,sequence;
 double x_m,y_m;
 char session[33];unsigned char padding[7];
 /* 实物预览使用服务端核验的目标身份。 */
 uint64_t object_epoch;int32_t object_track_id;uint32_t object_reserved;
 char association_id[96];
 /* v5 AUTO保存上游原请求有效期，使用RK单调微秒。 */
 uint64_t request_expires_us;
 /* v6 AUTO保存服务端准备屏障代次，其他操作为0。 */
 uint64_t switch_generation;
 /* RK私有IPC回执关联，不属于FPGA报文。HTTP客户端序号独立于IPC排序及请求原始时刻。 */
 uint64_t request_id;
} phone_goal_message;
/* 实物 IPC 的 reserved3 表示 radar_follow；epoch/track 为 0 时仅用于传输，显示端从雷达状态解析目标身份。 */
/* v7实物reserved4表示STATIC_SEARCH_LEFT，epoch和track为0表示未绑定；association_id携带请求类别，不是测量身份或路点。
 */
/* v7的reserved6携带静止搜索类别、颜色和大小，reserved7表示直接选择的实际静止目标，不受PNG或选择ACK限制。
 */
enum phone_navigation_operation {PHONE_NAV_GOAL=1,PHONE_NAV_START=2,PHONE_NAV_CANCEL=3,PHONE_NAV_HEARTBEAT=4,PHONE_NAV_OVERRIDE=5,PHONE_NAV_RESET_MAP=6,PHONE_NAV_TAG_GOAL=7,PHONE_NAV_OBJECT_GOAL=8,PHONE_NAV_AUTO_START=9,PHONE_NAV_OBJECT_PREPARE=10,PHONE_NAV_OBJECT_START=11,PHONE_NAV_OBJECT_CANCEL=12,PHONE_NAV_OBJECT_PAUSE=13,PHONE_NAV_OBJECT_RESUME=14};
#endif
