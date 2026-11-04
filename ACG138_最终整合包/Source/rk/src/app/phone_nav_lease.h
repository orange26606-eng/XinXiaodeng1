#ifndef ACG_PHONE_NAV_LEASE_H
#define ACG_PHONE_NAV_LEASE_H
#include <stddef.h>
#include <stdint.h>

/* 仅在导航事件循环线程调用，调用方先认证HTTP请求并核对手机会话。
 */
typedef struct phone_nav_lease {
 char session[33];
 uint64_t last_seen_us, last_sequence;
 int claimed, motion_active;
} phone_nav_lease;
enum phone_nav_lease_result { PHONE_NAV_OK=0, PHONE_NAV_DENIED=-1, PHONE_NAV_STOP_REQUIRED=-2 };
void phone_nav_lease_init(phone_nav_lease *l);
int phone_nav_lease_claim(phone_nav_lease *l,const char *session,uint64_t now_us);
int phone_nav_lease_heartbeat(phone_nav_lease *l,const char *session,uint64_t sequence,uint64_t now_us);
/* 已验证请求保留原起点用于期限和撤销屏障，持有者租约按RK接收时钟刷新；不能因HTTP排队而丢弃更新请求。

 */
int phone_nav_lease_receive(phone_nav_lease*,const char*,uint64_t sequence,uint64_t origin_us,uint64_t received_us,uint64_t revoked_us,int may_claim);
int phone_nav_lease_start(phone_nav_lease *l,const char *session,uint64_t sequence,uint64_t now_us,int drive_capable);
int phone_nav_lease_stop(phone_nav_lease *l,const char *session,uint64_t sequence,uint64_t now_us);
int phone_nav_lease_tick(phone_nav_lease *l,uint64_t now_us,int phone_server_ok,int manual_override);
#endif
