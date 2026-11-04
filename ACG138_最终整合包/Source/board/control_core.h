#ifndef PHONE_CONTROL_CORE_H
#define PHONE_CONTROL_CORE_H
#include <stdint.h>
#include "phone_timing.h"
enum phone_op {PHONE_STOP=0,PHONE_MANUAL=1,PHONE_AUTO=2,PHONE_HOLD=3,PHONE_PING=4};
struct phone_control {
    char session[33];
    uint64_t last_sequence,last_client_ms,last_hold_ms,hold_until_ms,last_rx_ms,view_last_ms;
    uint32_t wire_token,wire_sequence;
    unsigned motion,queued_op;
    int has_session,client_seen,pending,stopped,control_expired,autonomous;
};
void phone_control_session(struct phone_control*,const char*,uint32_t,uint64_t);
int phone_control_accept(struct phone_control*,const char*,uint64_t,unsigned,unsigned,int,int,uint64_t);
int phone_control_accept_timed(struct phone_control*,const char*,uint64_t,unsigned,unsigned,int,int,uint64_t,uint64_t,uint64_t);
/* 控制活跃时间使用服务端接收时的单调毫秒时钟；sent_ms只用于命令与保持截止时间。 */

uint64_t phone_control_lease_ms(const struct phone_control*);
int phone_control_live(const struct phone_control*,const char*,uint64_t);
int phone_view_live(const struct phone_control*,const char*,uint64_t);
int phone_session_touch(struct phone_control*,const char*,uint64_t,int);
void phone_control_tick(struct phone_control*,int,uint64_t,unsigned*,unsigned*);
void phone_wire_packet(uint8_t[32],unsigned,unsigned,uint32_t,uint32_t);
uint32_t phone_crc32(const uint8_t*,unsigned);
#endif
