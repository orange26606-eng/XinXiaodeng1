#include "control_core.h"
#include <string.h>
uint32_t phone_crc32(const uint8_t *p,unsigned size){uint32_t v=UINT32_MAX;while(size--){v^=*p++;for(unsigned b=0;b<8;b++)v=(v>>1)^((v&1)?UINT32_C(0xedb88320):0);}return ~v;}
static void put32(uint8_t*p,uint32_t x){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(x>>(i*8));}
void phone_wire_packet(uint8_t p[32],unsigned op,unsigned motion,uint32_t token,uint32_t sequence){
    memset(p,0,32);memcpy(p,"MCV1",4);p[4]=1;p[5]=(uint8_t)op;p[6]=(uint8_t)motion;put32(p+8,token);put32(p+12,sequence);put32(p+28,phone_crc32(p,28));
}
void phone_control_session(struct phone_control*c,const char*session,uint32_t token,uint64_t now){
    memset(c,0,sizeof(*c));memcpy(c->session,session,33);c->wire_token=token;c->has_session=1;c->last_client_ms=now;c->last_rx_ms=now;c->view_last_ms=now;c->queued_op=PHONE_MANUAL;c->pending=1;c->stopped=1;
}
uint64_t phone_control_lease_ms(const struct phone_control*c){return c&&c->autonomous?PHONE_AUTONOMOUS_SESSION_MS:PHONE_SESSION_MS;}
int phone_control_live(const struct phone_control*c,const char*session,uint64_t now){
    return c&&session&&c->has_session&&!strcmp(session,c->session)&&!c->control_expired&&now>=c->last_rx_ms&&now-c->last_rx_ms<phone_control_lease_ms(c);
}
int phone_view_live(const struct phone_control*c,const char*session,uint64_t now){
    return c&&session&&c->has_session&&!strcmp(session,c->session)&&now>=c->view_last_ms&&now-c->view_last_ms<PHONE_VIEW_MS;
}
int phone_session_touch(struct phone_control*c,const char*session,uint64_t now,int control_mode){
    if(!phone_view_live(c,session,now))return 401;
    if(control_mode&&!phone_control_live(c,session,now)){c->control_expired=1;return 401;}
    c->view_last_ms=now;
    if(control_mode)c->last_rx_ms=now;
    return 200;
}
int phone_control_accept(struct phone_control*c,const char*session,uint64_t seq,unsigned op,unsigned motion,int ready,int auto_mode,uint64_t now){return phone_control_accept_timed(c,session,seq,op,motion,ready,auto_mode,now,now,op==PHONE_HOLD?now+PHONE_HOLD_MS:0);}
int phone_control_accept_timed(struct phone_control*c,const char*session,uint64_t seq,unsigned op,unsigned motion,int ready,int auto_mode,uint64_t now,uint64_t sent,uint64_t hold_until){
    (void)auto_mode;
    if(!c||!session||!c->has_session||strcmp(session,c->session)||!seq||seq<=c->last_sequence)return 401;
    if(op>PHONE_PING||motion>8||(op==PHONE_HOLD&&(motion<1||motion>8))||(op!=PHONE_HOLD&&motion))return 400;
    if(op==PHONE_AUTO)return 409; /* AUTO由显式自动启动事务管理。 */
    if(op!=PHONE_STOP&&!phone_control_live(c,session,now)){c->control_expired=1;return 401;}
    /* 保留原命令时效与期限，STOP不修改该时钟。 */
    if(op!=PHONE_STOP&&(sent>now||now-sent>=PHONE_SESSION_MS))return 409;
    if(op==PHONE_HOLD&&(hold_until<=now||hold_until<=sent||hold_until-sent>PHONE_HOLD_MS))return 409;
    c->last_sequence=seq; /* 已消费但被拒绝的输入，重试时不能变成有效输入。 */
    if(op==PHONE_HOLD&&!ready){c->motion=0;c->queued_op=PHONE_STOP;c->pending=1;return 409;}
    if(op==PHONE_STOP&&!phone_control_live(c,session,now))c->control_expired=1;
    c->last_client_ms=sent;c->last_rx_ms=now;c->view_last_ms=now;c->client_seen=1;
    if(op==PHONE_HOLD){c->autonomous=0;c->motion=motion;c->last_hold_ms=sent;c->hold_until_ms=hold_until;c->stopped=0;}
    else if(op==PHONE_STOP||op==PHONE_MANUAL){c->autonomous=0;c->motion=0;c->queued_op=op;c->pending=1;c->stopped=1;}
    return 200;
}
void phone_control_tick(struct phone_control*c,int telemetry_ok,uint64_t now,unsigned*op,unsigned*motion){
    *motion=0;
    if(!phone_control_live(c,c->session,now)||!telemetry_ok){if(c->has_session&&(now<c->last_rx_ms||now-c->last_rx_ms>=phone_control_lease_ms(c)))c->control_expired=1;c->motion=0;c->pending=0;c->stopped=1;*op=PHONE_STOP;return;}
    if(c->pending){*op=c->queued_op==PHONE_AUTO?PHONE_STOP:c->queued_op;c->pending=0;return;}
    if(c->motion&&now>=c->last_hold_ms&&now<c->hold_until_ms){*op=PHONE_HOLD;*motion=c->motion;return;}
    if(c->motion){c->motion=0;c->stopped=1;*op=PHONE_STOP;return;}
    *op=PHONE_PING;
}
