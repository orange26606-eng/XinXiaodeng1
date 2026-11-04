#include "s1_motion_policy.h"
#include "chassis_private.h"
#include <string.h>
static void stop(ch_receiver *r,uint8_t state,uint8_t reason){
 memset(r->status.target_centi,0,sizeof(r->status.target_centi));r->armed=0;r->deadline_ms=0;
 r->status.state=state;r->status.reason=reason;
 if(r->status.lease==UINT64_MAX){r->fault=1;r->status.reason=CH_RX_LEASE_EXHAUSTED;}
 else ++r->status.lease;
}
static int at(ch_receiver *r,uint64_t now){
 if(r->have_clock&&now<r->now_ms){r->fault=1;stop(r,CH_RX_INHIBITED,CH_RX_FAULT);return CH_TIME;}
 r->now_ms=now;r->have_clock=1;r->status.uptime_ms=(uint32_t)now;
 if(r->status.lease==UINT64_MAX){r->fault=1;stop(r,CH_RX_INHIBITED,CH_RX_LEASE_EXHAUSTED);return CH_READONLY;}
 if(r->deadline_ms&&now>=r->deadline_ms){++r->status.watchdogs;stop(r,CH_RX_EXPIRED,CH_RX_TIMEOUT);}
 return CH_OK;
}
void ch_receiver_init(ch_receiver *r,uint32_t id){if(!r)return;memset(r,0,sizeof(*r));r->status.lease=1;r->status.config_id=id;
 r->status.capabilities=7;/* 线上轮速目标、独立看门狗及代次屏障。 */
 r->status.max_ttl_ms=R6_COMMAND_TTL_MS;r->status.max_rpm_centi=12000;r->status.revision=S1_STATUS_REVISION;}
int ch_receiver_health(ch_receiver *r,int ready,int local,int fault,uint64_t now){
 if(!r||(ready!=0&&ready!=1)||(local!=0&&local!=1)||(fault!=0&&fault!=1))return CH_ARGUMENT;
 int time=at(r,now);if(time!=CH_OK)return time;r->ready=(uint32_t)ready;r->local_stop=(uint32_t)local;r->fault=(uint32_t)fault;
 r->status.fault_mask=(uint8_t)(fault?15:0);
 if((!ready||local||fault)&&r->armed)stop(r,CH_RX_INHIBITED,local?CH_RX_LOCAL_STOP:fault?CH_RX_FAULT:CH_RX_NOT_READY);
 return CH_OK;
}
static int rejected(ch_receiver *r,uint8_t reason){++r->status.rejected;r->status.reason=reason;return CH_READONLY;}
int ch_receiver_accept(ch_receiver *r,const ch_wire_command *c,uint64_t now){
 if(!r||!c)return CH_ARGUMENT;if(at(r,now)!=CH_OK)return CH_TIME;
 uint8_t raw[64];if(ch_command_encode(raw,c)!=CH_OK)return rejected(r,CH_RX_BAD_COMMAND);
 if(c->op==CH_PROBE)return CH_OK;/* 不影响心跳或控制权。 */
 if(c->config_id!=r->status.config_id)return rejected(r,CH_RX_BAD_CONFIG);
 if(c->op==CH_CLAIM){
  if(r->armed||r->local_stop||r->fault||!r->ready)return rejected(r,CH_RX_NOT_READY);
  if(c->lease!=r->status.lease)return rejected(r,CH_RX_BAD_LEASE);
  if(r->claimed&&c->session==r->status.session){uint32_t d=c->sequence-r->status.sequence;if(!d||d>=UINT32_C(0x80000000))return rejected(r,CH_RX_BAD_SEQUENCE);}
  stop(r,CH_RX_CLAIMED,CH_RX_NONE);r->claimed=1;r->have_sequence=1;r->mode=c->mode;r->status.mode=c->mode;r->status.session=c->session;r->status.sequence=c->sequence;++r->status.accepted;return CH_OK;
 }
 if(!r->claimed||c->session!=r->status.session||c->mode!=r->mode)return rejected(r,CH_RX_BAD_SESSION);
 uint32_t advance=c->sequence-r->status.sequence;
 if(r->have_sequence&&(!advance||advance>=UINT32_C(0x80000000)))return rejected(r,CH_RX_BAD_SEQUENCE);
 if(c->op!=CH_STOP&&c->lease!=r->status.lease)return rejected(r,CH_RX_BAD_LEASE);
 if(c->op==CH_STOP||c->op==CH_RELEASE){
  stop(r,c->op==CH_RELEASE?CH_RX_IDLE:CH_RX_STOPPED,CH_RX_NONE);
  r->status.sequence=c->sequence;++r->status.accepted;
  if(c->op==CH_RELEASE){r->claimed=0;r->status.session=0;r->mode=r->status.mode=0;r->have_sequence=0;}return CH_OK;
 }
 if(!r->ready||r->local_stop||r->fault)return rejected(r,r->local_stop?CH_RX_LOCAL_STOP:r->fault?CH_RX_FAULT:CH_RX_NOT_READY);
 if(c->op==CH_ARM){if(r->armed)return rejected(r,CH_RX_BAD_COMMAND);stop(r,CH_RX_ARMED,CH_RX_NONE);r->armed=1;}
 else if(c->op==CH_DRIVE){
  if(!r->armed)return rejected(r,CH_RX_NOT_READY);
  for(int i=0;i<4;++i){int q=c->rpm_centi[i];if(q>r->status.max_rpm_centi||q<-(int)r->status.max_rpm_centi)return rejected(r,CH_RX_BAD_COMMAND);}
  if(now>UINT64_MAX-c->ttl_ms)return rejected(r,CH_RX_BAD_COMMAND);
  memcpy(r->status.target_centi,c->rpm_centi,sizeof(c->rpm_centi));r->deadline_ms=now+c->ttl_ms;r->status.state=CH_RX_DRIVING;r->status.reason=CH_RX_NONE;
 }else return rejected(r,CH_RX_BAD_COMMAND);
 r->status.sequence=c->sequence;++r->status.accepted;return CH_OK;
}
int ch_receiver_sample(ch_receiver *r,uint64_t now,ch_wire_status *out){if(!r||!out)return CH_ARGUMENT;int s=at(r,now);*out=r->status;return s;}
