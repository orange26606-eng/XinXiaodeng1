#define _GNU_SOURCE
#include "s1_motion_policy.h"
#include "chassis_linux.h"
#include "continuous_policy.h"
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/filter.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdio.h>
struct ch_linux {int fd;struct sockaddr_ll address;uint8_t mac[6];ch_context context;
 uint32_t config_id,verified,have_status;uint64_t status_us,last_host,last_drive_us,last_drive_diag;ch_wire_status status;ch_wire_diagnostic diagnostic;uint64_t diagnostic_us;uint32_t have_diagnostic;};
static int now_us(uint64_t *out){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return CH_IO;
 *out=(uint64_t)t.tv_sec*UINT64_C(1000000)+(uint64_t)t.tv_nsec/1000;return CH_OK;}
int ch_linux_open(ch_linux **out,const char *name,const ch_context *ctx,uint32_t config_id){
 if(!out)return CH_ARGUMENT;*out=NULL;if(!name||!ctx||ctx->source!=CH_LIVE||!ctx->run_id||!ctx->origin_epoch||!config_id||strlen(name)>=IFNAMSIZ)return CH_ARGUMENT;
 ch_linux *t=calloc(1,sizeof(*t));if(!t)return CH_LIMIT;t->fd=-1;t->context=*ctx;t->config_id=config_id;
 t->fd=socket(AF_PACKET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,htons(ETH_P_IP));if(t->fd<0){free(t);return CH_IO;}
 /* 进入用户态前先过滤，避免相机RAW流量填满该套接字。 */
 struct sock_filter instructions[]={
  BPF_STMT(BPF_LD|BPF_H|BPF_ABS,12),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,0x0800,0,13),
  BPF_STMT(BPF_LD|BPF_B|BPF_ABS,14),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,0x45,0,11),
  BPF_STMT(BPF_LD|BPF_B|BPF_ABS,23),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,17,0,9),
  BPF_STMT(BPF_LD|BPF_H|BPF_ABS,36),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,CH_STATUS_PORT,0,7),
  BPF_STMT(BPF_LD|BPF_W|BPF_ABS,26),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,UINT32_C(0xc0a80002),0,5),
  BPF_STMT(BPF_LD|BPF_W|BPF_ABS,30),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,UINT32_C(0xc0a80003),0,3),
  BPF_STMT(BPF_LD|BPF_H|BPF_ABS,34),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,CH_STATUS_PORT,0,1),
  BPF_STMT(BPF_RET|BPF_K,65535),BPF_STMT(BPF_RET|BPF_K,0)};
 struct sock_fprog filter={(unsigned short)(sizeof(instructions)/sizeof(instructions[0])),instructions};
 int receive_bytes=65536;
 if(setsockopt(t->fd,SOL_SOCKET,SO_ATTACH_FILTER,&filter,sizeof(filter))||
    setsockopt(t->fd,SOL_SOCKET,SO_RCVBUF,&receive_bytes,sizeof(receive_bytes))){ch_linux_close(t);return CH_IO;}
 struct ifreq req;memset(&req,0,sizeof(req));memcpy(req.ifr_name,name,strlen(name));
 if(ioctl(t->fd,SIOCGIFINDEX,&req)){ch_linux_close(t);return CH_IO;}t->address.sll_ifindex=req.ifr_ifindex;
 if(ioctl(t->fd,SIOCGIFHWADDR,&req)){ch_linux_close(t);return CH_IO;}memcpy(t->mac,req.ifr_hwaddr.sa_data,6);
 t->address.sll_family=AF_PACKET;t->address.sll_protocol=htons(ETH_P_IP);t->address.sll_halen=6;
 const uint8_t mac[6]={0,10,53,1,254,192};memcpy(t->address.sll_addr,mac,6);
 if(bind(t->fd,(struct sockaddr*)&t->address,sizeof(t->address))){ch_linux_close(t);return CH_IO;}*out=t;return CH_OK;
}
int ch_linux_poll(ch_linux *t,uint64_t now,ch_wire_status *out){
 if(!t||!out)return CH_ARGUMENT;
 /* 调用方时刻早于send()读取的CLOCK_MONOTONIC，比较新传输时刻，调用方时间单独保留。
 */
 uint64_t actual=0;
 if(now_us(&actual)!=CH_OK){t->verified=0;return CH_IO;}
 if(actual<now||actual<t->last_host){
  fprintf(stderr,"CHASSIS_POLL_DIAG result=%d reason=HOST_TIME_BACKWARDS supplied_us=%llu actual_us=%llu last_host_us=%llu\n",CH_TIME,(unsigned long long)now,(unsigned long long)actual,(unsigned long long)t->last_host);
  t->verified=0;return CH_TIME;
 }
 t->last_host=actual;int found=0;
 for(int i=0;i<32;++i){uint8_t frame[2048];ssize_t n=recv(t->fd,frame,sizeof(frame),MSG_DONTWAIT);
  if(n<0){if(errno==EAGAIN||errno==EWOULDBLOCK)break;return CH_IO;}
  ch_wire_status status;
  static const uint8_t broadcast[6]={255,255,255,255,255,255};
  ch_wire_diagnostic diagnostic;
  if(ch_ethernet_diagnostic(&diagnostic,frame,(size_t)n)==CH_OK){
   if(diagnostic.config_id==t->config_id&&(!memcmp(frame,t->mac,6)||!memcmp(frame,broadcast,6))){
    if(!t->have_diagnostic||((uint32_t)(diagnostic.uptime_ms-t->diagnostic.uptime_ms)>0&&(uint32_t)(diagnostic.uptime_ms-t->diagnostic.uptime_ms)<UINT32_C(0x80000000))){
     t->diagnostic=diagnostic;t->diagnostic_us=actual;t->have_diagnostic=1;
    }
   }continue;
  }
  if(ch_ethernet_status(&status,frame,(size_t)n)!=CH_OK||(memcmp(frame,t->mac,6)&&memcmp(frame,broadcast,6)))continue;
  if(t->have_status){uint32_t elapsed=status.uptime_ms-t->status.uptime_ms;
   if(elapsed>=UINT32_C(0x80000000)){t->verified=0;continue;}
   if(!elapsed&&status.sequence==t->status.sequence&&status.lease==t->status.lease)continue;
   if(status.lease<t->status.lease){t->verified=0;continue;}}
  t->status=status;t->status_us=actual;t->have_status=1;t->verified=status.config_id==t->config_id&&
    (status.capabilities&7)==7&&status.revision==S1_STATUS_REVISION&&status.max_ttl_ms==R6_COMMAND_TTL_MS&&status.max_rpm_centi==12000;found=1;
 }
 if(!found)return CH_WAIT;*out=t->status;return t->verified?CH_OK:CH_READONLY;
}
int ch_linux_send(ch_linux *t,const ch_wire_command *c,uint64_t deadline,uint64_t supplied_now){
 if(!t||!c)return CH_ARGUMENT;uint64_t actual=0;int saved_errno=0;
#define TRACE_SEND(code,why) do {int trace_rc=(code);if(c->op!=CH_DRIVE||trace_rc!=CH_OK)fprintf(stderr,"CHASSIS_SEND_DIAG op=%u result=%d reason=%s errno=%d error=%s now_us=%llu supplied_us=%llu last_host_us=%llu status_age_us=%llu have_status=%u verified=%u session_match=%u mode_match=%u lease_match=%u config_match=%u\n",(unsigned)c->op,trace_rc,(why),saved_errno,saved_errno?strerror(saved_errno):"NONE",(unsigned long long)actual,(unsigned long long)supplied_now,(unsigned long long)t->last_host,(unsigned long long)(t->have_status&&actual>=t->status_us?actual-t->status_us:UINT64_MAX),t->have_status,t->verified,(unsigned)(t->have_status&&c->session==t->status.session),(unsigned)(t->have_status&&c->mode==t->status.mode),(unsigned)(t->have_status&&c->lease==t->status.lease),(unsigned)(c->config_id==t->config_id));return trace_rc;}while(0)
 if(now_us(&actual)!=CH_OK){saved_errno=errno;TRACE_SEND(CH_IO,"CLOCK_READ_FAILED");}
 if(actual<supplied_now||actual<t->last_host)TRACE_SEND(CH_TIME,"HOST_TIME_BACKWARDS");t->last_host=actual;
 if(c->op!=CH_PROBE){
  if(!t->verified||actual<t->status_us||actual-t->status_us>=CONT_FEEDBACK_US||c->config_id!=t->config_id)TRACE_SEND(CH_READONLY,"STATUS_NOT_VERIFIED_STALE_OR_CONFIG");
  if(c->op!=CH_STOP&&c->lease!=t->status.lease)TRACE_SEND(CH_READONLY,"LEASE_MISMATCH");
  if(c->op!=CH_CLAIM&&(c->session!=t->status.session||c->mode!=t->status.mode))TRACE_SEND(CH_READONLY,"SESSION_OR_MODE_MISMATCH");
 }
 ch_wire_command fresh=*c;
 if(c->op==CH_DRIVE){
  if(t->status.state!=CH_RX_ARMED&&t->status.state!=CH_RX_DRIVING)TRACE_SEND(CH_READONLY,"DRIVE_NOT_ARMED");
  if(deadline<=actual||(deadline-actual)/1000==0)TRACE_SEND(CH_TIME,"DRIVE_DEADLINE_EXPIRED");
  uint64_t remaining=(deadline-actual)/1000;
  if(fresh.ttl_ms>remaining)fresh.ttl_ms=(uint16_t)remaining;
  for(int i=0;i<4;++i)if(c->rpm_centi[i]>t->status.max_rpm_centi||c->rpm_centi[i]<-(int)t->status.max_rpm_centi)TRACE_SEND(CH_LIMIT,"RPM_LIMIT");
 }
 uint8_t frame[106];int s=ch_ethernet_command(frame,t->mac,&fresh);if(s!=CH_OK)TRACE_SEND(s,"ENCODE_FAILED");
 ssize_t n=sendto(t->fd,frame,sizeof(frame),MSG_DONTWAIT,(struct sockaddr*)&t->address,sizeof(t->address));
 if(n<0)saved_errno=errno;
 if(c->op==CH_DRIVE&&n==(ssize_t)sizeof(frame)){uint64_t gap=t->last_drive_us&&actual>=t->last_drive_us?actual-t->last_drive_us:0;t->last_drive_us=actual;
  if(!t->last_drive_diag||actual-t->last_drive_diag>=1000000||gap>150000){t->last_drive_diag=actual;fprintf(stderr,"DRIVE_TIMING sequence=%u actual_send_us=%llu supplied_us=%llu deadline_us=%llu remaining_ttl_ms=%u inter_send_us=%llu status_age_us=%llu\n",c->sequence,(unsigned long long)actual,(unsigned long long)supplied_now,(unsigned long long)deadline,fresh.ttl_ms,(unsigned long long)gap,(unsigned long long)(actual-t->status_us));}}
 TRACE_SEND(n==(ssize_t)sizeof(frame)?CH_OK:(n<0&&(saved_errno==EAGAIN||saved_errno==EWOULDBLOCK)?CH_WAIT:CH_IO),n==(ssize_t)sizeof(frame)?"SENDTO_OK":"SENDTO_FAILED");
#undef TRACE_SEND
}
int ch_linux_diagnostics(ch_linux*t,uint64_t now,ch_wire_diagnostic*out,uint64_t*received){
 if(!t||!out||!received)return CH_ARGUMENT;
 uint64_t actual=0;if(now_us(&actual)!=CH_OK||actual<now)return CH_TIME;
 if(!t->have_diagnostic||!t->have_status||!t->verified||actual<t->diagnostic_us||actual-t->diagnostic_us>=CONT_FEEDBACK_US)return CH_WAIT;
 int32_t delta=(int32_t)(t->diagnostic.uptime_ms-t->status.uptime_ms);if(delta>250||delta< -250)return CH_WAIT;
 *out=t->diagnostic;*received=t->diagnostic_us;return CH_OK;
}
void ch_linux_close(ch_linux *t){if(t){if(t->fd>=0)close(t->fd);free(t);}}
#else
struct ch_linux {int unused;};
int ch_linux_open(ch_linux **out,const char *name,const ch_context *ctx,uint32_t id){(void)name;(void)ctx;(void)id;if(out)*out=NULL;return CH_READONLY;}
int ch_linux_poll(ch_linux *t,uint64_t now,ch_wire_status *out){(void)t;(void)now;(void)out;return CH_READONLY;}
int ch_linux_send(ch_linux *t,const ch_wire_command *c,uint64_t deadline,uint64_t now){(void)t;(void)c;(void)deadline;(void)now;return CH_READONLY;}
int ch_linux_diagnostics(ch_linux*t,uint64_t now,ch_wire_diagnostic*out,uint64_t*received){(void)t;(void)now;(void)out;(void)received;return CH_READONLY;}
void ch_linux_close(ch_linux *t){free(t);}
#endif
