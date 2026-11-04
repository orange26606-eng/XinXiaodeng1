#define _GNU_SOURCE
/* 传感器接收模块与 REAL 接口的适配层。设备 I/O 由工厂创建，静态初始化阶段不打开设备。 */
#include "lab_live.h"
#include "live_shared_file.h"
#include "live_real_convert.h"
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

typedef struct shared_item {
 struct odom_shared snapshot;
 li_file_identity identity;
 uint64_t read_us,generation;
 lab_status status;
} shared_item;
typedef struct real_binding {
 lab_live_source_config config;
 lab_real_context context;
 li_source *core;
 int fd,thread_started,have_file,have_now,reset_required,calibration_declared;
 struct in_addr peer;
 pthread_t thread;
 pthread_mutex_t mutex;
 atomic_int stop;
 atomic_uint_fast64_t generation;
 shared_item shared;
 li_file_identity identity;
 lab_live_sample *pending;
 unsigned count,head;
 uint64_t now_us,min_source_time,accepted,rejected,gaps,reconnects,last_pose_sequence,last_scan_sequence;
 int have_pose_sequence,have_scan_sequence;
 lab_status file_status;
} real_binding;

static uint64_t host_time(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return UINT64_MAX;return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;}
static void *shared_worker(void *user){
 real_binding *b=user;
 while(!atomic_load_explicit(&b->stop,memory_order_relaxed)){
  uint64_t generation=atomic_load_explicit(&b->generation,memory_order_acquire);
  shared_item next={0};next.generation=generation;
  next.status=li_shared_file_read(b->config.odometry_path,&next.snapshot,&next.identity);
  next.read_us=host_time();if(next.read_us==UINT64_MAX)next.status=LAB_ERR_TIME;
  pthread_mutex_lock(&b->mutex);
  if(generation==atomic_load_explicit(&b->generation,memory_order_relaxed))b->shared=next;
  pthread_mutex_unlock(&b->mutex);
  struct timespec wait={0,10000000};nanosleep(&wait,NULL);
 }
 return NULL;
}
static int usable_pose(const lab_pose *p){return (p->quality&LAB_POSE_VALID)&&!(p->quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP));}
static lab_status enqueue_pose(real_binding *b,const li_pose_sample *p){
 lab_status r=li_real_pose_sample(p,&b->context,b->config.parameter_version,LAB_EVENT_POSE,&b->pending[b->count]);
 if(r!=LAB_OK)return r;++b->count;
 r=li_real_pose_sample(p,&b->context,b->config.parameter_version,LAB_EVENT_ENCODER,&b->pending[b->count]);
 if(r==LAB_OK)++b->count;return r;
}
static lab_status get_shared(real_binding *b,uint64_t now){
 if(!b->config.read_existing_odometry)return LAB_AGAIN;
 if(pthread_mutex_trylock(&b->mutex))return LAB_AGAIN;
 shared_item item=b->shared;pthread_mutex_unlock(&b->mutex);
 if(item.generation!=atomic_load_explicit(&b->generation,memory_order_relaxed)||item.read_us>now)return LAB_AGAIN;
 b->file_status=item.status;if(item.status!=LAB_OK)return LAB_AGAIN;
 if(item.snapshot.state.stamp_ms<=UINT64_MAX/1000&&item.snapshot.state.stamp_ms*1000<b->min_source_time)return LAB_AGAIN;
 if(b->have_file&&(item.identity.device!=b->identity.device||item.identity.inode!=b->identity.inode)){
  b->reset_required=1;++b->reconnects;return LAB_ERR_CONTEXT;
 }
 b->identity=item.identity;b->have_file=1;
 lab_status r=li_push_shared(b->core,&item.snapshot,atomic_load_explicit(&b->generation,memory_order_relaxed),now);
 if(r==LAB_ERR_CONTEXT||r==LAB_ERR_TIME){b->reset_required=1;++b->reconnects;return r;}
 if(r==LAB_OK){li_pose_sample p;if(li_latest_pose(b->core,now,&p)!=LAB_OK)return LAB_ERR_FORMAT;
  if(!b->calibration_declared){p.pose.calibrated=0;p.pose.quality|=LAB_POSE_UNCALIBRATED;}
  if(b->have_pose_sequence){uint32_t d=p.meta.sequence-(uint32_t)b->last_pose_sequence;if(d>1&&d<UINT32_C(0x80000000))b->gaps+=d-1;}
  b->have_pose_sequence=1;b->last_pose_sequence=p.meta.sequence;return enqueue_pose(b,&p);
 }else if(r<0)++b->rejected;
 return LAB_OK;
}
static lab_status get_scans(real_binding *b,uint64_t now){
 if(b->fd<0)return LAB_AGAIN;
 uint8_t bytes[SCAN_PACKET_MAX];
 for(unsigned i=0;i<64;++i){struct sockaddr_in from;socklen_t size=sizeof(from);
  ssize_t n=recvfrom(b->fd,bytes,sizeof(bytes),MSG_DONTWAIT|MSG_TRUNC,(struct sockaddr*)&from,&size);
  if(n<0){if(errno==EINTR)continue;if(errno==EAGAIN||errno==EWOULDBLOCK)break;++b->rejected;return LAB_ERR_IO;}
  if(size!=sizeof(from)||from.sin_family!=AF_INET||from.sin_addr.s_addr!=b->peer.s_addr){++b->rejected;continue;}
  if(b->min_source_time&&n>=SCAN_HEADER&&(size_t)n<=sizeof(bytes)&&!memcmp(bytes,"LS01",4)&&(bytes[7]&1u)){
   uint64_t t=0;for(unsigned j=0;j<8;++j)t=(t<<8)|bytes[16+j];
   uint32_t period=(uint32_t)bytes[32]<<24|(uint32_t)bytes[33]<<16|(uint32_t)bytes[34]<<8|bytes[35];
   if(t<b->min_source_time||(period&&t>=period&&t-period<b->min_source_time)){++b->rejected;continue;}
  }
  lab_status r=li_push_ls01(b->core,bytes,(size_t)n,atomic_load_explicit(&b->generation,memory_order_relaxed),now);
  if(r==LAB_ERR_CONTEXT){b->reset_required=1;++b->reconnects;return r;}
  if(r<0)++b->rejected;
 }
 li_scan_view view;lab_status r=li_latest_scan(b->core,now,&view);if(r!=LAB_OK)return r==LAB_AGAIN?LAB_AGAIN:r;
 if(b->have_scan_sequence&&view.meta.sequence==(uint32_t)b->last_scan_sequence)return LAB_AGAIN;
 if(b->have_scan_sequence){uint32_t d=view.meta.sequence-(uint32_t)b->last_scan_sequence;if(d>1&&d<UINT32_C(0x80000000))b->gaps+=d-1;}
 b->have_scan_sequence=1;b->last_scan_sequence=view.meta.sequence;
 r=li_real_scan_sample(&view,&b->context,b->config.parameter_version,&b->pending[b->count]);
 if(r==LAB_OK)++b->count;return r;
}
static lab_status binding_poll(void *user,uint64_t now,lab_live_sample *out){
 if(out)memset(out,0,sizeof(*out));real_binding *b=user;if(!b||!out)return LAB_ERR_ARGUMENT;
 if(b->reset_required)return LAB_ERR_CONTEXT;
 if(b->have_now&&now<b->now_us){b->reset_required=1;return LAB_ERR_TIME;}
 b->have_now=1;b->now_us=now;
 if(b->head==b->count){
  b->head=b->count=0;
  lab_status r=get_shared(b,now);if(r<0){b->head=b->count=0;return r;}
  r=get_scans(b,now);if(r<0){b->head=b->count=0;return r;}
 }
 if(b->head==b->count)return LAB_AGAIN;
 *out=b->pending[b->head++];if(!lab_live_sample_valid(out)){memset(out,0,sizeof(*out));++b->rejected;return LAB_ERR_FORMAT;}
 ++b->accepted;return LAB_OK;
}
static lab_status binding_health(void *user,uint64_t now,lab_live_health *out){
 if(out)memset(out,0,sizeof(*out));real_binding *b=user;if(!b||!out)return LAB_ERR_ARGUMENT;
 out->context=b->context;out->time_us=now;out->accepted=b->accepted;out->rejected=b->rejected;
 out->gaps=b->gaps;out->reconnects=b->reconnects;
 if(b->have_now&&now<b->now_us){b->reset_required=1;snprintf(out->reason,sizeof(out->reason),"CLOCK_ROLLBACK");return LAB_ERR_TIME;}
 if(b->reset_required){snprintf(out->reason,sizeof(out->reason),"RESET_REQUIRED");return LAB_OK;}
 li_pose_sample pose;li_scan_view scan;
 if(li_latest_pose(b->core,now,&pose)==LAB_OK){
  out->odometry_time_us=out->encoder_time_us=pose.meta.time_us;out->odometry_valid=(uint32_t)usable_pose(&pose.pose);
  if(out->odometry_valid)out->encoder_valid_mask=pose.counts_valid_mask;
 }
 if(li_latest_scan(b->core,now,&scan)==LAB_OK){out->scan_time_us=scan.meta.time_us;
  if(scan.meta.flags&LI_HOST_TIME)for(unsigned i=0;i<scan.frame->count;++i)if(scan.frame->points[i].quality&&scan.frame->points[i].distance_q2){out->scan_valid=1;break;}}
 const char *reason="RAW_IMU_UNAVAILABLE_RX_PROXY";
 if(b->config.read_existing_odometry&&!out->odometry_valid)reason=b->file_status<0?"ODOMETRY_SHARED_UNAVAILABLE":"ODOMETRY_WAIT_OR_STALE";
 else if(b->config.ls01_port&&!out->scan_valid)reason="SCAN_WAIT_STALE_OR_UNMAPPED";
 snprintf(out->reason,sizeof(out->reason),"%s",reason);return LAB_OK;
}
static lab_status binding_reset(void *user,const lab_real_context *ctx){
 real_binding *b=user;if(!b||!ctx||!lab_real_context_valid(ctx)||ctx->clock!=LAB_REAL_MONOTONIC||
  ctx->source!=b->context.source)return LAB_ERR_ARGUMENT;
 if(ctx->timeline_id==b->context.timeline_id&&ctx->origin_epoch==b->context.origin_epoch)return LAB_ERR_CONTEXT;
 uint64_t generation=atomic_load_explicit(&b->generation,memory_order_relaxed)+1;if(!generation)return LAB_ERR_LIMIT;
 li_context c={ctx->timeline_id,ctx->origin_epoch,ctx->source==LAB_REAL_HARDWARE?LI_REAL_READONLY:LI_OFFLINE_TEST,generation,generation};
 lab_status r=li_reset(b->core,&c);if(r!=LAB_OK)return r;
 pthread_mutex_lock(&b->mutex);atomic_store_explicit(&b->generation,generation,memory_order_release);memset(&b->shared,0,sizeof(b->shared));pthread_mutex_unlock(&b->mutex);
 b->context=*ctx;b->head=b->count=0;b->have_now=b->reset_required=b->have_file=b->have_pose_sequence=b->have_scan_sequence=0;
 b->min_source_time=host_time();if(b->min_source_time==UINT64_MAX){b->reset_required=1;return LAB_ERR_TIME;}
 b->file_status=LAB_AGAIN;
 if(b->fd>=0){uint8_t discard[SCAN_PACKET_MAX];for(unsigned i=0;i<64;++i)if(recv(b->fd,discard,sizeof(discard),MSG_DONTWAIT)<0)break;}
 return LAB_OK;
}
static void binding_close(void *user){
 real_binding *b=user;if(!b)return;atomic_store_explicit(&b->stop,1,memory_order_relaxed);
 if(b->thread_started)pthread_join(b->thread,NULL);
 if(b->fd>=0)close(b->fd);pthread_mutex_destroy(&b->mutex);li_close(b->core);free(b->pending);free(b);
}
static const lab_live_source_ops operations={LAB_REAL_API_VERSION,sizeof(lab_live_source_ops),binding_poll,binding_health,binding_reset,binding_close};
lab_status lab_live_source_create(lab_live_source_binding *out,const lab_live_source_config *cfg,const lab_real_context *ctx){
 if(out)memset(out,0,sizeof(*out));if(!out||!cfg||!ctx||!lab_real_context_valid(ctx)||ctx->clock!=LAB_REAL_MONOTONIC||
  (ctx->source!=LAB_REAL_HARDWARE&&ctx->source!=LAB_REAL_TEST_FIXTURE)||cfg->read_existing_odometry>1||cfg->ls01_port>65535||
  !cfg->max_scan_age_us||cfg->max_scan_age_us>1000000||!cfg->max_odometry_age_us||cfg->max_odometry_age_us>1000000||!cfg->parameter_version||
  !memchr(cfg->odometry_path,0,sizeof(cfg->odometry_path))||!memchr(cfg->allowed_source_ip,0,sizeof(cfg->allowed_source_ip))||
  !memchr(cfg->lidar_device,0,sizeof(cfg->lidar_device))||!memchr(cfg->calibration_path,0,sizeof(cfg->calibration_path))||
  (cfg->read_existing_odometry&&!cfg->odometry_path[0]))return LAB_ERR_ARGUMENT;
 if(!cfg->read_existing_odometry&&!cfg->ls01_port)return LAB_ERR_UNAVAILABLE;
 /* 测试输入只读文件，不打开真实传输通道。 */
 if(ctx->source==LAB_REAL_TEST_FIXTURE&&cfg->ls01_port)return LAB_ERR_ARGUMENT;
 struct in_addr peer={0};const char *allowed=cfg->allowed_source_ip[0]?cfg->allowed_source_ip:"127.0.0.1";
 if(cfg->ls01_port&&inet_pton(AF_INET,allowed,&peer)!=1)return LAB_ERR_ARGUMENT;
 struct odom_config declared;odom_default_config(&declared);
 if(cfg->calibration_path[0]&&odom_load_config(&declared,cfg->calibration_path))return LAB_ERR_FORMAT;
 real_binding *b=calloc(1,sizeof(*b));if(!b)return LAB_ERR_NOMEM;
 b->fd=-1;b->config=*cfg;b->context=*ctx;b->peer=peer;b->file_status=LAB_AGAIN;
 b->calibration_declared=cfg->calibration_path[0]&&declared.calibrated;
 atomic_init(&b->stop,0);atomic_init(&b->generation,1);
 if(pthread_mutex_init(&b->mutex,NULL)){free(b);return LAB_ERR_IO;}
 b->pending=calloc(3,sizeof(lab_live_sample));if(!b->pending){binding_close(b);return LAB_ERR_NOMEM;}
 li_config c;li_config_default(&c);c.max_pose_age_us=cfg->max_odometry_age_us;c.max_scan_age_us=cfg->max_scan_age_us;
 li_context lc={ctx->timeline_id,ctx->origin_epoch,ctx->source==LAB_REAL_HARDWARE?LI_REAL_READONLY:LI_OFFLINE_TEST,1,1};
 lab_status r=li_open(&b->core,&c,&lc);if(r!=LAB_OK){binding_close(b);return r;}
 if(cfg->ls01_port){
  b->fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
  struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons((uint16_t)cfg->ls01_port)};
  address.sin_addr.s_addr=(ntohl(peer.s_addr)>>24)==127?htonl(INADDR_LOOPBACK):htonl(INADDR_ANY);
  int buffer=65536;
  if(b->fd<0||setsockopt(b->fd,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer))||bind(b->fd,(struct sockaddr*)&address,sizeof(address))){binding_close(b);return LAB_ERR_IO;}
 }
 if(cfg->read_existing_odometry){
  pthread_attr_t attributes;
  if(pthread_attr_init(&attributes)){binding_close(b);return LAB_ERR_IO;}
  int failure=pthread_attr_setstacksize(&attributes,128u*1024u);
  if(!failure)failure=pthread_create(&b->thread,&attributes,shared_worker,b);
  pthread_attr_destroy(&attributes);if(failure){binding_close(b);return LAB_ERR_IO;}b->thread_started=1;
 }
 out->user=b;out->ops=&operations;return LAB_OK;
}
