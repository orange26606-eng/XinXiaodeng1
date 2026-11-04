#define _GNU_SOURCE
#include "lab_real_snapshot.h"
#include "lab_real_mpusvc.h"
#include "continuous_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <arpa/inet.h>
#include <errno.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
typedef struct snapshot_io {lab_real_snapshot *core;lab_real_mpu_config config;uint64_t parameter_version;uint64_t core_config_delay_min,core_config_delay_max;uint32_t core_config_drift_ppm;int scan_fd;struct in_addr peer;char path[512],history_path[512];mpusvc_history_cursor history;uint64_t polled,last_log,file_errors,history_errors,last_export,last_instance;int have_polled;lab_status file_status;} snapshot_io;
static uint64_t monotonic_us(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return UINT64_MAX;return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;}
static lab_status poll_source(void *u,uint64_t now,lab_live_sample *out){
 snapshot_io *s=u;if(!s||!out)return LAB_ERR_ARGUMENT;lab_status r=lab_real_snapshot_take(s->core,now,out);if(r!=LAB_AGAIN)return r;if(lab_real_snapshot_pending(s->core))return LAB_AGAIN;
 /* 先取完队列中最多3个现有事件，队列空后每次只送一行历史，避免将256次导出塞入8项队列。
 */
 if(access(s->history_path,F_OK)==0){
  for(unsigned skipped=0;skipped<32;skipped++){
   mpusvc_view v;uint8_t bytes[768];mpusvc_history_cursor next=s->history;
   if(!next.instance&&s->last_instance){next.instance=s->last_instance;next.sequence=s->last_export;}
   s->file_status=mpusvc_history_next(s->history_path,&next,bytes,&v);
   if(s->file_status==LAB_AGAIN)break;
   if(s->file_status==LAB_ERR_CONTEXT){
    /* 新服务历史有不同文件头标识，读取真实最新行，使核心显式产生复位，不吞掉上下文切换。
 */
    s->file_status=mpusvc_read(s->path,&v);
    if(s->file_status==LAB_OK){s->last_instance=v.instance;s->last_export=v.export_sequence;s->file_status=mpusvc_encode_timed(bytes,&v);if(s->file_status==LAB_OK)s->file_status=lab_real_snapshot_feed(s->core,bytes,sizeof(bytes),now);r=lab_real_snapshot_take(s->core,now,out);if(r!=LAB_AGAIN)return r;}break;
   }
   if(s->file_status<0){++s->history_errors;if(s->file_status==LAB_ERR_NOMEM)return s->file_status;if(next.sequence>s->history.sequence){s->history=next;continue;}break;}
   /* 新服务tick可能晚于回调固定的now，此时保留读取游标，下一回调再消费该真实行。
 */
   if(v.published_us>now||v.raw_received_us>now)break;
   s->history=next;s->last_instance=v.instance;s->last_export=v.export_sequence;
   s->file_status=lab_real_snapshot_feed(s->core,bytes,sizeof(bytes),now);
   if(s->file_status==LAB_ERR_CONTEXT||s->file_status==LAB_ERR_NOMEM)return s->file_status;
   if(s->file_status<0)++s->file_errors;
   r=lab_real_snapshot_take(s->core,now,out);if(r!=LAB_AGAIN)return r;
  }
 }else if(!s->have_polled||now!=s->polled){
  s->polled=now;s->have_polled=1;mpusvc_view v;s->file_status=mpusvc_read(s->path,&v);
  if(s->file_status==LAB_OK){uint8_t b[768];s->file_status=mpusvc_encode_timed(b,&v);if(s->file_status==LAB_OK)s->file_status=lab_real_snapshot_feed(s->core,b,sizeof(b),now);if(s->file_status>=0){s->last_instance=v.instance;s->last_export=v.export_sequence;}}
  if(s->file_status==LAB_ERR_CONTEXT||s->file_status==LAB_ERR_NOMEM)return s->file_status;
  if(s->file_status<0)++s->file_errors;
  r=lab_real_snapshot_take(s->core,now,out);if(r!=LAB_AGAIN)return r;
 }
 if(s->scan_fd>=0)for(unsigned i=0;i<128;++i){uint8_t b[64+SCAN_PACKET_MAX];struct sockaddr_in peer;socklen_t len=sizeof(peer);
  ssize_t n=recvfrom(s->scan_fd,b,sizeof(b),MSG_DONTWAIT|MSG_TRUNC,(struct sockaddr*)&peer,&len);
  if(n<0){if(errno==EINTR)continue;if(errno==EAGAIN||errno==EWOULDBLOCK)break;return LAB_ERR_IO;}
  if(len!=sizeof(peer)||peer.sin_family!=AF_INET||peer.sin_addr.s_addr!=s->peer.s_addr||(size_t)n>sizeof(b))continue;
  uint64_t rx=monotonic_us();if(rx==UINT64_MAX)return LAB_ERR_TIME;r=lab_real_snapshot_scan(s->core,b,(size_t)n,rx);if(r==LAB_ERR_CONTEXT||r==LAB_ERR_NOMEM)return r;
  r=lab_real_snapshot_take(s->core,now,out);if(r!=LAB_AGAIN)return r;
 }
 return LAB_AGAIN;
}
static lab_status health(void *u,uint64_t now,lab_live_health *out){snapshot_io *s=u;if(!s||!out)return LAB_ERR_ARGUMENT;lab_real_snapshot_health(s->core,now,out);
 lab_real_snapshot_stats st;lab_real_snapshot_stats_get(s->core,&st);
 if(s->file_status<0&&!st.failed)snprintf(out->reason,sizeof(out->reason),"SNAPSHOT_FILE_READ_ERROR_%d_NO_FALLBACK",s->file_status);
 if(!s->last_log||now-s->last_log>=1000000){s->last_log=now;
  fprintf(stderr,"SNAPSHOT_MAP file_only=true udp5016=NOT_OPENED exports=%llu poses=%llu scans=%llu missed_exports=%llu queue_drops=%llu file_errors=%llu history_rows=%llu history_overwrites=%llu latest_version_jumps=%llu history_errors=%llu history_invalid_rows=%llu pose_time_us=%llu scan_midpoint_us=%llu pose_error_us=%llu timing=%s reason=%s\n",(unsigned long long)st.exports,(unsigned long long)st.pose_samples,(unsigned long long)st.scans,(unsigned long long)st.missed_exports,(unsigned long long)st.queue_drops,(unsigned long long)s->file_errors,(unsigned long long)s->history.rows,(unsigned long long)s->history.overwritten,(unsigned long long)s->history.latest_version_jumps,(unsigned long long)s->history_errors,(unsigned long long)s->history.rejected_rows,(unsigned long long)st.pose_time_us,(unsigned long long)st.scan_time_us,(unsigned long long)st.pose_error_us,st.pose_error_us==UINT64_MAX?"UNKNOWN":"EXPERIMENTAL_ASSUMED",out->reason);
  fprintf(stderr,"MAP_SOURCE_AGE now_us=%llu raw_rx_us=%llu raw_age_us=%llu pose_us=%llu pose_age_us=%llu scan_us=%llu scan_age_us=%llu raw_limit_us=%llu scan_limit_us=%llu pose_uncertainty_us=%llu delay_min_us=%llu delay_max_us=%llu drift_ppm=%u failed=%u message_zh=年龄与不确定度分别记录并保留真实源时间\n",(unsigned long long)now,(unsigned long long)st.raw_received_us,(unsigned long long)(now>=st.raw_received_us?now-st.raw_received_us:UINT64_MAX),(unsigned long long)st.pose_time_us,(unsigned long long)(now>=st.pose_time_us?now-st.pose_time_us:UINT64_MAX),(unsigned long long)st.scan_time_us,(unsigned long long)(now>=st.scan_time_us?now-st.scan_time_us:UINT64_MAX),(unsigned long long)CONT_ODOM_US,(unsigned long long)CONT_SCAN_US,(unsigned long long)st.pose_error_us,(unsigned long long)s->core_config_delay_min,(unsigned long long)s->core_config_delay_max,s->core_config_drift_ppm,st.failed);
  fprintf(stderr,"SCAN_DESKEW time_basis=SDK_FIRST_PLUS_UNIFORM_INDEX_ESTIMATED hardware_point_times=false applied=%llu skipped=%llu replaced=%llu pending=%u span_us=%u first_us=%llu reference_us=%llu last_us=%llu points=%u invalid_preserved=%u translation_m=%.5f yaw_rad=%.5f max_ray_origin_m=%.5f pose_time_error_us=%llu storage_bytes=%llu reason=%s\n",(unsigned long long)st.deskew_applied,(unsigned long long)st.deskew_skipped,(unsigned long long)st.deskew_replaced,st.deskew_pending,st.deskew_span_us,(unsigned long long)st.deskew_first_us,(unsigned long long)st.deskew_reference_us,(unsigned long long)st.deskew_last_us,st.deskew_points,st.deskew_invalid_points,st.deskew_translation_m,st.deskew_yaw_rad,st.deskew_max_origin_m,(unsigned long long)st.deskew_pose_error_us,(unsigned long long)st.deskew_storage_bytes,st.deskew_reason);
 }return LAB_OK;
}
static lab_status reset(void *u,const lab_real_context *ctx){
 snapshot_io*s=u;if(!s||!lab_real_context_valid(ctx))return LAB_ERR_ARGUMENT;
 lab_real_snapshot*next=NULL;lab_status r=lab_real_snapshot_open(&next,&s->config,ctx,s->parameter_version);if(r!=LAB_OK)return r;
 lab_real_snapshot_close(s->core);s->core=next;memset(&s->history,0,sizeof(s->history));
 s->history.instance=s->last_instance;s->history.sequence=s->last_export?s->last_export-1:0;s->have_polled=0;s->polled=0;s->file_status=LAB_AGAIN;
 /* 缓存LT01分片属于旧运行上下文，切换时丢弃，下一完整真实扫描建立新接收状态。
 */
 if(s->scan_fd>=0){unsigned dropped=0;uint8_t discard[64+SCAN_PACKET_MAX];while(dropped<256){ssize_t n=recv(s->scan_fd,discard,sizeof(discard),MSG_DONTWAIT|MSG_TRUNC);if(n<0){if(errno==EINTR)continue;break;}++dropped;}fprintf(stderr,"SNAPSHOT_RESET_SCAN_QUEUE dropped_packets=%u bound=256\n",dropped);}

 fprintf(stderr,"SNAPSHOT_SOURCE_RESET new_timeline=%u new_epoch=%u action=NEW_CORE_EMPTY_HISTORY_CLOCK_RESEED old_pose_reused=0\n",ctx->timeline_id,ctx->origin_epoch);return LAB_OK;
}
static void close_source(void *u){snapshot_io *s=u;if(s){if(s->scan_fd>=0)close(s->scan_fd);lab_real_snapshot_close(s->core);free(s);}}
static const lab_live_source_ops ops={LAB_REAL_API_VERSION,sizeof(lab_live_source_ops),poll_source,health,reset,close_source};
lab_status lab_real_snapshot_source_create(lab_live_source_binding *out,const char *path,const lab_real_mpu_config *c,const lab_live_source_config *scan,const lab_real_context *ctx){
 if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));
 if(!path||!*path||strlen(path)>=512||!c||!scan||!lab_real_context_valid(ctx)||ctx->source!=LAB_REAL_HARDWARE||ctx->clock!=LAB_REAL_MONOTONIC||scan->read_existing_odometry||scan->ls01_port>65535||scan->ls01_port==5016||scan->parameter_version!=lab_real_snapshot_identity(c))return LAB_ERR_ARGUMENT;
 snapshot_io *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->scan_fd=-1;s->config=*c;s->parameter_version=scan->parameter_version;s->core_config_delay_min=c->delay_min_us;s->core_config_delay_max=c->delay_max_us;s->core_config_drift_ppm=c->drift_ppm;strcpy(s->path,path);const char*slash=strrchr(path,'/');size_t directory=slash?(size_t)(slash-path+1):0;if(directory+strlen("mpu9250.history")>=sizeof(s->history_path)){close_source(s);return LAB_ERR_LIMIT;}memcpy(s->history_path,path,directory);strcpy(s->history_path+directory,"mpu9250.history");
 if(inet_pton(AF_INET,scan->allowed_source_ip,&s->peer)!=1||s->peer.s_addr!=htonl(INADDR_LOOPBACK)){close_source(s);return LAB_ERR_ARGUMENT;}
 lab_status r=lab_real_snapshot_open(&s->core,c,ctx,scan->parameter_version);if(r!=LAB_OK){close_source(s);return r;}
 /* 唯一可选套接字为绑定回环的C1定时扫描，不创建FPGA套接字，不配置网络或启动传感服务及电机发送端。
 */
 if(scan->ls01_port){
  s->scan_fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(s->scan_fd<0){close_source(s);return LAB_ERR_IO;}
  int buffer=65536;struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons((uint16_t)scan->ls01_port),.sin_addr.s_addr=htonl(INADDR_LOOPBACK)};
  if(setsockopt(s->scan_fd,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer))||bind(s->scan_fd,(struct sockaddr*)&addr,sizeof(addr))){fprintf(stderr,"TIMED_SCAN_BIND_FAILED port=%u %s; no takeover\n",scan->ls01_port,strerror(errno));close_source(s);return LAB_ERR_IO;}
 }
 out->user=s;out->ops=&ops;return LAB_OK;
}
#else
lab_status lab_real_snapshot_source_create(lab_live_source_binding *out,const char *p,const lab_real_mpu_config *c,const lab_live_source_config *scan,const lab_real_context *ctx){(void)p;(void)c;(void)scan;(void)ctx;if(out)memset(out,0,sizeof(*out));return LAB_ERR_UNAVAILABLE;}
#endif
