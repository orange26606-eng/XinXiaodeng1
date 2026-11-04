#define _GNU_SOURCE
#include "lab_real_mpu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
typedef struct mpu_io {lab_real_mpu *core;lab_real_context context;lab_real_mpu_config config;int raw_fd,scan_fd;struct in_addr raw_peer,scan_peer;uint64_t packets,rejected,last_log;} mpu_io;
static uint64_t monotonic_us(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return UINT64_MAX;return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;}
static lab_status poll_source(void *u,uint64_t now,lab_live_sample *out){mpu_io *s=u;if(!s||!out)return LAB_ERR_ARGUMENT;
 lab_status ready=lab_real_mpu_take(s->core,now,out);if(ready!=LAB_AGAIN)return ready;
 for(unsigned kind=0;kind<2;++kind){int fd=kind?s->scan_fd:s->raw_fd;if(fd<0)continue;
  for(unsigned i=0;i<(kind?128u:32u);++i){uint8_t packet[LAB_REAL_TIMED_HEADER+SCAN_PACKET_MAX];struct sockaddr_in peer;socklen_t n=sizeof(peer);
   ssize_t bytes=recvfrom(fd,packet,sizeof(packet),MSG_DONTWAIT|MSG_TRUNC,(struct sockaddr*)&peer,&n);
   if(bytes<0){if(errno==EINTR)continue;if(errno==EAGAIN||errno==EWOULDBLOCK)break;return LAB_ERR_IO;}
   if(n!=sizeof(peer)||peer.sin_family!=AF_INET||peer.sin_addr.s_addr!=(kind?s->scan_peer.s_addr:s->raw_peer.s_addr)||(size_t)bytes>sizeof(packet)){++s->rejected;continue;}
   uint64_t rx=monotonic_us();if(rx==UINT64_MAX)return LAB_ERR_TIME;++s->packets;
   lab_status r=kind?lab_real_mpu_scan(s->core,packet,(size_t)bytes,rx):lab_real_mpu_feed(s->core,packet,(size_t)bytes,rx);
   if(r==LAB_ERR_CONTEXT||r==LAB_ERR_NOMEM)return r;if(r<0)++s->rejected;
   ready=lab_real_mpu_take(s->core,now,out);if(ready!=LAB_AGAIN)return ready;
  }
 }
 return LAB_AGAIN;
}
static lab_status health(void *u,uint64_t now,lab_live_health *out){mpu_io *s=u;if(!s||!out)return LAB_ERR_ARGUMENT;lab_real_mpu_stats v;lab_real_mpu_get_stats(s->core,&v);memset(out,0,sizeof(*out));out->context=s->context;out->time_us=now;out->scan_time_us=v.scan_time_us;out->odometry_time_us=out->encoder_time_us=v.pose_time_us;out->imu_time_us=v.imu_time_us;
 out->accepted=v.packets+v.scan_frames;out->rejected=s->rejected;out->gaps=v.gaps+v.queue_drops;
 out->scan_valid=v.scan_valid&&v.scan_time_us<=now&&now-v.scan_time_us<=250000&&v.scan_frames;
 out->odometry_valid=v.pose_valid&&v.bias_ready&&v.counts_mask==15&&v.pose_time_us<=now&&now-v.pose_time_us<=250000;out->encoder_valid_mask=out->odometry_valid?v.counts_mask:0;
 out->imu_valid_flags=v.bias_ready&&v.imu_time_us&&v.imu_time_us<=now&&now-v.imu_time_us<=150000?LAB_IMU_YAW_VALID|LAB_IMU_RATE_VALID:0;
 const char *reason=v.reason;if(!v.packets)reason="MPU9250_WAIT_UDP5016";else if(v.mpu_id!=0x71)reason="MPU9250_ID_NOT_0X71";else if(!(v.imu_flags&4))reason="MPU9250_DEVICE_NOT_READY";else if(!v.bias_ready)reason="MPU9250_KEEP_STILL_FOR_BIAS";else if(!out->odometry_valid)reason="MPU9250_ODOMETRY_STALE_OR_INVALID";else if(!v.scan_frames)reason="WAIT_LT01_TIMESTAMPED_USB_SCAN";
 snprintf(out->reason,sizeof(out->reason),"%s",reason);
 if(!s->last_log||now-s->last_log>=1000000){s->last_log=now;fprintf(stderr,"MPU_READONLY bias=%s id=%02x flags=%02x i2c_error=%u mask=%x counts=%d,%d,%d,%d rpm=%.2f,%.2f,%.2f,%.2f rpy_deg=%.2f,%.2f,%.2f xy_m=%.4f,%.4f distance_m=%.4f clock_error_us=%llu errors=%llu profile=%s\n",v.bias_ready?"READY":"KEEP_STILL",v.mpu_id,v.imu_flags,v.i2c_error,v.counts_mask,v.counts[0],v.counts[1],v.counts[2],v.counts[3],v.rpm[0],v.rpm[1],v.rpm[2],v.rpm[3],v.roll_rad*57.29577951308232,v.pitch_rad*57.29577951308232,v.yaw_rad*57.29577951308232,v.x_m,v.y_m,v.distance_m,(unsigned long long)v.uncertainty_us,(unsigned long long)(v.crc_errors+v.timing_rejected),v.have_profile?"EXPERIMENTAL_UNVERIFIED":"UNKNOWN");}
 return LAB_OK;
}
static lab_status reset(void *u,const lab_real_context *ctx){(void)u;(void)ctx;return LAB_ERR_UNAVAILABLE;/* 显式重建，使融合零偏与数据源代次一起重新开始。 */}
static void close_source(void *u){mpu_io *s=u;if(!s)return;if(s->raw_fd>=0)close(s->raw_fd);if(s->scan_fd>=0)close(s->scan_fd);lab_real_mpu_core_close(s->core);free(s);}
static int socket_bound(const char *ip,unsigned port){int fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(fd<0)return -1;int buffer=65536;struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons((uint16_t)port)};
 if(inet_pton(AF_INET,ip,&address.sin_addr)!=1||setsockopt(fd,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer))||bind(fd,(struct sockaddr*)&address,sizeof(address))){fprintf(stderr,"READONLY_BIND_FAILED %s:%u: %s; no port reuse or service takeover\n",ip,port,strerror(errno));close(fd);return -1;}return fd;
}
static const lab_live_source_ops operations={LAB_REAL_API_VERSION,sizeof(lab_live_source_ops),poll_source,health,reset,close_source};
lab_status lab_real_mpu_source_create(lab_live_source_binding *out,const lab_real_mpu_config *c,const lab_live_source_config *scan,const lab_real_context *ctx){
 if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));if(!c||!scan||!lab_real_context_valid(ctx)||ctx->source!=LAB_REAL_HARDWARE||ctx->clock!=LAB_REAL_MONOTONIC||c->port!=5016||scan->ls01_port>65535||scan->ls01_port==5016||scan->read_existing_odometry)return LAB_ERR_ARGUMENT;
 if(lab_real_mpu_identity(c)!=scan->parameter_version)return LAB_ERR_VERSION;
 mpu_io *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->raw_fd=s->scan_fd=-1;s->context=*ctx;s->config=*c;
 if(inet_pton(AF_INET,c->peer_ip,&s->raw_peer)!=1||inet_pton(AF_INET,scan->allowed_source_ip,&s->scan_peer)!=1||s->scan_peer.s_addr!=htonl(INADDR_LOOPBACK)){close_source(s);return LAB_ERR_ARGUMENT;}
 lab_status r=lab_real_mpu_core_open(&s->core,c,ctx,scan->parameter_version);if(r!=LAB_OK){close_source(s);return r;}
 s->raw_fd=socket_bound(c->bind_ip,c->port);if(s->raw_fd<0){close_source(s);return LAB_ERR_IO;}
 if(scan->ls01_port){s->scan_fd=socket_bound("127.0.0.1",scan->ls01_port);if(s->scan_fd<0){close_source(s);return LAB_ERR_IO;}}
 out->user=s;out->ops=&operations;return LAB_OK;
}
#else
lab_status lab_real_mpu_source_create(lab_live_source_binding *out,const lab_real_mpu_config *c,const lab_live_source_config *scan,const lab_real_context *ctx){(void)c;(void)scan;(void)ctx;if(out)memset(out,0,sizeof(*out));return LAB_ERR_UNAVAILABLE;}
#endif
