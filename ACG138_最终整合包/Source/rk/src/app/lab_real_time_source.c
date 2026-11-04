#define _GNU_SOURCE
#include "lab_real_time_source.h"
#include "lab_real_time_io.h"
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
typedef struct time_source {
 lab_real_timed_source_config config;lab_real_context context;lab_live_source_binding scans;
 lab_real_timed_odom_state odom;lab_live_sample *pending,*held,*scratch;unsigned head,count;
 lab_real_scan_time metadata[8];unsigned metadata_at;
 int fd,started,have_cache,have_held,have_now,failed;pthread_t thread;pthread_mutex_t mutex;atomic_int stop;
 lab_real_odom_time cache;lab_status file_status;uint64_t read_us,now_us,minimum_us,held_since;
 lab_live_health health;
} time_source;
static uint64_t now_us(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return UINT64_MAX;return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;}
static void *reader(void *u){time_source *s=u;while(!atomic_load_explicit(&s->stop,memory_order_relaxed)){lab_real_odom_time c;lab_status r=lab_real_odom_cache_read(s->config.timed_odometry_path,&c);uint64_t t=now_us();pthread_mutex_lock(&s->mutex);s->cache=c;s->file_status=r;s->read_us=t;pthread_mutex_unlock(&s->mutex);struct timespec sleep={0,10000000};nanosleep(&sleep,NULL);}return NULL;}
static lab_status metadata(time_source *s){if(s->fd<0)return LAB_OK;uint8_t bytes[LAB_SCAN_TIME_BYTES];for(unsigned i=0;i<64;++i){struct sockaddr_in from;socklen_t n=sizeof(from);ssize_t len=recvfrom(s->fd,bytes,sizeof(bytes),MSG_DONTWAIT|MSG_TRUNC,(struct sockaddr*)&from,&n);if(len<0){if(errno==EINTR)continue;if(errno==EAGAIN||errno==EWOULDBLOCK)break;return LAB_ERR_IO;}
 struct in_addr allowed;inet_pton(AF_INET,s->config.legacy.allowed_source_ip[0]?s->config.legacy.allowed_source_ip:"127.0.0.1",&allowed);
 if(n!=sizeof(from)||from.sin_family!=AF_INET||from.sin_addr.s_addr!=allowed.s_addr){++s->health.rejected;continue;}uint64_t actual_rx=now_us();if(actual_rx==UINT64_MAX)return LAB_ERR_TIME;lab_real_scan_time m;lab_status r=lab_real_scan_time_decode(&m,bytes,(size_t)len);if(r!=LAB_OK||m.fetched_us<s->minimum_us||m.fetched_us>actual_rx){++s->health.rejected;continue;}s->metadata[s->metadata_at++%8]=m;}
 return LAB_OK;
}
static lab_status cache(time_source *s,uint64_t now){if(!s->started)return LAB_OK;if(pthread_mutex_trylock(&s->mutex))return LAB_OK;lab_real_odom_time c=s->cache;lab_status r=s->file_status;uint64_t read=s->read_us;pthread_mutex_unlock(&s->mutex);
 if(r!=LAB_OK||read>now||c.received_us<s->minimum_us){if(r<0)snprintf(s->health.reason,sizeof(s->health.reason),"TIMED_ODOMETRY_CACHE_UNAVAILABLE");return LAB_OK;}
 uint32_t n=0;r=lab_real_timed_odom_convert(&s->odom,&c,&s->context,now,s->pending,&n);
 snprintf(s->health.reason,sizeof(s->health.reason),"%s",s->odom.clock.reason);
 if(r==LAB_AGAIN)return LAB_OK;if(r!=LAB_OK)return r;s->count=n;s->have_cache=1;
 if(n){s->health.odometry_time_us=s->health.encoder_time_us=s->pending[0].meta.time_us;s->health.odometry_valid=!!(s->pending[0].meta.flags&LAB_REAL_DATA_VALID);s->health.encoder_valid_mask=s->pending[1].data.encoder.valid_mask;}
 if(n==3){s->health.imu_time_us=s->pending[2].meta.time_us;s->health.imu_valid_flags=s->pending[2].data.imu.valid_flags;}return LAB_OK;
}
static lab_status scan(time_source *s,uint64_t now){
 if(!s->scans.ops)return LAB_OK;
 /* 复用 LS01 接收器返回的完整扫描副本，不重复绑定 UDP 端口。 */
 uint64_t actual=now_us();if(actual==UINT64_MAX)return LAB_ERR_TIME;lab_status r=s->scans.ops->poll(s->scans.user,actual,s->scratch);
 if(r<0)return r;if(r==LAB_OK){if(s->scratch->type!=LAB_EVENT_SCAN)return LAB_ERR_FORMAT;if(s->have_held)++s->health.gaps;*s->held=*s->scratch;s->held_since=now;s->have_held=1;}
 if(!s->have_held)return LAB_OK;
 /* 报文可能晚于调用方采样tick到达，保留至下一tick，不作为未来测量拒绝。
 */
 if(s->held->data.scan.timestamp_us>now)return LAB_OK;
 const lab_real_scan_time *m=NULL;for(unsigned i=0;i<8;++i)if(s->metadata[i].session==s->held->meta.stream_session&&s->metadata[i].sequence==s->held->meta.sequence){m=&s->metadata[i];break;}
 if(!m&&now-s->held_since<50000)return LAB_OK;
 s->held->meta.received_us=now;
 if(m){r=lab_real_scan_time_apply(m,&s->config.profile,s->held);if(r!=LAB_OK){++s->health.rejected;s->have_held=0;return LAB_OK;}}
 else if(s->config.profile.sdk_time_declared){++s->health.gaps;s->have_held=0;snprintf(s->health.reason,sizeof(s->health.reason),"SDK_TIME_COMPANION_MISSING");return LAB_OK;}
 else snprintf(s->health.reason,sizeof(s->health.reason),"SDK_TIME_COMPANION_MISSING_UNBOUNDED");
 if(s->count>=4)return LAB_ERR_LIMIT;s->pending[s->count++]=*s->held;s->have_held=0;s->health.scan_time_us=s->pending[s->count-1].meta.time_us;s->health.scan_valid=!!(s->pending[s->count-1].meta.flags&LAB_REAL_DATA_VALID);return LAB_OK;
}
static lab_status poll_source(void *u,uint64_t now,lab_live_sample *out){time_source *s=u;if(!s||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));if(s->failed)return LAB_ERR_CONTEXT;if((s->have_now&&now<s->now_us)||now==UINT64_MAX){s->failed=1;return LAB_ERR_TIME;}s->have_now=1;s->now_us=now;
 if(s->head==s->count){s->head=s->count=0;lab_status r=metadata(s);if(r==LAB_OK)r=cache(s,now);if(r==LAB_OK)r=scan(s,now);if(r<0){s->failed=1;++s->health.rejected;snprintf(s->health.reason,sizeof(s->health.reason),"%s",s->odom.clock.failed?s->odom.clock.reason:"TIMED_SOURCE_CONTEXT_OR_CONFIG_ERROR");return r;}}
 if(s->head==s->count)return LAB_AGAIN;*out=s->pending[s->head++];if(!lab_live_sample_valid(out)){s->failed=1;return LAB_ERR_FORMAT;}++s->health.accepted;return LAB_OK;
}
static lab_status health_source(void *u,uint64_t now,lab_live_health *out){time_source *s=u;if(!s||!out)return LAB_ERR_ARGUMENT;*out=s->health;out->context=s->context;out->time_us=now;if(now<s->now_us)return LAB_ERR_TIME;
 if(!s->have_cache&&s->started&&!out->reason[0])snprintf(out->reason,sizeof(out->reason),"WAIT_TIMED_ODOMETRY_CACHE");
 if(!s->started&&!out->reason[0])snprintf(out->reason,sizeof(out->reason),"SDK_SCAN_TIME_ONLY");
 if(!s->config.profile.sdk_time_declared&&s->config.legacy.ls01_port&&s->have_cache&&s->odom.clock.profile.fpga_delay_declared&&s->odom.clock.profile.fpga_stamp_declared)snprintf(out->reason,sizeof(out->reason),"SDK_TIME_BOUND_UNCONFIRMED");
 uint64_t a=s->config.legacy.max_scan_age_us,b=s->config.legacy.max_odometry_age_us;
 if(out->scan_time_us>now||now-out->scan_time_us>a)out->scan_valid=0;if(out->odometry_time_us>now||now-out->odometry_time_us>b){out->odometry_valid=0;out->encoder_valid_mask=0;}if(out->imu_time_us>now||now-out->imu_time_us>b)out->imu_valid_flags=0;
 if(s->failed&&!out->reason[0])snprintf(out->reason,sizeof(out->reason),"TIMING_CONTEXT_RESET_REQUIRED");return LAB_OK;
}
static lab_status reset_source(void *u,const lab_real_context *c){time_source *s=u;if(!s||!lab_real_context_valid(c)||c->source!=s->context.source||c->clock!=s->context.clock||lab_real_context_equal(c,&s->context))return LAB_ERR_CONTEXT;if(s->scans.ops){lab_status r=s->scans.ops->reset(s->scans.user,c);if(r!=LAB_OK)return r;}
 s->context=*c;s->minimum_us=s->now_us;s->head=s->count=s->have_cache=s->have_held=s->failed=0;memset(s->metadata,0,sizeof(s->metadata));memset(&s->health,0,sizeof(s->health));s->health.context=*c;lab_real_timed_odom_init(&s->odom,&s->config.profile);return LAB_OK;
}
static void close_source(void *u){time_source *s=u;if(!s)return;if(s->started){atomic_store(&s->stop,1);pthread_join(s->thread,NULL);}if(s->scans.ops)s->scans.ops->close(s->scans.user);if(s->fd>=0)close(s->fd);pthread_mutex_destroy(&s->mutex);free(s->pending);free(s->held);free(s->scratch);free(s);}
static const lab_live_source_ops operations={LAB_REAL_API_VERSION,sizeof(lab_live_source_ops),poll_source,health_source,reset_source,close_source};
lab_status lab_real_timed_source_create(lab_live_source_binding *out,const lab_real_timed_source_config *c,const lab_real_context *ctx){
 if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));if(!c||!lab_real_context_valid(ctx)||ctx->source!=LAB_REAL_HARDWARE||ctx->clock!=LAB_REAL_MONOTONIC||!lab_real_time_profile_valid(&c->profile)||c->profile.parameter_version!=c->legacy.parameter_version||c->legacy.read_existing_odometry||c->legacy.odometry_path[0]||c->scan_time_port>65535||(c->legacy.ls01_port&&c->scan_time_port==c->legacy.ls01_port)||!memchr(c->timed_odometry_path,0,256)||(!c->timed_odometry_path[0]&&!c->legacy.ls01_port)||(c->legacy.ls01_port&&!c->scan_time_port))return LAB_ERR_ARGUMENT;
 time_source *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->config=*c;s->context=*ctx;s->fd=-1;atomic_init(&s->stop,0);if(pthread_mutex_init(&s->mutex,NULL)){free(s);return LAB_ERR_IO;}s->pending=calloc(4,sizeof(*s->pending));s->held=calloc(1,sizeof(*s->held));s->scratch=calloc(1,sizeof(*s->scratch));if(!s->pending||!s->held||!s->scratch){close_source(s);return LAB_ERR_NOMEM;}lab_real_timed_odom_init(&s->odom,&c->profile);lab_status r;
 if(c->legacy.ls01_port){r=lab_live_source_create(&s->scans,&c->legacy,ctx);if(r!=LAB_OK){close_source(s);return r;}struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons((uint16_t)c->scan_time_port)};struct in_addr peer;const char *ip=c->legacy.allowed_source_ip[0]?c->legacy.allowed_source_ip:"127.0.0.1";if(inet_pton(AF_INET,ip,&peer)!=1){close_source(s);return LAB_ERR_ARGUMENT;}address.sin_addr.s_addr=(ntohl(peer.s_addr)>>24)==127?htonl(INADDR_LOOPBACK):htonl(INADDR_ANY);s->fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);int size=32768;if(s->fd<0||setsockopt(s->fd,SOL_SOCKET,SO_RCVBUF,&size,sizeof(size))||bind(s->fd,(struct sockaddr*)&address,sizeof(address))){close_source(s);return LAB_ERR_IO;}}
 if(c->timed_odometry_path[0]){pthread_attr_t a;if(pthread_attr_init(&a)){close_source(s);return LAB_ERR_IO;}int e=pthread_attr_setstacksize(&a,128u*1024u);if(!e)e=pthread_create(&s->thread,&a,reader,s);pthread_attr_destroy(&a);if(e){close_source(s);return LAB_ERR_IO;}s->started=1;}
 out->user=s;out->ops=&operations;return LAB_OK;
}
