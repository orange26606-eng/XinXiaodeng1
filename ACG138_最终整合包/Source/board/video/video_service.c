#define _GNU_SOURCE
#include "video_service.h"
#include "h264_encoder.h"
#include <errno.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
enum { RING=4, SOURCE_BYTES=1920*1080*3/2 };
struct video_packet {unsigned char *data;size_t bytes;uint64_t serial,capture_ms,encoded_ms;unsigned flags;};
static struct {
 pthread_mutex_t mutex;pthread_cond_t changed;pthread_t worker;
 _Atomic int stop,client,force_idr;int started,stream_fd;
 struct h264_encoder *encoder;video_source_reader reader;
 struct video_packet packets[RING];uint64_t serial;struct video_stats stats;
} service={.mutex=PTHREAD_MUTEX_INITIALIZER,.changed=PTHREAD_COND_INITIALIZER,.stream_fd=-1};
static uint64_t mono_us(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;}
static void pause_ms(unsigned ms){struct timespec p={ms/1000,(long)(ms%1000)*1000000};nanosleep(&p,NULL);}
static void put16(unsigned char *p,unsigned v){p[0]=(unsigned char)(v>>8);p[1]=(unsigned char)v;}
static void put32(unsigned char *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(unsigned char)(v>>(24-i*8));}
static void put64(unsigned char *p,uint64_t v){for(unsigned i=0;i<8;i++)p[i]=(unsigned char)(v>>(56-i*8));}
static int send_deadline(int fd,const void *data,size_t bytes,uint64_t end){
 const unsigned char *p=data;
 while(bytes&&!atomic_load(&service.stop)){
  if(mono_us()>=end)return -1;
  ssize_t n=send(fd,p,bytes,MSG_NOSIGNAL|MSG_DONTWAIT);
  if(n>0){p+=n;bytes-=(size_t)n;continue;}
  if(n<0&&errno==EINTR)continue;
  if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
   struct pollfd poller={.fd=fd,.events=POLLOUT};if(poll(&poller,1,5)<0&&errno!=EINTR)return -1;continue;
  }
  return -1;
 }
 return bytes?-1:0;
}
static void *encoder_worker(void *unused){
 (void)unused;unsigned char *raw=malloc(SOURCE_BYTES);uint32_t previous=0;
 uint64_t window=mono_us(),count=0,next_encode=0;
 if(!raw)goto fail;
 while(!atomic_load(&service.stop)){
  if(!atomic_load(&service.client)){pause_ms(10);window=mono_us();count=0;next_encode=0;continue;}
  if(mono_us()<next_encode){pause_ms(1);continue;}
  struct video_source frame={0};int rc=service.reader(raw,SOURCE_BYTES,previous,&frame);
  if(rc<0)goto fail;if(!rc){pause_ms(2);continue;}
  previous=frame.sequence;uint64_t now=mono_us()/1000;
  if(now<frame.capture_ms||now-frame.capture_ms>200){pthread_mutex_lock(&service.mutex);service.stats.dropped++;pthread_mutex_unlock(&service.mutex);atomic_store(&service.force_idr,1);continue;}
  unsigned char *pixels=h264_encoder_pixels(service.encoder),*encoded=NULL;size_t size=0;unsigned flags=0;
  if(video_scale_nv12_720(pixels,raw,SOURCE_BYTES,frame.width,frame.height,frame.stride))goto fail;
  uint64_t start=mono_us();
  /* 使用绝对时间节拍，避免缩放与轮询开销逐帧累积。 */
  if(!next_encode||start-next_encode>33333)next_encode=start;
  next_encode+=33333;int force=atomic_exchange(&service.force_idr,0);
  if(h264_encoder_encode(service.encoder,frame.capture_ms,force,&encoded,&size,&flags))goto fail;
  uint64_t end=mono_us();if((flags&2)&&!(flags&1)){fputs("H264 IDR missing SPS/PPS\n",stderr);goto fail;}
  if(force&&!(flags&2)){fputs("MPP did not honor forced IDR\n",stderr);goto fail;}
  pthread_mutex_lock(&service.mutex);
  uint64_t serial=++service.serial;struct video_packet *packet=&service.packets[serial%RING];
  memcpy(packet->data,encoded,size);packet->bytes=size;packet->flags=flags;packet->serial=serial;packet->capture_ms=frame.capture_ms;packet->encoded_ms=end/1000;
  service.stats.encoded++;service.stats.last_capture_ms=frame.capture_ms;service.stats.encode_us=(unsigned)(end-start);count++;
  if(end-window>=2000000){
   service.stats.fps_milli=(unsigned)(count*1000000000ULL/(end-window));
   fprintf(stderr,"PHONE_H264 fps=%.2f encoded=%llu encoder_ms=%.2f source_age_ms=%llu resyncs=%llu\n",service.stats.fps_milli/1000.0,(unsigned long long)service.stats.encoded,service.stats.encode_us/1000.0,(unsigned long long)(end/1000-frame.capture_ms),(unsigned long long)service.stats.resyncs);
   window=end;count=0;
  }
  pthread_cond_broadcast(&service.changed);pthread_mutex_unlock(&service.mutex);
 }
 free(raw);return NULL;
fail:
 free(raw);pthread_mutex_lock(&service.mutex);service.stats.failed=1;pthread_cond_broadcast(&service.changed);pthread_mutex_unlock(&service.mutex);
 fputs("PHONE_H264 failed; control watchdog and DSI remain independent\n",stderr);return NULL;
}
int video_service_start(const char *library,video_source_reader reader){
 if(service.started||!reader)return -1;service.reader=reader;
 if(h264_encoder_open(&service.encoder,library))return -1;
 /* 私有灰度图仅用于编码器自检。 */
 unsigned char *pixels=h264_encoder_pixels(service.encoder),*data;size_t n;unsigned flags;
 memset(pixels,126,1280*720);memset(pixels+1280*720,128,1280*720/2);
 if(h264_encoder_encode(service.encoder,0,1,&data,&n,&flags)||flags!=3){fputs("H264 hardware probe failed (SPS/PPS/IDR required)\n",stderr);goto fail;}
 for(unsigned i=0;i<RING;i++){service.packets[i].data=malloc(VIDEO_MAX_PACKET);if(!service.packets[i].data)goto fail;}
 service.stats.ready=1;atomic_store(&service.stop,0);atomic_store(&service.client,0);atomic_store(&service.force_idr,1);
 if(pthread_create(&service.worker,NULL,encoder_worker,NULL))goto fail;
 service.started=1;fputs("PHONE_H264_READY 1280x720 target=30 hardware=MPP profile=baseline b_frames=0 ring=4 probe_not_published=1\n",stderr);return 0;
fail:
 for(unsigned i=0;i<RING;i++){free(service.packets[i].data);service.packets[i].data=NULL;}
 h264_encoder_close(service.encoder);service.encoder=NULL;return -1;
}
void video_service_stats(struct video_stats *stats){pthread_mutex_lock(&service.mutex);*stats=service.stats;stats->connected=atomic_load(&service.client);pthread_mutex_unlock(&service.mutex);}
void video_service_stop(void){
 if(!service.started)return;
 atomic_store(&service.stop,1);pthread_mutex_lock(&service.mutex);if(service.stream_fd>=0)shutdown(service.stream_fd,SHUT_RDWR);pthread_cond_broadcast(&service.changed);pthread_mutex_unlock(&service.mutex);
 pthread_join(service.worker,NULL);
 /* 分离的HTTP处理线程仍持有套接字，等待视频流释放后再退出。 */
 for(unsigned i=0;i<100&&atomic_load(&service.client);i++)pause_ms(5);
 if(atomic_load(&service.client)){fputs("Video handler still exiting; retaining its buffers until process exit\n",stderr);return;}
 for(unsigned i=0;i<RING;i++){free(service.packets[i].data);service.packets[i].data=NULL;}
 h264_encoder_close(service.encoder);service.encoder=NULL;service.started=0;
}
void video_service_stream(int fd,const char *session,video_session_check valid){
 int expected=0;
 if(!service.started||!atomic_compare_exchange_strong(&service.client,&expected,1)){
  const char response[]="HTTP/1.1 409 Conflict\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
  (void)send_deadline(fd,response,sizeof(response)-1,mono_us()+100000);return;
 }
 unsigned char *copy=malloc(VIDEO_MAX_PACKET);if(!copy)goto done;
 int one=1,buffer=32768;setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));setsockopt(fd,SOL_SOCKET,SO_SNDBUF,&buffer,sizeof(buffer));
 const char response[]="HTTP/1.1 200 OK\r\nContent-Type: application/x-acg138-h264\r\nCache-Control: no-store\r\nConnection: close\r\nX-Video-Width: 1280\r\nX-Video-Height: 720\r\nX-Video-FPS: 30\r\n\r\n";
 if(send_deadline(fd,response,sizeof(response)-1,mono_us()+100000))goto done;
 pthread_mutex_lock(&service.mutex);service.stream_fd=fd;uint64_t next=service.serial+1;pthread_mutex_unlock(&service.mutex);
 int need_key=1;atomic_store(&service.force_idr,1);uint64_t last_frame=mono_us();
 while(!atomic_load(&service.stop)&&valid(session)){
  if(mono_us()-last_frame>2000000)break;
  pthread_mutex_lock(&service.mutex);
  if(service.stats.failed){pthread_mutex_unlock(&service.mutex);break;}
  if(service.serial<next){
   struct timespec deadline;clock_gettime(CLOCK_REALTIME,&deadline);deadline.tv_nsec+=20000000;if(deadline.tv_nsec>=1000000000){deadline.tv_sec++;deadline.tv_nsec-=1000000000;}
   pthread_cond_timedwait(&service.changed,&service.mutex,&deadline);pthread_mutex_unlock(&service.mutex);continue;
  }
  if(service.serial-next>=RING){next=service.serial+1;need_key=1;service.stats.resyncs++;pthread_mutex_unlock(&service.mutex);atomic_store(&service.force_idr,1);continue;}
  struct video_packet packet=service.packets[next%RING];next++;
  if(packet.serial!=next-1){pthread_mutex_unlock(&service.mutex);break;}
  if(mono_us()/1000<packet.capture_ms||mono_us()/1000-packet.capture_ms>200){need_key=1;service.stats.dropped++;pthread_mutex_unlock(&service.mutex);atomic_store(&service.force_idr,1);continue;}
  if(need_key&&!(packet.flags&2)){pthread_mutex_unlock(&service.mutex);continue;}
  need_key=0;memcpy(copy,packet.data,packet.bytes);pthread_mutex_unlock(&service.mutex);
  unsigned char header[40]={0};memcpy(header,"ACV1",4);put16(header+4,40);put16(header+6,packet.flags);
  put32(header+8,(uint32_t)packet.bytes);put32(header+12,(uint32_t)packet.serial);put64(header+16,packet.capture_ms);put64(header+24,packet.encoded_ms);
  put16(header+32,1280);put16(header+34,720);put32(header+36,30000);
  uint64_t end=mono_us()+100000;
  if(send_deadline(fd,header,sizeof(header),end)||send_deadline(fd,copy,packet.bytes,end))break;
  last_frame=mono_us();
 }
done:
 free(copy);pthread_mutex_lock(&service.mutex);service.stream_fd=-1;pthread_mutex_unlock(&service.mutex);atomic_store(&service.client,0);
}
