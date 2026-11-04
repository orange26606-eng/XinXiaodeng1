#ifndef ACG_CAMERA_FAST_RX_H
#define ACG_CAMERA_FAST_RX_H
/* 独立接收线程保留最新完整帧，缓存容量固定。 */
#include "raw_rows.h"
#include <arpa/inet.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <sys/socket.h>
#include <time.h>
#define RX_W 1920u
#define RX_H 1080u
#define RX_BYTES (RX_W*RX_H)
#ifndef CAMERA_BIND_IP
#define CAMERA_BIND_IP 0xc0a80003u
#endif
#ifndef CAMERA_PEER_IP
#define CAMERA_PEER_IP 0xc0a80002u
#endif
static struct {
    pthread_t thread;pthread_mutex_t mutex;pthread_cond_t ready;
    _Atomic int stop;
    int fd,pending,started,swap;
    unsigned char *latest;
    uint64_t complete,malformed,incomplete,superseded,ignored,delivered;
} camera_rx;
static void *isp_rx_thread(void *unused) {
    (void)unused;
    unsigned char *working=malloc(RX_BYTES);
    if(!working){atomic_store(&camera_rx.stop,1);pthread_cond_broadcast(&camera_rx.ready);return NULL;}
    struct raw_rows rows={.swap_pairs=camera_rx.swap};
    unsigned char packets[32][RX_W+2];
    struct mmsghdr messages[32];struct iovec vectors[32];struct sockaddr_in peers[32];
    while(!atomic_load(&camera_rx.stop)) {
        struct pollfd p={.fd=camera_rx.fd,.events=POLLIN};
        if(poll(&p,1,100)<=0)continue;
        memset(messages,0,sizeof(messages));
        for(unsigned i=0;i<32;i++){
            vectors[i]=(struct iovec){.iov_base=packets[i],.iov_len=RX_W+2};
            messages[i].msg_hdr.msg_iov=&vectors[i];messages[i].msg_hdr.msg_iovlen=1;
            messages[i].msg_hdr.msg_name=&peers[i];messages[i].msg_hdr.msg_namelen=sizeof(peers[i]);
        }
        int count=recvmmsg(camera_rx.fd,messages,32,MSG_DONTWAIT|MSG_TRUNC,NULL);
        if(count<0){if(errno==EAGAIN||errno==EINTR)continue;atomic_store(&camera_rx.stop,1);break;}
        for(int i=0;i<count;i++) {
            pthread_mutex_lock(&camera_rx.mutex);
            if(peers[i].sin_addr.s_addr!=htonl(CAMERA_PEER_IP)){camera_rx.ignored++;pthread_mutex_unlock(&camera_rx.mutex);continue;}
            size_t n=messages[i].msg_len;
            if(n!=RX_W+2 || (messages[i].msg_hdr.msg_flags&MSG_TRUNC)){
                camera_rx.malformed++;if(rows.collecting)camera_rx.incomplete++;rows.collecting=0;
                pthread_mutex_unlock(&camera_rx.mutex);continue;
            }
            unsigned row=packets[i][0]|((unsigned)packets[i][1]<<8);
            if(rows.collecting && (row==0 || row!=rows.next || row>=RX_H))camera_rx.incomplete++;
            if(row>=RX_H)camera_rx.malformed++;
            if(raw_rows_push(&rows,working,RX_W,RX_W,RX_H,packets[i],n)){
                if(camera_rx.pending)camera_rx.superseded++;
                memcpy(camera_rx.latest,working,RX_BYTES);camera_rx.pending=1;camera_rx.complete++;
                pthread_cond_signal(&camera_rx.ready);
            }
            pthread_mutex_unlock(&camera_rx.mutex);
        }
    }
    free(working);pthread_cond_broadcast(&camera_rx.ready);return NULL;
}
static int isp_udp_open(int swap_pairs) {
    memset(&camera_rx,0,sizeof(camera_rx));camera_rx.fd=-1;camera_rx.swap=swap_pairs;
    int fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);if(fd<0)return -1;
    int bytes=8*1024*1024;
    if(setsockopt(fd,SOL_SOCKET,SO_RCVBUFFORCE,&bytes,sizeof(bytes)))setsockopt(fd,SOL_SOCKET,SO_RCVBUF,&bytes,sizeof(bytes));
    struct sockaddr_in a={.sin_family=AF_INET,.sin_port=htons(6102),.sin_addr={.s_addr=htonl(CAMERA_BIND_IP)}};
    if(bind(fd,(struct sockaddr*)&a,sizeof(a))<0){perror("RAW UDP bind");close(fd);return -1;}
    camera_rx.latest=malloc(RX_BYTES);if(!camera_rx.latest){close(fd);return -1;}
    pthread_mutex_init(&camera_rx.mutex,NULL);pthread_cond_init(&camera_rx.ready,NULL);
    camera_rx.fd=fd;
    if(pthread_create(&camera_rx.thread,NULL,isp_rx_thread,NULL)){
        free(camera_rx.latest);close(fd);pthread_mutex_destroy(&camera_rx.mutex);pthread_cond_destroy(&camera_rx.ready);return -1;
    }
    camera_rx.started=1;return fd;
}
static int isp_udp_frame(int fd,unsigned char *dst,unsigned stride,unsigned width,unsigned height,int swap_pairs) {
    if(fd!=camera_rx.fd||width!=RX_W||height!=RX_H||stride<width||swap_pairs!=camera_rx.swap)return -1;
    struct timespec end;clock_gettime(CLOCK_REALTIME,&end);end.tv_sec+=4;
    pthread_mutex_lock(&camera_rx.mutex);
    while(!camera_rx.pending&&!atomic_load(&camera_rx.stop)){
        if(pthread_cond_timedwait(&camera_rx.ready,&camera_rx.mutex,&end)==ETIMEDOUT)break;
    }
    int ok=camera_rx.pending&&!atomic_load(&camera_rx.stop);
    if(ok){for(unsigned y=0;y<height;y++)memcpy(dst+(size_t)y*stride,camera_rx.latest+(size_t)y*width,width);camera_rx.pending=0;camera_rx.delivered++;}
    pthread_mutex_unlock(&camera_rx.mutex);
    if(!ok)fputs("RAW UDP complete-frame timeout\n",stderr);
    return ok?0:-1;
}
static void isp_udp_report(unsigned output_frames) {
    static double last;static uint64_t prev_rx,prev_output;
    struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);double now=t.tv_sec+t.tv_nsec/1e9;
    if(!last){pthread_mutex_lock(&camera_rx.mutex);prev_rx=camera_rx.complete;prev_output=output_frames;pthread_mutex_unlock(&camera_rx.mutex);last=now;return;}
    if(now-last<2)return;
    pthread_mutex_lock(&camera_rx.mutex);
    printf("FASTPATH rx_fps=%.2f output_fps=%.2f complete=%llu incomplete=%llu malformed=%llu rx_superseded=%llu ignored_peer=%llu\n",
        (camera_rx.complete-prev_rx)/(now-last),(output_frames-prev_output)/(now-last),
        (unsigned long long)camera_rx.complete,(unsigned long long)camera_rx.incomplete,
        (unsigned long long)camera_rx.malformed,(unsigned long long)camera_rx.superseded,(unsigned long long)camera_rx.ignored);
    prev_rx=camera_rx.complete;prev_output=output_frames;last=now;
    pthread_mutex_unlock(&camera_rx.mutex);fflush(stdout);
}
static void isp_udp_close(int fd) {
    if(camera_rx.started){atomic_store(&camera_rx.stop,1);pthread_join(camera_rx.thread,NULL);camera_rx.started=0;free(camera_rx.latest);pthread_mutex_destroy(&camera_rx.mutex);pthread_cond_destroy(&camera_rx.ready);}
    if(fd>=0)close(fd);
}
#endif
