#define _GNU_SOURCE
/* 独占 UDP5010 的只读 OD01 接收程序，不能与其他接收器同时运行。不包含电机发送、控制权申请或使能功能。 */
#include "lab_real_time.h"
#include "lab_real_time_io.h"
#include "lab_codec.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
static volatile sig_atomic_t running=1;
static void stop(int s){(void)s;running=0;}
static uint64_t now_us(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;}
static int num(const char *s,uint64_t *v){if(!s||*s=='-'||*s=='+')return 0;char *end;errno=0;unsigned long long n=strtoull(s,&end,10);if(errno||end==s||*end||!n)return 0;*v=n;return 1;}
static uint32_t config_crc(const struct odom_config *c,uint32_t cpr){
 /* 规范文字参数指纹，不对结构填充或本机布局做哈希。 */
 char b[1024];int n=snprintf(b,sizeof(b),"%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %d %d %d %d %d %d %u",c->mm_per_count[0],c->mm_per_count[1],c->mm_per_count[2],c->mm_per_count[3],c->track_mm,c->yaw_sign,c->yaw_offset,c->correction_tau,c->signs[0],c->signs[1],c->signs[2],c->signs[3],c->left_mask,c->calibrated,cpr);return n>0&&(size_t)n<sizeof(b)?lab_crc32(b,(size_t)n):0;
}
static lab_pose pose(const struct odom_state *s){uint32_t q=0;if(s->quality==ODOM_UNCALIBRATED||s->quality==ODOM_FUSED||s->quality==ODOM_WHEELS)q|=LAB_POSE_VALID;if(!s->calibrated)q|=LAB_POSE_UNCALIBRATED;if(s->quality==ODOM_WAIT||s->quality==ODOM_STALE)q|=LAB_POSE_STALE;if(s->quality==ODOM_GAP)q|=LAB_POSE_GAP;if(s->quality==ODOM_SLIP||s->slip)q|=LAB_POSE_SLIP;if(s->imu_used)q|=LAB_POSE_IMU_USED;return (lab_pose){s->x,s->y,s->heading,s->total_distance,q,s->calibrated,s->uncertainty_m};}
int main(int argc,char **argv){const char *bind_ip=NULL,*peer_ip=NULL,*path=NULL,*config=NULL,*csv_path=NULL;int run=0;uint64_t version=1,cpr=1320;
 for(int i=1;i<argc;++i){const char *a=argv[i];if(!strcmp(a,"--help")){puts("lab-real-odom-source [--run] --bind RK_IP --peer FPGA_IP --cache NEW.odt [--config odom.conf] [--parameter-version 1] [--cpr 1320] [--csv NEW.csv]\nUnique read-only UDP5010 owner; no motor commands. Never run together with old receiver. Default preflight opens nothing.");return 0;}if(!strcmp(a,"--info")){puts("RK canonical ODT1 cache: OD01 raw FPGA tick + RK CLOCK_MONOTONIC receipt + unchanged odometry fusion; no motor output");return 0;}if(!strcmp(a,"--run")){run=1;continue;}if(i+1>=argc)return 2;const char *v=argv[++i];if(!strcmp(a,"--bind"))bind_ip=v;else if(!strcmp(a,"--peer"))peer_ip=v;else if(!strcmp(a,"--cache"))path=v;else if(!strcmp(a,"--config"))config=v;else if(!strcmp(a,"--csv"))csv_path=v;else if(!strcmp(a,"--parameter-version")){if(!num(v,&version))return 2;}else if(!strcmp(a,"--cpr")){if(!num(v,&cpr)||cpr>1000000)return 2;}else return 2;}
 struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(5010)};struct in_addr peer;if(!bind_ip||!peer_ip||!path||inet_pton(AF_INET,bind_ip,&address.sin_addr)!=1||inet_pton(AF_INET,peer_ip,&peer)!=1)return 2;
 struct odom_config parameters;odom_default_config(&parameters);if(config&&odom_load_config(&parameters,config)){fprintf(stderr,"Invalid odometry configuration\n");return 2;}uint32_t identity=config_crc(&parameters,(uint32_t)cpr);if(!identity)return 2;
 if(!run){printf("PREFLIGHT ONLY UDP=%s:5010 peer=%s cache=%s config_crc=%08x calibrated=%d; no socket/file created\n",bind_ip,peer_ip,path,identity,parameters.calibrated);return 0;}
 int fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(fd<0||bind(fd,(struct sockaddr*)&address,sizeof(address))){fprintf(stderr,"Cannot own UDP5010; stop the previous receiver explicitly before choosing this one.\n");if(fd>=0)close(fd);return 3;}int size=65536;setsockopt(fd,SOL_SOCKET,SO_RCVBUF,&size,sizeof(size));int output=-1;
 if(lab_real_odom_cache_create(path,&output)!=LAB_OK){fprintf(stderr,"Cache must be a NEW regular path, no overwrite: %s\n",path);close(fd);return 4;}
 FILE *csv=NULL;if(csv_path){csv=fopen(csv_path,"wbx");if(!csv){close(output);close(fd);return 4;}fprintf(csv,"source,producer_id,session,accepted_sequence,fpga_sequence,fpga_ms,rx_us,published_us,imu_age_ms,encoder_mask,config_crc32\n");}
 uint64_t id=0;if(getrandom(&id,sizeof(id),0)!=(ssize_t)sizeof(id))id=now_us()^((uint64_t)getpid()<<32);if(!id)id=1;struct odom_model model;odom_init(&model,&parameters);uint64_t accepted=0,rejected=0,last_log=0;uint32_t generation=0;int result=0;
 signal(SIGINT,stop);signal(SIGTERM,stop);while(running){struct pollfd p={fd,POLLIN,0};int ready=poll(&p,1,100);if(ready<0){if(errno==EINTR)continue;result=5;break;}if(!ready)continue;for(unsigned i=0;i<64;++i){uint8_t raw[64];struct sockaddr_in from;socklen_t n=sizeof(from);ssize_t len=recvfrom(fd,raw,sizeof(raw),MSG_DONTWAIT|MSG_TRUNC,(struct sockaddr*)&from,&n);uint64_t rx=now_us();if(len<0){if(errno==EINTR)continue;break;}if(n!=sizeof(from)||from.sin_family!=AF_INET||from.sin_addr.s_addr!=peer.s_addr||!lab_real_od01_valid(raw,(size_t)len)){++rejected;continue;}
   uint32_t previous=model.state.accepted;int status=odom_push(&model,raw,64,rx/1000);if(status<0||model.state.accepted==previous){++rejected;continue;}
   lab_real_odom_time c={0};c.producer_id=id;c.parameter_version=version;c.counts_per_revolution=(uint32_t)cpr;c.config_crc32=identity;c.session=model.state.session;c.accepted_sequence=model.state.accepted;c.received_us=rx;c.published_us=now_us();c.pose=pose(&model.state);memcpy(c.raw_od01,raw,64);generation+=2;if(!generation)generation=2;c.generation=generation;
   if(lab_real_odom_cache_write(output,&c)!=LAB_OK){result=6;running=0;break;}++accepted;
   if(csv){uint32_t seq=(uint32_t)raw[8]|(uint32_t)raw[9]<<8|(uint32_t)raw[10]<<16|(uint32_t)raw[11]<<24;uint32_t ms=(uint32_t)raw[12]|(uint32_t)raw[13]<<8|(uint32_t)raw[14]<<16|(uint32_t)raw[15]<<24;uint32_t age=(uint32_t)raw[40]|(uint32_t)raw[41]<<8|(uint32_t)raw[42]<<16|(uint32_t)raw[43]<<24;
    if(fprintf(csv,"HARDWARE,%llu,%u,%u,%u,%u,%llu,%llu,%u,%u,%08x\n",(unsigned long long)id,c.session,c.accepted_sequence,seq,ms,(unsigned long long)rx,(unsigned long long)c.published_us,age,raw[5]?0:raw[6],identity)<0){result=7;running=0;break;}}
   if(rx-last_log>=1000000){printf("ODT1 accepted=%llu rejected=%llu config_crc=%08x calibrated=%d raw_tick_preserved=1 clock_bound=UNCONFIRMED\n",(unsigned long long)accepted,(unsigned long long)rejected,identity,parameters.calibrated);fflush(stdout);last_log=rx;}
  }}if(csv&&fclose(csv))result=7;close(output);close(fd);return result;
}
