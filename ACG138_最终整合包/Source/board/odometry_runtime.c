#define _GNU_SOURCE
#include "odometry_runtime.h"
#include "mpu9250/mpu_adapter.h"
#include <pthread.h>
#include <math.h>
#include <stdatomic.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
static struct {pthread_t thread;pthread_mutex_t mutex;_Atomic int stop;int fd,started;uint32_t peer;struct mpu_adapter adapter;struct odom_shared*shared;} rt;
static uint64_t now_ms(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000;}
static void publish(void){
 if(!rt.shared)return;
 __atomic_add_fetch(&rt.shared->sequence,1,__ATOMIC_SEQ_CST);rt.shared->state=rt.adapter.odom.state;__atomic_add_fetch(&rt.shared->sequence,1,__ATOMIC_RELEASE);
}
static void diagnostic(uint64_t now){
 const char*dir=getenv("ACG_PHONE_DIR");if(!dir||!*dir)return;
 char path[1024],tmp[1024],json[2048];
 if(snprintf(path,sizeof(path),"%s/mpu9250-status.json",dir)>=(int)sizeof(path)||
    snprintf(tmp,sizeof(tmp),"%s/mpu9250-status.json.tmp",dir)>=(int)sizeof(tmp))return;
 size_t n=mpu_adapter_json(&rt.adapter,json,sizeof(json),now);
 int fd=open(tmp,O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC|O_NOFOLLOW,0600);
 if(fd<0)return;
 ssize_t written=write(fd,json,n);close(fd);
 if(written==(ssize_t)n)rename(tmp,path);
}
static void*receive_loop(void*unused){
 (void)unused;uint64_t logged=0,diagnosed=0;
 while(!atomic_load(&rt.stop)){
  uint64_t tick=now_ms();
  if(tick-diagnosed>=1000){pthread_mutex_lock(&rt.mutex);diagnostic(tick);pthread_mutex_unlock(&rt.mutex);diagnosed=tick;}
  struct pollfd f={rt.fd,POLLIN,0};if(poll(&f,1,100)<=0)continue;
  for(unsigned i=0;i<64;i++){
   uint8_t p[100];struct sockaddr_in from;socklen_t bytes=sizeof(from);
   ssize_t n=recvfrom(rt.fd,p,sizeof(p),MSG_DONTWAIT|MSG_TRUNC,(struct sockaddr*)&from,&bytes);
   if(n<0){if(errno==EINTR)continue;break;}
   if(rt.peer &&from.sin_addr.s_addr!=rt.peer)continue;
   uint64_t now=now_ms();pthread_mutex_lock(&rt.mutex);mpu_adapter_push(&rt.adapter,p,(size_t)n,now);publish();
   if(now-logged>=5000){struct odom_state*s=&rt.adapter.odom.state;fprintf(stderr,"ODOMETRY x=%.3f y=%.3f heading=%.1f distance=%.3f quality=%u accepted=%u rejected=%u gaps=%u counts=%d,%d,%d,%d\n",s->x,s->y,s->heading*57.295779513,s->total_distance,s->quality,s->accepted,s->rejected,s->gaps,s->raw_counts[0],s->raw_counts[1],s->raw_counts[2],s->raw_counts[3]);fprintf(stderr,"MPU9250 bias=%s samples=%u ID=%02x/%02x I2C=%u err=%u raw_seq=%u restarts=%llu\n",
    rt.adapter.fusion.bias_ready?"READY":"KEEP_STILL",rt.adapter.fusion.cal_count,
    rt.adapter.fusion.packet.whoami,rt.adapter.fusion.packet.mag_id,
    rt.adapter.fusion.packet.errors,rt.adapter.fusion.packet.i2c_error,rt.adapter.fusion.packet.sequence,
    (unsigned long long)rt.adapter.restarts);logged=now;}
   pthread_mutex_unlock(&rt.mutex);
  }
 }
 return NULL;
}
int odom_runtime_start(const char*bind_address,const char*peer){
 memset(&rt,0,sizeof(rt));rt.fd=-1;pthread_mutex_init(&rt.mutex,NULL);
 struct odom_config config;odom_default_config(&config);const char*path=getenv("ODOM_CONFIG");
 if(path&&odom_load_config(&config,path)){fprintf(stderr,"ODOMETRY invalid/missing configuration: %s; disabled, motor control unchanged.\n",path);return -1;}
 struct i9_config fusion;i9_default_config(&fusion);
 const char*mpu_path=getenv("MPU9250_CONFIG");
 if(mpu_path&&mpu_config_load(&fusion,mpu_path)){
  fprintf(stderr,"MPU9250 invalid configuration: %s; acquisition/pose disabled, original control retained.\n",mpu_path);return -1;
 }
 if(mpu_adapter_init(&rt.adapter,&config,&fusion))return -1;
 struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(5016)},sender;
 if(inet_pton(AF_INET,bind_address,&addr.sin_addr)!=1||inet_pton(AF_INET,peer,&sender.sin_addr)!=1)return -1;
 rt.peer=sender.sin_addr.s_addr;
 rt.fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
 if(rt.fd<0||bind(rt.fd,(struct sockaddr*)&addr,sizeof(addr))){if(rt.fd>=0)close(rt.fd);rt.fd=-1;return -1;}
 const char*dir=getenv("ACG_PHONE_DIR");
 if(dir&&*dir){char file[1024];if(snprintf(file,sizeof(file),"%s/odometry.shm",dir)<(int)sizeof(file)){
   int fd=open(file,O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
   if(fd>=0){if(!ftruncate(fd,sizeof(struct odom_shared)))rt.shared=mmap(NULL,sizeof(*rt.shared),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);close(fd);
    if(rt.shared==MAP_FAILED)rt.shared=NULL;
    if(rt.shared){memset(rt.shared,0,sizeof(*rt.shared));rt.shared->version=1;rt.shared->bytes=sizeof(*rt.shared);__atomic_store_n(&rt.shared->magic,ODOM_MAGIC,__ATOMIC_RELEASE);}}
  }}
 if(pthread_create(&rt.thread,NULL,receive_loop,NULL)){close(rt.fd);rt.fd=-1;return -1;}
 rt.started=1;fprintf(stderr,"MPU9250 passive UDP5016 receiver; calibrated=%d; RK attitude + existing odometry; no motor command path.\n",config.calibrated);return 0;
}
void odom_runtime_imu_snapshot(struct imu_model*m){
 pthread_mutex_lock(&rt.mutex);*m=rt.adapter.imu;pthread_mutex_unlock(&rt.mutex);
}
void odom_runtime_snapshot(struct odom_state*s){pthread_mutex_lock(&rt.mutex);*s=rt.adapter.odom.state;pthread_mutex_unlock(&rt.mutex);}
void odom_runtime_stop(void){if(rt.started){atomic_store(&rt.stop,1);pthread_join(rt.thread,NULL);close(rt.fd);rt.started=0;}if(rt.shared){munmap(rt.shared,sizeof(*rt.shared));rt.shared=NULL;}}
int odom_shared_read(const char*dir,struct odom_state*s){
 char path[1024];if(!dir||snprintf(path,sizeof(path),"%s/odometry.shm",dir)>=(int)sizeof(path))return -1;
 int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);if(fd<0)return -1;
 struct stat st;if(fstat(fd,&st)||st.st_size!=(off_t)sizeof(struct odom_shared)){close(fd);return -1;}
 const struct odom_shared*m=mmap(NULL,sizeof(*m),PROT_READ,MAP_SHARED,fd,0);close(fd);if(m==MAP_FAILED)return -1;
 int result=-1;if(__atomic_load_n(&m->magic,__ATOMIC_ACQUIRE)==ODOM_MAGIC&&m->version==1&&m->bytes==sizeof(*m)){
  for(int i=0;i<5;i++){uint32_t a=__atomic_load_n(&m->sequence,__ATOMIC_ACQUIRE);if(a&1)continue;memcpy(s,&m->state,sizeof(*s));__atomic_thread_fence(__ATOMIC_ACQUIRE);
    if(a==__atomic_load_n(&m->sequence,__ATOMIC_ACQUIRE)&&s->points<=ODOM_POINTS&&isfinite(s->x)&&isfinite(s->y)&&isfinite(s->heading)&&isfinite(s->total_distance)){result=0;break;}}
 }
 munmap((void*)m,sizeof(*m));return result;
}
