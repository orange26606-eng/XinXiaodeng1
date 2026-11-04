#define _GNU_SOURCE
#include "lab_real_mpusvc_posix.h"
#include "lab_codec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
uint64_t mpusvc_now(void){LARGE_INTEGER n,f;QueryPerformanceCounter(&n);QueryPerformanceFrequency(&f);return (uint64_t)((long double)n.QuadPart*1000000/f.QuadPart);}
int mpusvc_run_posix(const mpusvc_options *o,const struct odom_config *d,const struct i9_config *i){(void)o;(void)d;(void)i;fputs("Windows build supports offline inspection/reprocessing only; no network opened.\n",stderr);return 3;}
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stddef.h>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
uint64_t mpusvc_now(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;}
static volatile sig_atomic_t keep_running=1;
static void stop_signal(int n){(void)n;keep_running=0;}
typedef struct raw_item {uint8_t packet[100];uint64_t rx;} raw_item;
typedef struct service {
 const mpusvc_options *options;mpusvc_core *core;pthread_mutex_t mutex;atomic_int stop;
 raw_item queue[64];unsigned head,count;uint64_t queue_drops;int record_enabled;
 int lockfd,legacyfd,rawfd,io_error;uint32_t legacy_sequence;uint64_t export_sequence;
 char snapshot[512],legacy[512],diagnostic[512];FILE *record;mpusvc_history_writer history;
 uint64_t records,record_io_drops,observed;uint32_t stream_crc;
} service;
static int write_all(int fd,const void *data,size_t n,off_t offset){size_t done=0;while(done<n){ssize_t k=pwrite(fd,(const uint8_t*)data+done,n-done,offset+(off_t)done);if(k<0&&errno==EINTR)continue;if(k<=0)return -1;done+=(size_t)k;}return 0;}
static int atomic_file(const char *path,const void *bytes,size_t n){char tmp[576];if(snprintf(tmp,sizeof(tmp),"%s.tmp.%ld",path,(long)getpid())>=(int)sizeof(tmp))return -1;
 int fd=open(tmp,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)return -1;int bad=write_all(fd,bytes,n,0);if(close(fd))bad=1;if(!bad&&rename(tmp,path))bad=1;if(bad)unlink(tmp);return bad?-1:0;
}
static int legacy_start(service *s){struct odom_shared m={.magic=ODOM_MAGIC,.version=1,.bytes=sizeof(m)};char tmp[576];if(snprintf(tmp,sizeof(tmp),"%s.new.%ld",s->legacy,(long)getpid())>=(int)sizeof(tmp))return -1;
 int fd=open(tmp,O_RDWR|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)return -1;if(write_all(fd,&m,sizeof(m),0)||rename(tmp,s->legacy)){close(fd);unlink(tmp);return -1;}s->legacyfd=fd;return 0;
}
static int legacy_publish(service *s,const mpusvc_view *v,int stopped){
 struct odom_state o=v->odometry;if(stopped)o.quality=ODOM_STALE;uint32_t odd=s->legacy_sequence+1,even=s->legacy_sequence+2;
 if(write_all(s->legacyfd,&odd,4,(off_t)offsetof(struct odom_shared,sequence))||write_all(s->legacyfd,&o,sizeof(o),(off_t)offsetof(struct odom_shared,state))||write_all(s->legacyfd,&even,4,(off_t)offsetof(struct odom_shared,sequence)))return -1;s->legacy_sequence=even;return 0;
}
static void diagnostics(service *s,const mpusvc_view *v,uint64_t now){char b[4096];uint64_t age=now>=v->raw_received_us?(now-v->raw_received_us)/1000:UINT64_MAX;
 int n=snprintf(b,sizeof(b),"{\"source\":\"MPU9250_RAW_RK_FUSION\",\"service\":\"HEADLESS\",\"motor_sender\":false,\"seen\":%s,\"stopped\":%s,\"age_ms\":%llu,\"bias_ready\":%s,\"cal_samples\":%u,\"whoami\":%u,\"mag_id\":%u,\"flags\":%u,\"last_error\":%u,\"i2c_errors\":%u,\"sequence\":%u,\"quality\":\"%s\",\"time_error\":\"UNKNOWN\",\"rpy_deg\":[%.5f,%.5f,%.5f],\"rpm\":[%.3f,%.3f,%.3f,%.3f],\"temperature_c\":%.2f,\"record_count\":%llu,\"record_drops\":%llu,\"record_error\":%s,\"config_crc\":%u,\"parameter_version\":%llu}\n",v->flags&MPUSVC_SEEN?"true":"false",v->flags&MPUSVC_STOPPED?"true":"false",(unsigned long long)age,v->bias_ready?"true":"false",v->cal_samples,v->raw[6],v->raw[7],v->raw[5],v->raw[63],(unsigned)(v->raw[64]|(uint32_t)v->raw[65]<<8|(uint32_t)v->raw[66]<<16|(uint32_t)v->raw[67]<<24),(unsigned)(v->raw[8]|(uint32_t)v->raw[9]<<8|(uint32_t)v->raw[10]<<16|(uint32_t)v->raw[11]<<24),i9_quality_name((int)v->fusion_quality),(double)(v->rpy[0]*57.29577951308232),(double)(v->rpy[1]*57.29577951308232),(double)(v->rpy[2]*57.29577951308232),v->rpm[0],v->rpm[1],v->rpm[2],v->rpm[3],v->temperature,(unsigned long long)v->record_count,(unsigned long long)v->record_drops,v->flags&MPUSVC_RECORD_ERROR?"true":"false",v->config_crc,(unsigned long long)v->parameter_version);
 if(n<=0||(size_t)n>=sizeof(b)||atomic_file(s->diagnostic,b,(size_t)n))s->io_error=1;
}
static void *publish_loop(void *user){service *s=user;uint64_t logged=0;for(;;){
  raw_item batch[64];unsigned count=0;uint64_t drops=0;mpusvc_view v;int stopping=atomic_load(&s->stop);
  pthread_mutex_lock(&s->mutex);mpusvc_snapshot(s->core,mpusvc_now(),&v);drops=s->queue_drops;
  while(s->count&&count<64){batch[count++]=s->queue[s->head];s->head=(s->head+1)%64;--s->count;}pthread_mutex_unlock(&s->mutex);
  for(unsigned i=0;i<count;++i){if(!s->record||s->io_error||s->records>=524287){++s->record_io_drops;continue;}uint8_t b[128];mpusvc_capture_record(b,batch[i].packet,batch[i].rx,s->records+1);if(fwrite(b,1,128,s->record)!=128){s->io_error=1;++s->record_io_drops;continue;}++s->records;s->stream_crc=lab_crc32_extend(s->stream_crc,b,128);}
  v.export_sequence=++s->export_sequence;v.record_count=s->records;v.record_drops=drops+s->record_io_drops;v.latest_skipped=v.accepted>s->observed+1?v.accepted-s->observed-1:0;s->observed=v.accepted;if(s->io_error||s->record_io_drops)v.flags|=MPUSVC_RECORD_ERROR;
  if(stopping){v.flags=(v.flags&~MPUSVC_RUNNING)|MPUSVC_STOPPED;if(s->record){uint8_t footer[40];mpusvc_capture_footer(footer,s->records,v.record_drops,s->stream_crc,1);if(fwrite(footer,1,40,s->record)!=40||fflush(s->record))s->io_error=1;if(s->io_error)v.flags|=MPUSVC_RECORD_ERROR;}}
  uint8_t b[768];if(mpusvc_history_append(&s->history,&v)!=LAB_OK||mpusvc_encode(b,&v)!=LAB_OK||atomic_file(s->snapshot,b,sizeof(b))||legacy_publish(s,&v,stopping))s->io_error=1;
  if(stopping||!logged||v.published_us-logged>=1000000){diagnostics(s,&v,v.published_us);mpusvc_print(&v,v.published_us);logged=v.published_us;}
  if(stopping)break;struct timespec wait={0,20000000};nanosleep(&wait,NULL);
 }return NULL;}
int mpusvc_run_posix(const mpusvc_options *o,const struct odom_config *odom,const struct i9_config *imu){
 if(!o||o->state_dir[0]!='/'||strlen(o->state_dir)>240)return 2;service s={.options=o,.lockfd=-1,.legacyfd=-1,.rawfd=-1};int result=3,mutex_ready=0,thread_ready=0;pthread_t thread;
 struct stat st;if(mkdir(o->state_dir,0700)&&errno!=EEXIST){perror("state-dir");return 3;}if(lstat(o->state_dir,&st)||!S_ISDIR(st.st_mode)||st.st_uid!=geteuid()){fputs("state-dir must be an owned regular directory, no symlink\n",stderr);return 3;}
 char path[512];snprintf(path,sizeof(path),"%s/.mpu-service.lock",o->state_dir);s.lockfd=open(path,O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);if(s.lockfd<0||flock(s.lockfd,LOCK_EX|LOCK_NB)){fputs("SERVICE_STATE_ALREADY_OWNED\n",stderr);goto done;}
 struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(5016)};struct in_addr peer;if(inet_pton(AF_INET,o->bind_ip,&address.sin_addr)!=1||inet_pton(AF_INET,o->peer_ip,&peer)!=1){fputs("invalid explicit IP\n",stderr);goto done;}
 s.rawfd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);int buffer=65536;
 if(s.rawfd<0||setsockopt(s.rawfd,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer))||bind(s.rawfd,(struct sockaddr*)&address,sizeof(address))){perror("UDP5016 bind: stop the old IMU receiver explicitly; no takeover/reuse");goto done;}
 uint64_t id=0;if(getrandom(&id,sizeof(id),0)!=(ssize_t)sizeof(id)||!id)id=(mpusvc_now()^((uint64_t)getpid()<<32))|1;
 if(mpusvc_open(&s.core,LAB_REAL_HARDWARE,id,o->parameter_version,odom,imu)!=LAB_OK)goto done;
 snprintf(s.snapshot,sizeof(s.snapshot),"%s/mpu9250.snapshot",o->state_dir);snprintf(s.legacy,sizeof(s.legacy),"%s/odometry.shm",o->state_dir);snprintf(s.diagnostic,sizeof(s.diagnostic),"%s/mpu9250-status.json",o->state_dir);
 snprintf(path,sizeof(path),"%s/mpu9250.history",o->state_dir);if(mpusvc_history_create(&s.history,path)!=LAB_OK){perror("history file");goto done;}
 if(legacy_start(&s)){perror("legacy shared file");goto done;}
 if(o->record_path[0]){s.record=fopen(o->record_path,"wbx");if(!s.record){perror("capture new file");goto done;}s.record_enabled=1;mpusvc_view v;mpusvc_snapshot(s.core,mpusvc_now(),&v);uint8_t h[64];mpusvc_capture_header(h,v.source,v.instance,v.published_us,v.parameter_version,v.config_crc);if(fwrite(h,1,64,s.record)!=64)goto done;}
 if(pthread_mutex_init(&s.mutex,NULL))goto done;mutex_ready=1;atomic_init(&s.stop,0);if(pthread_create(&thread,NULL,publish_loop,&s))goto done;thread_ready=1;keep_running=1;signal(SIGINT,stop_signal);signal(SIGTERM,stop_signal);struct sigaction inherited;if(!sigaction(SIGHUP,NULL,&inherited)&&inherited.sa_handler!=SIG_IGN)signal(SIGHUP,stop_signal);
 uint64_t start=mpusvc_now();result=0;fprintf(stderr,"MPU headless receiver %s:5016 peer=%s state=%s; no Wayland/camera/motor sender\n",o->bind_ip,o->peer_ip,o->state_dir);
 while(keep_running&&(!o->duration_us||mpusvc_now()-start<o->duration_us)){
  struct pollfd f={s.rawfd,POLLIN,0};int ready=poll(&f,1,100);if(ready<0){if(errno==EINTR)continue;result=5;break;}if(!ready)continue;
  for(unsigned i=0;i<64;++i){uint8_t p[128];struct sockaddr_in from;socklen_t n=sizeof(from);ssize_t bytes=recvfrom(s.rawfd,p,sizeof(p),MSG_DONTWAIT|MSG_TRUNC,(struct sockaddr*)&from,&n);
   if(bytes<0){if(errno==EINTR)continue;if(errno==EAGAIN||errno==EWOULDBLOCK)break;result=5;keep_running=0;break;}
   if(n!=sizeof(from)||from.sin_family!=AF_INET||from.sin_addr.s_addr!=peer.s_addr)continue;uint64_t rx=mpusvc_now();pthread_mutex_lock(&s.mutex);
   lab_status status=mpusvc_push(s.core,p,(size_t)bytes,rx);if(status==LAB_ERR_TIME){result=5;keep_running=0;}
   if(s.record_enabled&&bytes==100){if(s.count==64)++s.queue_drops;else{raw_item *item=&s.queue[(s.head+s.count)%64];memcpy(item->packet,p,100);item->rx=rx;++s.count;}}
   pthread_mutex_unlock(&s.mutex);
  }
 }
done:
 if(thread_ready){atomic_store(&s.stop,1);pthread_join(thread,NULL);}mpusvc_history_close(&s.history);if(s.record&&fclose(s.record))s.io_error=1;if(mutex_ready)pthread_mutex_destroy(&s.mutex);mpusvc_close(s.core);if(s.rawfd>=0)close(s.rawfd);if(s.legacyfd>=0)close(s.legacyfd);if(s.lockfd>=0)close(s.lockfd);return s.io_error?6:result;
}
#endif
