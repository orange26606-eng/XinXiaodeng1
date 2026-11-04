#define _GNU_SOURCE
#include "lab_real_mpusvc_posix.h"
#include "lab_codec.h"
#include "mpusvc_arrival_guard.h"
#include "mpusvc_error_budget.h"
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
 int lockfd,legacyfd,rawfd,attfd,attitude_port_error,io_error;unsigned logged_attitude_source;char logged_attitude_reason[64];uint32_t legacy_sequence;uint64_t export_sequence;
 char snapshot[512],timed_snapshot[512],legacy[512],diagnostic[512];FILE *record;mpusvc_history_writer history;
 uint64_t records,record_io_drops,observed;uint32_t stream_crc;
 int have_clock_bridge;int64_t clock_bridge_us;uint64_t last_arrival_us,arrival_clamps;
 mpusvc_error_budget timing;atomic_uint_fast64_t timing_total;atomic_uint timing_consecutive,timing_window;
} service;
static int kernel_arrival_us(service *s,struct msghdr *msg,uint64_t *out){
 struct timespec *stamp=NULL;struct cmsghdr *c=CMSG_FIRSTHDR(msg);if(c&&c->cmsg_level==SOL_SOCKET&&c->cmsg_type==SO_TIMESTAMPNS&&c->cmsg_len>=CMSG_LEN(sizeof(struct timespec)))stamp=(struct timespec*)CMSG_DATA(c);
 if(!stamp||msg->msg_flags&MSG_CTRUNC||stamp->tv_sec<0||stamp->tv_nsec<0||stamp->tv_nsec>=1000000000L){fprintf(stderr,"MPU_KERNEL_TIMESTAMP_MISSING_OR_INVALID\n");return -1;}
 struct timespec m1,real,m2;int64_t a=0,b=0;int good=0;
 for(int attempt=0;attempt<8;attempt++){
  if(clock_gettime(CLOCK_MONOTONIC,&m1)||clock_gettime(CLOCK_REALTIME,&real)||clock_gettime(CLOCK_MONOTONIC,&m2))return -1;
  a=(int64_t)m1.tv_sec*1000000+m1.tv_nsec/1000;b=(int64_t)m2.tv_sec*1000000+m2.tv_nsec/1000;
  if(b>=a&&b-a<=1000){good=1;break;}
 }
 int64_t wall=(int64_t)real.tv_sec*1000000+real.tv_nsec/1000,kernel=(int64_t)stamp->tv_sec*1000000+stamp->tv_nsec/1000;
 if(!good){fprintf(stderr,"MPU_CLOCK_BRIDGE_SAMPLING_ERROR retries=8\n");return -1;}
 int64_t bridge=b-wall;if(s->have_clock_bridge&&(bridge-s->clock_bridge_us>2000||s->clock_bridge_us-bridge>2000)){fprintf(stderr,"MPU_CLOCK_REALTIME_STEP_NEW_CONTEXT_REQUIRED\n");return -1;}
 s->clock_bridge_us=bridge;s->have_clock_bridge=1;int64_t mapped=kernel+bridge;
 if(mapped<0||mapped>b+1000){fprintf(stderr,"MPU_KERNEL_TIMESTAMP_OUT_OF_RANGE\n");return -1;}*out=(uint64_t)mapped;return 0;
}
static int timing_event(service*s,int error,const char*reason,uint64_t rx,unsigned sequence){
 uint64_t now=mpusvc_now();int exhausted=mpusvc_error_budget_event(&s->timing,now,error);
 atomic_store(&s->timing_total,s->timing.total);atomic_store(&s->timing_consecutive,s->timing.consecutive);atomic_store(&s->timing_window,s->timing.count);
 if(error&&(s->timing.total<=5||s->timing.total%50==0||exhausted))fprintf(stderr,"MPU_TIMING_DROP reason=%s sequence=%u rx_us=%llu previous_rx_us=%llu total=%llu consecutive=%u window10s=%u exiting=%u\n",reason,sequence,(unsigned long long)rx,(unsigned long long)s->last_arrival_us,(unsigned long long)s->timing.total,s->timing.consecutive,s->timing.count,exhausted!=0);
 return exhausted;
}
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
 int n=snprintf(b,sizeof(b),"{\"source\":\"%s\",\"service\":\"HEADLESS\",\"motor_sender\":false,\"seen\":%s,\"stopped\":%s,\"age_ms\":%llu,\"bias_ready\":%s,\"cal_samples\":%u,\"whoami\":%u,\"mag_id\":%u,\"flags\":%u,\"last_error\":%u,\"i2c_errors\":%u,\"sequence\":%u,\"quality\":\"%s\",\"time_error\":\"UNKNOWN\",\"rpy_deg\":[%.5f,%.5f,%.5f],\"rpm\":[%.3f,%.3f,%.3f,%.3f],\"temperature_c\":%.2f,\"record_count\":%llu,\"record_drops\":%llu,\"record_error\":%s,\"config_crc\":%u,\"parameter_version\":%llu}\n",(v->attitude_source==F6_SOURCE_FPGA?"FPGA6_ATTITUDE_ENCODER_ODOMETRY":"MPU9250_RAW_RK_FUSION"),v->flags&MPUSVC_SEEN?"true":"false",v->flags&MPUSVC_STOPPED?"true":"false",(unsigned long long)age,v->bias_ready?"true":"false",v->cal_samples,v->raw[6],v->raw[7],v->raw[5],v->raw[63],(unsigned)(v->raw[64]|(uint32_t)v->raw[65]<<8|(uint32_t)v->raw[66]<<16|(uint32_t)v->raw[67]<<24),(unsigned)(v->raw[8]|(uint32_t)v->raw[9]<<8|(uint32_t)v->raw[10]<<16|(uint32_t)v->raw[11]<<24),i9_quality_name((int)v->fusion_quality),(double)(v->rpy[0]*57.29577951308232),(double)(v->rpy[1]*57.29577951308232),(double)(v->rpy[2]*57.29577951308232),v->rpm[0],v->rpm[1],v->rpm[2],v->rpm[3],v->temperature,(unsigned long long)v->record_count,(unsigned long long)v->record_drops,v->flags&MPUSVC_RECORD_ERROR?"true":"false",v->config_crc,(unsigned long long)v->parameter_version);
 if(n>1&&(size_t)n<sizeof(b)){
  size_t used=(size_t)n;while(used&&((b[used-1]=='\n')||(b[used-1]=='\r'))) --used;
  if(used&&b[used-1]=='}'){int added=snprintf(b+used-1,sizeof(b)-used+1,",\"timing_errors\":%llu,\"timing_error_consecutive\":%u,\"timing_errors_window10s\":%u}\n",(unsigned long long)atomic_load(&s->timing_total),(unsigned)atomic_load(&s->timing_consecutive),(unsigned)atomic_load(&s->timing_window));if(added>0&&(size_t)added<sizeof(b)-used+1)n=(int)(used-1)+(int)added;else n=-1;}
 }
 if(n>1&&(size_t)n<sizeof(b)){
  size_t used=(size_t)n;while(used&&(b[used-1]=='\n'||b[used-1]=='\r'))--used;
  if(used&&b[used-1]=='}'){int extra=snprintf(b+used-1,sizeof(b)-used+1,",\"attitude_source\":\"%s\",\"attitude_source_reason\":\"%s\",\"attitude_mode\":%u,\"attitude_port_error\":%d,\"fpga_attitude\":{\"epoch\":%u,\"ag_sequence\":%u,\"source_us\":%u,\"status\":%u,\"flags\":%u,\"source_age_us\":%u,\"received_us\":\"%llu\",\"paired\":%llu,\"unmatched\":%llu,\"received\":%llu,\"rejected\":%llu,\"crc_errors\":%llu,\"duplicates\":%llu,\"reorders\":%llu,\"source_switches\":%llu,\"raw_wait_us\":%llu,\"pending_overflow\":%llu,\"yaw_offset_rad\":%.9g}}\n",f6_source_name(v->attitude_source),v->attitude_reason,s->options->attitude_mode,s->attitude_port_error,v->attitude_epoch,v->attitude_ag_sequence,v->attitude_source_us,v->attitude_status,v->attitude_flags,v->attitude_age_us,(unsigned long long)v->attitude_received_us,(unsigned long long)v->attitude_matches,(unsigned long long)v->attitude_misses,(unsigned long long)v->attitude_received,(unsigned long long)v->attitude_rejected,(unsigned long long)v->attitude_crc_errors,(unsigned long long)v->attitude_duplicates,(unsigned long long)v->attitude_reorders,(unsigned long long)v->attitude_switches,(unsigned long long)v->raw_wait_us,(unsigned long long)v->raw_pending_overflow,v->attitude_yaw_offset);
   if(extra>0&&(size_t)extra<sizeof(b)-used+1)n=(int)(used-1)+(int)extra;else n=-1;
  }
 }
 fprintf(stderr,"IMU_PAIR raw_epoch=%u raw_seq=%u ag_seq=%u raw_source_us=%u raw_rx_us=%llu F6_epoch=%u F6_ag_seq=%u F6_source_us=%u status=%u flags=%u source_age_us=%u F6_rx_us=%llu source=%s reason=%s pairs=%llu misses=%llu rejected=%llu crc=%llu duplicates=%llu reorder=%llu wait_us=%llu overflow=%llu offset_rad=%.7f restarts=%llu counts=%x pair_epoch=%u pair_ag_seq=%u pair_source_us=%u measurement_rx_us=%llu\n",v->epoch,(unsigned)(v->raw[8]|(uint32_t)v->raw[9]<<8|(uint32_t)v->raw[10]<<16|(uint32_t)v->raw[11]<<24),v->ag_sequence,(unsigned)(v->raw[16]|(uint32_t)v->raw[17]<<8|(uint32_t)v->raw[18]<<16|(uint32_t)v->raw[19]<<24),(unsigned long long)v->raw_received_us,v->attitude_epoch,v->attitude_ag_sequence,v->attitude_source_us,v->attitude_status,v->attitude_flags,v->attitude_age_us,(unsigned long long)v->attitude_received_us,f6_source_name(v->attitude_source),v->attitude_reason,(unsigned long long)v->attitude_matches,(unsigned long long)v->attitude_misses,(unsigned long long)v->attitude_rejected,(unsigned long long)v->attitude_crc_errors,(unsigned long long)v->attitude_duplicates,(unsigned long long)v->attitude_reorders,(unsigned long long)v->raw_wait_us,(unsigned long long)v->raw_pending_overflow,v->attitude_yaw_offset,(unsigned long long)v->restarts,v->counts_mask,v->attitude_paired_epoch,v->attitude_paired_ag_sequence,v->attitude_paired_source_us,(unsigned long long)v->attitude_paired_received_us);
 if(n>1&&(size_t)n<sizeof b){size_t used=(size_t)n;while(used&&(b[used-1]=='\n'||b[used-1]=='\r'))--used;if(used&&b[used-1]=='}'){int extra=snprintf(b+used-1,sizeof b-used+1,",\"attitude_pair\":{\"epoch\":%u,\"ag_sequence\":%u,\"source_us\":%u,\"measurement_received_us\":\"%llu\",\"heartbeat_received_us\":\"%llu\",\"heartbeat_does_not_refresh_measurement\":true}}\n",v->attitude_paired_epoch,v->attitude_paired_ag_sequence,v->attitude_paired_source_us,(unsigned long long)v->attitude_paired_received_us,(unsigned long long)v->attitude_received_us);if(extra>0&&(size_t)extra<sizeof b-used+1)n=(int)(used-1)+(int)extra;else n=-1;}}
 if(n<=0||(size_t)n>=sizeof(b)||atomic_file(s->diagnostic,b,(size_t)n))s->io_error=1;
}
static void *publish_loop(void *user){service *s=user;uint64_t logged=0;for(;;){
  raw_item batch[64];unsigned count=0;uint64_t drops=0;mpusvc_view v;int stopping=atomic_load(&s->stop);
  pthread_mutex_lock(&s->mutex);mpusvc_snapshot(s->core,mpusvc_now(),&v);drops=s->queue_drops;
  while(s->count&&count<64){batch[count++]=s->queue[s->head];s->head=(s->head+1)%64;--s->count;}pthread_mutex_unlock(&s->mutex);
  for(unsigned i=0;i<count;++i){if(!s->record||s->io_error||s->records>=524287){++s->record_io_drops;continue;}uint8_t b[128];mpusvc_capture_record(b,batch[i].packet,batch[i].rx,s->records+1);if(fwrite(b,1,128,s->record)!=128){s->io_error=1;++s->record_io_drops;continue;}++s->records;s->stream_crc=lab_crc32_extend(s->stream_crc,b,128);}
  if(s->options->attitude_mode==F6_MODE_AUTO&&s->attfd<0)snprintf(v.attitude_reason,sizeof v.attitude_reason,"ATTITUDE_PORT_UNAVAILABLE_RK_FALLBACK");
  if(s->logged_attitude_source!=v.attitude_source||strcmp(s->logged_attitude_reason,v.attitude_reason)){
   fprintf(stderr,"IMU_SOURCE previous=%s selected=%s reason=%s raw_epoch=%u raw_seq=%u ag_seq=%u raw_rx_us=%llu F6_seq=%u F6_source_us=%u wait_us=%llu offset_rad=%.7f\n",f6_source_name(s->logged_attitude_source),f6_source_name(v.attitude_source),v.attitude_reason,v.epoch,(unsigned)(v.raw[8]|(uint32_t)v.raw[9]<<8|(uint32_t)v.raw[10]<<16|(uint32_t)v.raw[11]<<24),v.ag_sequence,(unsigned long long)v.raw_received_us,v.attitude_ag_sequence,v.attitude_source_us,(unsigned long long)v.raw_wait_us,v.attitude_yaw_offset);
   s->logged_attitude_source=v.attitude_source;snprintf(s->logged_attitude_reason,sizeof s->logged_attitude_reason,"%s",v.attitude_reason);
  }
  v.export_sequence=++s->export_sequence;v.record_count=s->records;v.record_drops=drops+s->record_io_drops;v.latest_skipped=v.accepted>s->observed+1?v.accepted-s->observed-1:0;s->observed=v.accepted;if(s->io_error||s->record_io_drops)v.flags|=MPUSVC_RECORD_ERROR;
  if(stopping){v.flags=(v.flags&~MPUSVC_RUNNING)|MPUSVC_STOPPED;if(s->record){uint8_t footer[40];mpusvc_capture_footer(footer,s->records,v.record_drops,s->stream_crc,1);if(fwrite(footer,1,40,s->record)!=40||fflush(s->record))s->io_error=1;if(s->io_error)v.flags|=MPUSVC_RECORD_ERROR;}}
  uint8_t b[768];if(mpusvc_history_append(&s->history,&v)!=LAB_OK)s->io_error=1;if(mpusvc_encode(b,&v)!=LAB_OK||atomic_file(s->snapshot,b,sizeof(b))||legacy_publish(s,&v,stopping))s->io_error=1;
  if(mpusvc_encode_timed(b,&v)!=LAB_OK||atomic_file(s->timed_snapshot,b,sizeof(b)))s->io_error=1;
  if(stopping||!logged||v.published_us-logged>=1000000){diagnostics(s,&v,v.published_us);mpusvc_print(&v,v.published_us);logged=v.published_us;}
  if(stopping)break;struct timespec wait={0,20000000};nanosleep(&wait,NULL);
 }return NULL;}
int mpusvc_run_posix(const mpusvc_options *o,const struct odom_config *odom,const struct i9_config *imu){
 if(!o||o->state_dir[0]!='/'||strlen(o->state_dir)>240)return 2;service s={.options=o,.lockfd=-1,.legacyfd=-1,.rawfd=-1,.attfd=-1};int result=3,mutex_ready=0,thread_ready=0;pthread_t thread;
 struct stat st;if(mkdir(o->state_dir,0700)&&errno!=EEXIST){perror("state-dir");return 3;}if(lstat(o->state_dir,&st)||!S_ISDIR(st.st_mode)||st.st_uid!=geteuid()){fputs("state-dir must be an owned regular directory, no symlink\n",stderr);return 3;}
 char path[512];snprintf(path,sizeof(path),"%s/.mpu-service.lock",o->state_dir);s.lockfd=open(path,O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);if(s.lockfd<0||flock(s.lockfd,LOCK_EX|LOCK_NB)){fputs("SERVICE_STATE_ALREADY_OWNED\n",stderr);goto done;}
 struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(5016)};struct in_addr peer;if(inet_pton(AF_INET,o->bind_ip,&address.sin_addr)!=1||inet_pton(AF_INET,o->peer_ip,&peer)!=1){fputs("invalid explicit IP\n",stderr);goto done;}
 s.rawfd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);int buffer=65536,timestamps=1;
 if(s.rawfd<0||setsockopt(s.rawfd,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer))||setsockopt(s.rawfd,SOL_SOCKET,SO_TIMESTAMPNS,&timestamps,sizeof(timestamps))||bind(s.rawfd,(struct sockaddr*)&address,sizeof(address))){perror("UDP5016 bind: stop the old IMU receiver explicitly; no takeover/reuse");goto done;}
 if(o->attitude_mode!=F6_MODE_RK){
  address.sin_port=htons(5017);s.attfd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
  if(s.attfd<0||setsockopt(s.attfd,SOL_SOCKET,SO_RCVBUF,&buffer,sizeof(buffer))||setsockopt(s.attfd,SOL_SOCKET,SO_TIMESTAMPNS,&timestamps,sizeof(timestamps))||bind(s.attfd,(struct sockaddr*)&address,sizeof(address))){
   s.attitude_port_error=errno;perror("UDP5017 attitude bind; no takeover/reuse");if(s.attfd>=0)close(s.attfd);s.attfd=-1;if(o->attitude_mode==F6_MODE_FPGA)goto done;
  }
 }
 uint64_t id=0;if(getrandom(&id,sizeof(id),0)!=(ssize_t)sizeof(id)||!id)id=(mpusvc_now()^((uint64_t)getpid()<<32))|1;
 if(mpusvc_open(&s.core,LAB_REAL_HARDWARE,id,o->parameter_version,odom,imu)!=LAB_OK)goto done;
 if(mpusvc_attitude_mode(s.core,s.attfd<0?F6_MODE_RK:o->attitude_mode)!=LAB_OK)goto done;
 snprintf(s.snapshot,sizeof(s.snapshot),"%s/mpu9250.snapshot",o->state_dir);snprintf(s.legacy,sizeof(s.legacy),"%s/odometry.shm",o->state_dir);snprintf(s.diagnostic,sizeof(s.diagnostic),"%s/mpu9250-status.json",o->state_dir);
 snprintf(s.timed_snapshot,sizeof(s.timed_snapshot),"%s/mpu9250.timed.snapshot",o->state_dir);
 snprintf(path,sizeof(path),"%s/mpu9250.history",o->state_dir);if(mpusvc_history_create(&s.history,path)!=LAB_OK){perror("history file");goto done;}
 if(legacy_start(&s)){perror("legacy shared file");goto done;}
 if(o->record_path[0]){s.record=fopen(o->record_path,"wbx");if(!s.record){perror("capture new file");goto done;}s.record_enabled=1;if(o->attitude_mode!=F6_MODE_RK)fprintf(stderr,"IMU_CAPTURE_RAW_ONLY replay_source=RK_REFERENCE F6_attitude_not_recorded; live snapshot orientation may differ\n");mpusvc_view v;mpusvc_snapshot(s.core,mpusvc_now(),&v);uint8_t h[64];mpusvc_capture_header(h,v.source,v.instance,v.published_us,v.parameter_version,v.config_crc);if(fwrite(h,1,64,s.record)!=64)goto done;}
 if(pthread_mutex_init(&s.mutex,NULL))goto done;mutex_ready=1;atomic_init(&s.stop,0);atomic_init(&s.timing_total,0);atomic_init(&s.timing_consecutive,0);atomic_init(&s.timing_window,0);if(pthread_create(&thread,NULL,publish_loop,&s))goto done;thread_ready=1;keep_running=1;signal(SIGINT,stop_signal);signal(SIGTERM,stop_signal);struct sigaction inherited;if(!sigaction(SIGHUP,NULL,&inherited)&&inherited.sa_handler!=SIG_IGN)signal(SIGHUP,stop_signal);
 uint64_t start=mpusvc_now();result=0;fprintf(stderr,"MPU headless receiver %s:5016 peer=%s state=%s; no Wayland/camera/motor sender\n",o->bind_ip,o->peer_ip,o->state_dir);
 while(keep_running&&(!o->duration_us||mpusvc_now()-start<o->duration_us)){
  struct pollfd f[2]={{s.rawfd,POLLIN,0},{s.attfd,POLLIN,0}};int ready=poll(f,2,2);if(ready<0){if(errno==EINTR)continue;result=5;break;}if(!ready){pthread_mutex_lock(&s.mutex);lab_status flushed=mpusvc_flush_pending(s.core,mpusvc_now());pthread_mutex_unlock(&s.mutex);if(flushed==LAB_ERR_TIME){result=5;break;}continue;}
  for(unsigned channel=0;channel<2;channel++){if(!(f[channel].revents&POLLIN))continue;
  for(unsigned i=0;i<64;++i){uint8_t p[128];struct sockaddr_in from;socklen_t n=sizeof(from);char ancillary[CMSG_SPACE(sizeof(struct timespec))];struct iovec iov={.iov_base=p,.iov_len=sizeof(p)};struct msghdr msg={.msg_name=&from,.msg_namelen=n,.msg_iov=&iov,.msg_iovlen=1,.msg_control=ancillary,.msg_controllen=sizeof(ancillary)};ssize_t bytes=recvmsg(f[channel].fd,&msg,MSG_DONTWAIT|MSG_TRUNC);n=msg.msg_namelen;
   if(bytes<0){if(errno==EINTR)continue;if(errno==EAGAIN||errno==EWOULDBLOCK)break;result=5;keep_running=0;break;}
   if(n!=sizeof(from)||from.sin_family!=AF_INET||from.sin_addr.s_addr!=peer.s_addr)continue;
   unsigned sequence=bytes>=12?(unsigned)(p[8]|(uint32_t)p[9]<<8|(uint32_t)p[10]<<16|(uint32_t)p[11]<<24):0;
   uint64_t rx=0;
   if(channel==1){
    if(kernel_arrival_us(&s,&msg,&rx))continue;
    pthread_mutex_lock(&s.mutex);(void)mpusvc_push_attitude(s.core,p,(size_t)bytes,rx);(void)mpusvc_flush_pending(s.core,mpusvc_now());pthread_mutex_unlock(&s.mutex);continue;
   }
   if(kernel_arrival_us(&s,&msg,&rx)||mpusvc_guard_arrival(s.last_arrival_us,&rx)<0){
    if(timing_event(&s,1,"ARRIVAL_TIMESTAMP",rx,sequence)){result=5;keep_running=0;break;}continue;
   }
   pthread_mutex_lock(&s.mutex);
   lab_status status=mpusvc_submit_raw(s.core,p,(size_t)bytes,rx,mpusvc_now());
   if(status==LAB_ERR_TIME){pthread_mutex_unlock(&s.mutex);if(timing_event(&s,1,"CORE_TIME",rx,sequence)){result=5;keep_running=0;break;}continue;}
   s.last_arrival_us=rx;
   if((status==LAB_OK||status==LAB_AGAIN)&&timing_event(&s,0,"VALID",rx,sequence)){pthread_mutex_unlock(&s.mutex);result=5;keep_running=0;break;}
   if(s.record_enabled&&bytes==100){if(s.count==64)++s.queue_drops;else{raw_item *item=&s.queue[(s.head+s.count)%64];memcpy(item->packet,p,100);item->rx=rx;++s.count;}}
   pthread_mutex_unlock(&s.mutex);
  }
  }
  pthread_mutex_lock(&s.mutex);(void)mpusvc_flush_pending(s.core,mpusvc_now());pthread_mutex_unlock(&s.mutex);
 }
done:
 if(thread_ready){atomic_store(&s.stop,1);pthread_join(thread,NULL);}mpusvc_history_close(&s.history);if(s.record&&fclose(s.record))s.io_error=1;if(mutex_ready)pthread_mutex_destroy(&s.mutex);mpusvc_close(s.core);if(s.rawfd>=0)close(s.rawfd);if(s.attfd>=0)close(s.attfd);if(s.legacyfd>=0)close(s.legacyfd);if(s.lockfd>=0)close(s.lockfd);return s.io_error?6:result;
}
#endif
