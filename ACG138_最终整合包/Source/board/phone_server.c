/* YOLO有效显示区域、密码验证与本地设备发现。 */
#define _GNU_SOURCE
#include "phone_shared.h"
#include "voice_mission.h"
#include "voice_perception.h"
#include "odometry_runtime.h"
#include "control_core.h"
#include "video/video_service.h"
#include "phone_navigation.h"
#include "phone_control_link.h"
#include "s1_mode_state.h"
#include "phone_auto_switch.h"
#include "phone_auto_sonar.h"
#include "../rk/src/app/phone_control_fence.h"
#include <linux/filter.h>
#include <stdatomic.h>
#include "../rk/src/app/phone_goal_ipc.h"
#include "../rk/src/app/tag_goal.h"
#include "../rk/src/app/object_readonly.h"
#include "reference/robot_protocol.h"
#include "reference/dashboard_video.h"
#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include "phone_persistent_log.h"
#include "../rk/src/app/s5_persistent_log.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#include "vendor/stb_image_write.h"
static volatile sig_atomic_t running=1;
static pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t map_mutex=PTHREAD_MUTEX_INITIALIZER;
static uint64_t navigation_sequence,navigation_request_sequence;
static phone_auto_switch auto_switch;
static phone_sonar_auto fpga_auto;
static void wire_send(unsigned op,unsigned motion);
static voice_mission voice_task;static voice_receipt voice_controller;static int voice_internal;
static void voice_external_cancel_locked(const char*reason){if(!voice_internal&&voice_task.active)voice_mission_cancel(&voice_task,voice_task.id,reason);}
static uint64_t object_request_sequence,object_request_id;
static char object_request_session[33],object_directory[PATH_MAX];
static char navigation_request_session[33];
static struct phone_control control;
static struct phone_sensor_state sensor;
static struct phone_control_link direct_link;
static char control_directory[PATH_MAX];
static uint64_t last_control_log;
static int forced_stop_active;
static int have_sensor,enabled,simulate,listener=-1,raw_fd=-1,interface_index,clients;
static int discovery_fd=-1;
static unsigned http_port=8080,discovery_port=37020;
static unsigned char source_mac[6],destination_mac[6]={0,10,53,1,254,192};
static char pairing[65],shared_path[PATH_MAX],navigation_map_path[PATH_MAX],vision_tags_path[PATH_MAX],navigation_goal_path[sizeof(((struct sockaddr_un*)0)->sun_path)];
static struct phone_shared *shared;
static uint8_t *jpeg;
static size_t jpeg_size;
static unsigned jpeg_width,jpeg_height,preview_width=1280;
static uint64_t jpeg_ms;
static int claiming;
static _Atomic int h264_enabled;
static int h264_requested;
static uint64_t claim_after,claim_first_sent;
static uint64_t millis(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000;}
static uint64_t micros(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;}
static s5_log*phone_disk_log;
static phone_diag_state phone_diag;
static pthread_mutex_t phone_diag_mutex=PTHREAD_MUTEX_INITIALIZER;
static void phone_log_event(const char*operation,int status,const char*code){
 /* 此路径不访问Flash、不等待互斥锁，也不进行可变大小分配。 */
 if(pthread_mutex_trylock(&phone_diag_mutex))return;
 char json[384];const char*event=NULL;uint64_t now=millis();int n=phone_diag_prepare(&phone_diag,now,operation,status,code,json,sizeof(json),&event);
 if(n&&phone_disk_log)s5_log_submit(phone_disk_log,now*1000,event,json,(size_t)n,1);
 pthread_mutex_unlock(&phone_diag_mutex);
}
static void phone_log_close(void){
 pthread_mutex_lock(&phone_diag_mutex);s5_log*l=phone_disk_log;phone_disk_log=NULL;pthread_mutex_unlock(&phone_diag_mutex);
 if(l){const char*json="{\"operation\":\"service\",\"code\":\"PHONE_EXIT\"}";s5_log_submit(l,micros(),"PHONE_EXIT",json,strlen(json),1);s5_log_close(l);}
}
static void phone_log_open(void){
 const char*base=getenv("ACG_S5_LOG_DIR");char dir[PATH_MAX];
 if(!base||base[0]!='/'||snprintf(dir,sizeof(dir),"%s/phone",base)>=(int)sizeof(dir)){fprintf(stderr,"PHONE_LOG_UNAVAILABLE reason=NO_PRIVATE_FLASH_DIRECTORY\n");return;}
 phone_disk_log=s5_log_open(dir,getenv("ACG_S5_FS_SHA256"));
 if(!phone_disk_log){fprintf(stderr,"PHONE_LOG_UNAVAILABLE reason=WRITER_OPEN_FAILED\n");return;}
 atexit(phone_log_close);const char*json="{\"operation\":\"service\",\"code\":\"PHONE_STARTED\"}";
 s5_log_submit(phone_disk_log,micros(),"PHONE_STARTED",json,strlen(json),1);
}
static int fresh(uint64_t now){return simulate||(direct_link.robot.seen&&now>=direct_link.robot.last_ms&&now-direct_link.robot.last_ms<PHONE_LINK_MS);}
static int supported(void){return enabled&&(simulate||direct_link.robot.remote_supported);}
static int final_fault(void){return !simulate&&(direct_link.fault_mask||(direct_link.robot.feedback_supported&&direct_link.robot.feedback_phase==4));}
static int control_transport_ready(uint64_t now){return simulate||phone_link_ready(&direct_link,now);}
static int feedback_controls_ok(void){return !final_fault();}
static int session_ready(uint64_t now){return control_transport_ready(now)&&supported()&&feedback_controls_ok()&&!claiming&&(sensor.remote_flags&3)==3&&!(sensor.remote_flags&12);}
static int ready(uint64_t now){return session_ready(now);}
static int mode_read(s1_mode_state*out,uint64_t now){
 char path[PATH_MAX],data[2049];memset(out,0,sizeof(*out));if(snprintf(path,sizeof(path),"%s/mode-state.json",control_directory)>=(int)sizeof(path))return 0;
 int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);if(fd<0)return 0;struct stat st;int ok=fstat(fd,&st)==0&&S_ISREG(st.st_mode)&&st.st_size>1&&st.st_size<=2048&&(st.st_mode&0022)==0;
 ssize_t bytes=ok?read(fd,data,(size_t)st.st_size):-1;close(fd);if(bytes<2||bytes!=st.st_size)return 0;data[bytes]=0;return s1_mode_parse(out,data,now*1000);
}
static const char *drive_status(int live,uint64_t now){
 if(!live||!control_transport_ready(now))return "unknown";
 if(final_fault()||!sensor.actual)return "stopped";
 if(sensor.feedback_supported&&(sensor.feedback_phase==1||sensor.feedback_phase==2||sensor.feedback_phase==3))return "slow";
 return "normal";
}
static void stop_signal(int n){(void)n;running=0;if(listener>=0)shutdown(listener,SHUT_RDWR);}
static int send_all(int fd,const void *data,size_t bytes){const char*p=data;while(bytes){ssize_t n=send(fd,p,bytes,MSG_NOSIGNAL);if(n<0&&errno==EINTR)continue;if(n<=0)return -1;p+=n;bytes-=(size_t)n;}return 0;}
static void reply(int fd,int status,const char *type,const void *data,size_t bytes,const char *extra){char head[768];int n=snprintf(head,sizeof(head),"HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-store\r\nConnection: close\r\nX-Content-Type-Options: nosniff\r\n%s\r\n",status,status==200?"OK":status==204?"No Content":status==401?"Unauthorized":status==409?"Conflict":"Error",type,bytes,extra?extra:"");if(n>0&&(size_t)n<sizeof(head)&&!send_all(fd,head,(size_t)n)&&bytes)send_all(fd,data,bytes);}
static void error_reply(int fd,int status,const char *message){char body[256];int n=snprintf(body,sizeof(body),"{\"error\":\"%s\"}",message);reply(fd,status,"application/json; charset=utf-8",body,(size_t)n,NULL);}
/* 日志仅使用静态诊断文字，不记录会话令牌、密码或任意输入。 */
static void nav_error_reply(int fd,int status,const char*operation,const char*code,const char*message){
 phone_log_event(operation,status,code);
 char body[512];int n=snprintf(body,sizeof(body),"{\"error\":\"%s\",\"code\":\"%s\",\"operation\":\"%s\"}",message,code,operation);
 static pthread_mutex_t log_mutex=PTHREAD_MUTEX_INITIALIZER;static uint64_t last_log_ms,suppressed;
 uint64_t now=millis();pthread_mutex_lock(&log_mutex);
 if(!last_log_ms||now-last_log_ms>=1000){fprintf(stderr,"%s time_ms=%llu operation=%s status=%d code=%s suppressed=%llu\n",!strcmp(operation,"map")?"MAP_HTTP_ERROR":"NAV_HTTP_ERROR",(unsigned long long)now,operation,status,code,(unsigned long long)suppressed);last_log_ms=now;suppressed=0;}else suppressed++;
 pthread_mutex_unlock(&log_mutex);
 if(n<0||(size_t)n>=sizeof(body)){error_reply(fd,500,"诊断信息过长");return;}
 reply(fd,status,"application/json; charset=utf-8",body,(size_t)n,NULL);
}
static int equal_secret(const char *s){size_t a=strlen(pairing),b=strlen(s);unsigned diff=(unsigned)(a^b);for(size_t i=0;i<a;i++)diff|=(unsigned char)pairing[i]^(i<b?(unsigned char)s[i]:0);return diff==0;}
static int entropy(void*p,size_t n){size_t have=0;while(have<n){ssize_t got=getrandom((char*)p+have,n-have,0);if(got<0&&errno==EINTR)continue;if(got<=0)return -1;have+=(size_t)got;}return 0;}
static int new_session(char s[33],uint32_t *token){uint8_t bytes[16];if(entropy(bytes,sizeof(bytes))||entropy(token,sizeof(*token)))return -1;if(!*token)*token=1;for(unsigned i=0;i<16;i++)sprintf(s+2*i,"%02x",bytes[i]);return 0;}
/* 设备发现接口仅返回公开设备信息。 */
static void *discovery_thread(void *unused){
    (void)unused;
    while(running){
        char request[128],response[128];struct sockaddr_in peer;socklen_t size=sizeof(peer);
        ssize_t count=recvfrom(discovery_fd,request,sizeof(request)-1,0,(struct sockaddr*)&peer,&size);
        if(count<0){if(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR)continue;break;}
        static const char prefix[]="ACG138_DISCOVER_V1 ";
        size_t head=sizeof(prefix)-1;
        if(count!=(ssize_t)(head+32)||memcmp(request,prefix,head))continue;
        int valid=1;for(unsigned i=0;i<32;i++)if(!isxdigit((unsigned char)request[head+i]))valid=0;
        if(!valid)continue;
        int n=snprintf(response,sizeof(response),"ACG138_ROVER_V1 %.*s %u",32,request+head,http_port);
        if(n>0&&(size_t)n<sizeof(response))sendto(discovery_fd,response,(size_t)n,0,(struct sockaddr*)&peer,size);
    }
    return NULL;
}
#include "phone_json.h"
/* 调用方持有互斥锁，只有通过认证且仍有效的手机会话能到达此处。服务端统一序号为目标、心跳及手动取消排序。
 */
/* 严格实物请求格式，不接受坐标、名称、空值或扩展键。 */
static int phone_object_navigation_json(const char*p,phone_goal_message*m,uint64_t*seq){
 unsigned seen=0;space(&p);if(*p++!='{')return -1;
 for(unsigned i=0;i<7;i++){
  char key[24],value[96],*end;unsigned bit;if(string(&p,key,sizeof(key)))return -1;space(&p);if(*p++!=':')return -1;space(&p);
  if(!strcmp(key,"map_id")||!strcmp(key,"map_version")||!strcmp(key,"epoch")){
   bit=!strcmp(key,"map_id")?1:!strcmp(key,"map_version")?2:4;if(string(&p,value,sizeof(value))||!value[0])return -1;
   for(size_t j=0;value[j];j++)if(!isdigit((unsigned char)value[j]))return -1;
   errno=0;uint64_t n=strtoull(value,&end,10);if(errno||*end||!n)return -1;if(bit==1)m->map_id=n;else if(bit==2)m->map_version=n;else m->object_epoch=n;
  }else if(!strcmp(key,"track_id")||!strcmp(key,"seq")){
   bit=!strcmp(key,"track_id")?8:16;if(!isdigit((unsigned char)*p)||(*p=='0'&&isdigit((unsigned char)p[1])))return -1;errno=0;uint64_t n=strtoull(p,&end,10);
   if(errno||end==p||(bit==8?n>INT32_MAX:(!n||n>UINT64_C(9007199254740991))))return -1;p=end;if(bit==8)m->object_track_id=(int32_t)n;else *seq=n;
  }else if(!strcmp(key,"association_id")){
   bit=32;if(string(&p,m->association_id,sizeof(m->association_id))||!m->association_id[0])return -1;
   for(size_t j=0;m->association_id[j];j++){unsigned char c=(unsigned char)m->association_id[j];if(!isalnum(c)&&c!='-'&&c!='_'&&c!='.'&&c!=':')return -1;}
  }else if(!strcmp(key,"standoff_m")){
   bit=64;if(!isdigit((unsigned char)*p))return -1;
   const char*q=p;if(*q=='0')q++;else while(isdigit((unsigned char)*q))q++;
   if(*q=='.'){q++;if(!isdigit((unsigned char)*q))return -1;while(isdigit((unsigned char)*q))q++;}
   if(*q=='e'||*q=='E'){q++;if(*q=='+'||*q=='-')q++;if(!isdigit((unsigned char)*q))return -1;while(isdigit((unsigned char)*q))q++;}
   if(*q&&*q!=','&&*q!='}'&&!isspace((unsigned char)*q))return -1;
   errno=0;double n=strtod(p,&end);if(errno||end==p||!isfinite(n)||n<.2||n>3)return -1;p=end;m->x_m=n;
  }else return -1;
  if(seen&bit)return -1;seen|=bit;space(&p);if(i<6&&*p++!=',')return -1;
 }
 if(*p++!='}')return -1;space(&p);return *p||seen!=127?-1:0;
}

static int navigation_send_locked(phone_goal_message*m,const char*session,unsigned operation){
 if(fpga_auto.failed&&operation!=PHONE_NAV_HEARTBEAT)phone_sonar_auto_cancel(&fpga_auto);
 if(fpga_auto.active&&operation!=PHONE_NAV_HEARTBEAT){wire_send(PHONE_STOP,0);control.autonomous=0;}
 if((operation<=PHONE_NAV_AUTO_START&&operation!=PHONE_NAV_HEARTBEAT)||operation==PHONE_NAV_OBJECT_CANCEL)voice_external_cancel_locked("明确停车、手动接管、取消或控制会话变化");
 struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return -1;
 m->magic=PHONE_GOAL_MAGIC;m->version=(operation>=PHONE_NAV_OBJECT_PREPARE&&operation<=PHONE_NAV_OBJECT_RESUME)?7:operation==PHONE_NAV_AUTO_START?6:operation==PHONE_NAV_OBJECT_GOAL?4:operation==PHONE_NAV_TAG_GOAL?3:2;m->operation=operation;if(!m->sent_us)m->sent_us=(uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;
 if(operation==PHONE_NAV_HEARTBEAT)m->sent_us=control.last_rx_ms*1000;
  m->sequence=++navigation_sequence;memcpy(m->session,session,33);
 int sock=socket(AF_UNIX,SOCK_DGRAM|SOCK_CLOEXEC,0);struct sockaddr_un addr={.sun_family=AF_UNIX};if(sock<0)return -1;
 memcpy(addr.sun_path,navigation_goal_path,strlen(navigation_goal_path)+1);
 ssize_t sent=sendto(sock,m,sizeof(*m),MSG_DONTWAIT,(struct sockaddr*)&addr,sizeof(addr));close(sock);return sent==(ssize_t)sizeof(*m)?0:-1;
}
static void wire_send(unsigned op,unsigned motion){
    if(op==PHONE_AUTO&&!fpga_auto.active){fputs("AUTO_REJECTED reason=NO_AUTHENTICATED_AUTO_INTENT\n",stderr);return;}
    if((fpga_auto.active||fpga_auto.failed)&&(op==PHONE_STOP||op==PHONE_MANUAL||op==PHONE_HOLD)){
     phone_sonar_auto_cancel(&fpga_auto);control.autonomous=0;
     if(op==PHONE_HOLD)wire_send(PHONE_STOP,0);
    }
    if(raw_fd<0)return;
    uint8_t frame[74]={0},payload[32];phone_wire_packet(payload,op,motion,control.wire_token,++control.wire_sequence);
    memcpy(frame,destination_mac,6);memcpy(frame+6,source_mac,6);frame[12]=8;
    uint8_t *ip=frame+14;ip[0]=0x45;ip[3]=60;ip[6]=0x40;ip[8]=64;ip[9]=17;
    ip[12]=192;ip[13]=168;ip[15]=3;ip[16]=192;ip[17]=168;ip[19]=2;
    unsigned sum=0;for(unsigned i=0;i<20;i+=2)sum+=((unsigned)ip[i]<<8)|ip[i+1];while(sum>>16)sum=(sum&65535)+(sum>>16);sum=(~sum)&65535;ip[10]=(uint8_t)(sum>>8);ip[11]=(uint8_t)sum;
    uint8_t *udp=ip+20;udp[0]=udp[2]=0x13;udp[1]=udp[3]=0x91;udp[5]=40;memcpy(udp+8,payload,32);
    struct sockaddr_ll address={.sll_family=AF_PACKET,.sll_protocol=htons(ETH_P_IP),.sll_ifindex=interface_index,.sll_halen=6};memcpy(address.sll_addr,destination_mac,6);
    if(sendto(raw_fd,frame,sizeof(frame),0,(struct sockaddr*)&address,sizeof(address))!=(ssize_t)sizeof(frame)){control.motion=0;control.queued_op=PHONE_STOP;control.pending=1;}
}
static int raw_open(const char*name){
    if(!name||strlen(name)>=IFNAMSIZ){fprintf(stderr,"--interface requires the FPGA internal Ethernet interface\n");return -1;}
    int fd=socket(AF_PACKET,SOCK_RAW|SOCK_CLOEXEC|SOCK_NONBLOCK,htons(ETH_P_IP));if(fd<0){perror("raw socket (root required)");return -1;}
    struct ifreq req={0};strcpy(req.ifr_name,name);
    if(ioctl(fd,SIOCGIFINDEX,&req)){close(fd);return -1;}interface_index=req.ifr_ifindex;
    if(ioctl(fd,SIOCGIFHWADDR,&req)){close(fd);return -1;}memcpy(source_mac,req.ifr_hwaddr.sa_data,6);
    struct sockaddr_ll link={.sll_family=AF_PACKET,.sll_protocol=htons(ETH_P_IP),.sll_ifindex=interface_index};if(bind(fd,(struct sockaddr*)&link,sizeof(link))){close(fd);return -1;}
    struct sock_filter code[]={BPF_STMT(BPF_LD|BPF_H|BPF_ABS,12),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,ETH_P_IP,0,6),BPF_STMT(BPF_LD|BPF_B|BPF_ABS,23),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,17,0,4),BPF_STMT(BPF_LD|BPF_H|BPF_ABS,34),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,5008,1,0),BPF_JUMP(BPF_JMP|BPF_JEQ|BPF_K,5013,0,1),BPF_STMT(BPF_RET|BPF_K,256),BPF_STMT(BPF_RET|BPF_K,0)};
    struct sock_fprog filter={.len=(unsigned short)(sizeof(code)/sizeof(code[0])),.filter=code};int one=1;
    if(setsockopt(fd,SOL_SOCKET,SO_ATTACH_FILTER,&filter,sizeof(filter))||setsockopt(fd,SOL_SOCKET,SO_TIMESTAMPNS,&one,sizeof(one))){close(fd);return -1;}raw_fd=fd;return 0;
}
static void read_control_link_locked(uint64_t now){
 if(raw_fd<0)return;for(unsigned i=0;i<64;i++){
  uint8_t frame[256];char ancillary[CMSG_SPACE(sizeof(struct timespec))];struct iovec io={frame,sizeof(frame)};struct msghdr msg={.msg_iov=&io,.msg_iovlen=1,.msg_control=ancillary,.msg_controllen=sizeof(ancillary)};
  ssize_t n=recvmsg(raw_fd,&msg,MSG_DONTWAIT);if(n<0)break;uint64_t stamp=0;
  for(struct cmsghdr*c=CMSG_FIRSTHDR(&msg);c;c=CMSG_NXTHDR(&msg,c))if(c->cmsg_level==SOL_SOCKET&&c->cmsg_type==SCM_TIMESTAMPNS&&c->cmsg_len>=CMSG_LEN(sizeof(struct timespec))){
   struct timespec received,real;memcpy(&received,CMSG_DATA(c),sizeof(received));if(clock_gettime(CLOCK_REALTIME,&real))continue;
   int64_t age=((int64_t)real.tv_sec-received.tv_sec)*1000+(real.tv_nsec-received.tv_nsec)/1000000;
   if(age>=0&&(uint64_t)age<=now)stamp=now-(uint64_t)age;
  }
  if(stamp&&now-stamp<PHONE_LINK_MS&&!(msg.msg_flags&MSG_TRUNC))phone_link_push(&direct_link,frame,(size_t)n,source_mac,stamp);
 }
 const struct robot_model*m=&direct_link.robot;if(!m->seen)return;
 sensor.robot_seen=1;sensor.robot_bad=0;sensor.robot_ms=m->last_ms;memcpy(sensor.mm,m->mm,sizeof(sensor.mm));memcpy(sensor.error,m->error,sizeof(sensor.error));
 sensor.range_valid=m->valid;sensor.auto_mode=m->auto_mode;sensor.actual=m->actual;sensor.feedback=m->feedback;sensor.stop_reason=m->stop_reason;sensor.remote_supported=m->remote_supported;
 sensor.remote_flags=m->remote_flags;sensor.remote_sequence=m->remote_sequence;sensor.nav_supported=m->nav_supported;sensor.nav_state=m->state;sensor.near_mask=m->near_mask;sensor.feedback_supported=m->feedback_supported;sensor.feedback_phase=m->feedback_phase;sensor.feedback_retries=m->feedback_retries;
 have_sensor=1;
}
static int map_shared_unlocked(void){
    if(shared)return 0;int fd=open(shared_path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);if(fd<0)return -1;struct stat st;
    if(fstat(fd,&st)||st.st_size!=(off_t)sizeof(struct phone_shared)){close(fd);return -1;}
    shared=mmap(NULL,sizeof(*shared),PROT_READ,MAP_SHARED,fd,0);close(fd);if(shared==MAP_FAILED){shared=NULL;return -1;}return 0;
}
static int map_shared(void){pthread_mutex_lock(&map_mutex);int result=map_shared_unlocked();pthread_mutex_unlock(&map_mutex);return result;}
static int read_sensor(struct phone_sensor_state*out){
    if(map_shared()||phone_read_seq(&shared->magic)!=PHONE_MAGIC||shared->version!=1)return -1;
    for(unsigned attempt=0;attempt<3;attempt++){uint32_t seq=phone_read_seq(&shared->state_sequence);if(seq&1)continue;memcpy(out,&shared->state,sizeof(*out));__atomic_thread_fence(__ATOMIC_ACQUIRE);if(seq==phone_read_seq(&shared->state_sequence)&&out->stamp_ms)return 0;}return -1;
}
static void dispatch_control_locked(uint64_t now){
        unsigned op,motion,previous=control.motion;
        static int logged_mode=-1;static uint64_t lease_log;
        if(logged_mode!=control.autonomous||!lease_log||now<lease_log||now-lease_log>=1000){
         fprintf(stderr,"CONTROL_LEASE mode=%s lease_ms=%llu rx_age_ms=%llu hold_remaining_ms=%llu watchdog_ms=500 config=31110301\n",control.autonomous?"AUTONOMOUS":"MANUAL",(unsigned long long)phone_control_lease_ms(&control),(unsigned long long)(now>=control.last_rx_ms?now-control.last_rx_ms:UINT64_MAX),(unsigned long long)(control.hold_until_ms>now?control.hold_until_ms-now:0));logged_mode=control.autonomous;lease_log=now;
        }
        int transport=control_transport_ready(now)&&supported();
        int force=control.has_session&&(!transport||!phone_control_live(&control,control.session,now));
        phone_control_tick(&control,transport,now,&op,&motion);
        if((force&&!forced_stop_active)||(previous&&op==PHONE_STOP)){
         uint64_t revoke=0;(void)phone_control_fence_revoke(control_directory,&revoke);phone_goal_message msg={0};(void)navigation_send_locked(&msg,control.session,PHONE_NAV_OVERRIDE);
         fprintf(stderr,"CONTROL_STOP_DIAG source=%s now_ms=%llu last_rx_ms=%llu hold_until_ms=%llu generation=%llu\n",final_fault()?"FINAL_MOTOR_FAULT":!transport?"CONTROL_LINK_LOST":force?"UPSTREAM_SESSION_EXPIRED":"HOLD_INPUT_EXPIRED",(unsigned long long)now,(unsigned long long)control.last_rx_ms,(unsigned long long)control.hold_until_ms,(unsigned long long)revoke);
        }
        if(force||final_fault())phone_log_event("control-link",503,final_fault()?"FINAL_MOTOR_FAULT":!transport?"CONTROL_LINK_LOST":"UPSTREAM_SESSION_EXPIRED");
        else phone_log_event("control-link",200,"CONTROL_LINK_RECOVERED");
        forced_stop_active=force;
        if(claiming&&phone_control_live(&control,control.session,now)&&fresh(now)&&supported()){
            if(now<claim_after){op=PHONE_STOP;motion=0;}
            else{op=PHONE_MANUAL;motion=0;if(!claim_first_sent)claim_first_sent=now;
                if(sensor.robot_ms>=claim_first_sent&&(sensor.remote_flags&3)==3&&!(sensor.remote_flags&12)&&
                   (uint16_t)(control.wire_sequence-sensor.remote_sequence)<32){claiming=0;control.pending=0;}}
        }
        if(fpga_auto.active&&op==PHONE_PING){
         int next=phone_sonar_auto_tick(&fpga_auto,control.session,phone_control_live(&control,control.session,now),fresh(now),sensor.auto_mode,sensor.robot_ms,now);
         if(next<0){
          phone_sonar_auto failed=fpga_auto;
          wire_send(PHONE_STOP,0);
          failed.active=0;failed.failed=1;fpga_auto=failed;
          fprintf(stderr,"AUTO_STOP reason=%s request_us=%llu now_ms=%llu\n",failed.acknowledged?"FPGA_AUTO_EXITED":"FPGA_AUTO_START_NOT_ACKNOWLEDGED",(unsigned long long)failed.request_us,(unsigned long long)now);
         }
         else if(next>0){wire_send(PHONE_AUTO,0);fprintf(stderr,"AUTO_START_RETRY attempt=%u request_us=%llu\n",fpga_auto.attempts,(unsigned long long)fpga_auto.request_us);}
        }
        wire_send(op,motion);
        if(simulate){sensor.auto_mode=fpga_auto.active;sensor.actual=op==PHONE_HOLD?motion:0;sensor.remote_flags=control.has_session?3:0;}
}
static void *watchdog_thread(void*unused){
    (void)unused;while(running){uint64_t now=millis();struct phone_sensor_state update;
        int got=read_sensor(&update)==0;pthread_mutex_lock(&mutex);
        if(got){sensor=update;have_sensor=1;}
        read_control_link_locked(now);
        if(simulate){sensor.stamp_ms=sensor.robot_ms=sensor.imu_ms=now;sensor.robot_seen=1;sensor.imu_state=1;sensor.roll=2.4;sensor.pitch=-1.8;sensor.yaw=128.6;sensor.mm[0]=1380;sensor.mm[1]=820;sensor.mm[2]=2140;sensor.range_valid=7;sensor.remote_supported=1;sensor.nav_supported=1;sensor.stop_reason=11;have_sensor=1;}
        dispatch_control_locked(now);
        pthread_mutex_unlock(&mutex);struct timespec pause={0,80000000};nanosleep(&pause,NULL);
    }
    pthread_mutex_lock(&mutex);for(unsigned i=0;i<3;i++)wire_send(PHONE_STOP,0);pthread_mutex_unlock(&mutex);return NULL;
}
static int h264_source(unsigned char *dst,size_t capacity,uint32_t previous,struct video_source *meta){
    if(map_shared()||phone_read_seq(&shared->magic)!=PHONE_MAGIC||shared->version!=1)return 0;
    for(unsigned attempt=0;attempt<3;attempt++){
        uint32_t sequence=phone_read_seq(&shared->frame_sequence);
        if(sequence&1)continue;if(sequence==previous)return 0;
        struct phone_frame_meta f=shared->frame;
        if(!f.stamp_ms)return 0;
        if(f.fourcc!=DASHBOARD_NV12||f.width!=1920||f.height!=1080||f.stride!=1920||f.bytes!=1920*1080*3/2||f.bytes>capacity)return -1;
        memcpy(dst,shared->pixels,f.bytes);__atomic_thread_fence(__ATOMIC_ACQUIRE);
        if(sequence!=phone_read_seq(&shared->frame_sequence))continue;
        *meta=(struct video_source){sequence,f.stamp_ms,f.width,f.height,f.stride};return 1;
    }
    return 0;
}
/* 会话所有权和租约时钟独立于感知数据源年龄。 */
static int h264_session_live(const char *session){
    pthread_mutex_lock(&mutex);int valid=running&&phone_control_live(&control,session,millis());pthread_mutex_unlock(&mutex);return valid;
}
static void session_touch_diag_locked(int control_mode,uint64_t now){
    static uint64_t last[2];unsigned k=control_mode?1u:0u;
    if(!last[k]||now<last[k]||now-last[k]>=1000){
        fprintf(stderr,"SESSION_TOUCH kind=%s now_ms=%llu control_live=%d view_live=%d policy_revision=5\n",control_mode?"control":"view",(unsigned long long)now,phone_control_live(&control,control.session,now),phone_view_live(&control,control.session,now));last[k]=now;
    }
}
static int view_session_live(const char*session){
    pthread_mutex_lock(&mutex);uint64_t now=millis();int status=phone_session_touch(&control,session,now,0);
    if(status==200)session_touch_diag_locked(0,now);pthread_mutex_unlock(&mutex);return status==200;
}
/* 心跳维持会话；只读查看不续租控制权。 */
static void heartbeat_reply(int fd,const char*json,const char*session){
    int mode=0;if(phone_heartbeat_json(json,&mode)){nav_error_reply(fd,400,"heartbeat","HEARTBEAT_MODE_INVALID","心跳模式无效，仅允许control或view");return;}
    pthread_mutex_lock(&mutex);uint64_t now=millis();int status=phone_session_touch(&control,session,now,mode);
    int control_live=phone_control_live(&control,session,now);uint64_t control_expires=control_live?control.last_rx_ms+phone_control_lease_ms(&control):0,view_expires=control.view_last_ms+PHONE_VIEW_MS;
    if(status==200){session_touch_diag_locked(mode,now);if(mode){phone_goal_message msg={0};(void)navigation_send_locked(&msg,session,PHONE_NAV_HEARTBEAT);}}
    pthread_mutex_unlock(&mutex);
    if(status!=200){nav_error_reply(fd,401,"heartbeat","SESSION_EXPIRED","控制或查看会话已过期，请重新连接；不会自动恢复运动");return;}
    phone_log_event("heartbeat",200,"HEARTBEAT_OK");
    char body[256];int n=snprintf(body,sizeof(body),"{\"now_ms\":%llu,\"control_expires_ms\":%llu,\"view_expires_ms\":%llu,\"control_live\":%s,\"mode\":\"%s\",\"policy_revision\":5}",(unsigned long long)now,(unsigned long long)control_expires,(unsigned long long)view_expires,control_live?"true":"false",mode?"control":"view");reply(fd,200,"application/json",body,(size_t)n,NULL);
}
struct jpeg_build{uint8_t*data;size_t size,capacity;int failed;};
static void jpeg_chunk(void*context,void*data,int bytes){struct jpeg_build*j=context;if(j->failed||bytes<0)return;size_t need=j->size+(size_t)bytes;if(need>4u*1024u*1024u){j->failed=1;return;}if(need>j->capacity){size_t cap=need*2;uint8_t*p=realloc(j->data,cap);if(!p){j->failed=1;return;}j->data=p;j->capacity=cap;}memcpy(j->data+j->size,data,(size_t)bytes);j->size=need;}
static void *video_thread(void*unused){
    (void)unused;uint8_t*raw=malloc(PHONE_MAX_FRAME);uint32_t*rgb=malloc((size_t)preview_width*(preview_width*9/16)*4);uint8_t*packed=malloc((size_t)preview_width*(preview_width*9/16)*3);uint64_t last=0;
    const char*fit=getenv("ACG_PHONE_VIDEO_FIT");int yolo640=fit&&!strcmp(fit,"yolo640");
    if(!raw||!rgb||!packed){free(raw);free(rgb);free(packed);return NULL;}
    while(running){struct timespec pause={0,40000000};nanosleep(&pause,NULL);if(map_shared()||phone_read_seq(&shared->magic)!=PHONE_MAGIC)continue;
        struct phone_frame_meta f;uint32_t sequence=phone_read_seq(&shared->frame_sequence);if(sequence&1)continue;f=shared->frame;
        if(!f.stamp_ms||f.stamp_ms==last||millis()<f.stamp_ms||millis()-f.stamp_ms>1200||f.bytes>PHONE_MAX_FRAME||!f.bytes)continue;
        memcpy(raw,shared->pixels,f.bytes);__atomic_thread_fence(__ATOMIC_ACQUIRE);if(sequence!=phone_read_seq(&shared->frame_sequence))continue;
        unsigned width=preview_width,height=preview_width*9/16;
        struct dashboard_video_layout layout={f.fourcc,f.width,f.height,f.stride,(enum dashboard_matrix)f.matrix,(enum dashboard_range)f.range};
        if(dashboard_video_preview(&layout,raw,f.bytes,rgb,(size_t)width*height,width,height,yolo640))continue;
        for(size_t i=0;i<(size_t)width*height;i++){packed[i*3]=(uint8_t)(rgb[i]>>16);packed[i*3+1]=(uint8_t)(rgb[i]>>8);packed[i*3+2]=(uint8_t)rgb[i];}
        struct jpeg_build build={0};int ok=stbi_write_jpg_to_func(jpeg_chunk,&build,(int)width,(int)height,3,packed,72);
        if(ok&&!build.failed){pthread_mutex_lock(&mutex);free(jpeg);jpeg=build.data;jpeg_size=build.size;jpeg_width=width;jpeg_height=height;jpeg_ms=f.stamp_ms;pthread_mutex_unlock(&mutex);}else free(build.data);
        last=f.stamp_ms;
    }free(raw);free(rgb);free(packed);return NULL;
}
static void *video_boot(void*library){
 if(h264_requested){if(video_service_start((const char*)library,h264_source)==0){h264_enabled=1;return NULL;}
  fputs("PHONE_VIDEO_DEGRADED MPP unavailable; manual API independent\n",stderr);}
 return video_thread(NULL);
}

/* 双眼图像可能达到3MiB，仅解析末尾不超过64KiB的关联数据；文件由独立融合模块原子发布。
 */
static inline int object_snapshot_eye(const char*path){
 if(!strcmp(path,"/api/objects/snapshot"))return -1;
 if(!strcmp(path,"/api/objects/snapshot?eye=0"))return 0;
 if(!strcmp(path,"/api/objects/snapshot?eye=1"))return 1;
 return !strncmp(path,"/api/objects/snapshot?",22)?-3:-2;
}
static inline const char*object_snapshot_file(int eye){return eye==0?"objects-snapshot-eye0.json":eye==1?"objects-snapshot-eye1.json":"objects-snapshot.json";}
static const char*object_identity_admission(const obj_frame*f,uint64_t epoch,int track,uint64_t now,int prepare){
 if(!f)return "OBJECT_STATE_UNAVAILABLE";
 if(f->epoch!=epoch)return "OBJECT_EPOCH_MISMATCH";
 (void)prepare; /* 双击请求可能先于异步选择确认到达。 */
 uint64_t stamp=prepare?f->generated_us:f->frame_us;
 if(!stamp)return prepare?"OBJECT_PUBLICATION_TIME_MISSING":"OBJECT_FRAME_TIME_MISSING";
 if(stamp>now)return prepare?"OBJECT_PUBLICATION_TIME_IN_FUTURE":"OBJECT_FRAME_TIME_IN_FUTURE";
 if(!prepare&&now-stamp>UINT64_C(2000000))return "OBJECT_CURRENT_FRAME_STALE";
 if(f->count<0||f->count>OBJ_MAX)return "OBJECT_STATE_COUNT_INVALID";
 for(int i=0;i<f->count;i++)if(f->objects[i].track_id==track)return "NONE";
 return prepare?"NONE":"OBJECT_TRACK_NOT_IN_CURRENT_STATE";
}
static void object_action_log(const char*action,const char*stage,int status,const char*reason,uint64_t epoch,int track,uint64_t seq,uint64_t request,uint64_t now,uint64_t stamp){
 fprintf(stderr,"OBJECT_ACTION action=%s stage=%s status=%d reason=%s epoch=%llu track_id=%d seq=%llu request_id=%llu age_known=%d age_ms=%llu\n",action,stage,status,reason,(unsigned long long)epoch,track,(unsigned long long)seq,(unsigned long long)request,stamp&&stamp<=now,(unsigned long long)(stamp&&stamp<=now?(now-stamp)/1000:0));
}
static void object_association_reply(int fd,int snapshot){
 char path[PATH_MAX];if(snprintf(path,sizeof(path),"%s/%s",object_directory,snapshot?"objects-association-snapshot.json":"objects-association.json")>=(int)sizeof(path)){error_reply(fd,503,"OBJECT_ASSOCIATION_PATH_INVALID");return;}
 char*data;size_t n;if(!obj_file_read(path,snapshot?3u*1024u*1024u:65536u,&data,&n)){error_reply(fd,503,"OBJECT_ASSOCIATION_SNAPSHOT_UNAVAILABLE");return;}
 while(n&&isspace((unsigned char)data[n-1]))n--;uint64_t stamp=0,frame=0;int valid=0;
 if(!snapshot){valid=obj_json_time(data,n,"published_us",&stamp);obj_json_time(data,n,"frame_time_us",&frame);}
 else{
  const char*begin=NULL,*search=data;while((search=strstr(search,"\"association\":"))){begin=search+14;search++;}
  if(begin){while(isspace((unsigned char)*begin))begin++;size_t len=n-(size_t)(begin-data);if(len>1&&len<=65537&&data[n-1]=='}'){
    valid=obj_json_time(begin,len-1,"published_us",&stamp);obj_json_time(begin,len-1,"frame_time_us",&frame);
    /* 独立核对顶层原始帧标识，仅剔除较大的base64图像字段，以便现有有界JSON解析器处理。
 */
    const char*image=strstr(data,"\"image_data_url\":");if(valid&&image){
     const char*value=image+17;while(isspace((unsigned char)*value))value++;
     if(*value=='"'){const char*end=value+1;while((size_t)(end-data)<n&&*end!='"'){if(*end=='\\'&&(size_t)(end-data)+1<n)end++;end++;}
      size_t prefix=(size_t)(value-data),suffix=(size_t)(end-data);if(suffix<n&&n-(suffix-prefix)+1<=256u*1024u){size_t mn=n-(suffix-prefix)+1;char*meta=malloc(mn+1);if(meta){memcpy(meta,data,prefix);meta[prefix]='"';meta[prefix+1]='"';memcpy(meta+prefix+2,end+1,n-suffix-1);meta[mn]=0;
       const char*keys[]={"epoch","frame_id","frame_time_us"};for(unsigned k=0;k<3;k++){uint64_t original=0,paired=0;if(!obj_json_time(meta,mn,keys[k],&original)||!obj_json_time(begin,len-1,keys[k],&paired)||original!=paired)valid=0;}free(meta);
      }else valid=0;}else valid=0;
     }else valid=0;
    }else valid=0;
   }}
 }
 uint64_t now=millis()*1000;
 if(!valid||!stamp||stamp>now||now-stamp>2000000){free(data);error_reply(fd,503,valid?"OBJECT_ASSOCIATION_PUBLICATION_STALE":"OBJECT_ASSOCIATION_INVALID_OR_NO_EXACT_PAIR");return;}
 uint64_t age=frame&&frame<=now?(now-frame)/1000:UINT64_MAX;char suffix[128];int extra=snprintf(suffix,sizeof(suffix),",\"stale\":%s,\"age_ms\":%llu}",(!frame||frame>now||now-frame>2000000)?"true":"false",(unsigned long long)age);
 size_t limit=snapshot?3u*1024u*1024u:65536u;if(n<2||data[n-1]!='}'||extra<0||(size_t)extra>=sizeof(suffix)||n-1+(size_t)extra>limit){free(data);error_reply(fd,503,"OBJECT_ASSOCIATION_RESPONSE_LIMIT");return;}
 char*body=realloc(data,n+(size_t)extra);if(!body){free(data);error_reply(fd,503,"OBJECT_ASSOCIATION_MEMORY_UNAVAILABLE");return;}memcpy(body+n-1,suffix,(size_t)extra+1);reply(fd,200,"application/json; charset=utf-8",body,n-1+(size_t)extra,NULL);free(body);
}
/* 实物路由只读取感知文件或原子提交选择请求，不调用wire_send或navigation_send_locked。
 */
static void object_get_reply(int fd,const char*kind,int eye){
 const char*file=!strcmp(kind,"state")?"objects-state.json":!strcmp(kind,"snapshot")?object_snapshot_file(eye):!strcmp(kind,"task")?"objects-task.json":!strcmp(kind,"scene")?"objects-scene.json":"objects-map.json";
 size_t limit=!strcmp(kind,"snapshot")?OBJ_SNAPSHOT_MAX:!strcmp(kind,"task")?16384u:!strcmp(kind,"scene")?262144u:OBJ_STATE_MAX;char path[PATH_MAX];
 if(snprintf(path,sizeof(path),"%s/%s",object_directory,file)>=(int)sizeof(path)){error_reply(fd,503,"实物路径过长");return;}
 char*data;size_t n;if(!obj_file_read(path,limit,&data,&n)){error_reply(fd,503,"等待实物感知服务");return;}
 if(!strcmp(kind,"snapshot")){obj_frame shot,current;
  if(!obj_snapshot_parse(data,n,&shot)){fprintf(stderr,"OBJECT_SNAPSHOT_REJECT reason=INVALID_SNAPSHOT_JSON bytes=%zu limit=%u eye=%d\n",n,OBJ_SNAPSHOT_MAX,eye);free(data);error_reply(fd,503,"抓拍元数据格式错误；请查看RK日志OBJECT_SNAPSHOT_REJECT");return;}
  if(!obj_snapshot_matches(data,n,&shot)){fprintf(stderr,"OBJECT_SNAPSHOT_REJECT reason=IMAGE_FRAME_IDENTITY_MISMATCH bytes=%zu frame=%llu eye=%d\n",n,(unsigned long long)shot.frame_id,shot.eye);free(data);error_reply(fd,503,"抓拍图像与帧号或检测框不一致；正在等待下一帧");return;}
  if(eye>=0&&shot.eye!=eye){fprintf(stderr,"OBJECT_SNAPSHOT_REJECT reason=REQUESTED_EYE_MISMATCH requested=%d actual=%d frame=%llu\n",eye,shot.eye,(unsigned long long)shot.frame_id);free(data);error_reply(fd,503,"请求镜头与抓拍镜头不一致；正在等待下一帧");return;}
  char state_path[PATH_MAX];if(snprintf(state_path,sizeof(state_path),"%s/objects-state.json",object_directory)<(int)sizeof(state_path)&&obj_frame_load(state_path,&current)&&current.epoch==shot.epoch&&current.frame_id==shot.frame_id&&current.selected==shot.selected&&!obj_snapshot_matches(data,n,&current)){free(data);error_reply(fd,503,"同帧左右眼检测集合不一致；正在等待原子发布完成");return;}
 }
 uint64_t stamp;const char*key=(!strcmp(kind,"map")||!strcmp(kind,"scene")||!strcmp(kind,"task"))?"published_us":"frame_time_us";if(!obj_json_time(data,n,key,&stamp)){free(data);error_reply(fd,503,"实物数据格式错误");return;}
 while(n&&isspace((unsigned char)data[n-1]))n--;uint64_t now=millis()*1000,age=now>=stamp?(now-stamp)/1000:UINT64_MAX;int stale=!stamp||stamp>now||age>((!strcmp(kind,"map")||!strcmp(kind,"scene"))?PHONE_MAP_MS:(!strcmp(kind,"task")||!strcmp(kind,"snapshot"))?8000u:2000u);
 char suffix[128];int z=snprintf(suffix,sizeof(suffix),",\"stale\":%s,\"age_ms\":%llu}",stale?"true":"false",(unsigned long long)age);
 if(n<2||data[n-1]!='}'||z<0||(size_t)z>=sizeof(suffix)||n-1+(size_t)z>limit){free(data);error_reply(fd,503,"实物数据格式或响应长度超限");return;}
 char*body=realloc(data,n+(size_t)z+1);if(!body){free(data);error_reply(fd,503,"实物数据内存不足");return;}memcpy(body+n-1,suffix,(size_t)z+1);reply(fd,200,"application/json; charset=utf-8",body,n-1+(size_t)z,NULL);free(body);
}
static void object_selection_reply(int fd,const char*json,const char*session,int clear){
 const char*action=clear?"clear":"select";obj_selection s={0};
 if(!obj_select_parse(json,strlen(json),clear,&s)){object_action_log(action,"RESULT",400,"OBJECT_SELECTION_FORMAT_INVALID",0,0,0,0,0,0);nav_error_reply(fd,400,action,"OBJECT_SELECTION_FORMAT_INVALID","实物选择格式错误");return;}
 pthread_mutex_lock(&mutex);int status=200;const char*reason="NONE";uint64_t request=0,now=millis()*1000,stamp=0;
 object_action_log(action,"RECEIVED",0,"REQUEST_RECEIVED",s.epoch,s.track_id,s.seq,0,now,0);
 if(!control.has_session||strcmp(session,control.session)||!phone_control_live(&control,session,millis())){status=401;reason="OBJECT_SESSION_EXPIRED";}
 else{
  if(strcmp(object_request_session,session)){memcpy(object_request_session,session,33);object_request_sequence=0;}
  if(s.seq<=object_request_sequence){status=409;reason="OBJECT_SEQUENCE_REPLAYED";}
  else{
   char path[PATH_MAX];obj_frame frame;
   if(!clear){
    if(snprintf(path,sizeof(path),"%s/objects-state.json",object_directory)>=(int)sizeof(path)||!obj_frame_load(path,&frame)){status=409;reason="OBJECT_STATE_UNAVAILABLE";}
    else{stamp=frame.frame_us;reason=object_identity_admission(&frame,s.epoch,s.track_id,now,0);if(strcmp(reason,"NONE"))status=409;}
   }
   if(status==200){
    if(now<=object_request_id)now=object_request_id+1;char body[384];int z=snprintf(body,sizeof(body),"{\"version\":1,\"request_id\":\"%llu\",\"epoch\":\"%llu\",\"snapshot_id\":\"%s\",\"track_id\":%d,\"operation\":\"%s\"}",(unsigned long long)now,(unsigned long long)s.epoch,s.snapshot_id,s.track_id,action);
    if(z<0||(size_t)z>=sizeof(body)||snprintf(path,sizeof(path),"%s/objects-select.json",object_directory)>=(int)sizeof(path)||!obj_atomic_write(path,body,(size_t)z)){status=503;reason="OBJECT_SELECTION_QUEUE_WRITE_FAILED";}
    else{voice_external_cancel_locked("用户已明确重新选择实物");object_request_sequence=s.seq;object_request_id=now;request=now;reason="QUEUED_WAIT_PERCEPTION_ACK";}
   }
  }
 }
 pthread_mutex_unlock(&mutex);
 object_action_log(action,"RESULT",status,reason,s.epoch,s.track_id,s.seq,request,now,stamp);
 if(status!=200){nav_error_reply(fd,status,action,reason,status==401?"实物会话已失效":status==409?"选择未排队，请查看具体原因；已发布照片由感知服务核对":"实物选择队列不可写");return;}
 char body[100];int n=snprintf(body,sizeof(body),"{\"queued\":true,\"request_id\":\"%llu\"}",(unsigned long long)request);reply(fd,200,"application/json",body,(size_t)n,NULL);
}
static int object_uint(const char**p,uint64_t*v,uint64_t max){char*end;if(!isdigit((unsigned char)**p)||(**p=='0'&&isdigit((unsigned char)(*p)[1])))return -1;errno=0;*v=strtoull(*p,&end,10);if(errno||end==*p||*v>max)return -1;*p=end;return 0;}
static int object_epoch(const char**p,uint64_t*v){char t[24],*e;if(string(p,t,sizeof(t))||t[0]<'1'||t[0]>'9')return -1;for(size_t i=0;t[i];i++)if(!isdigit((unsigned char)t[i]))return -1;errno=0;*v=strtoull(t,&e,10);return errno||*e||!*v?-1:0;}
static int object_real(const char**p,double*v){const char*q=*p;char*e;if(!isdigit((unsigned char)*q))return -1;if(*q=='0')q++;else while(isdigit((unsigned char)*q))q++;if(*q=='.'){q++;if(!isdigit((unsigned char)*q))return -1;while(isdigit((unsigned char)*q))q++;}if(*q=='e'||*q=='E'){q++;if(*q=='+'||*q=='-')q++;if(!isdigit((unsigned char)*q))return -1;while(isdigit((unsigned char)*q))q++;}errno=0;*v=strtod(*p,&e);if(errno||e!=q||!isfinite(*v))return -1;*p=q;return 0;}
static int object_task_parse(const char*p,phone_goal_message*m,uint64_t*seq){unsigned seen=0;space(&p);if(*p++!='{')return -1;for(unsigned i=0;i<5;i++){char k[24],v[16];unsigned bit;uint64_t n;if(string(&p,k,sizeof(k)))return -1;space(&p);if(*p++!=':')return -1;space(&p);if(!strcmp(k,"seq")){bit=1;if(object_uint(&p,seq,UINT64_C(9007199254740991))||!*seq)return -1;}else if(!strcmp(k,"epoch")){bit=2;if(*p=='"'&&p[1]=='0'&&p[2]=='"'){m->object_epoch=0;p+=3;}else if(object_epoch(&p,&m->object_epoch))return -1;}else if(!strcmp(k,"track_id")){bit=4;if(object_uint(&p,&n,INT32_MAX))return -1;m->object_track_id=(int32_t)n;}else if(!strcmp(k,"mode")){bit=8;if(string(&p,v,sizeof(v)))return -1;m->reserved=!strcmp(v,"static")?1:!strcmp(v,"follow")?2:!strcmp(v,"radar_follow")?3:0;if(!m->reserved)return -1;}else if(!strcmp(k,"standoff_m")){bit=16;if(object_real(&p,&m->x_m)||m->x_m<.1||m->x_m>3)return -1;}else return -1;if(seen&bit)return -1;seen|=bit;space(&p);if(i<4&&*p++!=',')return -1;}if(*p++!='}')return -1;space(&p);return *p||seen!=31||(m->reserved==3?(m->object_epoch!=0||m->object_track_id!=0):(!m->object_epoch||m->object_track_id<1))?-1:0;}
typedef struct object_roi {uint64_t seq,epoch;char snapshot[64];int eye,bbox[4];} object_roi;
static int object_roi_parse(const char*p,object_roi*r){unsigned seen=0;space(&p);if(*p++!='{')return -1;for(unsigned i=0;i<5;i++){char k[24];unsigned bit;uint64_t n;if(string(&p,k,sizeof(k)))return -1;space(&p);if(*p++!=':')return -1;space(&p);if(!strcmp(k,"seq")){bit=1;if(object_uint(&p,&r->seq,UINT64_C(9007199254740991))||!r->seq)return -1;}else if(!strcmp(k,"epoch")){bit=2;if(object_epoch(&p,&r->epoch))return -1;}else if(!strcmp(k,"snapshot_id")){bit=4;if(string(&p,r->snapshot,sizeof(r->snapshot)))return -1;}else if(!strcmp(k,"eye")){bit=8;if(object_uint(&p,&n,1))return -1;r->eye=(int)n;}else if(!strcmp(k,"bbox")){bit=16;if(*p++!='[')return -1;for(int j=0;j<4;j++){space(&p);if(object_uint(&p,&n,j%2?1080:960))return -1;r->bbox[j]=(int)n;space(&p);if(*p++!=(j==3?']':','))return -1;}}else return -1;if(seen&bit)return -1;seen|=bit;space(&p);if(i<4&&*p++!=',')return -1;}if(*p++!='}')return -1;space(&p);if(*p||seen!=31||r->bbox[2]-r->bbox[0]<8||r->bbox[3]-r->bbox[1]<8)return -1;char prefix[32];int z=snprintf(prefix,sizeof(prefix),"%llu-",(unsigned long long)r->epoch);if(z<1||strncmp(r->snapshot,prefix,(size_t)z))return -1;const char*q=r->snapshot+z;uint64_t frame;if(object_uint(&q,&frame,UINT64_MAX)||!frame||*q)return -1;return 0;}
static void object_roi_reply(int fd,const char*json,const char*session){
 object_roi r={0};if(object_roi_parse(json,&r)){error_reply(fd,400,"框选格式错误：必须是当前原图眼身份和有效像素区域");return;}int status=200;uint64_t request=0,now=micros();pthread_mutex_lock(&mutex);
 if(!phone_control_live(&control,session,now/1000))status=401;else{if(strcmp(object_request_session,session)){memcpy(object_request_session,session,33);object_request_sequence=0;}if(r.seq<=object_request_sequence)status=409;else{char file[PATH_MAX],body[512];if(now<=object_request_id)now=object_request_id+1;int n=snprintf(body,sizeof(body),"{\"version\":1,\"request_id\":\"%llu\",\"epoch\":\"%llu\",\"snapshot_id\":\"%s\",\"operation\":\"roi\",\"eye\":%d,\"bbox\":[%d,%d,%d,%d]}",(unsigned long long)now,(unsigned long long)r.epoch,r.snapshot,r.eye,r.bbox[0],r.bbox[1],r.bbox[2],r.bbox[3]);if(n<0||(size_t)n>=sizeof(body)||snprintf(file,sizeof(file),"%s/objects-select.json",object_directory)>=(int)sizeof(file)||!obj_atomic_write(file,body,(size_t)n))status=503;else{object_request_sequence=r.seq;object_request_id=request=now;}}}pthread_mutex_unlock(&mutex);if(status!=200){error_reply(fd,status,"框选会话失效、请求重复或队列不可写；感知端还会核对原图身份");return;}char b[100];int n=snprintf(b,sizeof(b),"{\"queued\":true,\"request_id\":\"%llu\"}",(unsigned long long)request);reply(fd,200,"application/json",b,(size_t)n,NULL);
}
static uint64_t voice_phase_id;
static uint64_t voice_next_id(uint64_t now){if(now<=voice_phase_id)now=voice_phase_id+1;voice_phase_id=now;return now;}
static void *voice_mission_worker(void*unused){(void)unused;
 while(running){struct timespec delay={0,200000000};nanosleep(&delay,NULL);pthread_mutex_lock(&mutex);semantic_spec intent=voice_task.spec;uint64_t rev=voice_task.revision;int active=voice_task.active,phase=voice_task.phase,search_sync=voice_task.search_requested||voice_task.direct_static,sync_track=voice_task.track,direct_mode=!strcmp(voice_task.spec.mode,"FOLLOW")||!strcmp(voice_task.spec.mode,"GO"),unbound_go=!strcmp(voice_task.spec.mode,"GO")&&voice_task.spec.category[0]&&!voice_task.spec.position[0]&&!voice_task.spec.ordinal&&!voice_task.spec.reference;pthread_mutex_unlock(&mutex);if(!active){pthread_mutex_lock(&mutex);if(voice_task.id&&voice_task.revision==rev&&voice_task.last_log_us){char final[4096],dst[PATH_MAX];int n=voice_mission_json(&voice_task,&voice_controller,micros(),final,sizeof final);if(n>0&&n<(int)sizeof final){snprintf(dst,sizeof dst,"%s/voice-state.json",object_directory);obj_atomic_write(dst,final,(size_t)n);fprintf(stderr,"VOICE_FINAL %s\n",final);}voice_task.last_log_us=0;}pthread_mutex_unlock(&mutex);continue;}
  obj_frame *f=NULL,*shot=NULL;voice_receipt receipt={0};voice_perception_diag perception={.source="NOT_NEEDED",.reason="TASK_ALREADY_BOUND"};char path[PATH_MAX],*raw=NULL;size_t bytes=0;uint64_t now=micros();
  if((phase<2&&!unbound_go)||(search_sync&&phase>=4)){f=calloc(1,sizeof *f);if((phase<2&&!direct_mode)||(search_sync&&sync_track>0))shot=calloc(1,sizeof *shot);if(f&&!voice_perception_load_attributes(object_directory,now,f,&perception,intent.category,intent.color,intent.size)){free(f);f=NULL;}
   if(shot){snprintf(path,sizeof path,"%s/objects-snapshot.json",object_directory);int valid=obj_file_read(path,OBJ_SNAPSHOT_MAX,&raw,&bytes)&&obj_snapshot_parse(raw,bytes,shot)&&obj_snapshot_matches(raw,bytes,shot);free(raw);raw=NULL;if(!valid){free(shot);shot=NULL;}}
  }
  snprintf(path,sizeof path,"%s/objects-task.json",object_directory);int have=obj_file_read(path,32768,&raw,&bytes)&&voice_receipt_parse(raw,bytes,&receipt)&&receipt.published_us<=now&&now-receipt.published_us<=1500000;free(raw);raw=NULL;
  pthread_mutex_lock(&mutex);now=micros();if(voice_task.active&&voice_task.revision==rev){voice_task.linked=phone_control_live(&control,voice_task.session,now/1000);if(voice_task.linked)control.autonomous=1;voice_controller=have?receipt:(voice_receipt){0};
   if(f){voice_task.input_frame_us=f->frame_us;voice_task.input_frame_id=f->frame_id;voice_task.input_count=f->count;voice_task.input_observed_persons=0;for(int i=0;i<f->count;i++)if(f->objects[i].observed&&!strcmp(f->objects[i].label,"person"))voice_task.input_observed_persons++;snprintf(voice_task.input_kind,sizeof voice_task.input_kind,"%s",perception.source);}
   unsigned action=(unsigned)voice_mission_step(&voice_task,f,shot,have?&receipt:NULL,now);uint64_t id=voice_next_id(now);int ok=0;
   if(action==VOICE_SELECT&&shot){if(id<=object_request_id)id=object_request_id+1;object_request_id=id;char body[384];int n=snprintf(body,sizeof body,"{\"version\":1,\"request_id\":\"%llu\",\"epoch\":\"%llu\",\"snapshot_id\":\"%llu-%llu\",\"track_id\":%d,\"operation\":\"select\"}",(unsigned long long)id,(unsigned long long)voice_task.epoch,(unsigned long long)shot->epoch,(unsigned long long)shot->frame_id,voice_task.track);snprintf(path,sizeof path,"%s/objects-select.json",object_directory);ok=n>0&&n<(int)sizeof body&&obj_atomic_write(path,body,(size_t)n);}
   else if(action==VOICE_PREPARE||action==VOICE_SEARCH_PREPARE||action==VOICE_DIRECT_PREPARE||action==VOICE_DIRECT_STATIC_PREPARE||action==VOICE_START){phone_goal_message m={0};if(voice_mission_control_message(&voice_task,&m)){m.request_id=id;voice_internal=1;ok=!navigation_send_locked(&m,voice_task.session,action==VOICE_START?PHONE_NAV_OBJECT_START:PHONE_NAV_OBJECT_PREPARE);voice_internal=0;}}

   if(ok)voice_mission_issued(&voice_task,action,id,now);
   if(!voice_task.last_log_us||now-voice_task.last_log_us>=1000000){char b[4096];int n=voice_mission_json(&voice_task,have?&receipt:NULL,now,b,sizeof b);if(n>0&&n<(int)sizeof b){fprintf(stderr,"VOICE_V10_1 %s\n",b);snprintf(path,sizeof path,"%s/voice-state.json",object_directory);obj_atomic_write(path,b,(size_t)n);}fprintf(stderr,"VOICE_PERCEPTION source=%s reason=%s detector_frame=%llu detector_source_us=%llu detector_objects=%d tracking_frame=%llu tracking_source_us=%llu tracking_objects=%d consumed_frame=%llu consumed_source_us=%llu age_ms=%lld message_zh=优先近期真实检测，预测发布不刷新检测时间\n",perception.source,perception.reason,(unsigned long long)perception.detector_frame,(unsigned long long)perception.detector_us,perception.detector_objects,(unsigned long long)perception.tracking_frame,(unsigned long long)perception.tracking_us,perception.tracking_objects,(unsigned long long)(f?f->frame_id:0),(unsigned long long)(f?f->frame_us:0),(long long)(f&&f->frame_us<=now?(now-f->frame_us)/1000:UINT64_MAX));voice_task.last_log_us=now;}
  }pthread_mutex_unlock(&mutex);free(f);free(shot);
 }return NULL;
}
static void voice_http_reply(int fd,const char*path,const char*json,const char*session,int get){
 pthread_mutex_lock(&mutex);uint64_t now=micros();int status=200;const char*error="NONE";
 if(!phone_control_live(&control,session,now/1000)){status=401;error="语音控制会话已失效，旧任务不能自动换会话开始";}
 if(status==200&&get){char b[4096];voice_receipt *receipt=voice_controller.published_us&&voice_controller.published_us<=now&&now-voice_controller.published_us<=1500000?&voice_controller:NULL;int n=voice_mission_json(&voice_task,receipt,now,b,sizeof b);pthread_mutex_unlock(&mutex);if(n>0&&n<(int)sizeof b)reply(fd,200,"application/json; charset=utf-8",b,(size_t)n,NULL);else error_reply(fd,503,"语音状态编码超出上限");return;}
 phone_goal_message msg={0};uint64_t seq=0;unsigned op=!strcmp(path,"/api/semantic/task")?PHONE_SEMANTIC_TASK:!strcmp(path,"/api/semantic/start")?PHONE_SEMANTIC_START:PHONE_SEMANTIC_CANCEL;
 if(status==200&&!semantic_payload(json,op,&msg,&seq)){status=400;error="语义字段无效：请核对动作、类别、颜色、大小、方位、序号或0.1～3米间距";}
 if(status==200){if(strcmp(navigation_request_session,session)){memcpy(navigation_request_session,session,33);navigation_request_sequence=0;}
  if(seq<=navigation_request_sequence){status=409;error="旧语音请求已被更新请求或停车取代";}
  else if(op==PHONE_SEMANTIC_TASK&&(control.motion||sensor.auto_mode)){status=409;error="先结束手动运动/巡航；语音任务未接管电机";}
  else {int outcome=voice_mission_accept(&voice_task,&msg,session,seq,now);if(!outcome){status=409;error="语音任务已取消、身份不一致或尚无可开始的预览；原任务不会被无效请求重建";}
   else{navigation_request_sequence=seq;if(op==PHONE_SEMANTIC_TASK){memcpy(object_request_session,session,33);if(seq>object_request_sequence)object_request_sequence=seq;}
    if(outcome==1||outcome==3){control.autonomous=outcome==1;uint64_t revoke=0;phone_switch_clear(&auto_switch);(void)phone_control_fence_revoke(control_directory,&revoke);wire_send(PHONE_STOP,0);control.motion=control.pending=0;control.stopped=1;phone_goal_message stop={0};voice_internal=1;(void)navigation_send_locked(&stop,session,PHONE_NAV_OVERRIDE);voice_internal=0;}
   }
  }
 }
 pthread_mutex_unlock(&mutex);if(status!=200){error_reply(fd,status,error);return;}char b[128];int n=snprintf(b,sizeof b,"{\"queued\":true,\"task_id\":\"%llu\"}",(unsigned long long)msg.map_id);reply(fd,200,"application/json",b,(size_t)n,NULL);
}
static void object_task_reply(int fd,const char*json,const char*session,unsigned operation){
 const char*action=operation==PHONE_NAV_OBJECT_PREPARE?"prepare":operation==PHONE_NAV_OBJECT_START?"start":operation==PHONE_NAV_OBJECT_CANCEL?"cancel":operation==PHONE_NAV_OBJECT_PAUSE?"pause":"resume";
 phone_goal_message m={0};uint64_t seq=0;if(object_task_parse(json,&m,&seq)){object_action_log(action,"RESULT",400,"OBJECT_TASK_FORMAT_INVALID",0,0,0,0,0,0);error_reply(fd,400,"任务格式错误：仅接收目标身份、模式和间距");return;}
 if((operation==PHONE_NAV_OBJECT_PAUSE||operation==PHONE_NAV_OBJECT_RESUME)&&m.reserved!=2&&m.reserved!=3){object_action_log(action,"RESULT",400,"OBJECT_FOLLOW_MODE_REQUIRED",m.object_epoch,m.object_track_id,seq,0,0,0);error_reply(fd,400,"暂停/继续只适用于跟随模式");return;}
 int status=200;const char*reason="NONE",*code="NONE";uint64_t now=micros(),stamp=0;pthread_mutex_lock(&mutex);
 object_action_log(action,"RECEIVED",0,"REQUEST_RECEIVED",m.object_epoch,m.object_track_id,seq,0,now,0);
 if(!phone_control_live(&control,session,now/1000)){status=401;reason="会话失效，请重新连接";code="OBJECT_SESSION_EXPIRED";}else{
  if(strcmp(navigation_request_session,session)){memcpy(navigation_request_session,session,33);navigation_request_sequence=0;}
  if(seq<=navigation_request_sequence){status=409;reason="任务序号重复或过期";code="OBJECT_SEQUENCE_REPLAYED";}else{
   char file[PATH_MAX];obj_frame frame;if(m.reserved!=3&&operation!=PHONE_NAV_OBJECT_CANCEL&&operation!=PHONE_NAV_OBJECT_PAUSE){
    if(snprintf(file,sizeof(file),"%s/objects-state.json",object_directory)>=(int)sizeof(file)||!obj_frame_load(file,&frame)){status=409;code="OBJECT_STATE_UNAVAILABLE";reason="感知状态不可读，请等待服务发布";}
    else{stamp=frame.generated_us;code=object_identity_admission(&frame,m.object_epoch,m.object_track_id,now,1);if(strcmp(code,"NONE")){status=409;reason="当前锁定目标身份不一致、目标不存在或感知服务发布过期";}}
   }
   if(status==200){voice_external_cancel_locked("已明确切换到手动实物任务");m.request_id=seq;navigation_request_sequence=seq;if(operation==PHONE_NAV_OBJECT_CANCEL){uint64_t generation=0;phone_switch_clear(&auto_switch);control.motion=0;control.pending=0;control.stopped=1;(void)phone_control_fence_revoke(control_directory,&generation);wire_send(PHONE_STOP,0);}if(navigation_send_locked(&m,session,operation)){status=503;reason="任务控制器通信失败，请检查地图进程";code="OBJECT_NAV_IPC_UNAVAILABLE";}else{if(operation==PHONE_NAV_OBJECT_START||operation==PHONE_NAV_OBJECT_RESUME)control.autonomous=1;else if(operation==PHONE_NAV_OBJECT_CANCEL||operation==PHONE_NAV_OBJECT_PAUSE)control.autonomous=0;code=operation==PHONE_NAV_OBJECT_PREPARE?"QUEUED_REGISTER_TASK_MOTION_WAITS_OBSERVATION":"QUEUED_WAIT_CONTROLLER_ACK";}}
  }
 }pthread_mutex_unlock(&mutex);object_action_log(action,"RESULT",status,code,m.object_epoch,m.object_track_id,seq,status==200?seq:0,now,stamp);if(status!=200){nav_error_reply(fd,status,action,code,reason);return;}char b[128];int n=snprintf(b,sizeof(b),"{\"queued\":true,\"request_id\":\"%llu\"}",(unsigned long long)seq);reply(fd,200,"application/json",b,(size_t)n,NULL);
}
/* AUTO事务时间由服务端记录。 */
static void auto_switch_reply(int fd,const char*json,const char*session,int prepare){
 uint64_t q=0;char supplied[33]={0};if(phone_auto_request_json(json,&q,supplied,prepare)){nav_error_reply(fd,400,prepare?"auto/prepare":"auto","AUTO_REQUEST_INVALID","自动模式请求格式不兼容，请使用配套App");return;}
 uint64_t issued=0,expires=0,generation=0,now=micros();int status=200;const char*code="NONE";char token[33]={0};
 pthread_mutex_lock(&mutex);
 if(!now||!phone_control_live(&control,session,now/1000)){status=401;code="SESSION_EXPIRED";}
 else if(!simulate&&direct_link.policy_mismatch){status=409;code="FPGA_POLICY_MISMATCH";}
 else{
  if(strcmp(navigation_request_session,session)){memcpy(navigation_request_session,session,33);navigation_request_sequence=0;}
  if(q<=navigation_request_sequence){status=409;code="REQUEST_DUPLICATE";}
  else{navigation_request_sequence=q;
   if(claiming){status=409;code="CONTROL_CLAIM_PENDING";}
   else if(prepare){
    phone_switch_clear(&auto_switch);control.motion=0;control.pending=0;control.stopped=1;
    uint8_t bytes[16];phone_goal_message override={0};int revoked=phone_control_fence_revoke(control_directory,&generation);wire_send(PHONE_STOP,0);
    if(revoked){status=503;code="REVOCATION_UNAVAILABLE";}
    else if(navigation_send_locked(&override,session,PHONE_NAV_OVERRIDE)){
     fprintf(stderr,"AUTO_MAP_CANCEL_NOTE reason=NAV_IPC_UNAVAILABLE motor_stop_sent=1\n");
     if(entropy(bytes,sizeof(bytes))){status=503;code="AUTO_TOKEN_UNAVAILABLE";}
     else{for(unsigned i=0;i<16;i++)sprintf(token+i*2,"%02x",bytes[i]);issued=micros();expires=issued+PHONE_AUTO_INTENT_US;if(phone_switch_issue(&auto_switch,session,token,issued,generation)!=PHONE_SWITCH_OK){status=503;code="AUTO_TOKEN_UNAVAILABLE";}}
    }
    else if(entropy(bytes,sizeof(bytes))){status=503;code="AUTO_TOKEN_UNAVAILABLE";}
    else{for(unsigned i=0;i<16;i++)sprintf(token+i*2,"%02x",bytes[i]);issued=micros();expires=issued+PHONE_AUTO_INTENT_US;if(phone_switch_issue(&auto_switch,session,token,issued,generation)!=PHONE_SWITCH_OK){status=503;code="AUTO_TOKEN_UNAVAILABLE";}}
   }else{
    int fs=phone_control_fence_read(control_directory,&generation),take=fs<0?PHONE_SWITCH_REVOKED:phone_switch_consume(&auto_switch,session,supplied,now,generation,&issued,&expires);
    if(take!=PHONE_SWITCH_OK){status=409;code=take==PHONE_SWITCH_EXPIRED?"AUTO_SWITCH_EXPIRED":take==PHONE_SWITCH_REVOKED?"AUTO_SWITCH_REVOKED":"AUTO_SWITCH_INVALID";}
    else if(control.motion){status=409;code="MANUAL_CONTROL_ACTIVE";}
    else if(!simulate&&(!getenv("ROBOT_NAV_MODE")||(strcmp(getenv("ROBOT_NAV_MODE"),"drive")&&strcmp(getenv("ROBOT_NAV_MODE"),"cruise")))){status=409;code="PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE";}
    else if(!control_transport_ready(now/1000)||final_fault()){status=409;code="CONTROL_LINK_OR_ENCODER_NOT_READY";}
    else{
     phone_sonar_auto_begin(&fpga_auto,session,issued,now/1000);control.autonomous=1;control.motion=0;control.pending=0;control.stopped=0;
     wire_send(PHONE_AUTO,0);
     fprintf(stderr,"AUTO_CONTROLLER backend=FPGA_SONAR_NEARSTOP request_us=%llu front_mm=400/150/200 side_mm=180/100/130 resume_mm=250\n",(unsigned long long)issued);
    }
   }
  }
 }
 pthread_mutex_unlock(&mutex);
 if(status!=200){fprintf(stderr,"AUTO_TRANSACTION stage=%s seq=%llu outcome=REJECTED code=%s now_us=%llu\n",prepare?"PREPARE":"CONSUME",(unsigned long long)q,code,(unsigned long long)now);nav_error_reply(fd,status,prepare?"auto/prepare":"auto",code,"自动模式未启动，请根据原因重新操作");return;}
 char body[256];int n;if(prepare)n=snprintf(body,sizeof(body),"{\"switch_token\":\"%s\",\"issued_us\":\"%llu\",\"expires_us\":\"%llu\"}",token,(unsigned long long)issued,(unsigned long long)expires);
 else n=snprintf(body,sizeof(body),"{\"queued\":true,\"auto_request_us\":\"%llu\"}",(unsigned long long)issued);
 phone_log_event(prepare?"auto/prepare":"auto",200,"AUTO_QUEUED");fprintf(stderr,"AUTO_TRANSACTION stage=%s seq=%llu outcome=%s issued_us=%llu expires_us=%llu generation=%llu\n",prepare?"PREPARE":"CONSUME",(unsigned long long)q,prepare?"TOKEN_ISSUED":"QUEUED",(unsigned long long)issued,(unsigned long long)expires,(unsigned long long)generation);reply(fd,200,"application/json",body,(size_t)n,NULL);
}
static void motion_reply(int fd){
 char path[PATH_MAX],data[16385];if(snprintf(path,sizeof(path),"%s/navigation-motion.json",control_directory)>=(int)sizeof(path)){nav_error_reply(fd,503,"motion","MOTION_NOT_READY","运动预览尚未就绪");return;}
 int source=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);struct stat st;int valid=source>=0&&fstat(source,&st)==0&&S_ISREG(st.st_mode)&&st.st_size>1&&st.st_size<=16384&&(st.st_mode&0022)==0;
 ssize_t bytes=valid?read(source,data,(size_t)st.st_size):-1;if(source>=0)close(source);uint64_t publication=0,now=micros();
 if(bytes<2||bytes!=st.st_size||!obj_motion_meta(data,(size_t)bytes,&publication)||!publication||publication>now){nav_error_reply(fd,503,"motion","MOTION_NOT_READY","运动预览尚未就绪或文件无效");return;}
 uint64_t age=now-publication;if(age>1000000){nav_error_reply(fd,503,"motion","MOTION_STALE","运动预览已过期，保留旧线条仅作历史显示");return;}
 phone_log_event("motion",200,"MOTION_OK");char extra[96];snprintf(extra,sizeof(extra),"X-Motion-Age-Ms: %llu\r\n",(unsigned long long)((age+999)/1000));reply(fd,200,"application/json; charset=utf-8",data,(size_t)bytes,extra);
}
static void state_reply(int fd){
    struct video_stats video={0};if(h264_enabled)video_service_stats(&video);
    char body[24576],ranges[1536],permissions[256];uint64_t now=millis();s1_mode_state rk={0};mode_read(&rk,now);pthread_mutex_lock(&mutex);
    if(fpga_auto.active){
     rk.known=1;rk.ready=control_transport_ready(now);rk.intent_accepted=1;rk.active=fresh(now)&&sensor.auto_mode;
     rk.waiting=!rk.active||sensor.actual==0;rk.request_us=fpga_auto.request_us;memcpy(rk.session,fpga_auto.session,33);
     snprintf(rk.mode,sizeof(rk.mode),"AUTO");snprintf(rk.status,sizeof(rk.status),"%s",rk.waiting?"WAITING":"ACTIVE");
     snprintf(rk.reason,sizeof(rk.reason),"%s",phone_sonar_auto_reason(sensor.nav_state,sensor.stop_reason,sensor.feedback_phase,rk.active));
    }else if(fpga_auto.failed&&!strcmp(fpga_auto.session,control.session)){
     memset(&rk,0,sizeof(rk));rk.known=1;rk.request_us=fpga_auto.request_us;memcpy(rk.session,fpga_auto.session,33);
     snprintf(rk.mode,sizeof(rk.mode),"AUTO");snprintf(rk.status,sizeof(rk.status),"DENIED");
     snprintf(rk.reason,sizeof(rk.reason),"%s",fpga_auto.acknowledged?"FPGA已退出自动模式，请检查通信、按键和停车记录后重新启动":"FPGA未确认自动启动，请检查配套FS和板内通信后重试");
    }
    int rk_session=!rk.session[0]||!strcmp(rk.session,control.session),auto_active=rk.known&&rk_session&&rk.active&&!strcmp(rk.mode,"AUTO");
    int auto_intent=rk.known&&rk_session&&rk.intent_accepted&&!strcmp(rk.mode,"AUTO");
    const char*mode=rk.known&&rk_session&&!strcmp(rk.mode,"NAV")?"navigation":auto_intent||auto_active?"auto":"manual";
    int live=fresh(now),can=ready(now),support=supported(),session=session_ready(now);
    const char *reason=direct_link.policy_mismatch?(direct_link.max_rpm_centi!=12000?"轮速能力不匹配：本版需要120RPM，请配套更新FS与RK":"FPGA_POLICY_MISMATCH：需要CFG31110301、状态5、命令500ms，请成套更新FS与RK"):!live?"等待新鲜的板端数据":!support?"板端尚未启用遥控":claiming?"正在确认遥控会话":final_fault()?"电机最终故障，请检查后重新启动":(sensor.remote_flags&3)!=3||(sensor.remote_flags&12)?"请切回手动以建立遥控会话":!control_transport_ready(now)?"等待真实控制链路与编码器初始化":strcmp(mode,"manual")?rk.reason:"手动控制独立运行，测距与定位仅作提示";
    unsigned used=0;for(unsigned i=0;i<3;i++){
        unsigned e=sensor.error[i];const char *kind=!live?"stale":!sensor.nav_supported?(e?"invalid":(sensor.range_valid&(1u<<i))?"live":"max"):
            e==5?"waiting":e==6?"lost":e==4?"blind":e?(sensor.range_valid&(1u<<i)?"hold":"invalid"):(sensor.range_valid&(1u<<i))?"live":"max";
        int age_known=direct_link.have_status&&now>=direct_link.status_ms&&now-direct_link.status_ms<PHONE_LINK_MS&&(direct_link.sonar_valid&(1u<<i));
        unsigned bucket=direct_link.sonar_age_bucket[i],lower=bucket*200;
        int raw_zero=direct_link.have_status&&now>=direct_link.status_ms&&now-direct_link.status_ms<PHONE_LINK_MS&&(direct_link.sonar_raw_zero_mask&(1u<<i));
        char lo[24]="null",hi[24]="null";if(age_known){snprintf(lo,sizeof(lo),"%u",lower);if(bucket<15)snprintf(hi,sizeof(hi),"%u",lower+200);}
        used+=(unsigned)snprintf(ranges+used,sizeof(ranges)-used,"%s{\"mm\":%u,\"state\":\"%s\",\"error\":%u,\"near\":%s,\"stop_mm\":%u,\"nonzero_age_ms\":null,\"age_known\":%s,\"nonzero_age_min_ms\":%s,\"nonzero_age_max_ms\":%s,\"raw_zero_ignored\":%s}",i?",":"",sensor.mm[i],kind,e,(sensor.near_mask&(1u<<i))?"true":"false",(fpga_auto.active?(i==0?150u:100u):phone_range_stop_mm(&sensor,i)),age_known?"true":"false",lo,hi,raw_zero?"true":"false");
    }
    static const char *const names[]={"forward","reverse","curve_left","curve_right","spin_left","spin_right","back_left","back_right"};
    used=0;for(unsigned i=0;i<8;i++)used+=(unsigned)snprintf(permissions+used,sizeof(permissions)-used,"%s\"%s\":%s",i?",":"",names[i],session?"true":"false");
    uint64_t imu_age=now>=sensor.imu_ms?now-sensor.imu_ms:UINT32_MAX;int imu_live=have_sensor&&sensor.imu_state==1&&imu_age<1000;
    int n=snprintf(body,sizeof(body),"{\"protocol\":1,\"simulated\":%s,\"control\":{\"ready\":%s,\"auto_ready\":%s,\"supported\":%s,\"telemetry_fresh\":%s,\"mode\":\"%s\",\"moving\":%s,\"drive_status\":\"%s\",\"escape_active\":%s,\"reason\":\"%s\",\"motion_allowed\":{%s},\"watchdog_ms\":500,\"upstream_lease_ms\":3000,\"autonomous_lease_ms\":8000,\"manual_hold_ms\":1500,\"manual_ready\":%s,\"final_motor_fault\":%s},\"telemetry\":{\"imu\":{\"status\":\"%s\",\"age_ms\":%"PRIu64",\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f},\"robot\":{\"mode\":\"%s\",\"actual\":%u,\"action\":\"%s\",\"feedback\":%u,\"feedback_supported\":%s,\"feedback_phase\":%u,\"feedback_retries\":%u,\"stop_reason\":%u,\"nav_supported\":%s,\"nav_state\":%u,\"near_mask\":%u,\"remote_flags\":%u,\"remote_sequence\":%u,\"ranges\":[%s]}},\"video\":{\"width\":%u,\"height\":%u,\"age_ms\":%"PRIu64",\"codec\":\"%s\",\"fps\":%.3f,\"encoder_ms\":%.3f,\"failed\":%s}}",
        simulate?"true":"false",can?"true":"false",(control_transport_ready(now)&&supported()&&!claiming&&!final_fault()&&(simulate||(getenv("ROBOT_NAV_MODE")&&(!strcmp(getenv("ROBOT_NAV_MODE"),"drive")||!strcmp(getenv("ROBOT_NAV_MODE"),"cruise")))))?"true":"false",support?"true":"false",live?"true":"false",mode,live&&sensor.actual?"true":"false",drive_status(live,now),strcmp(mode,"manual")&&live&&phone_escape_motion(sensor.actual)?"true":"false",reason,permissions,session?"true":"false",final_fault()?"true":"false",
        imu_live?"live":sensor.imu_state==2||sensor.imu_state==3?"invalid":"waiting",imu_age,isfinite(sensor.roll)?sensor.roll:0,isfinite(sensor.pitch)?sensor.pitch:0,isfinite(sensor.yaw)?sensor.yaw:0,
        mode,sensor.actual,robot_action_text(sensor.actual),sensor.feedback,sensor.feedback_supported?"true":"false",sensor.feedback_phase,sensor.feedback_retries,sensor.stop_reason,sensor.nav_supported?"true":"false",sensor.nav_state,sensor.near_mask,sensor.remote_flags,sensor.remote_sequence,ranges,h264_enabled?1280:jpeg_width,h264_enabled?720:jpeg_height,h264_enabled?(video.last_capture_ms&&now>=video.last_capture_ms?now-video.last_capture_ms:UINT32_MAX):(jpeg_ms&&now>=jpeg_ms?now-jpeg_ms:UINT32_MAX),h264_enabled?"h264":"jpeg",video.fps_milli/1000.0,video.encode_us/1000.0,video.failed?"true":"false");
    if(n>0&&(size_t)n+512<sizeof(body)){n--;n+=snprintf(body+n,sizeof(body)-(size_t)n,",\"autonomy\":{\"mode_known\":%s,\"mode\":\"%s\",\"auto_active\":%s,\"auto_intent_accepted\":%s,\"auto_waiting\":%s,\"request_us\":\"%llu\",\"status\":\"%s\",\"reason\":\"%s\"}}",rk.known?"true":"false",mode,auto_active?"true":"false",auto_intent?"true":"false",auto_intent&&rk.waiting?"true":"false",(unsigned long long)rk.request_us,rk.known?rk.status:"WAIT_RK_MODE",rk.known?rk.reason:"WAIT_RK_MODE");}
    if(n>0&&(size_t)n+384<sizeof(body)){n--;n+=snprintf(body+n,sizeof(body)-(size_t)n,",\"policy_revision\":5,\"fpga_policy\":{\"expected_config_id\":823198465,\"expected_status_revision\":5,\"expected_command_ttl_ms\":500,\"received_command_ttl_ms\":%u,\"expected_max_rpm\":120,\"received_max_rpm\":%.2f,\"received_config_id\":%u,\"received_status_revision\":%u,\"mismatch\":%s,\"reason\":\"%s\"}}",(unsigned)direct_link.max_ttl_ms,direct_link.max_rpm_centi/100.,direct_link.config,(unsigned)direct_link.status_revision,direct_link.policy_mismatch?"true":"false",direct_link.policy_mismatch?"FPGA_POLICY_MISMATCH":"NONE");}
    if(n>0&&(size_t)n+1400<sizeof(body)){
        int diag=direct_link.have_diagnostic&&now>=direct_link.diagnostic_ms&&now-direct_link.diagnostic_ms<PHONE_LINK_MS;
        n--;n+=snprintf(body+n,sizeof(body)-(size_t)n,",\"s5_diagnostics\":{\"policy_revision\":5,\"available\":%s,\"age_ms\":%llu,\"raw_available_mask\":%u,\"raw_mm\":[%u,%u,%u],\"nonzero_sequence\":[%u,%u,%u],\"raw_age_us\":[%u,%u,%u],\"effective_duty\":[%u,%u,%u,%u],\"pwm_period\":2500,\"pwm_max\":1750,\"flags\":%u,\"fresh\":%u,\"near\":%u,\"pending\":%u,\"observation\":%u}}",diag?"true":"false",(unsigned long long)(direct_link.have_diagnostic&&now>=direct_link.diagnostic_ms?now-direct_link.diagnostic_ms:UINT64_MAX),direct_link.raw_available,direct_link.raw_mm[0],direct_link.raw_mm[1],direct_link.raw_mm[2],direct_link.raw_sequence[0],direct_link.raw_sequence[1],direct_link.raw_sequence[2],direct_link.raw_age_us[0],direct_link.raw_age_us[1],direct_link.raw_age_us[2],direct_link.effective_duty[0],direct_link.effective_duty[1],direct_link.effective_duty[2],direct_link.effective_duty[3],direct_link.diagnostic_flags,direct_link.diagnostic_fresh,direct_link.diagnostic_near,direct_link.diagnostic_pending,direct_link.diagnostic_observation);
    }
    struct odom_state odometry={0};char pose_json[16384];
    const char *odom_dir=getenv("ACG_PHONE_DIR");if(!odom_dir||!*odom_dir)odom_dir="/run/acg138-phone";
    (void)odom_shared_read(odom_dir,&odometry);
    size_t pose_bytes=odom_json(pose_json,sizeof(pose_json),&odometry,now);
    if(n>0&&pose_bytes&&(size_t)n+pose_bytes+16<sizeof(body)){
        n--;n+=snprintf(body+n,sizeof(body)-(size_t)n,",\"odometry\":%s}",pose_json);
    }
    pthread_mutex_unlock(&mutex);if(n<0||(size_t)n>=sizeof(body)){error_reply(fd,500,"状态数据过长");return;}reply(fd,200,"application/json; charset=utf-8",body,(size_t)n,NULL);
}
static void *client_thread(void*arg){
    int fd=(int)(intptr_t)arg;char request[8192];size_t used=0;char*head_end=NULL;int content_length=0,have_length=0,authorized=0,have_video_session=0;char method[8],path[128],video_session[33]={0};
    int no_delay=1;setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&no_delay,sizeof(no_delay));
    struct timeval timeout={.tv_sec=1};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
    while(used<sizeof(request)-1){ssize_t n=recv(fd,request+used,sizeof(request)-1-used,0);if(n<=0)goto finish;used+=(size_t)n;request[used]=0;head_end=strstr(request,"\r\n\r\n");if(head_end)break;}
    if(!head_end||sscanf(request,"%7s %127s",method,path)!=2){error_reply(fd,400,"请求格式错误");goto finish;}
    char*line=strstr(request,"\r\n");if(!line)goto finish;line+=2;
    while(line<head_end){char*end=strstr(line,"\r\n");if(!end||end>head_end)goto finish;*end=0;
        if(!strncasecmp(line,"Authorization:",14)){const char*s=line+14;while(*s==' ')s++;if(!strncmp(s,"Bearer ",7)&&equal_secret(s+7))authorized++;}
        if(!strncasecmp(line,"X-Rover-Session:",16)){const char *value=line+16;while(*value==' ')value++;if(have_video_session++||strlen(value)!=32){error_reply(fd,400,"视频会话格式错误");goto finish;}for(unsigned i=0;i<32;i++)if(!isxdigit((unsigned char)value[i])){error_reply(fd,400,"视频会话格式错误");goto finish;}memcpy(video_session,value,33);}
        if(!strncasecmp(line,"Content-Length:",15)){char*endnum;errno=0;long value=strtol(line+15,&endnum,10);if(errno||*endnum||value<0||value>1024||have_length++){error_reply(fd,400,"请求长度错误");goto finish;}content_length=(int)value;}
        if(!strncasecmp(line,"Transfer-Encoding:",18)){error_reply(fd,400,"不支持该传输格式");goto finish;}line=end+2;
    }
    if(!strcmp(method,"GET")&&!strcmp(path,"/api/info")){
        static const char info[]="{\"service\":\"acg138-rover\",\"protocol\":1,\"auth\":\"password\",\"version\":\"1.36.0-r81-yolo-fpgaimu\",\"manual_independent\":true,\"config_id\":823198465,\"status_revision\":5,\"command_version\":3,\"policy_revision\":5}";
        reply(fd,200,"application/json",info,sizeof(info)-1,NULL);goto finish;
    }
    if(authorized!=1){phone_log_event("auth",401,"AUTH_REQUIRED");if(!strncmp(path,"/api/nav/",9))nav_error_reply(fd,401,"auth","AUTH_REQUIRED","连接密码不正确");else error_reply(fd,401,"连接密码不正确");goto finish;}
    phone_log_event("auth",200,"AUTH_OK");
    size_t offset=(size_t)(head_end+4-request);while(used-offset<(size_t)content_length){if(used>=sizeof(request)-1)goto finish;ssize_t n=recv(fd,request+used,sizeof(request)-1-used,0);if(n<=0)goto finish;used+=(size_t)n;}request[offset+(size_t)content_length]=0;
    if(!strcmp(method,"GET")&&!strcmp(path,"/api/nav/motion")){
        if(!have_video_session||!view_session_live(video_session)){nav_error_reply(fd,401,"motion","SESSION_EXPIRED","导航会话已失效，请重新连接");goto finish;}motion_reply(fd);goto finish;
    }
    if(!strcmp(method,"GET")&&!strcmp(path,"/api/state")){state_reply(fd);goto finish;}
    int snapshot_eye=object_snapshot_eye(path);
    if(!strcmp(method,"GET")&&snapshot_eye==-3){error_reply(fd,400,"图像眼参数无效，只接受eye=0或eye=1");goto finish;}
    if((!strcmp(method,"GET")&&!strcmp(path,"/api/semantic/state"))||(!strcmp(method,"POST")&&(!strcmp(path,"/api/semantic/task")||!strcmp(path,"/api/semantic/start")||!strcmp(path,"/api/semantic/cancel")))){
      if(!have_video_session||(!strcmp(method,"POST")&&!have_length)){error_reply(fd,401,"语音请求缺少有效会话或消息长度");goto finish;}
      voice_http_reply(fd,path,request+offset,video_session,!strcmp(method,"GET"));goto finish;
    }
    if(!strcmp(method,"GET")&&(snapshot_eye>=0||!strcmp(path,"/api/objects/state")||!strcmp(path,"/api/objects/snapshot")||!strcmp(path,"/api/objects/map")||!strcmp(path,"/api/objects/scene")||!strcmp(path,"/api/objects/task")||!strcmp(path,"/api/objects/association")||!strcmp(path,"/api/objects/association-snapshot"))){
        if(!have_video_session||!h264_session_live(video_session)){error_reply(fd,401,"实物会话已失效");goto finish;}
        if(!strcmp(path,"/api/objects/association")||!strcmp(path,"/api/objects/association-snapshot"))object_association_reply(fd,!strcmp(path,"/api/objects/association-snapshot"));else object_get_reply(fd,snapshot_eye>=0?"snapshot":path+13,snapshot_eye);goto finish;
    }
    if(!strcmp(method,"POST")&&(!strcmp(path,"/api/objects/select")||!strcmp(path,"/api/objects/clear")||!strcmp(path,"/api/objects/roi")||!strcmp(path,"/api/objects/task/prepare")||!strcmp(path,"/api/objects/task/start")||!strcmp(path,"/api/objects/task/cancel")||!strcmp(path,"/api/objects/task/pause")||!strcmp(path,"/api/objects/task/resume"))){
        if(!have_length||!have_video_session||!h264_session_live(video_session)){error_reply(fd,401,"实物会话已失效");goto finish;}
        if(!strcmp(path,"/api/objects/roi"))object_roi_reply(fd,request+offset,video_session);
        else if(!strncmp(path,"/api/objects/task/",18))object_task_reply(fd,request+offset,video_session,!strcmp(path+18,"prepare")?PHONE_NAV_OBJECT_PREPARE:!strcmp(path+18,"start")?PHONE_NAV_OBJECT_START:!strcmp(path+18,"pause")?PHONE_NAV_OBJECT_PAUSE:!strcmp(path+18,"resume")?PHONE_NAV_OBJECT_RESUME:PHONE_NAV_OBJECT_CANCEL);
        else object_selection_reply(fd,request+offset,video_session,!strcmp(path,"/api/objects/clear"));goto finish;
    }
    if(!strcmp(method,"GET")&&!strcmp(path,"/api/vision/tags")){
        if(!have_video_session||!h264_session_live(video_session)){error_reply(fd,401,"视觉会话已失效");goto finish;}
        int vf=open(vision_tags_path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);struct stat st;
        if(vf<0){error_reply(fd,503,"等待 AprilTag 服务");goto finish;}
        if(fstat(vf,&st)||!S_ISREG(st.st_mode)||st.st_size<2||st.st_size>32768){close(vf);error_reply(fd,503,"视觉快照格式错误");goto finish;}
        char data[32768+80];size_t done=0,length=(size_t)st.st_size;while(done<length){ssize_t z=read(vf,data+done,length-done);if(z<=0)break;done+=(size_t)z;}close(vf);
        if(done==length){while(length&&isspace((unsigned char)data[length-1]))length--;tag_snapshot parsed;uint64_t now_us=millis()*1000;if(!tag_snapshot_parse(data,length,&parsed)){error_reply(fd,503,"视觉快照 JSON 无效");goto finish;}
         int stale=!parsed.frame_us||parsed.frame_us>now_us||now_us-parsed.frame_us>500000;
         if(length<2||data[length-1]!='}'){error_reply(fd,503,"视觉快照 JSON 无效");goto finish;}
         int extra=snprintf(data+length-1,sizeof(data)-length+1,",\"stale\":%s,\"age_ms\":%llu}",stale?"true":"false",(unsigned long long)(now_us>=parsed.frame_us?(now_us-parsed.frame_us)/1000:UINT64_MAX));
         if(extra<0||(size_t)extra>=sizeof(data)-length+1){error_reply(fd,503,"视觉快照过大");goto finish;}length=length-1+(size_t)extra;reply(fd,200,"application/json; charset=utf-8",data,length,NULL);
        }else error_reply(fd,503,"读取视觉快照失败");goto finish;
    }
    if(!strcmp(method,"GET")&&!strcmp(path,"/api/nav/map")){
        if(!have_video_session||!view_session_live(video_session)){nav_error_reply(fd,401,"map","SESSION_EXPIRED","导航会话已失效，请重新连接");goto finish;}
        int mapfd=open(navigation_map_path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);struct stat st;struct timespec now;
        if(mapfd<0){nav_error_reply(fd,503,"map","MAP_NOT_READY","地图尚未生成，保留上次画面等待更新");goto finish;}
        if(fstat(mapfd,&st)||!S_ISREG(st.st_mode)||st.st_size<2||st.st_size>2*1024*1024){close(mapfd);nav_error_reply(fd,503,"map","MAP_FILE_INVALID","地图文件尺寸或类型无效");goto finish;}
        /* 保留最后一张完整地图可读，运动许可仍由RK判断。 */
        int file_age_known=!clock_gettime(CLOCK_REALTIME,&now)&&(st.st_mtim.tv_sec<now.tv_sec||(st.st_mtim.tv_sec==now.tv_sec&&st.st_mtim.tv_nsec<=now.tv_nsec));
        uint64_t file_age_ms=file_age_known?(uint64_t)(now.tv_sec-st.st_mtim.tv_sec)*1000+(now.tv_nsec-st.st_mtim.tv_nsec)/1000000:0;
        size_t length=(size_t)st.st_size,done=0;char *data=malloc(length);if(!data){close(mapfd);nav_error_reply(fd,503,"map","MAP_MEMORY_LIMIT","地图内存不足");goto finish;}
        while(done<length){ssize_t got=read(mapfd,data+done,length-done);if(got<=0)break;done+=(size_t)got;}close(mapfd);
        if(done==length){
         uint64_t publication=0;const char*key="\"published_us\":\"";for(size_t i=0;i+strlen(key)<length;i++)if(!memcmp(data+i,key,strlen(key))){const char*q=data+i+strlen(key);size_t remain=length-i-strlen(key);for(size_t j=0;j<remain&&j<20&&q[j]>='0'&&q[j]<='9';j++){if(publication>UINT64_MAX/10){publication=0;break;}publication=publication*10+(unsigned)(q[j]-'0');}break;}
         uint64_t current=millis()*1000;char age[320];uint64_t age_ms=publication&&current>=publication?(current-publication+999)/1000:PHONE_MAP_MS+1;
         int known=publication&&current>=publication;phone_log_event("map",200,!known?"MAP_AGE_UNKNOWN":age_ms>PHONE_MAP_MS?"MAP_RETAINED_OLD":"MAP_OK");snprintf(age,sizeof(age),"X-Map-Age-Ms: %llu\r\nX-Map-File-Age-Ms: %llu\r\nX-Map-Refresh-Warning: %s\r\nX-Map-Publication-Age-Known: %s\r\n",(unsigned long long)age_ms,(unsigned long long)file_age_ms,!known?"AGE_UNKNOWN":age_ms>PHONE_MAP_MS?"RETAINED_OLD":"CURRENT",known?"true":"false");reply(fd,200,"application/json; charset=utf-8",data,length,age);
        }else nav_error_reply(fd,503,"map","MAP_READ_FAILED","读取地图失败");free(data);goto finish;
    }
    if(!strcmp(method,"GET")&&!strcmp(path,"/api/video.h264")){
        if(!h264_enabled){error_reply(fd,503,"板端未启用 H.264，请安装配套运行包");goto finish;}
        if(!have_video_session||!h264_session_live(video_session)){error_reply(fd,401,"视频会话已失效");goto finish;}
        video_service_stream(fd,video_session,h264_session_live);goto finish;
    }
    if(!strcmp(method,"GET")&&!strcmp(path,"/api/frame.jpg")){
    pthread_mutex_lock(&mutex);size_t bytes=jpeg_size;uint8_t*copy=NULL;char extra[160];
        if(jpeg&&millis()>=jpeg_ms&&millis()-jpeg_ms<=1200){copy=malloc(bytes);if(copy)memcpy(copy,jpeg,bytes);}
        snprintf(extra,sizeof(extra),"X-Frame-Width: %u\r\nX-Frame-Height: %u\r\n",jpeg_width,jpeg_height);pthread_mutex_unlock(&mutex);
        if(copy){reply(fd,200,"image/jpeg",copy,bytes,extra);free(copy);}else reply(fd,204,"image/jpeg",NULL,0,NULL);goto finish;
    }
    if(strcmp(method,"POST")||!have_length){error_reply(fd,404,"接口不存在");goto finish;}
    if(!strcmp(path,"/api/nav/auto/prepare")||!strcmp(path,"/api/nav/auto")){
        if(!have_video_session||!h264_session_live(video_session)){nav_error_reply(fd,401,"auto","SESSION_EXPIRED","导航会话已失效，请重新连接");goto finish;}
        auto_switch_reply(fd,request+offset,video_session,!strcmp(path,"/api/nav/auto/prepare"));goto finish;
    }
    if(!strcmp(path,"/api/nav/object")||!strcmp(path,"/api/nav/tag")||!strcmp(path,"/api/nav/goal")||!strcmp(path,"/api/nav/start")||!strcmp(path,"/api/nav/stop")||!strcmp(path,"/api/nav/reset")){
        const char*nav_operation=path+9;
        if(!have_video_session||!h264_session_live(video_session)){nav_error_reply(fd,401,nav_operation,"SESSION_EXPIRED","导航会话已失效，请重新连接");goto finish;}
        phone_goal_message msg={0};uint64_t request_sequence=0;
        if(!strcmp(path,"/api/nav/object")?phone_object_navigation_json(request+offset,&msg,&request_sequence):phone_navigation_json(request+offset,&msg,&request_sequence,!strcmp(path,"/api/nav/tag"))){nav_error_reply(fd,400,nav_operation,"REQUEST_INVALID","目标格式错误");goto finish;}
        msg.request_id=request_sequence;
        unsigned operation=!strcmp(path,"/api/nav/object")?PHONE_NAV_OBJECT_GOAL:!strcmp(path,"/api/nav/tag")?PHONE_NAV_TAG_GOAL:!strcmp(path,"/api/nav/goal")?PHONE_NAV_GOAL:!strcmp(path,"/api/nav/start")?PHONE_NAV_START:!strcmp(path,"/api/nav/reset")?PHONE_NAV_RESET_MAP:PHONE_NAV_CANCEL;
        if(operation==PHONE_NAV_START&&!msg.sent_us){nav_error_reply(fd,400,nav_operation,"REQUEST_TIMING_REQUIRED","请使用配套App重新开始，旧请求不能接管车辆");goto finish;}
        uint64_t action_now=micros();if(msg.sent_us&&(msg.sent_us>action_now||action_now-msg.sent_us>PHONE_AUTONOMOUS_SESSION_MS*UINT64_C(1000))){nav_error_reply(fd,409,nav_operation,"REQUEST_EXPIRED","本次请求已过期，请重新操作");goto finish;}
        pthread_mutex_lock(&mutex);int status=200;const char*nav_code="NONE";
        if(!phone_control_live(&control,video_session,millis())){status=401;nav_code="SESSION_EXPIRED";}
        else{
          if(strcmp(navigation_request_session,video_session)){memcpy(navigation_request_session,video_session,33);navigation_request_sequence=0;}
          if(request_sequence<=navigation_request_sequence){status=409;nav_code="REQUEST_DUPLICATE";}
          else{navigation_request_sequence=request_sequence;
           if((operation==PHONE_NAV_START||operation==PHONE_NAV_RESET_MAP)&&(control.motion||claiming)){status=409;nav_code=claiming?"CONTROL_CLAIM_PENDING":"MANUAL_CONTROL_ACTIVE";}
           else if(navigation_send_locked(&msg,video_session,operation)){status=503;nav_code="NAV_IPC_UNAVAILABLE";}
           if(status==200&&operation==PHONE_NAV_START)control.autonomous=1;
           if(operation==PHONE_NAV_CANCEL){control.autonomous=0;wire_send(PHONE_STOP,0);control.motion=0;control.pending=0;control.stopped=1;}
          }
        }
        pthread_mutex_unlock(&mutex);
        if(status!=200){nav_error_reply(fd,status,nav_operation,nav_code,status==409?"请求冲突；请先松开手动方向，等待当前控制请求完成":status==401?"导航会话已失效，请重新连接":"导航服务尚未就绪，请查看板端状态");goto finish;}
        phone_log_event(nav_operation,200,"NAV_QUEUED");fprintf(stderr,"NAV_HTTP_QUEUED time_ms=%llu operation=%s seq=%llu\n",(unsigned long long)millis(),nav_operation,(unsigned long long)request_sequence);
        if(operation==PHONE_NAV_TAG_GOAL||operation==PHONE_NAV_OBJECT_GOAL){char body[192];int count=snprintf(body,sizeof(body),"{\"queued\":true,\"request_id\":\"%llu\",\"%s\":\"%llu\"}",(unsigned long long)request_sequence,operation==PHONE_NAV_OBJECT_GOAL?"object_request_us":"vision_request_us",(unsigned long long)msg.sent_us);reply(fd,200,"application/json",body,(size_t)count,NULL);}
        else{char body[96];int count=snprintf(body,sizeof(body),"{\"queued\":true,\"request_id\":\"%llu\"}",(unsigned long long)request_sequence);reply(fd,200,"application/json",body,(size_t)count,NULL);}goto finish;
    }
    if(!strcmp(path,"/api/heartbeat")){
        if(!have_video_session){nav_error_reply(fd,401,"heartbeat","SESSION_EXPIRED","缺少查看会话，请重新连接");goto finish;}
        heartbeat_reply(fd,request+offset,video_session);goto finish;
    }
    if(!strcmp(path,"/api/session")){
        char session[33],body[512];uint32_t wire_token;if(new_session(session,&wire_token)){phone_log_event("session",500,"SESSION_ENTROPY_FAILED");error_reply(fd,500,"无法建立会话");goto finish;}
        pthread_mutex_lock(&mutex);uint64_t now=millis();if(phone_control_live(&control,control.session,now)&&!control.stopped){pthread_mutex_unlock(&mutex);phone_log_event("session",409,"SESSION_CONTROL_ACTIVE");error_reply(fd,409,"已有遥控会话，请先停车或等待断开");goto finish;}
        int had_wire=control.has_session;uint32_t wire_sequence=control.wire_sequence;
        if(control.has_session){uint64_t revoke=0;(void)phone_control_fence_revoke(control_directory,&revoke);phone_goal_message old={0};navigation_send_locked(&old,control.session,PHONE_NAV_OVERRIDE);}
        if(had_wire)wire_token=control.wire_token;
        wire_send(PHONE_STOP,0);wire_sequence=control.wire_sequence;
        phone_control_session(&control,session,wire_token,now);control.wire_sequence=wire_sequence;
        claiming=!simulate;claim_after=now+(had_wire?0:600);claim_first_sent=0;
        pthread_mutex_unlock(&mutex);
        phone_log_event("session",200,"SESSION_CREATED");int n=snprintf(body,sizeof(body),"{\"session\":\"%s\",\"protocol\":1,\"watchdog_ms\":500,\"server_ms\":%llu,\"upstream_lease_ms\":3000,\"autonomous_lease_ms\":8000,\"view_ms\":10000,\"heartbeat_ms\":200,\"policy_revision\":5,\"config_id\":823198465,\"status_revision\":5}",session,(unsigned long long)now);reply(fd,200,"application/json",body,(size_t)n,NULL);goto finish;
    }
    if(!strcmp(path,"/api/control")){
        char session[33];uint64_t seq=0,sent=0,hold=0;unsigned op=0,motion=0;
        if(command_json(request+offset,session,&seq,&op,&motion,&sent,&hold)){error_reply(fd,400,"控制数据格式错误，请使用配套App");goto finish;}
        pthread_mutex_lock(&mutex);uint64_t now=millis();
        if(op==PHONE_AUTO){pthread_mutex_unlock(&mutex);nav_error_reply(fd,409,"auto","LEGACY_AUTO_REMOVED","请使用配套App自动模式，旧FPGA自动命令已移除");goto finish;}
        int allowed=session_ready(now);unsigned previous=control.motion;
        int status=phone_control_accept_timed(&control,session,seq,op,motion,allowed,0,now,sent,hold);const char*code=status==401?"SESSION_EXPIRED":"NONE";
        if(status==409)code=final_fault()?"FINAL_MOTOR_FAULT":!control_transport_ready(now)?"CONTROL_LINK_LOST":(op==PHONE_HOLD&&(hold<=now||hold-sent>PHONE_HOLD_MS))?"INPUT_EXPIRED":"CONTROL_NOT_READY";
        if(status==200){
         int takeover=op==PHONE_STOP||op==PHONE_MANUAL||(op==PHONE_HOLD&&!previous);
         uint64_t revocation=0;if(takeover&&phone_control_fence_revoke(control_directory,&revocation)){
          /* 文件系统屏障不可用时仍发送实际STOP。 */
          wire_send(PHONE_STOP,0);control.motion=0;control.pending=0;control.stopped=1;if(op!=PHONE_STOP){status=503;code="REVOCATION_UNAVAILABLE";}
         }
         if(status==200){phone_goal_message msg={0};navigation_send_locked(&msg,session,op==PHONE_PING?PHONE_NAV_HEARTBEAT:PHONE_NAV_OVERRIDE);}
        }
        if(status==200&&(op==PHONE_STOP||op==PHONE_MANUAL)){
         if(op==PHONE_MANUAL&&!simulate){claiming=1;claim_after=now;claim_first_sent=0;}
         wire_send(op,0);control.pending=0;if(simulate){sensor.auto_mode=0;sensor.actual=0;sensor.remote_flags=control.has_session?3:0;}
        }
        if(status==200&&op==PHONE_HOLD)dispatch_control_locked(now);
        if(status==409)dispatch_control_locked(now);
        phone_log_event("control",status,status==200?"CONTROL_OK":code);
        if(status!=200&&now-last_control_log>=1000){last_control_log=now;fprintf(stderr,"CONTROL_HTTP_ERROR status=%d code=%s server_rx_age_ms=%llu input_remaining_ms=%llu\n",status,code,(unsigned long long)(now>=control.last_rx_ms?now-control.last_rx_ms:UINT64_MAX),(unsigned long long)(hold>now?hold-now:0));}
        const char*message=status==401?"控制会话已失效":final_fault()?"电机最终故障":!control_transport_ready(now)?"真实控制链路失联或编码器未初始化":"等待控制握手或本次操作已过期";
        pthread_mutex_unlock(&mutex);
        if(status!=200){char body[256];int n=snprintf(body,sizeof(body),"{\"error\":\"%s\",\"code\":\"%s\"}",message,code);reply(fd,status,"application/json",body,(size_t)n,NULL);}
        else reply(fd,200,"application/json","{\"accepted\":true}",17,NULL);goto finish;
    }
    error_reply(fd,404,"接口不存在");
finish:
    close(fd);pthread_mutex_lock(&mutex);clients--;pthread_mutex_unlock(&mutex);return NULL;
}
/* S4自动配对兼容值，不是SSH凭据或会话令牌；显式文件及环境配置优先。
 */
static int pairing_from_environment(char out[65],const char*configured){
    if(!configured||!*configured)configured="123456";
    size_t n=strlen(configured);if(n<6||n>64)return -1;
    for(size_t i=0;i<n;i++)if(!((configured[i]>='A'&&configured[i]<='Z')||(configured[i]>='a'&&configured[i]<='z')||(configured[i]>='0'&&configured[i]<='9')||configured[i]=='_'||configured[i]=='-'))return -1;
    memcpy(out,configured,n+1);return 0;
}
int main(int argc,char**argv){
    const char*dir="/run/acg138-phone",*interface=NULL,*bind_ip="0.0.0.0",*token_file=NULL,*mpp_library="librockchip_mpp.so.0";unsigned port=8080;
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--dir")&&i+1<argc)dir=argv[++i];else if(!strcmp(argv[i],"--interface")&&i+1<argc)interface=argv[++i];
        else if(!strcmp(argv[i],"--bind")&&i+1<argc)bind_ip=argv[++i];else if(!strcmp(argv[i],"--port")&&i+1<argc)port=(unsigned)strtoul(argv[++i],NULL,10);
        else if(!strcmp(argv[i],"--width")&&i+1<argc)preview_width=(unsigned)strtoul(argv[++i],NULL,10);
        else if((!strcmp(argv[i],"--token-file")||!strcmp(argv[i],"--password-file"))&&i+1<argc)token_file=argv[++i];
        else if(!strcmp(argv[i],"--discovery-port")&&i+1<argc)discovery_port=(unsigned)strtoul(argv[++i],NULL,10);
        else if(!strcmp(argv[i],"--h264"))h264_requested=1;
        else if(!strcmp(argv[i],"--mpp-library")&&i+1<argc)mpp_library=argv[++i];
        else if(!strcmp(argv[i],"--enable-control"))enabled=1;
        else if(!strcmp(argv[i],"--simulate"))simulate=1;else{fprintf(stderr,"Usage: %s [--dir DIR] [--bind IP] [--port 8080] [--discovery-port 37020|0] [--width 1280|1920] [--password-file FILE] [--enable-control --interface FPGA_ETH] [--simulate]\n",argv[0]);return 2;}}
    if(port<1||port>65535||discovery_port>65535||(preview_width!=640&&preview_width!=1280&&preview_width!=1920))return 2;
    http_port=port;
    if(simulate){if(strcmp(bind_ip,"127.0.0.1")){fprintf(stderr,"Simulation requires --bind 127.0.0.1; no hardware socket is opened\n");return 2;}enabled=1;}
    if(mkdir(dir,0700)&&errno!=EEXIST){perror("runtime directory");return 1;}
    if(snprintf(control_directory,sizeof(control_directory),"%s",dir)>=(int)sizeof(control_directory))return 2;
    if(snprintf(shared_path,sizeof(shared_path),"%s/frames.shm",dir)>=(int)sizeof(shared_path))return 2;
    if(snprintf(vision_tags_path,sizeof(vision_tags_path),"%s/apriltag.json",dir)>=(int)sizeof(vision_tags_path))return 2;
    if(snprintf(object_directory,sizeof(object_directory),"%s",dir)>=(int)sizeof(object_directory))return 2;
    if(snprintf(navigation_map_path,sizeof(navigation_map_path),"%s/navigation-map.json",dir)>=(int)sizeof(navigation_map_path))return 2;
    if(snprintf(navigation_goal_path,sizeof(navigation_goal_path),"%s/navigation-goal.sock",dir)>=(int)sizeof(navigation_goal_path))return 2;
    if(token_file){FILE*f=fopen(token_file,"r");if(!f||!fgets(pairing,sizeof(pairing),f)){if(f)fclose(f);perror("password file");return 1;}fclose(f);pairing[strcspn(pairing,"\r\n")]=0;}
    else if(pairing_from_environment(pairing,getenv("ROVER_PASSWORD"))){fprintf(stderr,"AUTH_CONFIG_INVALID: supplied pairing configuration is invalid; credentials are never logged\n");return 2;}
    if(strlen(pairing)<6||strlen(pairing)>64)return 2;for(const char*p=pairing;*p;p++)if(!isalnum((unsigned char)*p)&&*p!='_'&&*p!='-')return 2;
    phone_log_open();
    if(enabled&&!simulate&&raw_open(interface)){perror("FPGA control interface");return 1;}
    listener=socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC,0);if(listener<0)return 1;int one=1;setsockopt(listener,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
    struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons((uint16_t)port)};if(inet_pton(AF_INET,bind_ip,&address.sin_addr)!=1||bind(listener,(struct sockaddr*)&address,sizeof(address))||listen(listener,8)){perror("HTTP listen");return 1;}
    if(discovery_port){
        discovery_fd=socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);
        struct timeval timeout={.tv_sec=0,.tv_usec=200000};
        struct sockaddr_in discovery_address=address;discovery_address.sin_port=htons((uint16_t)discovery_port);
        if(discovery_fd<0||setsockopt(discovery_fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))||bind(discovery_fd,(struct sockaddr*)&discovery_address,sizeof(discovery_address))){
            perror("Discovery unavailable (manual connection still works)");if(discovery_fd>=0)close(discovery_fd);discovery_fd=-1;
        }
    }
    signal(SIGTERM,stop_signal);signal(SIGINT,stop_signal);signal(SIGPIPE,SIG_IGN);
    printf("ACG138 phone service :%u\nCredentials: configured (not logged)\nDiscovery: UDP%u %s\nControl: %s\nPreview: %ux%u (video mode logged separately; no new YOLO inference)\n",port,discovery_port,discovery_fd>=0?"enabled":"disabled",enabled?"FPGA capability required":"monitor only",preview_width,preview_width*9/16);fflush(stdout);
    pthread_t voice_thread;int voice_started=pthread_create(&voice_thread,NULL,voice_mission_worker,NULL)==0;
    pthread_t watchdog,encoder,discovery;int discovery_started=discovery_fd>=0&&pthread_create(&discovery,NULL,discovery_thread,NULL)==0;
    pthread_create(&watchdog,NULL,watchdog_thread,NULL);pthread_create(&encoder,NULL,video_boot,(void*)mpp_library);
    while(running){int fd=accept4(listener,NULL,NULL,SOCK_CLOEXEC);if(fd<0){if(errno==EINTR)continue;if(!running)break;continue;}pthread_mutex_lock(&mutex);if(clients>=12){pthread_mutex_unlock(&mutex);close(fd);continue;}clients++;pthread_mutex_unlock(&mutex);pthread_t thread;if(pthread_create(&thread,NULL,client_thread,(void*)(intptr_t)fd)){close(fd);pthread_mutex_lock(&mutex);clients--;pthread_mutex_unlock(&mutex);}else pthread_detach(thread);}
    running=0;if(voice_started)pthread_join(voice_thread,NULL);pthread_join(watchdog,NULL);pthread_join(encoder,NULL);if(h264_enabled)video_service_stop();if(discovery_started)pthread_join(discovery,NULL);if(discovery_fd>=0)close(discovery_fd);close(listener);if(raw_fd>=0)close(raw_fd);return 0;
}
