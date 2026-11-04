#define _POSIX_C_SOURCE 200809L
#include "phone_goal_socket.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#ifdef PHONE_GOAL_SOCKET_TEST
#include "goal_socket_posix_fixture.h"
#else
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/un.h>
#endif
#define GOAL_NAME "navigation-goal.sock"
#define OWNER_NAME ".navigation-goal.owner"
#define LOCK_NAME ".navigation-goal.lock"
typedef struct owner_record {uint64_t pid,birth,device,inode;} owner_record;
static int safe_regular(const struct stat*s){return S_ISREG(s->st_mode)&&s->st_uid==geteuid()&&!(s->st_mode&0077)&&s->st_nlink==1;}
/* 1表示读到存活身份，0表示已退出或僵尸进程，-1表示无法确定。 */
static int process_birth(uint64_t pid,uint64_t*out){
#ifdef PHONE_GOAL_SOCKET_TEST
 return fixture_process_birth(pid,out);
#else
 if(pid<2||pid>INT_MAX)return -1;char path[64],line[4096];snprintf(path,sizeof(path),"/proc/%llu/stat",(unsigned long long)pid);
 FILE*f=fopen(path,"r");if(!f)return errno==ENOENT?0:-1;size_t n=fread(line,1,sizeof(line)-1,f);int bad=ferror(f);fclose(f);if(bad||!n||n==sizeof(line)-1)return -1;line[n]=0;
 char*end=strrchr(line,')');if(!end||end[1]!=' '||!end[2])return -1;char*p=end+2;if(*p=='Z'||*p=='X')return 0;
 for(unsigned field=3;field<22;field++){p=strchr(p,' ');if(!p)return -1;while(*p==' ')p++;}
 errno=0;char*tail=NULL;unsigned long long v=strtoull(p,&tail,10);if(errno||tail==p||!v||(*tail!=' '&&*tail!='\n'&&*tail))return -1;*out=(uint64_t)v;return 1;
#endif
}
static int owner_read(int dir,owner_record*out){
 int fd=openat(dir,OWNER_NAME,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);if(fd<0)return 0;struct stat s;char data[192];ssize_t n=-1;
 if(!fstat(fd,&s)&&safe_regular(&s)&&s.st_size>0&&s.st_size<(long)sizeof(data))n=read(fd,data,sizeof(data)-1);close(fd);if(n<=0||n!=s.st_size)return 0;data[n]=0;
 for(ssize_t i=5;i<n;i++)if((data[i]<'0'||data[i]>'9')&&data[i]!=' '&&data[i]!='\n')return 0;
 unsigned long long pid,birth,dev,ino;int used=0;if(sscanf(data,"PGS1 %llu %llu %llu %llu\n%n",&pid,&birth,&dev,&ino,&used)!=4||used!=n||pid<2||pid>INT_MAX||!birth||!ino)return 0;
 *out=(owner_record){pid,birth,dev,ino};return 1;
}
static int node_identity(const struct stat*s,uint64_t dev,uint64_t ino){return S_ISSOCK(s->st_mode)&&s->st_uid==geteuid()&&(uint64_t)s->st_dev==dev&&(uint64_t)s->st_ino==ino;}
static int same_node(const struct stat*s,uint64_t dev,uint64_t ino){return node_identity(s,dev,ino)&&!(s->st_mode&0077);}
void phone_goal_socket_close(phone_goal_socket*p){if(!p)return;
 if(p->fd>=0){close(p->fd);p->fd=-1;}
 if(p->dirfd>=0&&p->inode){struct stat s;
  if(!fstatat(p->dirfd,GOAL_NAME,&s,AT_SYMLINK_NOFOLLOW)&&node_identity(&s,p->device,p->inode)){
   if(!unlinkat(p->dirfd,GOAL_NAME,0)){owner_record o;if(owner_read(p->dirfd,&o)&&o.pid==p->pid&&o.birth==p->birth&&o.device==p->device&&o.inode==p->inode)(void)unlinkat(p->dirfd,OWNER_NAME,0);}
  }
 }
 if(p->lockfd>=0){close(p->lockfd);p->lockfd=-1;}if(p->dirfd>=0){close(p->dirfd);p->dirfd=-1;}p->inode=0;
}
int phone_goal_socket_open(phone_goal_socket*p,const char*dir,char*why,size_t cap){
 if(!p||!why||!cap)return -1;*p=(phone_goal_socket){.fd=-1,.dirfd=-1,.lockfd=-1};why[0]=0;
#define FAIL(reason) do{snprintf(why,cap,"%s",reason);phone_goal_socket_close(p);return -1;}while(0)
 if(!dir||dir[0]!='/')FAIL("PRIVATE_DIRECTORY_REQUIRED");
 struct sockaddr_un address={.sun_family=AF_UNIX};if(snprintf(address.sun_path,sizeof(address.sun_path),"%s/%s",dir,GOAL_NAME)>=(int)sizeof(address.sun_path))FAIL("SOCKET_PATH_TOO_LONG");
 p->dirfd=open(dir,O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW);struct stat ds;if(p->dirfd<0||fstat(p->dirfd,&ds)||!S_ISDIR(ds.st_mode)||ds.st_uid!=geteuid()||(ds.st_mode&0077))FAIL("UNSAFE_PRIVATE_DIRECTORY");
 p->lockfd=openat(p->dirfd,LOCK_NAME,O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);struct stat ls;if(p->lockfd<0||fstat(p->lockfd,&ls)||!safe_regular(&ls))FAIL("UNSAFE_OWNER_LOCK");
 if(flock(p->lockfd,LOCK_EX|LOCK_NB)){owner_record locked;if(owner_read(p->dirfd,&locked))fprintf(stderr,"PHONE_GOAL_OWNER_CONFLICT recorded_pid=%llu birth_ticks=%llu; stop original owner before retry\n",(unsigned long long)locked.pid,(unsigned long long)locked.birth);else fprintf(stderr,"PHONE_GOAL_OWNER_CONFLICT owner_record=UNKNOWN; inspect private directory, no takeover\n");FAIL("LIVE_OR_UNKNOWN_OWNER_LOCKED");}
 p->pid=(uint64_t)getpid();if(process_birth(p->pid,&p->birth)!=1)FAIL("SELF_IDENTITY_UNAVAILABLE");
 struct stat previous;if(!fstatat(p->dirfd,GOAL_NAME,&previous,AT_SYMLINK_NOFOLLOW)){
  if(!S_ISSOCK(previous.st_mode)||previous.st_uid!=geteuid()||(previous.st_mode&0077))FAIL("EXISTING_PATH_NOT_PRIVATE_SOCKET");
  owner_record o;if(!owner_read(p->dirfd,&o))FAIL("EXISTING_SOCKET_OWNER_UNKNOWN");
  if(!same_node(&previous,o.device,o.inode))FAIL("EXISTING_SOCKET_OWNER_INODE_MISMATCH");
  uint64_t birth=0;int identity=process_birth(o.pid,&birth);if(identity<0)FAIL("EXISTING_OWNER_IDENTITY_UNKNOWN");if(identity==1&&birth==o.birth){fprintf(stderr,"PHONE_GOAL_OWNER_CONFLICT verified_live_pid=%llu birth_ticks=%llu\n",(unsigned long long)o.pid,(unsigned long long)o.birth);FAIL("EXISTING_OWNER_STILL_ALIVE");}
  int probe=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(probe<0)FAIL("EXISTING_SOCKET_PROBE_FAILED");
  int connected=connect(probe,(struct sockaddr*)&address,sizeof(address));int probe_errno=errno;close(probe);
  if(!connected)FAIL("EXISTING_SOCKET_STILL_LIVE");if(probe_errno!=ECONNREFUSED)FAIL("EXISTING_SOCKET_PROBE_UNCERTAIN");
  struct stat again;if(fstatat(p->dirfd,GOAL_NAME,&again,AT_SYMLINK_NOFOLLOW)||!same_node(&again,o.device,o.inode))FAIL("EXISTING_SOCKET_CHANGED_DURING_CHECK");
  if(unlinkat(p->dirfd,GOAL_NAME,0))FAIL("STALE_SOCKET_REMOVE_FAILED");
  snprintf(why,cap,"RECOVERED_VERIFIED_DEAD_OWNER");
 }else if(errno!=ENOENT)FAIL("EXISTING_SOCKET_STAT_FAILED");
 p->fd=socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(p->fd<0||bind(p->fd,(struct sockaddr*)&address,sizeof(address)))FAIL("SOCKET_BIND_FAILED");
 struct stat own;if(fstatat(p->dirfd,GOAL_NAME,&own,AT_SYMLINK_NOFOLLOW)||!S_ISSOCK(own.st_mode)||own.st_uid!=geteuid())FAIL("NEW_SOCKET_IDENTITY_UNAVAILABLE");p->device=(uint64_t)own.st_dev;p->inode=(uint64_t)own.st_ino;
 if(fchmodat(p->dirfd,GOAL_NAME,0600,0))FAIL("NEW_SOCKET_PERMISSIONS_FAILED");
 int owner=openat(p->dirfd,OWNER_NAME,O_WRONLY|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);struct stat os;if(owner<0)FAIL("OWNER_RECORD_OPEN_FAILED");
 int valid=!fstat(owner,&os)&&safe_regular(&os);char data[192];int count=snprintf(data,sizeof(data),"PGS1 %llu %llu %llu %llu\n",(unsigned long long)p->pid,(unsigned long long)p->birth,(unsigned long long)p->device,(unsigned long long)p->inode);
 if(!valid||ftruncate(owner,0)||write(owner,data,(size_t)count)!=count){close(owner);FAIL("OWNER_RECORD_WRITE_FAILED");}if(close(owner))FAIL("OWNER_RECORD_CLOSE_FAILED");
 if(!why[0])snprintf(why,cap,"NEW_PRIVATE_SOCKET");return p->fd;
#undef FAIL
}
