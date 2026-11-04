#ifndef ACG_S5_PERSISTENT_LOG_H
#define ACG_S5_PERSISTENT_LOG_H
/* 固定双文件诊断写入器，只有工作线程访问Flash。接口不接受HTTP请求、密码、会话令牌或任意输入。
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#ifdef _WIN32
#include <io.h>
#define s5_sync_fd _commit
#define s5_file_fd _fileno
#else
#include <unistd.h>
#define s5_sync_fd fsync
#define s5_file_fd fileno
#endif
#ifndef S5_LOG_SOFTWARE_POLICY
#define S5_LOG_SOFTWARE_POLICY 5u
#endif
#define S5_LOG_FILE_BYTES (2u*1024u*1024u)
#define S5_LOG_RECORD_BYTES 16384u
#define S5_LOG_QUEUE 16u
typedef struct s5_log_disk {FILE*file;char directory[512];size_t bytes;uint64_t records,syncs;int failed;} s5_log_disk;
static int s5_log_disk_open(s5_log_disk*d,const char*directory){
 memset(d,0,sizeof(*d));if(!directory||strlen(directory)>sizeof(d->directory)-32)return 0;
 snprintf(d->directory,sizeof(d->directory),"%s",directory);char path[560];snprintf(path,sizeof(path),"%s/s5-diagnostics.0.jsonl",directory);
 d->file=fopen(path,"ab+");if(!d->file)return 0;
 if(fseek(d->file,0,SEEK_END)||ftell(d->file)<0){fclose(d->file);d->file=NULL;return 0;}
 d->bytes=(size_t)ftell(d->file);return 1;
}
static int s5_log_disk_sync(s5_log_disk*d){if(!d->file)return 0;if(fflush(d->file)||s5_sync_fd(s5_file_fd(d->file))){d->failed=1;return 0;}d->syncs++;return 1;}
static int s5_log_disk_write(s5_log_disk*d,const char*record,size_t n){
 if(!d||!d->file||!record||!n||n>S5_LOG_RECORD_BYTES||n>S5_LOG_FILE_BYTES)return 0;
 if(d->bytes+n>S5_LOG_FILE_BYTES){
  char from[560],to[560];if(!s5_log_disk_sync(d))return 0;
  if(fclose(d->file)){d->file=NULL;d->failed=1;return 0;}d->file=NULL;
  snprintf(from,sizeof(from),"%s/s5-diagnostics.0.jsonl",d->directory);snprintf(to,sizeof(to),"%s/s5-diagnostics.1.jsonl",d->directory);
  if(remove(to)&&errno!=ENOENT){d->failed=1;return 0;}
  if(rename(from,to)){d->failed=1;return 0;}d->file=fopen(from,"wb");d->bytes=0;
  if(!d->file){d->failed=1;return 0;}
 }
 if(fwrite(record,1,n,d->file)!=n){d->failed=1;return 0;}d->bytes+=n;d->records++;return 1;
}
static void s5_log_disk_close(s5_log_disk*d){if(d&&d->file){s5_log_disk_sync(d);fclose(d->file);d->file=NULL;}}

#if defined(__linux__) && !defined(S5_LOG_DISK_ONLY)
#include <pthread.h>
#include <sys/stat.h>
#include <sys/vfs.h>
#include <fcntl.h>
#define S5_TMPFS_MAGIC 0x01021994
typedef struct s5_log_record {size_t bytes;int urgent;char text[S5_LOG_RECORD_BYTES];} s5_log_record;
typedef struct s5_log {
 pthread_t thread;pthread_mutex_t mutex;pthread_cond_t ready;s5_log_disk disk;
 s5_log_record records[S5_LOG_QUEUE];unsigned head,count;int stop;
 uint64_t dropped,submitted,written;char fs_sha256[65];
} s5_log;
static uint64_t s5_log_clock(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;return (uint64_t)t.tv_sec*1000000u+t.tv_nsec/1000u;}
static void*s5_log_worker(void*user){s5_log*l=user;uint64_t last_sync=s5_log_clock();for(;;){
 s5_log_record item;int have=0,done=0;pthread_mutex_lock(&l->mutex);
 while(!l->count&&!l->stop){struct timespec until;clock_gettime(CLOCK_REALTIME,&until);until.tv_sec++;if(pthread_cond_timedwait(&l->ready,&l->mutex,&until)==ETIMEDOUT)break;}
 if(l->count){item=l->records[l->head];l->head=(l->head+1)%S5_LOG_QUEUE;l->count--;have=1;}done=l->stop&&!l->count;pthread_mutex_unlock(&l->mutex);
 if(have&&s5_log_disk_write(&l->disk,item.text,item.bytes))l->written++;
 uint64_t now=s5_log_clock();if((have&&item.urgent)||now-last_sync>=1000000u){s5_log_disk_sync(&l->disk);last_sync=now;}
 if(done)break;
 }s5_log_disk_close(&l->disk);return NULL;}
static s5_log*s5_log_open(const char*directory,const char*fs_sha){
 if(!directory||directory[0]!='/')return NULL;
 if(mkdir(directory,0700)&&errno!=EEXIST){fprintf(stderr,"S5_LOG_UNAVAILABLE reason=MKDIR errno=%d\n",errno);return NULL;}
 struct stat st;struct statfs fs;
 if(lstat(directory,&st)||!S_ISDIR(st.st_mode)||st.st_uid!=geteuid()||(st.st_mode&0077)||statfs(directory,&fs)||fs.f_type==S5_TMPFS_MAGIC){fprintf(stderr,"S5_LOG_UNAVAILABLE reason=NOT_PRIVATE_FLASH_DIRECTORY\n");return NULL;}
 s5_log*l=calloc(1,sizeof(*l));if(!l)return NULL;
 snprintf(l->fs_sha256,sizeof(l->fs_sha256),"UNAVAILABLE");
 if(fs_sha&&strlen(fs_sha)==64&&strspn(fs_sha,"0123456789abcdefABCDEF")==64)snprintf(l->fs_sha256,sizeof(l->fs_sha256),"%s",fs_sha);
 if(!s5_log_disk_open(&l->disk,directory)){free(l);return NULL;}
 if(pthread_mutex_init(&l->mutex,NULL)){s5_log_disk_close(&l->disk);free(l);return NULL;}
 if(pthread_cond_init(&l->ready,NULL)){pthread_mutex_destroy(&l->mutex);s5_log_disk_close(&l->disk);free(l);return NULL;}
 if(pthread_create(&l->thread,NULL,s5_log_worker,l)){pthread_cond_destroy(&l->ready);pthread_mutex_destroy(&l->mutex);s5_log_disk_close(&l->disk);free(l);return NULL;}return l;
}
static int s5_log_submit(s5_log*l,uint64_t now,const char*event,const char*json,size_t n,int urgent){
 if(!l||!event||!json||!n||json[0]!='{'||json[n-1]!='}')return 0;
 /* 控制路径只提交日志，刷盘由工作线程处理。 */
 if(pthread_mutex_trylock(&l->mutex))return 0;
 if(l->stop||l->count==S5_LOG_QUEUE){l->dropped++;pthread_mutex_unlock(&l->mutex);return 0;}
 for(const char*c=event;*c;c++)if(!((*c>='A'&&*c<='Z')||*c=='_'||(*c>='0'&&*c<='9'))){pthread_mutex_unlock(&l->mutex);return 0;}
 unsigned slot=(l->head+l->count)%S5_LOG_QUEUE;s5_log_record*r=&l->records[slot];time_t wall=time(NULL);struct tm utc;char stamp[40];gmtime_r(&wall,&utc);strftime(stamp,sizeof(stamp),"%Y-%m-%dT%H:%M:%SZ",&utc);
 int size=snprintf(r->text,sizeof(r->text),"{\"monotonic_us\":\"%llu\",\"utc\":\"%s\",\"event\":\"%s\",\"policy_revision\":%u,\"config_id\":\"0x31110107\",\"fs_sha256\":\"%s\",\"queue_drops\":%llu,\"snapshot\":%.*s}\n",(unsigned long long)now,stamp,event,(unsigned)S5_LOG_SOFTWARE_POLICY,l->fs_sha256,(unsigned long long)l->dropped,(int)n,json);
 if(size<0||(size_t)size>=sizeof(r->text)){l->dropped++;pthread_mutex_unlock(&l->mutex);return 0;}
 r->bytes=(size_t)size;r->urgent=urgent;l->submitted++;l->count++;pthread_cond_signal(&l->ready);pthread_mutex_unlock(&l->mutex);return 1;
}
static void s5_log_close(s5_log*l){if(!l)return;pthread_mutex_lock(&l->mutex);l->stop=1;pthread_cond_signal(&l->ready);pthread_mutex_unlock(&l->mutex);pthread_join(l->thread,NULL);pthread_cond_destroy(&l->ready);pthread_mutex_destroy(&l->mutex);free(l);}
#endif
#endif
