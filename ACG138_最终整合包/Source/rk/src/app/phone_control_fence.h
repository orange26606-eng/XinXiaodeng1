#ifndef ACG_PHONE_CONTROL_FENCE_H
#define ACG_PHONE_CONTROL_FENCE_H
/* 私有目录中的认证服务端撤销屏障，只撤销不授予运动。RV01版本1固定32字节，代次位于小端偏移8，原revoked_us在16，24至27保留，28为CRC32；缺失按初始代次0，损坏则禁止控制。手机服务互斥锁串行写入，读取者不持有文件数据。


 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
static inline uint32_t phone_fence_crc(const unsigned char*p,size_t n){uint32_t c=UINT32_MAX;while(n--){c^=*p++;for(unsigned b=0;b<8;b++)c=(c>>1)^(UINT32_C(0xedb88320)&(uint32_t)-(int32_t)(c&1));}return ~c;}
static inline uint32_t phone_fence_u32(const unsigned char*p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
/* 0有效，1表示初始文件不存在，-1表示损坏、不安全或不可用。 */
static inline int phone_control_fence_read_state(const char*dir,uint64_t*out,uint64_t*revoked_us){
 char path[1024];if(!dir||!out||snprintf(path,sizeof(path),"%s/navigation-revoke",dir)>=(int)sizeof(path))return -1;
 int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW);if(fd<0){if(errno==ENOENT){*out=0;if(revoked_us)*revoked_us=0;return 1;}return -1;}
 unsigned char p[32];struct stat s;int ok=fstat(fd,&s)==0&&S_ISREG(s.st_mode)&&s.st_size==32&&(s.st_mode&0022)==0;
 ssize_t n=ok?read(fd,p,sizeof(p)):-1;close(fd);if(n!=32||memcmp(p,"RV01",4)||phone_fence_u32(p+4)!=1||phone_fence_crc(p,28)!=phone_fence_u32(p+28))return -1;
 for(unsigned i=24;i<28;i++)if(p[i])return -1;if(revoked_us)*revoked_us=phone_fence_u32(p+16)|(uint64_t)phone_fence_u32(p+20)<<32;*out=phone_fence_u32(p+8)|(uint64_t)phone_fence_u32(p+12)<<32;return *out?0:-1;
}
static inline int phone_control_fence_read(const char*dir,uint64_t*out){return phone_control_fence_read_state(dir,out,NULL);}
static inline int phone_control_fence_revoke(const char*dir,uint64_t*out){
 uint64_t gen=0;int old=phone_control_fence_read(dir,&gen);if(old<0||gen==UINT64_MAX)return -1;gen++;
 char path[1024],temp[1088];if(!dir||snprintf(path,sizeof(path),"%s/navigation-revoke",dir)>=(int)sizeof(path)||snprintf(temp,sizeof(temp),"%s.%lu.tmp",path,(unsigned long)getpid())>=(int)sizeof(temp))return -1;
 struct timespec now;if(clock_gettime(CLOCK_MONOTONIC,&now))return -1;uint64_t at=(uint64_t)now.tv_sec*1000000+(uint64_t)now.tv_nsec/1000;
 unsigned char p[32]={0};memcpy(p,"RV01",4);p[4]=1;for(unsigned i=0;i<8;i++){p[8+i]=(unsigned char)(gen>>(8*i));p[16+i]=(unsigned char)(at>>(8*i));}uint32_t crc=phone_fence_crc(p,28);for(unsigned i=0;i<4;i++)p[28+i]=(unsigned char)(crc>>(8*i));
 int fd=open(temp,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)return -1;
 int ok=write(fd,p,sizeof(p))==32;int closed=close(fd);if(!ok||closed||rename(temp,path)){unlink(temp);return -1;}if(out)*out=gen;return 0;
}
#endif
