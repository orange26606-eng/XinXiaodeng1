#define _GNU_SOURCE
#include "lab_real_time_io.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
static int at(int fd,uint8_t *p,size_t n,off_t where,int writing){size_t k=0;while(k<n){ssize_t a=writing?pwrite(fd,p+k,n-k,where+(off_t)k):pread(fd,p+k,n-k,where+(off_t)k);if(a<0&&errno==EINTR)continue;if(a<=0)return 0;k+=(size_t)a;}return 1;}
lab_status lab_real_odom_cache_read(const char *path,lab_real_odom_time *out){if(!path||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);if(fd<0)return LAB_ERR_IO;struct stat st;lab_status s=LAB_AGAIN;if(fstat(fd,&st)||!S_ISREG(st.st_mode)||st.st_size!=256){close(fd);return LAB_ERR_FORMAT;}uint8_t before[4],after[4],all[256];
 for(unsigned i=0;i<5;++i){if(!at(fd,before,4,180,0)||!at(fd,all,256,0,0)||!at(fd,after,4,180,0)){s=LAB_ERR_TRUNCATED;break;}if(memcmp(before,after,4)||memcmp(before,all+180,4)||(before[3]&1))continue;s=lab_real_odom_time_decode(out,all,256);if(s==LAB_OK||s==LAB_ERR_VERSION)break;}
 close(fd);return s;
}
lab_status lab_real_odom_cache_create(const char *path,int *fd){if(!path||!fd)return LAB_ERR_ARGUMENT;*fd=open(path,O_RDWR|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(*fd<0)return LAB_ERR_IO;if(ftruncate(*fd,256)){close(*fd);*fd=-1;return LAB_ERR_IO;}return LAB_OK;}
lab_status lab_real_odom_cache_write(int fd,const lab_real_odom_time *c){uint8_t b[256];lab_status s=lab_real_odom_time_encode(b,c);if(s!=LAB_OK)return s;uint8_t odd[4];memcpy(odd,b+180,4);odd[3]|=1;
 if(fd<0||!at(fd,odd,4,180,1)||!at(fd,b,180,0,1)||!at(fd,b+184,72,184,1)||!at(fd,b+180,4,180,1))return LAB_ERR_IO;return LAB_OK;
}
