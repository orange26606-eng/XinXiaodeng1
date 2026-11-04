#define _GNU_SOURCE
#include "live_shared_file.h"
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int read_at(int fd,void *out,size_t n,off_t offset) {
 size_t have=0;
 while(have<n){ssize_t k=pread(fd,(char*)out+have,n-have,offset+(off_t)have);
  if(k<0&&errno==EINTR)continue;if(k<=0)return -1;have+=(size_t)k;}
 return 0;
}
lab_status li_shared_file_read(const char *path,struct odom_shared *out,li_file_identity *id) {
 if(out)memset(out,0,sizeof(*out));if(id)memset(id,0,sizeof(*id));
 if(!path||!*path||!out||!id)return LAB_ERR_ARGUMENT;
 int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);if(fd<0)return LAB_ERR_IO;
 struct stat st;lab_status status=LAB_AGAIN;
 if(fstat(fd,&st)||!S_ISREG(st.st_mode)){close(fd);return LAB_ERR_FORMAT;}
 if(st.st_size!=(off_t)sizeof(*out)){close(fd);return LAB_ERR_VERSION;}
 for(unsigned i=0;i<5;++i){
  uint32_t before,after;struct odom_shared copy;
  if(read_at(fd,&before,4,(off_t)offsetof(struct odom_shared,sequence))){status=LAB_ERR_TRUNCATED;break;}
  if(before&1)continue;
  if(read_at(fd,&copy,sizeof(copy),0)||read_at(fd,&after,4,(off_t)offsetof(struct odom_shared,sequence))){status=LAB_ERR_TRUNCATED;break;}
  if(before!=after||before!=copy.sequence||(after&1))continue;
  if(copy.magic!=ODOM_MAGIC||copy.version!=1||copy.bytes!=sizeof(copy)){status=LAB_ERR_VERSION;break;}
  *out=copy;*id=(li_file_identity){(uint64_t)st.st_dev,(uint64_t)st.st_ino,sizeof(copy)};status=LAB_OK;break;
 }
 close(fd);return status;
}
