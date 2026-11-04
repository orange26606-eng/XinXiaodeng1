#ifndef W1_REAL_TIME_IO_H
#define W1_REAL_TIME_IO_H
#include "lab_real_time.h"
/* 本机缓存不是录制文件，使用规范固定字节及代次、CRC。 */
lab_status lab_real_odom_cache_read(const char *path,lab_real_odom_time *out);
lab_status lab_real_odom_cache_create(const char *new_path,int *fd);
lab_status lab_real_odom_cache_write(int fd,const lab_real_odom_time *cache);
#endif
