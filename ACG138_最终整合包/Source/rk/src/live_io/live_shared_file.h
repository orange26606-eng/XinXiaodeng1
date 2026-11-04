#ifndef ACG_LIVE_SHARED_FILE_CANDIDATE_H
#define ACG_LIVE_SHARED_FILE_CANDIDATE_H
#include "live_sensor.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct li_file_identity {uint64_t device,inode;size_t native_bytes;} li_file_identity;
/* Linux和Buildroot只读常规文件适配，不使用mmap，避免文件截断导致SIGBUS；不创建共享文件或套接字。最多尝试五次序列锁读取，头部按现有生产者本机ABI布局，不是磁盘编码；路径替换需创建新生产者代次。


 */
lab_status li_shared_file_read(const char *path,struct odom_shared *out,li_file_identity *identity);
#ifdef __cplusplus
}
#endif
#endif
