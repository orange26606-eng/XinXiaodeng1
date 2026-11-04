#ifndef ACG_PHONE_GOAL_SOCKET_H
#define ACG_PHONE_GOAL_SOCKET_H
#include <stdint.h>
#include <stddef.h>
typedef struct phone_goal_socket {
 int fd,dirfd,lockfd;
 uint64_t device,inode,pid,birth;
} phone_goal_socket;
/* 探测现有AF_UNIX端点时仅检查记录、进程、inode及ECONNREFUSED；确认旧属主已退出后清理端点。 */


int phone_goal_socket_open(phone_goal_socket*,const char*,char*,size_t);
void phone_goal_socket_close(phone_goal_socket*);
#endif
