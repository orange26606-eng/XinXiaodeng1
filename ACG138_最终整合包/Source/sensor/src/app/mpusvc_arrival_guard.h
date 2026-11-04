#ifndef MPUSVC_ARRIVAL_GUARD_H
#define MPUSVC_ARRIVAL_GUARD_H
#include <stdint.h>
static inline int mpusvc_guard_arrival(uint64_t previous,uint64_t *now){
 if(!previous||*now>=previous)return 0;
 /* 丢弃映射后时间倒退的数据并计入服务错误预算，保留可信样本的原时间戳和年龄。 */
 return -1;
}
#endif
