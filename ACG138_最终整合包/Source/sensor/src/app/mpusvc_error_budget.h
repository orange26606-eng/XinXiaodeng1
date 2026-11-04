#ifndef MPUSVC_ERROR_BUDGET_H
#define MPUSVC_ERROR_BUDGET_H
#include <stdint.h>
#include <string.h>
#define MPUSVC_TIMING_CONSECUTIVE_LIMIT 200u
#define MPUSVC_TIMING_WINDOW_LIMIT 500u
#define MPUSVC_TIMING_WINDOW_US UINT64_C(10000000)
typedef struct mpusvc_error_budget {
 uint64_t times[MPUSVC_TIMING_WINDOW_LIMIT],total,last_check;
 unsigned head,count,consecutive;
} mpusvc_error_budget;
static inline void mpusvc_error_budget_prune(mpusvc_error_budget*b,uint64_t now){
 while(b->count&&now>=b->times[b->head]&&now-b->times[b->head]>=MPUSVC_TIMING_WINDOW_US){
  b->head=(b->head+1)%MPUSVC_TIMING_WINDOW_LIMIT;--b->count;
 }
}
/* 错误预算由接收线程更新；拒收样本保持原测量时间。 */

static inline int mpusvc_error_budget_event(mpusvc_error_budget*b,uint64_t now,int timing_error){
 if(!b||!now||(b->last_check&&now<b->last_check))return -1;
 b->last_check=now;mpusvc_error_budget_prune(b,now);
 if(!timing_error){b->consecutive=0;return 0;}
 ++b->total;++b->consecutive;
 if(b->count<MPUSVC_TIMING_WINDOW_LIMIT){b->times[(b->head+b->count)%MPUSVC_TIMING_WINDOW_LIMIT]=now;++b->count;}
 return b->consecutive>=MPUSVC_TIMING_CONSECUTIVE_LIMIT||b->count>=MPUSVC_TIMING_WINDOW_LIMIT?-1:0;
}
#endif
