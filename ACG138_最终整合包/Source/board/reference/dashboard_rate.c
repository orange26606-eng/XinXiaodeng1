/* 按单调时钟统计有效UDP姿态包的接收率。 */

#include "dashboard_rate.h"
void dashboard_rate_init(struct dashboard_rate *r,uint64_t count,uint64_t now)
{ r->start_ms=now; r->start_count=count; r->hz=0; r->ready=0; }
void dashboard_rate_update(struct dashboard_rate *r,uint64_t count,uint64_t now)
{
    if(now<r->start_ms || count<r->start_count) { dashboard_rate_init(r,count,now); return; }
    uint64_t elapsed=now-r->start_ms;
    if(elapsed>=1000) {
        r->hz=(double)(count-r->start_count)*1000.0/(double)elapsed;
        r->start_ms=now; r->start_count=count; r->ready=1;
    }
}
