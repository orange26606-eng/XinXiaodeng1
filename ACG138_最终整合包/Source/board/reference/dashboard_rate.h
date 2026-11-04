/* 统计有效姿态报文的实际接收率。 */
#ifndef DASHBOARD_RATE_H
#define DASHBOARD_RATE_H
#include <stdint.h>
struct dashboard_rate { uint64_t start_ms, start_count; double hz; int ready; };
void dashboard_rate_init(struct dashboard_rate *r, uint64_t count, uint64_t now);
void dashboard_rate_update(struct dashboard_rate *r, uint64_t count, uint64_t now);
#endif
