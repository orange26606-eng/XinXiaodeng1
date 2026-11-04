#ifndef ACG_CADENCE_POLICY_H
#define ACG_CADENCE_POLICY_H
#include <stdint.h>
#include "continuous_policy.h"
typedef struct cadence_budget {uint32_t streak,reduced,good;uint64_t warnings,events;}cadence_budget;
/* 实际耗时预算仅供负载调整，单次或连续三次慢结果不撤销任务，而是选择更低计算量控制流程。
 */
static inline void cadence_observe(cadence_budget*b,uint64_t elapsed){
 if(elapsed>CONT_CONTROL_WARN_US)++b->warnings;
 if(elapsed>CONT_CONTROL_SOFT_US){b->good=0;if(b->streak<3)++b->streak;if(b->streak==3&&!b->reduced){b->reduced=1;++b->events;}}
 else{b->streak=0;if(b->reduced){if(elapsed<50000){if(++b->good>=20){b->reduced=0;b->good=0;}}else b->good=0;}}
}
#endif
