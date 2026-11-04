#ifndef ACG_LAB_REAL_RUNTIME_CONTROL_H
#define ACG_LAB_REAL_RUNTIME_CONTROL_H
#include "lab_real_runtime.h"
/* 当前目标候选的实际就绪状态不依赖原因文字比较；AGAIN表示需下一次真实传感或导航tick，不是ARM许可。
 */
lab_status lab_real_runtime_start_ready(const lab_real_runtime*,uint64_t now_us);
/* 仅限定初次AUTO使能及首命令事务，之后正常行驶沿用原手机与命令租约。
 */
lab_status lab_real_runtime_start_deadline(lab_real_runtime*,uint64_t expires_us);
uint64_t lab_real_runtime_first_apply_us(const lab_real_runtime*);
#endif
