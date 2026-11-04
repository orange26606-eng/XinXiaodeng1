#ifndef W1_REAL_NAV_MATH_H
#define W1_REAL_NAV_MATH_H
/* 内部计算上下文，与 v1 事件及录制格式分开。数据来源保存在外层 REAL 上下文中。 */
#include "lab_types.h"
#define W1_REAL_CARRIER UINT32_C(0x524c)
#define nav_open w1_math_open
#define nav_close w1_math_close
#define nav_step w1_math_step
#define nav_set_map w1_math_set_map
#define nav_set_goal w1_math_set_goal
#define nav_cancel w1_math_cancel
#define nav_get_view w1_math_get_view
#define nav_config_default w1_math_config_default
#define nav_state_text w1_math_state_text
#define nav_reason_text w1_math_reason_text
#define nav_phase_text w1_math_phase_text
#include "nav_private.h"
int w1_real_carrier_context(const lab_context *context);
int w1_real_carrier_event(const lab_event *event);
#endif
