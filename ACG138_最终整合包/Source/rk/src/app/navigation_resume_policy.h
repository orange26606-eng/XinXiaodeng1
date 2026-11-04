#ifndef ACG_NAVIGATION_RESUME_POLICY_H
#define ACG_NAVIGATION_RESUME_POLICY_H
#include "lab_real_runtime.h"
#include <string.h>
/* 仅已有认证运动意图可自动恢复，最终重试耗尽后需要新的显式START；STOP可清除FPGA故障，但不自动建立新重试预算。

 */
static inline int navigation_auto_resume_allowed(const lab_real_runtime_view*v,int phone_intent,int phone_claimed){
 return v&&phone_intent&&phone_claimed&&!v->manual_revoked&&!v->resume_requires_start&&!(v->feedback.local_stop_sources&0x3eu)&&v->has_goal&&!v->enabled&&!v->pending_enable&&
  (v->localization.state==LAB_REAL_LOC_TRACKING||v->localization.odometry_fallback)&&
  (v->navigation.path_count||v->navigation.can_start_local)&&!strcmp(v->reason,"HARDWARE_DISABLED");
}
#endif
