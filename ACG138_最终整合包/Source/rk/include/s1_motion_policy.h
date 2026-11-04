#ifndef ACG_S1_MOTION_POLICY_H
#define ACG_S1_MOTION_POLICY_H
#include <stdint.h>
#include "r6_motion_policy.h"
#define S1_CONFIG_ID R6_CONFIG_ID
#define S1_COMMAND_VERSION 3u
#define S5_POLICY_REVISION 5u
#define S1_STATUS_REVISION 5u
enum s1_motion_intent { S1_ZERO=0,S1_FORWARD=1,S1_ESCAPE=2,S1_PIVOT=3,S1_REVERSE=4 };
/* 输入为最终量化的有符号机械RPM×100，左M1/M3、右M2/M4。混合极性且中心速度非零时，不能按近似相等授予原地转向权限。

 */
static inline unsigned s1_classify_lr_centi(int left,int right){
 int32_t sum=(int32_t)left+right;
 return !left&&!right?S1_ZERO:sum==0?S1_PIVOT:sum>0?S1_FORWARD:S1_REVERSE;
}
static inline int s1_intent_matches(unsigned intent,int left,int right){
 unsigned shape=s1_classify_lr_centi(left,right);
 int32_t diff=(int32_t)right-left,sum=(int32_t)left+right;if(diff<0)diff=-diff;
 return intent==S1_ESCAPE?(shape==S1_FORWARD&&left!=right&&4*diff>=sum):intent==shape;
}
static inline int s1_direction_allows(unsigned intent,int left,int right,const uint16_t mm[3],unsigned valid,unsigned fresh,unsigned near,unsigned pending){
 if(!s1_intent_matches(intent,left,right)||!mm)return 0;
 if(intent==S1_ZERO)return 1;
 unsigned ok=valid&fresh&7u,blocked=near&pending&7u;
 if(intent==S1_REVERSE)return (ok&6u)==6u;
 if(intent==S1_PIVOT)return ok==7u&&!blocked&&mm[0]>=30&&mm[1]>=30&&mm[2]>=30;
 if(!(ok&1u)||(blocked&1u))return 0;
 if(intent==S1_FORWARD&&((near&1u)||mm[0]<R6_FRONT_FORWARD_STOP_MM))return 0;
 if(intent==S1_ESCAPE&&(mm[0]<R6_FRONT_FORWARD_STOP_MM||left==right))return 0;
 if(left!=right){unsigned side=right>left?1u:2u,bit=1u<<side;
  if(!(ok&bit)||(blocked&bit)||mm[side]<30)return 0;
  if((left<0||right<0)&&(ok!=7u||blocked||mm[1]<30||mm[2]<30))return 0;
 }
 return 1;
}
/* ESCAPE几何空间需由RK另外核对，仅设置标签或角速度非零不能证明整车扫掠区域可通行。
 */
#endif
