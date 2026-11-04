#ifndef PHONE_NAVIGATION_H
#define PHONE_NAVIGATION_H
#include "phone_shared.h"
/* 依据遥测进行预检，FPGA另以50MHz时钟核对原始距离。 */
static inline int phone_ranges_fresh(const struct phone_sensor_state *s)
{
    for(unsigned i=0;i<3;i++) {
        /* CRC、UART和半帧错误保留FPGA可信缓存。完整正确报文年龄超限后，解析器切换到HOLD(7)或LOST(6)；valid与错误1～3只提供有限宽限，session_ready()仍核对传感器和报文时效。


 */
        if(s->error[i] && !(s->error[i]<=3 && (s->range_valid&(1u<<i)))) return 0;
        if(s->mm[i]<30 || s->mm[i]>4500) return 0;
        /* 正常超量程数据归一为valid=0、error=0、4500mm，其他缺失有效标记的组合视为未知。
 */
        if(!(s->range_valid&(1u<<i)) && s->mm[i]!=4500) return 0;
    }
    return 1;
}
static inline int phone_escape_motion(unsigned motion)
{ return motion==2 || motion==5 || motion==6 || motion==7 || motion==8; }
/* 仅供界面诊断，自主运动许可由RK与FPGA的配套策略决定。 */
static inline int phone_recovery_clear(const struct phone_sensor_state*s){return !s->near_mask&&s->mm[0]>=250&&s->mm[1]>=120&&s->mm[2]>=120;}
static inline unsigned phone_range_stop_mm(const struct phone_sensor_state*s,unsigned channel){(void)s;return channel==0?100:80;}
static inline int phone_motion_allowed(const struct phone_sensor_state*s,unsigned motion){(void)s;return motion>=1&&motion<=8;}
#endif
