#ifndef ACG_OBJECT_TURN_SMOOTH_H
#define ACG_OBJECT_TURN_SMOOTH_H
#include <math.h>
#include <stdint.h>
#include <string.h>
/* 仅整形实物任务角速度，不改变源日期、租约或直线速度。控制器及规划器的真实STOP或零命令直接绕过；FPGA启动PWM助力仍独立执行，此处不模拟。

 */
#define OBJ_SEARCH_YAW_CAP .20
#define OBJ_ALIGN_YAW_CAP .30
#define OBJ_YAW_ACCEL .40
#define OBJ_YAW_DECEL .80
#define OBJ_YAW_SETTLED .08
typedef struct obj_turn_measure {
 uint64_t stamp; double yaw,rate; int valid;
} obj_turn_measure;
typedef struct obj_turn_smoother {
 uint64_t tick,zero_since,log_us; double output,selected,limited;
 int initialized,last_sign,waiting; const char*reason;
 obj_turn_measure motion;
} obj_turn_smoother;
static inline double obj_turn_clip(double v,double lo,double hi){return fmax(lo,fmin(hi,v));}
static inline void obj_turn_observe(obj_turn_measure*m,uint64_t stamp,double yaw){
 if(!stamp||!isfinite(yaw))return;
 if(m->stamp&&stamp>m->stamp){double dt=(stamp-m->stamp)/1e6;
  if(dt>=.005&&dt<=.30){m->rate=remainder(yaw-m->yaw,6.283185307179586)/dt;m->valid=isfinite(m->rate);}
  else m->valid=0;
 }else if(m->stamp&&stamp<m->stamp)m->valid=0;
 if(!m->stamp||stamp!=m->stamp){m->stamp=stamp;m->yaw=yaw;}
}
static inline int obj_turn_measured(const obj_turn_measure*m,uint64_t now){
 return m->valid&&m->stamp<=now&&now-m->stamp<=300000;
}
static inline void obj_turn_reset(obj_turn_smoother*s){memset(s,0,sizeof(*s));s->reason="RESET";}
static inline double obj_turn_shape(obj_turn_smoother*s,uint64_t now,double target,int hard_zero){
 s->selected=target;s->waiting=0;
 if(hard_zero||!isfinite(target)||!now){s->output=s->limited=0;s->tick=now;s->initialized=1;if(!s->zero_since)s->zero_since=now;s->reason="STOP_PRIORITY";return 0;}
 if(s->initialized&&now<=s->tick){s->output=s->limited=0;s->last_sign=0;s->zero_since=0;s->reason="NONADVANCING_CLOCK";return 0;}
 double dt=s->initialized?fmin(.10,(now-s->tick)/1e6):.05;s->tick=now;s->initialized=1;
 int sign=target>1e-6?1:target< -1e-6?-1:0;
 double aim=target;
 if(sign&&s->last_sign&&sign!=s->last_sign){
  aim=0;s->waiting=1;s->reason="REVERSAL_DECEL";
  if(fabs(s->output)<1e-6){
   if(!s->zero_since)s->zero_since=now;
   int measured=obj_turn_measured(&s->motion,now);
   uint64_t settle=measured?100000:250000;
   if(now-s->zero_since>=settle&&(!measured||fabs(s->motion.rate)<=OBJ_YAW_SETTLED)){
    s->last_sign=sign;s->zero_since=0;aim=target;s->waiting=0;s->reason="REVERSAL_RELEASED";
   }else s->reason=measured&&fabs(s->motion.rate)>OBJ_YAW_SETTLED?"REVERSAL_WAIT_MEASURED_YAW":"REVERSAL_SETTLE";
  }
 }else{s->zero_since=0;s->reason="YAW_SLEW";if(sign&&!s->last_sign)s->last_sign=sign;}
 double rate=fabs(aim)<fabs(s->output)||s->output*aim<0?OBJ_YAW_DECEL:OBJ_YAW_ACCEL;
 s->output+=obj_turn_clip(aim-s->output,-rate*dt,rate*dt);
 if(fabs(s->output)<1e-6)s->output=0;
 s->limited=s->output;return s->output;
}
/* 按实测航向计算终点减速，并计入命令和调度期间的转角。RK 限值尚未经过实车制动标定。 */
static inline double obj_turn_endpoint(double error,double cap){
 double room=fmax(0,fabs(error)-.015);
 return copysign(fmin(cap,fmin(sqrt(2*OBJ_YAW_DECEL*room),room/.60)),error);
}
#endif
