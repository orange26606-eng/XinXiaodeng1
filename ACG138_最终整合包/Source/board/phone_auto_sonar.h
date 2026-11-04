#ifndef PHONE_AUTO_SONAR_H
#define PHONE_AUTO_SONAR_H
#include <stdint.h>
#include <string.h>
typedef struct phone_sonar_auto {
 int active,acknowledged,failed;
 unsigned attempts;
 uint64_t request_us,last_start_ms,last_ack_ms;
 char session[33];
} phone_sonar_auto;
static inline void phone_sonar_auto_cancel(phone_sonar_auto*s){memset(s,0,sizeof(*s));}
static inline void phone_sonar_auto_begin(phone_sonar_auto*s,const char*session,uint64_t request,uint64_t now){
 memset(s,0,sizeof(*s));s->active=1;s->request_us=request;s->last_start_ms=now;s->attempts=1;memcpy(s->session,session,33);
}
/* 返回1重发首次启动；返回-1表示控制权或启动确认失效。 */
static inline int phone_sonar_auto_tick(phone_sonar_auto*s,const char*session,int live,int fresh,int auto_mode,uint64_t telemetry_ms,uint64_t now){
 if(!s->active)return 0;
 if(!live||strcmp(s->session,session))return -1;
 if(fresh&&telemetry_ms>=s->last_start_ms&&auto_mode){s->acknowledged=1;s->last_ack_ms=telemetry_ms;return 0;}
 if(s->acknowledged)return fresh&&telemetry_ms>s->last_ack_ms&&!auto_mode?-1:0;
 if(now<s->last_start_ms)return -1;
 if(now-s->last_start_ms<200)return 0;
 if(s->attempts>=4)return -1;
 s->attempts++;s->last_start_ms=now;return 1;
}
static inline const char*phone_sonar_auto_reason(unsigned state,unsigned reason,unsigned feedback,int confirmed){
 if(!confirmed)return "等待FPGA确认超声波自动模式";
 if(feedback)return "电机反馈恢复中，超声波自动任务保留";
 if(reason==3)return "前方超声波失联，等待新报文";
 if(reason==10)return "编码器尚未就绪";
 switch(state){
 case 0:return "等待三路超声波和编码器就绪";
 case 1:return "超声波自动：正常前进";
 case 2:return "超声波自动：左弧线避让";
 case 3:return "超声波自动：左原地转向";
 case 4:return "超声波自动：右原地转向";
 case 5:return "超声波自动：转向后恢复前进";
 case 6:return "超声波自动：右弧线避让";
 case 7:return "超声波自动：沿近期已行驶区域后退";
 case 8:return "超声波自动：减速前进";
 case 9:return "超声波自动：沿左弧线后退";
 case 10:return "超声波自动：沿右弧线后退";
 case 11:return "可用方向暂未打开，等待测距更新";
 case 12:return "已确认近障，清零输出后选择转向";
 case 13:case 14:return "恢复近期行驶方向，准备沿原路后退";
 default:return "超声波自动状态更新中";
 }
}
#endif
