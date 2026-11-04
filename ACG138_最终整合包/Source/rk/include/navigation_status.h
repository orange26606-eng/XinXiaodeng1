#ifndef ACG_NAVIGATION_STATUS_H
#define ACG_NAVIGATION_STATUS_H
#include "lab_real_runtime.h"
#include "object_time_policy.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "../../board/phone_shared.h"
/* 只读已有相机接收器的小型序列锁头，不创建套接字或映射，不接管传感器，不复制视频像素。
 */
static inline int navigation_shared_motor_state(uint64_t now_us,unsigned*phase,unsigned*retries,unsigned*stop_reason){
 const char*dir=getenv("ACG_PHONE_DIR");if(!dir||!*dir)return 0;
#ifndef _WIN32
 if(dir[0]!='/')return 0;
#endif
 char path[640];if(snprintf(path,sizeof(path),"%s/frames.shm",dir)>=(int)sizeof(path))return 0;
 FILE*f=fopen(path,"rb");if(!f)return 0;
 struct{uint32_t magic,version,state_sequence,frame_sequence;struct phone_sensor_state state;} p;
 _Static_assert(sizeof(p)==offsetof(struct phone_shared,frame),"phone shared prefix ABI");
 int valid=0;for(unsigned i=0;i<3;i++){uint32_t seq=0;if(fseek(f,0,SEEK_SET)||fread(&p,sizeof(p),1,f)!=1)break;if(p.state_sequence&1u)continue;if(fseek(f,(long)offsetof(struct phone_shared,state_sequence),SEEK_SET)||fread(&seq,sizeof(seq),1,f)!=1)break;if(seq!=p.state_sequence||seq&1u)continue;
  uint64_t now=now_us/1000;if(p.magic!=PHONE_MAGIC||p.version!=1||!p.state.robot_seen||now<p.state.robot_ms||now-p.state.robot_ms>CONT_FEEDBACK_US/1000||!p.state.feedback_supported||p.state.feedback_phase>4||p.state.feedback_retries>2)break;
  *phase=p.state.feedback_phase;*retries=p.state.feedback_retries;*stop_reason=p.state.stop_reason;valid=1;break;
 }fclose(f);return valid;
}
static inline int navigation_feedback_fresh(const lab_real_runtime_view*v){
 return v->feedback.time_us&&v->time_us>=v->feedback.time_us&&v->time_us-v->feedback.time_us<=(v->object_mode?OBJECT_CONTROL_FEEDBACK_US:CONT_FEEDBACK_US);
}
/* 报告源快照中的实际FPGA非零缓存年龄分桶，不能把收到状态当作新超声波测量。
 */
static inline void navigation_sonar_sample_text(const lab_real_runtime_view*v,unsigned i,char*out,size_t cap){
 const char names[]="FLR";if(i>=3){snprintf(out,cap,"Sonar UNKNOWN");return;}
 int live=navigation_feedback_fresh(v),valid=live&&(v->feedback.sonar_valid&(1u<<i));unsigned bucket=v->feedback.sonar_age_bucket[i];
 if(!valid){snprintf(out,cap,"%c -- / age UNKNOWN%s",names[i],live&&(v->feedback.sonar_raw_zero_mask&(1u<<i))?" / Z0 IGNORED":"");return;}
 if(bucket==15)snprintf(out,cap,"%c %umm age>=3000ms%s",names[i],v->feedback.sonar_mm[i],v->feedback.sonar_raw_zero_mask&(1u<<i)?" Z0 HOLD":"");
 else snprintf(out,cap,"%c %umm age[%u,%u)ms%s",names[i],v->feedback.sonar_mm[i],bucket*200,bucket*200+200,v->feedback.sonar_raw_zero_mask&(1u<<i)?" Z0 HOLD":"");
}
static inline int navigation_wheels_moving(const lab_real_runtime_view*v){
 if(!navigation_feedback_fresh(v))return 0;
 for(unsigned i=0;i<4;i++)if(isfinite(v->feedback.actual_rpm[i])&&fabs(v->feedback.actual_rpm[i])>1)return 1;
 return 0;
}
static inline int navigation_pose_valid(const lab_real_runtime_view*v){
 const lab_localization_view*l=&v->localization;
 return v->has_localization&&l->has_pose&&isfinite(l->pose.x_m)&&isfinite(l->pose.y_m)&&isfinite(l->pose.yaw_rad)&&
 l->pose_time_us&&v->time_us>=l->pose_time_us&&v->time_us-l->pose_time_us<=CONT_ODOM_US&&
 (l->state==LAB_REAL_LOC_TRACKING||(l->state==LAB_REAL_LOC_DEGRADED&&l->odometry_fallback))&&
 (l->pose.quality&LAB_POSE_VALID)&&!(l->pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP));
}
static inline const char*navigation_stop_label(const lab_real_runtime_view*v){
 const char*c=v->last_stop_reason,*d=v->last_stop_detail;
 if(!c[0])return "NONE";
 if(!strcmp(c,"GOAL_REACHED"))return "TARGET REACHED";
 if(!strcmp(c,"USER_OR_PHONE_CANCEL"))return "USER / PHONE STOP";
 if(!strcmp(d,"OBSTACLE"))return "LIDAR OBSTACLE";
 if(strstr(d,"UNKNOWN"))return "UNOBSERVED AREA";
 if(strstr(d,"ODOMETRY")||!strcmp(d,"POSE_STALE"))return "ODOMETRY TOO OLD";
 if(strstr(d,"LIDAR")||!strcmp(d,"SCAN_STALE"))return "LIDAR DATA TOO OLD";
 if(strstr(c,"OVERRUN"))return "CONTROL TOOK TOO LONG";
 if(!strcmp(c,"INVALID_CHASSIS_FEEDBACK"))return "MOTOR FEEDBACK FAULT";
 if(!strcmp(c,"STALE_OR_INVALID_LOCALIZATION"))return "LOCALIZATION WAIT";
 if(!strcmp(c,"NAVIGATION_WAIT_OR_FAILURE"))return "ROUTE / OBSTACLE WAIT";
 if(strstr(c,"TRANSPORT"))return "STOP SEND FAILURE";
 return "SYSTEM PROTECTION";
}
static inline const char*navigation_vehicle_status(const lab_real_runtime_view*v){
 if(v->resume_requires_start)return "MOTOR_FEEDBACK_FAULT";
 if(navigation_feedback_fresh(v)&&v->feedback.faults)return "MOTOR_FEEDBACK_FAULT";
 if(!strcmp(v->reason,"CONTROL_COMPUTE_OVERRUN_RETRY"))return "CONTROL_RETRY";
 if(!strcmp(v->reason,"GOAL_REACHED"))return "GOAL_REACHED";
 if(v->pending_enable)return "ARMING";
 /* 任务取消后按当前任务状态显示；轮速反馈独立判断。 */

 if(!v->enabled&&!v->has_goal&&!v->navigation.cruise_mode&&v->last_status>=0){
  if(!navigation_feedback_fresh(v))return "IDLE_FEEDBACK_UNKNOWN";
  return navigation_wheels_moving(v)?"MANUAL_RUNNING":"IDLE_NO_GOAL";
 }
 if(v->enabled){
  if(!v->feedback.time_us)return "WAIT_FIRST_FEEDBACK";
  if(!navigation_feedback_fresh(v))return "FEEDBACK_STALE";
  if(v->feedback.faults)return "MOTOR_FEEDBACK_FAULT";
  if(strstr(v->feedback.reason,"RECOVER")||strstr(v->reason,"MOTOR_RECOVER"))return "MOTOR_RECOVERY";
  /* ALIGN、DETOUR、BACKING 表示计划动作；实际运动状态以最新轮速反馈为准，零轮速显示停止。 */
  if(!navigation_wheels_moving(v)){
   if(strstr(v->feedback.reason,"ULTRASONIC"))return "ULTRASONIC_WAIT";
   if(fabs(v->request.linear_m_s)>.0001||fabs(v->request.angular_rad_s)>.0001)return "WHEELS_ZERO";
   if(!strcmp(v->navigation.phase_text,"ALIGN")||!strcmp(v->navigation.phase_text,"DETOUR_LEFT")||!strcmp(v->navigation.phase_text,"DETOUR_RIGHT")||!strcmp(v->navigation.phase_text,"BACKING")||!strcmp(v->navigation.phase_text,"REJOIN")||!strcmp(v->navigation.phase_text,"PROBING")||!strcmp(v->navigation.phase_text,"CRUISE"))return "NAVIGATION_WAIT";
  }
  if(!strcmp(v->navigation.phase_text,"ALIGNMENT_HOLD"))return "ALIGNMENT_HOLD";
  if(!strcmp(v->navigation.phase_text,"ALIGNMENT_WAIT"))return "ALIGNMENT_WAIT";
  if(!strcmp(v->navigation.phase_text,"BACK_CHECK"))return "BACK_CHECK";
  if(!strcmp(v->navigation.phase_text,"BACKING"))return "BACKING";
  if(!strcmp(v->navigation.phase_text,"DETOUR_LEFT"))return "DETOUR_LEFT";
  if(!strcmp(v->navigation.phase_text,"DETOUR_RIGHT"))return "DETOUR_RIGHT";
  if(!strcmp(v->navigation.phase_text,"ALIGN"))return "ALIGNING";
  if(!strcmp(v->navigation.phase_text,"REJOIN"))return "REJOINING";
  if(!strcmp(v->navigation.phase_text,"PROBING"))return "PROBING";
  if(!strcmp(v->navigation.phase_text,"CRUISE"))return "CRUISE";
  if(v->feedback.sonar_near&&(fabs(v->request.linear_m_s)>.0001||fabs(v->request.angular_rad_s)>.0001))return "SONAR_AVOIDING";
  if(strstr(v->feedback.reason,"ULTRASONIC"))return "ULTRASONIC_WAIT";
  if(!strcmp(v->navigation.reason,"REAR_BODY_OCCUPIED"))return "REAR_BLOCKED";
  if(!strcmp(v->navigation.reason,"REAR_SCAN_STALE")||!strcmp(v->navigation.reason,"REAR_COVERAGE")||!strcmp(v->navigation.reason,"REAR_SONAR_STALE")||!strcmp(v->navigation.reason,"REAR_GEOMETRY_UNKNOWN")||!strcmp(v->navigation.reason,"REAR_CLEARANCE"))return "REAR_OBSERVATION_WAIT";
  if(strstr(v->navigation.blocking_reason,"UNKNOWN")||strstr(v->navigation.reason,"UNKNOWN"))return "OBSERVATION_WAIT";
  if(!strcmp(v->navigation.reason,"RECOVERY_EXHAUSTED"))return "RECOVERY_EXHAUSTED";
  if(!strcmp(v->navigation.reason,"RECOVERY_RETRY_WAIT"))return "RECOVERY_RETRY_WAIT";
  if(!strcmp(v->navigation.phase_text,"BLOCKED"))return "OBSTACLE_WAIT";
  if(!strcmp(v->navigation.reason,"REPLANNING"))return "PLANNING";
  if(fabs(v->request.linear_m_s)>.0001||fabs(v->request.angular_rad_s)>.0001){
   if(navigation_wheels_moving(v))return v->localization.odometry_fallback?"ODOMETRY_RUNNING":"RUNNING";
   return "WHEELS_ZERO";
  }
  return "NAVIGATION_WAIT";
 }
 if(!strcmp(v->reason,"STALE_OR_INVALID_LOCALIZATION")||!strcmp(v->reason,"WAITING_FOR_LOCALIZATION"))return "SENSOR_OR_LOCALIZATION_WAIT";
 if(v->has_goal&&(strstr(v->navigation.blocking_reason,"UNKNOWN")||strstr(v->navigation.reason,"UNKNOWN")))return "OBSERVATION_WAIT";
 if(v->navigation.state==2&&!strcmp(v->navigation.reason,"REPLANNING"))return "PLANNING";
 if(strstr(v->navigation.reason,"OBSTACLE")||strstr(v->navigation.reason,"BLOCKED"))return "OBSTACLE_WAIT";
 if(strstr(v->navigation.reason,"SEARCH_LIMIT")||strstr(v->navigation.reason,"NO_PATH"))return "ROUTE_RETRY";
 if(v->mode==LAB_REAL_SHADOW)return "PREVIEW_ONLY";
 if(v->last_status<0)return "FAULT_STOPPED";
 return v->has_goal||v->navigation.cruise_mode?"WAIT_START":"STOPPED";
}
#endif
