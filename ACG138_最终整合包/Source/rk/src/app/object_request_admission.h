#ifndef ACG_OBJECT_REQUEST_ADMISSION_H
#define ACG_OBJECT_REQUEST_ADMISSION_H
#include "phone_goal_ipc.h"
#include "continuous_policy.h"
#include <math.h>
/* 选择即为显式操作指令，不再等待感知ACK；仍检查报文损坏、STOP前旧命令、所有者及序号屏障。
 */
static inline const char*object_request_admission(const phone_goal_message*m,uint64_t now,uint64_t revoked){
 if(!m||!m->request_id||!m->sequence)return "OBJECT_REQUEST_ID_MISSING";
 if(!m->sent_us||m->sent_us>now||now-m->sent_us>CONT_PHONE_US)return "OBJECT_REQUEST_TIME_INVALID";
 if(m->sent_us<=revoked)return "OBJECT_REQUEST_REVOKED_BY_STOP";
 if(m->reserved<1||m->reserved>7||!isfinite(m->x_m)||m->x_m<.10||m->x_m>3)return "OBJECT_REQUEST_MODE_OR_STANDOFF_INVALID";
 if(m->reserved!=3&&m->reserved!=4&&m->reserved!=6&&(!m->object_epoch||m->object_track_id<1))return "OBJECT_IDENTITY_MISSING";
 return NULL;
}
#endif
