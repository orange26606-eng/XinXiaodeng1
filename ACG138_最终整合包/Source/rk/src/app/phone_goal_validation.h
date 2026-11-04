#ifndef ACG_PHONE_GOAL_VALIDATION_H
#define ACG_PHONE_GOAL_VALIDATION_H
#include "phone_goal_ipc.h"
#include "continuous_policy.h"
#include <stddef.h>
#include <math.h>
#include "object_attribute_request.h"
static inline int phone_goal_message_valid(const phone_goal_message*m,size_t bytes,uint64_t now){
 if(m&&bytes==sizeof(*m)&&m->operation>=PHONE_NAV_OBJECT_PREPARE&&m->operation<=PHONE_NAV_OBJECT_RESUME){
  if(m->magic!=PHONE_GOAL_MAGIC||m->version!=7||!m->sent_us||m->sent_us>now||now-m->sent_us>CONT_PHONE_US||m->session[32]||m->reserved<1||m->reserved>7||!isfinite(m->x_m)||m->x_m<.05||m->x_m>3||m->y_m||m->request_expires_us||m->switch_generation||m->object_reserved||(m->reserved!=4&&m->reserved!=6&&m->association_id[0])||!m->request_id)return 0;
  if(m->operation>=PHONE_NAV_OBJECT_PAUSE&&m->reserved!=2&&m->reserved!=3)return 0;
  for(unsigned i=0;i<32;i++)if(!((m->session[i]>='0'&&m->session[i]<='9')||(m->session[i]>='a'&&m->session[i]<='f')))return 0;
  for(unsigned i=0;i<sizeof(m->padding);i++)if(m->padding[i])return 0;
  if(m->reserved==6){char category[64],color[16],size[12];return (m->operation==PHONE_NAV_OBJECT_PREPARE||m->operation==PHONE_NAV_OBJECT_START)&&!m->object_epoch&&!m->object_track_id&&memchr(m->association_id,0,sizeof(m->association_id))&&object_attribute_search_parse(m->association_id,category,sizeof(category),color,sizeof(color),size,sizeof(size));}
  if(m->reserved==5||m->reserved==7)return (m->operation==PHONE_NAV_OBJECT_PREPARE||m->operation==PHONE_NAV_OBJECT_START)&&m->object_epoch&&m->object_track_id>0;
  if(m->reserved==4){
   if((m->operation!=PHONE_NAV_OBJECT_PREPARE&&m->operation!=PHONE_NAV_OBJECT_START)||m->object_epoch||m->object_track_id||!m->association_id[0])return 0;
   unsigned n=0;for(;n<sizeof(m->association_id)&&m->association_id[n];n++)if(!((m->association_id[n]>='a'&&m->association_id[n]<='z')||m->association_id[n]=='_'||m->association_id[n]==' '))return 0;
   return n>0&&n<64;
  }
  if(m->reserved==3)return !m->object_epoch&&!m->object_track_id;
  return m->operation==PHONE_NAV_OBJECT_CANCEL||(m->object_epoch&&m->object_track_id>0);
 }
 if(!m||bytes!=sizeof(*m)||m->magic!=PHONE_GOAL_MAGIC||m->operation<PHONE_NAV_GOAL||m->operation>PHONE_NAV_AUTO_START||!m->sent_us||m->sent_us>now||m->session[32]||!isfinite(m->x_m)||!isfinite(m->y_m))return 0;
 for(unsigned i=0;i<32;i++)if(!((m->session[i]>='0'&&m->session[i]<='9')||(m->session[i]>='a'&&m->session[i]<='f')))return 0;
 for(unsigned i=0;i<sizeof(m->padding);i++)if(m->padding[i])return 0;
 if(m->operation==PHONE_NAV_TAG_GOAL){if(m->version!=3||!m->reserved||m->reserved>75536||m->y_m!=0)return 0;}else if(m->reserved)return 0;
 if(m->operation==PHONE_NAV_OBJECT_GOAL){if(m->version!=4||m->object_reserved||m->object_track_id<0||m->association_id[95]||!m->association_id[0]||m->y_m!=0)return 0;}
 else if(m->object_epoch||m->object_track_id||m->object_reserved||m->association_id[0])return 0;
 uint64_t age=CONT_PHONE_US;
 if(m->operation==PHONE_NAV_AUTO_START){if(m->version!=6||!m->switch_generation||m->request_expires_us<=m->sent_us||m->request_expires_us-m->sent_us>CONT_PHONE_US||m->x_m||m->y_m)return 0;age=CONT_PHONE_US;}
 else{if(m->request_expires_us||m->switch_generation)return 0;if(m->operation!=PHONE_NAV_TAG_GOAL&&m->operation!=PHONE_NAV_OBJECT_GOAL&&m->version!=2)return 0;}
 if(m->operation==PHONE_NAV_HEARTBEAT||m->operation==PHONE_NAV_OVERRIDE||m->operation==PHONE_NAV_CANCEL)age=CONT_PHONE_US;
 return now-m->sent_us<=age;
}
static inline const char*phone_goal_validation_reason(const phone_goal_message*m,size_t bytes,uint64_t now){
 if(!m||bytes!=sizeof(*m))return "NAV_IPC_SIZE_VERSION_MISMATCH";
 if(m->magic!=PHONE_GOAL_MAGIC)return "NAV_IPC_MAGIC_INVALID";
 if(!m->sent_us)return "REQUEST_ORIGIN_TIME_MISSING";
 if(m->sent_us>now)return "REQUEST_ORIGIN_IN_FUTURE";
 uint64_t age=(m->operation>=PHONE_NAV_OBJECT_PREPARE&&m->operation<=PHONE_NAV_OBJECT_RESUME)?3000000:m->operation==PHONE_NAV_AUTO_START?CONT_PHONE_US:
  (m->operation==PHONE_NAV_HEARTBEAT||m->operation==PHONE_NAV_OVERRIDE||m->operation==PHONE_NAV_CANCEL)?CONT_PHONE_US:CONT_PHONE_US;
 if(now-m->sent_us>age)return "REQUEST_EXPIRED_BEFORE_NAV_RECEIPT";
 return "NAV_IPC_PAYLOAD_OR_SESSION_INVALID";
}
#endif
