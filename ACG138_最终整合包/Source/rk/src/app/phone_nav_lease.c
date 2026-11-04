#include "phone_nav_lease.h"
#include <string.h>
#include "continuous_policy.h"

#define PHONE_NAV_LEASE_US CONT_PHONE_US
static int session_ok(const char *s){
 if(!s||strlen(s)!=32)return 0;
 for(int i=0;i<32;i++)if(!((s[i]>='0'&&s[i]<='9')||(s[i]>='a'&&s[i]<='f')))return 0;
 return s[32]=='\0';
}
static int is_owner(const phone_nav_lease*l,const char*s){return l&&l->claimed&&session_ok(s)&&!strcmp(l->session,s);}
static int timed_out(const phone_nav_lease*l,uint64_t t){return t<l->last_seen_us||t-l->last_seen_us>=PHONE_NAV_LEASE_US;}
void phone_nav_lease_init(phone_nav_lease*l){if(l)memset(l,0,sizeof(*l));}
int phone_nav_lease_claim(phone_nav_lease*l,const char*s,uint64_t t){
 if(!l||!session_ok(s)||!t)return PHONE_NAV_DENIED;
 if(l->claimed){if(is_owner(l,s)&&(t<=l->last_seen_us||!timed_out(l,t)))return PHONE_NAV_OK;return l->motion_active?PHONE_NAV_STOP_REQUIRED:PHONE_NAV_DENIED;}
 memcpy(l->session,s,33);l->claimed=1;l->last_seen_us=t;l->last_sequence=0;return PHONE_NAV_OK;
}
int phone_nav_lease_heartbeat(phone_nav_lease*l,const char*s,uint64_t q,uint64_t t){
 /* 忽略旧、重复或错误所有者心跳，不刷新租约；只有实际持有者租约到期才请求停车，START前排队PING不算新的通信故障。

 */
 if(!is_owner(l,s)||!q||q<=l->last_sequence||t<l->last_seen_us)return PHONE_NAV_DENIED;
 if(t-l->last_seen_us>=PHONE_NAV_LEASE_US)return l->motion_active?PHONE_NAV_STOP_REQUIRED:PHONE_NAV_DENIED;
 l->last_sequence=q;l->last_seen_us=t;return PHONE_NAV_OK;
}
int phone_nav_lease_start(phone_nav_lease*l,const char*s,uint64_t q,uint64_t t,int drive){
 if(!drive||!is_owner(l,s)||!q||q<=l->last_sequence||(t>l->last_seen_us&&timed_out(l,t)))return PHONE_NAV_DENIED;
 if(t>l->last_seen_us)l->last_seen_us=t;l->last_sequence=q;l->motion_active=1;return PHONE_NAV_OK;
}
int phone_nav_lease_receive(phone_nav_lease*l,const char*s,uint64_t q,uint64_t origin,uint64_t received,uint64_t revoked,int claim){
 if(!l||!origin||origin>received||received-origin>CONT_PHONE_US||origin<=revoked)return PHONE_NAV_DENIED;
 if(claim&&phone_nav_lease_claim(l,s,received)!=PHONE_NAV_OK)return PHONE_NAV_DENIED;
 return phone_nav_lease_heartbeat(l,s,q,received);
}
int phone_nav_lease_stop(phone_nav_lease*l,const char*s,uint64_t q,uint64_t t){
 if(!is_owner(l,s)||!q||q<=l->last_sequence||t<l->last_seen_us)return PHONE_NAV_DENIED;
 l->last_sequence=q;l->last_seen_us=t;l->motion_active=0;return PHONE_NAV_STOP_REQUIRED;
}
int phone_nav_lease_tick(phone_nav_lease*l,uint64_t t,int phone_ok,int manual_override){
 if(!l)return PHONE_NAV_DENIED;
 if(l->claimed&&(!phone_ok||manual_override||timed_out(l,t))){int stop=l->motion_active;phone_nav_lease_init(l);return stop?PHONE_NAV_STOP_REQUIRED:PHONE_NAV_DENIED;}
 return PHONE_NAV_OK;
}
