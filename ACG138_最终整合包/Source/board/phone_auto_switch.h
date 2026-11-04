#ifndef ACG_PHONE_AUTO_SWITCH_H
#define ACG_PHONE_AUTO_SWITCH_H
#include <stdint.h>
#include <string.h>
#define PHONE_AUTO_INTENT_US UINT64_C(8000000)
typedef struct phone_auto_switch {char token[33],session[33];uint64_t issued_us,expires_us,generation;int valid;} phone_auto_switch;
enum phone_auto_switch_result {PHONE_SWITCH_OK=0,PHONE_SWITCH_INVALID=-1,PHONE_SWITCH_EXPIRED=-2,PHONE_SWITCH_REVOKED=-3};
static inline int phone_switch_hex(const char*s){if(!s||strlen(s)!=32)return 0;for(unsigned i=0;i<32;i++)if(!((s[i]>='0'&&s[i]<='9')||(s[i]>='a'&&s[i]<='f')))return 0;return 1;}
static inline void phone_switch_clear(phone_auto_switch*s){memset(s,0,sizeof(*s));}
static inline int phone_switch_issue(phone_auto_switch*s,const char*session,const char*token,uint64_t now,uint64_t generation){
 if(!s||!phone_switch_hex(session)||!phone_switch_hex(token)||!now||!generation||now>UINT64_MAX-PHONE_AUTO_INTENT_US)return PHONE_SWITCH_INVALID;
 memset(s,0,sizeof(*s));memcpy(s->session,session,33);memcpy(s->token,token,33);s->issued_us=now;s->expires_us=now+PHONE_AUTO_INTENT_US;s->generation=generation;s->valid=1;return PHONE_SWITCH_OK;
}
/* 令牌成功消费后失效。停车、会话切换或手动输入会更新generation并撤销旧请求。 */

static inline int phone_switch_consume(phone_auto_switch*s,const char*session,const char*token,uint64_t now,uint64_t generation,uint64_t*issued,uint64_t*expires){
 if(!s||!s->valid||!phone_switch_hex(session)||!phone_switch_hex(token)||strcmp(session,s->session))return PHONE_SWITCH_INVALID;
 unsigned diff=0;for(unsigned i=0;i<32;i++)diff|=(unsigned char)token[i]^(unsigned char)s->token[i];if(diff)return PHONE_SWITCH_INVALID;
 if(generation!=s->generation){s->valid=0;return PHONE_SWITCH_REVOKED;}
 if(now<s->issued_us||now>=s->expires_us){s->valid=0;return PHONE_SWITCH_EXPIRED;}
 *issued=s->issued_us;*expires=s->expires_us;s->valid=0;return PHONE_SWITCH_OK;
}
#endif
