#ifndef ACG_S1_MODE_STATE_H
#define ACG_S1_MODE_STATE_H
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
typedef struct s1_mode_state {uint64_t published_us,request_us;int known,active,ready,intent_accepted,waiting;char mode[12],status[48],reason[120],session[33];} s1_mode_state;
static inline void s1_ms_space(const char**p){while(isspace((unsigned char)**p))++*p;}
static inline int s1_ms_string(const char**p,char*out,size_t cap){if(*(*p)++!='"')return 0;size_t n=0;while(**p&&**p!='"'){unsigned char c=(unsigned char)*(*p)++;if(c<32||c=='\\'||n+1>=cap)return 0;out[n++]=(char)c;}if(**p!='"')return 0;++*p;out[n]=0;return 1;}
/* 独立模块的扁平状态，最多2048字节，以原发布时间判断时效。仅报告模式，不授予手动控制或电机输出权限。
 */
static inline int s1_mode_parse(s1_mode_state*out,const char*json,uint64_t now_us){
 memset(out,0,sizeof(*out));s1_mode_state v={0};const char*p=json;unsigned seen=0;int closed=0;s1_ms_space(&p);if(*p++!='{')return 0;
 for(unsigned fields=0;fields<16;fields++){
  char key[32],str[128]={0};uint64_t number=0;int boolean=0,type=0;if(!s1_ms_string(&p,key,sizeof(key)))return 0;s1_ms_space(&p);if(*p++!=':')return 0;s1_ms_space(&p);
  if(*p=='"'){type=1;if(!s1_ms_string(&p,str,sizeof(str)))return 0;}
  else if(!strncmp(p,"true",4)){type=2;boolean=1;p+=4;}else if(!strncmp(p,"false",5)){type=2;p+=5;}
  else{char*end;if(!isdigit((unsigned char)*p)||(*p=='0'&&isdigit((unsigned char)p[1])))return 0;errno=0;number=strtoull(p,&end,10);if(errno||end==p)return 0;p=end;type=3;}
  unsigned bit=!strcmp(key,"version")?1:!strcmp(key,"published_us")?2:!strcmp(key,"control_mode")?4:!strcmp(key,"auto_active")?8:!strcmp(key,"auto_ready")?16:!strcmp(key,"auto_request_us")?32:!strcmp(key,"auto_status")?64:!strcmp(key,"auto_reason")?128:!strcmp(key,"session")?256:!strcmp(key,"auto_intent_accepted")?512:!strcmp(key,"auto_waiting")?1024:0;
  if(bit&&(seen&bit))return 0;seen|=bit;
  if(bit==1&&(type!=3||number!=1))return 0;
  if(bit==2||bit==32){if(type==1){char*end;if(!str[0])return 0;for(size_t i=0;str[i];i++)if(!isdigit((unsigned char)str[i]))return 0;errno=0;number=strtoull(str,&end,10);if(errno||*end)return 0;}else if(type!=3)return 0;if(bit==2)v.published_us=number;else v.request_us=number;}
  if(bit==4){if(type!=1||strlen(str)>=sizeof(v.mode)||(strcmp(str,"MANUAL")&&strcmp(str,"AUTO")&&strcmp(str,"NAV")))return 0;strcpy(v.mode,str);}
  if(bit==8||bit==16||bit==512||bit==1024){if(type!=2)return 0;if(bit==8)v.active=boolean;else if(bit==16)v.ready=boolean;else if(bit==512)v.intent_accepted=boolean;else v.waiting=boolean;}
  if(bit==64||bit==128||bit==256){size_t cap=bit==64?sizeof(v.status):bit==128?sizeof(v.reason):sizeof(v.session);if(type!=1||strlen(str)>=cap)return 0;strcpy(bit==64?v.status:bit==128?v.reason:v.session,str);}
  s1_ms_space(&p);if(*p=='}'){p++;closed=1;break;}if(*p++!=',')return 0;s1_ms_space(&p);
 }
 s1_ms_space(&p);if(!closed||*p||(seen&511)!=511||!v.published_us||v.published_us>now_us||now_us-v.published_us>2500000|| (v.active&&strcmp(v.mode,"AUTO")))return 0;
 if(!(seen&512))v.intent_accepted=v.active;
 if((v.intent_accepted&&strcmp(v.mode,"AUTO"))||(v.waiting&&(!v.intent_accepted||v.active)))return 0;
 v.known=1;*out=v;return 1;
}
#endif
