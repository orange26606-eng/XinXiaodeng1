#ifndef ACG_PHONE_JSON_H
#define ACG_PHONE_JSON_H
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include "../rk/src/app/phone_goal_ipc.h"
/* 命令使用有长度上限的扁平JSON，仅接受规定的键和标量类型。 */
static void space(const char**p){while(isspace((unsigned char)**p))++*p;}
static int string(const char**p,char*out,size_t cap){space(p);if(*(*p)++!='"')return -1;size_t n=0;while(**p&&**p!='"'){unsigned char ch=(unsigned char)*(*p)++;if(ch<32||ch=='\\'||n+1>=cap)return -1;out[n++]=(char)ch;}if(**p!='"')return -1;++*p;out[n]=0;return 0;}
static int command_json(const char*p,char session[33],uint64_t*seq,unsigned*op,unsigned*motion,uint64_t*sent,uint64_t*hold){
 unsigned seen=0;space(&p);if(*p++!='{')return -1;*sent=*hold=0;
 for(unsigned field=0;field<6;field++){
  char key[24],value[48];if(string(&p,key,sizeof(key)))return -1;space(&p);if(*p++!=':')return -1;space(&p);unsigned bit;
  if(!strcmp(key,"session")){bit=1;if(string(&p,session,33)||strlen(session)!=32)return -1;}
  else if(!strcmp(key,"seq")||!strcmp(key,"sent_ms")||!strcmp(key,"hold_until_ms")){
   bit=!strcmp(key,"seq")?2:!strcmp(key,"sent_ms")?16:32;char*end;if(!isdigit((unsigned char)*p))return -1;errno=0;uint64_t n=strtoull(p,&end,10);if(errno||end==p||n>UINT64_C(9007199254740991))return -1;p=end;if(bit==2)*seq=n;else if(bit==16)*sent=n;else *hold=n;
  }else if(!strcmp(key,"op")){bit=4;if(string(&p,value,sizeof(value)))return -1;const char*names[]={"stop","manual","auto","hold","ping"};for(*op=0;*op<5;(*op)++)if(!strcmp(value,names[*op]))break;if(*op==5)return -1;}
  else if(!strcmp(key,"motion")){bit=8;if(string(&p,value,sizeof(value)))return -1;const char*names[]={"","forward","reverse","curve_left","curve_right","spin_left","spin_right","back_left","back_right"};for(*motion=0;*motion<9;(*motion)++)if(!strcmp(value,names[*motion]))break;if(*motion==9)return -1;}
  else return -1;if(seen&bit)return -1;seen|=bit;space(&p);if(*p=='}')break;if(*p++!=',')return -1;
 }
 if(*p++!='}')return -1;space(&p);return *p||((seen!=63)&&!(seen==15&&*op==PHONE_STOP))?-1:0;
}
/* 心跳使用严格字段格式，不含运动、客户端时钟或任意扩展字段。 */
static int phone_heartbeat_json(const char*p,int*control_mode){
 char key[16],value[16];space(&p);if(*p++!='{'||string(&p,key,sizeof(key))||strcmp(key,"mode"))return -1;
 space(&p);if(*p++!=':'||string(&p,value,sizeof(value)))return -1;space(&p);if(*p++!='}')return -1;space(&p);if(*p)return -1;
 if(!strcmp(value,"control"))*control_mode=1;else if(!strcmp(value,"view"))*control_mode=0;else return -1;return 0;
}
/* 权限和时间由服务端签发，客户端只提供seq和token。 */
static int phone_auto_request_json(const char*p,uint64_t*seq,char token[33],int prepare){
 unsigned seen=0;space(&p);if(*p++!='{')return -1;
 for(unsigned i=0;i<(prepare?1u:2u);i++){
  char key[24],*end;if(string(&p,key,sizeof(key)))return -1;space(&p);if(*p++!=':')return -1;space(&p);
  unsigned bit=!strcmp(key,"seq")?1:!prepare&&!strcmp(key,"switch_token")?2:0;if(!bit||(seen&bit))return -1;
  if(bit==1){if(!isdigit((unsigned char)*p)||(*p=='0'&&isdigit((unsigned char)p[1])))return -1;errno=0;uint64_t n=strtoull(p,&end,10);if(errno||end==p||!n||n>UINT64_C(9007199254740991))return -1;*seq=n;p=end;}
  else{if(string(&p,token,33)||strlen(token)!=32)return -1;for(unsigned j=0;j<32;j++)if(!((token[j]>='0'&&token[j]<='9')||(token[j]>='a'&&token[j]<='f')))return -1;}
  seen|=bit;space(&p);if(i+1<(prepare?1u:2u)&&*p++!=',')return -1;
 }
 if(*p++!='}')return -1;space(&p);return *p||seen!=(prepare?1u:3u)?-1:0;
}
static int phone_navigation_json(const char*p,phone_goal_message*m,uint64_t*request_sequence,int tag){
 unsigned seen=0;space(&p);if(*p++!='{')return -1;
 for(unsigned i=0;i<6;i++){
  char key[24],value[32],*end;unsigned bit;if(string(&p,key,sizeof(key)))return -1;space(&p);if(*p++!=':')return -1;space(&p);
  if(!strcmp(key,"map_id")||!strcmp(key,"map_version")){
   bit=!strcmp(key,"map_id")?1:2;if(string(&p,value,sizeof(value))||!isdigit((unsigned char)value[0]))return -1;
   errno=0;uint64_t n=strtoull(value,&end,10);if(errno||*end||!n)return -1;if(bit==1)m->map_id=n;else m->map_version=n;
  }else if(!strcmp(key,"sent_us")){
    bit=32;errno=0;if(!isdigit((unsigned char)*p))return -1;m->sent_us=strtoull(p,&end,10);if(errno||end==p||!m->sent_us||m->sent_us>UINT64_C(9007199254740991))return -1;p=end;
   }else if(!strcmp(key,"seq")){
   bit=16;errno=0;if(!isdigit((unsigned char)*p))return -1;*request_sequence=strtoull(p,&end,10);if(errno||end==p||!*request_sequence||*request_sequence>UINT64_C(9007199254740991))return -1;p=end;
  }else if(tag&&!strcmp(key,"tag_id")){
   bit=4;errno=0;double n=strtod(p,&end);if(errno||end==p||!isfinite(n)||n<0||n>65535||floor(n)!=n)return -1;p=end;m->reserved=(uint32_t)n+1;
  }else if(tag&&!strcmp(key,"standoff_m")){
   bit=8;errno=0;double n=strtod(p,&end);if(errno||end==p||!isfinite(n)||n<.15||n>3)return -1;p=end;m->x_m=n;
  }else if(!tag&&(!strcmp(key,"x")||!strcmp(key,"y"))){
   bit=!strcmp(key,"x")?4:8;errno=0;double n=strtod(p,&end);if(errno||end==p||!isfinite(n)||fabs(n)>100000)return -1;p=end;if(bit==4)m->x_m=n;else m->y_m=n;
  }else return -1;
  if(seen&bit)return -1;seen|=bit;space(&p);if(*p=='}')break;if(*p++!=',')return -1;
 }
 if(*p++!='}')return -1;space(&p);return *p||(seen!=31&&seen!=63)?-1:0;
}

#endif
