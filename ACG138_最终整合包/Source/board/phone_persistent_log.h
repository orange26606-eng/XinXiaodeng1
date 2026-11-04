#ifndef PHONE_PERSISTENT_LOG_H
#define PHONE_PERSISTENT_LOG_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* 不接收请求体、路径、凭据或会话值。操作使用固定白名单，原因码为长度受限的静态诊断。
 */
typedef struct phone_diag_slot {uint64_t last_ms,suppressed;int status;char code[64];} phone_diag_slot;
typedef struct phone_diag_state {phone_diag_slot slots[18];} phone_diag_state;
static int phone_diag_prepare(phone_diag_state*s,uint64_t now,const char*operation,int status,const char*code,char*out,size_t cap,const char**event){
 static const char*const names[]={"http","map","motion","heartbeat","session","control","auto","auto/prepare","object","tag","goal","start","stop","reset","service","control-link","view","auth"};
 unsigned k=0;for(unsigned i=0;i<sizeof(names)/sizeof(names[0]);i++)if(!strcmp(operation,names[i])){k=i;break;}
 char safe[64];size_t n=code?strlen(code):0;int valid=n&&n<sizeof(safe);
 for(size_t i=0;valid&&i<n;i++)if(!((code[i]>='A'&&code[i]<='Z')||(code[i]>='0'&&code[i]<='9')||code[i]=='_'))valid=0;
 snprintf(safe,sizeof(safe),"%s",valid?code:"HTTP_UNCLASSIFIED");phone_diag_slot*p=&s->slots[k];int failed=status>=400,recovered=!failed&&p->status>=400;
 if(!failed&&!recovered)return 0;
 if(failed&&p->status==status&&!strcmp(p->code,safe)&&p->last_ms&&now>=p->last_ms&&now-p->last_ms<1000){p->suppressed++;return 0;}
 int size=snprintf(out,cap,"{\"operation\":\"%s\",\"status\":%d,\"code\":\"%s\",\"previous_code\":\"%s\",\"suppressed\":%llu}",names[k],status,safe,recovered?p->code:"NONE",(unsigned long long)p->suppressed);
 if(size<0||(size_t)size>=cap)return 0;
 *event=recovered?"HTTP_RECOVERED":"HTTP_ERROR";p->status=status;p->last_ms=now;p->suppressed=0;snprintf(p->code,sizeof(p->code),"%s",safe);return size;
}
#endif
