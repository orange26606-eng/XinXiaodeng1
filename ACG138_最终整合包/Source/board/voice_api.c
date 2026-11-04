#include "voice_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
static void space(const char**p){while(isspace((unsigned char)**p))++*p;}
static int text(const char**p,char*b,size_t cap){space(p);if(*(*p)++!='"')return 0;size_t n=0;while(**p&&**p!='"'){unsigned char c=(unsigned char)*(*p)++;if(c<32||c=='\\'||n+1>=cap)return 0;b[n++]=(char)c;}if(*(*p)++!='"')return 0;b[n]=0;return 1;}
static int read_uint(const char**p,uint64_t*v,int quoted){char b[32],*end;space(p);const char*s=*p;if(quoted){if(!text(p,b,sizeof b))return 0;s=b;}if(!isdigit((unsigned char)*s)||(*s=='0'&&isdigit((unsigned char)s[1])))return 0;errno=0;*v=strtoull(s,&end,10);if(errno||end==s||(quoted&&*end))return 0;if(!quoted)*p=end;return 1;}
static int member(const char*s,const char*const*a){for(;*a;a++)if(!strcmp(s,*a))return 1;return 0;}
static int valid(const semantic_spec*s){
 const char*const modes[]={"FIND","SELECT","PREVIEW","GO","FOLLOW",NULL};
 const char*const pos[]={"","left","right","center","near","far",NULL};
 const char*const colors[]={"","red","orange","yellow","green","cyan","blue","purple","pink","brown","white","gray","black","mixed",NULL};
 const char*const sizes[]={"","small","medium","large",NULL};
 if(!member(s->mode,modes)||!member(s->position,pos)||!member(s->color,colors)||!member(s->size,sizes)||s->ordinal>65||s->reference>1||!isfinite(s->distance)||s->distance<.1||s->distance>3)return 0;
 for(const char*p=s->category;*p;p++)if(!islower((unsigned char)*p)&&*p!=' '&&*p!='_')return 0;
 return s->category[0]||s->reference||s->ordinal||s->position[0];
}
static int unpack_uint(const char*s,unsigned*out){uint64_t v=0;if(!*s)return 0;for(;*s;s++){if(!isdigit((unsigned char)*s)||v>650)return 0;v=v*10+(unsigned)(*s-'0');}if(v>65)return 0;*out=(unsigned)v;return 1;}
int semantic_unpack(const phone_goal_message*m,semantic_spec*s){
 if(!m||!s||m->version!=6||m->operation!=PHONE_SEMANTIC_TASK||!memchr(m->association_id,0,sizeof(m->association_id)))return 0;
 memset(s,0,sizeof *s);char buf[sizeof(m->association_id)],*parts[7];memcpy(buf,m->association_id,sizeof buf);unsigned n=0;parts[n++]=buf;
 for(char*p=buf;*p;p++)if(*p=='|'){*p=0;if(n>=7)return 0;parts[n++]=p+1;}
 if(n!=5&&n!=7)return 0;for(unsigned k=0;k<n;k++)if(!*parts[k])return 0;
 if(strlen(parts[0])>=sizeof s->mode||strlen(parts[1])>=sizeof s->category||strlen(parts[2])>=sizeof s->position)return 0;
 strcpy(s->mode,parts[0]);if(strcmp(parts[1],"_"))strcpy(s->category,parts[1]);if(strcmp(parts[2],"_"))strcpy(s->position,parts[2]);
 if(!unpack_uint(parts[3],&s->ordinal)||!unpack_uint(parts[4],&s->reference)||s->reference>1)return 0;
 if(n==7){if(strlen(parts[5])>=sizeof s->color||strlen(parts[6])>=sizeof s->size)return 0;if(strcmp(parts[5],"_"))strcpy(s->color,parts[5]);if(strcmp(parts[6],"_"))strcpy(s->size,parts[6]);}
 s->distance=m->x_m;return valid(s);
}
int semantic_payload(const char*p,unsigned op,phone_goal_message*m,uint64_t*seq){
 if(!p||!m||!seq||(op!=13&&op!=14&&op!=15))return 0;memset(m,0,sizeof *m);*seq=0;semantic_spec s={0};unsigned mask=0,count=0;space(&p);if(*p++!='{')return 0;
 while(1){char k[32];uint64_t v=0;unsigned bit=0;if(++count>10||!text(&p,k,sizeof k))return 0;space(&p);if(*p++!=':')return 0;space(&p);
 if(!strcmp(k,"seq")){bit=1;if(!read_uint(&p,seq,0)||!*seq||*seq>UINT64_C(9007199254740991))return 0;}
 else if(!strcmp(k,"task_id")){bit=2;if(!read_uint(&p,&m->map_id,1)||!m->map_id||m->map_id>UINT64_C(9007199254740991))return 0;}
 else if(op==13&&!strcmp(k,"mode")){bit=4;if(!text(&p,s.mode,sizeof s.mode))return 0;}
 else if(op==13&&!strcmp(k,"category")){bit=8;if(!text(&p,s.category,sizeof s.category))return 0;}
 else if(op==13&&!strcmp(k,"position")){bit=16;if(!text(&p,s.position,sizeof s.position))return 0;}
 else if(op==13&&!strcmp(k,"ordinal")){bit=32;if(!read_uint(&p,&v,0)||v>65)return 0;s.ordinal=(unsigned)v;}
 else if(op==13&&!strcmp(k,"reference")){bit=64;if(!strncmp(p,"true",4)){s.reference=1;p+=4;}else if(!strncmp(p,"false",5))p+=5;else return 0;}
 else if(op==13&&!strcmp(k,"color")){bit=256;if(!text(&p,s.color,sizeof s.color))return 0;}
 else if(op==13&&!strcmp(k,"size")){bit=512;if(!text(&p,s.size,sizeof s.size))return 0;}
 else if(op==13&&!strcmp(k,"distance_m")){bit=128;const char*end=p;if(!isdigit((unsigned char)*end))return 0;if(*end=='0')end++;else while(isdigit((unsigned char)*end))end++;if(*end=='.'){end++;if(!isdigit((unsigned char)*end))return 0;while(isdigit((unsigned char)*end))end++;}if(*end=='e'||*end=='E'){end++;if(*end=='+'||*end=='-')end++;if(!isdigit((unsigned char)*end))return 0;while(isdigit((unsigned char)*end))end++;}char*parsed;errno=0;s.distance=strtod(p,&parsed);if(errno||parsed!=end||!isfinite(s.distance))return 0;p=end;}
 else return 0;if(mask&bit)return 0;mask|=bit;space(&p);if(*p=='}'){p++;break;}if(*p++!=',')return 0;
 }
 space(&p);if(*p||(op==13?(mask&255)!=255:mask!=3))return 0;
 if(op==13){if(!valid(&s))return 0;m->x_m=s.distance;int written;if(s.color[0]||s.size[0])written=snprintf(m->association_id,sizeof(m->association_id),"%s|%s|%s|%u|%u|%s|%s",s.mode,s.category[0]?s.category:"_",s.position[0]?s.position:"_",s.ordinal,s.reference,s.color[0]?s.color:"_",s.size[0]?s.size:"_");else written=snprintf(m->association_id,sizeof(m->association_id),"%s|%s|%s|%u|%u",s.mode,s.category[0]?s.category:"_",s.position[0]?s.position:"_",s.ordinal,s.reference);if(written<0||(size_t)written>=sizeof(m->association_id))return 0;}
 m->version=6;m->operation=op;return 1;
}

int semantic_search_pack(phone_goal_message*m,const semantic_spec*s){
 if(!m||!s||!valid(s)||!s->category[0])return 0;int n;
 if(s->color[0]||s->size[0]){m->reserved=6;n=snprintf(m->association_id,sizeof m->association_id,"%s|%s|%s",s->category,s->color[0]?s->color:"_",s->size[0]?s->size:"_");}
 else{m->reserved=4;n=snprintf(m->association_id,sizeof m->association_id,"%s",s->category);}
 return n>0&&(size_t)n<sizeof m->association_id;
}
