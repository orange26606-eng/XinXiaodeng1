#include "voice_mission.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
typedef struct ot {size_t a,b;int end;char type;} ot;
typedef struct oj {const char*s;size_t n,p;int used;ot t[16384];} oj;
static void ws(oj*j){while(j->p<j->n&&isspace((unsigned char)j->s[j->p]))j->p++;}
static int value(oj*j,int depth){ws(j);if(depth>16||j->used>=16384||j->p>=j->n)return -1;int k=j->used++;ot*t=&j->t[k];t->a=j->p;t->type=j->s[j->p++];
 if(t->type=='{'||t->type=='['){char close=t->type=='{'?'}':']';ws(j);if(j->p<j->n&&j->s[j->p]==close)j->p++;else for(;;){if(t->type=='{'){int key=value(j,depth+1);if(key<0||j->t[key].type!='"')return -1;ws(j);if(j->p>=j->n||j->s[j->p++]!=':')return -1;}if(value(j,depth+1)<0)return -1;ws(j);if(j->p>=j->n)return -1;char c=j->s[j->p++];if(c==close)break;if(c!=',')return -1;}}
 else if(t->type=='"'){int done=0;while(j->p<j->n){unsigned char c=(unsigned char)j->s[j->p++];if(c=='"'){done=1;break;}if(c<32)return -1;if(c=='\\'){if(j->p>=j->n)return -1;c=(unsigned char)j->s[j->p++];if(c=='u'){for(int q=0;q<4;q++)if(j->p>=j->n||!isxdigit((unsigned char)j->s[j->p++]))return -1;}else if(!strchr("\"\\/bfnrt",c))return -1;}}if(!done)return -1;}
 else{while(j->p<j->n&&!isspace((unsigned char)j->s[j->p])&&!strchr(",]}",j->s[j->p]))j->p++;size_t z=j->p-t->a;const char*p=j->s+t->a;if(!((z==4&&!memcmp(p,"true",4))||(z==5&&!memcmp(p,"false",5))||(z==4&&!memcmp(p,"null",4)))){size_t q=0;if(q<z&&p[q]=='-')q++;if(q>=z)return -1;if(p[q]=='0')q++;else{if(p[q]<'1'||p[q]>'9')return -1;while(q<z&&isdigit((unsigned char)p[q]))q++;}if(q<z&&p[q]=='.'){q++;size_t a=q;while(q<z&&isdigit((unsigned char)p[q]))q++;if(q==a)return -1;}if(q<z&&(p[q]=='e'||p[q]=='E')){q++;if(q<z&&(p[q]=='+'||p[q]=='-'))q++;size_t a=q;while(q<z&&isdigit((unsigned char)p[q]))q++;if(q==a)return -1;}if(q!=z)return -1;}}
 t->b=j->p;t->end=j->used;return k;}
static oj*parse(const char*s,size_t n){if(!s||n<2||n>OBJ_SNAPSHOT_MAX)return NULL;oj*j=calloc(1,sizeof(*j));if(!j)return NULL;j->s=s;j->n=n;int k=value(j,0);ws(j);if(k!=0||j->p!=n||j->t[0].type!='{'){free(j);return NULL;}return j;}
static int eq(oj*j,int k,const char*s){return k>=0&&j->t[k].type=='"'&&j->t[k].b-j->t[k].a==strlen(s)+2&&!memcmp(j->s+j->t[k].a+1,s,strlen(s));}
static int field(oj*j,int k,const char*s){if(k<0||j->t[k].type!='{')return -1;int found=-1;for(int q=k+1;q<j->t[k].end;){int v=q+1;if(eq(j,q,s)){if(found>=0)return -1;found=v;}q=j->t[v].end;}return found;}
static int str(oj*j,int k,char*out,size_t cap){if(k<0||j->t[k].type!='"')return 0;size_t a=j->t[k].a+1,b=j->t[k].b-1,n=0;for(size_t i=a;i<b;i++){unsigned char c=(unsigned char)j->s[i];if(c=='\\'){if(++i>=b)return 0;c=(unsigned char)j->s[i];if(c=='n'||c=='r'||c=='t')c=' ';else if(c!='"'&&c!='\\'&&c!='/')return 0;}if(c<32||n+1>=cap)return 0;out[n++]=(char)c;}out[n]=0;return 1;}
static int uintval(oj*j,int k,uint64_t*out,int quoted){if(k<0||(quoted?j->t[k].type!='"':j->t[k].type=='"'))return 0;char b[24],*e;size_t a=j->t[k].a+(quoted?1:0),n=j->t[k].b-j->t[k].a-(quoted?2:0);if(!n||n>=sizeof(b))return 0;memcpy(b,j->s+a,n);b[n]=0;for(size_t q=0;q<n;q++)if(!isdigit((unsigned char)b[q]))return 0;errno=0;*out=strtoull(b,&e,10);return !errno&&!*e;}
static int num(oj*j,int k,double*out){if(k<0||j->t[k].type=='"'||j->t[k].b-j->t[k].a>=80)return 0;char b[80],*e;size_t n=j->t[k].b-j->t[k].a;memcpy(b,j->s+j->t[k].a,n);b[n]=0;errno=0;double d=strtod(b,&e);if(errno||e==b||*e||!isfinite(d))return 0;*out=d;return 1;}
static int integer(oj*j,int k,int min,int max,int*out){double d;if(!num(j,k,&d)||d<min||d>max||floor(d)!=d)return 0;*out=(int)d;return 1;}
static int boolean(oj*j,int k,int*out){if(k<0)return 0;size_t n=j->t[k].b-j->t[k].a;const char*s=j->s+j->t[k].a;if(n==4&&!memcmp(s,"true",4)){*out=1;return 1;}if(n==5&&!memcmp(s,"false",5)){*out=0;return 1;}return 0;}
int voice_receipt_parse(const char*s,size_t n,voice_receipt*r){oj*j=parse(s,n);if(!j)return 0;memset(r,0,sizeof *r);uint64_t v=0;int k=field(j,0,"task_request");int ok=uintval(j,field(j,0,"version"),&v,0)&&v==1&&uintval(j,field(j,0,"published_us"),&r->published_us,1)&&uintval(j,field(j,0,"request_id"),&r->request_id,1)&&uintval(j,field(j,0,"epoch"),&r->epoch,1)&&integer(j,field(j,0,"track_id"),-1,2147483647,&r->track)&&str(j,field(j,0,"mode"),r->mode,sizeof r->mode)&&str(j,field(j,0,"state"),r->state,sizeof r->state)&&str(j,field(j,0,"reason"),r->reason,sizeof r->reason)&&str(j,field(j,0,"message_zh"),r->message,sizeof r->message)&&boolean(j,field(j,0,"prepared"),&r->prepared)&&boolean(j,field(j,0,"active"),&r->active);
 if(ok&&k>=0){ok=uintval(j,field(j,k,"id"),&r->receipt_id,1)&&str(j,field(j,k,"state"),r->receipt_state,sizeof r->receipt_state)&&str(j,field(j,k,"reason"),r->receipt_reason,sizeof r->receipt_reason);}
 if(ok){(void)boolean(j,field(j,0,"drive_capable"),&r->drive);(void)boolean(j,field(j,0,"enabled"),&r->enabled);(void)boolean(j,field(j,0,"pending_enable"),&r->pending);(void)num(j,field(j,0,"requested_v"),&r->v);(void)num(j,field(j,0,"requested_w"),&r->w);(void)num(j,field(j,0,"protected_v"),&r->pv);(void)num(j,field(j,0,"protected_w"),&r->pw);}
 free(j);return ok;}
