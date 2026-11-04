#define _POSIX_C_SOURCE 200809L
#include "tag_goal.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
/* 有界分词器保留不可变输入偏移，用值结束标记跳过完整嵌套数组或对象，避免匹配无关对象的键，不按子串猜JSON。

 */
typedef struct jt {size_t a,b;int end;char type;} jt;
typedef struct jp {const char*s;size_t n,p;int used; jt t[4096];} jp;
static void ws(jp*j){while(j->p<j->n&&isspace((unsigned char)j->s[j->p]))j->p++;}
static int val(jp*j,int depth){ws(j);if(depth>16||j->used>=4096||j->p>=j->n)return -1;int k=j->used++;jt*t=&j->t[k];t->a=j->p;t->type=j->s[j->p++];
 if(t->type=='{'||t->type=='['){char close=t->type=='{'?'}':']';ws(j);if(j->p<j->n&&j->s[j->p]==close)j->p++;else for(;;){if(t->type=='{'){int q=val(j,depth+1);if(q<0||j->t[q].type!='"')return -1;ws(j);if(j->p>=j->n||j->s[j->p++]!=':')return -1;}if(val(j,depth+1)<0)return -1;ws(j);if(j->p>=j->n)return -1;char c=j->s[j->p++];if(c==close)break;if(c!=',')return -1;}}
 else if(t->type=='"'){while(j->p<j->n&&j->s[j->p]!='"'){unsigned char c=(unsigned char)j->s[j->p++];if(c<32||c=='\\')return -1;}if(j->p>=j->n)return -1;j->p++;}
 else{while(j->p<j->n&&!isspace((unsigned char)j->s[j->p])&&j->s[j->p]!=','&&j->s[j->p]!=']'&&j->s[j->p]!='}')j->p++;}
 t->b=j->p;t->end=j->used;return k;}
static int eq(jp*j,int k,const char*s){return k>=0&&j->t[k].type=='"'&&j->t[k].b-j->t[k].a==strlen(s)+2&&!memcmp(j->s+j->t[k].a+1,s,strlen(s));}
static int field(jp*j,int k,const char*s){if(k<0||j->t[k].type!='{')return -1;int result=-1;for(int q=k+1;q<j->t[k].end;){int v=q+1;if(eq(j,q,s)){if(result>=0)return -1;result=v;}q=j->t[v].end;}return result;}
static int number(jp*j,int k,double*out){if(k<0||j->t[k].b-j->t[k].a>=80)return 0;char b[80],*end;size_t n=j->t[k].b-j->t[k].a;memcpy(b,j->s+j->t[k].a,n);b[n]=0;errno=0;double x=strtod(b,&end);if(errno||end==b||*end||!isfinite(x))return 0;*out=x;return 1;}
static int u64(jp*j,int k,uint64_t*out){if(k<0||j->t[k].b-j->t[k].a>=24)return 0;char b[24],*end;size_t n=j->t[k].b-j->t[k].a;memcpy(b,j->s+j->t[k].a,n);b[n]=0;if(!n)return 0;for(size_t i=0;i<n;i++)if(!isdigit((unsigned char)b[i]))return 0;errno=0;unsigned long long v=strtoull(b,&end,10);if(errno||*end)return 0;*out=(uint64_t)v;return 1;}
static int boolean(jp*j,int k,int*out){if(k<0)return 0;size_t n=j->t[k].b-j->t[k].a;const char*p=j->s+j->t[k].a;if(n==4&&!memcmp(p,"true",4)){*out=1;return 1;}if(n==5&&!memcmp(p,"false",5)){*out=0;return 1;}return 0;}
int tag_snapshot_parse(const char*s,size_t n,tag_snapshot*out){if(!s||!out||n<2||n>TAG_JSON_MAX)return 0;jp*j=calloc(1,sizeof(*j));if(!j)return 0;j->s=s;j->n=n;tag_snapshot q={0};int ok=0,root=val(j,0);ws(j);uint64_t version;double eye;
 if(root!=0||j->p!=n||!u64(j,field(j,0,"version"),&version)||version!=1||!u64(j,field(j,0,"sequence"),&q.sequence)||!u64(j,field(j,0,"frame_time_us"),&q.frame_us)||!u64(j,field(j,0,"generated_us"),&q.generated_us)||!boolean(j,field(j,0,"calibrated"),&q.calibrated)||!number(j,field(j,0,"eye"),&eye)||(eye!=0&&eye!=1)||!eq(j,field(j,0,"time_source"),"RK_DEQUEUE"))goto done;q.eye=(int)eye;
 int a=field(j,0,"tags");if(a<0||j->t[a].type!='[')goto done;
 for(int k=a+1;k<j->t[a].end;k=j->t[k].end){if(q.count>=TAG_MAX||j->t[k].type!='{')goto done;tag_observation*t=&q.tags[q.count];double id;
  if(!number(j,field(j,k,"id"),&id)||id<0||id>65535||floor(id)!=id||!boolean(j,field(j,k,"confirmed"),&t->confirmed))goto done;t->id=(int)id;
  /* 未标定输出允许坐标为null，编号仍可显示。 */
  if(q.calibrated&&!boolean(j,field(j,k,"pose_valid"),&t->pose_valid))goto done;
  if(q.calibrated&&t->pose_valid){if(!number(j,field(j,k,"x_m"),&t->x)||!number(j,field(j,k,"y_m"),&t->y)||!number(j,field(j,k,"z_m"),&t->z)||t->z<=0||!number(j,field(j,k,"normal_x"),&t->nx)||!number(j,field(j,k,"normal_y"),&t->ny)||!number(j,field(j,k,"normal_z"),&t->nz)||!number(j,field(j,k,"reprojection_error_px"),&t->error)||!number(j,field(j,k,"decision_margin"),&t->margin))goto done;}
  for(int z=0;z<q.count;z++)if(q.tags[z].id==t->id)goto done;q.count++;
 }*out=q;ok=1;
done:free(j);return ok;}
int tag_snapshot_load(const char*path,tag_snapshot*out){if(!path||!out)return 0;FILE*f=fopen(path,"rb");if(!f)return 0;struct stat st;if(fstat(fileno(f),&st)||!S_ISREG(st.st_mode)||st.st_size<2||st.st_size>TAG_JSON_MAX){fclose(f);return 0;}char*b=malloc((size_t)st.st_size+1);if(!b){fclose(f);return 0;}size_t n=fread(b,1,(size_t)st.st_size,f);int ok=n==(size_t)st.st_size&&!ferror(f)&&tag_snapshot_parse(b,n,out);fclose(f);free(b);return ok;}
static char*config_trim(char*s){while(*s&&isspace((unsigned char)*s))s++;char*e=s+strlen(s);while(e>s&&isspace((unsigned char)e[-1]))*--e=0;return s;}
int tag_mount_load(const char*path,tag_mount*out){
 if(!path||!out)return 0;tag_mount m={.max_age_us=500000};FILE*f=fopen(path,"r");if(!f)return 0;char line[512];int valid=1;unsigned seen=0;
 const char*keys[]={"calibration_enabled","camera_to_body_verified","eye","frame_max_age_ms","camera_delay_ms","camera_x_m","camera_y_m","camera_z_m","camera_roll_deg","camera_pitch_deg","camera_yaw_deg"};
 while(fgets(line,sizeof(line),f)){
  if(!strchr(line,'\n')&&!feof(f)){valid=0;break;}char*k=config_trim(line);if(!*k||*k=='#')continue;char*eq=strchr(k,'=');if(!eq){valid=0;break;}*eq=0;k=config_trim(k);char*v=eq+1,*comment=strchr(v,'#');if(comment)*comment=0;v=config_trim(v);
  unsigned i;for(i=0;i<sizeof(keys)/sizeof(keys[0]);i++)if(!strcmp(k,keys[i]))break;
  if(i==sizeof(keys)/sizeof(keys[0]))continue; /* 检测模块持有的内参等配置。 */
  if(seen&(1u<<i)){valid=0;break;}seen|=1u<<i;char*end;errno=0;double d=strtod(v,&end);
  if(errno||end==v||*config_trim(end)||!isfinite(d)){valid=0;break;}
  if(i<=2&&(d!=0&&d!=1)){valid=0;break;}
  switch(i){
  case 0:m.calibrated=(int)d;break;case 1:m.verified=(int)d;break;case 2:m.eye=(int)d;break;
  case 3:if(d<50||d>2000)valid=0;else m.max_age_us=(uint64_t)(d*1000);break;
  case 4:if(d<0||d>1000)valid=0;else m.delay_us=(uint64_t)(d*1000);break;
  case 5:m.x=d;break;case 6:m.y=d;break;case 7:m.z=d;break;
  case 8:m.roll=d*.017453292519943295;break;case 9:m.pitch=d*.017453292519943295;break;case 10:m.yaw=d*.017453292519943295;break;
  }
  if(!valid)break;
 }
 if(ferror(f))valid=0;fclose(f);
 if(!valid||fabs(m.x)>5||fabs(m.y)>5||fabs(m.z)>5||fabs(m.roll)>6.283185307179587||fabs(m.pitch)>6.283185307179587||fabs(m.yaw)>6.283185307179587)return 0;
 /* 安装核验需显式给出，缺失坐标不自动设零；仅检测或未标定模板可禁用该标记。
 */
 if(m.verified&&((seen&0x7e4u)!=0x7e4u))return 0;
 *out=m;return 1;
}
static void rotate(const tag_mount*m,double x,double y,double z,double*out){/* 光学到车体先映射为(z,-x,-y)，再施加Rz偏航、Ry俯仰及Rx横滚。 */double a=z,b=-x,c=-y,cr=cos(m->roll),sr=sin(m->roll),cp=cos(m->pitch),sp=sin(m->pitch),cy=cos(m->yaw),sy=sin(m->yaw);double bb=cr*b-sr*c,cc=sr*b+cr*c,aa=cp*a+sp*cc;cc=-sp*a+cp*cc;out[0]=cy*aa-sy*bb;out[1]=sy*aa+cy*bb;out[2]=cc;}
int tag_capture_pose(const tag_snapshot*s,const tag_mount*m,const lab_localization_binding*b,const lab_real_context*context,lab_pose*pose,char reason[80]){
 lab_real_meta meta={0};if(!s||!m||!b||!context||!pose||!reason)return 0;
 if(s->frame_us<m->delay_us||!b->ops||!b->ops->pose_at||b->ops->pose_at(b->user,s->frame_us-m->delay_us,&meta,pose)!=LAB_OK||meta.time_us!=s->frame_us-m->delay_us||memcmp(&meta.context,context,sizeof(*context))||!isfinite(pose->x_m)||!isfinite(pose->y_m)||!isfinite(pose->yaw_rad)||(meta.flags&(LAB_REAL_TIME_VALID|LAB_REAL_DATA_VALID))!=(LAB_REAL_TIME_VALID|LAB_REAL_DATA_VALID)||meta.time_uncertainty_us>250000){snprintf(reason,80,"TAG_CAPTURE_POSE_NOT_IN_HISTORY");return 0;}return 1;
}
static int deny(tag_task*t,const char*why){snprintf(t->state,sizeof(t->state),"REJECTED");snprintf(t->reason,sizeof(t->reason),"%s",why);t->active=0;return 0;}
static const tag_observation*usable(const tag_snapshot*s,const tag_mount*m,int id,uint64_t now,const char**why){if(!m->calibrated||!s->calibrated){*why="TAG_CAMERA_NOT_CALIBRATED";return NULL;}if(!m->verified){*why="TAG_CAMERA_MOUNT_NOT_VERIFIED";return NULL;}if(m->eye!=s->eye){*why="TAG_CAMERA_EYE_MISMATCH";return NULL;}if(!s->sequence||!s->frame_us||s->frame_us>now||now-s->frame_us>m->max_age_us||s->generated_us>now||s->generated_us<s->frame_us){*why="TAG_IMAGE_STALE_OR_TIME_INVALID";return NULL;}for(int i=0;i<s->count;i++)if(s->tags[i].id==id){const tag_observation*t=&s->tags[i];if(!t->confirmed){*why="TAG_NEEDS_THREE_NEW_FRAMES";return NULL;}if(!t->pose_valid){*why="TAG_POSE_INVALID";return NULL;}if(t->error<0||t->error>3||t->margin<10||hypot(hypot(t->nx,t->ny),t->nz)<.8||hypot(hypot(t->nx,t->ny),t->nz)>1.2){*why="TAG_POSE_QUALITY_REJECTED";return NULL;}return t;}*why="TAG_NOT_VISIBLE";return NULL;}
int tag_goal_compute(const tag_snapshot*s,const tag_mount*m,int id,double standoff,uint64_t now,const lab_pose*pose,double front,tag_task*out){if(!s||!m||!pose||!out)return 0;tag_task t={.id=id,.standoff=standoff,.frame_us=s->frame_us};const char*why;const tag_observation*o=usable(s,m,id,now,&why);if(!o){*out=t;return deny(out,why);}if(!isfinite(standoff)||standoff<.15||standoff>3||front<=0||front>2){*out=t;return deny(out,"TAG_INVALID_STANDOFF_OR_BODY");}
 double p[3],normal[3];rotate(m,o->x,o->y,o->z,p);rotate(m,-o->nx,-o->ny,-o->nz,normal);double norm=hypot(normal[0],normal[1]);if(norm<.5){*out=t;return deny(out,"TAG_PLANE_NOT_VERTICAL_ENOUGH");}normal[0]/=norm;normal[1]/=norm;
 /* 拒绝背面，光学位姿法线应朝向可见相机。 */
 if(p[0]*normal[0]+p[1]*normal[1]>=0){*out=t;return deny(out,"TAG_PLANE_NORMAL_AMBIGUOUS");}
 double ca=cos(pose->yaw_rad),sa=sin(pose->yaw_rad);p[0]+=m->x;p[1]+=m->y;t.target_x=pose->x_m+ca*p[0]-sa*p[1];t.target_y=pose->y_m+sa*p[0]+ca*p[1];double nx=ca*normal[0]-sa*normal[1],ny=sa*normal[0]+ca*normal[1];t.goal_x=t.target_x+nx*(standoff+front);t.goal_y=t.target_y+ny*(standoff+front);t.goal_yaw=atan2(-ny,-nx);t.active=1;snprintf(t.state,sizeof(t.state),"PREVIEW_PENDING");snprintf(t.reason,sizeof(t.reason),"TAG_CONFIRMED_WAIT_ROUTE");*out=t;return 1;}
int tag_task_reobserve(tag_task*t,const tag_snapshot*s,const tag_mount*m,uint64_t now,const lab_pose*pose){tag_task check;if(!t||!t->active)return 0;if(!tag_goal_compute(s,m,t->id,t->standoff,now,pose,.1665,&check)){snprintf(t->state,sizeof(t->state),"WAIT_REOBSERVE");snprintf(t->reason,sizeof(t->reason),"%.79s",check.reason);return 0;}if(hypot(check.target_x-t->target_x,check.target_y-t->target_y)>.2){snprintf(t->state,sizeof(t->state),"WAIT_REOBSERVE");snprintf(t->reason,sizeof(t->reason),"TAG_WORLD_POSITION_CHANGED");return 0;}double angle=atan2(sin(check.goal_yaw-pose->yaw_rad),cos(check.goal_yaw-pose->yaw_rad));if(hypot(check.goal_x-pose->x_m,check.goal_y-pose->y_m)>.15||fabs(angle)>.2617993877991494){snprintf(t->state,sizeof(t->state),"WAIT_REOBSERVE");snprintf(t->reason,sizeof(t->reason),"TAG_FINAL_POSE_OUTSIDE_TOLERANCE_REPREVIEW");return 0;}t->frame_us=s->frame_us;snprintf(t->state,sizeof(t->state),"ARRIVED");snprintf(t->reason,sizeof(t->reason),"TAG_REOBSERVED_AT_NAV_GOAL");return 1;}
void tag_task_cancel(tag_task*t,const char*reason){if(!t)return;t->active=0;snprintf(t->state,sizeof(t->state),"CANCELLED");snprintf(t->reason,sizeof(t->reason),"%.79s",reason?reason:"USER_CANCEL");}
