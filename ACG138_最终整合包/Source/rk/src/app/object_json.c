#define _POSIX_C_SOURCE 200809L
#include "object_readonly.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif
/* 有界JSON分词器，字符串检查转义，访问函数拒绝类型不符、请求键重复和错误数值。
 */
typedef struct ot {size_t a,b;int end;char type;} ot;
typedef struct oj {const char*s;size_t n,p;int used;ot t[32768];} oj;
static void ws(oj*j){while(j->p<j->n&&isspace((unsigned char)j->s[j->p]))j->p++;}
static int value(oj*j,int depth){ws(j);if(depth>16||j->used>=32768||j->p>=j->n)return -1;int k=j->used++;ot*t=&j->t[k];t->a=j->p;t->type=j->s[j->p++];
 if(t->type=='{'||t->type=='['){char close=t->type=='{'?'}':']';ws(j);if(j->p<j->n&&j->s[j->p]==close)j->p++;else for(;;){if(t->type=='{'){int key=value(j,depth+1);if(key<0||j->t[key].type!='"')return -1;ws(j);if(j->p>=j->n||j->s[j->p++]!=':')return -1;}if(value(j,depth+1)<0)return -1;ws(j);if(j->p>=j->n)return -1;char c=j->s[j->p++];if(c==close)break;if(c!=',')return -1;}}
 else if(t->type=='"'){int done=0;while(j->p<j->n){unsigned char c=(unsigned char)j->s[j->p++];if(c=='"'){done=1;break;}if(c<32)return -1;if(c=='\\'){if(j->p>=j->n)return -1;c=(unsigned char)j->s[j->p++];if(c=='u'){for(int q=0;q<4;q++)if(j->p>=j->n||!isxdigit((unsigned char)j->s[j->p++]))return -1;}else if(!strchr("\"\\/bfnrt",c))return -1;}}if(!done)return -1;}
 else{while(j->p<j->n&&!isspace((unsigned char)j->s[j->p])&&!strchr(",]}",j->s[j->p]))j->p++;size_t z=j->p-t->a;const char*p=j->s+t->a;if(!((z==4&&!memcmp(p,"true",4))||(z==5&&!memcmp(p,"false",5))||(z==4&&!memcmp(p,"null",4)))){size_t q=0;if(q<z&&p[q]=='-')q++;if(q>=z)return -1;if(p[q]=='0')q++;else{if(p[q]<'1'||p[q]>'9')return -1;while(q<z&&isdigit((unsigned char)p[q]))q++;}if(q<z&&p[q]=='.'){q++;size_t a=q;while(q<z&&isdigit((unsigned char)p[q]))q++;if(q==a)return -1;}if(q<z&&(p[q]=='e'||p[q]=='E')){q++;if(q<z&&(p[q]=='+'||p[q]=='-'))q++;size_t a=q;while(q<z&&isdigit((unsigned char)p[q]))q++;if(q==a)return -1;}if(q!=z)return -1;}}
 t->b=j->p;t->end=j->used;return k;}
static oj*parse(const char*s,size_t n){if(!s||n<2||n>OBJ_SNAPSHOT_MAX)return NULL;oj*j=calloc(1,sizeof(*j));if(!j)return NULL;j->s=s;j->n=n;int k=value(j,0);ws(j);if(k!=0||j->p!=n||j->t[0].type!='{'){free(j);return NULL;}return j;}
static int eq(oj*j,int k,const char*s){return k>=0&&j->t[k].type=='"'&&j->t[k].b-j->t[k].a==strlen(s)+2&&!memcmp(j->s+j->t[k].a+1,s,strlen(s));}
static int field(oj*j,int k,const char*s){if(k<0||j->t[k].type!='{')return -1;int found=-1;for(int q=k+1;q<j->t[k].end;){int v=q+1;if(eq(j,q,s)){if(found>=0)return -1;found=v;}q=j->t[v].end;}return found;}
static int fields(oj*j,int k){int n=0;for(int q=k+1;q<j->t[k].end;q=j->t[q+1].end)n++;return n;}
static int str(oj*j,int k,char*out,size_t cap){if(k<0||j->t[k].type!='"')return 0;size_t a=j->t[k].a+1,b=j->t[k].b-1,n=0;for(size_t i=a;i<b;i++){unsigned char c=(unsigned char)j->s[i];if(c=='\\'){if(++i>=b)return 0;c=(unsigned char)j->s[i];if(c=='n'||c=='r'||c=='t')c=' ';else if(c!='"'&&c!='\\'&&c!='/')return 0;}if(c<32||n+1>=cap)return 0;out[n++]=(char)c;}out[n]=0;return 1;}
static int uintval(oj*j,int k,uint64_t*out,int quoted){if(k<0||(quoted?j->t[k].type!='"':j->t[k].type=='"'))return 0;char b[24],*e;size_t a=j->t[k].a+(quoted?1:0),n=j->t[k].b-j->t[k].a-(quoted?2:0);if(!n||n>=sizeof(b))return 0;memcpy(b,j->s+a,n);b[n]=0;for(size_t q=0;q<n;q++)if(!isdigit((unsigned char)b[q]))return 0;errno=0;*out=strtoull(b,&e,10);return !errno&&!*e;}
static int num(oj*j,int k,double*out){if(k<0||j->t[k].type=='"'||j->t[k].b-j->t[k].a>=80)return 0;char b[80],*e;size_t n=j->t[k].b-j->t[k].a;memcpy(b,j->s+j->t[k].a,n);b[n]=0;errno=0;double d=strtod(b,&e);if(errno||e==b||*e||!isfinite(d))return 0;*out=d;return 1;}
static int integer(oj*j,int k,int min,int max,int*out){double d;if(!num(j,k,&d)||d<min||d>max||floor(d)!=d)return 0;*out=(int)d;return 1;}
static int boolean(oj*j,int k,int*out){if(k<0)return 0;size_t n=j->t[k].b-j->t[k].a;const char*s=j->s+j->t[k].a;if(n==4&&!memcmp(s,"true",4)){*out=1;return 1;}if(n==5&&!memcmp(s,"false",5)){*out=0;return 1;}return 0;}
int obj_motion_meta(const char*s,size_t n,uint64_t*out){oj*j=parse(s,n);if(!j)return 0;uint64_t schema=0;int ok=uintval(j,field(j,0,"schema"),&schema,0)&&schema==1&&uintval(j,field(j,0,"published_us"),out,1);free(j);return ok;}
int obj_json_time(const char*s,size_t n,const char*key,uint64_t*out){oj*j=parse(s,n);if(!j)return 0;uint64_t version;int ok=uintval(j,field(j,0,"version"),&version,0)&&version==1&&uintval(j,field(j,0,key),out,1);free(j);return ok;}
/* 接近状态v2使用数值published_us和字符串task_id。 */
int obj_approach_u64(const char*s,size_t n,const char*key,uint64_t*out){oj*j=parse(s,n);if(!j)return 0;uint64_t version;int k=field(j,0,key);int ok=uintval(j,field(j,0,"version"),&version,0)&&version==2&&(uintval(j,k,out,0)||uintval(j,k,out,1));free(j);return ok;}
int obj_select_parse(const char*s,size_t n,int clear,obj_selection*out){if(!out||n>1024)return 0;oj*j=parse(s,n);if(!j)return 0;obj_selection q={.track_id=-1};int ok=0;if(fields(j,0)!=(clear?1:4)||!uintval(j,field(j,0,"seq"),&q.seq,0)||!q.seq||q.seq>UINT64_C(9007199254740991))goto done;
 if(!clear){if(!uintval(j,field(j,0,"epoch"),&q.epoch,1)||!q.epoch||!str(j,field(j,0,"snapshot_id"),q.snapshot_id,sizeof(q.snapshot_id))||!integer(j,field(j,0,"track_id"),1,2147483647,&q.track_id))goto done;char*e;errno=0;uint64_t ep=strtoull(q.snapshot_id,&e,10);if(errno||ep!=q.epoch||*e++!='-'||!isdigit((unsigned char)*e))goto done;errno=0;uint64_t fr=strtoull(e,&e,10);if(errno||*e||!fr)goto done;for(const char*p=q.snapshot_id;*p;p++)if(!isdigit((unsigned char)*p)&&*p!='-')goto done;}
 *out=q;ok=1;done:free(j);return ok;}
static int follow_exit_parse_v5(oj*j,int root,uint64_t frame,uint64_t time,object_follow_exit_v5*out){
 object_follow_exit_v5 e={0};int list;
 if(j->t[root].type!='{'||!boolean(j,field(j,root,"valid"),&e.valid)||!boolean(j,field(j,root,"bearing_valid"),&e.bearing_valid)||!integer(j,field(j,root,"direction"),-1,1,&e.direction)||!integer(j,field(j,root,"kind"),0,4,&e.kind)||!integer(j,field(j,root,"eye"),0,1,&e.eye)||!uintval(j,field(j,root,"source_us"),&e.source_us,1)||!uintval(j,field(j,root,"frame_id"),&e.frame_id,1)||e.source_us>time||e.frame_id>frame||!num(j,field(j,root,"confidence"),&e.confidence)||e.confidence<0||e.confidence>1||!str(j,field(j,root,"reason"),e.reason,sizeof(e.reason)))return 0;
 int br=field(j,root,"bearing_rad"),vr=field(j,root,"angular_velocity_rad_s");if(e.bearing_valid&&(br<0||!num(j,br,&e.bearing_rad)||fabs(e.bearing_rad)>3.141592654))return 0;
 if(vr>=0&&j->t[vr].type!='n'&&(!num(j,vr,&e.angular_velocity_rad_s)||fabs(e.angular_velocity_rad_s)>20))return 0;
 list=field(j,root,"keyframes");if(list<0||j->t[list].type!='[')return 0;
 for(int k=list+1;k<j->t[list].end;k=j->t[k].end){
  if(e.count>=OBJECT_FOLLOW_KEYFRAMES_V5_MAX||j->t[k].type!='{')return 0;object_follow_keyframe_v5*q=e.keyframes+e.count;
  if(!uintval(j,field(j,k,"frame_id"),&q->frame_id,1)||!q->frame_id||q->frame_id>frame||!uintval(j,field(j,k,"source_us"),&q->source_us,1)||!q->source_us||q->source_us>time||!integer(j,field(j,k,"eye"),0,1,&q->eye))return 0;
  if(e.count&&(q->source_us<=e.keyframes[e.count-1].source_us||q->frame_id<=e.keyframes[e.count-1].frame_id))return 0;
  int bb=field(j,k,"bbox"),i=0;if(bb<0||j->t[bb].type!='[')return 0;for(int v=bb+1;v<j->t[bb].end;v=j->t[v].end){if(i>=4||!num(j,v,q->bbox+i))return 0;i++;}if(i!=4||q->bbox[0]<0||q->bbox[1]<0||q->bbox[2]>960||q->bbox[3]>1080||q->bbox[2]<=q->bbox[0]||q->bbox[3]<=q->bbox[1])return 0;
  int b=field(j,k,"bearing_rad");if(b>=0&&j->t[b].type!='n'){if(!num(j,b,&q->bearing_rad)||fabs(q->bearing_rad)>3.141592654)return 0;q->bearing_valid=1;}
  int ray=field(j,k,"ray_direction");if(ray>=0&&j->t[ray].type!='n'){if(j->t[ray].type!='[')return 0;int c=0;double norm=0;for(int v=ray+1;v<j->t[ray].end;v=j->t[v].end){if(c>=3||!num(j,v,q->ray_direction+c))return 0;norm+=q->ray_direction[c]*q->ray_direction[c];c++;}if(c!=3||fabs(norm-1)>.02)return 0;q->ray_valid=1;}
  e.count++;
 }
 if(e.valid&&(!e.source_us||!e.frame_id||e.count<2||(e.kind!=1&&e.kind!=2)||e.direction!=(e.kind==1?1:-1)))return 0;
 /* 只有RK历史位姿桥接可将提示标为已补偿。 */
 e.compensated=0;*out=e;return 1;
}
/* 可选颜色或大小分析失败仅降低该属性质量，不丢弃其他仍有效的相机或控制帧。
 */
static void attributes_parse(oj*j,int root,uint64_t frame,uint64_t stamp,int eye,obj_attributes*out){
 memset(out,0,sizeof(*out));out->color_eye=eye;
 if(root<0||j->t[root].type!='{')return;
 obj_attributes a={0};a.color_eye=eye;int valid=0;
 int c=field(j,root,"color_valid");
 if(c>=0&&boolean(j,c,&valid)&&valid&&str(j,field(j,root,"color"),a.color,sizeof(a.color))&&obj_color_id(a.color)&&
    num(j,field(j,root,"color_support"),&a.color_support)&&a.color_support>=0&&a.color_support<=1&&
    uintval(j,field(j,root,"color_frame_id"),&a.color_frame_id,1)&&a.color_frame_id&&a.color_frame_id<=frame&&
    uintval(j,field(j,root,"color_source_us"),&a.color_source_us,1)&&a.color_source_us&&a.color_source_us<=stamp&&
    integer(j,field(j,root,"color_eye"),0,1,&a.color_eye)&&a.color_eye==eye){
  a.color_valid=1;int secondary=field(j,root,"secondary_color");
  if(secondary>=0&&(!str(j,secondary,a.secondary_color,sizeof(a.secondary_color))||(!obj_color_id(a.secondary_color)&&a.secondary_color[0]&&strcmp(a.secondary_color,"unknown"))))a.secondary_color[0]=0;
  if(!strcmp(a.secondary_color,"unknown"))a.secondary_color[0]=0;
 }
 valid=0;int z=field(j,root,"size_valid");
 if(z>=0&&boolean(j,z,&valid)&&valid&&str(j,field(j,root,"size"),a.size,sizeof(a.size))&&
    (!strcmp(a.size,"small")||!strcmp(a.size,"medium")||!strcmp(a.size,"large"))&&
    str(j,field(j,root,"size_source"),a.size_source,sizeof(a.size_source))&&a.size_source[0]&&!strstr(a.size_source,"PIXEL")&&
    num(j,field(j,root,"size_support"),&a.size_support)&&a.size_support>=0&&a.size_support<=1&&
    num(j,field(j,root,"width_m"),&a.width_m)&&a.width_m>0&&a.width_m<100&&
    num(j,field(j,root,"height_m"),&a.height_m)&&a.height_m>0&&a.height_m<100&&
    uintval(j,field(j,root,"size_frame_id"),&a.size_frame_id,1)&&a.size_frame_id&&a.size_frame_id<=frame&&
    uintval(j,field(j,root,"size_source_us"),&a.size_source_us,1)&&a.size_source_us&&a.size_source_us<=stamp)a.size_valid=1;
 *out=a;
}
static int frame_parse(const char*s,size_t n,obj_frame*out){if(!out)return 0;oj*j=parse(s,n);if(!j)return 0;obj_frame q={.selected=-1};int ok=0;uint64_t ver;int width,height;
 if(!uintval(j,field(j,0,"version"),&ver,0)||ver!=1||!uintval(j,field(j,0,"epoch"),&q.epoch,1)||!uintval(j,field(j,0,"frame_id"),&q.frame_id,1)||!uintval(j,field(j,0,"frame_time_us"),&q.frame_us,1)||!uintval(j,field(j,0,"generated_us"),&q.generated_us,1)||!eq(j,field(j,0,"time_source"),"RK_DEQUEUE")||!integer(j,field(j,0,"reference_eye"),0,1,&q.eye)||!integer(j,field(j,0,"width"),960,960,&width)||!integer(j,field(j,0,"height"),1080,1080,&height)||!boolean(j,field(j,0,"calibrated"),&q.calibrated)||!integer(j,field(j,0,"selected_track_id"),-1,2147483647,&q.selected))goto done;
 int a=field(j,0,"objects");if(a<0||j->t[a].type!='[')goto done;for(int k=a+1;k<j->t[a].end;k=j->t[k].end){if(q.count>=OBJ_MAX||j->t[k].type!='{')goto done;obj_detection*t=&q.objects[q.count];
  if(!integer(j,field(j,k,"track_id"),1,2147483647,&t->track_id)||!integer(j,field(j,k,"class_id"),0,80,&t->class_id)||!str(j,field(j,k,"label"),t->label,sizeof(t->label))||!str(j,field(j,k,"tracking"),t->tracking,sizeof(t->tracking))||!str(j,field(j,k,"depth_reason"),t->reason,sizeof(t->reason))||!boolean(j,field(j,k,"observed"),&t->observed)||!boolean(j,field(j,k,"confirmed"),&t->confirmed)||!boolean(j,field(j,k,"depth_valid"),&t->depth_valid))goto done;
  if(t->class_id==80){int manual=0;if(!boolean(j,field(j,k,"manual"),&manual)||!manual)goto done;}
  t->task_class_id=t->observed_class_id=t->class_id;
  int tc=field(j,k,"task_class_id"),oc=field(j,k,"observed_class_id"),bg=field(j,k,"binding_generation"),bwhy=field(j,k,"binding_reason");uint64_t generation=0;
  if((tc>=0&&!integer(j,tc,-1,80,&t->task_class_id))||(oc>=0&&!integer(j,oc,-1,80,&t->observed_class_id))||(bg>=0&&(!uintval(j,bg,&generation,0)||generation>UINT32_MAX))||(bwhy>=0&&!str(j,bwhy,t->binding_reason,sizeof(t->binding_reason))))goto done;
  t->binding_generation=(uint32_t)generation;
  int ot=field(j,k,"observation_time_us"),of=field(j,k,"observation_frame_id");
  if((ot>=0)!=(of>=0))goto done;
  if(ot>=0&&(!uintval(j,ot,&t->observation_time_us,1)||!uintval(j,of,&t->observation_frame_id,1)||t->observation_time_us>q.frame_us||t->observation_frame_id>q.frame_id||(t->observation_time_us==0)!=(t->observation_frame_id==0)))goto done;
  int fe=field(j,k,"follow_exit");if(fe>=0&&!follow_exit_parse_v5(j,fe,q.frame_id,q.frame_us,&t->follow_exit))goto done;
  if(strcmp(t->tracking,"CONFIRMED")&&strcmp(t->tracking,"TENTATIVE")&&strcmp(t->tracking,"PREDICTED")&&strcmp(t->tracking,"LOST"))goto done;
  t->eye=q.eye;t->geometry_eye=q.eye;t->depth_eye=q.eye;
  int ey=field(j,k,"eye"),ge=field(j,k,"geometry_eye"),de=field(j,k,"depth_eye");
  if((ey>=0&&!integer(j,ey,0,1,&t->eye))||(ge>=0&&!integer(j,ge,0,1,&t->geometry_eye))||(de>=0&&!integer(j,de,0,1,&t->depth_eye)))goto done;
  int dt=field(j,k,"depth_time_us"),df=field(j,k,"depth_frame_id");
  if(dt>=0&&j->t[dt].type!='n'&&(!uintval(j,dt,&t->depth_time_us,1)||t->depth_time_us>q.frame_us))goto done;
  if(df>=0&&j->t[df].type!='n'&&(!uintval(j,df,&t->depth_frame_id,1)||t->depth_frame_id>q.frame_id))goto done;
  if((t->depth_time_us==0)!=(t->depth_frame_id==0))goto done;
  int bb=field(j,k,"bbox"),sc=field(j,k,"score");
  if(bb>=0||sc>=0){if(bb<0||j->t[bb].type!='['||sc<0||!num(j,sc,&t->score)||t->score<0||t->score>1)goto done;int z=0;for(int v=bb+1;v<j->t[bb].end;v=j->t[v].end){if(z>=4||!num(j,v,&t->bbox[z++]))goto done;}if(z!=4||t->bbox[0]<0||t->bbox[1]<0||t->bbox[2]>960||t->bbox[3]>1080||t->bbox[2]<=t->bbox[0]||t->bbox[3]<=t->bbox[1])goto done;t->bbox_valid=1;}
  if(t->depth_valid){if(!q.calibrated||(!t->depth_time_us&&(!t->observed||!t->confirmed||strcmp(t->tracking,"CONFIRMED")))||!num(j,field(j,k,"x_m"),&t->x)||!num(j,field(j,k,"y_m"),&t->y)||!num(j,field(j,k,"z_m"),&t->z)||t->z<=0||t->z>100||fabs(t->x)>100||fabs(t->y)>100||!num(j,field(j,k,"position_error_m"),&t->error)||t->error<0||t->error>100)goto done;}
  int dc=field(j,k,"depth_candidates");if(dc>=0){if(j->t[dc].type!='[')goto done;for(int di=dc+1;di<j->t[dc].end;di=j->t[di].end){if(t->depth_candidate_count>=3||j->t[di].type!='{')goto done;unsigned ci=t->depth_candidate_count;double points=0;if(!num(j,field(j,di,"range_m"),&t->depth_candidate_range[ci])||!num(j,field(j,di,"error_m"),&t->depth_candidate_error[ci])||!num(j,field(j,di,"points"),&points)||t->depth_candidate_range[ci]<=0||t->depth_candidate_range[ci]>100||t->depth_candidate_error[ci]<0||t->depth_candidate_error[ci]>100||points<1||points>1000000||floor(points)!=points)goto done;t->depth_candidate_points[ci]=(unsigned)points;t->depth_candidate_count++;}}
  int cv=field(j,k,"coarse_valid");if(cv>=0){if(!boolean(j,cv,&t->coarse_valid))goto done;if(t->coarse_valid&&((!t->observed&&!t->depth_time_us)||!num(j,field(j,k,"coarse_range_m"),&t->coarse_range)||!num(j,field(j,k,"coarse_error_m"),&t->coarse_error)||t->coarse_range<=0||t->coarse_range>100||t->coarse_error<0||t->coarse_error>100))goto done;}
  int rc=field(j,k,"reacquire_candidate_valid");if(rc>=0){if(!boolean(j,rc,&t->reacquire_candidate_valid))goto done;
   int ri=field(j,k,"reacquire_candidate_track_id"),rr=field(j,k,"reacquire_candidate_reason");if(ri>=0&&!integer(j,ri,-1,2147483647,&t->reacquire_candidate_track_id))goto done;if(rr>=0&&!str(j,rr,t->reacquire_candidate_reason,sizeof(t->reacquire_candidate_reason)))goto done;
   int rb=field(j,k,"reacquire_candidate_bearing_valid");if(rb>=0&&!boolean(j,rb,&t->reacquire_candidate_bearing_valid))goto done;
   if(t->reacquire_candidate_valid){if(!integer(j,field(j,k,"reacquire_candidate_eye"),0,1,&t->reacquire_candidate_eye)||!uintval(j,field(j,k,"reacquire_candidate_time_us"),&t->reacquire_candidate_time_us,1)||!t->reacquire_candidate_time_us||t->reacquire_candidate_time_us>q.frame_us)goto done;
    if(t->reacquire_candidate_bearing_valid&&(!num(j,field(j,k,"reacquire_candidate_bearing_rad"),&t->reacquire_candidate_bearing_rad)||fabs(t->reacquire_candidate_bearing_rad)>3.141592654))goto done;
   }else t->reacquire_candidate_bearing_valid=0;
  }
  int br=field(j,k,"bearing_rad");if(br>=0&&j->t[br].type!='n'){if(!num(j,br,&t->bearing)||fabs(t->bearing)>3.141592654)goto done;t->bearing_valid=1;}
  int bl=field(j,k,"bearing_left_rad"),brt=field(j,k,"bearing_right_rad");
  if(bl>=0&&brt>=0&&j->t[bl].type!='n'&&j->t[brt].type!='n'){if(!num(j,bl,&t->bearing_left)||!num(j,brt,&t->bearing_right)||fabs(t->bearing_left)>3.141592654||fabs(t->bearing_right)>3.141592654)goto done;t->bearing_edges_valid=1;}
  int db=field(j,k,"depth_bearing_rad");if(db>=0&&j->t[db].type!='n'){if(!num(j,db,&t->depth_bearing)||fabs(t->depth_bearing)>3.141592654)goto done;t->depth_bearing_valid=1;}
  const char*origin_keys[]={"ray_origin_x","ray_origin_y","ray_origin_z"};int origin_fields=0;
  for(int z=0;z<3;z++){int oi=field(j,k,origin_keys[z]);if(oi>=0){if(!num(j,oi,&t->ray_origin[z])||fabs(t->ray_origin[z])>1)goto done;origin_fields++;}}
  if(origin_fields&&origin_fields!=3)goto done;t->ray_origin_valid=origin_fields==3;
  const char*ray_keys[]={"ray_direction","ray_left","ray_right"};int ray_count=0;
  double*ray_arrays[]={t->ray_direction,t->ray_left,t->ray_right};
  for(int ri=0;ri<3;ri++){int ra=field(j,k,ray_keys[ri]);if(ra<0||j->t[ra].type=='n')continue;
   if(j->t[ra].type!='[')goto done;int rn=0;double norm=0;
   for(int rv=ra+1;rv<j->t[ra].end;rv=j->t[rv].end){if(rn>=3||!num(j,rv,&ray_arrays[ri][rn]))goto done;norm+=ray_arrays[ri][rn]*ray_arrays[ri][rn];rn++;}
   if(rn!=3||fabs(norm-1)>.02)goto done;ray_count++;
  }if(ray_count&&ray_count!=3)goto done;t->ray_direction_valid=ray_count==3;
  int dbb=field(j,k,"depth_bbox");if(dbb>=0){if(j->t[dbb].type!='[')goto done;int z=0;for(int v=dbb+1;v<j->t[dbb].end;v=j->t[v].end){if(z>=4||!num(j,v,&t->depth_bbox[z++]))goto done;}if(z!=4)goto done;if(t->depth_time_us&&(t->depth_bbox[0]<0||t->depth_bbox[1]<0||t->depth_bbox[2]>960||t->depth_bbox[3]>1080||t->depth_bbox[2]<=t->depth_bbox[0]||t->depth_bbox[3]<=t->depth_bbox[1]))goto done;}
  int cr=field(j,k,"coarse_reason"),cq=field(j,k,"coarse_quality");if(cr>=0&&!str(j,cr,t->coarse_reason,sizeof(t->coarse_reason)))goto done;if(cq>=0&&!str(j,cq,t->coarse_quality,sizeof(t->coarse_quality)))goto done;
  attributes_parse(j,field(j,k,"attributes"),q.frame_id,q.frame_us,t->eye,&t->attributes);
  for(int z=0;z<q.count;z++)if(q.objects[z].track_id==t->track_id)goto done;q.count++;
 }*out=q;ok=1;done:free(j);return ok;}
int obj_frame_parse(const char*s,size_t n,obj_frame*out){return n<=OBJ_STATE_MAX&&frame_parse(s,n,out);}
int obj_snapshot_parse(const char*s,size_t n,obj_frame*out){return n<=OBJ_SNAPSHOT_MAX&&frame_parse(s,n,out);}
int obj_snapshot_matches(const char*s,size_t n,const obj_frame*f){
 obj_frame q;if(!f||!frame_parse(s,n,&q)||q.epoch!=f->epoch||q.frame_id!=f->frame_id||q.frame_us!=f->frame_us||q.selected!=f->selected)return 0;
 int expected=0;for(int i=0;i<f->count;i++)if(f->objects[i].eye==q.eye)expected++;
 if(q.count!=expected)return 0;
 for(int i=0;i<q.count;i++){const obj_detection*a=&q.objects[i],*b=NULL;if(a->eye!=q.eye)return 0;
  for(int z=0;z<f->count;z++)if(f->objects[z].track_id==a->track_id)b=&f->objects[z];
  if(!b||a->eye!=b->eye||a->class_id!=b->class_id||a->observed!=b->observed||a->confirmed!=b->confirmed||a->bbox_valid!=b->bbox_valid||memcmp(a->bbox,b->bbox,sizeof(a->bbox))||a->score!=b->score||strcmp(a->tracking,b->tracking))return 0;
  if(a->task_class_id!=b->task_class_id||a->observed_class_id!=b->observed_class_id||a->binding_generation!=b->binding_generation||a->observation_frame_id!=b->observation_frame_id||a->observation_time_us!=b->observation_time_us)return 0;
 }
 oj*j=parse(s,n);if(!j)return 0;char id[64],want[64];snprintf(want,sizeof(want),"%llu-%llu",(unsigned long long)f->epoch,(unsigned long long)f->frame_id);int im=field(j,0,"image_data_url");int ok=field(j,0,"association")<0&&str(j,field(j,0,"snapshot_id"),id,sizeof(id))&&!strcmp(id,want)&&im>=0&&j->t[im].type=='"'&&j->t[im].b-j->t[im].a>24&&(!memcmp(s+j->t[im].a+1,"data:image/png;base64,",22)||(j->t[im].b-j->t[im].a>25&&!memcmp(s+j->t[im].a+1,"data:image/jpeg;base64,",23)));free(j);return ok;
}
int obj_file_read(const char*path,size_t max,char**out,size_t*n){if(!path||!out||!n)return 0;FILE*f=NULL;
#ifndef _WIN32
 int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);if(fd>=0){f=fdopen(fd,"rb");if(!f)close(fd);}
#else
 f=fopen(path,"rb");
#endif
 if(!f)return 0;struct stat st;if(fstat(fileno(f),&st)||!S_ISREG(st.st_mode)||st.st_size<2||(uint64_t)st.st_size>max){fclose(f);return 0;}size_t z=(size_t)st.st_size;char*b=malloc(z+1);if(!b){fclose(f);return 0;}size_t got=fread(b,1,z,f);int valid=got==z&&!ferror(f);fclose(f);if(!valid){free(b);return 0;}b[z]=0;*out=b;*n=z;return 1;}
int obj_frame_load(const char*p,obj_frame*out){char*b;size_t n;if(!obj_file_read(p,OBJ_STATE_MAX,&b,&n))return 0;int ok=obj_frame_parse(b,n,out);free(b);return ok;}
int obj_atomic_write(const char*path,const char*data,size_t n){char tmp[1024];if(!path||strlen(path)+40>=sizeof(tmp)||n>OBJ_SNAPSHOT_MAX)return 0;
#ifndef _WIN32
 snprintf(tmp,sizeof(tmp),"%s.%ld.tmp",path,(long)getpid());int fd=open(tmp,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)return 0;FILE*f=fdopen(fd,"wb");if(!f){close(fd);unlink(tmp);return 0;}
#else
 snprintf(tmp,sizeof(tmp),"%s.tmp",path);FILE*f=fopen(tmp,"wb");if(!f)return 0;
#endif
 int ok=fwrite(data,1,n,f)==n;if(fclose(f))ok=0;
#ifdef _WIN32
 if(ok)remove(path);
#endif
 if(ok&&rename(tmp,path)==0)return 1;remove(tmp);return 0;}
