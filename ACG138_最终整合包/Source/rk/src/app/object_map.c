#define _POSIX_C_SOURCE 200809L
#include "object_readonly.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static char*trim(char*s){while(*s&&isspace((unsigned char)*s))s++;char*e=s+strlen(s);while(e>s&&isspace((unsigned char)e[-1]))*--e=0;return s;}
int obj_mount_load(const char*path,obj_mount*out){if(!path||!out)return 0;FILE*f=fopen(path,"r");if(!f)return 0;obj_mount m={.max_age_us=500000};unsigned seen=0;int ok=1;char line[512];const char*keys[]={"stereo_enabled","camera_to_body_verified","reference_eye","frame_max_age_ms","camera_delay_ms","camera_x_m","camera_y_m","camera_z_m","camera_roll_deg","camera_pitch_deg","camera_yaw_deg"};
 while(fgets(line,sizeof(line),f)){if(!strchr(line,'\n')&&!feof(f)){ok=0;break;}char*k=trim(line);if(!*k||*k=='#')continue;char*eq=strchr(k,'=');if(!eq){ok=0;break;}*eq=0;k=trim(k);char*v=eq+1,*c=strchr(v,'#');if(c)*c=0;v=trim(v);unsigned i;for(i=0;i<11;i++)if(!strcmp(k,keys[i]))break;if(i==11)continue;if(seen&(1u<<i)){ok=0;break;}seen|=1u<<i;char*end;errno=0;double d=strtod(v,&end);if(errno||end==v||*trim(end)||!isfinite(d)||(i<=2&&d!=0&&d!=1)){ok=0;break;}
  switch(i){case 0:m.enabled=(int)d;break;case 1:m.verified=(int)d;break;case 2:m.eye=(int)d;break;case 3:if(d<50||d>2000)ok=0;else m.max_age_us=(uint64_t)(d*1000);break;case 4:if(d<0||d>1000)ok=0;else m.delay_us=(uint64_t)(d*1000);break;case 5:m.x=d;break;case 6:m.y=d;break;case 7:m.z=d;break;case 8:m.roll=d*.017453292519943295;break;case 9:m.pitch=d*.017453292519943295;break;case 10:m.yaw=d*.017453292519943295;break;}
  if(!ok)break;
 }if(ferror(f))ok=0;fclose(f);if(!ok||fabs(m.x)>5||fabs(m.y)>5||fabs(m.z)>5||fabs(m.roll)>6.283185307179587||fabs(m.pitch)>6.283185307179587||fabs(m.yaw)>6.283185307179587||(m.verified&&(seen&0x7e4u)!=0x7e4u))return 0;*out=m;return 1;}
static void body(const obj_mount*m,const obj_detection*d,double p[3]){double x=d->z,y=-d->x,z=-d->y;double cr=cos(m->roll),sr=sin(m->roll),cp=cos(m->pitch),sp=sin(m->pitch),cy=cos(m->yaw),sy=sin(m->yaw);double yy=cr*y-sr*z,zz=sr*y+cr*z,xx=cp*x+sp*zz;zz=-sp*x+cp*zz;p[0]=cy*xx-sy*yy+m->x;p[1]=sy*xx+cy*yy+m->y;p[2]=zz+m->z;}
static void why(obj_map_state*s,const char*reason){snprintf(s->reason,sizeof(s->reason),"%s",reason);if(strcmp(reason,"OBJECT_MAP_UPDATED"))for(int i=0;i<s->count;i++)s->entries[i].seen=0;}
void obj_map_update(obj_map_state*s,const obj_frame*f,const obj_mount*m,const lab_localization_view*v,const lab_localization_binding*b,uint64_t now){if(!s||!v)return;
 if(!s->initialized||memcmp(&s->context,&v->context,sizeof(v->context))||s->map_id!=v->map.map_id){memset(s,0,sizeof(*s));s->initialized=1;s->context=v->context;s->map_id=v->map.map_id;why(s,"OBJECT_MAP_CONTEXT_CHANGED");}
 if(!f){for(int i=0;i<s->count;i++)s->entries[i].seen=0;why(s,"OBJECT_STATE_UNAVAILABLE");return;}
 if(!f->epoch){why(s,"OBJECT_FRAME_STALE");return;}
 if(s->epoch!=f->epoch){s->epoch=f->epoch;s->last_frame=0;s->count=0;why(s,"OBJECT_TRACKING_EPOCH_CHANGED");}
 for(int i=0;i<s->count;i++)s->entries[i].selected=s->entries[i].epoch==f->epoch&&s->entries[i].track_id==f->selected;
 if(!m){why(s,"OBJECT_CONFIG_INVALID");return;}
 if(!m->enabled||!f->calibrated){why(s,"OBJECT_STEREO_UNCALIBRATED");return;}
 if(!m->verified){why(s,"OBJECT_CAMERA_MOUNT_NOT_VERIFIED");return;}
 if(m->eye!=f->eye){why(s,"OBJECT_REFERENCE_EYE_MISMATCH");return;}
 if(!f->frame_id||!f->epoch||!f->frame_us||f->frame_us>now||now-f->frame_us>m->max_age_us||f->generated_us<f->frame_us||f->generated_us>now){for(int i=0;i<s->count;i++)s->entries[i].seen=0;why(s,"OBJECT_FRAME_STALE");return;}
 if(f->frame_id==s->last_frame){return;}if(f->frame_id<s->last_frame){why(s,"OBJECT_FRAME_REORDERED");return;}
 for(int i=0;i<s->count;i++)s->entries[i].seen=0;
 if(!v->has_pose||!v->map.map_id||(v->state!=LAB_REAL_LOC_TRACKING&&!v->odometry_fallback)){why(s,"OBJECT_WAIT_LOCALIZATION");return;}
 if(!isfinite(v->map_odom_x_m)||!isfinite(v->map_odom_y_m)||!isfinite(v->map_odom_yaw_rad)){why(s,"OBJECT_MAP_TRANSFORM_INVALID");return;}
 lab_pose pose={0};lab_real_meta meta={0};uint64_t stamp=f->frame_us>=m->delay_us?f->frame_us-m->delay_us:0;
 if(!stamp||!b||!b->ops||!b->ops->pose_at||b->ops->pose_at(b->user,stamp,&meta,&pose)!=LAB_OK||meta.time_us!=stamp||memcmp(&meta.context,&v->context,sizeof(v->context))||(meta.flags&(LAB_REAL_TIME_VALID|LAB_REAL_DATA_VALID))!=(LAB_REAL_TIME_VALID|LAB_REAL_DATA_VALID)||meta.time_uncertainty_us>250000||!isfinite(pose.x_m)||!isfinite(pose.y_m)||!isfinite(pose.yaw_rad)||!isfinite(pose.uncertainty_m)||pose.uncertainty_m<0){why(s,"OBJECT_CAPTURE_POSE_NOT_IN_HISTORY");return;}
 int added=0;for(int q=0;q<f->count;q++){const obj_detection*d=&f->objects[q];if(!d->observed||!d->confirmed||!d->depth_valid||strcmp(d->tracking,"CONFIRMED"))continue;double p[3];body(m,d,p);double ca=cos(pose.yaw_rad),sa=sin(pose.yaw_rad);double mx=pose.x_m+ca*p[0]-sa*p[1],my=pose.y_m+sa*p[0]+ca*p[1];double dx=mx-v->map_odom_x_m,dy=my-v->map_odom_y_m,mc=cos(v->map_odom_yaw_rad),ms=sin(v->map_odom_yaw_rad);
  int k;for(k=0;k<s->count;k++)if(s->entries[k].epoch==f->epoch&&s->entries[k].track_id==d->track_id)break;
  if(k==s->count){if(s->count<OBJ_MAX)s->count++;else{uint64_t oldest=UINT64_MAX;int slot=-1;for(int j=0;j<s->count;j++)if(!s->entries[j].selected&&s->entries[j].frame_us<oldest){oldest=s->entries[j].frame_us;slot=j;}if(slot<0)continue;k=slot;}}
  obj_map_entry*e=&s->entries[k];*e=(obj_map_entry){.epoch=f->epoch,.frame_us=f->frame_us,.track_id=d->track_id,.selected=d->track_id==f->selected,.seen=1,.odom_x=mc*dx+ms*dy,.odom_y=-ms*dx+mc*dy,.error=hypot(d->error,pose.uncertainty_m)};snprintf(e->label,sizeof(e->label),"%s",d->label);added++;
 }
 s->last_frame=f->frame_id;why(s,added?"OBJECT_MAP_UPDATED":"OBJECT_NO_CONFIRMED_DEPTH");
}
static int add(char*b,size_t cap,size_t*n,const char*format,...){if(*n>=cap)return 0;va_list a;va_start(a,format);int k=vsnprintf(b+*n,cap-*n,format,a);va_end(a);if(k<0||(size_t)k>=cap-*n)return 0;*n+=(size_t)k;return 1;}
static void safe(const char*in,char*out,size_t cap){size_t n=0;while(in[n]&&n+1<cap){unsigned char c=(unsigned char)in[n];out[n]=(c<32||c=='"'||c=='\\')?'_':(char)c;n++;}out[n]=0;}
int obj_map_json(const obj_map_state*s,const lab_localization_view*v,uint64_t now,char*out,size_t cap,size_t*length){if(!s||!v||!out||!length)return 0;size_t n=0;int transform=isfinite(v->map_odom_x_m)&&isfinite(v->map_odom_y_m)&&isfinite(v->map_odom_yaw_rad)&&s->map_id==v->map.map_id&&!memcmp(&s->context,&v->context,sizeof(v->context));char reason[80];safe(s->reason,reason,sizeof(reason));
 if(!add(out,cap,&n,"{\"version\":1,\"map_id\":\"%llu\",\"map_version\":\"%llu\",\"published_us\":\"%llu\",\"reason\":\"%s\",\"objects\":[",(unsigned long long)v->map.map_id,(unsigned long long)v->map.map_version,(unsigned long long)now,reason))return 0;
 for(int i=0;i<s->count;i++){const obj_map_entry*e=&s->entries[i];uint64_t age=now>=e->frame_us?(now-e->frame_us)/1000:UINT64_MAX;int valid=transform&&isfinite(e->odom_x)&&isfinite(e->odom_y)&&isfinite(e->error);double c=cos(v->map_odom_yaw_rad),sn=sin(v->map_odom_yaw_rad),x=v->map_odom_x_m+c*e->odom_x-sn*e->odom_y,y=v->map_odom_y_m+sn*e->odom_x+c*e->odom_y;char label[64];safe(e->label,label,sizeof(label));
  if(!add(out,cap,&n,"%s{\"epoch\":\"%llu\",\"track_id\":%d,\"object_id\":\"OBJ_%llu_%d\",\"label\":\"%s\",\"selected\":%s,\"map_valid\":%s,\"map_x_m\":",i?",":"",(unsigned long long)e->epoch,e->track_id,(unsigned long long)e->epoch,e->track_id,label,e->selected?"true":"false",valid?"true":"false"))return 0;
  if(valid){if(!add(out,cap,&n,"%.9g,\"map_y_m\":%.9g",x,y))return 0;}else if(!add(out,cap,&n,"null,\"map_y_m\":null"))return 0;
  if(!add(out,cap,&n,",\"age_ms\":%llu,\"historical\":%s,\"reason\":\"%s\",\"position_error_m\":%.9g}",(unsigned long long)age,(!e->seen||age>500)?"true":"false",valid?((!e->seen||age>500)?"OBJECT_HISTORICAL_OBSERVATION":"OBJECT_MAP_CONFIRMED"):"OBJECT_MAP_TRANSFORM_INVALID",e->error))return 0;
 }if(!add(out,cap,&n,"]}"))return 0;*length=n;return 1;}
void obj_map_export(obj_map_state*s,const char*dir,const char*config,const lab_localization_view*v,const lab_localization_binding*b,uint64_t now){if(!s||!dir||dir[0]!='/'||!v)return;char input[512],output[512];if(snprintf(input,sizeof(input),"%s/objects-state.json",dir)>=(int)sizeof(input)||snprintf(output,sizeof(output),"%s/objects-map.json",dir)>=(int)sizeof(output))return;obj_frame*f=malloc(sizeof(*f));if(!f)return;obj_mount mount;int have=obj_frame_load(input,f),configured=obj_mount_load(config,&mount);obj_map_update(s,have?f:NULL,configured?&mount:NULL,v,b,now);free(f);char*json=malloc(OBJ_STATE_MAX);size_t n;if(json&&obj_map_json(s,v,now,json,OBJ_STATE_MAX,&n))obj_atomic_write(output,json,n);free(json);}
