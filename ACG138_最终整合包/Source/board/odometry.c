#include "odometry.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#define PI 3.14159265358979323846
static double wrap(double x){return remainder(x,2*PI);}
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static uint32_t crc(const uint8_t *p,size_t n){uint32_t c=~0u;for(size_t i=0;i<n;i++){c^=p[i];for(int k=0;k<8;k++)c=(c>>1)^((c&1)?0xedb88320u:0);}return ~c;}
void odom_default_config(struct odom_config *c){
 memset(c,0,sizeof(*c));for(int i=0;i<4;i++){c->mm_per_count[i]=PI*67.5/1320;c->signs[i]=(i&1)?-1:1;}
 c->left_mask=5;c->track_mm=290;c->yaw_sign=1;c->correction_tau=.35;
}
int odom_load_config(struct odom_config *c,const char *path){
 FILE*f=fopen(path,"r");if(!f)return -1;
 struct odom_config next=*c;char line[256],key[64],tail;double v;int bad=0;
 while(fgets(line,sizeof(line),f)){
  if(line[0]=='#'||line[0]=='\n'||line[0]=='\r')continue;
  if(sscanf(line," %63[^=]= %lf %c",key,&v,&tail)!=2||!isfinite(v)){bad=1;break;}
  int known=0;
  if(!strcmp(key,"track_mm"))next.track_mm=v;
  else if(!strcmp(key,"yaw_sign"))next.yaw_sign=v;
  else if(!strcmp(key,"yaw_offset_deg"))next.yaw_offset=v*PI/180;
  else if(!strcmp(key,"correction_tau"))next.correction_tau=v;
  else if(!strcmp(key,"left_mask")){if(v!=trunc(v)||v<1||v>14)bad=1;else next.left_mask=(int)v;}
  else if(!strcmp(key,"calibrated")){if(v!=0&&v!=1)bad=1;else next.calibrated=(int)v;}
  else {
   for(int i=0;i<4;i++){char name[64];snprintf(name,sizeof(name),"m%d_sign",i+1);
    if(!strcmp(key,name)){if(v!=1&&v!=-1)bad=1;else next.signs[i]=(int)v;known=1;}
    snprintf(name,sizeof(name),"m%d_mm_per_count",i+1);
    if(!strcmp(key,name)){next.mm_per_count[i]=v;known=1;}
   }
   if(!known){bad=1;break;}
  }
 }
 fclose(f);
 if(next.track_mm<50||next.track_mm>1000||fabs(next.yaw_sign)!=1||next.correction_tau<.05||next.correction_tau>10||
    next.left_mask<1||next.left_mask>14||next.calibrated<0||next.calibrated>1)bad=1;
 int left=0;for(int i=0;i<4;i++){left+=(next.left_mask>>i)&1;if(abs(next.signs[i])!=1||next.mm_per_count[i]<.01||next.mm_per_count[i]>5)bad=1;}
 if(left!=2)bad=1;
 if(!bad)*c=next;return bad?-1:0;
}
void odom_init(struct odom_model *m,const struct odom_config *c){memset(m,0,sizeof(*m));m->config=*c;m->state.calibrated=c->calibrated;}
static void point(struct odom_model*m,int force){
 struct odom_state*s=&m->state;
 if(s->points&&!force&&hypot(s->x-s->trace[s->points-1].x,s->y-s->trace[s->points-1].y)<.02)return;
 if(s->points==ODOM_POINTS){for(unsigned i=0;i<ODOM_POINTS/2;i++)s->trace[i]=s->trace[i*2];s->points=ODOM_POINTS/2;}
 s->trace[s->points++]=(struct odom_point){s->x,s->y};
}
static void baseline(struct odom_model*m,const uint8_t*p,uint64_t now,int reset){
 if(reset){uint32_t session=m->state.session+1,accepted=m->state.accepted,rejected=m->state.rejected;
  memset(&m->state,0,sizeof(m->state));m->state.session=session;m->state.accepted=accepted;m->state.rejected=rejected;
  m->state.calibrated=m->config.calibrated;m->have_origin=0;m->have_imu=0;}
 for(int i=0;i<4;i++){m->previous[i]=u32(p+20+4*i);m->state.raw_counts[i]=(int32_t)m->previous[i];}
 m->sequence=u32(p+8);m->fpga_ms=u32(p+12);m->epoch=u32(p+16);m->imu_token=u32(p+48);m->have_packet=1;m->state.stamp_ms=now;
}
int odom_push(struct odom_model*m,const uint8_t*p,size_t n,uint64_t now){
 struct odom_state*s=&m->state;
 if(n!=64||memcmp(p,"OD01",4)||p[4]!=1||p[5]>1||p[6]>15||p[7]>7||u32(p+52)||u32(p+56)||crc(p,60)!=u32(p+60)){
  s->rejected++;return -1;
 }
 uint32_t seq=u32(p+8),ms=u32(p+12),epoch=u32(p+16),token=u32(p+48),dt=ms-m->fpga_ms;
 // 板卡复位、S7边沿或IMU重新校准会建立新的局部原点。
 int reboot=m->have_packet&&(int32_t)(ms-m->fpga_ms)<0&&(int32_t)(seq-m->sequence)<0&&ms<1000&&now>m->state.stamp_ms+300;
 if(m->have_packet&&!reboot&&(int32_t)(seq-m->sequence)<=0){s->rejected++;return -1;}
 int restart=m->have_packet&&(epoch!=m->epoch||token!=m->imu_token||reboot);
 if(!m->have_packet||restart){baseline(m,p,now,1);s=&m->state;}
 else if((int32_t)(seq-m->sequence)<=0){s->rejected++;return -1;}
 float yawf;uint32_t yf=u32(p+36);memcpy(&yawf,&yf,4);
 double yaw=wrap((double)yawf*m->config.yaw_sign+m->config.yaw_offset);
 int imu_good=p[7]==0&&u32(p+40)<=150&&isfinite(yawf)&&fabs(yawf)<1000000;
 if(p[5]||p[6]!=15){baseline(m,p,now,0);s->quality=ODOM_WAIT;s->accepted++;return 0;}
 if(!m->have_origin){
  baseline(m,p,now,0);s->accepted++;
  if(imu_good){m->have_origin=1;m->have_imu=1;m->previous_imu_yaw=yaw;m->imu_sequence=u32(p+44);s->origin_yaw=yaw;s->quality=m->config.calibrated?ODOM_FUSED:ODOM_UNCALIBRATED;point(m,1);}
  else s->quality=ODOM_WAIT;
  return 0;
 }
 if(!dt||dt>250){baseline(m,p,now,0);s->gaps++;s->quality=ODOM_GAP;s->uncertainty_m+=.05;point(m,1);return 0;}
 double d[4];int implausible=0;
 for(int i=0;i<4;i++){
  int32_t delta=(int32_t)(u32(p+20+4*i)-m->previous[i]);
  d[i]=delta*m->config.signs[i]*m->config.mm_per_count[i]/1000;
  if(fabs(d[i])>dt*.001*1.5+.005)implausible=1;
 }
 if(implausible){baseline(m,p,now,0);s->rejected++;s->gaps++;s->quality=ODOM_GAP;s->uncertainty_m+=.05;point(m,1);return -1;}
 double dl=0,dr=0;int li[2],ri[2],nl=0,nr=0;
 for(int i=0;i<4;i++)if((m->config.left_mask>>i)&1){dl+=d[i];li[nl++]=i;}else{dr+=d[i];ri[nr++]=i;}
 dl/=2;dr/=2;
 double seconds=dt*.001,ds=(dl+dr)/2,dtheta=(dr-dl)/(m->config.track_mm/1000),pred=wrap(s->heading+dtheta);
 int slip=fabs(d[li[0]]-d[li[1]])>fmax(.003,1.0*seconds)||fabs(d[ri[0]]-d[ri[1]])>fmax(.003,1.0*seconds);
 int used=0;
 if(imu_good &&u32(p+44)!=m->imu_sequence){
  double step=wrap(yaw-m->previous_imu_yaw),innovation=wrap(yaw-s->origin_yaw-pred);
  if((!m->have_imu||fabs(step)<.30)&&fabs(innovation)<.70){
   pred=wrap(pred+(1-exp(-seconds/m->config.correction_tau))*innovation);used=1;
  }else s->imu_rejected++;
  m->previous_imu_yaw=yaw;m->imu_sequence=u32(p+44);m->have_imu=1;
 }
 double turn=wrap(pred-s->heading),scale=fabs(turn)<1e-7?1:sin(turn/2)/(turn/2);
 double heading_mid=s->heading+turn/2;
 s->x+=ds*scale*cos(heading_mid);s->y+=ds*scale*sin(heading_mid);s->heading=pred;
 s->total_distance+=fabs(ds);s->uncertainty_m+=fabs(ds)*(used?.02:.05)+(slip?.01:0);
 s->slip=slip;s->imu_used=used;
 s->quality=!m->config.calibrated?ODOM_UNCALIBRATED:slip?ODOM_SLIP:imu_good?ODOM_FUSED:ODOM_WHEELS;
 baseline(m,p,now,0);s->accepted++;point(m,0);return 1;
}
const char*odom_quality_text(unsigned q){
 static const char*const text[]={"等待定位数据","未标定估计","IMU融合估计","仅编码器估计","定位数据过期","数据中断·轨迹有缺口","疑似轮滑"};return q<7?text[q]:"定位异常";
}
static void append(char*out,size_t size,size_t*pos,const char*fmt,...){
 if(*pos>=size)return;va_list ap;va_start(ap,fmt);int n=vsnprintf(out+*pos,size-*pos,fmt,ap);va_end(ap);
 if(n<0||((size_t)n)>=size-*pos)*pos=size;else *pos+=(size_t)n;
}
size_t odom_json(char*out,size_t size,const struct odom_state*s,uint64_t now){
 size_t pos=0;unsigned q=!s->accepted?ODOM_WAIT:now<s->stamp_ms||now-s->stamp_ms>500?ODOM_STALE:s->quality;
 append(out,size,&pos,"{\"quality\":%u,\"status\":\"%s\",\"calibrated\":%s,\"x\":%.4f,\"y\":%.4f,\"heading_deg\":%.2f,\"distance_m\":%.4f,\"age_ms\":%llu,\"session\":%u,\"uncertainty_m\":%.3f,\"gaps\":%u,\"imu_rejected\":%u,\"counts\":[%d,%d,%d,%d],\"trace\":[",
  q,odom_quality_text(q),s->calibrated?"true":"false",s->x,s->y,s->heading*180/PI,s->total_distance,
  (unsigned long long)(now>=s->stamp_ms?now-s->stamp_ms:0),s->session,s->uncertainty_m,s->gaps,s->imu_rejected,s->raw_counts[0],s->raw_counts[1],s->raw_counts[2],s->raw_counts[3]);
 for(unsigned i=0;i<s->points&&i<ODOM_POINTS;i++)append(out,size,&pos,"%s[%.3f,%.3f]",i?",":"",s->trace[i].x,s->trace[i].y);
 append(out,size,&pos,"]}");return pos<size?pos:0;
}
