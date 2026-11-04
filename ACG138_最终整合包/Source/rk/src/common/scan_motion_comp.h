#ifndef ACG_R8_SCAN_MOTION_COMP_H
#define ACG_R8_SCAN_MOTION_COMP_H
/* 进程内运动去畸变，LT01首节点时间及SDK均匀采样相位均为估计，不是逐点硬件时间戳。位姿仅来自MPU与编码器服务原始里程计，不用SLAM校正或仿真真值。头文件实现供各快照构建共用。


 */
#include "lab_live.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#define R8_DESKEW_HISTORY 256u
#define R8_DESKEW_WAIT_US UINT64_C(80000)
#define R8_DESKEW_POSE_GAP_US UINT64_C(100000)
#define R8_DESKEW_PI 3.14159265358979323846

typedef struct r8_deskew_pose {uint64_t time_us,error_us;lab_pose pose;} r8_deskew_pose;
typedef struct r8_deskew_history {r8_deskew_pose poses[R8_DESKEW_HISTORY];unsigned count;uint64_t overwritten,rejected;lab_real_context context;int have_context;} r8_deskew_history;
typedef struct r8_deskew_result {
 unsigned corrected_points,invalid_points;uint32_t span_us;
 uint64_t first_us,reference_us,last_us,pose_error_us;
 double translation_m,yaw_rad,max_origin_m;
 int code;const char *reason;
} r8_deskew_result;
enum {R8_DESKEW_APPLIED=1,R8_DESKEW_WAIT=0,R8_DESKEW_NO_PROFILE=-1,
 R8_DESKEW_NO_HISTORY=-2,R8_DESKEW_HISTORY_GAP=-3,R8_DESKEW_INVALID=-4,
 R8_DESKEW_MOTION_LIMIT=-5,R8_DESKEW_ORDER_UNSUPPORTED=-6};
static inline double r8_deskew_wrap(double a){return remainder(a,2*R8_DESKEW_PI);}
static inline int r8_deskew_same_context(const lab_real_context*a,const lab_real_context*b){return a->source==b->source&&a->clock==b->clock&&a->timeline_id==b->timeline_id&&a->origin_epoch==b->origin_epoch;}
static inline int r8_deskew_pose_valid(const lab_live_sample*e){
 const lab_pose*p=&e->data.odometry;
 return e->type==LAB_EVENT_POSE&&e->meta.provider==LAB_REAL_ODOMETRY&&
  (e->meta.flags&3)==3&&e->meta.time_uncertainty_us!=UINT64_MAX&&
  (p->quality&LAB_POSE_VALID)&&!(p->quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))&&
  isfinite(p->x_m)&&isfinite(p->y_m)&&isfinite(p->yaw_rad)&&e->meta.context.clock==LAB_REAL_MONOTONIC;
}
static inline void r8_deskew_observe(r8_deskew_history*h,const lab_live_sample*e){
 if(!h||!e||e->type!=LAB_EVENT_POSE)return;
 if(!h->have_context||!r8_deskew_same_context(&h->context,&e->meta.context)){
  h->count=0;h->context=e->meta.context;h->have_context=1;
 }
 if(!r8_deskew_pose_valid(e)){h->count=0;++h->rejected;return;}
 if(h->count&&e->meta.time_us<=h->poses[h->count-1].time_us){++h->rejected;return;}
 if(h->count==R8_DESKEW_HISTORY){memmove(h->poses,h->poses+1,(R8_DESKEW_HISTORY-1)*sizeof(h->poses[0]));--h->count;++h->overwritten;}
 h->poses[h->count++]=(r8_deskew_pose){e->meta.time_us,e->meta.time_uncertainty_us,e->data.odometry};
}
static inline int r8_deskew_at(const r8_deskew_history*h,uint64_t t,lab_pose*out,uint64_t*error){
 if(!h->count||t<h->poses[0].time_us)return R8_DESKEW_NO_HISTORY;
 if(t>h->poses[h->count-1].time_us)return R8_DESKEW_WAIT;
 unsigned lo=0,hi=h->count-1;while(lo<hi){unsigned mid=lo+(hi-lo)/2;if(h->poses[mid].time_us<t)lo=mid+1;else hi=mid;}
 const r8_deskew_pose*b=h->poses+lo;
 if(b->time_us==t){*out=b->pose;if(error)*error=b->error_us;return R8_DESKEW_APPLIED;}
 if(!lo)return R8_DESKEW_NO_HISTORY;const r8_deskew_pose*a=b-1;
 if(b->time_us-a->time_us>R8_DESKEW_POSE_GAP_US)return R8_DESKEW_HISTORY_GAP;
 double f=(double)(t-a->time_us)/(double)(b->time_us-a->time_us);*out=a->pose;
 out->x_m+=f*(b->pose.x_m-a->pose.x_m);out->y_m+=f*(b->pose.y_m-a->pose.y_m);
 out->yaw_rad=r8_deskew_wrap(a->pose.yaw_rad+f*r8_deskew_wrap(b->pose.yaw_rad-a->pose.yaw_rad));
 if(error)*error=a->error_us>b->error_us?a->error_us:b->error_us;return R8_DESKEW_APPLIED;
}
static inline const char*r8_deskew_reason(int code){switch(code){
 case R8_DESKEW_APPLIED:return "APPLIED_SDK_FIRST_UNIFORM_PHASE_ESTIMATED";
 case R8_DESKEW_WAIT:return "WAIT_FUTURE_RAW_POSE";
 case R8_DESKEW_NO_PROFILE:return "SKIP_TIMING_PROFILE_UNAVAILABLE";
 case R8_DESKEW_NO_HISTORY:return "SKIP_RAW_POSE_HISTORY_TOO_SHORT";
 case R8_DESKEW_HISTORY_GAP:return "SKIP_RAW_POSE_INTERPOLATION_GAP";
 case R8_DESKEW_MOTION_LIMIT:return "SKIP_MOTION_OUTSIDE_SUPPORTED_ENVELOPE";
 case R8_DESKEW_ORDER_UNSUPPORTED:return "SKIP_NODE_ACQUISITION_ORDER_UNSUPPORTED";
 default:return "SKIP_INVALID_INPUT";}}
/* 检查原C1顺时针采集及一次回绕，补偿后点不重排，原采样索引就是时间标识。
 */
static inline int r8_deskew_order(const struct scan_frame*f){
 if(f->count<20)return 0;double total=0;unsigned backwards=0;
 for(unsigned i=1;i<f->count;i++){int d=(int)f->points[i].angle_q14-(int)f->points[i-1].angle_q14;if(d>32768)d-=65536;if(d< -32768)d+=65536;
  double a=d*(2*R8_DESKEW_PI/65536.);if(a<-.035)++backwards;if(a>0)total+=a;}
 return backwards==0&&total>5.2&&total<7.5;
}
/* 调用方提供独立输出存储，任一点不支持时原完整帧保持不变；射线原点和端点使用同一参考雷达坐标，消费者从各自原点投射。

 */
static inline int r8_deskew_apply(const r8_deskew_history*h,const struct scan_frame*in,
 struct scan_frame*out,uint64_t first,uint32_t span,const double mount[3],int profile,r8_deskew_result*d){
 memset(d,0,sizeof(*d));d->first_us=first;d->span_us=span;d->reference_us=first+span/2;d->last_us=first+span;
 int code=R8_DESKEW_INVALID;
 if(!profile){code=R8_DESKEW_NO_PROFILE;goto done;}
 if(!h||!in||!out||!mount||in->geometry_kind!=SCAN_GEOMETRY_RAW||!in->count||in->count>SCAN_MAX_POINTS||!span||span>250000||first>UINT64_MAX-span||!isfinite(mount[0])||!isfinite(mount[1])||!isfinite(mount[2]))goto done;
 if(!r8_deskew_order(in)){code=R8_DESKEW_ORDER_UNSUPPORTED;goto done;}
 lab_pose a,b,ref;uint64_t ea=0,eb=0,er=0;
 code=r8_deskew_at(h,first,&a,&ea);if(code!=R8_DESKEW_APPLIED)goto done;
 code=r8_deskew_at(h,first+span,&b,&eb);if(code!=R8_DESKEW_APPLIED)goto done;
 code=r8_deskew_at(h,first+span/2,&ref,&er);if(code!=R8_DESKEW_APPLIED)goto done;
 d->translation_m=hypot(b.x_m-a.x_m,b.y_m-a.y_m);d->yaw_rad=r8_deskew_wrap(b.yaw_rad-a.yaw_rad);
 d->pose_error_us=ea>eb?ea:eb;if(er>d->pose_error_us)d->pose_error_us=er;
 if(d->translation_m>.25||fabs(d->yaw_rad)>.80){code=R8_DESKEW_MOTION_LIMIT;goto done;}
 *out=*in;out->geometry_kind=SCAN_GEOMETRY_DESKEW_ORIGINS;
 double cr=cos(ref.yaw_rad),sr=sin(ref.yaw_rad),cm=cos(mount[2]),sm=sin(mount[2]);
 for(unsigned i=0;i<in->count;i++){
  const struct scan_point*p=in->points+i;out->origins_q4[i][0]=out->origins_q4[i][1]=0;out->raw_distance_q2[i]=p->distance_q2;
  if(!p->quality||!p->distance_q2){++d->invalid_points;continue;}
  uint64_t t=first+(uint64_t)span*i/(in->count-1);lab_pose at;uint64_t e=0;code=r8_deskew_at(h,t,&at,&e);if(code!=R8_DESKEW_APPLIED)goto done;
  if(e>d->pose_error_us)d->pose_error_us=e;
  double ci=cos(at.yaw_rad),si=sin(at.yaw_rad),dx=at.x_m-ref.x_m,dy=at.y_m-ref.y_m;
  double ox=cr*(dx+ci*mount[0]-si*mount[1])+sr*(dy+si*mount[0]+ci*mount[1])-mount[0];
  double oy=-sr*(dx+ci*mount[0]-si*mount[1])+cr*(dy+si*mount[0]+ci*mount[1])-mount[1];
  double osx=cm*ox+sm*oy,osy=-sm*ox+cm*oy;
  double origin=hypot(osx,osy);if(origin>d->max_origin_m)d->max_origin_m=origin;
  if(origin>8){code=R8_DESKEW_MOTION_LIMIT;goto done;}
  out->origins_q4[i][0]=(int16_t)llround(osx*4000.);out->origins_q4[i][1]=(int16_t)llround(osy*4000.);
  double angle=-p->angle_q14*(2*R8_DESKEW_PI/65536.),range=p->distance_q2/4000.;
  double lx=range*cos(angle),ly=range*sin(angle),bx=mount[0]+cm*lx-sm*ly,by=mount[1]+sm*lx+cm*ly;
  double wx=dx+ci*bx-si*by,wy=dy+si*bx+ci*by;
  double rx=cr*wx+sr*wy-mount[0],ry=-sr*wx+cr*wy-mount[1];
  double sx=cm*rx+sm*ry,sy=-sm*rx+cm*ry,q=hypot(sx,sy)*4000.;
  if(!isfinite(q)||q<0||q>UINT32_MAX){code=R8_DESKEW_INVALID;goto done;}
  /* 静止样本逐位保持相同，避免无谓的极坐标再量化，保留静态建图端点一致性。
 */
  if(fabs(dx)>1e-12||fabs(dy)>1e-12||fabs(r8_deskew_wrap(at.yaw_rad-ref.yaw_rad))>1e-12){
   out->points[i].distance_q2=(uint32_t)llround(q);
   long aq=llround(-atan2(sy,sx)*(65536./(2*R8_DESKEW_PI)));out->points[i].angle_q14=(uint16_t)((uint32_t)aq&65535u);
  }
  ++d->corrected_points;
 }
 code=R8_DESKEW_APPLIED;
 done:d->code=code;d->reason=r8_deskew_reason(code);return code;
}
#endif
