#ifndef ACG_S3_NEARFIELD_H
#define ACG_S3_NEARFIELD_H
/* 有界直接观测与滚动短期障碍证据，不访问设备，不用真值，不清全局图，也不独立授权电机。 */
#include "lab_types.h"
#include "s3_avoidance_types.h"
#include "passage.h"
#include "continuous_policy.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define NF_PI 3.14159265358979323846
#define NF_POSES 256u
#define NF_PENDING 2u
#define NF_SIDE 200u
#define NF_CELLS (NF_SIDE*NF_SIDE)
#define NF_CELL_M .020
#define NF_POINTS NF_CELLS
#define NF_RAYS 720u
#define NF_FRESH_US CONT_SCAN_US
#define NF_HOLD_US UINT64_C(600000)
#define NF_EXPIRE_US UINT64_C(2000000)
#define NF_POSE_GAP_US UINT64_C(350000)
/* EMPTY是未记录，不证明自由空间；丢弃旧UNKNOWN记录，过期障碍实际删除。
 */
enum {NF_CELL_EMPTY=0,NF_CELL_HIT=1,NF_CELL_UNKNOWN=2};
typedef enum nf_check_code {NF_CLEAR=0,NF_COLLISION=1,NF_NEED_OBSERVATION=2,NF_DATA_STALE=3,NF_POSE_INVALID=4,NF_INVALID_ACTION=5} nf_check_code;
typedef struct nf_check_result {nf_check_code code;double first_collision_x,first_collision_y;uint64_t source_time;uint32_t source_kind,unknown_cells;double ttc;} nf_check_result;
typedef struct nf_pose {uint64_t time;lab_pose p;} nf_pose;
typedef struct nf_cell {double x,y;uint64_t time;uint32_t clear_sequence;uint8_t clears,state;uint16_t reserved;} nf_cell;
typedef struct nf_raw_point {double x,y;} nf_raw_point;
typedef struct nf_pending {uint32_t used;lab_meta meta;struct scan_frame scan;} nf_pending;
typedef struct nf_ray {double angle,range;unsigned valid,invalid;} nf_ray;
typedef struct nf_state {
 passage_config c;lab_context context;uint32_t have_context,have_scan,have_input_scan;
 nf_pose poses[NF_POSES];unsigned pose_count;nf_pending pending[NF_PENDING];
 nf_cell cells[NF_CELLS];nf_raw_point latest[SCAN_MAX_POINTS];unsigned latest_count,count;
 int32_t origin_x,origin_y;unsigned have_window;
 uint64_t scan_time,last_received,updates,rejected,last_input_scan;
 uint64_t pose_max_age_us,scan_max_age_us; /* 仅使用配置的源时效。 */
 uint64_t cleared_points,expired_cells,window_evicted,bracket_cleared,last_expiry_us;
 uint32_t scan_sequence,raw_points,focus_points;
} nf_state;
_Static_assert(sizeof(nf_state)<=2u*1024u*1024u,"nearfield fixed storage exceeds 2MiB");
static inline uint64_t nf_scan_age_limit(const nf_state*s){return s&&s->scan_max_age_us?s->scan_max_age_us:CONT_SCAN_US;}
static inline uint64_t nf_pose_age_limit(const nf_state*s){return s&&s->pose_max_age_us?s->pose_max_age_us:CONT_ODOM_US;}
static inline const char*nf_check_reason(nf_check_code code){switch(code){case NF_CLEAR:return "CLEAR";case NF_COLLISION:return "COLLISION";case NF_NEED_OBSERVATION:return "NEED_OBSERVATION";case NF_DATA_STALE:return "DATA_STALE";case NF_POSE_INVALID:return "POSE_INVALID";default:return "INVALID_ACTION";}}
static inline double nf_wrap(double a){return remainder(a,2*NF_PI);}
static inline int nf_context_equal(const lab_context*a,const lab_meta*b){return a->mode==b->mode&&a->clock_domain==b->clock_domain&&a->timeline_id==b->timeline_id&&a->origin_epoch==b->origin_epoch;}
static inline int nf_pose_valid(const lab_pose*p){return p&&(p->quality&LAB_POSE_VALID)&&!(p->quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))&&isfinite(p->x_m)&&isfinite(p->y_m)&&isfinite(p->yaw_rad)&&fabs(p->x_m)<=1000000&&fabs(p->y_m)<=1000000;}
static inline void nf_init(nf_state*s,const passage_config*c){if(!s)return;memset(s,0,sizeof(*s));if(c){s->c=*c;s->c.margin_m+=c->wheel_outer_margin_m+(c->r4_enabled?.01:(c->wheel_outer_margin_verified?0:.01)+(c->sonar_offsets_verified?0:.02));}}
static inline void nf_pose_insert(nf_state*s,uint64_t t,const lab_pose*p){
 if(!nf_pose_valid(p))return;unsigned i=0;while(i<s->pose_count&&s->poses[i].time<t)++i;
 if(i<s->pose_count&&s->poses[i].time==t){s->poses[i].p=*p;return;}
 if(s->pose_count==NF_POSES){if(!i)return;memmove(s->poses,s->poses+1,(NF_POSES-1)*sizeof(*s->poses));--s->pose_count;--i;}
 memmove(s->poses+i+1,s->poses+i,(s->pose_count-i)*sizeof(*s->poses));s->poses[i]=(nf_pose){t,*p};++s->pose_count;
}
static inline int nf_pose_at(const nf_state*s,uint64_t t,lab_pose*out){
 if(!s||!s->pose_count)return 0;for(unsigned i=0;i<s->pose_count;i++){
  if(s->poses[i].time==t){*out=s->poses[i].p;return 1;}
  if(s->poses[i].time>t){if(!i)return 0;const nf_pose*a=s->poses+i-1,*b=s->poses+i;uint64_t dt=b->time-a->time;
   if(!dt||dt>NF_POSE_GAP_US)return 0;double f=(double)(t-a->time)/(double)dt;*out=a->p;out->x_m+=f*(b->p.x_m-a->p.x_m);out->y_m+=f*(b->p.y_m-a->p.y_m);out->yaw_rad=nf_wrap(a->p.yaw_rad+f*nf_wrap(b->p.yaw_rad-a->p.yaw_rad));return 1;}
 }return 0;
}
static inline void nf_body_point(const lab_pose*p,double x,double y,double*bx,double*by){double dx=x-p->x_m,dy=y-p->y_m,c=cos(p->yaw_rad),sn=sin(p->yaw_rad);*bx=c*dx+sn*dy;*by=-sn*dx+c*dy;}
static inline double nf_signed_distance(const nf_state*s,double x,double y){
 double dx=fabs(x)-s->c.length_m*.5-s->c.margin_m,dy=fabs(y)-s->c.width_m*.5-s->c.margin_m;
 return hypot(fmax(0,dx),fmax(0,dy))+fmin(0,fmax(dx,dy));
}
static inline unsigned nf_bin(double angle){double a=nf_wrap(angle);unsigned b=(unsigned)floor((a+NF_PI)*NF_RAYS/(2*NF_PI));return b<NF_RAYS?b:NF_RAYS-1;}

static inline unsigned nf_mod(int32_t x){int32_t n=x%(int32_t)NF_SIDE;return(unsigned)(n<0?n+(int32_t)NF_SIDE:n);}
static inline nf_cell*nf_cell_at(nf_state*s,int32_t x,int32_t y){
 if(!s->have_window||x<s->origin_x||y<s->origin_y||x>=s->origin_x+(int32_t)NF_SIDE||y>=s->origin_y+(int32_t)NF_SIDE)return NULL;
 return s->cells+nf_mod(y)*NF_SIDE+nf_mod(x);
}
static inline const nf_cell*nf_const_cell(const nf_state*s,int32_t x,int32_t y){return nf_cell_at((nf_state*)s,x,y);}
/* 过期时删除记录，不把旧障碍改成UNKNOWN。 */
static inline void nf_expire(nf_state*s,uint64_t now){
 if(!s)return;
 for(unsigned k=0;k<NF_CELLS;k++){nf_cell*q=s->cells+k;if(q->state&&(q->state==NF_CELL_UNKNOWN||(q->time<=now&&now-q->time>NF_EXPIRE_US))){memset(q,0,sizeof(*q));if(s->count)--s->count;++s->expired_cells;}}
 s->last_expiry_us=now;
}
static inline void nf_roll(nf_state*s,const lab_pose*current,uint64_t now){
 int32_t ox=(int32_t)floor(current->x_m/NF_CELL_M)-(int32_t)NF_SIDE/2,oy=(int32_t)floor(current->y_m/NF_CELL_M)-(int32_t)NF_SIDE/2;
 if(!s->have_window){s->origin_x=ox;s->origin_y=oy;s->have_window=1;}
 /* 环形索引保留准确世界栅格，仅清除移出窗口条带，不反复采样或旋转历史位置，不写全局地图。
 */
 for(unsigned iy=0;iy<NF_SIDE;iy++)for(unsigned ix=0;ix<NF_SIDE;ix++){
  int32_t x=s->origin_x+(int32_t)ix,y=s->origin_y+(int32_t)iy;
  nf_cell*q=s->cells+nf_mod(y)*NF_SIDE+nf_mod(x);if(!q->state)continue;
  if(x<ox||y<oy||x>=ox+(int32_t)NF_SIDE||y>=oy+(int32_t)NF_SIDE){memset(q,0,sizeof(*q));--s->count;++s->window_evicted;continue;}
  if(q->state==NF_CELL_UNKNOWN||(q->time<=now&&now-q->time>NF_EXPIRE_US)){memset(q,0,sizeof(*q));if(s->count)--s->count;++s->expired_cells;}
 }
 s->origin_x=ox;s->origin_y=oy;
}
static inline void nf_add(nf_state*s,const lab_pose*pose,double x,double y,uint64_t t){
 int32_t cx=(int32_t)floor(x/NF_CELL_M),cy=(int32_t)floor(y/NF_CELL_M);nf_cell*q=nf_cell_at(s,cx,cy);if(!q)return;
 if(q->state==NF_CELL_HIT&&q->time==t){double ax,ay,bx,by;nf_body_point(pose,q->x,q->y,&ax,&ay);nf_body_point(pose,x,y,&bx,&by);if(nf_signed_distance(s,ax,ay)<nf_signed_distance(s,bx,by))return;}
 if(!q->state)++s->count;*q=(nf_cell){.x=x,.y=y,.time=t,.state=NF_CELL_HIT};
}
static inline void nf_clear_ray(nf_state*s,double sx,double sy,double angle,double distance,uint32_t sequence,uint32_t previous){
 double dx=cos(angle),dy=sin(angle),limit=distance-.04;if(limit<=0)return;
 int32_t x=(int32_t)floor(sx/NF_CELL_M),y=(int32_t)floor(sy/NF_CELL_M);int stepx=dx>=0?1:-1,stepy=dy>=0?1:-1;
 double tx=fabs(dx)<1e-12?1e30:(((dx>=0?x+1:x)*NF_CELL_M-sx)/dx),ty=fabs(dy)<1e-12?1e30:(((dy>=0?y+1:y)*NF_CELL_M-sy)/dy);
 double ddx=fabs(dx)<1e-12?1e30:NF_CELL_M/fabs(dx),ddy=fabs(dy)<1e-12?1e30:NF_CELL_M/fabs(dy),entry=0;
 for(unsigned step=0;step<2*NF_SIDE+2&&entry<limit;step++){
  nf_cell*q=nf_cell_at(s,x,y);if(!q)break;
  /* 射线跨格仅作粗筛，邻格擦边不证明穿过实际测量端点。2mm覆盖近车线上及射线量化，不是20mm格半径。

 */
  double exit=fmin(tx,ty),along=q?(q->x-sx)*dx+(q->y-sy)*dy:0;
  double perpendicular=q?fabs((q->x-sx)*dy-(q->y-sy)*dx):1e30;
  if(q->state&&exit-entry>1e-9&&exit<=limit&&along>=entry-1e-9&&along<=exit+1e-9&&perpendicular<=.002&&(!q->clears||q->clear_sequence!=sequence)){
   q->clears=(uint8_t)(q->clear_sequence==previous&&q->clears?2:1);q->clear_sequence=sequence;
   if(q->clears>=2){memset(q,0,sizeof(*q));--s->count;++s->cleared_points;}}
  /* 射线经过准确格角时直接进入对角格，不访问单轴相等分支造成的零长度侧格。
 */
  if(fabs(tx-ty)<=1e-12){entry=fmin(tx,ty);tx+=ddx;ty+=ddy;x+=stepx;y+=stepy;}
  else if(tx<ty){entry=tx;tx+=ddx;x+=stepx;}else{entry=ty;ty+=ddy;y+=stepy;}
 }
}
/* 运动或噪声C1很少在2mm内重复旧端点。为避免旧未知格长期阻挡，两次完整扫描可用夹住旧点的两条有效更远射线清除该点：总角宽不超过2度，两侧横向间隙各不超过半格对角，至少远40mm，角域内无无效样本。该规则依赖实测角覆盖，不按年龄猜自由空间；当前端点仍独立写入检查。






 */
static inline void nf_clear_bracket(nf_state*s,double sx,double sy,double yaw,const nf_ray*rays,uint32_t sequence,uint32_t previous){
 const double max_angle=NF_PI/90.,max_cross=NF_CELL_M*.7071067811865476;
 for(unsigned index=0;index<NF_CELLS;index++){
  nf_cell*q=s->cells+index;if(!q->state||(q->clears&&q->clear_sequence==sequence))continue;
  double dx=q->x-sx,dy=q->y-sy,d=hypot(dx,dy);if(d<.03)continue;
  double a=nf_wrap(atan2(dy,dx)-yaw);unsigned b=nf_bin(a);int lo=-99,hi=-99;double dlo=-1e9,dhi=1e9;
  for(int offset=-4;offset<=4;offset++){
   const nf_ray*r=rays+(unsigned)((int)b+offset+(int)NF_RAYS)%(unsigned)NF_RAYS;if(!r->valid||r->invalid||r->range<d+.04)continue;
   double delta=nf_wrap(r->angle-a);if(fabs(delta)>max_angle||fabs(sin(delta))*d>max_cross)continue;
   if(delta<=0&&delta>dlo){lo=offset;dlo=delta;}if(delta>=0&&delta<dhi){hi=offset;dhi=delta;}
  }
  if(lo==-99||hi==-99||dhi-dlo>max_angle)continue;
  int invalid=0;for(int offset=lo;offset<=hi;offset++)if(rays[(unsigned)((int)b+offset+(int)NF_RAYS)%NF_RAYS].invalid){invalid=1;break;}
  if(invalid)continue;
  q->clears=(uint8_t)(q->clears&&q->clear_sequence==previous?2:1);q->clear_sequence=sequence;
  if(q->clears>=2){memset(q,0,sizeof(*q));--s->count;++s->cleared_points;++s->bracket_cleared;}
 }
}
static inline void nf_process(nf_state*s,const nf_pending*q,const lab_pose*at,uint64_t now){
 nf_ray rays[NF_RAYS];memset(rays,0,sizeof(rays));unsigned valid=0;double c=cos(at->yaw_rad),sn=sin(at->yaw_rad);
 double sx=at->x_m+c*s->c.mount_x_m-sn*s->c.mount_y_m,sy=at->y_m+sn*s->c.mount_x_m+c*s->c.mount_y_m;
 for(unsigned i=0;i<q->scan.count;i++){
  const struct scan_point*p=q->scan.points+i;double d=p->distance_q2/4000.;
  double a=-(double)p->angle_q14*NF_PI/32768.;unsigned b=nf_bin(a);
  if(!scan_return_in_range(&q->scan,i,s->c.lidar_min_m,s->c.lidar_max_m)){rays[b].invalid=1;continue;}
  ++valid;if(!rays[b].valid||d<rays[b].range){rays[b].angle=a;rays[b].range=d;rays[b].valid=1;}
 }
 if(!valid){++s->rejected;return;}
 nf_roll(s,at,now);
 double mount_yaw=at->yaw_rad+s->c.mount_yaw_rad,mc=cos(mount_yaw),ms=sin(mount_yaw),max_origin=0;
 if(q->scan.geometry_kind==SCAN_GEOMETRY_DESKEW_ORIGINS){
  /* 校正端点保留各自实际采集射线原点，不从扫描中点原点清除未经测量的空间。
 */
  for(unsigned i=0;i<q->scan.count;i++){
   const struct scan_point*p=q->scan.points+i;double d=p->distance_q2/4000.;
   if(!scan_return_in_range(&q->scan,i,s->c.lidar_min_m,s->c.lidar_max_m))continue;
   double ox,oy;scan_ray_origin(&q->scan,i,&ox,&oy);max_origin=fmax(max_origin,hypot(ox,oy));
   double a=-(double)p->angle_q14*NF_PI/32768.+mount_yaw;
   double ax=sx+mc*ox-ms*oy,ay=sy+ms*ox+mc*oy,dx=sx+d*cos(a)-ax,dy=sy+d*sin(a)-ay;
   nf_clear_ray(s,ax,ay,atan2(dy,dx),hypot(dx,dy),q->meta.sequence,s->scan_sequence);
  }
 }else for(unsigned b=0;b<NF_RAYS;b++)if(rays[b].valid)nf_clear_ray(s,sx,sy,mount_yaw+rays[b].angle,rays[b].range,q->meta.sequence,s->scan_sequence);
 /* 双射线包围证据要求共同原点，保留上方准确单射线清除，不将移动原点拼成假覆盖。
 */
 if(max_origin<=.002)nf_clear_bracket(s,sx,sy,mount_yaw,rays,q->meta.sequence,s->scan_sequence);
 s->latest_count=0;s->focus_points=0;
 /* 每个有效原始端点独立保留，角桶只限制清除工作量，不丢细杆端点。
 */
 for(unsigned i=0;i<q->scan.count;i++){
  const struct scan_point*p=q->scan.points+i;double d=p->distance_q2/4000.;
  if(!scan_return_in_range(&q->scan,i,s->c.lidar_min_m,s->c.lidar_max_m))continue;
  double a=-(double)p->angle_q14*NF_PI/32768.+s->c.mount_yaw_rad,bx=s->c.mount_x_m+d*cos(a),by=s->c.mount_y_m+d*sin(a);
  double x=at->x_m+c*bx-sn*by,y=at->y_m+sn*bx+c*by;
  s->latest[s->latest_count++]=(nf_raw_point){x,y};
  if(bx>=-s->c.length_m*.5&&bx<=s->c.length_m*.5+1.&&fabs(by)<=s->c.width_m*.5+.45)++s->focus_points;
  nf_add(s,at,x,y,q->meta.time_us);
 }
 s->raw_points=valid;s->scan_time=q->meta.time_us;s->scan_sequence=q->meta.sequence;s->have_scan=1;++s->updates;
}
static inline void nf_drain(nf_state*s,uint64_t now){
 for(unsigned i=0;i<NF_PENDING;i++)if(s->pending[i].used){nf_pending*q=s->pending+i;
  if(q->meta.time_us>now||now-q->meta.time_us>nf_scan_age_limit(s)){q->used=0;++s->rejected;continue;}
  lab_pose p;if(!nf_pose_at(s,q->meta.time_us,&p))continue;
  if(!s->have_scan||q->meta.time_us>s->scan_time)nf_process(s,q,&p,now);q->used=0;
 }
}
static inline void nf_observe(nf_state*s,const lab_event*e,uint64_t now){
 if(!s||!e||(e->type!=LAB_EVENT_POSE&&e->type!=LAB_EVENT_SCAN))return;
 const lab_meta*m=&e->meta;
 if(m->provider==LAB_PROVIDER_TRUTH||!(m->flags&LAB_TIME_VALID)||!(m->flags&LAB_DATA_VALID)||!m->timeline_id||!m->origin_epoch||m->time_us>m->received_us||m->received_us>now||m->clock_domain==LAB_CLOCK_REMOTE_UNMAPPED){++s->rejected;return;}
 if(!s->have_context){s->context=(lab_context){m->mode,m->clock_domain,m->timeline_id,m->origin_epoch};s->have_context=1;}
 if(!nf_context_equal(&s->context,m)){++s->rejected;return;}
 if(now>s->last_received)s->last_received=now;
 if(!s->last_expiry_us||now<s->last_expiry_us||now-s->last_expiry_us>=100000)nf_expire(s,now);
 if(e->type==LAB_EVENT_POSE){nf_pose_insert(s,m->time_us,&e->data.pose);nf_drain(s,now);return;}
 if(!e->data.scan.count||e->data.scan.count>SCAN_MAX_POINTS||!scan_geometry_valid(&e->data.scan)||e->data.scan.session!=m->stream_session||e->data.scan.sequence!=m->sequence||(s->have_input_scan&&m->time_us<=s->last_input_scan)||now-m->time_us>nf_scan_age_limit(s)){++s->rejected;return;}
 s->last_input_scan=m->time_us;s->have_input_scan=1;unsigned at=0;
 if(s->pending[0].used){at=1;if(s->pending[1].used){s->pending[0]=s->pending[1];++s->rejected;}}
 s->pending[at].used=1;s->pending[at].meta=*m;s->pending[at].scan=e->data.scan;nf_drain(s,now);
}
static inline int nf_current(const nf_state*s,uint64_t now,lab_pose*out){
 if(!s||!s->pose_count)return 0;const nf_pose*p=s->poses+s->pose_count-1;
 if(p->time>now||now-p->time>nf_pose_age_limit(s)||!nf_pose_valid(&p->p))return 0;*out=p->p;return 1;
}
static inline void nf_view_add(const nf_state*s,const lab_pose*current,double wx,double wy,uint64_t t,uint64_t now,s3_nearfield_view*out,double*nearest,double*distances){
 double x,y;nf_body_point(current,wx,wy,&x,&y);if(t>now||hypot(x,y)>1.8)return;
 ++out->point_count;if(t!=s->scan_time)++out->held_points;double hx=s->c.length_m*.5,hy=s->c.width_m*.5;int overlap=fabs(x)<=hx&&fabs(y)<=hy;
 if(fabs(y)<=hy+.03&&(x>=hx||(overlap&&x>=0))){double d=fmax(0,x-hx);if(out->front_m<0||d<out->front_m)out->front_m=d;}
 if(fabs(y)<=hy+.03&&(x<=-hx||(overlap&&x<=0))){double d=fmax(0,-x-hx);if(out->rear_m<0||d<out->rear_m)out->rear_m=d;}
 if(fabs(x)<=hx+.03&&(y>=hy||(overlap&&y>=0))){double d=fmax(0,y-hy);if(out->left_m<0||d<out->left_m)out->left_m=d;}
 if(fabs(x)<=hx+.03&&(y<=-hy||(overlap&&y<=0))){double d=fmax(0,-y-hy);if(out->right_m<0||d<out->right_m)out->right_m=d;}
 double d=nf_signed_distance(s,x,y);if(d<*nearest){*nearest=d;out->obstacle_x_m=x;out->obstacle_y_m=y;}
 unsigned at=0;while(at<out->hazard_count&&distances[at]<=d)++at;
 if(at<S3_HAZARD_VIEW_MAX){unsigned n=out->hazard_count<S3_HAZARD_VIEW_MAX?out->hazard_count++:S3_HAZARD_VIEW_MAX-1;for(unsigned j=n;j>at;j--){out->hazards[j]=out->hazards[j-1];distances[j]=distances[j-1];}out->hazards[at]=(s3_hazard_point){x,y,t};distances[at]=d;}
}
static inline void nf_view(const nf_state*s,uint64_t now,s3_nearfield_view*out){
 if(!out)return;memset(out,0,sizeof(*out));out->front_m=out->left_m=out->right_m=out->rear_m=-1;out->forward_return_m=out->forward_corridor_m=-1;out->age_us=UINT64_MAX;out->stale=1;
 if(!s){snprintf(out->reason,sizeof(out->reason),"WAIT_NEARFIELD");return;}
 out->scan_time_us=s->scan_time;out->updates=s->updates;out->rejected_scans=s->rejected;out->raw_points=s->raw_points;out->focus_points=s->focus_points;
 out->history_capacity=NF_CELLS;out->storage_bytes=(uint32_t)sizeof(*s);out->history_expired=s->expired_cells;out->ray_cleared=s->cleared_points;out->window_evicted=s->window_evicted;out->bracket_cleared=s->bracket_cleared;
 if(s->have_scan&&s->scan_time<=now)out->age_us=now-s->scan_time;lab_pose current;
 if(!nf_current(s,now,&current)){snprintf(out->reason,sizeof(out->reason),"WAIT_NEARFIELD_POSE");return;}
 out->stale=!s->have_scan||out->age_us>nf_scan_age_limit(s);out->valid=!out->stale;double nearest=1e9,distances[S3_HAZARD_VIEW_MAX];for(unsigned i=0;i<S3_HAZARD_VIEW_MAX;i++)distances[i]=1e9;
 unsigned bins=0;double hx=s->c.length_m*.5,hy=s->c.width_m*.5;
 for(unsigned i=0;i<s->latest_count;i++){
  double x,y;nf_body_point(&current,s->latest[i].x,s->latest[i].y,&x,&y);
  if(fabs(x)<=hx+.01&&fabs(y)<=hy+.01){++out->forward_self_returns;}
  else {
   double a=atan2(y,x);if(fabs(a)<=NF_PI/12.){unsigned b=(unsigned)floor((a+NF_PI/12.)/(NF_PI/36.));if(b>5)b=5;bins|=1u<<b;++out->forward_returns;double d=hypot(x,y);if(out->forward_return_m<0||d<out->forward_return_m)out->forward_return_m=d;}
   if(x>=hx&&fabs(y)<=hy+.03){++out->forward_corridor_returns;double d=x-hx;if(out->forward_corridor_m<0||d<out->forward_corridor_m)out->forward_corridor_m=d;}
  }
  nf_view_add(s,&current,s->latest[i].x,s->latest[i].y,s->scan_time,now,out,&nearest,distances);
 }
 for(unsigned b=0;b<6;b++)if(bins&(1u<<b))++out->forward_bins;
 snprintf(out->forward_reason,sizeof(out->forward_reason),"%s",out->stale?"FORWARD_SCAN_STALE":!out->forward_returns?"NO_VALID_FORWARD_RETURN":!out->forward_corridor_returns?"FORWARD_RETURNS_OUTSIDE_BODY_CORRIDOR":out->front_m<0?"FORWARD_CORRIDOR_RETURN_BEYOND_NEAR_1P8M":"FORWARD_NEAR_CORRIDOR_RETURN");
 for(unsigned i=0;i<NF_CELLS;i++){
  const nf_cell*q=s->cells+i;if(!q->state||q->time>now)continue;++out->history_cells;uint64_t age=now-q->time;
  if(q->state==NF_CELL_UNKNOWN||age>NF_EXPIRE_US){++out->history_unknown;continue;}
  if(age>NF_HOLD_US)++out->history_pending;else ++out->history_recent;
  /* 历史计数仅作诊断，当前危险几何使用最新扫描。 */
 }
 out->front_blocked=out->front_m>=0&&out->front_m<.35;out->left_blocked=out->left_m>=0&&out->left_m<.12;out->right_blocked=out->right_m>=0&&out->right_m<.12;out->rear_blocked=out->rear_m>=0&&out->rear_m<.20;
 snprintf(out->reason,sizeof(out->reason),"%s",out->stale?"NEARFIELD_STALE":out->front_blocked?"FRONT_OBSTACLE":out->left_blocked||out->right_blocked?"SIDE_OBSTACLE":"CURRENT_OBSERVATIONS");
}
/* 每端点检查所有采样车体位姿，历史格中心半对角只表示空间数值误差，不作膨胀；侧后点不阻止与其分离的动作。

 */
static inline int nf_point_swept(const nf_state*s,double x,double y,double pad,unsigned steps,const double*xs,const double*ys,const double*cs,const double*ss,double envelope,unsigned*hit){
 double hx=s->c.length_m*.5+s->c.margin_m+pad,hy=s->c.width_m*.5+s->c.margin_m+pad;
 double initial=nf_signed_distance(s,x,y)-pad,last=initial;
 for(unsigned k=1;k<=steps;k++){
  double dx=x-xs[k],dy=y-ys[k],bx=cs[k]*dx+ss[k]*dy,by=-ss[k]*dx+cs[k]*dy;
  if(initial>envelope){if(fabs(bx)<=hx+envelope&&fabs(by)<=hy+envelope){*hit=k;return 1;}}
  else{double d=nf_signed_distance(s,bx,by)-pad;if((initial>0&&d<=0)||d<initial-.001){*hit=k;return 1;}last=d;}
 }
 /* 实际测得重叠需增加分离度，历史误差外壳允许平行滑动，不当作实际车体穿透。
 */
 if(initial<=0&&last<initial+(pad>0?0:.005)){*hit=steps;return 1;}return 0;
}
static inline nf_check_result nf_check(const nf_state*s,uint64_t now,double v,double w,double horizon){
 nf_check_result result={.code=NF_CLEAR};
 if(!isfinite(v)||!isfinite(w)||!isfinite(horizon)||horizon<0||horizon>6){result.code=NF_INVALID_ACTION;return result;}
 if(fabs(v)<1e-9&&fabs(w)<1e-9)return result;
 if(horizon<=0){result.code=NF_INVALID_ACTION;return result;}
 if(!s||(!s->have_scan&&!s->have_input_scan))return result; /* 仅地图兼容，不授权真实电机。 */
 if(!s->have_scan||s->scan_time>now||now-s->scan_time>nf_scan_age_limit(s)){result.code=NF_DATA_STALE;return result;}
 if(!s->raw_points){result.code=NF_NEED_OBSERVATION;return result;} /* 新鲜无回波扫描仍是未知，不是自由空间。 */
 lab_pose current;if(!nf_current(s,now,&current)){result.code=NF_POSE_INVALID;return result;}
 unsigned steps=(unsigned)ceil(fmax(fabs(v)*horizon/.005,fabs(w)*horizon/.025));if(!steps)steps=1;if(steps>512){result.code=NF_INVALID_ACTION;return result;}
 double xs[513],ys[513],cs[513],ss[513],dt=horizon/steps,hx=s->c.length_m*.5+s->c.margin_m,hy=s->c.width_m*.5+s->c.margin_m;
 double radius=hypot(hx,hy),envelope=.5*dt*(fabs(v)+radius*fabs(w)),reach=radius+fabs(v)*horizon+envelope;
 for(unsigned k=0;k<=steps;k++){double t=horizon*k/steps,a=w*t;cs[k]=cos(a);ss[k]=sin(a);if(fabs(w)<1e-9){xs[k]=v*t;ys[k]=0;}else{xs[k]=v/w*sin(a);ys[k]=v/w*(1-cos(a));}}
 for(unsigned i=0;i<s->latest_count;i++){
  double x,y;nf_body_point(&current,s->latest[i].x,s->latest[i].y,&x,&y);if(x*x+y*y>reach*reach)continue;unsigned hit=0;
  if(nf_point_swept(s,x,y,0,steps,xs,ys,cs,ss,envelope,&hit))return(nf_check_result){.code=NF_COLLISION,.first_collision_x=x,.first_collision_y=y,.source_time=s->scan_time,.source_kind=1,.ttc=hit*dt};
 }
 /* 粗格筛选把查询限制在可达圆盘，不对每候选遍历全部4万格，也不让远处占用格阻挡当前动作。
 */
 double pad=NF_CELL_M*.7071067811865476,bound=reach+pad;
 int32_t x0=(int32_t)floor((current.x_m-bound)/NF_CELL_M),x1=(int32_t)floor((current.x_m+bound)/NF_CELL_M),y0=(int32_t)floor((current.y_m-bound)/NF_CELL_M),y1=(int32_t)floor((current.y_m+bound)/NF_CELL_M);
 if(x0<s->origin_x)x0=s->origin_x;if(y0<s->origin_y)y0=s->origin_y;if(x1>=s->origin_x+(int32_t)NF_SIDE)x1=s->origin_x+(int32_t)NF_SIDE-1;if(y1>=s->origin_y+(int32_t)NF_SIDE)y1=s->origin_y+(int32_t)NF_SIDE-1;
 nf_check_result uncertain={.code=NF_CLEAR};
 for(int32_t gy=y0;gy<=y1;gy++)for(int32_t gx=x0;gx<=x1;gx++){
  const nf_cell*q=nf_const_cell(s,gx,gy);if(!q||q->state!=NF_CELL_HIT||q->time>now||now-q->time>NF_EXPIRE_US||q->time==s->scan_time)continue;
  double x,y;nf_body_point(&current,(gx+.5)*NF_CELL_M,(gy+.5)*NF_CELL_M,&x,&y);if(x*x+y*y>bound*bound)continue;unsigned hit=0;
  if(!nf_point_swept(s,x,y,pad,steps,xs,ys,cs,ss,envelope,&hit))continue;
  uint64_t age=now-q->time;if(q->state==NF_CELL_HIT&&age<=NF_HOLD_US)return(nf_check_result){.code=NF_COLLISION,.first_collision_x=x,.first_collision_y=y,.source_time=q->time,.source_kind=2,.ttc=hit*dt};
  if(uncertain.code==NF_CLEAR)uncertain=(nf_check_result){.code=NF_NEED_OBSERVATION,.first_collision_x=x,.first_collision_y=y,.source_time=q->time,.source_kind=3,.ttc=hit*dt};++uncertain.unknown_cells;
 }
 return uncertain;
}
/* 执行策略使用当前实测防护，预测仅作建议。只核对最新真实扫描端点和当前实际位姿，未来车体样本、历史误差外壳及未知格不直接授权或否决运动。

 */
static inline nf_check_result nf_check_current(const nf_state*s,uint64_t now,double v,double w){
 nf_check_result result={.code=NF_CLEAR};
 if(!isfinite(v)||!isfinite(w)){result.code=NF_INVALID_ACTION;return result;}
 if(fabs(v)<1e-9&&fabs(w)<1e-9)return result;
 if(!s||!s->have_scan||s->scan_time>now||now-s->scan_time>nf_scan_age_limit(s)||!s->raw_points||!s->latest_count){result.code=NF_DATA_STALE;return result;}
 lab_pose current;if(!nf_current(s,now,&current)){result.code=NF_POSE_INVALID;return result;}
 double hx=s->c.length_m*.5+s->c.margin_m+s->c.wheel_outer_margin_m,hy=s->c.width_m*.5+s->c.margin_m+s->c.wheel_outer_margin_m;
 int turning=fabs(w)>.1,pivot=fabs(v)<1e-9;
 for(unsigned k=0;k<s->latest_count;k++){
  double x,y;nf_body_point(&current,s->latest[k].x,s->latest[k].y,&x,&y);
  int collision=fabs(x)<=hx&&fabs(y)<=hy;
  if((v>0||turning)&&x>=hx&&fabs(y)<=hy+.01&&x-hx<.030)collision=1;
  if(fabs(x)<=hx+.01&&fabs(y)>=hy){double gap=fabs(y)-hy;
   if(!turning&&fabs(v)>1e-9&&gap<.015)collision=1;
   if(turning&&(pivot||y*w>0)&&gap<.030)collision=1;
  }
  if(collision)return(nf_check_result){.code=NF_COLLISION,.first_collision_x=x,.first_collision_y=y,.source_time=s->scan_time,.source_kind=1,.ttc=0};
 }
 return result;
}
static inline int nf_motion_clear(const nf_state*s,uint64_t now,double v,double w,double horizon){(void)horizon;return nf_check_current(s,now,v,w).code==NF_CLEAR;}
static inline int nf_allows(const nf_state*s,uint64_t now,double v,double w){return nf_motion_clear(s,now,v,w,.8);}
static inline int nf_rotation_clear(const nf_state*s,uint64_t now,double angle){if(!isfinite(angle)||fabs(angle)>6)return 0;if(fabs(angle)<1e-9)return 1;return nf_motion_clear(s,now,0,angle>0?1:-1,fabs(angle));}
#endif
