/* 只读目标融合输出回波表面位置，保留各来源时间。 */
#include "object_scene.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "follow_association_v5.h"
#include "static_wall_v10.h"
_Static_assert(sizeof(fv5_state)<=24576,"follow classifier exceeds fixed scene storage");
#define SF_PI 3.14159265358979323846
#define SF_CLUSTERS 64u
#define SF_VISUAL_MAX_US 2000000u
#define SF_SCAN_MAX_US 1500000u
#define SF_HISTORY_MAX_US 10000000u
#define SF_APPROX_POSE_MAX_US 200000u

typedef struct sf_pose {double x,y,yaw,error;uint64_t stamp;int approximate;} sf_pose;
typedef struct sf_hit {double x,y,angle,diff,range,wall;int full_track;} sf_hit;
typedef struct sf_cluster {unsigned n;double x,y,min_angle,max_angle,score,camera_range,stereo_residual,span,wall,motion,isolation;int full_track;} sf_cluster;
static double sf_wrap(double x){return remainder(x,2*SF_PI);}
static uint64_t sf_delta(uint64_t a,uint64_t b){return a>=b?a-b:b-a;}
static int sf_time(uint64_t stamp,uint64_t now,uint64_t age){return stamp&&stamp<=now&&now-stamp<=age;}
static int sf_finite_pose(double x,double y,double a){return isfinite(x)&&isfinite(y)&&isfinite(a);}
static int sf_context(const lab_real_context*a,const lab_real_context*b){return !memcmp(a,b,sizeof(*a));}
static void sf_reason(obj_scene*s,const char*r,const char*zh){snprintf(s->reason,sizeof(s->reason),"%s",r);snprintf(s->message_zh,sizeof(s->message_zh),"%s",zh);}
void obj_scene_config_default(obj_scene_config*c){if(!c)return;memset(c,0,sizeof(*c));c->camera_ahead_lidar_m=.14;c->standoff_m=.20;c->range_max_m=12;c->coarse_max_age_us=2000000;c->mount_approximate=1;}
int obj_scene_config_load_mount(obj_scene_config*c,const char*path){obj_mount m;if(!c||!obj_mount_load(path,&m))return 0;c->camera=m;c->mount_configured=1;c->mount_approximate=!m.verified;return 1;}
void obj_scene_camera_origin(const obj_scene_config*c,const passage_config*p,double out[3]){out[0]=c->mount_configured?c->camera.x:(p?p->mount_x_m:0)+c->camera_ahead_lidar_m;out[1]=c->mount_configured?c->camera.y:(p?p->mount_y_m:0);out[2]=c->mount_configured?c->camera.z:0;}
void obj_scene_camera_vector(const obj_scene_config*c,const double optical[3],double body[3]){
 double x=optical[2],y=-optical[0],z=-optical[1];double roll=c->mount_configured?c->camera.roll:0,pitch=c->mount_configured?c->camera.pitch:0,yaw=c->mount_configured?c->camera.yaw:0;
 double yy=cos(roll)*y-sin(roll)*z,zz=sin(roll)*y+cos(roll)*z,xx=cos(pitch)*x+sin(pitch)*zz;
 body[0]=cos(yaw)*xx-sin(yaw)*yy;body[1]=sin(yaw)*xx+cos(yaw)*yy;body[2]=-sin(pitch)*x+cos(pitch)*zz;
}
static double sf_ray_bearing(const obj_scene_config*c,const double ray[3]){double b[3];obj_scene_camera_vector(c,ray,b);return hypot(b[0],b[1])>1e-6?atan2(b[1],b[0]):NAN;}
static int sf_visual_source(const obj_frame*f,int id,uint64_t*time,uint64_t*frame){
 if(!f)return 0;for(int i=0;i<f->count&&i<OBJ_MAX;i++){const obj_detection*d=&f->objects[i];if(d->track_id!=id)continue;
  if(!d->observed||!strcmp(d->tracking,"PREDICTED")||!strcmp(d->tracking,"LOST"))return 0;
  *time=d->observation_time_us?d->observation_time_us:f->frame_us;*frame=d->observation_frame_id?d->observation_frame_id:f->frame_id;
  return *time&&*frame&&*time<=f->frame_us&&*frame<=f->frame_id;
 }return 0;
}
static void sf_measure(obj_scene*s,const obj_scene_config*c,const obj_frame*f,const lab_real_runtime_view*v,const passage_config*p,double x,double y,double error,uint64_t source,unsigned kind){
 object_target_observation_v2*m=&s->measurement_v2;memset(m,0,sizeof(*m));m->valid=1;m->confirmed=s->confirmed;m->surface_valid=kind!=OBJECT_MEASURE_STEREO_COARSE;m->mount_verified=!c->mount_approximate;m->kind=kind;m->context=v->context;m->epoch=s->epoch;m->track_id=s->track_id;m->source_us=source;m->image_us=f->frame_us;m->scan_us=kind!=OBJECT_MEASURE_STEREO_COARSE?v->scan_time_us:0;m->frame_id=f->frame_id;
 m->odom_x_m=x;m->odom_y_m=y;double dx=x-v->raw_odometry.x_m,dy=y-v->raw_odometry.y_m,co=cos(v->raw_odometry.yaw_rad),sn=sin(v->raw_odometry.yaw_rad);m->body_x_m=co*dx+sn*dy;m->body_y_m=-sn*dx+co*dy;
 m->surface_gap_m=fmax(0,hypot(m->body_x_m,m->body_y_m)-(p&&p->length_m>0?p->length_m:.333)*.5);m->bearing_rad=atan2(m->body_y_m,m->body_x_m);m->error_m=error;
 snprintf(m->source,sizeof(m->source),"%s",s->source);snprintf(m->reason,sizeof(m->reason),"%s",s->reason);
 m->binding_generation=s->binding_generation;
 m->association_revision=s->association_revision;
 m->support_points=s->follow_support_points;m->sparse=s->follow_sparse;if(m->sparse)m->confirmed=0;
 uint64_t image=0,frame=0;
 if(kind==OBJECT_MEASURE_STEREO_COARSE){for(int i=0;i<f->count&&i<OBJ_MAX;i++)if(f->objects[i].track_id==s->track_id){image=f->objects[i].depth_time_us?f->objects[i].depth_time_us:f->frame_us;frame=f->objects[i].depth_frame_id?f->objects[i].depth_frame_id:f->frame_id;break;}}
 else if(!sf_visual_source(f,s->track_id,&image,&frame)){image=s->lidar_visual_us;frame=s->lidar_visual_frame;}
 m->image_us=image;m->frame_id=frame;
}
static int sf_current_pose(const lab_real_runtime_view*v,uint64_t stamp,uint64_t now,sf_pose*p){
 if(!v->has_odometry||!(v->raw_odometry.quality&LAB_POSE_VALID)||(v->raw_odometry.quality&(LAB_POSE_GAP|LAB_POSE_STALE))||!sf_time(v->odometry_time_us,now,SF_VISUAL_MAX_US)||sf_delta(v->odometry_time_us,stamp)>SF_APPROX_POSE_MAX_US||!sf_finite_pose(v->raw_odometry.x_m,v->raw_odometry.y_m,v->raw_odometry.yaw_rad))return 0;
 *p=(sf_pose){v->raw_odometry.x_m,v->raw_odometry.y_m,v->raw_odometry.yaw_rad,fmax(0,v->raw_odometry.uncertainty_m),v->odometry_time_us,1};
 /* 该界限表示显式近似误差，不代表准确的指定时刻位姿。 */
 p->error+=.30*sf_delta(v->odometry_time_us,stamp)/1000000.;return 1;
}
static int sf_image_pose(const approach_observation*o,const obj_frame*f,const lab_real_runtime_view*v,uint64_t now,sf_pose*p){
 if(o&&(!o->context_valid||sf_context(&o->context,&v->context))&&o->epoch==f->epoch&&o->track_id==f->selected&&o->frame_us==f->frame_us&&o->frame_pose_valid&&sf_finite_pose(o->frame_pose_x,o->frame_pose_y,o->frame_pose_yaw)){
  *p=(sf_pose){o->frame_pose_x,o->frame_pose_y,o->frame_pose_yaw,0,f->frame_us,0};return 1;}
 return sf_current_pose(v,f->frame_us,now,p);
}
static int sf_scan_pose(const approach_observation*o,const obj_frame*f,const lab_real_runtime_view*v,uint64_t now,sf_pose*p){
 if(o&&f&&(!o->context_valid||sf_context(&o->context,&v->context))&&o->epoch==f->epoch&&o->track_id==f->selected&&o->scan_us==v->scan_time_us&&o->scan_pose_valid&&sf_finite_pose(o->scan_pose_x,o->scan_pose_y,o->scan_pose_yaw)){
  *p=(sf_pose){o->scan_pose_x,o->scan_pose_y,o->scan_pose_yaw,0,v->scan_time_us,0};return 1;}
 /* 当前运动ODOM不能直接当作扫描时位姿；使用实际原始历史两端插值，缺失历史时只接受相同源时间。
 */
 if(v->odometry_time_us!=v->scan_time_us)return 0;
 return sf_current_pose(v,v->scan_time_us,now,p);
}
static int sf_depth_pose(const approach_observation*o,const obj_frame*f,const lab_real_runtime_view*v,uint64_t stamp,uint64_t now,sf_pose*p){
 if(o&&(!o->context_valid||sf_context(&o->context,&v->context))&&o->epoch==f->epoch&&o->track_id==f->selected&&o->stereo_us==stamp&&o->stereo_pose_valid&&sf_finite_pose(o->stereo_pose_x,o->stereo_pose_y,o->stereo_pose_yaw)){
  *p=(sf_pose){o->stereo_pose_x,o->stereo_pose_y,o->stereo_pose_yaw,0,stamp,0};return 1;}
 if(stamp==f->frame_us)return sf_image_pose(o,f,v,now,p);
 return sf_current_pose(v,stamp,now,p);
}
static void sf_transform(const sf_pose*p,double x,double y,double*wx,double*wy){double c=cos(p->yaw),s=sin(p->yaw);*wx=p->x+c*x-s*y;*wy=p->y+s*x+c*y;}
static int sf_map_available(const lab_real_runtime_view*v,uint64_t now){
 const lab_localization_view*l=&v->localization;
 return v->has_localization&&l->has_pose&&l->map.map_id&&sf_context(&l->context,&v->context)&&sf_context(&l->map.context,&v->context)&&sf_time(l->pose_time_us,now,SF_VISUAL_MAX_US)&&sf_finite_pose(l->pose.x_m,l->pose.y_m,l->pose.yaw_rad)&&sf_finite_pose(l->map_odom_x_m,l->map_odom_y_m,l->map_odom_yaw_rad);
}
static int sf_project(obj_scene*s,const lab_real_runtime_view*v,const passage_config*p,uint64_t now){
 if(!s->target_odom_valid||!sf_map_available(v,now)){s->target_valid=s->dock_valid=0;return 0;}
 const lab_localization_view*l=&v->localization;double c=cos(l->map_odom_yaw_rad),sn=sin(l->map_odom_yaw_rad);
 s->target_x=l->map_odom_x_m+c*s->target_odom_x-sn*s->target_odom_y;
 s->target_y=l->map_odom_y_m+sn*s->target_odom_x+c*s->target_odom_y;
 double dx=s->target_x-l->pose.x_m,dy=s->target_y-l->pose.y_m,d=hypot(dx,dy);s->range_m=d;s->target_valid=isfinite(d);
 s->bearing_rad=sf_wrap(atan2(dy,dx)-l->pose.yaw_rad);s->bearing_valid=1;
 double len=p&&isfinite(p->length_m)&&p->length_m>0?p->length_m:.333,offset=len*.5+s->standoff_m;
 s->dock_valid=s->target_valid&&d>offset+.01;
 if(s->dock_valid){s->dock_x=s->target_x-offset*dx/d;s->dock_y=s->target_y-offset*dy/d;s->dock_yaw=atan2(dy,dx);}
 else {s->dock_x=l->pose.x_m;s->dock_y=l->pose.y_m;s->dock_yaw=l->pose.yaw_rad;}
 return s->target_valid;
}
static void sf_clear_target(obj_scene*s){s->target_valid=s->dock_valid=s->target_odom_valid=s->confirmed=s->historical=0;s->source_us=0;s->support=0;s->support_source_us=0;s->path_count=0;snprintf(s->preview_status,sizeof(s->preview_status),"UNAVAILABLE");}
static int sf_hold(obj_scene*s,const lab_real_runtime_view*v,const passage_config*p,uint64_t now,const char*reason,const char*zh){
 if(s->target_odom_valid&&sf_time(s->source_us,now,SF_HISTORY_MAX_US)&&sf_project(s,v,p,now)){
  s->historical=1;s->confirmed=0;sf_reason(s,reason,zh);return 1;}
 sf_clear_target(s);sf_reason(s,reason,zh);return 0;
}
static double sf_depth_hint(const obj_detection*d,uint64_t frame_us,uint64_t now,uint64_t max_age,double range,double*error){
 uint64_t stamp=d->depth_time_us?d->depth_time_us:frame_us;if(!sf_time(stamp,now,max_age))return 1e6;
 double best=1e6,e=0;
 if(d->coarse_valid&&isfinite(d->coarse_range)&&isfinite(d->coarse_error)&&d->coarse_range>0&&d->coarse_error>=0){best=fabs(range-d->coarse_range);e=fmax(.15,d->coarse_error);}
 else if(d->depth_valid&&isfinite(d->x)&&isfinite(d->z)&&d->z>0){best=fabs(range-hypot(d->x,d->z));e=fmax(.15,d->error);}
 for(unsigned i=0;i<d->depth_candidate_count&&i<3;i++)if(isfinite(d->depth_candidate_range[i])&&isfinite(d->depth_candidate_error[i])&&d->depth_candidate_range[i]>0&&d->depth_candidate_error[i]>=0){double z=fabs(range-d->depth_candidate_range[i]);if(z<best){best=z;e=fmax(.15,d->depth_candidate_error[i]);}}
 if(error)*error=e;return best;
}
/* 在各自实际采集位姿下，将两数据源都换到ODOM比较，车辆平移不应被误认为目标身份改变。
 */
static double sf_depth_hint_aligned(const obj_detection*d,const obj_scene_config*c,const obj_frame*f,const approach_observation*o,const lab_real_runtime_view*v,uint64_t now,double camera_x,double camera_y,double range,double*error){
 uint64_t stamp=d->depth_time_us?d->depth_time_us:f->frame_us;sf_pose dp;
 if(!sf_depth_pose(o,f,v,stamp,now,&dp))return 1e6;
 double be=d->depth_bearing_valid?d->depth_bearing:d->depth_valid?atan2(-d->x,d->z):stamp==f->frame_us&&d->bearing_valid?d->bearing:NAN;
 if(!isfinite(be))return 1e6;
 obj_detection copy=*d;copy.depth_valid=0;copy.coarse_valid=0;copy.depth_candidate_count=0;
 double inputs[4],errs[4];unsigned n=0;
 if(d->coarse_valid){inputs[n]=d->coarse_range;errs[n++]=d->coarse_error;}else if(d->depth_valid){inputs[n]=hypot(d->x,d->z);errs[n++]=d->error;}
 for(unsigned i=0;i<d->depth_candidate_count&&i<3&&n<4;i++){inputs[n]=d->depth_candidate_range[i];errs[n++]=d->depth_candidate_error[i];}
 double origin[3];obj_scene_camera_origin(c,NULL,origin);double best=1e6;
 for(unsigned i=0;i<n;i++){double op[3]={-inputs[i]*sin(be),0,inputs[i]*cos(be)},body[3],x,y;obj_scene_camera_vector(c,op,body);sf_transform(&dp,origin[0]+body[0],origin[1]+body[1],&x,&y);double residual=fabs(range-hypot(x-camera_x,y-camera_y));if(residual<best){best=residual;if(error)*error=fmax(.15,errs[i])+dp.error;}}
 return sf_time(stamp,now,c->coarse_max_age_us)?best:1e6;
}
/* 框扩大只提供低权重关联先验，不形成测量，参考尺度需由独立真实距离建立。
 */
/* 静止箱子自身也可能有长平面，仅墙面标签不能否决当前标定双目支持的箱体表面。
 */
static int sf_static_depth_available(const obj_detection*d,const obj_frame*f,uint64_t now){
 uint64_t stamp=d&&d->depth_time_us?d->depth_time_us:f?f->frame_us:0;
 return d&&d->depth_valid&&isfinite(d->z)&&d->z>.05&&isfinite(d->error)&&d->error>=0&&d->error<=fmax(.30,.30*d->z)&&sf_time(stamp,now,1200000);
}
static int sf_static_box_depth_support(const obj_detection*d,const obj_scene_config*c,const obj_frame*f,const approach_observation*o,const lab_real_runtime_view*v,uint64_t now,double camx,double camy,double range){
 if(!d||strcmp(d->label,"box")||!sf_static_depth_available(d,f,now))return 0;
 uint64_t stamp=d->depth_time_us?d->depth_time_us:f->frame_us;if(sf_delta(stamp,v->scan_time_us)>300000)return 0;
 double error=0,residual=sf_depth_hint_aligned(d,c,f,o,v,now,camx,camy,range,&error);
 return residual<1e5&&residual<=fmax(.20,2*error);
}
static void sf_static_surface_point(sf_cluster*q,const sf_hit*h,unsigned n,double camx,double camy){
 /* 保留角度点簇前缘附近的实际回波，质心可能落在物体内部，不作为车辆目标表面。
 */
 double nearest=1e9;int best=-1;for(unsigned i=0;i<n;i++)if(h[i].angle>=q->min_angle-1e-8&&h[i].angle<=q->max_angle+1e-8){double range=hypot(h[i].x-camx,h[i].y-camy);if(range<nearest){nearest=range;best=(int)i;}}
 if(best>=0){q->x=h[best].x;q->y=h[best].y;q->camera_range=nearest;}
}
static int sf_scale_box(const double*b,double*area,double*aspect){
 if(!b||!isfinite(b[0])||!isfinite(b[1])||!isfinite(b[2])||!isfinite(b[3])||b[0]<2||b[1]<2||b[2]>958||b[3]>1078||b[2]-b[0]<4||b[3]-b[1]<4)return 0;
 *area=(b[2]-b[0])*(b[3]-b[1]);*aspect=(b[2]-b[0])/(b[3]-b[1]);return isfinite(*area)&&isfinite(*aspect);
}
static void sf_scale_predict(obj_scene*s,const obj_detection*d,double yaw,uint64_t now){
 s->scale_valid=0;s->scale_range_m=0;double area,aspect;
 if(!s->scale_reference_us||s->scale_reference_area<=0)return;
 if(!d->bbox_valid||!sf_scale_box(d->bbox,&area,&aspect)||d->eye!=s->scale_reference_eye||!sf_time(s->scale_reference_us,now,5000000)||fabs(aspect/s->scale_reference_aspect-1)>.25||fabs(sf_wrap(yaw-s->scale_reference_yaw))>20*SF_PI/180){s->scale_reference_area=0;return;}
 double ratio=area/s->scale_reference_area;if(ratio<.0625||ratio>16){s->scale_reference_area=0;return;}
 double range=s->scale_reference_range*sqrt(s->scale_reference_area/area);
 if(!isfinite(range)||range<=0||range>30)return;s->scale_range_m=range;s->scale_valid=1;
}
static void sf_scale_seed(obj_scene*s,const obj_detection*d,const double*box,double range,double yaw,uint64_t stamp,uint64_t now){
 double area,aspect;if(s->scale_reference_area>0||stamp<=s->scale_reference_us||!sf_time(stamp,now,800000)||!isfinite(range)||range<=0||range>30||!sf_scale_box(box,&area,&aspect))return;
 s->scale_reference_eye=d->eye;s->scale_reference_range=range;s->scale_reference_area=area;s->scale_reference_aspect=aspect;s->scale_reference_yaw=yaw;s->scale_reference_us=stamp;
}
static int sf_order(const void*a,const void*b){double x=((const sf_hit*)a)->angle,y=((const sf_hit*)b)->angle;return x<y?-1:x>y;}
static void sf_add_cluster_min(sf_cluster*c,unsigned*count,const sf_hit*h,unsigned start,unsigned end,double camx,double camy,unsigned minimum){
 if(end<=start||end-start<minimum||*count>=SF_CLUSTERS)return;
 /* 稀疏模式共用整扫优先准入，未进入固定32轨迹表的点组不能仅因角度较早而先占满64个裁剪槽。

 */
 if(minimum==1){int tracked=0;for(unsigned i=start;i<end;i++)if(h[i].full_track>=0&&h[i].full_track<(int)FV5_TRACKS){tracked=1;break;}if(!tracked)return;}
 sf_cluster*q=&c[(*count)++];memset(q,0,sizeof(*q));q->n=end-start;q->full_track=-1;unsigned tracks[FV5_TRACKS]={0},largest=0;
 for(unsigned i=start;i<end;i++){q->x+=h[i].x;q->y+=h[i].y;q->wall+=h[i].wall;if(h[i].full_track>=0&&h[i].full_track<(int)FV5_TRACKS){unsigned hits=++tracks[h[i].full_track];if(hits>largest){largest=hits;q->full_track=h[i].full_track;}}}
 q->wall/=q->n;
 q->x/=q->n;q->y/=q->n;q->min_angle=h[start].angle;q->max_angle=h[end-1].angle;q->span=hypot(h[start].x-h[end-1].x,h[start].y-h[end-1].y);q->camera_range=hypot(q->x-camx,q->y-camy);
}
static void sf_add_cluster(sf_cluster*c,unsigned*count,const sf_hit*h,unsigned a,unsigned b,double x,double y){sf_add_cluster_min(c,count,h,a,b,x,y,2);}
static void sf_commit(obj_scene*s,const obj_scene_config*c,const lab_real_runtime_view*v,const passage_config*p,uint64_t now,double x,double y,double error,uint64_t stamp,const char*source,const char*reason,const char*zh){
 if(s->source_us&&stamp<s->source_us){sf_hold(s,v,p,now,"TARGET_SOURCE_REORDERED","收到较旧的目标测量，保留更新的原位置，不刷新观测时间");return;}
 int consistent=s->target_odom_valid&&hypot(x-s->previous_x,y-s->previous_y)<=fmax(.25,.10*hypot(x-v->raw_odometry.x_m,y-v->raw_odometry.y_m));
 if(stamp>s->support_source_us){s->support=consistent?s->support+1:1;if(s->support>20)s->support=20;s->support_source_us=stamp;s->previous_x=x;s->previous_y=y;}
 if(!consistent&&stamp==s->support_source_us)s->support=1;
 s->target_odom_x=x;s->target_odom_y=y;s->target_odom_valid=1;s->source_us=stamp;s->error_m=fmax(c->mount_approximate?.12:.05,error);s->confirmed=s->support>=3;s->historical=0;s->approximate=c->mount_approximate||s->error_m>.1;
 snprintf(s->source,sizeof(s->source),"%s",source);sf_reason(s,reason,zh);
 if(!sf_project(s,v,p,now))sf_reason(s,"MAP_POSE_UNAVAILABLE","已取得目标表面观测，但当前地图位姿不可用，暂时不能生成地图坐标");
}
/* 雷达轨迹支持身份关联，不执行语义分类。视觉缺失时只允许围绕预测点有限延续，重复扫描不刷新时钟或支持次数。

 */
static void sf_lidar_update(obj_scene*s,double x,double y,double error,uint64_t stamp,uint64_t visual){
 if(!stamp||stamp<=s->lidar_source_us)return;
 double dt=s->lidar_source_us?(stamp-s->lidar_source_us)/1e6:0;
 if(s->lidar_valid&&dt>=.03&&dt<=2){double vx=(x-s->lidar_x)/dt,vy=(y-s->lidar_y)/dt;
  /* 表面切换时限制预测速度，近距原始值原样保留。 */
  s->lidar_vx=.65*s->lidar_vx+.35*fmax(-2,fmin(2,vx));s->lidar_vy=.65*s->lidar_vy+.35*fmax(-2,fmin(2,vy));
 }else s->lidar_vx=s->lidar_vy=0;
 s->lidar_valid=s->lidar_support_valid=1;s->lidar_source_us=stamp;s->lidar_x=x;s->lidar_y=y;s->lidar_error_m=error;
 if(visual>s->lidar_visual_us)s->lidar_visual_us=visual;if(s->lidar_updates<1000000)s->lidar_updates++;
}
static fv5_state*sf_follow_state(obj_scene*s){return (fv5_state*)(void*)s->follow_state_v5.bytes;}
static void sf_static_wall_hold(obj_scene*s,const lab_real_runtime_view*v,const passage_config*p,uint64_t now){
 if(s->target_odom_valid&&st10_wall_at(sf_follow_state(s),s->target_odom_x,s->target_odom_y)>=.75){s->target_odom_valid=s->target_valid=s->dock_valid=s->lidar_valid=0;s->support=0;s->support_source_us=0;}
 snprintf(s->follow_reason,sizeof(s->follow_reason),"STATIC_WALL_EXCLUDED");sf_hold(s,v,p,now,"STATIC_WALL_EXCLUDED","当前目标角区只有经完整扫描及区外延伸支持的背景墙；保留真实视觉方向，不把墙面距离当成目标");
}
static int sf_position_supported(const obj_scene*s,double x,double y,uint64_t now){
 return s->target_odom_valid&&sf_time(s->source_us,now,2000000)&&isfinite(s->error_m)&&s->error_m>=0&&s->error_m<=.35&&hypot(x-s->target_odom_x,y-s->target_odom_y)<=fmin(.90,.20+1.6*(now-s->source_us)/1e6+s->error_m);
}
static void sf_follow_prepare(obj_scene*s,const obj_frame*f,const approach_observation*o,const lab_real_runtime_view*v,const passage_config*p,uint64_t now,int walls){
 fv5_state*state=sf_follow_state(s);const struct scan_frame*scan=v->latest_scan;sf_pose pose;
 if(!scan||scan->count>FV5_BEAMS||!sf_time(v->scan_time_us,now,SF_SCAN_MAX_US)||!sf_scan_pose(o,f,v,now,&pose)||v->scan_time_us<=state->source_us)return;
 state->priority_us=s->target_odom_valid&&sf_time(s->source_us,now,2000000)?s->source_us:0;state->priority_x=s->target_odom_x;state->priority_y=s->target_odom_y;state->priority_gate=fmin(.90,.20+1.6*(state->priority_us?(now-state->priority_us)/1e6:0));
 fv5_point*points=malloc(sizeof(*points)*FV5_BEAMS);if(!points)return;unsigned n=0;double mx=p?p->mount_x_m:0,my=p?p->mount_y_m:0,ma=p?p->mount_yaw_rad:0;
 for(unsigned i=0;i<scan->count;i++){if(!scan_return_in_range(scan,i,p?p->lidar_min_m:.05,p?p->lidar_max_m:12))continue;const struct scan_point*q=&scan->points[i];double a=-q->angle_q14*(SF_PI/32768.)+ma,r=q->distance_q2/4000.,x,y;sf_transform(&pose,mx+r*cos(a),my+r*sin(a),&x,&y);points[n++]=(fv5_point){x,y,sf_wrap(a),r,i};}
 if(walls)fv5_prepare_mode(state,points,n,v->scan_time_us,scan->sequence,walls);else st10_prepare(state,points,n,v->scan_time_us,scan->sequence);free(points);
}
static void sf_follow_hit(obj_scene*s,sf_hit*h,unsigned beam,uint64_t stamp){
 fv5_state*state=sf_follow_state(s);h->full_track=-1;if(beam>=FV5_BEAMS||!state->prepared||state->source_us!=stamp)return;h->wall=state->wall[beam]/255.;if(state->track[beam]<FV5_TRACKS)h->full_track=state->track[beam];
}
static double sf_follow_map_wall(const lab_real_runtime_view*v,double x,double y,uint64_t now){
 const lab_localization_view*l=&v->localization;const lab_real_grid_view*g=&l->map;
 if(!v->has_localization||!l->has_pose||!g->cells||!g->width||!g->height||g->width>4096||g->height>4096||!isfinite(g->resolution_m)||g->resolution_m<=0||!isfinite(g->origin_x_m)||!isfinite(g->origin_y_m)||!sf_context(&l->context,&v->context)||!sf_time(l->pose_time_us,now,2000000)||!sf_finite_pose(l->map_odom_x_m,l->map_odom_y_m,l->map_odom_yaw_rad))return 0;
 double c=cos(l->map_odom_yaw_rad),sn=sin(l->map_odom_yaw_rad),mx=l->map_odom_x_m+c*x-sn*y,my=l->map_odom_y_m+sn*x+c*y;double xx=floor((mx-g->origin_x_m)/g->resolution_m),yy=floor((my-g->origin_y_m)/g->resolution_m);if(!isfinite(xx)||!isfinite(yy)||xx<0||yy<0||xx>=g->width||yy>=g->height)return 0;int ix=(int)xx,iy=(int)yy;
 if(ix<0||iy<0||ix>=(int)g->width||iy>=(int)g->height)return 0;return g->cells[(size_t)iy*g->width+(unsigned)ix]>=65?.18:0;
}
static void sf_follow_features(obj_scene*s,sf_cluster*q,const lab_real_runtime_view*v,uint64_t now){
 fv5_state*state=sf_follow_state(s);q->motion=0;q->isolation=fv5_clamp(1-q->span/.9,.15,1);q->wall=fmax(q->wall,sf_follow_map_wall(v,q->x,q->y,now));
 if(q->full_track>=0&&q->full_track<(int)FV5_TRACKS){const fv5_track*t=state->tracks+q->full_track;if(t->used&&t->last==v->scan_time_us){q->motion=t->motion;q->isolation=fmax(q->isolation,t->isolation);}}
}
static int sf_follow_select(obj_scene*s,sf_cluster*q,unsigned n,double camx,double camy,double bearing,double half,const lab_real_runtime_view*v,uint64_t now){
 fv5_state*state=sf_follow_state(s);int best=-1;double highest=-1e9;s->follow_candidates=n;
 for(unsigned i=0;i<n;i++){
  sf_follow_features(s,q+i,v,now);if(q[i].wall>=.78){s->follow_wall_rejected++;continue;}
  if(q[i].n==1){const fv5_track*t=q[i].full_track>=0?state->tracks+q[i].full_track:NULL;int known=(t&&q[i].full_track==state->chosen&&s->lidar_valid)||sf_position_supported(s,q[i].x,q[i].y,now);if(!t||!t->updated||t->last!=v->scan_time_us||(!known&&!fv5_sparse_supported(t)))continue;}
  double angle=fabs(sf_wrap(atan2(q[i].y-camy,q[i].x-camx)-bearing))/fmax(.035,half),visual=1/(1+angle*angle),continuity=s->lidar_valid?1/(1+pow(hypot(q[i].x-s->lidar_x,q[i].y-s->lidar_y)/.5,2)):0;
  double compact=fv5_clamp(1-fabs(q[i].span-.25)/1.,0,1);double score=2.5*visual+1.2*q[i].isolation+4.0*q[i].motion+.4*compact+1.8*continuity-4.5*q[i].wall-.15*fmin(q[i].camera_range,10);
  if(q[i].full_track==state->chosen&&state->chosen>=0)score+=1.0; /* 选中目标停止移动后仍保留身份。 */
  if(score>highest){highest=score;best=(int)i;}
 }
 if(best<0){s->follow_wall_score=n?1:0;s->follow_motion_score=0;s->follow_isolation_score=0;snprintf(s->follow_reason,sizeof(s->follow_reason),"%s",s->follow_wall_rejected?"FOLLOW_WALL_EXCLUDED":"FOLLOW_REASSOCIATING_NO_SURFACE");return -1;}
 sf_cluster*b=q+best;s->follow_sparse=b->n==1;s->follow_support_points=b->n;s->follow_wall_score=b->wall;s->follow_motion_score=b->motion;s->follow_isolation_score=b->isolation;s->follow_motion_support=b->full_track>=0?state->tracks[b->full_track].hits:0;
 snprintf(s->follow_reason,sizeof(s->follow_reason),"%s",b->n==1?"FOLLOW_SPARSE_TRACK":b->motion>.35?"FOLLOW_MOVING_CANDIDATE":"FOLLOW_STATIONARY_CANDIDATE");return best;
}
static void sf_follow_binding(obj_scene*s,const sf_cluster*q){
 fv5_state*state=sf_follow_state(s);double displacement=s->target_odom_valid?hypot(q->x-s->target_odom_x,q->y-s->target_odom_y):0;double old_wall=s->target_odom_valid?fv5_wall_at(state,s->target_odom_x,s->target_odom_y):0;
 int corrected=s->target_odom_valid&&((old_wall>=.70&&displacement>.30)||(state->chosen>=0&&q->full_track!=state->chosen&&displacement>.65&&q->isolation>.60));
 if(corrected){if(s->association_revision<UINT32_MAX)s->association_revision++;s->lidar_valid=0;s->lidar_vx=s->lidar_vy=0;s->support=0;s->support_source_us=0;snprintf(s->follow_reason,sizeof(s->follow_reason),"FOLLOW_ASSOCIATION_CORRECTED");}
 state->chosen=q->full_track;
}
static int sf_follow_continue(obj_scene*s,const obj_scene_config*c,const obj_frame*f,const approach_observation*o,const lab_real_runtime_view*v,const passage_config*p,uint64_t now){
 fv5_state*state=sf_follow_state(s);sf_pose pose;if(!s->lidar_valid||!state->prepared||state->source_us!=v->scan_time_us||v->scan_time_us<=s->lidar_source_us||!sf_scan_pose(o,f,v,now,&pose))return 0;
 uint32_t initial_revision=s->association_revision;
 int id=state->chosen;fv5_track*t=id>=0&&id<(int)FV5_TRACKS?state->tracks+id:NULL;
 if(!t||!t->used||!t->updated||t->last!=v->scan_time_us||fv5_wall_at(state,t->x,t->y)>=.78){
  /* 按最后真实视觉角域重新关联，不沿用误选旧墙点；两个不同新扫描支持修正，但不作为新视觉观测。
 */
  int candidate=-1;double score=-1e9;if(sf_time(state->visual_us,now,5000000))for(unsigned i=0;i<FV5_TRACKS;i++){
   const fv5_track*q=state->tracks+i;if(!q->used||!q->updated||q->last!=v->scan_time_us||q->hits<2||(q->points==1&&!fv5_sparse_supported(q))||q->isolation<.45||q->span>.65||fv5_wall_at(state,q->x,q->y)>=.70)continue;
   double angle=fabs(sf_wrap(atan2(q->y-state->visual_y,q->x-state->visual_x)-state->visual_yaw));double half=fmin(1.0,state->visual_half+.20+.10*(now-state->visual_us)/1e6);if(angle>half)continue;
   double value=1.5*q->isolation+q->motion+1/(1+angle*angle)-.05*hypot(q->x-state->visual_x,q->y-state->visual_y);if(value>score){score=value;candidate=(int)i;}
  }
  if(candidate<0){state->pending_hits=0;return 0;}
  if(state->pending_track!=candidate){state->pending_track=candidate;state->pending_hits=0;state->pending_us=0;}
  if(state->pending_us!=v->scan_time_us){state->pending_us=v->scan_time_us;state->pending_hits++;}
  if(state->pending_hits<2){snprintf(s->follow_reason,sizeof(s->follow_reason),"FOLLOW_REASSOCIATING_NO_SURFACE");return 0;}
  t=state->tracks+candidate;sf_cluster corrected={0};corrected.x=t->x;corrected.y=t->y;corrected.full_track=candidate;corrected.isolation=t->isolation;sf_follow_binding(s,&corrected);state->pending_hits=0;
 }
 s->follow_sparse=t->points==1;s->follow_support_points=t->points;
 double error=.12+.025*hypot(t->x-pose.x,t->y-pose.y)+pose.error+(s->follow_sparse?.15:0);
 sf_commit(s,c,v,p,now,t->x,t->y,error,t->last,"LIDAR_TARGET_TRACK","FOLLOW_RADAR_TRACK","当前新扫描仍跟踪到原独立目标；静止也保留，视觉缺失不刷新图像时刻");
 sf_lidar_update(s,t->x,t->y,error,t->last,0);s->follow_motion_score=t->motion;s->follow_wall_score=fv5_wall_at(state,t->x,t->y);s->follow_isolation_score=t->isolation;s->follow_motion_support=t->hits;snprintf(s->follow_reason,sizeof(s->follow_reason),"%s",s->association_revision!=initial_revision?"FOLLOW_ASSOCIATION_CORRECTED":s->follow_sparse?"FOLLOW_SPARSE_TRACK":"FOLLOW_RADAR_TRACK");
 if(s->follow_sparse)s->confirmed=0;
 sf_measure(s,c,f,v,p,t->x,t->y,error,t->last,OBJECT_MEASURE_RADAR_VISUAL);return 1;
}
static int sf_lidar_continue(obj_scene*s,const obj_scene_config*c,const obj_frame*f,const approach_observation*o,const lab_real_runtime_view*v,const passage_config*p,uint64_t now){
 if(c->mode==2)return sf_follow_continue(s,c,f,o,v,p,now);
 /* 静止模式也使用当前扫描关联，但不采用跟随模式的墙面否决和运动偏好；稀疏支持不确认到达。
 */
 if(c->mode==1&&s->lidar_valid&&sf_time(s->lidar_source_us,now,2000000)&&v->scan_time_us>s->lidar_source_us){
  fv5_state*state=sf_follow_state(s);int id=state->chosen;const fv5_track*t=id>=0&&id<(int)FV5_TRACKS?state->tracks+id:NULL;sf_pose pose;
  if(state->source_us==v->scan_time_us&&t&&t->used&&t->updated&&t->last==v->scan_time_us&&st10_wall_at(state,t->x,t->y)<.75&&sf_scan_pose(o,f,v,now,&pose)){
   s->follow_sparse=t->points==1;s->follow_support_points=t->points;double error=.12+.025*hypot(t->x-pose.x,t->y-pose.y)+pose.error+(s->follow_sparse?.15:0);
   sf_commit(s,c,v,p,now,t->x,t->y,error,t->last,"LIDAR_TARGET_TRACK","STATIC_RADAR_TRACK","当前新扫描与原静态目标连续；单回波仅作弱距离支持，不确认到达");
   sf_lidar_update(s,t->x,t->y,error,t->last,0);if(s->follow_sparse)s->confirmed=0;sf_measure(s,c,f,v,p,t->x,t->y,error,t->last,OBJECT_MEASURE_RADAR_VISUAL);return 1;
  }
 }
 const struct scan_frame*scan=v->latest_scan;sf_pose pose;
 if(!s->lidar_valid||!f||(c->mode!=2&&!sf_time(s->lidar_visual_us,now,5000000))||!sf_time(s->lidar_source_us,now,1500000)||!scan||scan->count>SCAN_MAX_POINTS||!sf_time(v->scan_time_us,now,SF_SCAN_MAX_US)||v->scan_time_us<=s->lidar_source_us||!sf_scan_pose(o,f,v,now,&pose))return 0;
 double dt=(v->scan_time_us-s->lidar_source_us)/1e6;if(dt<=0||dt>1.5)return 0;
 double predx=s->lidar_x+s->lidar_vx*fmin(dt,1.),predy=s->lidar_y+s->lidar_vy*fmin(dt,1.);
 double gate=fmin(.85,.25+dt*.4+fmin(.20,s->lidar_error_m));
 sf_hit*hits=malloc(sizeof(*hits)*SCAN_MAX_POINTS);if(!hits)return 0;unsigned n=0,nc=0;sf_cluster groups[SF_CLUSTERS];
 double mx=p?p->mount_x_m:0,my=p?p->mount_y_m:0,ma=p?p->mount_yaw_rad:0;
 for(unsigned i=0;i<scan->count;i++){if(!scan_return_in_range(scan,i,p?p->lidar_min_m:.05,fmin(c->range_max_m,p?p->lidar_max_m:12)))continue;
  const struct scan_point*q=&scan->points[i];double a=-q->angle_q14*(SF_PI/32768.)+ma,r=q->distance_q2/4000.,x,y;sf_transform(&pose,mx+r*cos(a),my+r*sin(a),&x,&y);
  if(c->mode==1&&st10_wall_at(sf_follow_state(s),x,y)>=.75)continue;
  if(hypot(x-predx,y-predy)>gate)continue;hits[n++]=(sf_hit){x,y,sf_wrap(a),0,r};
 }
 qsort(hits,n,sizeof(*hits),sf_order);unsigned start=0;
 for(unsigned i=1;i<=n;i++)if(i==n||((c->mode==1||c->mode==2)?!fv5_connected(hits[i-1].x,hits[i-1].y,hits[i-1].angle,hits[i-1].range,hits[i].x,hits[i].y,hits[i].angle,hits[i].range):(hypot(hits[i].x-hits[i-1].x,hits[i].y-hits[i-1].y)>fmin(.35,.18+.015*hits[i-1].range)||hits[i].angle-hits[i-1].angle>.06))){sf_add_cluster(groups,&nc,hits,start,i,pose.x,pose.y);start=i;}free(hits);
 int best=-1;double bs=1e9,second=1e9;for(unsigned i=0;i<nc;i++){double score=hypot(groups[i].x-predx,groups[i].y-predy)/gate+.10/fmin(8,groups[i].n);if(score<bs){second=bs;bs=score;best=(int)i;}else if(score<second)second=score;}
 s->lidar_candidates=nc;if(best<0||(nc>1&&second-bs<.20))return 0;
 sf_cluster*q=&groups[best];double error=.12+.025*q->camera_range+pose.error+dt*.10;
 sf_commit(s,c,v,p,now,q->x,q->y,error,v->scan_time_us,"LIDAR_TARGET_TRACK","LIDAR_TRACK_VISUAL_GAP","视觉短暂缺失，当前新雷达点簇与原目标轨迹连续；按真实扫描更新目标，等待视觉复核");
 sf_lidar_update(s,q->x,q->y,error,v->scan_time_us,0);
 sf_measure(s,c,f,v,p,q->x,q->y,error,v->scan_time_us,OBJECT_MEASURE_RADAR_VISUAL);return 1;
}
void obj_scene_fuse(obj_scene*s,const obj_scene_config*cfg,const obj_frame*f,const approach_observation*o,const obj_radar_anchor_view*a,const lab_real_runtime_view*v,const passage_config*p,uint64_t now){
 if(!s||!v)return;obj_scene_config defaults;if(!cfg){obj_scene_config_default(&defaults);cfg=&defaults;}
 uint64_t epoch=f?f->epoch:0;int id=f?f->selected:-1;
 if(!s->context_valid||!sf_context(&s->context,&v->context)||(!cfg->mode&&s->map_id!=v->localization.map.map_id)||s->epoch!=epoch||s->track_id!=id){
  memset(s,0,sizeof(*s));s->context=v->context;s->context_valid=1;s->epoch=epoch;s->track_id=id;s->map_id=v->localization.map.map_id;
 }
 if(s->map_id!=v->localization.map.map_id){s->map_id=v->localization.map.map_id;s->target_valid=s->dock_valid=0;s->path_count=0;s->plan_us=0;}
 memset(&s->measurement_v2,0,sizeof(s->measurement_v2));s->lidar_support_valid=0;s->lidar_candidates=0;s->stereo_prior_only=s->stereo_conflict=0;s->follow_sparse=s->follow_support_points=0;
 s->published_us=now;s->map_version=v->localization.map.map_version;s->approximate=cfg->mount_approximate;s->standoff_m=cfg->standoff_m;s->frame_id=f?f->frame_id:0;s->scan_us=v->scan_time_us;s->scale_valid=0;s->scale_range_m=0;
 if(!isfinite(cfg->camera_ahead_lidar_m)||fabs(cfg->camera_ahead_lidar_m)>2||!isfinite(cfg->standoff_m)||cfg->standoff_m<.05||cfg->standoff_m>3||!isfinite(cfg->range_max_m)||cfg->range_max_m<=0||cfg->range_max_m>30||!cfg->coarse_max_age_us){sf_clear_target(s);sf_reason(s,"SCENE_CONFIG_INVALID","目标融合参数无效，未生成导航位置");return;}
 if(!f||!epoch||id<1){sf_clear_target(s);s->bearing_valid=0;sf_reason(s,"NO_SELECTED_OBJECT","尚未选择真实物体，只有雷达点云，不生成目标位置");return;}
 const obj_detection*d=NULL;for(int i=0;i<f->count&&i<OBJ_MAX;i++)if(f->objects[i].track_id==id){d=&f->objects[i];break;}
 if(d&&d->binding_generation<s->binding_generation){sf_hold(s,v,p,now,"OLDER_BINDING_FRAME_IGNORED","收到旧绑定的图像状态，保留较新的目标轨迹，不将旧位置或测距重新标成当前目标");return;}
 if(d&&d->binding_generation!=s->binding_generation){
  uint32_t binding=d->binding_generation;memset(s,0,sizeof(*s));s->context=v->context;s->context_valid=1;s->epoch=epoch;s->track_id=id;s->map_id=v->localization.map.map_id;s->map_version=v->localization.map.map_version;s->published_us=now;s->frame_id=f->frame_id;s->scan_us=v->scan_time_us;s->standoff_m=cfg->standoff_m;s->approximate=cfg->mount_approximate;s->binding_generation=binding;
 }
 int observation=o&&(!o->context_valid||sf_context(&o->context,&v->context))&&o->epoch==epoch&&o->track_id==id;
 if(!d||!d->observed||!strcmp(d->tracking,"LOST")||!strcmp(d->tracking,"PREDICTED")||!sf_time(f->frame_us,now,SF_VISUAL_MAX_US)){
  if(observation&&o->direction_valid&&isfinite(o->bearing_rad)){s->bearing_valid=1;s->bearing_rad=o->bearing_rad;}
  if(cfg->mode==1||cfg->mode==2)sf_follow_prepare(s,f,o,v,p,now,cfg->mode==2);
  if(sf_lidar_continue(s,cfg,f,o,v,p,now))return;
  if(cfg->mode==1&&sf_follow_state(s)->source_us==v->scan_time_us&&st10_background_in_target(sf_follow_state(s))){sf_static_wall_hold(s,v,p,now);return;}
  sf_hold(s,v,p,now,"TARGET_NOT_CURRENTLY_OBSERVED","目标当前没有新视觉观测；保留原位置记忆，不将其他同类物体自动当成目标");return;
 }
 double bearing=d->bearing_valid?d->bearing:observation&&o->direction_valid?o->bearing_rad:NAN;
 if(!isfinite(bearing)||fabs(bearing)>SF_PI){s->bearing_valid=0;sf_hold(s,v,p,now,"TARGET_BEARING_UNAVAILABLE","已识别物体，但单目方向未就绪，不能将整片雷达回波当成目标");return;}
 if(d->ray_direction_valid)bearing=sf_ray_bearing(cfg,d->ray_direction);
 else if(cfg->mount_configured)bearing=sf_wrap(bearing+cfg->camera.yaw);
 if(!isfinite(bearing)){sf_hold(s,v,p,now,"CAMERA_RAY_VERTICAL","相机射线接近竖直，无法确定水平方向；请检查安装旋转参数");return;}
 s->bearing_rad=bearing;s->bearing_valid=1;
 sf_pose image_pose;if(!sf_image_pose(o,f,v,now,&image_pose)){sf_hold(s,v,p,now,"IMAGE_CAPTURE_POSE_UNAVAILABLE","目标图像采集时刻缺少对应里程计位姿，暂保留方向与历史位置");return;}
 if(d->geometry_eye!=0){sf_hold(s,v,p,now,"RIGHT_EYE_CANONICAL_EXTRINSICS_UNAVAILABLE","当前右眼方向尚未转换到左眼安装坐标；请核对 stereo_extrinsics_file 与 left_eye=0，暂保留原目标记忆和单眼方向");return;}
 sf_scale_predict(s,d,image_pose.yaw,now);
 double mount_x=p?p->mount_x_m:0,mount_y=p?p->mount_y_m:0,mount_yaw=p?p->mount_yaw_rad:0;
 double origin[3],offset[3]={0};obj_scene_camera_origin(cfg,p,origin);if(d->ray_origin_valid)obj_scene_camera_vector(cfg,d->ray_origin,offset);
 double camera_x=origin[0]+offset[0],camera_y=origin[1]+offset[1],camx,camy;
 sf_transform(&image_pose,camera_x,camera_y,&camx,&camy);double world_bearing=image_pose.yaw+bearing;
 double half=.12;if(d->bearing_edges_valid){double left=d->ray_direction_valid?sf_ray_bearing(cfg,d->ray_left):sf_wrap(d->bearing_left+(cfg->mount_configured?cfg->camera.yaw:0)),right=d->ray_direction_valid?sf_ray_bearing(cfg,d->ray_right):sf_wrap(d->bearing_right+(cfg->mount_configured?cfg->camera.yaw:0));half=fmax(fabs(sf_wrap(left-bearing)),fabs(sf_wrap(right-bearing)))+.035;half=fmax(.035,fmin(.55,half));}
 if(cfg->mode==2){double r=s->target_odom_valid?hypot(s->target_odom_x-camx,s->target_odom_y-camy):3;half=fmin(.65,fmax(half,.10+.012*fmin(r,10)));s->follow_search_half_rad=half;}
 if(cfg->mode==1||cfg->mode==2){fv5_state*state=sf_follow_state(s);uint64_t image=0,frame=0;if(sf_visual_source(f,id,&image,&frame)){state->visual_us=image;state->visual_frame=frame;state->visual_x=camx;state->visual_y=camy;state->visual_yaw=world_bearing;state->visual_half=half;}}
 if(cfg->mode==1||cfg->mode==2)sf_follow_prepare(s,f,o,v,p,now,cfg->mode==2);
 const struct scan_frame*scan=v->latest_scan;sf_pose scan_pose;int range_rejected=0,ambiguous=0;
 if(scan&&scan->count<=SCAN_MAX_POINTS&&sf_time(v->scan_time_us,now,SF_SCAN_MAX_US)&&sf_delta(f->frame_us,v->scan_time_us)<=800000&&sf_scan_pose(o,f,v,now,&scan_pose)){
  sf_hit*hits=malloc(sizeof(*hits)*SCAN_MAX_POINTS);sf_cluster clusters[SF_CLUSTERS];unsigned n=0,nc=0;
  if(hits){s->follow_wall_rejected=0;for(unsigned i=0;i<scan->count;i++){
   const struct scan_point*q=&scan->points[i];double range=scan_acquisition_range_m(scan,i);
   if(!scan_return_in_range(scan,i,p?p->lidar_min_m:.05,fmin(cfg->range_max_m,p?p->lidar_max_m:12)))continue;
   double corrected=q->distance_q2/4000.0;double angle=-q->angle_q14*(SF_PI/32768.0)+mount_yaw,x=mount_x+corrected*cos(angle),y=mount_y+corrected*sin(angle),wx,wy;
   sf_transform(&scan_pose,x,y,&wx,&wy);double diff=sf_wrap(atan2(wy-camy,wx-camx)-world_bearing);
   if(fabs(diff)>half)continue;hits[n++]=(sf_hit){wx,wy,sf_wrap(angle),diff,(cfg->mode==1||cfg->mode==2)?corrected:range};if(cfg->mode==1||cfg->mode==2)sf_follow_hit(s,hits+n-1,i,v->scan_time_us);if((cfg->mode==1||cfg->mode==2)&&hits[n-1].wall>=.75&&!(cfg->mode==1&&sf_static_box_depth_support(d,cfg,f,o,v,now,camx,camy,hypot(wx-camx,wy-camy)))){s->follow_wall_rejected++;n--;continue;}
  }
  qsort(hits,n,sizeof(*hits),sf_order);unsigned start=0;
  for(unsigned i=1;i<=n;i++)if(i==n||((cfg->mode==1||cfg->mode==2)?!fv5_connected(hits[i-1].x,hits[i-1].y,hits[i-1].angle,hits[i-1].range,hits[i].x,hits[i].y,hits[i].angle,hits[i].range):(hypot(hits[i].x-hits[i-1].x,hits[i].y-hits[i-1].y)>fmin(.35,.18+.015*hits[i-1].range)||hits[i].angle-hits[i-1].angle>.06))||(cfg->mode==2&&((hits[i].wall>=.75)!=(hits[i-1].wall>=.75)))){sf_add_cluster_min(clusters,&nc,hits,start,i,camx,camy,cfg->mode==1?0:cfg->mode==2?1:2);start=i;}
  if(cfg->mode==1)for(unsigned k=0;k<nc;k++)sf_static_surface_point(clusters+k,hits,n,camx,camy);
  free(hits);
  int best=-1;double best_score=1e9,second=1e9;
  uint64_t hint_stamp=d->depth_time_us?d->depth_time_us:f->frame_us;
  int hint_current=sf_time(hint_stamp,now,1200000)&&sf_delta(hint_stamp,v->scan_time_us)<=300000;
  s->stereo_prior_only=!hint_current;s->lidar_candidates=nc;
  for(unsigned i=0;i<nc;i++){
   sf_cluster*q=&clusters[i];double angular=fabs(sf_wrap(atan2(q->y-camy,q->x-camx)-world_bearing))/half;
   double hint_error=0,res=sf_depth_hint_aligned(d,cfg,f,o,v,now,camx,camy,q->camera_range,&hint_error);q->stereo_residual=res;
   double score=angular+.15/fmin(8,q->n);if(res<1e5&&hint_current)score+=.8*fmin(3,res/fmax(.25,hint_error));
   if(s->lidar_valid&&sf_time(s->lidar_source_us,now,1500000)){double dt=fmin(1.,(now-s->lidar_source_us)/1e6);score+=.50*fmin(3,hypot(q->x-s->lidar_x-s->lidar_vx*dt,q->y-s->lidar_y-s->lidar_vy*dt)/fmax(.30,.10*q->camera_range));}
   if(s->scale_valid)score+=.10*fmin(1,fabs(q->camera_range-s->scale_range_m)/fmax(.25,.30*s->scale_range_m));
   if(s->target_odom_valid&&sf_time(s->source_us,now,2000000))score+=.35*fmin(3,hypot(q->x-s->target_odom_x,q->y-s->target_odom_y)/fmax(.35,.12*q->camera_range));
   else if(a&&observation&&o->anchor_valid&&a->valid&&sf_time(a->observed_us,now,cfg->coarse_max_age_us))score+=.25*fmin(3,hypot(q->x-a->odom_x,q->y-a->odom_y)/.4);
   q->score=score;if(score<best_score){second=best_score;best_score=score;best=(int)i;}else if(score<second)second=score;
  }
  /* 简单场景策略在实际选中视觉锥内选择最近独立前景点簇，仍要求几何一致，不将全局最近点或旧双目当作目标。

 */
  if(cfg->mode==1){
   int foreground=-1;for(unsigned i=0;i<nc;i++)if(clusters[i].score<1e8&&(foreground<0||clusters[i].camera_range<clusters[foreground].camera_range-.08||(fabs(clusters[i].camera_range-clusters[foreground].camera_range)<=.08&&clusters[i].score<clusters[foreground].score)))foreground=(int)i;
   best=foreground;
  }
  if(cfg->mode==2)best=sf_follow_select(s,clusters,nc,camx,camy,world_bearing,half,v,now);
  if(best>=0){sf_cluster*q=&clusters[best];if(cfg->mode==1){s->follow_sparse=q->n==1;s->follow_support_points=q->n;}ambiguous=nc>1&&second-best_score<.25;
   if(cfg->mode==1||cfg->mode==2)ambiguous=0;
   /* 双目提示完全不一致时降低关联质量，不因墙点更多而直接选取目标后方墙面。
 */
   double hint_error=0;double res=sf_depth_hint_aligned(d,cfg,f,o,v,now,camx,camy,q->camera_range,&hint_error);
   range_rejected=hint_current&&res<1e5&&res>fmax(.50,2.5*hint_error);s->stereo_conflict=range_rejected;
   if(cfg->mode==1||cfg->mode==2)range_rejected=0;
   /* 独立且方向一致的真实前景即使与双目不符，也进入原始近处保护，不退回远处记忆。
 */
   if(range_rejected&&nc==1&&fabs(sf_wrap(atan2(q->y-camy,q->x-camx)-world_bearing))/half<.8)range_rejected=0;
   if(!ambiguous&&!range_rejected){double pose_error=image_pose.error+scan_pose.error;double error=.08+.025*q->camera_range+.25*fmin(q->span,q->camera_range*half)+pose_error;
    if(cfg->mount_approximate)error+=.10;
    int fresh=v->scan_time_us>s->support_source_us;
    if(cfg->mode==2&&fresh)sf_follow_binding(s,q);else if(cfg->mode==1&&fresh){if(s->target_odom_valid&&st10_wall_at(sf_follow_state(s),s->target_odom_x,s->target_odom_y)>=.75&&hypot(q->x-s->target_odom_x,q->y-s->target_odom_y)>.025){if(s->association_revision<UINT32_MAX)s->association_revision++;s->lidar_valid=0;s->lidar_vx=s->lidar_vy=0;s->support=0;s->support_source_us=0;snprintf(s->follow_reason,sizeof(s->follow_reason),"STATIC_ASSOCIATION_CORRECTED");}sf_follow_state(s)->chosen=q->full_track;}
    sf_commit(s,cfg,v,p,now,q->x,q->y,error,v->scan_time_us,res<1e5&&hint_current?"RADAR_VISUAL_STEREO":"RADAR_VISUAL","REAL_SURFACE_POSITION","由真实雷达回波与目标方向生成表面粗位置；相机安装参数为近似值");
    if(s->follow_sparse){s->confirmed=0;s->error_m=fmax(s->error_m,.30);}
    if(s->source_us==v->scan_time_us){uint64_t image=0,frame=0;if(sf_visual_source(f,s->track_id,&image,&frame)){sf_lidar_update(s,q->x,q->y,error,v->scan_time_us,image);s->lidar_visual_frame=frame;}}
    if(s->stereo_conflict)sf_reason(s,"FRESH_LIDAR_STEREO_CONFLICT","当前唯一雷达目标点簇与双目距离冲突；采用新雷达表面观测并持续复核，未回退旧远距离");
    else if(s->stereo_prior_only&&res<1e5)sf_reason(s,"FRESH_LIDAR_OLD_STEREO_PRIOR","采用当前雷达目标表面；较旧双目仅作历史参考，不否决当前近距");
    if(fresh&&s->source_us==v->scan_time_us)sf_measure(s,cfg,f,v,p,q->x,q->y,error+(s->follow_sparse?.15:0),v->scan_time_us,res<1e5&&hint_current?OBJECT_MEASURE_RADAR_STEREO:OBJECT_MEASURE_RADAR_VISUAL);
    if(s->support>=3&&s->source_us==v->scan_time_us&&sf_delta(f->frame_us,v->scan_time_us)<=200000){sf_scale_seed(s,d,d->bbox,q->camera_range,image_pose.yaw,v->scan_time_us,now);sf_scale_predict(s,d,image_pose.yaw,now);}return;}
  }
  }
 }
 if(cfg->mode==1&&sf_follow_state(s)->source_us==v->scan_time_us&&st10_background_in_target(sf_follow_state(s))&&!sf_static_depth_available(d,f,now)){sf_static_wall_hold(s,v,p,now);return;}
 /* 雷达点组不唯一时可由双目形成粗位置，规范双目几何属于物理左眼，不随显示眼切换。
 */
 uint64_t depth_stamp=d->depth_time_us?d->depth_time_us:f->frame_us;sf_pose dp;
 if((d->coarse_valid||d->depth_valid)&&sf_time(depth_stamp,now,cfg->coarse_max_age_us)&&sf_depth_pose(o,f,v,depth_stamp,now,&dp)){
  double r=d->coarse_valid?d->coarse_range:hypot(d->x,d->z),be=d->depth_bearing_valid?d->depth_bearing:observation&&o->stereo_bearing_valid&&o->stereo_us==depth_stamp?o->stereo_bearing_rad:d->depth_valid?atan2(-d->x,d->z):depth_stamp==f->frame_us?bearing:NAN;
  if(isfinite(r)&&r>.05&&r<=cfg->range_max_m&&isfinite(be)){
   double x,y,bvec[3],op[3];
   if(d->depth_valid){op[0]=d->x;op[1]=d->y;op[2]=d->z;}else{op[0]=-r*sin(be);op[1]=0;op[2]=r*cos(be);}
   obj_scene_camera_vector(cfg,op,bvec);sf_transform(&dp,origin[0]+bvec[0],origin[1]+bvec[1],&x,&y);
   double e=(d->coarse_valid?d->coarse_error:d->error);if(!isfinite(e)||e<0)e=.5;
   e=fmax(.15,fmax(e,.15*r))+dp.error+(cfg->mount_approximate?.10:0)+(now-depth_stamp)/1000000.*.05;
   if(cfg->mode==2&&fv5_wall_at(sf_follow_state(s),x,y)>=.78){snprintf(s->follow_reason,sizeof(s->follow_reason),"FOLLOW_WALL_EXCLUDED");sf_hold(s,v,p,now,"FOLLOW_WALL_EXCLUDED","当前方向只有墙面回波，双目粗距也落在墙面；保留寻找方向，未把墙面当作目标");return;}
   int fresh=depth_stamp>s->support_source_us;
   sf_commit(s,cfg,v,p,now,x,y,e,depth_stamp,"STEREO_COARSE",ambiguous?"STEREO_COARSE_RADAR_AMBIGUOUS":"STEREO_COARSE_POSITION",ambiguous?"双目粗距生成候选位置；当前雷达方向内存在多个相近候选，尚未确定对应点簇":"由原始双目及采集位姿生成表面候选；低矮目标无需等待雷达回波");s->confirmed=0;
   if(fresh&&s->source_us==depth_stamp)sf_measure(s,cfg,f,v,p,x,y,e,depth_stamp,OBJECT_MEASURE_STEREO_COARSE);
   if(d->depth_valid&&s->source_us==depth_stamp&&d->depth_eye==d->eye&&d->bbox_valid){const double*source_box=d->depth_time_us?d->depth_bbox:d->bbox;sf_scale_seed(s,d,source_box,r,dp.yaw,depth_stamp,now);sf_scale_predict(s,d,image_pose.yaw,now);}return;
  }
 }
 sf_hold(s,v,p,now,ambiguous?"RADAR_CANDIDATES_AMBIGUOUS":range_rejected?"RADAR_STEREO_RANGE_CONFLICT":"BEARING_ONLY_NO_TARGET_RANGE",ambiguous?"目标方向内多个雷达点簇得分接近，暂保留方向或原位置，未任意绑定障碍物":range_rejected?"雷达候选与双目粗距明显冲突，暂保留方向或原位置，等待新的实测":"目标方向可用，但没有对应的有效非零雷达点簇或双目粗距；尚未生成目标坐标");
}

size_t obj_scene_candidates(const obj_scene_config*c,const obj_frame*f,const approach_observation*o,const lab_real_runtime_view*v,const passage_config*p,uint64_t now,obj_scene_candidate*out,size_t capacity){
 if(!c||!f||!v||!out||!capacity||capacity>16||f->count<0||f->count>OBJ_MAX||!sf_time(f->frame_us,now,1200000)||!v->latest_scan||v->latest_scan->count>SCAN_MAX_POINTS||!sf_time(v->scan_time_us,now,800000)||sf_delta(f->frame_us,v->scan_time_us)>800000)return 0;
 sf_pose image,scan_pose;if(!sf_image_pose(o,f,v,now,&image)||!sf_scan_pose(o,f,v,now,&scan_pose))return 0;
 sf_hit*hits=malloc(sizeof(*hits)*SCAN_MAX_POINTS);if(!hits)return 0;size_t count=0;const struct scan_frame*scan=v->latest_scan;
 const double mx=p?p->mount_x_m:0,my=p?p->mount_y_m:0,ma=p?p->mount_yaw_rad:0;
 double origin[3];obj_scene_camera_origin(c,p,origin);
 for(int k=0;k<f->count&&count<capacity;k++){
  const obj_detection*d=&f->objects[k];if(!d->observed||!d->bbox_valid||d->geometry_eye!=0||!strcmp(d->tracking,"PREDICTED")||!strcmp(d->tracking,"LOST"))continue;
  double b=d->ray_direction_valid?sf_ray_bearing(c,d->ray_direction):d->bearing_valid?sf_wrap(d->bearing+(c->mount_configured?c->camera.yaw:0)):NAN;if(!isfinite(b))continue;
  double half=.12;if(d->bearing_edges_valid){double left=d->ray_direction_valid?sf_ray_bearing(c,d->ray_left):sf_wrap(d->bearing_left+(c->mount_configured?c->camera.yaw:0)),right=d->ray_direction_valid?sf_ray_bearing(c,d->ray_right):sf_wrap(d->bearing_right+(c->mount_configured?c->camera.yaw:0));half=fmax(.035,fmin(.55,fmax(fabs(sf_wrap(left-b)),fabs(sf_wrap(right-b)))+.035));}
  double offset[3]={0};if(d->ray_origin_valid)obj_scene_camera_vector(c,d->ray_origin,offset);double camx,camy;sf_transform(&image,origin[0]+offset[0],origin[1]+offset[1],&camx,&camy);
  fv5_state*static_state=NULL;
  if(c->mode==1){
   static_state=calloc(1,sizeof(*static_state));fv5_point*full=malloc(FV5_BEAMS*sizeof(*full));
   if(!static_state||!full){free(static_state);free(full);continue;}unsigned nf=0;
   static_state->visual_us=f->frame_us;static_state->visual_x=camx;static_state->visual_y=camy;static_state->visual_yaw=image.yaw+b;static_state->visual_half=half;
   for(unsigned i=0;i<scan->count;i++){if(!scan_return_in_range(scan,i,p?p->lidar_min_m:.05,p?p->lidar_max_m:12))continue;const struct scan_point*q=&scan->points[i];double angle=-q->angle_q14*(SF_PI/32768.)+ma,radius=q->distance_q2/4000.,x,y;sf_transform(&scan_pose,mx+radius*cos(angle),my+radius*sin(angle),&x,&y);full[nf++]=(fv5_point){x,y,sf_wrap(angle),radius,i};}
   int prepared=st10_prepare(static_state,full,nf,v->scan_time_us,scan->sequence);free(full);if(!prepared){free(static_state);continue;}
  }
  unsigned n=0,nc=0;sf_cluster clusters[SF_CLUSTERS];
  for(unsigned i=0;i<scan->count;i++){
   if(!scan_return_in_range(scan,i,p?p->lidar_min_m:.05,fmin(c->range_max_m,p?p->lidar_max_m:12)))continue;
   if(static_state&&static_state->wall[i]>=190)continue;
   const struct scan_point*q=&scan->points[i];double angle=-q->angle_q14*(SF_PI/32768.)+ma,range=q->distance_q2/4000.,x,y;sf_transform(&scan_pose,mx+range*cos(angle),my+range*sin(angle),&x,&y);
   double diff=sf_wrap(atan2(y-camy,x-camx)-(image.yaw+b));if(fabs(diff)>half)continue;hits[n++]=(sf_hit){x,y,sf_wrap(angle),diff,range};if(static_state)hits[n-1].full_track=static_state->track[i]<FV5_TRACKS?static_state->track[i]:-1;
  }
  qsort(hits,n,sizeof(*hits),sf_order);unsigned start=0;for(unsigned i=1;i<=n;i++)if(i==n||((c->mode==1||c->mode==2)?!fv5_connected(hits[i-1].x,hits[i-1].y,hits[i-1].angle,hits[i-1].range,hits[i].x,hits[i].y,hits[i].angle,hits[i].range):(hypot(hits[i].x-hits[i-1].x,hits[i].y-hits[i-1].y)>fmin(.35,.18+.015*hits[i-1].range)||hits[i].angle-hits[i-1].angle>.06))){if(c->mode==1)sf_add_cluster_min(clusters,&nc,hits,start,i,camx,camy,1);else sf_add_cluster(clusters,&nc,hits,start,i,camx,camy);start=i;}
  free(static_state);
  int best=-1;for(unsigned i=0;i<nc;i++)if(best<0||clusters[i].camera_range<clusters[best].camera_range)best=(int)i;
  if(best<0)continue;const sf_cluster*q=&clusters[best];obj_scene_candidate*z=out+count++;memset(z,0,sizeof(*z));
  z->frame_id=f->frame_id;z->image_time_us=f->frame_us;z->source_us=z->scan_us=v->scan_time_us;z->track_id=d->track_id;z->class_id=d->class_id;z->eye=d->eye;memcpy(z->bbox,d->bbox,sizeof(z->bbox));z->bearing_rad=d->bearing_valid?d->bearing:b;z->range_m=q->camera_range;z->odom_x_m=q->x;z->odom_y_m=q->y;z->error_m=.08+.025*q->camera_range+.25*fmin(q->span,q->camera_range*half)+image.error+scan_pose.error+(c->mount_approximate?.10:0);
 }
 free(hits);return count;
}
