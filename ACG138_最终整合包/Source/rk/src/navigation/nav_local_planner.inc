/* 局部模块只提出候选及排序，不持有电机、规则、传感器或硬件。加减速度默认值仍待实测。存储由调用方持有，最多一个候选和40组目标；航向加速度解析积分，弧线XY用数值积分。 */
#ifndef ACG_S6_NAV_LOCAL_PLANNER_INC
#define ACG_S6_NAV_LOCAL_PLANNER_INC
#include "s6_planning_types.h"
#include "continuous_policy.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

#define S6_PI 3.14159265358979323846
#define S6_EPS 1e-9

static double s6_clamp(double x,double lo,double hi) {return x<lo?lo:(x>hi?hi:x);}
static void s6_reason(char *s,const char *why) {snprintf(s,64,"%s",why);}
static double s6_angle(double x) {return atan2(sin(x),cos(x));}
/* 制动上限与B阶段双侧连续约束取交集。允许上限默认0.45，实际构建的制动配置默认使用更保守0.40。

 */
static double s6_effective_brake(const s6_config *c) {return fmin(c->brake,c->a_lin);}

static void s6_config_default(s6_config *c) {
 if(!c)return;memset(c,0,sizeof(*c));
 c->cycle_s=.1;c->horizon_s=1.5;c->dt_s=.05;c->lethal_m=.18;c->inflation_m=.25;
 c->w_heading=.50;c->w_path=.30;c->w_clearance=.40;c->w_smoothness=.30;c->w_velocity=.15;
 c->a_lin=.40;c->a_yaw=2.5;c->brake=.45;c->terminal_margin=.01;
 c->wheel_radius=.03375;c->track=.29;c->max_rpm=120;
 c->body_length_m=.333;c->body_width_m=.320;c->body_buffer_m=.010;
 c->front_lethal_from_edge_m=.120;c->straight_w_threshold=.1;
 c->allow_window_boundary_samples=0; /* 默认采用固定28采样方案。 */
}

static int s6_config_valid(const s6_config *c) {
 if(!c)return 0;
 const double positive[]={c->cycle_s,c->horizon_s,c->dt_s,c->lethal_m,c->inflation_m,
  c->a_lin,c->a_yaw,c->brake,c->terminal_margin,c->wheel_radius,c->track,c->max_rpm};
 const double weights[]={c->w_heading,c->w_path,c->w_clearance,c->w_smoothness,c->w_velocity};
 for(size_t i=0;i<sizeof(positive)/sizeof(positive[0]);i++)if(!isfinite(positive[i])||positive[i]<=0)return 0;
 double sum=0;for(size_t i=0;i<5;i++){if(!isfinite(weights[i])||weights[i]<0)return 0;sum+=weights[i];}
 if(c->scene_geometry_enabled&&(!isfinite(c->body_length_m)||c->body_length_m<=0||c->body_length_m>1||
   !isfinite(c->body_width_m)||c->body_width_m<=0||c->body_width_m>1||
   !isfinite(c->body_buffer_m)||c->body_buffer_m<0||c->body_buffer_m>.1||
   !isfinite(c->front_lethal_from_edge_m)||c->front_lethal_from_edge_m<0||c->front_lethal_from_edge_m>.5||
   !isfinite(c->straight_w_threshold)||c->straight_w_threshold<0||c->straight_w_threshold>.1))return 0;
 return sum>0&&isfinite(sum)&&c->inflation_m>c->lethal_m&&c->cycle_s<=c->horizon_s&&
  c->dt_s<=c->cycle_s&&ceil(c->horizon_s/c->dt_s-1e-9)<=S6_TRAJ_MAX&&
  c->max_rpm<=120&&c->allow_window_boundary_samples<=1&&c->scene_geometry_enabled<=1&&c->scene_straight_mode<=1&&c->r4_speed_bands<=1;
}

static void s6_rpm(const s6_config *c,double v,double w,double *left,double *right) {
 double scale=60/(2*S6_PI*c->wheel_radius);
 *left=(v-w*c->track*.5)*scale;*right=(v+w*c->track*.5)*scale;
}
static int s6_rpm_ok(const s6_config *c,double v,double w) {
 double l,r;s6_rpm(c,v,w,&l,&r);return fabs(l)<=c->max_rpm+S6_EPS&&fabs(r)<=c->max_rpm+S6_EPS;
}

/* v和w沿同一凸线段变化，保持轮速菱形可行域，并分别满足轴加速度上限。
 */
static void s6_ramp(const s6_config *c,const s6_point *p,double tv,double tw,int brake,
                    double *av,double *aw,double *duration) {
 double dv=tv-p->v,dw=tw-p->w;
 *duration=fmax(fabs(dv)/(brake?s6_effective_brake(c):c->a_lin),fabs(dw)/c->a_yaw);
 *av=*duration>S6_EPS?dv/ *duration:0;*aw=*duration>S6_EPS?dw/ *duration:0;
}

/* 航向与线距离使用恒加速度精确式，曲线XY采用五点高斯积分，避免欧拉航向偏差。
 */
static void s6_integrate(s6_point *p,double av,double aw,double dt) {
 static const double node[]={-.9061798459386640,-.5384693101056831,0,.5384693101056831,.9061798459386640};
 static const double weight[]={.2369268850561891,.4786286704993665,.5688888888888889,.4786286704993665,.2369268850561891};
 double dx=0,dy=0;
 for(unsigned j=0;j<5;j++){double t=.5*dt*(1+node[j]),yaw=p->yaw+p->w*t+.5*aw*t*t;
  double v=p->v+av*t;dx+=weight[j]*v*cos(yaw);dy+=weight[j]*v*sin(yaw);}
 p->x+=.5*dt*dx;p->y+=.5*dt*dy;p->yaw+=p->w*dt+.5*aw*dt*dt;
 p->v+=av*dt;p->w+=aw*dt;p->t+=dt;
}

/* 在准确到达目标时刻分段，不按50ms量化渐变。 */
static void s6_advance(const s6_config *c,s6_point *p,double tv,double tw,int brake,double dt) {
 double av,aw,duration;s6_ramp(c,p,tv,tw,brake,&av,&aw,&duration);
 double moving=fmin(dt,duration);if(moving>S6_EPS)s6_integrate(p,av,aw,moving);
 if(duration<=dt+S6_EPS){p->v=tv;p->w=tw;double rest=dt-moving;if(rest>0)s6_integrate(p,0,0,rest);}
}

enum s6_probe_result {S6_PROBE_OK=0,S6_PROBE_LETHAL=1,S6_PROBE_UNKNOWN=2,S6_PROBE_RPM=3};
#include "nav_scene_geometry.inc"
static int s6_probe(const cm_state *cm,const s6_config *c,const s6_point *p,double *minimum) {
 if(!s6_rpm_ok(c,p->v,p->w))return S6_PROBE_RPM;
 if(c->scene_geometry_enabled)return s6_scene_geometry_probe(cm,c,p,minimum,0);
 uint8_t cost=cm_cost_at(cm,p->x,p->y);
 if(cost==255)return S6_PROBE_UNKNOWN; /* 未知区域既不直接算自由，也不直接算致命障碍。 */
 double clear=cm_clearance_at(cm,p->x,p->y);
 if(isnan(clear)||clear<0)return S6_PROBE_UNKNOWN;
 if(clear<*minimum)*minimum=clear;
 return cost==254||clear<=c->lethal_m+S6_EPS?S6_PROBE_LETHAL:S6_PROBE_OK;
}

/* brake_at小于0表示巡航。内部探测间隔不超过5ms，外部只暴露最多30个、间隔50ms的未来样本；额外探测有界，不增加规划候选数量。

 */
static int s6_simulate(const cm_state *cm,const s6_config *c,const s6_plan_input *in,
                       double tv,double tw,double brake_at,s6_trajectory *out) {
 /* 整条候选按目标及起始角速度分级，不能让 .3 候选的起步瞬间借用直行宽限。 */
 s6_config geometry=*c;
 if(fabs(tw)>c->straight_w_threshold+S6_EPS||fabs(in->current_w)>c->straight_w_threshold+S6_EPS)geometry.scene_straight_mode=0;
 c=&geometry;
 memset(out,0,sizeof(*out));out->target_v=tv;out->target_w=tw;out->clear_min=INFINITY;
 s6_point p={in->pose.x,in->pose.y,in->pose.yaw,in->current_v,in->current_w,0};out->initial=p;
 int rc=s6_probe(cm,c,&p,&out->clear_min);if(rc)return rc;
 unsigned count=(unsigned)ceil(c->horizon_s/c->dt_s-1e-9);
 for(unsigned i=0;i<count;i++){
  double until=fmin(c->horizon_s,(i+1)*c->dt_s);
  while(p.t<until-S6_EPS){
   int braking=brake_at>=0&&p.t>=brake_at-S6_EPS;
   double dt=fmin(.005,until-p.t);
   if(brake_at>=0&&!braking)dt=fmin(dt,brake_at-p.t);
   s6_advance(c,&p,braking?0:tv,braking?0:tw,braking,dt);
   rc=s6_probe(cm,c,&p,&out->clear_min);if(rc)return rc;
  }
  out->points[out->count++]=p;
 }
 out->terminal_stopped=brake_at>=0&&fabs(p.v)<1e-8&&fabs(p.w)<1e-8;
 if(brake_at>=0){
  if(!out->terminal_stopped)return S6_PROBE_LETHAL;
  if(c->scene_geometry_enabled){double minimum=INFINITY;rc=s6_scene_geometry_probe(cm,c,&p,&minimum,c->terminal_margin);if(rc)return rc;}
  else if(cm_clearance_at(cm,p.x,p.y)<c->lethal_m+c->terminal_margin-S6_EPS)return S6_PROBE_LETHAL;
 }
 return S6_PROBE_OK;
}

/* 巡航末端仍有速度时，后续延伸及显式停车尾段需均已观测且畅通；尾段失败则在1.5秒输出内构造制动，而非事后删除。

 */
static int s6_tail_probe(const cm_state *cm,const s6_config *c,const s6_trajectory *tr) {
 s6_config geometry=*c;
 if(fabs(tr->target_w)>c->straight_w_threshold+S6_EPS||fabs(tr->initial.w)>c->straight_w_threshold+S6_EPS)geometry.scene_straight_mode=0;
 c=&geometry;
 s6_point p=tr->points[tr->count-1];double minimum=INFINITY;
 double duration=fmax(fabs(p.v)/s6_effective_brake(c),fabs(p.w)/c->a_yaw);
 for(unsigned i=0;i<400&&duration>S6_EPS;i++){
  double dt=fmin(.005,duration);s6_advance(c,&p,0,0,1,dt);duration-=dt;
  int rc=s6_probe(cm,c,&p,&minimum);if(rc)return rc;
 }
 if(duration>S6_EPS)return S6_PROBE_LETHAL;
 if(c->scene_geometry_enabled)return s6_scene_geometry_probe(cm,c,&p,&minimum,c->terminal_margin);
 return cm_clearance_at(cm,p.x,p.y)<c->lethal_m+c->terminal_margin-S6_EPS?S6_PROBE_LETHAL:S6_PROBE_OK;
}

static double s6_path_distance(const s6_plan_input *in,const s6_point *p) {
 if(!in->path||!in->path_count){
  return fabs(-(p->x-in->pose.x)*sin(in->goal_heading)+(p->y-in->pose.y)*cos(in->goal_heading));
 }
 if(in->path_count==1)return hypot(p->x-in->path[0].x,p->y-in->path[0].y);
 double best=INFINITY;
 for(uint32_t j=1;j<in->path_count;j++){
  double ax=in->path[j-1].x,ay=in->path[j-1].y,dx=in->path[j].x-ax,dy=in->path[j].y-ay;
  double len=dx*dx+dy*dy,u=len>0?s6_clamp(((p->x-ax)*dx+(p->y-ay)*dy)/len,0,1):0;
  double d=hypot(p->x-(ax+u*dx),p->y-(ay+u*dy));if(d<best)best=d;
 }return best;
}

static void s6_score(const cm_state *cm,const s6_config *c,const s6_plan_input *in,s6_trajectory *tr) {
 s6_point *end=&tr->points[tr->count-1];double lateral=s6_path_distance(in,&tr->initial);
 for(uint32_t i=0;i<tr->count;i++){double d=s6_path_distance(in,&tr->points[i]);if(d>lateral)lateral=d;}
 /* cm_min_clearance_along与逐步探测来自同一数据源，保留包括t=0的更保守内部探测，未知已在前面拒绝。
 */
 if(!c->scene_geometry_enabled){double grid_min=cm_min_clearance_along(cm,tr->points,tr->count);if(grid_min<tr->clear_min)tr->clear_min=grid_min;}
 tr->heading_score=s6_clamp(1-fabs(s6_angle(end->yaw-in->goal_heading))/S6_PI,0,1);
 tr->path_score=s6_clamp(1-lateral/.5,0,1);
 tr->clearance_score=s6_clamp(1-(tr->clear_min-c->lethal_m)/(c->inflation_m-c->lethal_m),0,1);
 tr->smoothness_score=s6_clamp((fabs(tr->target_v-in->current_v)/(c->a_lin*c->cycle_s)+
   fabs(tr->target_w-in->current_w)/(c->a_yaw*c->cycle_s))*.5,0,1);
 tr->velocity_score=s6_clamp(tr->target_v/(c->r4_speed_bands?.40:.25),0,1);
 tr->score=c->w_heading*tr->heading_score+c->w_path*tr->path_score-c->w_clearance*tr->clearance_score-
  c->w_smoothness*tr->smoothness_score+c->w_velocity*tr->velocity_score;
}

static int s6_make_trajectory(const cm_state *cm,const s6_config *c,const s6_plan_input *in,
                             double v,double w,s6_trajectory *tr) {
 int rc=s6_simulate(cm,c,in,v,w,-1,tr);
 if(rc==S6_PROBE_UNKNOWN||rc==S6_PROBE_RPM)return rc;
 if(rc==S6_PROBE_OK){rc=s6_tail_probe(cm,c,tr);if(rc==S6_PROBE_OK){s6_reason(tr->reason,"CRUISE_CLEAR");return rc;}}
 if(rc==S6_PROBE_UNKNOWN||rc==S6_PROBE_RPM)return rc;
 /* 尽可能晚但仍可行的制动起点保留前进进展，每次重试均从实测v和w构建受加速度限制的停车轨迹。
 */
 unsigned steps=(unsigned)ceil(c->horizon_s/c->dt_s-1e-9);
 for(int k=(int)steps-1;k>=0;k--){
  rc=s6_simulate(cm,c,in,v,w,k*c->dt_s,tr);
  if(rc==S6_PROBE_OK){snprintf(tr->reason,sizeof(tr->reason),"TERMINAL_BRAKE effective_brake=%.6g",s6_effective_brake(c));return rc;}
 }
 return S6_PROBE_LETHAL;
}

static void s6_plan_compute(const cm_state *cm,const s6_config *c,const s6_plan_input *in,s6_plan *out) {
 if(!out)return;memset(out,0,sizeof(*out));
 if(!in||!s6_config_valid(c)){s6_reason(out->reason,"INVALID_INPUT_CONFIG");return;}out->generated_us=in->now_us;
 if(!cm||!cm->initialized||cm_age_ms(cm,in->now_us)>CONT_SCAN_US/1000||in->now_us<cm->source_us||in->now_us-cm->source_us>CONT_SCAN_US){s6_reason(out->reason,"COSTMAP_STALE");return;}
 if(!isfinite(in->pose.x)||!isfinite(in->pose.y)||!isfinite(in->pose.yaw)||!isfinite(in->current_v)||
  !isfinite(in->current_w)||!isfinite(in->goal_heading)||!isfinite(in->front_clearance_m)||in->front_clearance_m<0||
  in->current_v<0||in->path_count>4096||(!in->path&&in->path_count)){s6_reason(out->reason,"INVALID_INPUT_CONFIG");return;}
 for(uint32_t j=0;j<in->path_count;j++)if(!isfinite(in->path[j].x)||!isfinite(in->path[j].y)){s6_reason(out->reason,"INVALID_PATH");return;}
 if(in->front_clearance_m<.1){s6_reason(out->reason,"NEAR_AVOID_PRIORITY");return;}
 if(!s6_rpm_ok(c,in->current_v,in->current_w)){s6_reason(out->reason,"CURRENT_RPM_LIMIT");return;}
 double vmax=c->r4_speed_bands?
  (in->front_clearance_m<=.15?0:in->front_clearance_m<=.3?.16:in->front_clearance_m<=.6?.25:in->front_clearance_m<=1?.35:.40):
  (in->front_clearance_m<=.15?0:(in->front_clearance_m<=.3?.12:(in->front_clearance_m<=.6?.20:.25)));
 if(in->current_v>vmax+S6_EPS){s6_reason(out->reason,"CURRENT_S5_ENVELOPE");return;}
 double vs[S6_SAMPLES_MAX],ws[S6_SAMPLES_MAX];unsigned n=0;
 static const double linear[]={.06,.12,.20,.25,.35,.40},yaw[]={0,.3,-.3,1,-1,1.9,-1.9};
 for(unsigned i=0;i<(c->r4_speed_bands?6u:4u);i++)for(unsigned j=0;j<7;j++){vs[n]=linear[i];ws[n++]=yaw[j];}
 if(c->allow_window_boundary_samples){
  for(int i=-1;i<=1;i++)for(int j=-1;j<=1;j++){
   double v=s6_clamp(in->current_v+i*c->a_lin*c->cycle_s,0,c->r4_speed_bands?.40:.25);
   double w=s6_clamp(in->current_w+j*c->a_yaw*c->cycle_s,-1.9,1.9);int duplicate=0;
   for(unsigned k=0;k<n;k++)if(fabs(v-vs[k])<S6_EPS&&fabs(w-ws[k])<S6_EPS)duplicate=1;
   if(!duplicate&&n<S6_SAMPLES_MAX){vs[n]=v;ws[n++]=w;}
  }
 }
 unsigned unknown=0;
 for(unsigned j=0;j<n;j++){
  double v=vs[j],w=ws[j];out->sampled++;
  if(fabs(v-in->current_v)>c->a_lin*c->cycle_s+S6_EPS||fabs(w-in->current_w)>c->a_yaw*c->cycle_s+S6_EPS){out->dynamic_rejected++;continue;}
  if(!s6_rpm_ok(c,v,w)){out->rpm_rejected++;continue;}
  if(v>vmax+S6_EPS){out->envelope_rejected++;continue;}
  s6_trajectory tr;int rc=s6_make_trajectory(cm,c,in,v,w,&tr);
  if(rc==S6_PROBE_UNKNOWN){unknown++;continue;}
  if(rc==S6_PROBE_RPM){out->rpm_rejected++;continue;}
  if(rc){out->lethal_rejected++;continue;}
  s6_score(cm,c,in,&tr);
  unsigned pos=out->count;while(pos&&out->ranked[pos-1].score<tr.score){out->ranked[pos]=out->ranked[pos-1];pos--;}
  out->ranked[pos]=tr;out->count++;
 }
 if(out->count)s6_reason(out->reason,"RANKED");
 else if(out->dynamic_rejected==out->sampled)s6_reason(out->reason,"DYNAMIC_WINDOW_NO_SAMPLE");
 else if(unknown)s6_reason(out->reason,"UNKNOWN_COSTMAP");
 else if(out->envelope_rejected)s6_reason(out->reason,"S5_ENVELOPE_NO_SAMPLE");
 else if(out->lethal_rejected)s6_reason(out->reason,"LETHAL_OR_STOP_INFEASIBLE");
 else s6_reason(out->reason,"RPM_NO_SAMPLE");
}

/* 线性插值包含实测t=0，凸插值保持已核对的各轴加速度和轮速边界。
 */
static int s6_profile_at(const s6_trajectory *tr,double t,double *v,double *w) {
 if(!tr||!v||!w||!tr->count||tr->count>S6_TRAJ_MAX||!isfinite(t))return 0;
 const s6_point *a=&tr->initial,*b=a;
 if(t<=0){*v=a->v;*w=a->w;return 1;}
 for(uint32_t j=0;j<tr->count;j++){b=&tr->points[j];if(t<=b->t){
   double fraction=b->t>a->t?s6_clamp((t-a->t)/(b->t-a->t),0,1):1;
   *v=a->v+(b->v-a->v)*fraction;*w=a->w+(b->w-a->w)*fraction;return 1;}a=b;}
 *v=b->v;*w=b->w;return 1;
}
#endif
