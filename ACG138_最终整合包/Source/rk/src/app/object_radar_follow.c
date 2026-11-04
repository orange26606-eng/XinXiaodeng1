/* 雷达找回状态归当前任务所有，原始全周障碍及静态分类独立处理。 */

#include "object_radar_follow.h"
#include "follow_association_v5.h"
#include <stdio.h>
#define RF_PI 3.14159265358979323846
#define RF_MAGIC UINT32_C(0x52463831)
#define RF_POSES 16u
#define RF_SCAN_AGE_US UINT64_C(500000)
#define RF_POSE_MATCH_US UINT64_C(100000)
#define RF_LOST_US UINT64_C(1200000)
#define RF_HINT_US UINT64_C(10000000)
typedef struct rf_pose {uint64_t us;double x,y,yaw,error,angular_error;uint64_t clock_error_us;} rf_pose;
typedef struct rf_state {
 uint32_t magic,binding,session,sequence,revision;
 lab_real_context context;uint64_t epoch,last_scan,last_selected;
 int canonical,chosen,initialized,independent;unsigned poses;
 rf_pose pose[RF_POSES];
 double selected_x,selected_y,selected_vx,selected_vy,pose_error;
 double scan_error[3];uint64_t scan_clock_error[3];unsigned scan_errors;
 fv5_state classifier;
} rf_state;
_Static_assert(sizeof(rf_state)<=OBJ_RADAR_FOLLOW_BYTES,"radar private context exceeds bound");
static uint64_t rf_delta(uint64_t a,uint64_t b){return a>b?a-b:b-a;}
static int rf_recent(uint64_t t,uint64_t now,uint64_t age){return t&&t<=now&&now-t<=age;}
static int rf_context(const lab_real_context*c){return c->source>=LAB_REAL_HARDWARE&&c->source<=LAB_REAL_TEST_FIXTURE&&(c->clock==LAB_REAL_MONOTONIC||c->clock==LAB_REAL_REPLAY_CLOCK)&&c->timeline_id&&c->origin_epoch;}
void obj_radar_follow_reset(obj_radar_follow*c){if(c)memset(c,0,sizeof(*c));}
static void rf_initialize(rf_state*s,const lab_real_runtime_view*v,uint64_t epoch,int canonical,uint32_t binding,uint32_t session){
 memset(s,0,sizeof(*s));s->magic=RF_MAGIC;s->context=v->context;s->epoch=epoch;s->canonical=canonical;s->binding=binding;s->session=session;s->chosen=s->classifier.chosen=-1;s->initialized=1;
}
static int rf_pose_collect(rf_state*s,const lab_real_runtime_view*v,uint64_t now){
 const lab_pose*p=&v->raw_odometry;
 if(!v->has_odometry||!rf_recent(v->odometry_time_us,now,RF_SCAN_AGE_US)||!(p->quality&LAB_POSE_VALID)||(p->quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))||!isfinite(p->x_m)||!isfinite(p->y_m)||!isfinite(p->yaw_rad)||!isfinite(p->uncertainty_m)||p->uncertainty_m<0||p->uncertainty_m>100)return 0;
 if(s->poses&&v->odometry_time_us<s->pose[s->poses-1].us)return 0;
 if(s->poses&&v->odometry_time_us==s->pose[s->poses-1].us){const rf_pose*q=&s->pose[s->poses-1];return hypot(q->x-p->x_m,q->y-p->y_m)<1e-9&&fabs(remainder(q->yaw-p->yaw_rad,2*RF_PI))<1e-9;}
 if(s->poses==RF_POSES){memmove(s->pose,s->pose+1,(RF_POSES-1)*sizeof(*s->pose));s->poses--;}
 s->pose[s->poses++]=(rf_pose){v->odometry_time_us,p->x_m,p->y_m,p->yaw_rad,p->uncertainty_m,0};return 1;
}
/* 在连续ODOM位姿中插值。缺少包围区间时允许至多100ms近似，并按0.4m/s、0.7rad/s计入误差。 */


static int rf_scan_pose(const rf_state*s,uint64_t stamp,rf_pose*out){
 for(unsigned i=0;i<s->poses;i++)if(s->pose[i].us==stamp){*out=s->pose[i];return 1;}
 for(unsigned i=1;i<s->poses;i++){const rf_pose*a=s->pose+i-1,*b=s->pose+i;if(a->us<stamp&&b->us>stamp&&b->us-a->us<=250000){double t=(double)(stamp-a->us)/(b->us-a->us);*out=(rf_pose){stamp,a->x+t*(b->x-a->x),a->y+t*(b->y-a->y),a->yaw+t*remainder(b->yaw-a->yaw,2*RF_PI),fmax(a->error,b->error)+.005,0};return 1;}}
 if(!s->poses)return 0;const rf_pose*p=&s->pose[s->poses-1];uint64_t d=rf_delta(p->us,stamp);if(d>RF_POSE_MATCH_US)return 0;*out=*p;out->error+=.4*d/1e6;out->angular_error=.7*d/1e6;return 1;
}
/* 移动目标建立需3次独立扫描、至少200ms、净位移至少5cm且超过量化与位姿误差，两段方向一致。静态抖动、孤立跳变、重复读取及自车运动不算目标出现。扫描间隔50至350ms，证据窗口不超过800ms，速度不超过2m/s。


 */
static int rf_moving(const fv5_track*t,double pose_error){
 if(!t->used||!t->updated||t->history_count<3||t->span>.65||t->isolation<.45)return 0;
 uint64_t a=t->history_time[0],b=t->history_time[1],c=t->history_time[2];
 if(b<=a||c<=b||b-a<50000||b-a>350000||c-b<50000||c-b>350000||c-a<200000||c-a>800000)return 0;
 double ax=t->history_x[1]-t->history_x[0],ay=t->history_y[1]-t->history_y[0],bx=t->history_x[2]-t->history_x[1],by=t->history_y[2]-t->history_y[1];
 double da=hypot(ax,ay),db=hypot(bx,by),net=hypot(ax+bx,ay+by);
 return net>=.05+2*pose_error&&da>=.012&&db>=.012&&ax*bx+ay*by>0&&net>=.90*(da+db)&&da<=2*(b-a)/1e6&&db<=2*(c-b)/1e6;
}
typedef struct rf_guide {int position,bearing,immediate;uint64_t source_us;double x,y,gate,yaw,half;} rf_guide;
/* 在目标历史位置或已知方向附近搜索；历史提示保留时间与范围限制。 */


static rf_guide rf_guidance(const rf_state*r,const obj_radar_follow_hint*h,uint64_t now){
 rf_guide g={0};if(r->independent)return g;
 int prior=h&&h->position_valid&&rf_recent(h->position_source_us,now,RF_HINT_US)&&isfinite(h->odom_x)&&isfinite(h->odom_y)&&isfinite(h->error)&&h->error>=0&&h->error<=100;
 double prior_error=prior?h->error:0,common=r->poses?r->pose[r->poses-1].error:0;
 /* 任务位置和扫描共用连续ODOM，只有输入误差已包含公共累计估计时才分离残差用于关联，完整当前误差仍保留在measurement.error。

 */
 if(common>.25&&prior_error>=common)prior_error-=common;
 if(r->last_selected&&rf_recent(r->last_selected,now,RF_HINT_US)&&(!prior||r->last_selected>=h->position_source_us)){
  double age=(now-r->last_selected)/1e6;g.position=1;g.source_us=r->last_selected;g.immediate=age<=2.;g.x=r->selected_x+r->selected_vx*fmin(age,2.);g.y=r->selected_y+r->selected_vy*fmin(age,2.);g.gate=fmin(.90,.15+1.6*age)+fmin(.15,r->pose_error);
 }else if(prior){double age=(now-h->position_source_us)/1e6;g.position=1;g.source_us=h->position_source_us;g.immediate=age<=2.&&prior_error<=.35;g.x=h->odom_x;g.y=h->odom_y;g.gate=fmin(.90,.20+.12*age+fmin(.30,prior_error));}
 if(!g.position&&h&&h->bearing_valid&&rf_recent(h->bearing_source_us,now,RF_HINT_US)&&isfinite(h->world_yaw)){double age=(now-h->bearing_source_us)/1e6;g.bearing=1;g.yaw=remainder(h->world_yaw,2*RF_PI);g.half=fmin(.65,.22+.06*age);}
 return g;
}
static int rf_supported(const fv5_track*t){
 if(t->history_count<3)return 0;uint64_t a=t->history_time[0],b=t->history_time[1],c=t->history_time[2];return b>a&&c>b&&b-a>=50000&&b-a<=350000&&c-b>=50000&&c-b<=350000&&c-a>=200000&&c-a<=800000;
}
static int rf_guided(const rf_guide*g,const rf_pose*p,const fv5_track*t){
 if(g->position)return hypot(t->x-g->x,t->y-g->y)<=g->gate;
 if(g->bearing)return fabs(remainder(atan2(t->y-p->y,t->x-p->x)-g->yaw,2*RF_PI))<=g->half;
 return 0;
}
static void rf_historical(obj_scene*s){
 if(s->measurement_v2.kind==OBJECT_MEASURE_RADAR_ONLY)memset(&s->measurement_v2,0,sizeof(s->measurement_v2));
 if(!strcmp(s->source,"RADAR_ONLY")){s->historical=1;s->confirmed=0;snprintf(s->reason,sizeof(s->reason),"RADAR_HISTORY_NO_CURRENT_RETURN");snprintf(s->message_zh,sizeof(s->message_zh),"雷达搜寻；保留上次真实目标位置，当前没有新回波");}
}
static int rf_wait(obj_scene*s,const char*reason,const char*zh){
 snprintf(s->follow_reason,sizeof(s->follow_reason),"%s",reason);
 if(!s->measurement_v2.valid){snprintf(s->reason,sizeof(s->reason),"%s",reason);snprintf(s->message_zh,sizeof(s->message_zh),"%s",zh);}return 0;
}
int obj_radar_follow_update(obj_radar_follow*context,obj_scene*s,const obj_scene_config*c,const lab_real_runtime_view*v,const passage_config*p,uint64_t now,uint64_t epoch,int canonical,uint32_t binding,const obj_radar_follow_hint*hint){
 if(!context||!s||!c||!v)return 0;rf_historical(s);
 const struct scan_frame*scan=v->latest_scan;
 if(c->mode!=2||!epoch||canonical<=0||!rf_context(&v->context)||!isfinite(c->range_max_m)||c->range_max_m<=0||!isfinite(c->standoff_m)||c->standoff_m<.05||c->standoff_m>3)return rf_wait(s,"RADAR_WAIT_CONTEXT","雷达搜寻：等待有效任务与传感器上下文");
 rf_state*r=(rf_state*)(void*)context->bytes;
 uint32_t session=scan?scan->session:0;
 int independent=!hint||hint->independent;
 if(r->magic!=RF_MAGIC||!r->initialized||memcmp(&r->context,&v->context,sizeof(v->context))||r->epoch!=epoch||r->canonical!=canonical||r->binding!=binding||r->independent!=independent||(session&&r->session&&session!=r->session)){rf_initialize(r,v,epoch,canonical,binding,session);r->independent=independent;}
 if(s->context_valid&&!memcmp(&s->context,&v->context,sizeof(v->context))&&s->epoch==epoch&&s->track_id==canonical&&s->binding_generation==binding&&s->association_revision>r->revision)r->revision=s->association_revision;
 if(!rf_pose_collect(r,v,now))return rf_wait(s,"RADAR_WAIT_ODOM","雷达搜寻：等待连续、有效的真实里程计");
 if(!v->has_scan||!scan||!scan_geometry_valid(scan)||scan->count>FV5_BEAMS||!rf_recent(v->scan_time_us,now,RF_SCAN_AGE_US))return rf_wait(s,"RADAR_WAIT_SCAN","雷达搜寻：等待时效内的真实扫描");
 if(v->scan_time_us<=r->last_scan||(r->last_scan&&scan->sequence<=r->sequence))return rf_wait(s,"RADAR_WAIT_NEW_SCAN","雷达搜寻：当前扫描已处理，等待新扫描");
 double minimum=p?p->lidar_min_m:.05,maximum=fmin(12.,fmin(c->range_max_m,p?p->lidar_max_m:12.));
 double mx=p?p->mount_x_m:0,my=p?p->mount_y_m:0,ma=p?p->mount_yaw_rad:0;
 if(!isfinite(minimum)||minimum<=0||maximum<minimum||!isfinite(mx)||!isfinite(my)||!isfinite(ma))return rf_wait(s,"RADAR_CONFIG_INVALID","雷达搜寻：扫描范围或安装参数无效");
 rf_pose pose;
 if(hint&&hint->scan_pose_valid){
  if(hint->scan_pose_time_us!=v->scan_time_us||!isfinite(hint->scan_pose_x)||!isfinite(hint->scan_pose_y)||!isfinite(hint->scan_pose_yaw)||!isfinite(hint->scan_pose_error_m)||hint->scan_pose_error_m<0||hint->scan_pose_error_m>100||hint->scan_pose_error_us>100000)return rf_wait(s,"RADAR_WAIT_SCAN_POSE","雷达搜寻：扫描时刻位姿来源或误差无效");
  pose=(rf_pose){hint->scan_pose_time_us,hint->scan_pose_x,hint->scan_pose_y,hint->scan_pose_yaw,hint->scan_pose_error_m,0,hint->scan_pose_error_us};
 }else if(!rf_scan_pose(r,v->scan_time_us,&pose))return rf_wait(s,"RADAR_WAIT_SCAN_POSE","雷达搜寻：等待与扫描时间匹配的真实位姿");
 fv5_point*points=malloc(FV5_BEAMS*sizeof(*points));if(!points)return rf_wait(s,"RADAR_RESOURCE_LIMIT","雷达搜寻：当前扫描缓存不可用");
 unsigned n=0;double co=cos(pose.yaw),sn=sin(pose.yaw);
 for(unsigned i=0;i<scan->count;i++){if(!scan_return_in_range(scan,i,minimum,maximum))continue;double a=-scan->points[i].angle_q14*(RF_PI/32768.)+ma,range=scan->points[i].distance_q2/4000.,bx=mx+range*cos(a),by=my+range*sin(a);points[n++]=(fv5_point){pose.x+co*bx-sn*by,pose.y+sn*bx+co*by,remainder(a,2*RF_PI),scan_acquisition_range_m(scan,i),i};}
 rf_guide guide=rf_guidance(r,hint,now);
 r->classifier.priority_us=guide.position?guide.source_us:0;r->classifier.priority_x=guide.x;r->classifier.priority_y=guide.y;r->classifier.priority_gate=guide.gate;
 r->classifier.chosen=r->chosen;int prepared=fv5_prepare(&r->classifier,points,n,v->scan_time_us,scan->sequence);free(points);if(!prepared)return rf_wait(s,"RADAR_SCAN_REJECTED","雷达搜寻：当前扫描未通过有效性检查");
 r->last_scan=v->scan_time_us;r->sequence=scan->sequence;r->session=session;r->pose_error=pose.error;
 /* 短扫描窗口共享 ODOM 原点。输出保留累计不确定度；相对运动误差按不确定度增量和时钟区间变化估计，该误差预算尚未标定。 */
 if(r->scan_errors==3){for(unsigned i=0;i<2;i++){r->scan_error[i]=r->scan_error[i+1];r->scan_clock_error[i]=r->scan_clock_error[i+1];}r->scan_errors--;}
 unsigned ei=r->scan_errors++;r->scan_error[ei]=pose.error;r->scan_clock_error[ei]=pose.clock_error_us;
 double relative_error=fmax(0,pose.error-r->scan_error[0]);uint64_t clock_growth=pose.clock_error_us>r->scan_clock_error[0]?pose.clock_error_us-r->scan_clock_error[0]:0;
 int chosen=r->chosen;fv5_track*t=chosen>=0?r->classifier.tracks+chosen:NULL;
 if(t){double dt=(v->scan_time_us-r->last_selected)/1e6,gate=fmin(.90,.15+1.6*dt);double px=r->selected_x+r->selected_vx*fmin(dt,.5),py=r->selected_y+r->selected_vy*fmin(dt,.5);
  if(!t->used||!t->updated||t->last!=v->scan_time_us||t->span>.65||t->isolation<.35||fv5_wall_at(&r->classifier,t->x,t->y)>=.75||hypot(t->x-px,t->y-py)>gate||(!independent&&!rf_guided(&guide,&pose,t)))t=NULL;
 }
 /* 短暂遮挡期间保留所选目标身份。 */
 if(!t&&r->chosen>=0&&v->scan_time_us-r->last_selected<=RF_LOST_US)return rf_wait(s,"RADAR_WAIT_SELECTED_RETURN","雷达搜寻：原目标短暂遮挡，保留原始位置与时间");
 if(!t){int best=-1;double rank=1e9,second=1e9;for(unsigned i=0;i<FV5_TRACKS;i++){fv5_track*q=&r->classifier.tracks[i];double distance=hypot(q->x-pose.x,q->y-pose.y);int moving=rf_moving(q,relative_error+pose.angular_error*distance+(.4+.7*distance)*clock_growth/1e6);if(!q->used||!q->updated||q->span>.65||q->isolation<.45||q->last!=v->scan_time_us||fv5_wall_at(&r->classifier,q->x,q->y)>=.70)continue;
   if(independent?!moving:(!rf_guided(&guide,&pose,q)||(!guide.immediate&&!rf_supported(q))||(!guide.position&&!moving)))continue;
   double dx=q->x-pose.x,dy=q->y-pose.y,angle=fabs(remainder(atan2(dy,dx)-pose.yaw,2*RF_PI)),score;
   if(independent)score=distance+.4*angle+(angle>2.36?1.:0);
   else if(guide.position)score=4*hypot(q->x-guide.x,q->y-guide.y)/fmax(.10,guide.gate)-.20*moving;
   else score=4*fabs(remainder(atan2(dy,dx)-guide.yaw,2*RF_PI))/guide.half+.025*distance;
   if(score<rank){second=rank;best=(int)i;rank=score;}else if(score<second)second=score;
  }
  if(!independent&&best>=0&&second-rank<.35)return rf_wait(s,"RADAR_SELECTED_AMBIGUOUS","雷达搜寻：原目标邻域存在相似回波，等待对应目标证据");
  if(best<0){if(independent)r->chosen=r->classifier.chosen=-1;if(!independent&&!guide.position&&!guide.bearing)return rf_wait(s,"RADAR_WAIT_SELECTED_HINT","雷达搜寻：等待原视觉目标的真实位置或世界方向");return rf_wait(s,!independent?"RADAR_SEARCH_SELECTED_NEIGHBORHOOD":n?"RADAR_WAIT_MOTION_SUPPORT":"RADAR_WAIT_VALID_RETURN",!independent?"雷达搜寻：在原目标位置或方向附近寻找对应点簇":n?"雷达搜寻：等待三次独立扫描确认移动目标":"雷达搜寻：当前范围内没有有效目标回波");}if(r->last_selected&&r->revision<UINT32_MAX)r->revision++;chosen=best;t=r->classifier.tracks+best;r->chosen=r->classifier.chosen=best;
 }
 int stopped=t->hits>=3&&hypot(t->vx,t->vy)<.06;r->last_selected=t->last;r->selected_x=t->x;r->selected_y=t->y;r->selected_vx=t->vx;r->selected_vy=t->vy;
 object_target_observation_v2*m=&s->measurement_v2;memset(m,0,sizeof(*m));m->valid=1;m->kind=OBJECT_MEASURE_RADAR_ONLY;m->surface_valid=1;m->mount_verified=p&&p->mount_verified;m->context=v->context;m->epoch=epoch;m->track_id=canonical;m->binding_generation=binding;m->association_revision=r->revision;m->source_us=m->scan_us=t->last;
 m->odom_x_m=t->x;m->odom_y_m=t->y;double dx=t->x-v->raw_odometry.x_m,dy=t->y-v->raw_odometry.y_m,cc=cos(v->raw_odometry.yaw_rad),ss=sin(v->raw_odometry.yaw_rad);m->body_x_m=cc*dx+ss*dy;m->body_y_m=-ss*dx+cc*dy;m->bearing_rad=atan2(m->body_y_m,m->body_x_m);m->surface_gap_m=fmax(0,hypot(m->body_x_m,m->body_y_m)-.5*(p&&p->length_m>0?p->length_m:.333));m->error_m=.05+.003*hypot(dx,dy)+pose.error+pose.angular_error*hypot(t->x-pose.x,t->y-pose.y);m->support_points=t->points;m->sparse=t->points<2;m->confirmed=!m->sparse&&t->hits>=3;
 snprintf(m->source,sizeof(m->source),"RADAR_ONLY");snprintf(m->reason,sizeof(m->reason),"%s",stopped?"RADAR_STOPPED_CONTINUITY":independent?"RADAR_MOVING_TRACK":guide.position?(t->hits<3?"RADAR_SELECTED_CURRENT_RETURN":"RADAR_SELECTED_POSITION_TRACK"):"RADAR_SELECTED_DIRECTION_ACQUIRED");
 s->context=v->context;s->context_valid=1;s->epoch=epoch;s->track_id=canonical;s->binding_generation=binding;s->association_revision=r->revision;s->source_us=s->scan_us=t->last;s->published_us=now;s->frame_id=0;s->historical=0;s->confirmed=m->confirmed;s->target_odom_valid=1;s->target_odom_x=t->x;s->target_odom_y=t->y;s->error_m=m->error_m;s->range_m=hypot(m->body_x_m,m->body_y_m);s->bearing_valid=1;s->bearing_rad=m->bearing_rad;s->standoff_m=c->standoff_m;s->approximate=!m->mount_verified;s->target_valid=s->dock_valid=0;
 /* 地图显示为可选项，需使用实际匹配上下文的变换。 */
 const lab_localization_view*l=&v->localization;
 if(v->has_localization&&l->has_pose&&l->map.map_id&&!memcmp(&l->context,&v->context,sizeof(v->context))&&!memcmp(&l->map.context,&v->context,sizeof(v->context))&&rf_recent(l->pose_time_us,now,RF_SCAN_AGE_US)&&isfinite(l->map_odom_x_m)&&isfinite(l->map_odom_y_m)&&isfinite(l->map_odom_yaw_rad)){double cm=cos(l->map_odom_yaw_rad),sm=sin(l->map_odom_yaw_rad);s->target_x=l->map_odom_x_m+cm*t->x-sm*t->y;s->target_y=l->map_odom_y_m+sm*t->x+cm*t->y;s->target_valid=1;s->map_id=l->map.map_id;s->map_version=l->map.map_version;}
 s->lidar_valid=s->lidar_support_valid=1;s->lidar_source_us=t->last;s->lidar_x=t->x;s->lidar_y=t->y;s->lidar_vx=t->vx;s->lidar_vy=t->vy;s->lidar_error_m=m->error_m;s->follow_sparse=m->sparse;s->follow_support_points=t->points;s->follow_motion_support=t->hits;s->follow_motion_score=t->motion;s->follow_isolation_score=t->isolation;s->follow_wall_score=fv5_wall_at(&r->classifier,t->x,t->y);s->follow_candidates=0;for(unsigned i=0;i<FV5_TRACKS;i++)s->follow_candidates+=r->classifier.tracks[i].updated;
 snprintf(s->source,sizeof(s->source),"RADAR_ONLY");snprintf(s->reason,sizeof(s->reason),"%s",m->reason);snprintf(s->follow_reason,sizeof(s->follow_reason),"%s",m->reason);snprintf(s->message_zh,sizeof(s->message_zh),"%s",stopped?"目标已静止，保持真实雷达回波跟随":independent?"雷达目标：已由连续扫描确认移动小障碍":"雷达已找回原视觉目标对应点簇，持续更新实际位置");return 1;
}
