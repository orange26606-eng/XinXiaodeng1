#include "object_dual_task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static double od_clamp(double a,double l,double h){return a<l?l:a>h?h:a;}
static double od_angle(double a){return remainder(a,6.283185307179586);}
static int od_fresh(uint64_t t,uint64_t now,uint64_t age){return t&&t<=now&&now-t<=age;}
static void od_state(object_dual_task*t,const char*s,const char*r,const char*m){if(s!=t->state)snprintf(t->state,sizeof(t->state),"%s",s);snprintf(t->reason,sizeof(t->reason),"%s",r);snprintf(t->message_zh,sizeof(t->message_zh),"%s",m);}
/* 数据源切换在世界航向中平滑，而非直接平滑车体坐标，避免将车身偏航误认为目标横移。
 */
static double od_direction(object_dual_task*t,double yaw,double bearing,uint64_t now,uint64_t source,const char*kind,int strong){
 double world=od_angle(yaw+bearing),dt=t->direction_time_us&&now>=t->direction_time_us?fmin(.25,(now-t->direction_time_us)/1e6):.05;
 int new_source=strong&&source>t->direction_visual_us;if(new_source)t->direction_visual_us=source;
 if(!t->direction_initialized){t->direction_world_yaw=world;t->direction_initialized=1;}
 else if(strong&&new_source&&fabs(od_angle(world-t->direction_world_yaw))>=OBJECT_DIRECTION_STRONG_RAD)t->direction_world_yaw=world;
 else t->direction_world_yaw=od_angle(t->direction_world_yaw+(1-exp(-dt/OBJECT_DIRECTION_TAU_S))*od_angle(world-t->direction_world_yaw));
 t->direction_time_us=now;t->direction_raw_rad=bearing;t->direction_filtered_rad=od_angle(t->direction_world_yaw-yaw);t->direction_source_us=source;snprintf(t->direction_source,sizeof(t->direction_source),"%s",kind);
 double magnitude=fabs(t->direction_filtered_rad);
 if(t->direction_turning){if(magnitude<=OBJECT_DIRECTION_EXIT_RAD)t->direction_turning=0;}
 else if(magnitude>=OBJECT_DIRECTION_ENTER_RAD)t->direction_turning=1;
 return t->direction_turning?copysign(fmax(0,magnitude-OBJECT_DIRECTION_EXIT_RAD),t->direction_filtered_rad):0;
}
static double od_follow_speed(object_dual_task*,uint64_t,double,int);
static void od_directed_end(object_dual_task*t,uint64_t now,const char*status){
 if(t->directed_source_us>t->directed_consumed_source_us)t->directed_consumed_source_us=t->directed_source_us;
 if(t->directed_active)t->directed_ended_us=now;t->directed_active=0;snprintf(t->directed_status,sizeof(t->directed_status),"%s",status);
}
static void od_directed_start(object_dual_task*t,const object_follow_exit_v5*e,double yaw,uint64_t now){
 double rate=isfinite(e->angular_velocity_rad_s)?od_clamp(e->angular_velocity_rad_s,-1,1):0;
 double bearing=od_angle(t->follow_exit_world_yaw-yaw+od_clamp(rate*((now-e->source_us)/1e6),-.60,.60));
 t->directed_active=1;t->directed_sign=bearing>=0?1:-1;t->directed_source_us=e->source_us;t->directed_frame_id=e->frame_id;t->directed_started_us=now;t->directed_ended_us=0;
 t->directed_start_yaw=t->directed_last_yaw=yaw;t->directed_progress_rad=0;t->directed_world_hint=od_angle(yaw+bearing);t->directed_goal_yaw=od_angle(yaw+t->directed_sign*OBJECT_FOLLOW_DIRECTED_ANGLE_V6);snprintf(t->directed_status,sizeof(t->directed_status),"ACTIVE");
}
static void od_directed_step(object_dual_task*t,double yaw,uint64_t now,lab_real_command*out){
 t->directed_progress_rad+=t->directed_sign*od_angle(yaw-t->directed_last_yaw);t->directed_last_yaw=yaw;
 double remaining=OBJECT_FOLLOW_DIRECTED_ANGLE_V6-t->directed_progress_rad;out->linear_m_s=od_follow_speed(t,now,0,1);out->angular_rad_s=0;
 if(remaining<=OBJECT_FOLLOW_DIRECTED_TOLERANCE_V6){od_directed_end(t,now,"COMPLETED");od_state(t,"WAIT_EXIT_EVIDENCE","FOLLOW_DIRECTED_35DEG_COMPLETED","已按真实里程计完成首段约35度定向找回；保持当前方向，等待新目标证据，不用同一旧提示反向扫动");return;}
 if(now<t->directed_started_us||now-t->directed_started_us>=OBJECT_FOLLOW_DIRECTED_TIMEOUT_V6){od_directed_end(t,now,"TIMEOUT");od_state(t,"WAIT_EXIT_EVIDENCE","FOLLOW_DIRECTED_6S_TIMEOUT","首段定向动作达到6秒上限，真实转角尚未完成；停止旧动作，等待新的视觉或雷达证据");return;}
 /* 接近实测端点时计入共用R81的200ms命令期限。 */
 double rate=fabs(obj_turn_endpoint(remaining,OBJECT_FOLLOW_DIRECTED_W));
 out->angular_rad_s=t->directed_sign*rate;t->exit_hint_used=1;snprintf(t->exit_hint_status,sizeof(t->exit_hint_status),"ACTIVE_LATCHED");
 t->direction_raw_rad=od_angle(t->directed_goal_yaw-yaw);t->direction_filtered_rad=t->direction_raw_rad;t->direction_source_us=t->directed_source_us;snprintf(t->direction_source,sizeof(t->direction_source),"EXIT_LATCH");
 od_state(t,"FOLLOW_EXIT_HINT",t->directed_sign>0?"FOLLOW_EXIT_LEFT_DIRECTED":"FOLLOW_EXIT_RIGHT_DIRECTED",t->directed_sign>0?"按原始退出证据向左持续定向找回，首段35度按真实里程计计量；新视觉或雷达可立即接管":"按原始退出证据向右持续定向找回，首段35度按真实里程计计量；新视觉或雷达可立即接管");
}
static void od_binding_reset(object_dual_task*t,uint32_t generation){
 od_directed_end(t,t->last_tick_us,"BINDING_CHANGED");t->directed_source_us=t->directed_frame_id=t->directed_consumed_source_us=0;
 t->binding_generation=generation;t->association_revision=0;t->association_source_us=0;t->association_reason[0]=0;memset(&t->follow_exit_hint,0,sizeof(t->follow_exit_hint));t->exit_hint_used=0;t->exit_hint_status[0]=0;++t->generation;t->have_target=t->range_valid=t->bearing_valid=t->waiting_distance=0;
 t->target_vx=t->target_vy=t->follow_v=0;t->follow_trace_count=t->follow_trace_cursor=0;t->follow_control_us=0;
 t->last_source_us=t->observation_seen_us=t->arrival_source_us=t->last_hint_us=t->last_visual_us=0;
 t->current_visual_valid=0;t->current_visual_us=0;t->near_latch=t->near_clear_count=t->arrival_count=0;
 t->velocity_anchor_us=0;t->velocity_jump_suppressed=0;t->direction_initialized=t->direction_turning=0;t->direction_time_us=t->direction_source_us=t->direction_visual_us=0;t->direction_source[0]=0;
 memset(&t->observation_v2,0,sizeof(t->observation_v2));t->source[0]=0;
}
/* 米制位置校正不是视觉身份变化，不能撤销START。 */
static void od_association_reset(object_dual_task*t,uint32_t revision,uint64_t source,const char*reason){
 od_directed_end(t,t->last_tick_us,"RANGE_ASSOCIATION_REVISED");
 t->association_revision=revision;if(t->association_corrections<UINT32_MAX)++t->association_corrections;t->association_source_us=source;++t->generation;
 snprintf(t->association_reason,sizeof(t->association_reason),"%s",reason&&*reason?reason:"FOLLOW_RANGE_ASSOCIATION_REVISED");
 for(char*p=t->association_reason;*p;p++)if((unsigned char)*p<32||*p=='"'||*p=='\\')*p='_';
 t->have_target=t->range_valid=t->waiting_distance=0;t->target_vx=t->target_vy=t->follow_v=0;
 t->follow_trace_count=t->follow_trace_cursor=0;t->follow_control_us=t->velocity_anchor_us=0;
 t->last_source_us=t->observation_seen_us=t->arrival_source_us=t->near_latch_us=0;t->near_latch=t->near_clear_count=t->arrival_count=0;
 if(strcmp(t->direction_source,"VISUAL")){t->bearing_valid=t->direction_initialized=t->direction_turning=0;t->direction_time_us=t->direction_source_us=0;}
 memset(&t->observation_v2,0,sizeof(t->observation_v2));t->source[0]=0;
}
static void od_exit_status(object_dual_task*t,const obj_detection*d,uint64_t now,double yaw,int visual,int radar){
 t->exit_hint_used=0;
 if(d){t->follow_exit_hint=d->follow_exit;if(d->follow_exit.compensated&&d->follow_exit.bearing_valid&&isfinite(d->follow_exit.bearing_rad))t->follow_exit_world_yaw=od_angle(yaw+d->follow_exit.bearing_rad);for(char*p=t->follow_exit_hint.reason;*p;p++)if((unsigned char)*p<32||*p=='"'||*p=='\\')*p='_';}
 const object_follow_exit_v5*e=&t->follow_exit_hint;
 const char*state=!e->valid?"NONE":visual?"RECOVERED_VISUAL":radar?"RADAR_PRIORITY":!od_fresh(e->source_us,now,2000000)?"EXPIRED":e->kind!=1&&e->kind!=2?"NOT_LATERAL":!e->compensated||!e->bearing_valid||!isfinite(e->bearing_rad)||fabs(e->bearing_rad)>3.14159265358979323846?"UNCOMPENSATED":"READY_DIRECTED";
 snprintf(t->exit_hint_status,sizeof(t->exit_hint_status),"%s",state);
}
/* 新的近距离始终可用，点簇跳变时仅抑制速度前馈；实际基线至少200ms以减小差分噪声。
 */
static void od_follow_velocity(object_dual_task*t,uint64_t source,double x,double y,double error){
 t->velocity_jump_suppressed=0;
 if(!t->have_target||!t->velocity_anchor_us){t->velocity_anchor_us=source;t->velocity_anchor_x=x;t->velocity_anchor_y=y;t->target_vx=t->target_vy=0;return;}
 double step=source>t->last_source_us?(source-t->last_source_us)/1e6:0,move=hypot(x-t->previous_x,y-t->previous_y);
 double allowance=.12+fmin(.20,fmax(0,error))+1.0*step;
 if(step>0&&step<=2&&move>allowance){t->target_vx=t->target_vy=0;t->velocity_jump_suppressed=1;if(t->velocity_jump_count<UINT32_MAX)++t->velocity_jump_count;t->velocity_anchor_us=source;t->velocity_anchor_x=x;t->velocity_anchor_y=y;return;}
 double dt=source>t->velocity_anchor_us?(source-t->velocity_anchor_us)/1e6:0;
 if(dt>=.20){if(dt<=1.0){double vx=od_clamp((x-t->velocity_anchor_x)/dt,-1.5,1.5),vy=od_clamp((y-t->velocity_anchor_y)/dt,-1.5,1.5);t->target_vx=.65*t->target_vx+.35*vx;t->target_vy=.65*t->target_vy+.35*vy;}else t->target_vx=t->target_vy=0;t->velocity_anchor_us=source;t->velocity_anchor_x=x;t->velocity_anchor_y=y;}
}
static void od_follow_prune(object_dual_task*t,uint64_t now){
 unsigned old=0;while(old<t->follow_trace_count&&t->follow_trace[old].source_us<=now&&now-t->follow_trace[old].source_us>OBJECT_FOLLOW_TRACE_TTL_US)++old;
 if(old){memmove(t->follow_trace,t->follow_trace+old,(t->follow_trace_count-old)*sizeof(*t->follow_trace));t->follow_trace_count-=old;t->follow_trace_cursor=t->follow_trace_cursor>old?t->follow_trace_cursor-old:0;}
}
static void od_follow_record(object_dual_task*t,uint64_t source,double x,double y){
 if(t->mode!=OBJECT_MODE_FOLLOW)return;
 if(t->follow_trace_count&&source<=t->follow_trace[t->follow_trace_count-1].source_us)return;
 od_follow_prune(t,source);
 if(t->follow_trace_count==OBJECT_FOLLOW_TRACE_MAX){memmove(t->follow_trace,t->follow_trace+1,(OBJECT_FOLLOW_TRACE_MAX-1)*sizeof(*t->follow_trace));--t->follow_trace_count;if(t->follow_trace_cursor)--t->follow_trace_cursor;}
 t->follow_trace[t->follow_trace_count++]=(object_dual_trace){x,y,source};
}
static double od_follow_speed(object_dual_task*t,uint64_t now,double desired,int immediate){
 double dt=t->follow_control_us&&now>=t->follow_control_us?fmin(.10,(now-t->follow_control_us)/1e6):.05;t->follow_control_us=now;
 if(immediate)t->follow_v=0;else{double rate=desired<t->follow_v?.60:.35;t->follow_v+=od_clamp(desired-t->follow_v,-rate*dt,rate*dt);}
 t->follow_v=od_clamp(t->follow_v,0,.25);return t->follow_v;
}
/* 任务意图指向最新测量或预测目标，历史目标点仅作关联和诊断，由R8.1生成车辆路线。
 */
static double od_follow_target_bearing(object_dual_task*t,double px,double py,double yaw,uint64_t now){
 double age=t->last_source_us<=now?(now-t->last_source_us)/1e6:0,predict=fmin(OBJECT_FOLLOW_PREDICT_US/1e6,age);
 double x=t->target_x+t->target_vx*predict,y=t->target_y+t->target_vy*predict;
 t->follow_goal_x=x;t->follow_goal_y=y;return od_angle(atan2(y-py,x-px)-yaw);
}
#include "object_visual_docking.inc"
void object_dual_task_init(object_dual_task*t){if(!t)return;memset(t,0,sizeof(*t));t->mode=OBJECT_MODE_STATIC;t->track_id=-1;t->task_class_id=-1;t->standoff_m=.20;t->front_m=.1665;od_state(t,"IDLE","NO_SELECTED_OBJECT","请选择真实物体，并选择前往或跟随模式");}
int object_dual_task_prepare(object_dual_task*t,unsigned mode,uint64_t epoch,int id,double stop,uint64_t request){if(!t||(mode!=1&&mode!=2)||!epoch||id<1||!isfinite(stop)||stop<.10||stop>3)return 0;uint64_t gen=t->generation+1;object_dual_task_init(t);t->generation=gen;t->mode=mode;t->epoch=epoch;t->track_id=id;t->standoff_m=stop;t->request_id=request;t->prepared=1;od_state(t,"READY","EXPLICIT_START_REQUIRED",mode==1?"静止物体已准备，等待明确开始":"移动物体已准备，等待明确开始跟随");return 1;}
/* 语义任务本身不是相机测量身份或坐标。 */
int object_dual_task_prepare_search_attributes(object_dual_task*t,const char*category,const char*color,const char*size,double stop,uint64_t request){
 if(!t||!category||!isfinite(stop)||stop<.10||stop>3)return 0;char cat[64],col[16],sz[12];
 if(!ovd_text(cat,sizeof cat,category)||!cat[0]||!ovd_text(col,sizeof col,color)||!ovd_text(sz,sizeof sz,size))return 0;
 if(col[0]&&!obj_color_id(col))return 0;if(sz[0]&&strcmp(sz,"small")&&strcmp(sz,"medium")&&strcmp(sz,"large"))return 0;
 uint64_t gen=t->generation+1;object_dual_task_init(t);t->generation=gen;t->mode=OBJECT_MODE_STATIC;t->static_search=1;t->epoch=0;t->track_id=0;t->standoff_m=stop;t->request_id=request;t->prepared=1;
 snprintf(t->search_category,sizeof t->search_category,"%s",cat);snprintf(t->search_color,sizeof t->search_color,"%s",col);snprintf(t->search_size,sizeof t->search_size,"%s",sz);
 od_state(t,"READY","STATIC_ATTRIBUTE_SEARCH_READY","类别、颜色和大小任务已准备；明确开始后持续寻找匹配实物，尚未伪造图像身份");return 1;
}
int object_dual_task_prepare_search(object_dual_task*t,const char*category,double stop,uint64_t request){return object_dual_task_prepare_search_attributes(t,category,"","",stop,request);}
static int od_category_equal(const char*a,const char*b){
 for(unsigned k=0;k<64;k++){unsigned char x=(unsigned char)a[k],y=(unsigned char)b[k];if(x>='A'&&x<='Z')x+=32;if(y>='A'&&y<='Z')y+=32;if(x=='_'||x=='-')x=' ';if(y=='_'||y=='-')y=' ';if(x!=y)return 0;if(!x)return 1;}return 0;
}
static const obj_detection*od_static_search_detection(const object_dual_task*t,const obj_frame*f,uint64_t now){
 if(!f||!f->epoch||f->count<0||f->count>OBJ_MAX)return NULL;const obj_detection*best=NULL;double extent=0,center=1e9;unsigned metrics=0;
 for(int k=0;k<f->count;k++){const obj_detection*d=f->objects+k;uint64_t stamp=d->observation_time_us?d->observation_time_us:f->frame_us;
  if(!d->observed||d->track_id<1||!od_fresh(stamp,now,1500000)||!od_category_equal(d->label,t->search_category))continue;
  if(t->search_bound){if(f->epoch==t->epoch&&d->track_id==t->track_id)return d;continue;}
  if(!obj_attribute_color_match(&d->attributes,t->search_color))continue;
  if(t->search_color[0]&&d->attributes.color_source_us&&!od_fresh(d->attributes.color_source_us,now,2000000))continue;
  double e=obj_attribute_metric_extent(&d->attributes,t->search_category);if(!isfinite(e)||e<=0)e=0;
  if(t->search_size[0]){
   if(!e)continue;metrics++;
   if(!best||(!strcmp(t->search_size,"large")?e>extent:!strcmp(t->search_size,"small")?e<extent:fabs(e-.30)<fabs(extent-.30))){best=d;extent=e;}
  }else{double c=d->bearing_valid&&isfinite(d->bearing)?fabs(d->bearing):d->bbox_valid?fabs((d->bbox[0]+d->bbox[2])*.5-480.)/480.:1e9;if(!best||c<center){best=d;center=c;}}
 }
 if(best&&t->search_size[0]&&metrics==1&&strcmp(best->attributes.size,t->search_size))return NULL;
 return best;
}
static const obj_detection*od_static_probe_detection(const object_dual_task*t,const obj_frame*f,uint64_t now){
 if(!t->search_size[0]||!f||!f->epoch||f->count<0||f->count>OBJ_MAX)return NULL;const obj_detection*best=NULL;double score=1e9;
 for(int k=0;k<f->count;k++){const obj_detection*d=f->objects+k;uint64_t stamp=d->observation_time_us?d->observation_time_us:f->frame_us;
  if(!d->observed||!d->bbox_valid||d->track_id<1||!od_fresh(stamp,now,1500000)||!od_category_equal(d->label,t->search_category)||!obj_attribute_color_match(&d->attributes,t->search_color))continue;
  if(t->search_color[0]&&d->attributes.color_source_us&&!od_fresh(d->attributes.color_source_us,now,2000000))continue;
  double extent=obj_attribute_metric_extent(&d->attributes,t->search_category);if(isfinite(extent)&&extent>0)continue;
  if(t->probe_active&&t->probe_epoch==f->epoch&&t->probe_track_id==d->track_id)return d;
  double rank=d->bearing_valid&&isfinite(d->bearing)?fabs(d->bearing):fabs((d->bbox[0]+d->bbox[2])*.5-480.)/480.;if(!best||rank<score){best=d;score=rank;}
 }return best;
}
/* 未标定的框中心仅提供角度提示。 */
static int od_static_direction(obj_detection*d){
 if(d->bearing_valid&&isfinite(d->bearing)&&fabs(d->bearing)<=3.14159265358979323846)return 0;
 for(unsigned k=0;k<4;k++)if(!isfinite(d->bbox[k]))return 0;
 if(d->bbox[2]<=d->bbox[0]||d->bbox[3]<=d->bbox[1]||d->bbox[0]<0||d->bbox[2]>960||d->bbox[1]<0||d->bbox[3]>1080)return 0;
 double cx=(d->bbox[0]+d->bbox[2])*.5;d->bearing=od_clamp(atan((480.-cx)/480.),-.7853981633974483,.7853981633974483);d->bearing_valid=1;return 1;
}
int object_dual_task_prepare_radar(object_dual_task*t,uint64_t epoch,double stop,uint64_t request){
 if(!object_dual_task_prepare(t,OBJECT_MODE_FOLLOW,epoch,1,stop,request))return 0;
 t->radar_only=1;snprintf(t->source,sizeof(t->source),"RADAR_ONLY");
 od_state(t,"READY","EXPLICIT_RADAR_START_REQUIRED","雷达跟随已准备，等待明确开始；无需相机目标");return 1;
}
/* 类型4仅接受有真实扫描时刻且不带图像来源的观测。 */
static int od_measure_kind(const object_dual_task*t,const object_target_observation_v2*b,uint64_t now){
 if(b->kind==OBJECT_MEASURE_RADAR_ONLY)return t->mode==OBJECT_MODE_FOLLOW&&b->image_us==0&&b->frame_id==0&&b->scan_us==b->source_us&&b->scan_us>0;
 return !t->radar_only&&b->kind>=OBJECT_MEASURE_RADAR_VISUAL&&b->kind<=OBJECT_MEASURE_STEREO_COARSE&&b->image_us&&b->image_us<=now;
}
int object_dual_task_start(object_dual_task*t,unsigned mode,uint64_t epoch,int id,uint64_t now){if(!t||!t->prepared||t->arrived||t->mode!=mode||t->epoch!=epoch||t->track_id!=id||!now)return 0;t->active=t->explicit_start=1;memset(&t->turn_measure,0,sizeof t->turn_measure);t->static_endpoint_hold_us=t->fallback_hold_us=0;t->fallback_active=0;t->started_us=now;t->last_tick_us=0;t->follow_v=0;t->follow_control_us=0;t->direction_time_us=0;if(t->mode==OBJECT_MODE_FOLLOW)t->target_vx=t->target_vy=0;od_state(t,"WAIT_TARGET","WAIT_CURRENT_TARGET_EVIDENCE","任务已开始，等待当前目标方向和本地传感器");return 1;}
void object_dual_task_pause(object_dual_task*t,const char*why){if(!t)return;od_directed_end(t,t->last_tick_us,"PAUSED");t->active=t->explicit_start=0;t->requested_v=t->requested_w=t->follow_v=0;t->follow_control_us=t->last_tick_us=t->direction_time_us=0;od_state(t,"PAUSED",why?why:"USER_PAUSE","任务已暂停，目标身份与轨迹记忆保留，点击继续后恢复");}
void object_dual_task_cancel(object_dual_task*t,const char*why){if(!t)return;od_directed_end(t,t->last_tick_us,"CANCELLED");t->active=t->explicit_start=t->prepared=t->have_target=t->range_valid=t->bearing_valid=t->static_lost=0;t->probe_active=0;t->near_hold=t->near_metric=0;t->near_reason[0]=0;memset(&t->visual_dock,0,sizeof t->visual_dock);t->current_visual_valid=0;t->current_visual_us=0;t->near_latch=t->near_clear_count=0;t->observation_seen_us=0;memset(&t->observation_v2,0,sizeof(t->observation_v2));t->last_hint_us=0;t->follow_trace_count=t->follow_trace_cursor=0;t->follow_v=0;t->follow_control_us=0;t->requested_v=t->requested_w=0;t->direction_initialized=t->direction_turning=0;t->direction_source[0]=0;memset(&t->follow_exit_hint,0,sizeof(t->follow_exit_hint));t->exit_hint_used=0;t->exit_hint_status[0]=0;t->generation++;od_state(t,"CANCELLED",why?why:"USER_CANCEL","任务已取消，不会自动恢复运动");}
int object_dual_task_tick(object_dual_task*t,const obj_scene*s,const obj_frame*f,const lab_real_runtime_view*v,uint64_t now,lab_real_command*out){
 if(out)memset(out,0,sizeof(*out));if(!t||!v||!out)return 0;out->context=v->context;out->generated_us=now;out->expires_us=now+CONT_COMMAND_US;t->requested_v=t->requested_w=0;t->hint_suppressed_for_memory=0;
 if(t->mode==OBJECT_MODE_FOLLOW)od_follow_prune(t,now);
 if(!t->prepared||!t->active||!t->explicit_start||t->arrived)return 0;
 if(t->last_tick_us&&now<t->last_tick_us){od_state(t,"WAIT_SENSORS","OBJECT_CLOCK_ROLLBACK","目标任务时钟倒退，等待重新准备");return 1;}t->last_tick_us=now;
 if(t->have_context&&memcmp(&t->context,&v->context,sizeof(v->context))){od_directed_end(t,now,"CONTEXT_CHANGED");t->have_target=t->range_valid=t->bearing_valid=0;if(t->mode==OBJECT_MODE_STATIC){memset(&t->visual_dock,0,sizeof t->visual_dock);t->near_hold=t->near_metric=0;t->near_reason[0]=0;t->probe_active=t->probe_current=0;t->probe_reason[0]=0;}t->near_latch=0;t->observation_seen_us=0;memset(&t->observation_v2,0,sizeof(t->observation_v2));t->follow_trace_count=t->follow_trace_cursor=0;t->follow_v=0;t->context=v->context;t->have_context=1;t->direction_initialized=t->direction_turning=0;t->last_source_us=t->last_hint_us=0;t->follow_control_us=0;od_state(t,"WAIT_TARGET","ODOMETRY_CONTEXT_AUTO_REASSOCIATING","里程计参考已改变，保留目标身份并自动重新关联，旧位置已清除");return 1;}t->context=v->context;t->have_context=1;
 if(!v->has_odometry||!od_fresh(v->odometry_time_us,now,OBJECT_CONTROL_POSE_US)||!(v->raw_odometry.quality&LAB_POSE_VALID)||(v->raw_odometry.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))||!isfinite(v->raw_odometry.x_m)||!isfinite(v->raw_odometry.y_m)||!isfinite(v->raw_odometry.yaw_rad)){od_state(t,"WAIT_SENSORS","RAW_ODOMETRY_NOT_FRESH","当前编码器和航向不可用；任务保留，等待新数据");return 1;}
 obj_turn_observe(&t->turn_measure,v->odometry_time_us,v->raw_odometry.yaw_rad);
 t->static_bearing_estimated=0;
 const obj_detection*search_detection=NULL;
 if(t->mode==OBJECT_MODE_STATIC&&t->static_search){
  search_detection=od_static_search_detection(t,f,now);
  if(search_detection&&(!t->search_bound||t->epoch!=f->epoch||t->track_id!=search_detection->track_id)){
   od_binding_reset(t,search_detection->binding_generation);t->epoch=f->epoch;t->track_id=search_detection->track_id;t->task_class_id=search_detection->class_id;t->search_bound=1;t->static_lost=0;t->probe_active=0;
   od_state(t,"APPROACHING","STATIC_FIRST_DETECTOR_LOCK","第一笔指定类别实测检测已直接锁定，不等待额外确认或选择回执");
  }
  if(!t->search_bound){
   out->linear_m_s=0;out->angular_rad_s=OBJECT_STATIC_SEARCH_MAX_RATE;t->requested_v=0;t->requested_w=out->angular_rad_s;t->current_visual_valid=0;t->current_visual_us=0;
   unsigned category_count=0,color_count=0,color_available=0,metric_count=0;
   if(f&&f->count>=0&&f->count<=OBJ_MAX)for(int k=0;k<f->count;k++){const obj_detection*q=f->objects+k;uint64_t stamp=q->observation_time_us?q->observation_time_us:f->frame_us;if(!q->observed||!od_fresh(stamp,now,1500000)||!od_category_equal(q->label,t->search_category))continue;++category_count;int available=q->attributes.color_valid&&(!q->attributes.color_source_us||od_fresh(q->attributes.color_source_us,now,2000000));if(available)++color_available;if(t->search_color[0]&&(!available||!obj_attribute_color_match(&q->attributes,t->search_color)))continue;++color_count;double extent=obj_attribute_metric_extent(&q->attributes,t->search_category);if(isfinite(extent)&&extent>0)++metric_count;}
   t->search_class_candidates=category_count;t->search_color_candidates=color_count;t->search_metric_candidates=metric_count;
   const obj_detection*probe=od_static_probe_detection(t,f,now);
   if(probe){
    obj_detection candidate=*probe;od_static_direction(&candidate);
    if(candidate.bearing_valid&&isfinite(candidate.bearing)){
     if(!t->probe_active||t->probe_epoch!=f->epoch||t->probe_track_id!=candidate.track_id){memset(&t->visual_dock,0,sizeof t->visual_dock);t->near_hold=t->near_metric=t->near_latch=0;t->near_reason[0]=0;}
     t->probe_active=t->probe_current=1;t->probe_epoch=f->epoch;t->probe_track_id=candidate.track_id;t->probe_eye=candidate.eye;t->probe_frame_id=candidate.observation_frame_id?candidate.observation_frame_id:f->frame_id;t->probe_source_us=candidate.observation_time_us?candidate.observation_time_us:f->frame_us;t->probe_bearing_rad=candidate.bearing;t->probe_world_yaw=od_angle(v->raw_odometry.yaw_rad+candidate.bearing);
     snprintf(t->probe_reason,sizeof t->probe_reason,"SIZE_METRIC_UNAVAILABLE_ALIGN_REAL_CANDIDATE");
     /* 试探候选有真实观测身份，但大小尚未语义确认；执行近处保护，不导出目标坐标或ARRIVED。
 */
     uint64_t saved_epoch=t->epoch,saved_source=t->last_source_us;int saved_track=t->track_id;
     t->epoch=f->epoch;t->track_id=candidate.track_id;ovd_update(t,&candidate,f,NULL,0,v,now,1);t->epoch=saved_epoch;t->track_id=saved_track;t->last_source_us=saved_source;t->have_target=t->range_valid=0;
     out->linear_m_s=0;out->angular_rad_s=t->near_hold||fabs(candidate.bearing)<=OBJECT_TURN_DEADZONE_RAD?0:od_clamp(.8*candidate.bearing,-OBJECT_NORMAL_TURN_MAX_RATE,OBJECT_NORMAL_TURN_MAX_RATE);
     t->requested_v=0;t->requested_w=out->angular_rad_s;snprintf(t->direction_source,sizeof t->direction_source,"SIZE_CANDIDATE_OBSERVATION");t->direction_source_us=t->probe_source_us;
     od_state(t,"OBSERVING_CANDIDATE",t->near_hold?"SIZE_PROBE_NEAR_HOLD":"SIZE_PROBE_ALIGN_CURRENT_CANDIDATE",t->near_hold?"颜色和类别符合的候选已近，保持停止观察大小；尚未宣称选对大小或到达":"已看到颜色和类别符合的候选，正在对准以观察大小；不继续无差别左转，不把未知大小当作已选中");return 1;
    }
   }
   int probe_rejected=0;if(t->probe_active&&f&&f->epoch==t->probe_epoch)for(int k=0;k<f->count&&k<OBJ_MAX;k++){const obj_detection*q=f->objects+k;uint64_t stamp=q->observation_time_us?q->observation_time_us:f->frame_us;if(q->track_id==t->probe_track_id&&q->observed&&od_fresh(stamp,now,1500000)&&((q->attributes.size_valid&&obj_attribute_metric_extent(&q->attributes,t->search_category)>0)||!od_category_equal(q->label,t->search_category)))probe_rejected=1;}
   if(t->probe_active&&!probe_rejected&&(t->near_hold||od_fresh(t->probe_source_us,now,1500000))){
    t->probe_current=0;t->probe_bearing_rad=od_angle(t->probe_world_yaw-v->raw_odometry.yaw_rad);out->linear_m_s=0;out->angular_rad_s=t->near_hold||fabs(t->probe_bearing_rad)<=OBJECT_TURN_DEADZONE_RAD?0:od_clamp(.8*t->probe_bearing_rad,-OBJECT_NORMAL_TURN_MAX_RATE,OBJECT_NORMAL_TURN_MAX_RATE);t->requested_v=0;t->requested_w=out->angular_rad_s;
    snprintf(t->probe_reason,sizeof t->probe_reason,"SIZE_PROBE_SHORT_VISUAL_MEMORY");od_state(t,"OBSERVING_CANDIDATE",t->near_hold?"SIZE_PROBE_NEAR_HOLD":"SIZE_PROBE_SHORT_VISUAL_MEMORY",t->near_hold?"候选已近，保持停止等待真实大小证据；不因丢框转向搜索":"候选短时没有新观测，按原方位对准观察；保留原采集时间，不冒充当前发现");return 1;
   }
   if(t->probe_active){t->probe_active=0;t->probe_reason[0]=0;memset(&t->visual_dock,0,sizeof t->visual_dock);t->near_hold=t->near_metric=t->near_latch=0;t->near_reason[0]=0;}
   const char*why=!category_count?"STATIC_SEARCH_NO_CLASS":t->search_color[0]&&!color_available?"STATIC_SEARCH_COLOR_UNAVAILABLE":t->search_color[0]&&!color_count?"STATIC_SEARCH_COLOR_NOT_MATCHED":t->search_size[0]&&!metric_count?"STATIC_SEARCH_SIZE_METRIC_UNAVAILABLE":metric_count==1?"STATIC_SEARCH_SINGLE_SIZE_NOT_MATCHED":"STATIC_SEARCH_SIZE_NOT_MATCHED";
   const char*detail=!category_count?"当前未见指定类别":t->search_color[0]&&!color_available?"当前颜色不可辨":t->search_color[0]&&!color_count?"同类已有，但颜色不符":t->search_size[0]&&!metric_count?"大小尚无距离依据":metric_count==1?"唯一候选的实物大小不符":"继续寻找符合条件的物体";
   const char*label=!strcmp(t->search_category,"bottle")?"水瓶":!strcmp(t->search_category,"box")?"箱子":!strcmp(t->search_category,"person")?"人":!strcmp(t->search_category,"cup")?"杯子":t->search_category;
   const char*size=!strcmp(t->search_size,"large")?"较大":!strcmp(t->search_size,"small")?"较小":!strcmp(t->search_size,"medium")?"中等":"不限";
   char message[256];snprintf(message,sizeof message,"左转搜索：%.31s/%s/%s；同类%u，颜色符合%u，有尺度%u；%s",label,t->search_color[0]?obj_color_zh(obj_color_id(t->search_color)):"不限",size,category_count,color_count,metric_count,detail);
   snprintf(t->direction_source,sizeof(t->direction_source),"SEMANTIC_SEARCH_LEFT");t->direction_source_us=0;od_state(t,"SEARCHING",why,message);return 1;
  }
 }
 /* 只有同任务较新的扫描可显式撤销旧墙面距离，撤销扫描不转为新图像或新距离样本。
 */
 if(t->mode==OBJECT_MODE_STATIC&&s&&s->epoch==t->epoch&&s->track_id==t->track_id&&s->binding_generation==t->binding_generation&&lab_real_context_equal(&s->context,&v->context)&&!s->target_odom_valid&&!s->measurement_v2.valid&&(!strcmp(s->reason,"STATIC_WALL_EXCLUDED")||!strcmp(s->follow_reason,"STATIC_WALL_EXCLUDED"))&&od_fresh(s->scan_us,now,OBJECT_CONTROL_MEASUREMENT_US)&&s->scan_us>t->last_source_us&&s->scan_us>t->static_range_revoked_us){
  t->static_range_revoked_us=s->scan_us;t->have_target=t->range_valid=t->near_latch=t->near_clear_count=t->arrival_count=0;t->arrival_source_us=t->observation_seen_us=0;t->direction_initialized=t->direction_turning=0;t->static_lost=0;memset(&t->observation_v2,0,sizeof(t->observation_v2));++t->generation;
  snprintf(t->source,sizeof(t->source),"STATIC_WALL_EXCLUDED");
 }
 const object_target_observation_v2*radar=s?&s->measurement_v2:NULL;
 int radar_current=t->mode==OBJECT_MODE_FOLLOW&&radar&&radar->valid&&radar->kind==OBJECT_MEASURE_RADAR_ONLY&&od_measure_kind(t,radar,now)&&radar->epoch==t->epoch&&radar->track_id==t->track_id&&lab_real_context_equal(&radar->context,&v->context)&&od_fresh(radar->source_us,now,OBJECT_CONTROL_MEASUREMENT_US)&&isfinite(radar->odom_x_m)&&isfinite(radar->odom_y_m)&&isfinite(radar->bearing_rad)&&fabs(radar->bearing_rad)<=3.14159265358979323846&&isfinite(radar->error_m)&&radar->error_m>=0&&isfinite(radar->surface_gap_m)&&radar->surface_gap_m>=0;
 int radar_priority=radar_current||(t->mode==OBJECT_MODE_FOLLOW&&t->observation_v2.valid&&t->observation_v2.kind==OBJECT_MEASURE_RADAR_ONLY&&lab_real_context_equal(&t->observation_v2.context,&v->context)&&od_fresh(t->observation_v2.source_us,now,OBJECT_CONTROL_MEASUREMENT_US));
 if(radar_priority&&!radar_current)s=NULL;
 const obj_detection*d=NULL;int identity=f&&f->epoch==t->epoch&&(t->mode==OBJECT_MODE_STATIC||f->selected==t->track_id);if(identity&&!t->radar_only&&!radar_priority)for(int i=0;i<f->count;i++)if(f->objects[i].track_id==t->track_id)d=&f->objects[i];
 obj_detection static_detection;
 if(t->mode==OBJECT_MODE_STATIC){
  if(search_detection)d=search_detection;
  if(d){static_detection=*d;t->static_bearing_estimated=od_static_direction(&static_detection);d=&static_detection;t->binding_generation=d->binding_generation;}
 }
 if(d&&t->task_class_id<0&&d->task_class_id>=0)t->task_class_id=d->task_class_id;
 uint32_t incoming_binding=d?d->binding_generation:0;
 if(t->mode==OBJECT_MODE_FOLLOW&&s&&(radar_current||(s->epoch==t->epoch&&s->track_id==t->track_id))){
  const object_target_observation_v2*b=&s->measurement_v2;
  if(s->target_odom_valid&&!s->historical&&lab_real_context_equal(&s->context,&v->context)&&od_fresh(s->source_us,now,1500000)&&s->binding_generation>incoming_binding)incoming_binding=s->binding_generation;
  if(b->valid&&b->epoch==t->epoch&&b->track_id==t->track_id&&lab_real_context_equal(&b->context,&v->context)&&od_fresh(b->source_us,now,OBJECT_CONTROL_MEASUREMENT_US)&&b->binding_generation>incoming_binding)incoming_binding=b->binding_generation;
 }
 if(radar_current)incoming_binding=radar->binding_generation;
 if(t->mode==OBJECT_MODE_FOLLOW&&((radar_current&&incoming_binding!=t->binding_generation)||incoming_binding>t->binding_generation))od_binding_reset(t,incoming_binding);
 if(t->mode==OBJECT_MODE_FOLLOW&&s&&(radar_current||(s->epoch==t->epoch&&s->track_id==t->track_id))){
  uint32_t revision=t->association_revision;uint64_t revision_source=0;
  if(s->binding_generation==t->binding_generation&&s->target_odom_valid&&!s->historical&&lab_real_context_equal(&s->context,&v->context)&&od_fresh(s->source_us,now,1500000)&&isfinite(s->target_odom_x)&&isfinite(s->target_odom_y)&&s->association_revision>revision){revision=s->association_revision;revision_source=s->source_us;}
  const object_target_observation_v2*b=&s->measurement_v2;
  if(b->valid&&b->epoch==t->epoch&&b->track_id==t->track_id&&b->binding_generation==t->binding_generation&&lab_real_context_equal(&b->context,&v->context)&&od_fresh(b->source_us,now,OBJECT_CONTROL_MEASUREMENT_US)&&od_measure_kind(t,b,now)&&isfinite(b->odom_x_m)&&isfinite(b->odom_y_m)&&isfinite(b->surface_gap_m)&&b->surface_gap_m>=0&&isfinite(b->error_m)&&b->error_m>=0&&b->association_revision>revision){revision=b->association_revision;revision_source=b->source_us;}
  if(revision>t->association_revision)od_association_reset(t,revision,revision_source,s->follow_reason);
 }
 int binding_current=!d||d->binding_generation==t->binding_generation;
 uint64_t visual_us=d&&d->observation_time_us?d->observation_time_us:f?f->frame_us:0;
 int visual=d&&binding_current&&d->observed&&d->bearing_valid&&isfinite(d->bearing)&&(t->mode==OBJECT_MODE_STATIC||(strcmp(d->tracking,"LOST")&&strcmp(d->tracking,"PREDICTED")))&&od_fresh(visual_us,now,1500000);
 t->current_visual_valid=visual&&d->bearing_valid&&isfinite(d->bearing);t->current_visual_us=t->current_visual_valid?visual_us:0;
 if(visual){t->fallback_active=0;t->fallback_hold_us=0;}
 if(visual&&visual_us>t->last_visual_us){
  if(t->static_lost&&visual_us>t->static_lost_visual_us)t->static_lost=0;
  if(d->bearing_valid&&isfinite(d->bearing)){
   double fresh_yaw=od_angle(v->raw_odometry.yaw_rad+d->bearing);
   t->visual_world_yaw=fresh_yaw;
   t->bearing_rad=od_angle(t->visual_world_yaw-v->raw_odometry.yaw_rad);t->bearing_valid=1;t->search_center_yaw=t->visual_world_yaw;
  }
  t->last_visual_us=visual_us;t->last_visual_yaw=v->raw_odometry.yaw_rad;
 }

 /* 配对输入来自当前原始观测。 */
 obj_scene observation_scene;
 const object_target_observation_v2 *ob=s?&s->measurement_v2:NULL;
 int current_ob=ob&&ob->valid&&ob->epoch==t->epoch&&ob->track_id==t->track_id&&ob->binding_generation==t->binding_generation&&(t->mode!=OBJECT_MODE_FOLLOW||ob->association_revision==t->association_revision)&&
  od_measure_kind(t,ob,now)&&
  lab_real_context_equal(&ob->context,&v->context)&&od_fresh(ob->source_us,now,OBJECT_CONTROL_MEASUREMENT_US)&&
  isfinite(ob->bearing_rad)&&fabs(ob->bearing_rad)<=3.14159265358979323846&&isfinite(ob->error_m)&&ob->error_m>=0&&
  isfinite(ob->odom_x_m)&&isfinite(ob->odom_y_m)&&isfinite(ob->surface_gap_m)&&ob->surface_gap_m>=0;
 if(current_ob){
  if(ob->source_us>t->observation_seen_us){t->observation_v2=*ob;t->observation_seen_us=ob->source_us;
   if(t->mode==OBJECT_MODE_STATIC&&ob->surface_valid){
    double gap=fmin(ob->surface_gap_m,fmax(0,hypot(ob->odom_x_m-v->raw_odometry.x_m,ob->odom_y_m-v->raw_odometry.y_m)-t->front_m));
    if(gap<=t->standoff_m+.03){t->near_latch=1;t->near_latch_us=ob->source_us;t->near_clear_count=0;}
    else if(t->near_latch&&++t->near_clear_count>=2){t->near_latch=0;t->near_clear_count=0;}
   }
  }
  observation_scene=*s;observation_scene.epoch=t->epoch;observation_scene.track_id=t->track_id;observation_scene.binding_generation=ob->binding_generation;observation_scene.association_revision=ob->association_revision;observation_scene.source_us=ob->source_us;observation_scene.target_odom_valid=1;
  observation_scene.target_odom_x=ob->odom_x_m;observation_scene.target_odom_y=ob->odom_y_m;
  observation_scene.confirmed=ob->confirmed;observation_scene.historical=0;
  snprintf(observation_scene.source,sizeof(observation_scene.source),"%s",ob->source);s=&observation_scene;
 }
 ovd_update(t,d,f,ob,current_ob,v,now,visual);
 int ignored_static=0;int measured=(!ob||!ob->valid||current_ob)&&s&&s->epoch==t->epoch&&s->track_id==t->track_id&&s->binding_generation==t->binding_generation&&(t->mode!=OBJECT_MODE_FOLLOW||s->association_revision==t->association_revision)&&s->target_odom_valid&&!s->historical&&od_fresh(s->source_us,now,1500000)&&isfinite(s->target_odom_x)&&isfinite(s->target_odom_y);
 if(t->radar_only&&!current_ob)measured=0;
 if(t->mode==OBJECT_MODE_STATIC&&t->near_hold)measured=0;
 if(measured&&s->source_us>t->last_source_us){double x=s->target_odom_x,y=s->target_odom_y;if(t->mode==OBJECT_MODE_FOLLOW)od_follow_velocity(t,s->source_us,x,y,current_ob?ob->error_m:0);
  t->target_x=x;t->target_y=y; /* 当前静止前景立即校正粗位置先验。 */
  if(!ignored_static){t->previous_x=x;t->previous_y=y;t->have_target=1;t->last_source_us=s->source_us;od_follow_record(t,s->source_us,x,y);snprintf(t->source,sizeof(t->source),"%s",s->source);}else measured=0;
 }
 double px=v->raw_odometry.x_m,py=v->raw_odometry.y_m,yaw=v->raw_odometry.yaw_rad;int memory=t->have_target&&(t->mode==OBJECT_MODE_STATIC||od_fresh(t->last_source_us,now,OBJECT_FOLLOW_LOST_STOP_US));
 t->range_valid=memory;
 if(memory){double source_age=t->last_source_us<=now?(now-t->last_source_us)/1e6:1e9;double pred=t->mode==OBJECT_MODE_FOLLOW?fmin(OBJECT_FOLLOW_PREDICT_US/1e6,source_age):0,x=t->target_x+t->target_vx*pred,y=t->target_y+t->target_vy*pred;t->range_m=fmax(0,hypot(x-px,y-py)-t->front_m);t->bearing_rad=od_angle(atan2(y-py,x-px)-yaw);t->bearing_valid=1;t->search_center_yaw=od_angle(yaw+t->bearing_rad);}
 else if(t->bearing_valid&&od_fresh(t->last_visual_us,now,3000000)){t->bearing_rad=od_angle(t->search_center_yaw-yaw);}
 /* 当前真实图像方向优先于记忆位置方位，记忆仍提供距离，但不覆盖所选视觉射线。
 */
 if(visual&&d->bearing_valid&&isfinite(d->bearing)){t->bearing_rad=od_angle(t->visual_world_yaw-yaw);t->bearing_valid=1;t->search_center_yaw=t->visual_world_yaw;}
 if(!t->static_search&&!t->radar_only&&!radar_priority&&!identity&&f&&f->epoch&&f->epoch!=t->epoch){od_state(t,"WAIT_IDENTITY","CAMERA_EPOCH_CHANGED","相机采集身份改变，保留原任务记忆，不自动绑定其他物体");return 1;}
 if(t->mode==OBJECT_MODE_STATIC&&t->near_hold){
  t->near_latch=1;t->near_latch_us=t->visual_dock.hold_source_us;t->arrival_count=t->visual_dock.confirmations;
  if(t->visual_dock.confirmed){t->arrived=1;t->active=0;od_state(t,"ARRIVED",t->near_metric?"STATIC_VISUAL_SURFACE_ARRIVAL":"STATIC_VISUAL_COARSE_DOCKED",t->near_metric?"新的目标表面距离确认已到达，保持停止；不要求雷达回波":"视觉近距复核完成，保持停止；距离为粗估，未宣称精确20厘米");return 1;}
  od_state(t,"VERIFY_ARRIVAL",t->near_reason,t->near_metric?"当前目标表面已到停靠范围，立即停止并等待新测量复核":"视觉判断目标已很近，立即停止；保留原目标，不因裁切丢框搜索或沿旧路径前进");goto done;
 }
 int fresh_radar=t->mode==OBJECT_MODE_FOLLOW&&t->observation_v2.valid&&(t->observation_v2.kind==OBJECT_MEASURE_RADAR_VISUAL||t->observation_v2.kind==OBJECT_MEASURE_RADAR_STEREO||t->observation_v2.kind==OBJECT_MEASURE_RADAR_ONLY)&&t->observation_v2.binding_generation==t->binding_generation&&t->observation_v2.association_revision==t->association_revision&&od_fresh(t->observation_v2.source_us,now,OBJECT_CONTROL_MEASUREMENT_US);
 if(t->mode==OBJECT_MODE_FOLLOW){
  od_exit_status(t,binding_current?d:NULL,now,yaw,visual,fresh_radar);
  if(t->directed_active&&(visual||fresh_radar))od_directed_end(t,now,visual?"REACQUIRED_VISUAL":"RADAR_PRIORITY");
  if(!visual&&!fresh_radar){
   const object_follow_exit_v5*e=&t->follow_exit_hint;
   if(t->directed_active&&e->source_us>t->directed_source_us&&(e->kind==3||e->kind==4))od_directed_end(t,now,"WAIT_NEW_EVIDENCE");
   if(!strcmp(t->exit_hint_status,"READY_DIRECTED")&&e->source_us>t->directed_consumed_source_us&&(!t->directed_active||e->source_us>t->directed_source_us)){
    if(t->directed_active)od_directed_end(t,now,"REPLACED_BY_NEW_HINT");od_directed_start(t,e,yaw,now);
   }
   if(t->directed_active){od_directed_step(t,yaw,now,out);goto done;}
   if(t->directed_consumed_source_us&&(!strcmp(t->directed_status,"COMPLETED")||!strcmp(t->directed_status,"TIMEOUT")||!strcmp(t->directed_status,"WAIT_NEW_EVIDENCE"))){out->linear_m_s=od_follow_speed(t,now,0,1);out->angular_rad_s=0;od_state(t,"WAIT_EXIT_EVIDENCE","FOLLOW_WAIT_NEW_EXIT_EVIDENCE","旧退出动作已完成或到达执行上限，等待新的视觉、雷达或真实退出证据，不使用旧提示反复左右扫动");goto done;}
  }
 }
 int hint=0;
 if(!visual&&binding_current&&d&&d->reacquire_candidate_valid&&d->reacquire_candidate_bearing_valid&&d->reacquire_candidate_eye>=0&&d->reacquire_candidate_eye<=1&&od_fresh(d->reacquire_candidate_time_us,now,500000)&&isfinite(d->reacquire_candidate_bearing_rad)&&fabs(d->reacquire_candidate_bearing_rad)<=3.14159265358979323846){
  if(d->reacquire_candidate_time_us>t->last_hint_us){t->last_hint_us=d->reacquire_candidate_time_us;t->hint_world_yaw=od_angle(yaw+d->reacquire_candidate_bearing_rad);}hint=od_fresh(t->last_hint_us,now,500000);
 }
 if(t->mode==OBJECT_MODE_STATIC&&!visual&&t->last_visual_us&&(!memory||t->range_m<=t->standoff_m+.03)&&!(t->static_search&&!memory&&od_fresh(t->last_visual_us,now,3000000))){
  if(!t->static_lost){t->static_lost=1;t->static_lost_us=now;t->static_lost_visual_us=t->last_visual_us;t->static_search_yaw=yaw;t->static_search_stage=t->static_search_leg=t->static_search_rounds=0;t->static_search_sign=t->bearing_rad<0?-1:1;t->static_endpoint_hold_us=0;}
 }
 if(hint&&t->mode==OBJECT_MODE_FOLLOW&&fresh_radar)hint=0;
 if(hint&&t->mode==OBJECT_MODE_STATIC&&memory&&t->range_m>t->standoff_m+.03){hint=0;t->hint_suppressed_for_memory=1;}
 if(hint){
  double target=t->hint_world_yaw;
  if(t->mode==OBJECT_MODE_STATIC&&t->static_lost){static const double stages[]={24,36,48,60,72,90};double limit=stages[t->static_search_stage<6?t->static_search_stage:5]*3.14159265358979323846/180.-.025;target=od_angle(t->static_search_yaw+od_clamp(od_angle(target-t->static_search_yaw),-limit,limit));}
  double error=od_angle(target-yaw);out->angular_rad_s=fabs(error)<=OBJECT_TURN_DEADZONE_RAD?0:od_clamp(.5*error,-.10,.10);
  if(t->mode==OBJECT_MODE_FOLLOW)od_follow_speed(t,now,0,1);
  od_state(t,"CHECK_CANDIDATE","REACQUIRE_CANDIDATE_HINT_ONLY","候选重现，减缓对准核对；只小角度转向，不前进，不当作已认回，不刷新目标记忆");goto done;
 }
 if(t->mode==OBJECT_MODE_STATIC&&t->static_lost){
  static const double stages[OBJECT_STATIC_SEARCH_STAGES]={24,36,48,60,72,90};
  unsigned stage=t->static_search_stage<OBJECT_STATIC_SEARCH_STAGES?t->static_search_stage:OBJECT_STATIC_SEARCH_STAGES-1;
  double limit=stages[stage]*3.14159265358979323846/180.,turned=od_angle(yaw-t->static_search_yaw),sign=t->static_search_sign;
  /* 实际航向完成中心、首侧、另一侧再回中心后才扩大搜索，时间经过或预测帧不能推进扫描阶段。
 */
  double goal=t->static_search_leg==0?sign*(limit-.02):t->static_search_leg==1?-sign*(limit-.02):0;
  int direction=t->static_search_leg==1?-t->static_search_sign:t->static_search_sign;
  int reached=(goal-turned)*direction<=.025;
  if(reached&&!t->static_endpoint_hold_us)t->static_endpoint_hold_us=now;
  if(t->static_endpoint_hold_us){
   out->angular_rad_s=0;
   int measured=obj_turn_measured(&t->turn_measure,now);
   if(now-t->static_endpoint_hold_us<(measured?100000u:250000u)||(measured&&fabs(t->turn_measure.rate)>OBJ_YAW_SETTLED)){
    od_state(t,"SEARCHING","STATIC_ENDPOINT_SETTLING","搜索已到真实航向端点，先减稳再换向；不是任务结束，目标出现立即转为对准");goto done;
   }
   t->static_endpoint_hold_us=0;
   if(t->static_search_leg<2)++t->static_search_leg;
   else{t->static_search_leg=0;++t->static_search_rounds;if(t->static_search_stage+1<OBJECT_STATIC_SEARCH_STAGES)++t->static_search_stage;}
   stage=t->static_search_stage;limit=stages[stage]*3.14159265358979323846/180.;goal=t->static_search_leg==0?sign*(limit-.02):t->static_search_leg==1?-sign*(limit-.02):0;
  }
  double error=goal-turned;
  out->angular_rad_s=obj_turn_endpoint(error,OBJECT_STATIC_SEARCH_MAX_RATE);
  char message[256];snprintf(message,sizeof(message),"静止目标暂时丢失，固定中心左右%.0f度原地找回；完成往返才扩大，最大90度，不设总超时，不前进",stages[stage]);
  od_state(t,"SEARCHING","STATIC_BOUNDED_REACQUIRE",message);goto done;
 }

 if(t->mode==OBJECT_MODE_STATIC&&(!ob||!ob->valid)&&measured&&s->source_us>t->arrival_source_us){
  t->arrival_source_us=s->source_us;
  int arrival_source=(!strcmp(s->source,"RADAR_VISUAL")||!strcmp(s->source,"RADAR_VISUAL_STEREO"))&&od_fresh(s->source_us,now,800000)&&visual&&od_fresh(visual_us,now,800000);
  double actual_gap=fmax(0,hypot(s->target_odom_x-px,s->target_odom_y-py)-t->front_m);
  if(arrival_source&&t->range_valid&&t->range_m<=t->standoff_m+.03&&actual_gap<=t->standoff_m+.03&&fabs(t->bearing_rad)<.35)t->arrival_count++;else t->arrival_count=0;
  if(t->arrival_count>=2){t->arrived=1;t->active=0;od_state(t,"ARRIVED","STATIC_ARRIVAL_CONFIRMED","静止目标到达复核完成，保持停止，需新任务才会再次前往");return 1;}
 }

 if(t->mode==OBJECT_MODE_FOLLOW){
  uint64_t age=t->last_source_us&&t->last_source_us<=now?now-t->last_source_us:UINT64_MAX;
  if(!t->have_target||age>=OBJECT_FOLLOW_LOST_STOP_US){
   od_follow_speed(t,now,0,1);
   double error=t->bearing_valid?od_angle(t->search_center_yaw-yaw):0;
   out->angular_rad_s=fabs(error)<=OBJECT_DIRECTION_EXIT_RAD?0:od_clamp(.6*error,-OBJECT_FOLLOW_SEARCH_W,OBJECT_FOLLOW_SEARCH_W);
   od_state(t,"SEARCHING",t->have_target?"FOLLOW_TARGET_LOST_STOP_OLD_POSITION":"FOLLOW_WAIT_DISTANCE_MEASUREMENT",t->have_target?"跟随目标超过5秒无新距离，停止追逐旧位置，保留任务并缓慢找回原目标":"跟随模式尚无实测距离，保留目标并缓慢对准，等待实测距离后按设定距离跟随");goto done;
  }
  /* 位置预测最多1秒，之后不再继续向前外推旧位置。 */
  if(age>OBJECT_FOLLOW_PREDICT_US){t->range_m=fmax(0,hypot(t->target_x-px,t->target_y-py)-t->front_m);t->bearing_rad=od_angle(atan2(t->target_y-py,t->target_x-px)-yaw);t->range_valid=1;}
  double lower=t->standoff_m-OBJECT_FOLLOW_HOLD_HALF_M,upper=t->standoff_m+OBJECT_FOLLOW_HOLD_HALF_M;
  double trail_bearing=od_follow_target_bearing(t,px,py,yaw,now);double bearing=t->current_visual_valid?od_angle(t->visual_world_yaw-yaw):trail_bearing;t->bearing_rad=bearing;t->bearing_valid=1;
  double error=od_direction(t,yaw,bearing,now,t->current_visual_valid?t->current_visual_us:t->last_source_us,t->current_visual_valid?"VISUAL":age<=OBJECT_FOLLOW_PREDICT_US?"RADAR_TRAIL":"MEMORY_TRAIL",t->current_visual_valid);t->bearing_rad=t->direction_filtered_rad;
  out->angular_rad_s=od_clamp(.8*error,-OBJECT_FOLLOW_NORMAL_W,OBJECT_FOLLOW_NORMAL_W);
  if(t->range_m<=upper){
   t->waiting_distance=1;if(t->follow_trace_count)t->follow_trace_cursor=t->follow_trace_count-1;out->linear_m_s=od_follow_speed(t,now,0,t->range_m<lower);out->angular_rad_s=0;
   od_state(t,"WAIT_DISTANCE",t->range_m<lower?"FOLLOW_TARGET_TOO_CLOSE_STOP_NO_REVERSE":"FOLLOW_HOLD_DISTANCE",t->range_m<lower?"目标主动靠近，仅停车等待，不自动后退":"目标在期望距离前后0.1米保持区内，平滑停止，目标拉开后自动跟上");goto done;
  }
  t->waiting_distance=0;
  double direction=atan2(t->target_y-py,t->target_x-px),ff=t->target_vx*cos(direction)+t->target_vy*sin(direction);
  double desired=od_clamp(.65*(t->range_m-upper)+ff,0,.25);
  if(t->range_m<upper+.25)desired=fmin(.20,desired);
  if(age>OBJECT_FOLLOW_PREDICT_US)desired*=od_clamp((OBJECT_FOLLOW_LOST_STOP_US-age)/(double)(OBJECT_FOLLOW_LOST_STOP_US-OBJECT_FOLLOW_PREDICT_US),0,1);
  if(fabs(t->direction_filtered_rad)>.60)desired=0;
  out->linear_m_s=od_follow_speed(t,now,desired,0);
  od_state(t,"FOLLOWING",age>OBJECT_FOLLOW_PREDICT_US?"FOLLOW_LOSS_DECELERATING":"FOLLOW_MEASURED_TRAIL",age>OBJECT_FOLLOW_PREDICT_US?"目标短时丢失，沿最近实测轨迹逐步减速；超过5秒停止前进并找回":"按目标距离与速度连续跟随，持续更新当前目标停靠点，R8.1规划路径与绕障");goto done;
 }

 if(t->mode==OBJECT_MODE_STATIC&&!t->have_target&&!t->bearing_valid){od_state(t,"WAIT_DIRECTION","STATIC_BOUND_DIRECTION_UNAVAILABLE","真实检测已绑定，当前没有可用方向或距离；等待实际数值，不编造目标位置");goto done;}
 if(t->mode==OBJECT_MODE_STATIC&&t->range_valid&&t->range_m<=t->standoff_m+.03){od_state(t,"VERIFY_ARRIVAL","WAIT_NEW_RANGE_CONFIRMATION","已接近静止目标，等待新的独立距离复核");goto done;}
 if((!memory&&(!t->bearing_valid||(!od_fresh(t->last_visual_us,now,3000000)&&!od_fresh(t->last_source_us,now,1500000))))||(!visual&&memory&&t->range_m<=t->standoff_m+.03)){if(!t->fallback_active){t->fallback_active=1;t->fallback_leg=0;t->fallback_sign=t->bearing_rad<0?-1:1;t->fallback_center_yaw=t->search_center_yaw;t->fallback_hold_us=0;}
  double span=24*3.14159265358979323846/180.,goal=t->fallback_center_yaw+(t->fallback_leg?t->fallback_sign*-span:t->fallback_sign*span),error=od_angle(goal-yaw);
  int direction=t->fallback_leg?-t->fallback_sign:t->fallback_sign;
  if(error*direction<=.025&&!t->fallback_hold_us)t->fallback_hold_us=now;
  if(t->fallback_hold_us){out->angular_rad_s=0;int measured=obj_turn_measured(&t->turn_measure,now);if(now-t->fallback_hold_us>=(measured?100000u:250000u)&&(!measured||fabs(t->turn_measure.rate)<=OBJ_YAW_SETTLED)){t->fallback_leg^=1;t->fallback_hold_us=0;}}
  else out->angular_rad_s=obj_turn_endpoint(error,OBJECT_STATIC_SEARCH_MAX_RATE);od_state(t,"SEARCHING","TARGET_EVIDENCE_LOST_SEARCH","目标暂时不可确认，停止盲目前进，朝记忆方向搜索，找回后自动继续");goto done;}
 double bearing_error=od_direction(t,yaw,t->bearing_rad,now,t->current_visual_valid?t->current_visual_us:memory?t->last_source_us:t->last_visual_us,t->current_visual_valid?"VISUAL":memory?"STATIC_MEMORY":"STATIC_BEARING_MEMORY",t->current_visual_valid);t->bearing_rad=t->direction_filtered_rad;
 out->angular_rad_s=t->mode==OBJECT_MODE_STATIC?od_clamp(.8*bearing_error,-OBJECT_NORMAL_TURN_MAX_RATE,OBJECT_NORMAL_TURN_MAX_RATE):od_clamp(2*t->bearing_rad,-OBJECT_FOLLOW_NORMAL_W,OBJECT_FOLLOW_NORMAL_W);
 if(fabs(t->bearing_rad)>.85){out->linear_m_s=0;out->angular_rad_s=copysign(t->mode==OBJECT_MODE_STATIC?OBJECT_NORMAL_TURN_MAX_RATE:OBJECT_FOLLOW_NORMAL_W,t->bearing_rad);}else out->linear_m_s=(t->range_valid&&t->range_m<.5?.20:.25)*(t->mode==OBJECT_MODE_STATIC?1:fmax(.25,cos(t->bearing_rad)));
 if(t->mode==OBJECT_MODE_STATIC)od_state(t,"APPROACHING",visual?"STATIC_TARGET_APPROACH":"STATIC_MEMORY_APPROACH","前往固定物体位置；R8负责绕障、转向及执行检查");else od_state(t,"FOLLOWING",visual?"FOLLOW_VISIBLE_TARGET":"FOLLOW_TARGET_PREDICTION","持续跟随同一物体；短时遮挡用目标轨迹接续");
 done:
 if(!strcmp(t->state,"SEARCHING")){snprintf(t->direction_source,sizeof(t->direction_source),"SEARCH");t->direction_source_us=t->last_visual_us?t->last_visual_us:t->last_source_us;}
 else if(!strcmp(t->state,"CHECK_CANDIDATE")){snprintf(t->direction_source,sizeof(t->direction_source),"CANDIDATE");t->direction_source_us=t->last_hint_us;}
 if(!t->direction_log_us||now-t->direction_log_us>=250000){fprintf(stderr,"OBJECT_DIRECTION mode=%u epoch=%llu canonical=%d binding=%u task_class=%d source=%s source_us=%llu raw_rad=%.6f filtered_rad=%.6f turning=%d requested_v=%.5f requested_w=%.5f association_revision=%u association_corrections=%u exit_used=%d exit_status=%s velocity_jump=%d velocity_jumps=%u target_vx=%.5f target_vy=%.5f hint_kept_memory=%d state=%s\n",t->mode,(unsigned long long)t->epoch,t->track_id,t->binding_generation,t->task_class_id,t->direction_source,(unsigned long long)t->direction_source_us,t->direction_raw_rad,t->direction_filtered_rad,t->direction_turning,out->linear_m_s,out->angular_rad_s,t->association_revision,t->association_corrections,t->exit_hint_used,t->exit_hint_status,t->velocity_jump_suppressed,t->velocity_jump_count,t->target_vx,t->target_vy,t->hint_suppressed_for_memory,t->state);t->direction_log_us=now;}
 if(ignored_static&&!t->near_latch)od_state(t,t->state,"STATIC_CANDIDATE_DISAGREES_KEEP_ORIGINAL_SOURCE","新位置偏离已锁定静止目标，保留原位置和原观测时间，不冒充新测距");t->requested_v=out->linear_m_s;t->requested_w=out->angular_rad_s;return 1;
}
int object_dual_task_json(const object_dual_task*t,uint64_t now,char*out,size_t cap,size_t*written){if(!t||!out||!written)return 0;char range[40]="null",bearing[40]="null";if(t->range_valid&&isfinite(t->range_m))snprintf(range,sizeof(range),"%.9g",t->range_m);if(t->bearing_valid&&isfinite(t->bearing_rad))snprintf(bearing,sizeof(bearing),"%.9g",t->bearing_rad);int n=snprintf(out,cap,"{\"version\":1,\"published_us\":\"%llu\",\"request_id\":\"%llu\",\"epoch\":\"%llu\",\"track_id\":%d,\"mode\":\"%s\",\"state\":\"%s\",\"reason\":\"%s\",\"message_zh\":\"%s\",\"prepared\":%s,\"active\":%s,\"explicit_start\":%s,\"arrived\":%s,\"standoff_m\":%.9g,\"range_m\":%s,\"bearing_rad\":%s,\"source\":\"%s\",\"source_us\":\"%llu\",\"requested_v\":%.9g,\"requested_w\":%.9g,\"arrival_count\":%u}",(unsigned long long)now,(unsigned long long)t->request_id,(unsigned long long)t->epoch,t->track_id,t->mode==2?"follow":"static",t->state,t->reason,t->message_zh,t->prepared?"true":"false",t->active?"true":"false",t->explicit_start?"true":"false",t->arrived?"true":"false",t->standoff_m,range,bearing,t->source,(unsigned long long)t->last_source_us,t->requested_v,t->requested_w,t->arrival_count);if(n<0||(size_t)n>=cap)return 0;size_t used=(size_t)n-1;
 int radar_json=snprintf(out+used,cap-used,",\"radar_only\":%s,\"semantic_direct\":%s",t->radar_only?"true":"false",t->semantic_direct?"true":"false");if(radar_json<0||(size_t)radar_json>=cap-used)return 0;used+=(size_t)radar_json;
 int search_json=snprintf(out+used,cap-used,",\"static_search\":%s,\"search_bound\":%s,\"search_category\":\"%s\",\"static_bearing_estimated\":%s",t->static_search?"true":"false",t->search_bound?"true":"false",t->search_category,t->static_bearing_estimated?"true":"false");if(search_json<0||(size_t)search_json>=cap-used)return 0;used+=(size_t)search_json;
 int visual_json=snprintf(out+used,cap-used,",\"search_color\":\"%s\",\"search_size\":\"%s\",\"near_hold\":%s,\"near_metric\":%s,\"near_reason\":\"%s\",\"visual_docking\":{\"source\":\"%s\",\"source_us\":\"%llu\",\"frame_id\":\"%llu\",\"eye\":%d,\"growth_ratio\":%.6g,\"advance_m\":%.6g,\"clipped\":%s,\"gap_valid\":%s,\"gap_m\":%.6g,\"gap_error_m\":%.6g,\"geometry_reason\":\"%s\"}",t->search_color,t->search_size,t->near_hold?"true":"false",t->near_metric?"true":"false",t->near_reason,t->visual_dock.source,(unsigned long long)t->visual_dock.source_us,(unsigned long long)t->visual_dock.frame_id,t->visual_dock.eye,t->visual_dock.growth_ratio,t->visual_dock.advance_m,t->visual_dock.clipped?"true":"false",t->visual_dock.gap_valid?"true":"false",t->visual_dock.gap_m,t->visual_dock.gap_error_m,t->visual_geometry.reason);if(visual_json<0||(size_t)visual_json>=cap-used)return 0;used+=(size_t)visual_json;
 int probe_json=snprintf(out+used,cap-used,",\"probe_active\":%s,\"probe_track_id\":%d,\"probe_epoch\":\"%llu\",\"probe_frame_id\":\"%llu\",\"probe_source_us\":\"%llu\",\"probe_bearing_rad\":%.6g,\"probe_reason\":\"%s\"",t->probe_active?"true":"false",t->probe_track_id,(unsigned long long)t->probe_epoch,(unsigned long long)t->probe_frame_id,(unsigned long long)t->probe_source_us,t->probe_bearing_rad,t->probe_reason);if(probe_json<0||(size_t)probe_json>=cap-used)return 0;used+=(size_t)probe_json;
 int candidate_json=snprintf(out+used,cap-used,",\"search_candidates\":{\"class\":%u,\"color\":%u,\"metric_size\":%u}",t->search_class_candidates,t->search_color_candidates,t->search_metric_candidates);if(candidate_json<0||(size_t)candidate_json>=cap-used)return 0;used+=(size_t)candidate_json;
 int revoke_json=snprintf(out+used,cap-used,",\"static_range_revoked_scan_us\":\"%llu\"",(unsigned long long)t->static_range_revoked_us);if(revoke_json<0||(size_t)revoke_json>=cap-used)return 0;used+=(size_t)revoke_json;
 int detail=snprintf(out+used,cap-used,",\"task_class_id\":%d,\"binding_generation\":%u,\"direction_source\":\"%s\",\"direction_source_us\":\"%llu\",\"direction_raw_rad\":%.9g,\"direction_filtered_rad\":%.9g,\"direction_turning\":%s,\"velocity_jump_suppressed\":%s,\"velocity_jump_count\":%u,\"target_velocity_m_s\":[%.9g,%.9g],\"hint_kept_memory\":%s",t->task_class_id,t->binding_generation,t->direction_source,(unsigned long long)t->direction_source_us,t->direction_raw_rad,t->direction_filtered_rad,t->direction_turning?"true":"false",t->velocity_jump_suppressed?"true":"false",t->velocity_jump_count,t->target_vx,t->target_vy,t->hint_suppressed_for_memory?"true":"false");if(detail<0||(size_t)detail>=cap-used)return 0;used+=(size_t)detail;
 char exit_age[32]="null",exit_bearing[40]="null",exit_rate[40]="null";const object_follow_exit_v5*exit=&t->follow_exit_hint;
 if(exit->source_us&&exit->source_us<=now)snprintf(exit_age,sizeof(exit_age),"%llu",(unsigned long long)((now-exit->source_us)/1000));
 if(exit->bearing_valid&&isfinite(exit->bearing_rad))snprintf(exit_bearing,sizeof(exit_bearing),"%.9g",exit->bearing_rad);
 if(isfinite(exit->angular_velocity_rad_s))snprintf(exit_rate,sizeof(exit_rate),"%.9g",exit->angular_velocity_rad_s);
 int extra=snprintf(out+used,cap-used,",\"association_revision\":%u,\"association_corrections\":%u,\"association_source_us\":\"%llu\",\"association_reason\":\"%s\",\"follow_exit\":{\"valid\":%s,\"used\":%s,\"compensated\":%s,\"kind\":%d,\"direction\":%d,\"source_us\":\"%llu\",\"frame_id\":\"%llu\",\"age_ms\":%s,\"bearing_rad\":%s,\"angular_velocity_rad_s\":%s,\"reason\":\"%s\",\"status\":\"%s\",\"keyframe_count\":%u,\"keyframes\":[",t->association_revision,t->association_corrections,(unsigned long long)t->association_source_us,t->association_reason,exit->valid?"true":"false",t->exit_hint_used?"true":"false",exit->compensated?"true":"false",exit->kind,exit->direction,(unsigned long long)exit->source_us,(unsigned long long)exit->frame_id,exit_age,exit_bearing,exit_rate,exit->reason,t->exit_hint_status,exit->count<OBJECT_FOLLOW_KEYFRAMES_V5_MAX?exit->count:OBJECT_FOLLOW_KEYFRAMES_V5_MAX);if(extra<0||(size_t)extra>=cap-used)return 0;used+=(size_t)extra;
 for(unsigned k=0;k<exit->count&&k<OBJECT_FOLLOW_KEYFRAMES_V5_MAX;k++){const object_follow_keyframe_v5*q=exit->keyframes+k;char bearing_text[40]="null";if(q->bearing_valid&&isfinite(q->bearing_rad))snprintf(bearing_text,sizeof(bearing_text),"%.9g",q->bearing_rad);extra=snprintf(out+used,cap-used,"%s{\"frame_id\":\"%llu\",\"source_us\":\"%llu\",\"eye\":%d,\"bbox\":[%.9g,%.9g,%.9g,%.9g],\"bearing_rad\":%s}",k?",":"",(unsigned long long)q->frame_id,(unsigned long long)q->source_us,q->eye,q->bbox[0],q->bbox[1],q->bbox[2],q->bbox[3],bearing_text);if(extra<0||(size_t)extra>=cap-used)return 0;used+=(size_t)extra;}
 extra=snprintf(out+used,cap-used,"]}");if(extra<0||(size_t)extra>=cap-used)return 0;used+=(size_t)extra;
 extra=snprintf(out+used,cap-used,",\"target_measurement_sparse\":%s,\"target_measurement_support_points\":%u,\"directed_turn\":{\"active\":%s,\"status\":\"%s\",\"source_us\":\"%llu\",\"frame_id\":\"%llu\",\"started_us\":\"%llu\",\"elapsed_ms\":%llu,\"timeout_ms\":6000,\"sign\":%d,\"start_yaw_rad\":%.9g,\"goal_yaw_rad\":%.9g,\"target_angle_rad\":%.9g,\"progress_rad\":%.9g,\"remaining_rad\":%.9g}",t->observation_v2.sparse?"true":"false",t->observation_v2.support_points,t->directed_active?"true":"false",t->directed_status,(unsigned long long)t->directed_source_us,(unsigned long long)t->directed_frame_id,(unsigned long long)t->directed_started_us,(unsigned long long)(t->directed_started_us&&now>=t->directed_started_us?((t->directed_active?now:t->directed_ended_us)-t->directed_started_us)/1000:0),t->directed_sign,t->directed_start_yaw,t->directed_goal_yaw,OBJECT_FOLLOW_DIRECTED_ANGLE_V6,t->directed_progress_rad,fmax(0,OBJECT_FOLLOW_DIRECTED_ANGLE_V6-t->directed_progress_rad));if(extra<0||(size_t)extra>=cap-used)return 0;used+=(size_t)extra;
 int z=snprintf(out+used,cap-used,",\"follow_trace_frame\":\"ODOM\",\"follow_trace_cursor\":%u,\"follow_trace\":[",t->follow_trace_cursor);if(z<0||(size_t)z>=cap-used)return 0;used+=(size_t)z;
 for(unsigned k=0;k<t->follow_trace_count;k++){const object_dual_trace*q=t->follow_trace+k;z=snprintf(out+used,cap-used,"%s[%.6f,%.6f,\"%llu\"]",k?",":"",q->x,q->y,(unsigned long long)q->source_us);if(z<0||(size_t)z>=cap-used)return 0;used+=(size_t)z;}
 z=snprintf(out+used,cap-used,"]}");if(z<0||(size_t)z>=cap-used)return 0;used+=(size_t)z;*written=used;return 1;}

/* 控制器与路线显示共用同一份 ODOM 目标状态。 */
int object_dual_task_route_v2(const object_dual_task*t,const lab_real_runtime_view*v,uint64_t now,object_route_target_v2*out){
 if(!t||!v||!out)return 0;memset(out,0,sizeof(*out));out->context=v->context;out->mode=t->mode;out->epoch=t->epoch;out->track_id=t->track_id;out->generation=t->generation;out->source_us=t->last_source_us;out->published_us=now;out->standoff_m=t->standoff_m;out->visual_bearing_valid=t->current_visual_valid;out->visual_source_us=t->current_visual_us;out->visual_bearing_rad=t->current_visual_valid?od_angle(t->visual_world_yaw-v->raw_odometry.yaw_rad):0;
 out->control_bearing_valid=t->direction_initialized&&(!strcmp(t->state,"APPROACHING")||!strcmp(t->state,"FOLLOWING"));out->control_turning=t->direction_turning;out->control_bearing_rad=t->direction_filtered_rad;out->control_source_us=t->direction_source_us;
 out->zero_required=!t->active||!t->explicit_start||t->arrived||t->near_hold||t->near_latch||(fabs(t->requested_v)<1e-9&&fabs(t->requested_w)<1e-9);
 if(t->mode==OBJECT_MODE_STATIC){
  out->static_search_left=!t->near_hold&&!t->arrived&&t->static_search&&!t->search_bound;
  out->static_bearing_approach=!t->near_hold&&!t->arrived&&!t->have_target&&t->epoch&&t->track_id>0&&t->bearing_valid;
 }
 if(t->mode==OBJECT_MODE_STATIC&&t->probe_active&&!t->search_bound){
  out->epoch=t->probe_epoch;out->track_id=t->probe_track_id;out->source_us=t->probe_source_us;out->valid=out->goal_valid=out->trail_count=0;out->static_search_left=0;out->static_bearing_approach=!t->near_hold&&!t->arrived;
  out->visual_bearing_valid=t->probe_current;out->control_bearing_valid=1;out->visual_bearing_rad=out->control_bearing_rad=t->probe_bearing_rad;out->visual_source_us=out->control_source_us=t->probe_source_us;out->control_turning=fabs(t->probe_bearing_rad)>OBJECT_TURN_DEADZONE_RAD;return 1;
 }
 if(!t->have_target||!t->have_context||!lab_real_context_equal(&t->context,&v->context)||!v->has_odometry)return 1;
 if(t->mode==OBJECT_MODE_FOLLOW&&!od_fresh(t->last_source_us,now,OBJECT_FOLLOW_LOST_STOP_US))return 1;
 out->valid=out->goal_valid=1;out->target_x_m=t->target_x;out->target_y_m=t->target_y;out->error_m=t->observation_v2.valid?t->observation_v2.error_m:t->visual_dock.gap_valid?t->visual_dock.gap_error_m:0;
 double px=v->raw_odometry.x_m,py=v->raw_odometry.y_m,dx=t->target_x-px,dy=t->target_y-py,d=hypot(dx,dy),offset=t->front_m+t->standoff_m;
 if(t->mode==OBJECT_MODE_FOLLOW){double age=t->last_source_us<=now?(now-t->last_source_us)/1e6:0;double predict=fmin(OBJECT_FOLLOW_PREDICT_US/1e6,age);dx+=t->target_vx*predict;dy+=t->target_vy*predict;d=hypot(dx,dy);}
 if(t->mode==OBJECT_MODE_FOLLOW&&t->current_visual_valid){double heading=v->raw_odometry.yaw_rad+t->bearing_rad;dx=d*cos(heading);dy=d*sin(heading);}
 double ratio=d>offset?(d-offset)/d:0;out->goal_x_m=px+dx*ratio;out->goal_y_m=py+dy*ratio;
 if(t->mode==OBJECT_MODE_FOLLOW){
  for(unsigned k=t->follow_trace_cursor;k<t->follow_trace_count&&out->trail_count<OBJECT_ROUTE_V2_MAX;k++){const object_dual_trace*q=t->follow_trace+k;out->trail[out->trail_count++]=(object_route_point_v2){q->x,q->y,q->source_us};}
  /* 目标轨迹仅作诊断证据，R8.1规划到当前目标前截断终点的路线，旧物体测量点不直接成为车辆目标。
 */
 }
 return 1;
}
