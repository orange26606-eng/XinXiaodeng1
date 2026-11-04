#define _POSIX_C_SOURCE 200809L
#include "lab_real_session.h"
#include "live_nav_ui.h"
#include "phone_nav_snapshot.h"
#include "phone_goal_ipc.h"
#include "lab_real_runtime_control.h"
#include "phone_goal_validation.h"
#include "object_request_admission.h"
#include "phone_goal_socket.h"
#include "phone_nav_lease.h"
#include "tag_goal.h"
#include "object_readonly.h"
#include "object_navigation.h"
#include "object_dual_task.h"
#include "object_radar_follow.h"
#include "object_time_policy.h"
#include "object_measurement_mailbox.h"
#include "../common/scan_motion_comp.h"
#include "passage.h"
#include <math.h>
#include "lab_screen.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "continuous_policy.h"
#include "navigation_display_policy.h"
#include "navigation_resume_policy.h"
#include "navigation_status.h"
#include <unistd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <pthread.h>
#include "phone_control_fence.h"
#define S5_LOG_SOFTWARE_POLICY 9u
#include "s5_persistent_log.h"
#include "s5_status_text.h"
typedef struct map_exporter map_exporter;
typedef struct panel{obj_radar_follow radar_follow;object_dual_task dual_task;obj_scene dual_scene,dual_display_scene;obj_frame dual_frame;r8_deskew_history dual_history;uint64_t dual_prepare_origin,dual_last_export,dual_last_plan,dual_yaw_us;double dual_last_yaw,dual_measured_w;int dual_measured_w_valid;int dual_camera_compensation;double dual_camera_error;uint64_t dual_camera_source_us;char dual_camera_reason[48];struct scan_frame *dual_owned_scan;s5_log*s5log;uint64_t s5_last_log;char s5_last_reason[64],s5_last_avoid[48];uint32_t map_export_stalled;char s6_last_planner[64];uint32_t s6_last_active;uint64_t s6_last_mismatch;obj_lidar_state*object_lidar;object_navigation object_task;obj_map_state object_map;int cruise_mode,auto_pending;phone_goal_message auto_request;uint64_t auto_request_us,last_auto_generation,last_mode_export,last_object_export;char auto_status[24],auto_reason[96];map_exporter*exporter;phone_nav_snapshot_state snapshot_cache;tag_task tag;lab_real_session session;live_nav_ui ui;phone_nav_lease lease;uint64_t start,last_tick,last_log,arm_deadline,last_map_export,last_start_sent_us;uint64_t request_id,request_received_us,accepted_goal_origin_us,arm_request_id;unsigned request_operation;char request_state[16],request_reason[128];uint32_t goal_id;int done,arm_request,goal_fd,navigation_screen;char goal_path[sizeof(((struct sockaddr_un*)0)->sun_path)];}panel;
static int cruise_selected(const panel*p){return p->cruise_mode;}
/* 只读假设局部目标供START前检查候选几何，创建它本身不接管手机或电机。
 */
static void cruise_preview(panel*p,uint64_t now){
 if(!cruise_selected(p)||p->session.view.has_goal||!p->session.view.has_localization)return;
 const lab_pose*q=&p->session.view.localization.pose;
 lab_real_goal g={.map_id=p->session.view.localization.map.map_id,.id=++p->goal_id,.x_m=q->x_m+cos(q->yaw_rad),.y_m=q->y_m+sin(q->yaw_rad)};
 if(lab_real_runtime_set_goal(p->session.runtime,&g,now)==LAB_OK){uint32_t exp=p->session.view.experimental_timing;lab_real_runtime_view_get(p->session.runtime,&p->session.view);p->session.view.experimental_timing=exp;snprintf(p->ui.message,sizeof(p->ui.message),"CRUISE_PREVIEW_EXPLICIT_PHONE_START_REQUIRED");}
}
static uint64_t monotonic_at_least(uint64_t hint){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return hint;uint64_t now=(uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;return now>hint?now:hint;}
#include "dual_panel_bridge.inc"
static const char*request_operation_name(unsigned op){switch(op){case PHONE_NAV_GOAL:return "goal";case PHONE_NAV_START:return "start";case PHONE_NAV_CANCEL:return "cancel";case PHONE_NAV_RESET_MAP:return "reset";case PHONE_NAV_TAG_GOAL:return "tag";case PHONE_NAV_OBJECT_GOAL:return "object";case PHONE_NAV_AUTO_START:return "auto";case PHONE_NAV_OBJECT_PREPARE:return "object_prepare";case PHONE_NAV_OBJECT_START:return "object_start";case PHONE_NAV_OBJECT_CANCEL:return "object_cancel";case PHONE_NAV_OBJECT_PAUSE:return "object_pause";case PHONE_NAV_OBJECT_RESUME:return "object_resume";default:return "none";}}
static void request_result(panel*p,const phone_goal_message*m,uint64_t now,const char*state,const char*reason){
 if(m){p->request_id=m->request_id?m->request_id:m->sequence;p->request_operation=m->operation;p->request_received_us=now;}
 snprintf(p->request_state,sizeof(p->request_state),"%.15s",state);
 /* 诊断只接受有长度上限的标识。 */
 unsigned j=0;for(;reason[j]&&j<sizeof(p->request_reason)-1;j++){unsigned char c=(unsigned char)reason[j];p->request_reason[j]=(c=='"'||c=='\\'||c<32)?'_':(char)c;}p->request_reason[j]=0;
 p->last_map_export=0;
 fprintf(stderr,"NAV_REQUEST_RESULT id=%llu operation=%s state=%s reason=%s received_us=%llu\n",(unsigned long long)p->request_id,request_operation_name(p->request_operation),p->request_state,p->request_reason,(unsigned long long)p->request_received_us);
}
static void arm_result(panel*p,uint64_t now,const char*state,const char*reason){
 if((p->request_operation==PHONE_NAV_START||p->request_operation==PHONE_NAV_OBJECT_START||p->request_operation==PHONE_NAV_OBJECT_RESUME)&&p->request_id==p->arm_request_id)request_result(p,NULL,now,state,reason);
 else fprintf(stderr,"NAV_ARM_STATE id=%llu state=%s reason=%s now_us=%llu\n",(unsigned long long)p->arm_request_id,state,reason,(unsigned long long)now);
}
static const char*start_blocker(const panel*p,uint64_t now){
 if(!p->session.options.enable_drive)return "PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE";
 if(!p->session.options.passage.mount_verified)return "LIDAR_OFFSET_NOT_VERIFIED";
 if(!p->session.view.has_goal)return "SELECT_AND_PREVIEW_GOAL_FIRST";
 if(p->session.view.enabled||p->session.view.pending_enable||p->arm_request||p->auto_pending)return "NAVIGATION_ALREADY_REQUESTED";
 if(!p->lease.claimed||now<p->lease.last_seen_us||now-p->lease.last_seen_us>=CONT_PHONE_US)return "PHONE_SESSION_NOT_OWNED_OR_EXPIRED";
 return "NONE";
}
static const char*motion_blocker(const panel*p){
 if(!p->session.options.enable_drive)return "PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE";
 if(p->dual_task.active){if(!p->session.view.has_odometry)return "WAIT_FRESH_RAW_ODOMETRY";if(!p->session.view.has_scan)return "WAIT_FRESH_LOCAL_SCAN";if(!navigation_feedback_fresh(&p->session.view))return "WAIT_FRESH_WHEEL_FEEDBACK";return p->session.view.reason;}
 if(!navigation_pose_valid(&p->session.view))return "WAIT_FRESH_LOCALIZATION";
 if(!navigation_feedback_fresh(&p->session.view))return "WAIT_FRESH_WHEEL_FEEDBACK";
 if(p->session.view.feedback.faults||p->session.view.resume_requires_start)return "FINAL_MOTOR_FAULT_WAIT_FRESH_HEALTHY_FEEDBACK";
 if(!p->session.view.navigation.path_count&&!p->session.view.navigation.can_start_local)return "WAIT_CURRENT_FEASIBLE_ACTION_CONTINUE_EXPLORING";
 return "NONE";
}
/* 由事件循环调用，地图指针在此有效；原子替换避免HTTP端读到半写入快照。
 */
static int phone_export_map_sync(panel*p,uint64_t now){
 dual_scene_worker(p,now);
 if(p->last_map_export && now-p->last_map_export<500000)return 0;
 p->last_map_export=now;const char*dir=getenv("ACG_PHONE_DIR");if(!dir||dir[0]!='/')return 0;
 char dst[512],tmp[512];if(snprintf(dst,sizeof(dst),"%s/navigation-map.json",dir)>=(int)sizeof(dst)||snprintf(tmp,sizeof(tmp),"%s/navigation-map.%ld.tmp",dir,(long)getpid())>=(int)sizeof(tmp))return 0;
 size_t cap=2*1024*1024,n=0;char*json=malloc(cap);if(!json)return 0;
 if(!phone_nav_snapshot_json_cached(&p->snapshot_cache,&p->session.view,json,cap,&n)){free(json);return 0;}
 char status[sizeof(p->ui.message)];for(size_t i=0;i<sizeof(status);i++){unsigned char c=(unsigned char)p->ui.message[i];status[i]=(c=='"'||c=='\\'||c<32)?(c?'_':0):(char)c;if(!c)break;}status[sizeof(status)-1]=0;
 if(n==0||json[n-1]!='}'||n+strlen(status)+1200>=cap){free(json);return 0;}int extra=snprintf(json+n-1,cap-n+1,",\"continuous_test\":true,\"goal_adjusted\":%s,\"target_observation_pending\":%s,\"requested_goal\":[%.9g,%.9g],\"nav_usable\":%s,\"odometry_fallback\":%s,\"drive_capable\":%s,\"calibrated\":%s,\"goal_status\":\"%s\"}",p->ui.goal_adjusted?"true":"false",p->ui.target_observation_pending&&p->session.view.has_goal?"true":"false",p->ui.requested_x,p->ui.requested_y,navigation_pose_valid(&p->session.view)?"true":"false",p->session.view.localization.odometry_fallback?"true":"false",p->session.options.enable_drive?"true":"false",p->session.view.localization.pose.calibrated?"true":"false",status);if(extra<0||(size_t)extra>=cap-n+1){free(json);return 0;}n=n-1+(size_t)extra;
 extra=snprintf(json+n-1,cap-n+1,",\"vision_target\":{\"active\":%s,\"tag_id\":%d,\"state\":\"%s\",\"reason\":\"%s\",\"standoff_m\":%.6g,\"target_x\":%.9g,\"target_y\":%.9g,\"goal_x\":%.9g,\"goal_y\":%.9g,\"frame_time_us\":%llu,\"request_us\":\"%llu\"}}",p->tag.active?"true":"false",p->tag.id,p->tag.state,p->tag.reason,p->tag.standoff,p->tag.target_x,p->tag.target_y,p->tag.goal_x,p->tag.goal_y,(unsigned long long)p->tag.frame_us,(unsigned long long)p->tag.request_us);
 if(extra<0||(size_t)extra>=cap-n+1){free(json);return 0;}n=n-1+(size_t)extra;
 char object_tx[32]="null",object_ty[32]="null",object_gx[32]="null",object_gy[32]="null";
 if(p->object_task.frame_us){snprintf(object_tx,sizeof(object_tx),"%.9g",p->object_task.target_x);snprintf(object_ty,sizeof(object_ty),"%.9g",p->object_task.target_y);snprintf(object_gx,sizeof(object_gx),"%.9g",p->object_task.goal_x);snprintf(object_gy,sizeof(object_gy),"%.9g",p->object_task.goal_y);}
 extra=snprintf(json+n-1,cap-n+1,",\"object_navigation\":{\"active\":%s,\"state\":\"%s\",\"reason\":\"%s\",\"epoch\":\"%llu\",\"track_id\":%d,\"association_id\":\"%s\",\"request_us\":\"%llu\",\"target_x\":%s,\"target_y\":%s,\"goal_x\":%s,\"goal_y\":%s,\"standoff_m\":%.9g}}",p->object_task.active?"true":"false",p->object_task.state,p->object_task.reason,(unsigned long long)p->object_task.epoch,p->object_task.track_id,p->object_task.association_id,(unsigned long long)p->object_task.request_us,object_tx,object_ty,object_gx,object_gy,p->object_task.standoff_m);
 if(extra<0||(size_t)extra>=cap-n+1){free(json);return 0;}n=n-1+(size_t)extra;
 int auto_active=p->cruise_mode&&!p->auto_pending&&p->lease.motion_active;
 int auto_intent=p->cruise_mode&&p->lease.motion_active;
 extra=snprintf(json+n-1,cap-n+1,",\"control_mode\":\"%s\",\"auto_active\":%s,\"auto_intent_accepted\":%s,\"auto_waiting\":%s,\"extended_reverse_attempts\":%u,\"extended_reverse_active\":%s}",auto_intent?"AUTO":p->lease.motion_active?"NAV":"MANUAL",auto_active?"true":"false",auto_intent?"true":"false",auto_intent&&p->auto_pending?"true":"false",p->session.view.navigation.extended_reverse_attempts,p->session.view.navigation.extended_reverse_active?"true":"false");
 if(extra<0||(size_t)extra>=cap-n+1){free(json);return 0;}n=n-1+(size_t)extra;
 const char*block=start_blocker(p,now),*motion=motion_blocker(p);
 extra=snprintf(json+n-1,cap-n+1,",\"nav_request\":{\"id\":\"%llu\",\"operation\":\"%s\",\"state\":\"%s\",\"reason\":\"%s\",\"received_us\":\"%llu\"},\"start_allowed\":%s,\"start_blocker\":\"%s\",\"motion_ready\":%s,\"motion_blocker\":\"%s\",\"start_evaluated_us\":\"%llu\"}",(unsigned long long)p->request_id,request_operation_name(p->request_operation),p->request_state,p->request_reason,(unsigned long long)p->request_received_us,!strcmp(block,"NONE")?"true":"false",block,!strcmp(motion,"NONE")?"true":"false",motion,(unsigned long long)now);
 if(extra<0||(size_t)extra>=cap-n+1){free(json);return 0;}n=n-1+(size_t)extra;
 int fd=open(tmp,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0){free(json);return 0;}
 size_t done=0;while(done<n){ssize_t k=write(fd,json+done,n-done);if(k<=0)break;done+=(size_t)k;}
 int bad=close(fd),ok=0;if(done==n&&!bad){if(rename(tmp,dst))unlink(tmp);else ok=1;}else unlink(tmp);free(json);return ok;
}
/* 导出使用不可变最新待处理副本，JSON、文件写入和缓存由单一工作线程处理，控制循环只深复制有界数据。
 */
struct map_exporter {obj_radar_follow radar_cache;uint64_t radar_prepare_origin,published_prepare_origin;object_measurement_mailbox measurements;pthread_t thread;pthread_mutex_t mutex;pthread_cond_t ready;int stop;panel*pending;phone_nav_snapshot_state cache;uint64_t replaced,exports,last_success_us,started_us;uint32_t failures;obj_scene scene_cache,published_scene,published_display;obj_frame published_frame;uint64_t last_plan,last_map_attempt;};
static void export_job_free(panel*j){if(!j)return;free((void*)j->session.view.localization.map.cells);free((void*)j->session.view.localization.map.trace);free((void*)j->session.view.navigation.path);free((void*)j->session.view.navigation.local_path);free(j->dual_owned_scan);free(j);}
static panel*export_job_copy(const panel*p){const lab_real_grid_view*m=&p->session.view.localization.map;if((!p->dual_task.prepared&&!lab_real_grid_valid(m))||m->width>1000||m->height>1000||m->trace_count>LAB_TRACE_MAX||(m->trace_count&&!m->trace)||p->session.view.navigation.path_count>LAB_REAL_PATH_MAX||(p->session.view.navigation.path_count&&!p->session.view.navigation.path))return NULL;
 if(p->session.view.navigation.local_path_count>33||(p->session.view.navigation.local_path_count&&!p->session.view.navigation.local_path))return NULL;
 panel*j=malloc(sizeof(*j));if(!j)return NULL;*j=*p;j->last_map_export=0;j->session.view.localization.map.cells=NULL;j->session.view.localization.map.trace=NULL;j->session.view.navigation.path=NULL;j->session.view.navigation.local_path=NULL;j->session.view.latest_scan=NULL;j->dual_owned_scan=NULL;if(p->session.view.latest_scan){j->dual_owned_scan=malloc(sizeof(*j->dual_owned_scan));if(!j->dual_owned_scan){free(j);return NULL;}*j->dual_owned_scan=*p->session.view.latest_scan;j->session.view.latest_scan=j->dual_owned_scan;}
 size_t bytes=(size_t)m->width*m->height;void*cells=malloc(bytes?bytes:1),*trace=malloc(m->trace_count?m->trace_count*sizeof(*m->trace):1);uint32_t count=p->session.view.navigation.path_count;void*path=malloc(count?count*sizeof(*p->session.view.navigation.path):1);
 uint32_t local_count=p->session.view.navigation.local_path_count;void*local_path=malloc(local_count?local_count*sizeof(*p->session.view.navigation.local_path):1);
 if(!cells||!trace||!path||!local_path){free(cells);free(trace);free(path);free(local_path);free(j->dual_owned_scan);free(j);return NULL;}
 if(local_count)memcpy(local_path,p->session.view.navigation.local_path,local_count*sizeof(*p->session.view.navigation.local_path));j->session.view.navigation.local_path=local_path;if(bytes&&m->cells)memcpy(cells,m->cells,bytes);else if(bytes)memset(cells,-1,bytes);if(m->trace_count)memcpy(trace,m->trace,m->trace_count*sizeof(*m->trace));if(count)memcpy(path,p->session.view.navigation.path,count*sizeof(*p->session.view.navigation.path));j->session.view.localization.map.cells=cells;j->session.view.localization.map.trace=trace;j->session.view.navigation.path=path;return j;
}
static void*export_worker(void*u){map_exporter*e=u;for(;;){pthread_mutex_lock(&e->mutex);while(!e->pending&&!e->stop)pthread_cond_wait(&e->ready,&e->mutex);if(e->stop){panel*j=e->pending;e->pending=NULL;pthread_mutex_unlock(&e->mutex);export_job_free(j);return NULL;}panel*j=e->pending;e->pending=NULL;pthread_mutex_unlock(&e->mutex);
 uint64_t begin=monotonic_at_least(j->session.view.time_us);j->snapshot_cache=e->cache;if(e->radar_prepare_origin!=j->dual_prepare_origin||!lab_real_context_equal(&e->scene_cache.context,&j->session.view.context)){obj_radar_follow_reset(&e->radar_cache);memset(&e->scene_cache,0,sizeof(e->scene_cache));e->radar_prepare_origin=j->dual_prepare_origin;pthread_mutex_lock(&e->mutex);memset(&e->measurements,0,sizeof(e->measurements));pthread_mutex_unlock(&e->mutex);}j->radar_follow=e->radar_cache;j->dual_scene=e->scene_cache;j->dual_last_plan=e->last_plan;j->last_map_export=e->last_map_attempt;int due=!j->last_map_export||begin-j->last_map_export>=500000;int written=phone_export_map_sync(j,begin);if(due)e->last_map_attempt=j->last_map_export;e->cache=j->snapshot_cache;e->radar_cache=j->radar_follow;e->scene_cache=j->dual_scene;e->last_plan=j->dual_last_plan;++e->exports;uint64_t end=monotonic_at_least(begin);if(end-begin>20000||e->exports%10==1)fprintf(stderr,"MAP_EXPORT_COST elapsed_us=%llu snapshot_age_us=%llu exports=%llu written=%d\n",(unsigned long long)(end-begin),(unsigned long long)(end-j->session.view.time_us),(unsigned long long)e->exports,written);pthread_mutex_lock(&e->mutex);object_measurement_bind(&e->measurements,&j->dual_scene.context,j->dual_scene.epoch,j->dual_scene.track_id);object_measurement_push(&e->measurements,&j->dual_scene.measurement_v2);e->published_prepare_origin=j->dual_prepare_origin;e->published_scene=j->dual_scene;e->published_display=j->dual_display_scene;e->published_frame=j->dual_frame;if(written){e->last_success_us=end;e->failures=0;}else if(due)e->failures++;pthread_mutex_unlock(&e->mutex);
 if(due&&!written)fprintf(stderr,"MAP_EXPORT_FAILED original_snapshot_us=%llu\n",(unsigned long long)j->session.view.time_us);export_job_free(j);
 }}
static void phone_export_map(panel*p,uint64_t now){if(p->exporter&&!pthread_mutex_trylock(&p->exporter->mutex)){if(dual_export_for_task(&p->dual_task,&p->session.view,&p->dual_frame,&p->exporter->published_frame,&p->exporter->published_scene,p->dual_prepare_origin,p->exporter->published_prepare_origin)){p->dual_scene=p->exporter->published_scene;p->dual_display_scene=p->exporter->published_display;p->dual_frame=p->exporter->published_frame;object_measurement_pop(&p->exporter->measurements,&p->dual_scene.measurement_v2);}pthread_mutex_unlock(&p->exporter->mutex);}
 if(p->last_map_export&&now-p->last_map_export<(p->dual_task.prepared?100000:500000))return;p->last_map_export=now;
 if(!p->exporter){map_exporter*e=calloc(1,sizeof(*e));if(!e)return;if(pthread_mutex_init(&e->mutex,NULL)){free(e);return;}if(pthread_cond_init(&e->ready,NULL)){pthread_mutex_destroy(&e->mutex);free(e);return;}if(pthread_create(&e->thread,NULL,export_worker,e)){pthread_cond_destroy(&e->ready);pthread_mutex_destroy(&e->mutex);free(e);return;}e->started_us=now;p->exporter=e;}
 uint64_t begin=monotonic_at_least(now);panel*j=export_job_copy(p);if(!j)return;map_exporter*e=p->exporter;pthread_mutex_lock(&e->mutex);panel*old=e->pending;e->pending=j;if(old)++e->replaced;pthread_cond_signal(&e->ready);pthread_mutex_unlock(&e->mutex);export_job_free(old);uint64_t end=monotonic_at_least(begin);if(end-begin>10000)fprintf(stderr,"MAP_COPY_COST elapsed_us=%llu\n",(unsigned long long)(end-begin));
}
static void export_close(panel*p){map_exporter*e=p->exporter;if(!e)return;pthread_mutex_lock(&e->mutex);e->stop=1;pthread_cond_signal(&e->ready);pthread_mutex_unlock(&e->mutex);pthread_join(e->thread,NULL);pthread_cond_destroy(&e->ready);pthread_mutex_destroy(&e->mutex);free(e);p->exporter=NULL;}
static void s5_panel_diagnostics(panel*p,uint64_t now){
 uint32_t stalled=p->map_export_stalled;uint64_t success=0;uint32_t failures=0;
 if(p->exporter&&!pthread_mutex_trylock(&p->exporter->mutex)){
  map_exporter*e=p->exporter;success=e->last_success_us;failures=e->failures;
  uint64_t since=success?success:e->started_us;
  stalled=failures>=3||(since&&now>=since&&now-since>2000000);pthread_mutex_unlock(&e->mutex);
 }
 int export_change=stalled!=p->map_export_stalled;p->map_export_stalled=stalled;
 const char*r=p->session.view.navigation.reason,*a=p->session.view.navigation.avoidance.reason;
 const s3_avoidance_view*pl=&p->session.view.navigation.avoidance;int planner_change=strcmp(p->s6_last_planner,pl->planner_veto_reason)||p->s6_last_active!=pl->planner_active;
 int changed=planner_change||strcmp(p->s5_last_reason,r)||strcmp(p->s5_last_avoid,a)||export_change;
 if(!changed&&p->s5_last_log&&now-p->s5_last_log<1000000)return;
 char json[12288];size_t bytes=0;
 if(!phone_nav_motion_json(&p->session.view,now,json,sizeof(json),&bytes))return;
 if(bytes&&json[bytes-1]=='}'){
  int n=snprintf(json+bytes-1,sizeof(json)-bytes+1,",\"map_export\":{\"stalled\":%s,\"reason\":\"%s\",\"last_success_us\":\"%llu\",\"failures\":%u}}",stalled?"true":"false",stalled?"MAP_EXPORT_STALLED":"MAP_EXPORT_OK",(unsigned long long)success,failures);
  if(n<0||(size_t)n>=sizeof(json)-bytes+1)return;bytes=bytes-1+(size_t)n;
 }
 const char*event=export_change?(stalled?"MAP_EXPORT_STALLED":"MAP_EXPORT_RECOVERED"):changed?(a[0]?a:r):"S5_SNAPSHOT";
 if(changed)fprintf(stderr,"S5_STATE policy_revision=9 config_id=0x31110301 event=%s now_us=%llu front=%.4f side=%u,%u reverse_target=%.3f turn_hold=%u export_failures=%u\n",event,(unsigned long long)now,p->session.view.navigation.avoidance.front_clearance_m,p->session.view.navigation.avoidance.side_state[0],p->session.view.navigation.avoidance.side_state[1],p->session.view.navigation.avoidance.reverse_target_m,p->session.view.navigation.avoidance.turn_hold,failures);
 s5_log_submit(p->s5log,now,event,json,bytes,changed);
 if(planner_change||!p->s5_last_log||now-p->s5_last_log>=1000000){
  s5_log_submit(p->s5log,now,pl->planner_active?"PLANNER_CHOICE":"PLANNER_FALLBACK",json,bytes,planner_change);
  s5_log_submit(p->s5log,now,"COSTMAP_AGE",json,bytes,0);
 }
 if(pl->traj_mismatch_ms>=300&&(p->s6_last_mismatch<300||now-p->s5_last_log>=1000000))s5_log_submit(p->s5log,now,"TRAJ_MISMATCH",json,bytes,p->s6_last_mismatch<300);
 p->s6_last_mismatch=pl->traj_mismatch_ms;p->s6_last_active=pl->planner_active;snprintf(p->s6_last_planner,sizeof(p->s6_last_planner),"%s",pl->planner_veto_reason);p->s5_last_log=now;snprintf(p->s5_last_reason,sizeof(p->s5_last_reason),"%s",r);snprintf(p->s5_last_avoid,sizeof(p->s5_last_avoid),"%s",a);
}
static void cancel_diag(const panel*p,const char*origin,uint64_t now){fprintf(stderr,"PHONE_CANCEL_DIAG origin=%s now_us=%llu lease_age_us=%llu lease_seq=%llu lease_claimed=%d lease_motion=%d goal_id=%u runtime_enabled=%u runtime_pending=%u\n",origin,(unsigned long long)now,(unsigned long long)(now>=p->lease.last_seen_us?now-p->lease.last_seen_us:UINT64_MAX),(unsigned long long)p->lease.last_sequence,p->lease.claimed,p->lease.motion_active,p->goal_id,p->session.view.enabled,p->session.view.pending_enable);}
static void cancel(panel*p,uint64_t now){object_dual_task_cancel(&p->dual_task,"USER_OR_PHONE_CANCEL");(void)lab_real_runtime_object_mode(p->session.runtime,0,now);p->cruise_mode=0;p->auto_pending=0;snprintf(p->auto_status,sizeof(p->auto_status),"CANCELLED");snprintf(p->auto_reason,sizeof(p->auto_reason),"EXPLICIT_CANCEL_OR_MANUAL");object_navigation_cancel(&p->object_task,"USER_OR_PHONE_CANCEL");tag_task_cancel(&p->tag,"USER_OR_PHONE_CANCEL");p->arm_request=0;p->arm_request_id=0;p->lease.motion_active=0;lab_status r=lab_real_runtime_cancel(p->session.runtime,now);(void)lab_real_runtime_set_cruise(p->session.runtime,0);uint32_t exp=p->session.view.experimental_timing;lab_real_runtime_view_get(p->session.runtime,&p->session.view);p->session.view.experimental_timing=exp;snprintf(p->ui.message,sizeof(p->ui.message),r==LAB_OK?"Navigation cancelled; map continues":"STOP FAILED: confirm physical stop");}
static int goal(panel*p,uint64_t now){if(!p->ui.selected)return 0;lab_real_goal g={.id=++p->goal_id,.map_id=p->session.view.localization.map.map_id,.x_m=p->ui.goal_x,.y_m=p->ui.goal_y};lab_status r=lab_real_runtime_set_goal(p->session.runtime,&g,now);snprintf(p->ui.message,sizeof(p->ui.message),"GOAL %.2f %.2f: %s",g.x_m,g.y_m,lab_status_text(r));uint32_t exp=p->session.view.experimental_timing;lab_real_runtime_view_get(p->session.runtime,&p->session.view);p->session.view.experimental_timing=exp;return r==LAB_OK;}
/* 定位模块提供两端插值、地图校正后的历史位姿。RK_DEQUEUE只是近似相机时刻，延迟补偿显式配置，历史缺失时不以当前位姿代替。

 */
static int tag_inputs(panel*p,uint64_t now,tag_snapshot*s,tag_mount*m,lab_pose*pose,char reason[80]){
 const char*dir=getenv("ACG_PHONE_DIR"),*cfg=getenv("ACG_APRILTAG_CONFIG");char path[512];
 if(!cfg||!tag_mount_load(cfg,m)){snprintf(reason,80,"TAG_CONFIG_UNAVAILABLE");return 0;}
 if(!dir||snprintf(path,sizeof(path),"%s/apriltag.json",dir)>=(int)sizeof(path)||!tag_snapshot_load(path,s)){snprintf(reason,80,"TAG_SNAPSHOT_UNAVAILABLE_OR_INVALID");return 0;}
 if(!m->calibrated||!s->calibrated){snprintf(reason,80,"TAG_CAMERA_NOT_CALIBRATED");return 0;}
 if(!m->verified){snprintf(reason,80,"TAG_CAMERA_MOUNT_NOT_VERIFIED");return 0;}
 if(!s->frame_us||s->frame_us>now||now-s->frame_us>m->max_age_us||s->frame_us<m->delay_us){snprintf(reason,80,"TAG_IMAGE_STALE_OR_TIME_INVALID");return 0;}
 if(!p->session.view.has_localization||(!p->session.view.localization.odometry_fallback&&p->session.view.localization.state!=LAB_REAL_LOC_TRACKING)){snprintf(reason,80,"TAG_WAIT_MAP_LOCALIZATION");return 0;}
 if(!tag_capture_pose(s,m,&p->session.bindings.localization,&p->session.view.context,pose,reason))return 0;
 return 1;
}
static int tag_footprint_clear(panel*p,const tag_task*t){
 const lab_real_grid_view*m=&p->session.view.localization.map;if(!lab_real_grid_valid(m))return 0;
 passage_world w={0};w.config=p->session.options.passage;w.global=(passage_grid){m->width,m->height,m->resolution_m,m->origin_x_m,m->origin_y_m,m->cells};
 w.initial=(passage_pose){p->session.view.localization.pose.x_m,p->session.view.localization.pose.y_m,p->session.view.localization.pose.yaw_rad};w.allow_initial_unknown=0;passage_result hit;
 return passage_pose_clear(&w,(passage_pose){t->goal_x,t->goal_y,t->goal_yaw},w.config.width_m,&hit);
}
static void tag_reject(panel*p,const char*reason){p->tag.active=0;snprintf(p->tag.state,sizeof(p->tag.state),"REJECTED");snprintf(p->tag.reason,sizeof(p->tag.reason),"%.79s",reason);snprintf(p->ui.message,sizeof(p->ui.message),"%.79s",reason);}
static void tag_preview(panel*p,const phone_goal_message*msg,uint64_t now){
 /* 选择不同目标会取消旧任务，即使新目标被拒绝，也不能留下隐藏的旧目标。
 */
 cancel_diag(p,"NEW_TAG_GOAL",now);cancel(p,now);p->tag=(tag_task){.id=(int)msg->reserved-1,.standoff=msg->x_m,.request_us=msg->sent_us};p->ui.selected=0;
 if(msg->map_id!=p->session.view.localization.map.map_id||!msg->map_version||msg->map_version>p->session.view.localization.map.map_version){tag_reject(p,"TAG_STALE_MAP_CONTEXT_OR_FUTURE_VERSION");return;}
 tag_snapshot snapshot;tag_mount mount;lab_pose pose;char reason[80];
 if(!tag_inputs(p,now,&snapshot,&mount,&pose,reason)){tag_reject(p,reason);return;}
 tag_task target;if(!tag_goal_compute(&snapshot,&mount,p->tag.id,msg->x_m,now,&pose,p->session.options.passage.length_m*.5,&target)){p->tag=target;p->tag.request_us=msg->sent_us;snprintf(p->ui.message,sizeof(p->ui.message),"%.79s",target.reason);return;}
 target.map_id=msg->map_id;target.created_us=now;target.request_us=msg->sent_us;p->tag=target;
 if(!tag_footprint_clear(p,&target)){tag_reject(p,"TAG_APPROACH_OCCUPIED_OR_UNKNOWN");return;}
 p->ui.selected=1;p->ui.goal_adjusted=0;p->ui.goal_x=p->ui.requested_x=target.goal_x;p->ui.goal_y=p->ui.requested_y=target.goal_y;
 lab_real_goal g={.id=++p->goal_id,.map_id=target.map_id,.has_yaw=1,.x_m=target.goal_x,.y_m=target.goal_y,.yaw_rad=target.goal_yaw};
 lab_status r=lab_real_runtime_set_goal(p->session.runtime,&g,now);if(r!=LAB_OK){tag_reject(p,"TAG_NAVIGATOR_REJECTED_GOAL");return;}
 snprintf(p->ui.message,sizeof(p->ui.message),"TAG %d PREVIEW_PENDING - explicit phone START required",p->tag.id);
}
static void tag_task_tick(panel*p,uint64_t now){
 if(!p->tag.active)return;if(p->tag.map_id!=p->session.view.localization.map.map_id){cancel(p,now);tag_reject(p,"TAG_MAP_CONTEXT_CHANGED");return;}
 if(!strcmp(p->tag.state,"PREVIEW_PENDING")&&now>p->tag.created_us&&p->session.view.goal.id==p->goal_id&&p->session.view.navigation.path_count&&p->session.view.has_goal){snprintf(p->tag.state,sizeof(p->tag.state),"PREVIEW_READY");snprintf(p->tag.reason,sizeof(p->tag.reason),"TAG_ROUTE_AVAILABLE_EXPLICIT_START_REQUIRED");}
 if(!strcmp(p->tag.state,"PREVIEW_PENDING")&&!p->session.view.navigation.path_count){snprintf(p->tag.reason,sizeof(p->tag.reason),"TAG_ROUTE_%.60s",p->session.view.navigation.reason);}
 if((!strcmp(p->tag.state,"NAVIGATING")&&!strcmp(p->session.view.reason,"GOAL_REACHED"))||!strcmp(p->tag.state,"WAIT_REOBSERVE")){
  if(p->tag.last_poll_us&&now-p->tag.last_poll_us<200000)return;p->tag.last_poll_us=now;
  if(!p->tag.reached_us)p->tag.reached_us=now;
  snprintf(p->tag.state,sizeof(p->tag.state),"WAIT_REOBSERVE");tag_snapshot snap;tag_mount mount;lab_pose pose;char reason[80];
  if(!tag_inputs(p,now,&snap,&mount,&pose,reason)){snprintf(p->tag.reason,sizeof(p->tag.reason),"%.79s",reason);return;}
  if(snap.frame_us<p->tag.reached_us){snprintf(p->tag.reason,sizeof(p->tag.reason),"TAG_WAIT_NEW_IMAGE_AFTER_STOP");return;}
  if(tag_task_reobserve(&p->tag,&snap,&mount,now,&pose)){p->tag.active=0;p->lease.motion_active=0;snprintf(p->ui.message,sizeof(p->ui.message),"TAG %d ARRIVED - visual reobservation confirmed",p->tag.id);}
 }
}
/* 实物关联借用当前会话雷达扫描与定位，预览只登记目标，使能仍由显式START完成。
 */
static void object_preview(panel*p,const phone_goal_message*m,uint64_t now){
 cancel_diag(p,"NEW_OBJECT_GOAL",now);cancel(p,now);p->ui.selected=0;
 obj_lidar_target a={0};obj_lidar_get_target(p->object_lidar,&a);
 if(!object_navigation_preview(&p->object_task,m,&a,&p->session.view,p->session.options.passage.length_m*.5,now)){snprintf(p->ui.message,sizeof(p->ui.message),"%.95s",p->object_task.reason);return;}
 tag_task footprint={.goal_x=p->object_task.goal_x,.goal_y=p->object_task.goal_y,.goal_yaw=p->object_task.goal_yaw};
 if(!tag_footprint_clear(p,&footprint)){object_navigation_reject(&p->object_task,"OBJECT_APPROACH_OCCUPIED_OR_UNKNOWN");snprintf(p->ui.message,sizeof(p->ui.message),"%.95s",p->object_task.reason);return;}
 lab_real_goal g={.id=++p->goal_id,.map_id=p->object_task.map_id,.has_yaw=1,.x_m=p->object_task.goal_x,.y_m=p->object_task.goal_y,.yaw_rad=p->object_task.goal_yaw};
 if(lab_real_runtime_set_goal(p->session.runtime,&g,now)!=LAB_OK){lab_real_runtime_cancel(p->session.runtime,now);object_navigation_reject(&p->object_task,"OBJECT_NAVIGATOR_REJECTED_GOAL");return;}
 p->ui.selected=1;p->ui.goal_adjusted=0;p->ui.goal_x=p->ui.requested_x=g.x_m;p->ui.goal_y=p->ui.requested_y=g.y_m;
 snprintf(p->ui.message,sizeof(p->ui.message),"OBJECT PREVIEW_PENDING - explicit phone START required");
}
static void object_task_tick(panel*p,uint64_t now){
 obj_lidar_target a={0};obj_lidar_get_target(p->object_lidar,&a);
 if(object_navigation_tick(&p->object_task,&a,&p->session.view,p->session.view.goal.id==p->goal_id,now)){
  p->arm_request=0;p->lease.motion_active=0;if(lab_real_runtime_cancel(p->session.runtime,now)!=LAB_OK)object_navigation_reject(&p->object_task,"OBJECT_STOP_TRANSPORT_FAILURE");p->ui.selected=0;
  snprintf(p->ui.message,sizeof(p->ui.message),"%.95s",p->object_task.reason);
 }
}
static void reset_map(panel*p,uint64_t now){object_dual_task_cancel(&p->dual_task,"MAP_RESET");p->dual_yaw_us=0;p->dual_measured_w_valid=0;
 cancel_diag(p,"RESET_MAP",now);p->cruise_mode=p->auto_pending=0;object_navigation_cancel(&p->object_task,"MAP_RESET");tag_task_cancel(&p->tag,"MAP_RESET");
 if(lab_real_runtime_cancel(p->session.runtime,now)!=LAB_OK){snprintf(p->ui.message,sizeof(p->ui.message),"RESET_DENIED_STOP_TRANSPORT_FAILURE");return;}
 lab_real_options options=p->session.options;lab_real_context ctx=p->session.view.context;
 if(ctx.origin_epoch==UINT32_MAX){snprintf(p->ui.message,sizeof(p->ui.message),"RESET_DENIED_EPOCH_EXHAUSTED");return;}
 ++ctx.origin_epoch;p->arm_request=0;phone_nav_lease_init(&p->lease);lab_real_session_close(&p->session);
 lab_status r=lab_real_session_open(&p->session,&options,&ctx,NULL);
 live_nav_ui_init(&p->ui);p->goal_id=0;p->last_map_export=0;
 if(r!=LAB_OK){fprintf(stderr,"RESET_MAP_OPEN_FAILED status=%d\n",r);p->done=1;return;}
 lab_real_runtime_object_controller(p->session.runtime,dual_intent,p);lab_real_runtime_observe(p->session.runtime,dual_sample_observer,p);
 snprintf(p->ui.message,sizeof(p->ui.message),"MAP_RESET_WAIT_NEW_SCAN_NO_AUTOMATIC_START");
}
static phone_goal_socket goal_socket={.fd=-1,.dirfd=-1,.lockfd=-1};
static void dual_request(panel*p,const phone_goal_message*m,uint64_t now){
 const char*reject=NULL;uint64_t gen=0,revoked=0;const char*dir=getenv("ACG_PHONE_DIR");
 int radar=m->reserved==3,search=m->reserved==4||m->reserved==6,direct=m->reserved==5||m->reserved==7;
 char search_category[64]={0},search_color[16]={0},search_size[12]={0};
 if(m->reserved==6&&!object_attribute_search_parse(m->association_id,search_category,sizeof(search_category),search_color,sizeof(search_color),search_size,sizeof(search_size)))reject="OBJECT_ATTRIBUTE_SEARCH_INVALID";
 unsigned task_mode=(search||m->reserved==7)?OBJECT_MODE_STATIC:(radar||m->reserved==5)?OBJECT_MODE_FOLLOW:m->reserved;
 if(m->operation==PHONE_NAV_OBJECT_CANCEL){
  if(p->lease.claimed&&strcmp(p->lease.session,m->session)){request_result(p,m,now,"REJECTED","OBJECT_CANCEL_OWNER_MISMATCH");return;}
  if(!m->sequence||(p->lease.claimed&&m->sequence<=p->lease.last_sequence)||m->sent_us<p->last_start_sent_us){request_result(p,m,now,"REJECTED","OBJECT_CANCEL_OLDER_THAN_CURRENT_TASK");return;}
  /* 服务端可能在取消时刻撤销屏障，显式取消不依赖传感器健康或该屏障。
 */
  if(p->lease.claimed){p->lease.last_sequence=m->sequence;p->lease.last_seen_us=now;}
  cancel(p,now);p->dual_task.request_id=m->request_id;request_result(p,m,now,"ACCEPTED","OBJECT_TASK_CANCELLED");dual_publish_task(p,now);return;
 }
 if(!dir||phone_control_fence_read_state(dir,&gen,&revoked)<0)reject="OBJECT_CONTROL_FENCE_UNAVAILABLE";
 if(!reject)reject=object_request_admission(m,now,revoked);
 if(!reject&&m->operation!=PHONE_NAV_OBJECT_PREPARE&&!p->session.options.enable_drive)reject="PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE";
 if(!reject&&phone_nav_lease_claim(&p->lease,m->session,now)!=PHONE_NAV_OK)reject="OBJECT_OWNER_MISMATCH";
 if(!reject&&m->sequence<=p->lease.last_sequence)reject="OBJECT_DUPLICATE_OR_OLD_SEQUENCE";
 if(m->operation==PHONE_NAV_OBJECT_PAUSE||m->operation==PHONE_NAV_OBJECT_RESUME){
  if(!reject&&(task_mode!=OBJECT_MODE_FOLLOW||!p->dual_task.prepared||p->dual_task.mode!=OBJECT_MODE_FOLLOW||(!p->dual_task.radar_only&&(p->dual_task.epoch!=m->object_epoch||p->dual_task.track_id!=m->object_track_id))))reject="FOLLOW_CONTROL_TASK_MISMATCH";
  if(!reject&&m->operation==PHONE_NAV_OBJECT_PAUSE){
   if(phone_nav_lease_receive(&p->lease,m->session,m->sequence,m->sent_us,now,revoked,0)!=PHONE_NAV_OK)reject="FOLLOW_PAUSE_SEQUENCE_INVALID";
   else{object_dual_task_pause(&p->dual_task,"USER_PAUSED_FOLLOW");p->dual_task.request_id=m->request_id;p->arm_request=0;p->arm_request_id=0;p->lease.motion_active=0;
    lab_status a=lab_real_runtime_enable(p->session.runtime,0,now),b=lab_real_runtime_object_mode(p->session.runtime,0,now);uint32_t exp=p->session.view.experimental_timing;lab_real_runtime_view_get(p->session.runtime,&p->session.view);p->session.view.experimental_timing=exp;
    request_result(p,m,now,a==LAB_OK&&b==LAB_OK?"ACCEPTED":"REJECTED",a==LAB_OK&&b==LAB_OK?"FOLLOW_PAUSED_TARGET_MEMORY_RETAINED":"FOLLOW_PAUSE_STOP_TRANSPORT_FAILED");dual_publish_task(p,now);return;}
  }
  if(!reject&&m->operation==PHONE_NAV_OBJECT_RESUME){if(strcmp(p->dual_task.state,"PAUSED"))reject="FOLLOW_RESUME_REQUIRES_PAUSED_TASK";else p->dual_task.standoff_m=m->x_m;}
 }else if(m->operation==PHONE_NAV_OBJECT_PREPARE||m->operation==PHONE_NAV_OBJECT_START){
  if(!reject&&direct)dual_semantic_read_frame(dir,&p->dual_frame,now,m->object_epoch,m->object_track_id,NULL);
  if(!reject){
   /* 在同一事件循环事务中登记并启动，不要求选中帧、深度、地图点或先前PREPARE确认。
 */
   cancel(p,now);if(strstr(p->ui.message,"STOP FAILED"))reject="PREVIOUS_MOTION_STOP_FAILED";
   if(!reject){obj_radar_follow_reset(&p->radar_follow);int prepared=m->reserved==6?object_dual_task_prepare_search_attributes(&p->dual_task,search_category,search_color,search_size,m->x_m,m->request_id):search?object_dual_task_prepare_search(&p->dual_task,m->association_id,m->x_m,m->request_id):radar?object_dual_task_prepare_radar(&p->dual_task,p->session.view.context.timeline_id,m->x_m,m->request_id):object_dual_task_prepare(&p->dual_task,task_mode,m->object_epoch,m->object_track_id,m->x_m,m->request_id);if(!prepared)reject="OBJECT_MODE_OR_PARAMETERS_INVALID";else{object_dual_task_geometry_load(&p->dual_task,getenv("ACG_OBJECTS_CONFIG"));p->dual_task.semantic_direct=direct;p->dual_prepare_origin=m->sent_us;}}
  }
  if(!reject&&m->operation==PHONE_NAV_OBJECT_PREPARE){
   if(phone_nav_lease_receive(&p->lease,m->session,m->sequence,m->sent_us,now,revoked,0)!=PHONE_NAV_OK)reject="OBJECT_PREPARE_SEQUENCE_INVALID";
   else{request_result(p,m,now,"ACCEPTED","OBJECT_REGISTERED_LEGACY_PREPARE");dual_publish_task(p,now);return;}
  }
 }else if(!reject)reject="OBJECT_OPERATION_INVALID";
 if(!reject&&phone_nav_lease_start(&p->lease,m->session,m->sequence,now,1)!=PHONE_NAV_OK)reject="OBJECT_START_OWNER_OR_SEQUENCE_INVALID";
 if(!reject&&lab_real_runtime_object_mode(p->session.runtime,(int)task_mode,now)!=LAB_OK)reject="OBJECT_RUNTIME_MODE_UNAVAILABLE";
 if(!reject&&!object_dual_task_start(&p->dual_task,task_mode,p->dual_task.epoch,p->dual_task.track_id,now))reject="OBJECT_START_REJECTED";
 if(!reject){p->dual_task.request_id=m->request_id;p->last_start_sent_us=m->sent_us;p->arm_request=1;p->arm_request_id=m->request_id;p->arm_deadline=0;request_result(p,m,now,"ARMING","OBJECT_TASK_ACCEPTED_STARTING_WITH_CURRENT_SENSORS");dual_publish_task(p,now);return;}
 /* 被拒绝的重放或错误所有者请求不能取消运行中的任务。 */
 request_result(p,m,now,"REJECTED",reject?reject:"OBJECT_REQUEST_INVALID");snprintf(p->ui.message,sizeof(p->ui.message),"%s",reject?reject:"OBJECT_REQUEST_INVALID");dual_publish_task(p,now);
}
static int phone_goal_open(panel*p){
 const char*dir=getenv("ACG_PHONE_DIR");char reason[128];
 int fd=phone_goal_socket_open(&goal_socket,dir,reason,sizeof(reason));
 if(fd<0)fprintf(stderr,"PHONE_GOAL_SOCKET_UNAVAILABLE reason=%s; live/unknown owners are never removed; stop original owner or inspect private directory\n",reason);
 else fprintf(stderr,"PHONE_GOAL_SOCKET_READY reason=%s\n",reason);
 if(dir)snprintf(p->goal_path,sizeof(p->goal_path),"%s/navigation-goal.sock",dir);return fd;
}
/* 模式使用独立有界快照，手动控制不依赖该快照。 */
static int auto_ready(const panel*p,uint64_t now){const lab_real_runtime_view*v=&p->session.view;return p->session.options.enable_drive&&p->session.options.passage.mount_verified&&navigation_pose_valid(v)&&v->has_scan&&v->scan_time_us<=now&&now-v->scan_time_us<=CONT_SCAN_US&&v->feedback.time_us&&v->feedback.time_us<=now&&now-v->feedback.time_us<=CONT_FEEDBACK_US&&!v->feedback.faults;}
static void mode_publish(panel*p,uint64_t now){if(p->last_mode_export&&now-p->last_mode_export<250000)return;p->last_mode_export=now;const char*dir=getenv("ACG_PHONE_DIR");if(!dir)return;
 char path[512],temp[560],body[2048];if(snprintf(path,sizeof(path),"%s/mode-state.json",dir)>=(int)sizeof(path)||snprintf(temp,sizeof(temp),"%s.%lu.tmp",path,(unsigned long)getpid())>=(int)sizeof(temp))return;
 int active=p->cruise_mode&&!p->auto_pending&&p->lease.motion_active,intent=p->cruise_mode&&p->lease.motion_active;
 const char*mode=intent?"AUTO":p->lease.motion_active&&!p->cruise_mode?"NAV":"MANUAL";
 char reason[96];snprintf(reason,sizeof(reason),"%s",p->auto_reason);for(unsigned j=0;reason[j];j++)if(reason[j]=='"'||reason[j]=='\\'||(unsigned char)reason[j]<32)reason[j]='_';
 int n=snprintf(body,sizeof(body),"{\"version\":1,\"published_us\":\"%llu\",\"control_mode\":\"%s\",\"auto_active\":%s,\"auto_intent_accepted\":%s,\"auto_waiting\":%s,\"auto_ready\":%s,\"auto_request_us\":\"%llu\",\"auto_status\":\"%s\",\"auto_reason\":\"%s\",\"session\":\"%s\",\"drive_capable\":%s}",(unsigned long long)now,mode,active?"true":"false",intent?"true":"false",intent&&p->auto_pending?"true":"false",auto_ready(p,now)?"true":"false",(unsigned long long)p->auto_request_us,p->auto_status[0]?p->auto_status:"NONE",reason,p->lease.claimed?p->lease.session:"",p->session.options.enable_drive?"true":"false");
 if(n<=0||(size_t)n>=sizeof(body))return;int fd=open(temp,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)return;int ok=write(fd,body,(size_t)n)==n;int closed=close(fd);if(!ok||closed||rename(temp,path))unlink(temp);
}
static void auto_deny(panel*p,uint64_t now,const char*reason){fprintf(stderr,"AUTO_FINAL issued_us=%llu outcome=DENIED code=%s now_us=%llu\n",(unsigned long long)p->auto_request_us,reason,(unsigned long long)now);cancel(p,now);snprintf(p->auto_status,sizeof(p->auto_status),"DENIED");snprintf(p->auto_reason,sizeof(p->auto_reason),"%.95s",reason);snprintf(p->ui.message,sizeof(p->ui.message),"AUTO_NOT_STARTED: %.90s",reason);p->last_mode_export=0;mode_publish(p,now);}
static void auto_reject_request(panel*p,uint64_t now,uint64_t issued,const char*reason){fprintf(stderr,"AUTO_FINAL issued_us=%llu outcome=REJECTED code=%s now_us=%llu\n",(unsigned long long)issued,reason,(unsigned long long)now);if(p->auto_pending||p->cruise_mode)return;p->auto_request_us=issued;snprintf(p->auto_status,sizeof(p->auto_status),"DENIED");snprintf(p->auto_reason,sizeof(p->auto_reason),"%.95s",reason);p->last_mode_export=0;mode_publish(p,now);}
static void auto_prepare(panel*p,const phone_goal_message*m,uint64_t now){
 if(p->session.view.object_mode||p->dual_task.prepared){object_dual_task_cancel(&p->dual_task,"SWITCH_TO_AUTO");(void)lab_real_runtime_object_mode(p->session.runtime,0,now);}
 uint64_t generation=0,revoked=0;const char*dir=getenv("ACG_PHONE_DIR");
 if(m->request_expires_us<=now||m->request_expires_us<=m->sent_us||m->request_expires_us-m->sent_us>CONT_PHONE_US){auto_reject_request(p,now,m->sent_us,"AUTO_REQUEST_EXPIRED");return;}
 if(!dir||phone_control_fence_read_state(dir,&generation,&revoked)<0||!m->switch_generation||generation!=m->switch_generation){auto_reject_request(p,now,m->sent_us,"AUTO_SWITCH_GENERATION_REVOKED");return;}
 if(m->switch_generation<=p->last_auto_generation){auto_reject_request(p,now,m->sent_us,"AUTO_SWITCH_GENERATION_REPLAYED");return;}
 /* 认证服务端签发的代次决定模式切换，旧手机估计时间、旧会话或排队PING不能否决新切换；即使后续准备失败，该代次也只消费一次。

 */
 p->last_auto_generation=m->switch_generation;p->auto_request_us=m->sent_us;
 if(!p->session.options.enable_drive||!p->session.options.passage.mount_verified){auto_reject_request(p,now,m->sent_us,"AUTO_DRIVE_OR_MOUNT_NOT_CONFIGURED");return;}
 if(lab_real_runtime_cancel(p->session.runtime,now)!=LAB_OK){auto_deny(p,now,"AUTO_PREVIOUS_STOP_FAILED");return;}
 p->arm_request=0;phone_nav_lease_init(&p->lease);
 if(phone_nav_lease_claim(&p->lease,m->session,now)!=PHONE_NAV_OK||phone_nav_lease_start(&p->lease,m->session,m->sequence,now,1)!=PHONE_NAV_OK){auto_deny(p,now,"AUTO_PHONE_SESSION_DENIED");return;}
 object_navigation_cancel(&p->object_task,"SWITCH_TO_AUTO");tag_task_cancel(&p->tag,"SWITCH_TO_AUTO");p->ui.selected=0;
 /* 先消费新撤销代次，再创建本事务目标。过期只限制准入，成功消费后成为持续持有的任务，不要求2.5秒内开始移动；后续撤销仍取消全部，上方runtime_cancel已清除旧期限。


 */
 (void)lab_real_runtime_acknowledge_start(p->session.runtime,now);
 if(lab_real_runtime_set_cruise(p->session.runtime,1)!=LAB_OK){auto_deny(p,now,"AUTO_CONTROLLER_UNAVAILABLE");return;}
 p->cruise_mode=1;p->auto_request=*m;p->auto_pending=1;p->arm_request=0;p->last_start_sent_us=m->sent_us;snprintf(p->auto_status,sizeof(p->auto_status),"PREPARING");snprintf(p->auto_reason,sizeof(p->auto_reason),"WAIT_REAL_TICK_FIRST_ACTION");cruise_preview(p,now);p->last_mode_export=0;
 fprintf(stderr,"AUTO_PREPARING issued_us=%llu deadline_us=%llu generation=%llu now_us=%llu\n",(unsigned long long)m->sent_us,(unsigned long long)m->request_expires_us,(unsigned long long)m->switch_generation,(unsigned long long)now);
}
static void auto_finalize(panel*p,uint64_t now){if(!p->auto_pending)return;
 uint64_t generation=0,revoked=0;const char*dir=getenv("ACG_PHONE_DIR");if(!dir||phone_control_fence_read_state(dir,&generation,&revoked)<0||generation!=p->auto_request.switch_generation){auto_deny(p,now,"AUTO_SWITCH_GENERATION_REVOKED");return;}
 if(!p->lease.claimed||!p->lease.motion_active||strcmp(p->lease.session,p->auto_request.session)||now<p->lease.last_seen_us||now-p->lease.last_seen_us>=CONT_PHONE_US){auto_deny(p,now,"AUTO_PHONE_OWNER_EXPIRED_OR_REVOKED");return;}
 uint64_t applied=lab_real_runtime_first_apply_us(p->session.runtime);
 if(applied&&applied<=now&&p->session.view.enabled){p->auto_pending=0;snprintf(p->auto_status,sizeof(p->auto_status),"ACTIVE");snprintf(p->auto_reason,sizeof(p->auto_reason),"RK_SHARED_AUTO_CONTROLLER");p->last_mode_export=0;fprintf(stderr,"AUTO_FINAL issued_us=%llu outcome=ACTIVE first_apply_us=%llu now_us=%llu\n",(unsigned long long)p->auto_request_us,(unsigned long long)applied,(unsigned long long)now);return;}

 if(p->session.view.last_status<0&&p->session.view.last_status!=LAB_ERR_UNAVAILABLE){auto_deny(p,now,p->session.view.reason[0]?p->session.view.reason:"AUTO_INITIAL_RUNTIME_FAULT");return;}
 if(!auto_ready(p,now)||!p->session.view.has_goal){snprintf(p->auto_status,sizeof(p->auto_status),"WAITING");snprintf(p->auto_reason,sizeof(p->auto_reason),"WAIT_FRESH_SENSORS_OR_FIRST_ACTION");return;}
 if(p->session.view.enabled||p->session.view.pending_enable){snprintf(p->auto_status,sizeof(p->auto_status),"ARMING");snprintf(p->auto_reason,sizeof(p->auto_reason),"WAIT_POST_ARM_FRESH_COMMAND");return;}
 lab_status ready=lab_real_runtime_start_ready(p->session.runtime,now);if(ready==LAB_AGAIN){snprintf(p->auto_status,sizeof(p->auto_status),"WAITING");snprintf(p->auto_reason,sizeof(p->auto_reason),"WAIT_REAL_TICK_FIRST_ACTION");return;}if(ready!=LAB_OK){auto_deny(p,now,"AUTO_INITIAL_RUNTIME_FAULT");return;}
 lab_status ack=lab_real_runtime_acknowledge_start(p->session.runtime,now);if(ack==LAB_AGAIN)return;if(ack!=LAB_OK){auto_deny(p,now,"AUTO_START_REVOKED_OR_FINAL_MOTOR_FAULT");return;}
 lab_status r=lab_real_runtime_enable(p->session.runtime,1,now);uint32_t exp=p->session.view.experimental_timing;lab_real_runtime_view_get(p->session.runtime,&p->session.view);p->session.view.experimental_timing=exp;
 if(r<0){auto_deny(p,now,"AUTO_INITIAL_ENABLE_DENIED");return;}
 snprintf(p->auto_status,sizeof(p->auto_status),"ARMING");snprintf(p->auto_reason,sizeof(p->auto_reason),"WAIT_POST_ARM_FRESH_COMMAND");p->last_mode_export=0;
}
static void phone_goal_poll(panel*p,uint64_t now){
 now=monotonic_at_least(now);
 if(phone_nav_lease_tick(&p->lease,now,1,0)==PHONE_NAV_STOP_REQUIRED){cancel_diag(p,"LEASE_TIMEOUT_POLL",now);cancel(p,now);request_result(p,NULL,now,"REJECTED","PHONE_LINK_EXPIRED_NAVIGATION_STOPPED");snprintf(p->ui.message,sizeof(p->ui.message),"PHONE_LINK_EXPIRED_NAVIGATION_STOPPED");}
 if(p->goal_fd<0)return;for(int i=0;i<4;i++){
  phone_goal_message msg={0};ssize_t n=recv(p->goal_fd,&msg,sizeof(msg),MSG_DONTWAIT|MSG_TRUNC);if(n<0)break;now=monotonic_at_least(now);
  if(!phone_goal_message_valid(&msg,(size_t)n,now)){const char*why=phone_goal_validation_reason(&msg,(size_t)n,now);fprintf(stderr,"PHONE_IPC_REJECTED reason=%s operation=%u version=%u bytes=%lld sent_us=%llu now_us=%llu\n",why,msg.operation,msg.version,(long long)n,(unsigned long long)msg.sent_us,(unsigned long long)now);if((size_t)n==sizeof(msg)&&msg.magic==PHONE_GOAL_MAGIC&&msg.request_id&&msg.operation!=PHONE_NAV_HEARTBEAT&&msg.operation!=PHONE_NAV_OVERRIDE)request_result(p,&msg,now,"REJECTED",why);continue;}
  if(msg.operation>=PHONE_NAV_OBJECT_PREPARE&&msg.operation<=PHONE_NAV_OBJECT_RESUME){dual_request(p,&msg,now);continue;}
  if(msg.operation==PHONE_NAV_AUTO_START){auto_prepare(p,&msg,now);continue;}
  if(msg.operation==PHONE_NAV_OVERRIDE){if(msg.sent_us<=p->last_start_sent_us)continue;if(p->lease.claimed&&!strcmp(msg.session,p->lease.session)){cancel_diag(p,"MANUAL_OVERRIDE",now);cancel(p,now);phone_nav_lease_init(&p->lease);request_result(p,NULL,now,"REJECTED","MANUAL_OVERRIDE_NAVIGATION_STOPPED");snprintf(p->ui.message,sizeof(p->ui.message),"MANUAL_OVERRIDE_NAVIGATION_STOPPED");}continue;}
  if(msg.operation==PHONE_NAV_RESET_MAP){request_result(p,&msg,now,"ACCEPTED","MAP_RESET_REQUEST_RECEIVED");reset_map(p,now);if(p->done)return;continue;}
  if(msg.operation==PHONE_NAV_HEARTBEAT){if(p->lease.claimed){int r=phone_nav_lease_heartbeat(&p->lease,msg.session,msg.sequence,msg.sent_us);if(r==PHONE_NAV_STOP_REQUIRED){cancel_diag(p,"HEARTBEAT_REJECTED",now);cancel(p,now);phone_nav_lease_init(&p->lease);request_result(p,NULL,now,"REJECTED","PHONE_LINK_EXPIRED_NAVIGATION_STOPPED");}}continue;}
  if(msg.operation==PHONE_NAV_GOAL||msg.operation==PHONE_NAV_TAG_GOAL||msg.operation==PHONE_NAV_OBJECT_GOAL){
   uint64_t generation=0,revoked=0;const char*dir=getenv("ACG_PHONE_DIR");
   if(dir&&(phone_control_fence_read_state(dir,&generation,&revoked)<0||msg.sent_us<=revoked)){request_result(p,&msg,now,"REJECTED","GOAL_PREDATES_MANUAL_REVOKE");continue;}
   if(phone_nav_lease_receive(&p->lease,msg.session,msg.sequence,msg.sent_us,now,revoked,1)!=PHONE_NAV_OK){request_result(p,&msg,now,"REJECTED","PHONE_SESSION_BUSY_EXPIRED_OR_REQUEST_REPLAYED");continue;}
   if(cruise_selected(p))cancel(p,now);
   if(msg.operation==PHONE_NAV_OBJECT_GOAL){object_preview(p,&msg,now);if(p->object_task.active)p->accepted_goal_origin_us=msg.sent_us;request_result(p,&msg,now,p->object_task.active?"ACCEPTED":"REJECTED",p->object_task.reason);continue;}
   if(msg.operation==PHONE_NAV_TAG_GOAL){tag_preview(p,&msg,now);if(p->tag.active)p->accepted_goal_origin_us=msg.sent_us;request_result(p,&msg,now,p->tag.active?"ACCEPTED":"REJECTED",p->tag.reason);continue;}
   /* 新认证目标替换旧任务，即使新坐标被拒绝，也不能在新选择拒绝回执下继续执行旧待启动目标。

 */
   cancel_diag(p,"NEW_GOAL",now);cancel(p,now);
   if(live_nav_ui_select_world(&p->ui,&p->session.view,msg.map_id,msg.map_version,msg.x_m,msg.y_m)<=0){request_result(p,&msg,now,"REJECTED",p->ui.message);continue;}
   if(goal(p,now)){p->accepted_goal_origin_us=msg.sent_us;request_result(p,&msg,now,"ACCEPTED","GOAL_ACCEPTED_WAIT_CURRENT_ROUTE_OR_LOCAL_ACTION");}
   else request_result(p,&msg,now,"REJECTED",p->ui.message);
   continue;
  }
  if(msg.operation==PHONE_NAV_CANCEL){if(phone_nav_lease_stop(&p->lease,msg.session,msg.sequence,now)==PHONE_NAV_STOP_REQUIRED){cancel_diag(p,"EXPLICIT_STOP",now);cancel(p,now);phone_nav_lease_init(&p->lease);request_result(p,&msg,now,"ACCEPTED","USER_CANCELLED_TARGET_CLEARED");}else request_result(p,&msg,now,"REJECTED","CANCEL_OWNER_OR_SEQUENCE_MISMATCH");continue;}
  if(msg.operation==PHONE_NAV_START){
   const char*reject=p->auto_pending?"AUTO_INTENT_ALREADY_ACCEPTED_WAITING":NULL;
   if(!reject&&p->object_task.request_us&&strcmp(p->object_task.state,"CANCELLED")){obj_lidar_target a={0};obj_lidar_get_target(p->object_lidar,&a);if(p->session.view.goal.id!=p->goal_id||!object_navigation_can_start(&p->object_task,&a,&p->session.view,now))reject="OBJECT_START_DENIED_FRESH_MATCHING_PREVIEW_REQUIRED";}
   if(!reject&&p->object_task.active){tag_task f={.goal_x=p->object_task.goal_x,.goal_y=p->object_task.goal_y,.goal_yaw=p->object_task.goal_yaw};if(!tag_footprint_clear(p,&f))reject="OBJECT_START_FOOTPRINT_NO_LONGER_CLEAR";}
   if(!reject&&p->tag.active&&strcmp(p->tag.state,"PREVIEW_READY"))reject="TAG_START_DENIED_WAIT_TARGET_ROUTE_PREVIEW";
   if(!reject&&!p->session.options.enable_drive)reject="PREVIEW_ONLY_RESTART_WITH_BEGIN_DRIVE";
   if(!reject&&!p->session.options.passage.mount_verified)reject="LIDAR_OFFSET_NOT_VERIFIED";
   if(!reject&&(msg.map_id!=p->session.view.localization.map.map_id||!p->session.view.has_goal))reject="START_DENIED_CURRENT_MAP_AND_ACCEPTED_GOAL_REQUIRED";
   if(!reject&&!cruise_selected(p)&&(fabs(msg.x_m-p->session.view.goal.x_m)>1e-6||fabs(msg.y_m-p->session.view.goal.y_m)>1e-6))reject="START_TARGET_CHANGED_REPREVIEW_REQUIRED";
   if(!reject&&!cruise_selected(p)&&p->accepted_goal_origin_us&&msg.sent_us<p->accepted_goal_origin_us)reject="START_PREDATES_CURRENT_GOAL";
   uint64_t fence_generation=0,revoked_at=0;const char*dir=getenv("ACG_PHONE_DIR");
   if(!reject&&dir&&(phone_control_fence_read_state(dir,&fence_generation,&revoked_at)<0||msg.sent_us<=revoked_at))reject="START_PREDATES_MANUAL_REVOKE";
   if(!reject&&cruise_selected(p)&&phone_nav_lease_claim(&p->lease,msg.session,now)!=PHONE_NAV_OK)reject="START_DENIED_PHONE_SESSION";
   if(!reject&&phone_nav_lease_start(&p->lease,msg.session,msg.sequence,now,1)!=PHONE_NAV_OK)reject="START_DENIED_PHONE_SESSION_OR_SEQUENCE";
   if(reject){snprintf(p->ui.message,sizeof(p->ui.message),"%.127s",reject);request_result(p,&msg,now,"REJECTED",reject);continue;}
   /* START接收任务，具体动作仍逐tick检查实际反馈、位姿、碰撞和命令条件；当前路线阻塞时保留有效手机任务并等待重规划。

 */
   if(p->object_task.active){snprintf(p->object_task.state,sizeof(p->object_task.state),"NAVIGATING");snprintf(p->object_task.reason,sizeof(p->object_task.reason),"OBJECT_FIXED_SURFACE_GOAL_NAVIGATION");}
   if(p->tag.active){snprintf(p->tag.state,sizeof(p->tag.state),"NAVIGATING");snprintf(p->tag.reason,sizeof(p->tag.reason),"TAG_FIXED_GOAL_NAVIGATION");}
   p->last_start_sent_us=msg.sent_us;p->arm_request=1;p->arm_request_id=msg.request_id?msg.request_id:msg.sequence;p->arm_deadline=0;
   snprintf(p->ui.message,sizeof(p->ui.message),"EXPLICIT_PHONE_START_PENDING");request_result(p,&msg,now,"ARMING",motion_blocker(p));
  }
 }
}
static void phone_arm_tick(panel*p,uint64_t now){
 if(p->arm_request){
  uint64_t generation=0,revoked=0;const char*dir=getenv("ACG_PHONE_DIR");
  if(dir&&(phone_control_fence_read_state(dir,&generation,&revoked)<0||p->last_start_sent_us<=revoked)){
   arm_result(p,now,"REJECTED","PENDING_START_REVOKED_BY_STOP_OR_MANUAL");cancel_diag(p,"PENDING_START_REVOKED",now);cancel(p,now);snprintf(p->ui.message,sizeof(p->ui.message),"PENDING_START_REVOKED_BY_STOP_OR_MANUAL");return;
  }
  if(!p->lease.claimed||!p->lease.motion_active||now<p->lease.last_seen_us||now-p->lease.last_seen_us>=CONT_PHONE_US){p->arm_request=0;arm_result(p,now,"REJECTED","START_OWNER_LOST_OR_CANCELLED");}
  else{
   lab_status ack=lab_real_runtime_acknowledge_start(p->session.runtime,now);
   lab_status a=ack==LAB_OK?lab_real_runtime_enable(p->session.runtime,1,now):ack;
   uint32_t experimental=p->session.view.experimental_timing;lab_real_runtime_view_get(p->session.runtime,&p->session.view);p->session.view.experimental_timing=experimental;
   if(a==LAB_OK||p->session.view.pending_enable){p->arm_request=0;snprintf(p->ui.message,sizeof(p->ui.message),"EXPLICIT_NAVIGATION_ENABLE_REQUESTED");arm_result(p,now,p->session.view.enabled?"ACTIVE":"ARMING","EXPLICIT_NAVIGATION_ENABLE_REQUESTED");}
   else if(a==LAB_ERR_IO||a==LAB_ERR_CONTEXT||a==LAB_ERR_READONLY){p->arm_request=0;p->lease.motion_active=0;snprintf(p->ui.message,sizeof(p->ui.message),"ARM_DENIED_%.100s",p->session.view.reason);arm_result(p,now,"REJECTED",p->ui.message);}
   else{const char*wait=motion_blocker(p);if(!strcmp(wait,"NONE"))wait=p->session.view.reason[0]?p->session.view.reason:"WAIT_RUNTIME_START_READINESS";snprintf(p->ui.message,sizeof(p->ui.message),"%.127s",wait);if(strcmp(p->request_state,"ARMING")||strcmp(p->request_reason,wait))arm_result(p,now,"ARMING",wait);}
  }
 }
 if(!strcmp(p->request_state,"ARMING")&&(p->request_operation==PHONE_NAV_START||p->request_operation==PHONE_NAV_OBJECT_START||p->request_operation==PHONE_NAV_OBJECT_RESUME)&&p->session.view.enabled)arm_result(p,now,"ACTIVE","NAVIGATION_ACTIVE");
 if(!p->dual_task.active&&p->lease.motion_active&&!p->session.view.has_goal&&!p->session.view.enabled&&!p->session.view.pending_enable&&!p->auto_pending&&!p->cruise_mode){p->lease.motion_active=0;p->arm_request=0;if(p->request_operation==PHONE_NAV_START)arm_result(p,now,"ACCEPTED",!strcmp(p->session.view.reason,"GOAL_REACHED")?"GOAL_REACHED":"NO_ACTIVE_GOAL");}
}
static void start(panel*p,uint64_t now){if(p->session.options.enable_drive){snprintf(p->ui.message,sizeof(p->ui.message),"Use authenticated phone START for driving");return;}if(!p->ui.selected){snprintf(p->ui.message,sizeof(p->ui.message),"Select an observed free cell first");return;}goal(p,now);}
static int tick(void*u,uint64_t now){panel*p=u;now=monotonic_at_least(now);if(!p->start)p->start=now;if(now-p->start>=p->session.options.duration_us){p->done=1;return -1;}if((p->last_tick&&now<=p->last_tick)||now<=p->session.view.time_us)return 0;p->last_tick=now;
 phone_goal_poll(p,now);if(p->done)return -1;
 /* 轮询以同一单调时钟记录接收时刻，并将实际经过时间传回消费者；回调入口可能比START接收早数微秒，不能误判为租约时钟倒退。

 */
 now=monotonic_at_least(now);
 if(p->lease.claimed&&p->lease.motion_active&&(now<p->lease.last_seen_us||now-p->lease.last_seen_us>CONT_PHONE_US))cancel_diag(p,"LEASE_TIMEOUT_TICK",now);
 if(phone_nav_lease_tick(&p->lease,now,1,0)==PHONE_NAV_STOP_REQUIRED){cancel(p,now);request_result(p,NULL,now,"REJECTED","PHONE_LINK_EXPIRED_NAVIGATION_STOPPED");snprintf(p->ui.message,sizeof(p->ui.message),"PHONE_LINK_EXPIRED_NAVIGATION_STOPPED");}
 lab_status r=lab_real_session_tick(&p->session,now);
 if(p->session.view.object_mode&&(p->dual_task.arrived||(p->dual_task.explicit_start==0&&p->dual_task.prepared==0))){(void)lab_real_runtime_enable(p->session.runtime,0,now);(void)lab_real_runtime_object_mode(p->session.runtime,0,now);p->arm_request=0;p->lease.motion_active=0;lab_real_runtime_view_get(p->session.runtime,&p->session.view);}
 dual_publish_task(p,monotonic_at_least(now));
 if(p->session.view.time_us>now)now=p->session.view.time_us;
 cruise_preview(p,now);
 obj_lidar_update(p->object_lidar,getenv("ACG_PHONE_DIR"),getenv("ACG_OBJECT_LIDAR_CONFIG"),&p->session.view,&p->session.bindings.localization,&p->session.options.passage,now);
 if(!p->last_object_export||now-p->last_object_export>=500000){p->last_object_export=now;obj_map_export(&p->object_map,getenv("ACG_PHONE_DIR"),getenv("ACG_OBJECTS_CONFIG"),&p->session.view.localization,&p->session.bindings.localization,now);}
 now=monotonic_at_least(now);auto_finalize(p,now);
 phone_goal_poll(p,now);
 now=monotonic_at_least(now);
 if(p->done)return -1;
 tag_task_tick(p,now);object_task_tick(p,now);
 phone_export_map(p,now);
 /* 软停止保留当前实物任务，仅在新有效候选及源数据、反馈条件通过后恢复；不确认清除最终故障，也不构造手机新租约。

 */
 if(p->dual_task.active&&p->dual_task.explicit_start&&!p->dual_task.arrived&&!p->arm_request&&p->lease.claimed&&p->lease.motion_active&&p->lease.last_seen_us<=now&&now-p->lease.last_seen_us<CONT_PHONE_US){
  uint64_t gen=0,revoked=0;const char*directory=getenv("ACG_PHONE_DIR");
  if(directory&&phone_control_fence_read_state(directory,&gen,&revoked)>=0&&p->last_start_sent_us>revoked&&lab_real_runtime_object_rearm_ready(p->session.runtime,now)==LAB_OK){
   lab_status restored=lab_real_runtime_enable(p->session.runtime,1,now);uint32_t exp=p->session.view.experimental_timing;lab_real_runtime_view_get(p->session.runtime,&p->session.view);p->session.view.experimental_timing=exp;
   fprintf(stderr,"OBJECT_SOFT_REARM status=%d reason=%s retained_request=%llu enabled=%u pending=%u\n",restored,p->session.view.reason,(unsigned long long)p->arm_request_id,p->session.view.enabled,p->session.view.pending_enable);
  }
 }
 /* 仅恢复显式且仍持有控制权的手机START，不恢复已取消或过期租约；运行时效及底盘保护条件保留。
 */
 if(!p->dual_task.active&&!p->auto_pending&&!p->arm_request&&navigation_auto_resume_allowed(&p->session.view,p->lease.motion_active,p->lease.claimed)){
  p->arm_request=1;p->arm_deadline=now+3000000u;snprintf(p->ui.message,sizeof(p->ui.message),"RECOVERED_ROUTE_RETRYING_ENABLE");
 }
 now=monotonic_at_least(now);phone_arm_tick(p,now);
 if(p->session.view.time_us>now)now=p->session.view.time_us;
 now=monotonic_at_least(now);
 if(p->cruise_mode&&!p->auto_pending&&p->lease.motion_active){snprintf(p->auto_status,sizeof(p->auto_status),"ACTIVE");snprintf(p->auto_reason,sizeof(p->auto_reason),"%.95s",p->session.view.navigation.reason);}
 mode_publish(p,now);
 phone_nav_motion_export(&p->session.view,getenv("ACG_PHONE_DIR"),now);
 s5_panel_diagnostics(p,now);
 p->navigation_screen=navigation_display_policy(p->navigation_screen,p->session.options.enable_drive,p->lease.motion_active,p->session.view.has_goal,p->session.view.enabled);
 lab_wayland_set_visible(p->navigation_screen||(p->dual_task.active&&p->lease.motion_active));
 if(!p->last_log||now-p->last_log>=1000000){lab_real_print_view(&p->session.view);p->last_log=now;}
 if(r<0&&r!=LAB_ERR_UNAVAILABLE){const char*why=p->session.view.reason;
  if(strstr(why,"SENSOR_")||strstr(why,"LOCALIZER_PUSH")||strstr(why,"CONTEXT")||strstr(why,"CLOCK")||strstr(why,"STOP_TRANSPORT")){fprintf(stderr,"LIVE_NAV_FATAL status=%d reason=%s\n",r,why);return -1;}
  /* 规划失败停止导航，地图与屏幕继续可用。 */
 }
 return 0;
}
static void draw(void*u,uint32_t*pixels,int w,int h,uint64_t now){(void)now;panel*p=u;live_nav_ui_draw(&p->ui,pixels,w,h,&p->session.view,p->session.options.enable_drive);if(p->session.view.object_mode&&p->dual_task.prepared)live_nav_ui_draw_dual(&p->ui,pixels,w,h,&p->session.view,&p->dual_display_scene,&p->dual_task,&p->dual_frame,p->dual_measured_w,p->dual_measured_w_valid&&p->dual_yaw_us<=p->session.view.time_us&&p->session.view.time_us-p->dual_yaw_us<=500000);
 int mx,my;if(lab_wayland_pointer_position(&mx,&my))for(int d=-7;d<=7;++d){if(mx+d>=0&&mx+d<w&&my>=0&&my<h)pixels[(size_t)my*w+mx+d]=0xff3266;if(my+d>=0&&my+d<h&&mx>=0&&mx<w)pixels[(size_t)(my+d)*w+mx]=0xff3266;}}
static void key(void*u,uint32_t code,uint64_t now){panel*p=u;if(code==28)start(p,now);else if(code==57||code==46)cancel(p,now);else if(code==13&&p->ui.span_m>2)p->ui.span_m/=2;else if(code==12&&p->ui.span_m<32)p->ui.span_m*=2;}
static void click(void*u,int x,int y,uint32_t button,uint64_t now){panel*p=u;if(button!=1){cancel_diag(p,"SCREEN_BUTTON",now);cancel(p,now);return;}int b=live_nav_ui_button(x,y);if(b==1)start(p,now);else if(b==2){cancel_diag(p,"SCREEN_STOP",now);cancel(p,now);}else if(b==3&&p->ui.span_m>2)p->ui.span_m/=2;else if(b==4&&p->ui.span_m<32)p->ui.span_m*=2;else if(!b&&live_nav_ui_pick(&p->ui,&p->session.view,x,y)>0){cancel_diag(p,"SCREEN_NEW_GOAL",now);cancel(p,now);goal(p,now);}}
int main(int argc,char**argv){if(argc==2&&!strcmp(argv[1],"--help")){lab_real_usage();return 0;}lab_real_options o;char error[256];if(!lab_real_options_parse(&o,argc,argv,error,sizeof(error))){fprintf(stderr,"%s\n",error);return 2;}
 if(!o.run){puts("PREFLIGHT: no device, socket, Wayland or motor opened. Default preview; screen START required for drive.");return 0;}
 if(!o.mapping_shadow){fprintf(stderr,"Use --mode mapping-navigation or mapping-shadow\n");return 2;}
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);uint64_t now=(uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;lab_real_context ctx={LAB_REAL_HARDWARE,LAB_REAL_MONOTONIC,(uint32_t)(now^(now>>32))|1u,1};panel p={0};object_dual_task_init(&p.dual_task);p.object_lidar=obj_lidar_create();p.goal_fd=-1;live_nav_ui_init(&p.ui);
 lab_status r=lab_real_session_open(&p.session,&o,&ctx,NULL);if(r!=LAB_OK){fprintf(stderr,"OPEN_FAILED: %s\n",lab_status_text(r));return 3;}
 lab_real_runtime_object_controller(p.session.runtime,dual_intent,&p);lab_real_runtime_observe(p.session.runtime,dual_sample_observer,&p);
 p.s5log=s5_log_open(getenv("ACG_S5_LOG_DIR"),getenv("ACG_S5_FS_SHA256"));if(!p.s5log)fprintf(stderr,"S5_LOG_UNAVAILABLE policy_revision=9 reason=NO_WRITER_REALTIME_CONTINUES\n");
 const char*initial_cruise=getenv("ACG_LIDAR_CRUISE");if(initial_cruise&&!strcmp(initial_cruise,"1")&&lab_real_runtime_set_cruise(p.session.runtime,1)==LAB_OK){p.cruise_mode=1;snprintf(p.auto_status,sizeof(p.auto_status),"PREVIEW");snprintf(p.auto_reason,sizeof(p.auto_reason),"COMPATIBILITY_CRUISE_PREVIEW_EXPLICIT_START_REQUIRED");}
 p.goal_fd=phone_goal_open(&p);if(p.goal_fd<0)fprintf(stderr,"PHONE_GOAL_SOCKET_UNAVAILABLE: screen operation remains available\n");
 lab_wayland_set_visible(0);int result=lab_wayland_run_input(o.output,&p,tick,draw,key,click);lab_real_runtime_cancel(p.session.runtime,p.last_tick?p.last_tick:now);
 if(p.goal_fd>=0)phone_goal_socket_close(&goal_socket);
 if(o.save_map[0]&&(r=lab_real_session_save(&p.session))!=LAB_OK){fprintf(stderr,"MAP_NOT_SAVED: %s\n",lab_status_text(r));result=6;}
 obj_lidar_free(p.object_lidar);export_close(&p);s5_log_close(p.s5log);lab_real_session_close(&p.session);return p.done?0:result;
}
