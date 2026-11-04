#define _POSIX_C_SOURCE 200809L
#include "object_lidar.h"
#include "object_lidar_geometry.h"
#include "object_readonly.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif
#define PI 3.14159265358979323846
#define RING 8
#define CANDIDATES 16
typedef struct saved_scan{struct scan_frame scan;uint64_t time,uncertainty;lab_real_context context;}saved_scan;
typedef struct hit{double u,v,x,y,range;int inside,overlap;}hit;
typedef struct cluster{double x,y,range,umin,umax,score,surface_x,surface_y,surface_range,surface_u;unsigned count,points_n,external,overlap;double points[64][2];}cluster;
struct obj_lidar_state{
 obj_lidar_target target;ol_config config;lab_real_context context;uint64_t epoch,map_id,correction,serial,last_frame,last_scan,last_scan_session;
 double anchor_x,anchor_y,anchor_yaw,last_x,last_y,bearing;unsigned support,ring_count,ring_next,count,projected,rejected,flank_left,flank_right;double flank_left_range,flank_right_range;int selected,initialized,snapshot_ok;
 saved_scan scans[RING];hit hits[SCAN_MAX_POINTS];cluster candidates[CANDIDATES];
 uint64_t last_process_us,state_writes,snapshot_writes;obj_lidar_target snapshot_key;int snapshot_key_valid,snapshot_key_available;char*shot;size_t shot_n;char*output;char json[OBJ_LIDAR_JSON_MAX];
};
static uint64_t delta(uint64_t a,uint64_t b){return a>b?a-b:b-a;}
static int context_equal(const lab_real_context*a,const lab_real_context*b){return !memcmp(a,b,sizeof(*a));}
static void status(obj_lidar_state*s,const char*a,const char*b,int reset){s->target.valid=0;snprintf(s->target.status,sizeof(s->target.status),"%s",a);snprintf(s->target.reason,sizeof(s->target.reason),"%s",b);if(reset){s->support=0;s->target.association_id[0]=0;s->count=0;}s->snapshot_ok=0;}
obj_lidar_state*obj_lidar_create(void){obj_lidar_state*s=calloc(1,sizeof(*s));if(s){s->selected=-1;s->target.track_id=-1;status(s,"DISABLED","CONFIG_DISABLED",1);}return s;}
void obj_lidar_free(obj_lidar_state*s){if(s){free(s->shot);free(s->output);free(s);}}
int obj_lidar_get_target(const obj_lidar_state*s,obj_lidar_target*out){if(!s||!out)return 0;*out=s->target;return 1;}
typedef struct writer{char*b;size_t n,cap;int ok;}writer;
static void emit(writer*w,const char*fmt,...){if(!w->ok)return;va_list a;va_start(a,fmt);int n=vsnprintf(w->b+w->n,w->cap-w->n,fmt,a);va_end(a);if(n<0||(size_t)n>=w->cap-w->n){w->ok=0;return;}w->n+=(size_t)n;}
int obj_lidar_json(const obj_lidar_state*s,uint64_t now,char*b,size_t cap,size_t*n){if(!s||!b||!n||cap<2)return 0;writer w={b,0,cap,1};const obj_lidar_target*t=&s->target;int valid=t->valid&&now>=t->frame_us&&now-t->frame_us<=(uint64_t)(s->config.frame_max_age_ms*1000)&&now>=t->scan_us&&now-t->scan_us<=(uint64_t)(s->config.scan_max_age_ms*1000);
 emit(&w,"{\"version\":1,\"epoch\":\"%llu\",\"frame_id\":\"%llu\",\"frame_time_us\":\"%llu\",\"published_us\":\"%llu\",\"scan_time_us\":\"%llu\",\"reference_eye\":%d,\"track_id\":%d,\"map_id\":\"%llu\",\"map_version\":\"%llu\",\"association_id\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\",\"bearing_rad\":",(unsigned long long)t->epoch,(unsigned long long)t->frame_id,(unsigned long long)t->frame_us,(unsigned long long)t->published_us,(unsigned long long)t->scan_us,(int)s->config.reference_eye,t->track_id,(unsigned long long)t->map_id,(unsigned long long)t->map_version,t->association_id,t->valid&&!valid?"STALE":t->status,t->valid&&!valid?"ORIGINAL_OBSERVATION_EXPIRED":t->reason);
 if(valid)emit(&w,"%.9g",s->bearing);else emit(&w,"null");emit(&w,",\"target\":{\"valid\":%s,\"range_m\":",valid?"true":"false");if(valid)emit(&w,"%.9g,\"map_x_m\":%.9g,\"map_y_m\":%.9g,\"position_error_m\":%.9g",t->range_m,t->map_x,t->map_y,t->error_m);else emit(&w,"null,\"map_x_m\":null,\"map_y_m\":null,\"position_error_m\":null");
 emit(&w,"},\"candidates\":[");for(unsigned i=0;i<s->count;i++){const cluster*c=&s->candidates[i];emit(&w,"%s{\"id\":%u,\"range_m\":%.9g,\"score\":%.6g,\"confirmed\":%s,\"points\":[",i?",":"",i+1,c->range,c->score,valid&&i==0?"true":"false");for(unsigned k=0;k<c->points_n;k++)emit(&w,"%s[%.3f,%.3f]",k?",":"",c->points[k][0],c->points[k][1]);emit(&w,"]}");}
 emit(&w,"],\"diagnostics\":{\"distinct_support\":%u,\"scan_history_count\":%u,\"projected_points\":%u,\"rejected_points\":%u,\"scan_uncertainty_us\":\"%llu\",\"snapshot_available\":%s,\"surface_not_center\":true,\"exposure_synchronized\":false,\"heap_state_bytes\":%zu,\"state_writes\":%llu,\"snapshot_writes\":%llu}}",s->support,s->ring_count,s->projected,s->rejected,(unsigned long long)(s->ring_count?s->scans[(s->ring_next+RING-1)%RING].uncertainty:0),s->snapshot_ok?"true":"false",sizeof(*s),(unsigned long long)s->state_writes,(unsigned long long)s->snapshot_writes);*n=w.n;return w.ok&&w.n<=OBJ_LIDAR_JSON_MAX;
}
/* 独立写入器支持合计3MiB上限，旧导出仍为2MiB。 */
static int write_snapshot(const char*path,const char*b,size_t n){if(n>OBJ_LIDAR_SNAPSHOT_MAX)return 0;char tmp[1024];
#ifndef _WIN32
 if(snprintf(tmp,sizeof(tmp),"%s.%ld.tmp",path,(long)getpid())>=(int)sizeof(tmp))return 0;int fd=open(tmp,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)return 0;FILE*f=fdopen(fd,"wb");if(!f){close(fd);unlink(tmp);return 0;}
#else
 if(snprintf(tmp,sizeof(tmp),"%s.tmp",path)>=(int)sizeof(tmp))return 0;FILE*f=fopen(tmp,"wb");if(!f)return 0;
#endif
 int ok=fwrite(b,1,n,f)==n;if(fclose(f))ok=0;
#ifdef _WIN32
 if(ok)remove(path);
#endif
 if(ok&&rename(tmp,path)==0)return 1;remove(tmp);return 0;}
static int publish(obj_lidar_state*s,const char*dir,uint64_t now){char path[1024];size_t n;if(!dir||!obj_lidar_json(s,now,s->json,sizeof(s->json),&n))return 0;if(snprintf(path,sizeof(path),"%s/objects-association.json",dir)>=(int)sizeof(path)||!obj_atomic_write(path,s->json,n))return 0;
 s->state_writes++;obj_lidar_target key=s->target;key.published_us=0;if(s->snapshot_key_valid&&s->snapshot_key_available==s->snapshot_ok&&!memcmp(&key,&s->snapshot_key,sizeof(key)))return 1;
 if(!s->output){s->output=malloc(OBJ_LIDAR_SNAPSHOT_MAX+1u);if(!s->output)return 0;}writer w={s->output,0,OBJ_LIDAR_SNAPSHOT_MAX+1u,1};if(s->snapshot_ok&&s->shot){size_t z=s->shot_n;while(z&&(s->shot[z-1]==' '||s->shot[z-1]=='\r'||s->shot[z-1]=='\n'||s->shot[z-1]=='\t'))z--;if(z<2||s->shot[z-1]!='}')return 0;memcpy(w.b,s->shot,z-1);w.n=z-1;emit(&w,",\"association\":%s}",s->json);}else emit(&w,"{\"version\":1,\"published_us\":\"%llu\",\"status\":\"UNAVAILABLE\",\"reason\":\"EXACT_PAIRED_SNAPSHOT_UNAVAILABLE\",\"association\":%s}",(unsigned long long)now,s->json);
 if(!w.ok||snprintf(path,sizeof(path),"%s/objects-association-snapshot.json",dir)>=(int)sizeof(path))return 0;int ok=write_snapshot(path,w.b,w.n);if(ok){s->snapshot_writes++;s->snapshot_key=key;s->snapshot_key_valid=1;s->snapshot_key_available=s->snapshot_ok;}return ok;
}
static int pose(const lab_localization_binding*b,uint64_t time,const lab_real_context*c,const ol_config*cfg,lab_pose*out,lab_real_meta*m){return time&&b&&b->ops&&b->ops->pose_at&&b->ops->pose_at(b->user,time,m,out)==LAB_OK&&m->time_us==time&&context_equal(c,&m->context)&&(m->flags&(LAB_REAL_TIME_VALID|LAB_REAL_DATA_VALID))==(LAB_REAL_TIME_VALID|LAB_REAL_DATA_VALID)&&m->time_uncertainty_us<=(uint64_t)(cfg->max_time_uncertainty_ms*1000)&&out->calibrated&&isfinite(out->x_m)&&isfinite(out->y_m)&&isfinite(out->yaw_rad)&&isfinite(out->uncertainty_m)&&out->uncertainty_m>=0&&out->uncertainty_m<=cfg->max_pose_uncertainty_m;}
static int compare_hit(const void*a,const void*b){const hit*x=a,*y=b;if(x->u<y->u)return -1;if(x->u>y->u)return 1;return (x->range>y->range)-(x->range<y->range);}
static void append_cluster(obj_lidar_state*s,cluster*c,int*overflow){if(!c->count)return;double span=c->umax-c->umin;if(c->count<(unsigned)s->config.min_cluster_points||span<s->config.min_pixel_span)return;if(s->count==CANDIDATES){*overflow=1;return;}c->x=c->surface_x;c->y=c->surface_y;c->range=c->surface_range;c->score=fmin(1.0,c->count/12.0);s->candidates[s->count++]=*c;}
static void associate(obj_lidar_state*s,const obj_frame*f,const obj_detection*d,const saved_scan*scan,const passage_config*p,const lab_pose*sp,const lab_pose*ip,int*overflow){
 unsigned n=0;s->projected=s->rejected=s->count=s->flank_left=s->flank_right=0;s->flank_left_range=s->flank_right_range=1e9;for(unsigned k=0;k<scan->scan.count;k++){const struct scan_point*q=&scan->scan.points[k];double range=q->distance_q2/4000.0,u,v,x,y;if(!q->quality||!q->distance_q2||range<p->lidar_min_m||range>p->lidar_max_m||!ol_project(&s->config,p,sp,ip,range,-q->angle_q14*(PI/32768.0),&u,&v,&x,&y)){s->rejected++;continue;}s->projected++;double m=s->config.bbox_margin_px;if(u<d->bbox[0]-m||u>d->bbox[2]+m||v<d->bbox[1]||v>d->bbox[3])continue;int in=u>=d->bbox[0]&&u<=d->bbox[2],overlap=0;if(u<d->bbox[0]){s->flank_left++;s->flank_left_range=fmin(s->flank_left_range,range);}if(u>d->bbox[2]){s->flank_right++;s->flank_right_range=fmin(s->flank_right_range,range);}for(int z=0;z<f->count;z++){const obj_detection*o=&f->objects[z];if(o->track_id!=d->track_id&&o->observed&&o->bbox_valid&&u>=o->bbox[0]&&u<=o->bbox[2]&&v>=o->bbox[1]&&v<=o->bbox[3])overlap=1;}s->hits[n++]=(hit){u,v,x,y,range,in,overlap};}
 qsort(s->hits,n,sizeof(hit),compare_hit);cluster c={0};hit*prev=NULL;for(unsigned i=0;i<n;i++){hit*h=&s->hits[i];if(prev&&hypot(prev->x-h->x,prev->y-h->y)>s->config.cluster_gap_m){append_cluster(s,&c,overflow);memset(&c,0,sizeof(c));}if(h->inside){double center=(d->bbox[0]+d->bbox[2])*.5;if(!c.count||fabs(h->u-center)<fabs(c.surface_u-center)){c.surface_x=h->x;c.surface_y=h->y;c.surface_range=h->range;c.surface_u=h->u;}if(!c.count)c.umin=c.umax=h->u;else{c.umin=fmin(c.umin,h->u);c.umax=fmax(c.umax,h->u);}c.count++;c.x+=h->x;c.y+=h->y;c.range+=h->range;c.overlap+=h->overlap;if(c.points_n<64){c.points[c.points_n][0]=h->u;c.points[c.points_n++][1]=h->v;}}else c.external++;prev=h;}append_cluster(s,&c,overflow);
}
static void collect_scan(obj_lidar_state*s,const lab_real_runtime_view*v){ if(v->has_scan&&v->latest_scan&&v->latest_scan->count&&v->latest_scan->count<=SCAN_MAX_POINTS&&v->scan_time_us){const struct scan_frame*q=v->latest_scan;int duplicate=0;for(unsigned k=0;k<s->ring_count;k++)if(s->scans[k].scan.session==q->session&&s->scans[k].scan.sequence==q->sequence)duplicate=1;if(!duplicate){if(s->ring_count&&s->last_scan_session!=q->session){s->ring_count=s->ring_next=0;s->last_frame=s->last_scan=0;status(s,"CANDIDATE","SCAN_SESSION_CHANGED",1);}saved_scan*t=&s->scans[s->ring_next];t->scan=*q;t->time=v->scan_time_us;t->uncertainty=v->scan_uncertainty_us;t->context=v->context;s->ring_next=(s->ring_next+1)%RING;if(s->ring_count<RING)s->ring_count++;s->last_scan_session=q->session;}}}
static void remap_anchor(obj_lidar_state*s,const lab_localization_view*l){
 if(!isfinite(l->map_odom_x_m)||!isfinite(l->map_odom_y_m)||!isfinite(l->map_odom_yaw_rad)){status(s,"UNAVAILABLE","INVALID_MAP_ODOM_TRANSFORM",1);return;}
 if((s->anchor_x!=l->map_odom_x_m||s->anchor_y!=l->map_odom_y_m||s->anchor_yaw!=l->map_odom_yaw_rad)){
  double a=l->map_odom_yaw_rad-s->anchor_yaw,cs=cos(a),sn=sin(a),dx=s->last_x-s->anchor_x,dy=s->last_y-s->anchor_y;
  if(s->support){s->last_x=l->map_odom_x_m+cs*dx-sn*dy;s->last_y=l->map_odom_y_m+sn*dx+cs*dy;}
  dx=s->target.map_x-s->anchor_x;dy=s->target.map_y-s->anchor_y;s->target.map_x=l->map_odom_x_m+cs*dx-sn*dy;s->target.map_y=l->map_odom_y_m+sn*dx+cs*dy;
 }
 s->anchor_x=l->map_odom_x_m;s->anchor_y=l->map_odom_y_m;s->anchor_yaw=l->map_odom_yaw_rad;s->correction=l->map.correction_version;s->target.map_version=l->map.map_version;
}
int obj_lidar_update(obj_lidar_state*s,const char*dir,const char*config,const lab_real_runtime_view*v,const lab_localization_binding*b,const passage_config*p,uint64_t now){
 if(!s||!v||!dir)return 0;if(!s->serial)s->serial=now;
 if(s->initialized&&context_equal(&s->context,&v->context)&&s->map_id==v->localization.map.map_id){collect_scan(s,v);remap_anchor(s,&v->localization);}
 if(s->last_process_us&&now>=s->last_process_us&&now-s->last_process_us<100000&&(!s->initialized||(context_equal(&s->context,&v->context)&&s->map_id==v->localization.map.map_id)))return 1;
 s->last_process_us=now;s->target.published_us=now;ol_config cfg;if(!ol_config_load(config,&cfg)){status(s,"CALIBRATION_REQUIRED","INVALID_OR_UNVERIFIED_CALIBRATION",1);return publish(s,dir,now);}if(memcmp(&cfg,&s->config,sizeof(cfg))){s->config=cfg;status(s,"CANDIDATE","CONFIG_CHANGED_RECONFIRM",1);s->last_frame=s->last_scan=0;}
 if(!cfg.enabled){status(s,"DISABLED","CONFIG_DISABLED",1);return publish(s,dir,now);}if(!p||!p->mount_verified||!passage_config_valid(p)){status(s,"CALIBRATION_REQUIRED","LIDAR_MOUNT_NOT_VERIFIED",1);return publish(s,dir,now);}
 const lab_localization_view*l=&v->localization;int reset=!s->initialized||!context_equal(&s->context,&v->context)||s->map_id!=l->map.map_id;
 if(!isfinite(l->map_odom_x_m)||!isfinite(l->map_odom_y_m)||!isfinite(l->map_odom_yaw_rad)){status(s,"UNAVAILABLE","INVALID_MAP_ODOM_TRANSFORM",1);return publish(s,dir,now);}

 if(reset){s->initialized=1;s->context=v->context;s->map_id=l->map.map_id;s->correction=l->map.correction_version;s->anchor_x=l->map_odom_x_m;s->anchor_y=l->map_odom_y_m;s->anchor_yaw=l->map_odom_yaw_rad;s->ring_count=s->ring_next=0;s->last_frame=s->last_scan=0;status(s,"CANDIDATE","CONTEXT_OR_MAP_CHANGED_RECONFIRM",1);}
 s->anchor_x=l->map_odom_x_m;s->anchor_y=l->map_odom_y_m;s->anchor_yaw=l->map_odom_yaw_rad;s->correction=l->map.correction_version;
 s->target.context=v->context;s->target.map_id=l->map.map_id;s->target.map_version=l->map.map_version;
 if(!v->has_localization||!l->has_pose||(l->state!=LAB_REAL_LOC_TRACKING&&!(l->state==LAB_REAL_LOC_DEGRADED&&l->odometry_fallback))||!l->map.map_id||!context_equal(&l->context,&v->context)||!context_equal(&l->map.context,&v->context)){status(s,"UNAVAILABLE","LOCALIZATION_MAP_CONTEXT_UNAVAILABLE",1);return publish(s,dir,now);}
 collect_scan(s,v);
 char path[1024];obj_frame frame;if(snprintf(path,sizeof(path),"%s/objects-state.json",dir)>=(int)sizeof(path)||!obj_frame_load(path,&frame)){status(s,"UNAVAILABLE","OBJECT_FRAME_UNAVAILABLE",1);return publish(s,dir,now);}
 if(s->epoch!=frame.epoch||s->selected!=frame.selected){s->epoch=frame.epoch;s->selected=frame.selected;s->last_frame=s->last_scan=0;status(s,"WAIT_TARGET","SELECTION_OR_EPOCH_CHANGED",1);}
 s->target.epoch=frame.epoch;s->target.track_id=frame.selected;
 if(frame.selected<=0){status(s,"WAIT_TARGET","NO_SELECTED_TRACK",1);return publish(s,dir,now);}if(!frame.epoch||!frame.frame_id||!frame.frame_us||frame.frame_us>now||now-frame.frame_us>(uint64_t)(cfg.frame_max_age_ms*1000)||frame.generated_us>now||frame.generated_us<frame.frame_us){status(s,"STALE","ORIGINAL_FRAME_STALE",1);return publish(s,dir,now);}
 if((s->last_frame&&frame.frame_id<s->last_frame)||(s->target.frame_us&&s->last_frame&&frame.frame_us<s->target.frame_us)){status(s,"STALE","NON_MONOTONIC_ORIGINAL_FRAME",1);return publish(s,dir,now);}
 const obj_detection*d=NULL;for(int k=0;k<frame.count;k++)if(frame.objects[k].track_id==frame.selected)d=&frame.objects[k];if(!d||!d->observed||!d->confirmed||strcmp(d->tracking,"CONFIRMED")){status(s,"LOST","SELECTED_TRACK_NOT_FRESH_OBSERVED_CONFIRMED",1);return publish(s,dir,now);}if(!d->bbox_valid||d->score<.5||frame.eye!=(int)cfg.reference_eye){status(s,"UNAVAILABLE","BBOX_SCORE_OR_REFERENCE_EYE_INVALID",1);return publish(s,dir,now);}
 uint64_t delay=(uint64_t)(cfg.camera_delay_ms*1000);if(frame.frame_us<=delay){status(s,"UNAVAILABLE","IMAGE_DELAY_UNDERFLOW",1);return publish(s,dir,now);}uint64_t image_time=frame.frame_us-delay;const saved_scan*best=NULL;uint64_t separation=UINT64_MAX;for(unsigned k=0;k<s->ring_count;k++){const saved_scan*q=&s->scans[k];if(q->time>now||now-q->time>(uint64_t)(cfg.scan_max_age_ms*1000)||q->uncertainty>(uint64_t)(cfg.max_time_uncertainty_ms*1000)||!context_equal(&q->context,&v->context))continue;uint64_t dt=delta(image_time,q->time);if(dt<separation){best=q;separation=dt;}}
 if(!best||separation>(uint64_t)(cfg.pair_max_delta_ms*1000)){status(s,"UNAVAILABLE","NO_FRESH_SCAN_WITHIN_PAIR_WINDOW",1);return publish(s,dir,now);}
 if(s->last_scan&&best->time<s->last_scan){status(s,"STALE","NON_MONOTONIC_SCAN_PAIR",1);return publish(s,dir,now);}
 lab_pose sp,ip;lab_real_meta sm,im;if(!pose(b,best->time,&v->context,&cfg,&sp,&sm)||!pose(b,image_time,&v->context,&cfg,&ip,&im)||separation+best->uncertainty+sm.time_uncertainty_us+im.time_uncertainty_us>(uint64_t)(cfg.pair_max_delta_ms*1000)){status(s,"UNAVAILABLE","HISTORICAL_POSE_OR_TIME_UNCERTAINTY_INVALID",1);return publish(s,dir,now);}
 if(frame.frame_id==s->last_frame||best->time==s->last_scan){if(s->target.frame_us>now||now-s->target.frame_us>(uint64_t)(cfg.frame_max_age_ms*1000)){status(s,"STALE","ORIGINAL_ASSOCIATION_EXPIRED",1);}return publish(s,dir,now);}
 s->last_frame=frame.frame_id;s->last_scan=best->time;s->target.frame_id=frame.frame_id;s->target.frame_us=frame.frame_us;s->target.scan_us=best->time;
 char*shot=NULL;size_t shot_n=0;if(snprintf(path,sizeof(path),"%s/objects-snapshot.json",dir)>=(int)sizeof(path)||!obj_file_read(path,OBJ_SNAPSHOT_MAX,&shot,&shot_n)||!obj_snapshot_matches(shot,shot_n,&frame)){free(shot);status(s,"UNAVAILABLE","EXACT_SAME_FRAME_SNAPSHOT_REQUIRED",1);return publish(s,dir,now);}free(s->shot);s->shot=shot;s->shot_n=shot_n;
 int overflow=0;associate(s,&frame,d,best,p,&sp,&ip,&overflow);if(!s->count){status(s,"UNAVAILABLE","NO_SUPPORTED_PROJECTED_SURFACE",1);s->snapshot_ok=1;return publish(s,dir,now);}if(overflow||s->count>1){s->support=0;s->target.association_id[0]=0;status(s,"AMBIGUOUS","COMPETING_SURFACE_CLUSTERS",0);s->snapshot_ok=1;return publish(s,dir,now);}cluster*c=&s->candidates[0];if(c->external>=2||c->overlap||(c->umax-c->umin)/(d->bbox[2]-d->bbox[0])<cfg.min_bbox_fraction){s->support=0;s->target.association_id[0]=0;status(s,"AMBIGUOUS",c->external>=2?"SURFACE_CONTINUES_OUTSIDE_BBOX":"OVERLAP_OR_INSUFFICIENT_OBJECT_SUPPORT",0);s->snapshot_ok=1;return publish(s,dir,now);}
 if(s->flank_left<2||s->flank_right<2||s->flank_left_range<c->range+2*cfg.cluster_gap_m||s->flank_right_range<c->range+2*cfg.cluster_gap_m){s->support=0;s->target.association_id[0]=0;status(s,"AMBIGUOUS","INSUFFICIENT_FOREGROUND_BOUNDARY_EVIDENCE",0);s->snapshot_ok=1;return publish(s,dir,now);}
 if(!s->support||hypot(c->x-s->last_x,c->y-s->last_y)>cfg.temporal_gate_m){s->support=1;s->target.association_id[0]=0;}else if(s->support<20)s->support++;s->last_x=c->x;s->last_y=c->y;s->target.map_x=c->x;s->target.map_y=c->y;s->target.range_m=hypot(c->x-ip.x_m,c->y-ip.y_m);s->target.error_m=sp.uncertainty_m+ip.uncertainty_m+cfg.cluster_gap_m+((.25+1.1*(s->target.range_m+hypot(p->mount_x_m,p->mount_y_m)+hypot(cfg.camera_x_m,cfg.camera_y_m)))*(best->uncertainty+sm.time_uncertainty_us+im.time_uncertainty_us)/1000000.0);s->bearing=atan2(c->y-ip.y_m,c->x-ip.x_m)-ip.yaw_rad;s->bearing=atan2(sin(s->bearing),cos(s->bearing));
 if(s->support>=(unsigned)cfg.confirm_frames){if(!s->target.association_id[0])snprintf(s->target.association_id,sizeof(s->target.association_id),"%llu-%d-%llu-%u-%llu",(unsigned long long)frame.epoch,frame.selected,(unsigned long long)l->map.map_id,v->context.origin_epoch,(unsigned long long)++s->serial);s->target.valid=1;s->target.confirmed_us=now;snprintf(s->target.status,sizeof(s->target.status),"CONFIRMED");snprintf(s->target.reason,sizeof(s->target.reason),"DISTINCT_IMAGE_SCAN_SURFACE_SUPPORT");}else status(s,"CANDIDATE","DISTINCT_IMAGE_SCAN_CONFIRMATION_PENDING",0);s->snapshot_ok=1;return publish(s,dir,now);
}
