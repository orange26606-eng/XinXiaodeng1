#include "lab_localization.h"
#include "localization_internal.h"
#include "localization_grid_private.h"
#include "localization_range.h"
#include <math.h>
#include "continuous_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 候选 ABI 构建导出私有工厂符号；正常构建导出 lab_localization_create。 */
#ifdef W3_REAL_ABI_CANDIDATE
#define W3_FACTORY w3_localization_candidate_create
#else
#define W3_FACTORY lab_localization_create
#endif

typedef struct real_history {lab_real_meta meta;lab_pose pose;} real_history;
#define RECOVERY_YAW_BINS 24u
#define RECOVERY_XY_BINS 9u
#define RECOVERY_BUCKETS (RECOVERY_YAW_BINS*9u)
#define RECOVERY_TRIALS (RECOVERY_YAW_BINS*RECOVERY_XY_BINS*RECOVERY_XY_BINS)
typedef struct recovery_job {
 wl_point points[W3_LOC_POINTS_MAX];uint32_t point_count,cursor;
 w3_loc_pose prior,raw,best,buckets[RECOVERY_BUCKETS];double scores[RECOVERY_BUCKETS];
 uint64_t scan_time_us,map_version;int ready;
} recovery_job;
typedef struct real_localization {
    lab_localization_config config;lab_real_context context;
    w3_localizer *core;wl_grid *grid;w3_loc_config matcher;
    real_history history[LAB_REAL_HISTORY_MAX];uint32_t history_count;
    lab_real_meta pose_stream,scan_stream,pending_meta;
    struct scan_frame *pending;int have_pose_stream,have_scan_stream,have_pending;
    lab_pose manual_pose;uint64_t manual_time,received_us;
    int have_manual,have_delivery,blocked,have_evaluation_time;
    uint64_t evaluation_time_us;
    uint64_t scans,matched,rejected,last_matched_us,last_matched_uncertainty_us;
    int recovering;unsigned recovery_matches;uint64_t recovery_attempt_us;
    w3_loc_pose recovery_map,recovery_odom;
    recovery_job recovery_job;
    uint32_t recovery_state,recovery_attempts,recovery_ambiguous,recovery_failures;
    double recovery_best_score,recovery_second_score;
    uint64_t recovery_candidate_time;
    lab_real_xy trace[LAB_TRACE_MAX];uint32_t trace_count;
    w3_loc_pose last_keyframe;int have_keyframe;
    w3_loc_pose seed_odom,seed_map;int have_seed;
    uint64_t last_grid_time,correction_version;
    w3_loc_pose fallback_transform;int have_fallback;uint64_t fallback_time,fallback_uncertainty;
    char reason[64];
} real_localization;
lab_status lab_real_localization_set_sensor_range(lab_localization_binding*b,double minimum,double maximum){
 if(!b||!b->user||!isfinite(minimum)||!isfinite(maximum)||minimum<.03||maximum<=minimum||maximum>30)return LAB_ERR_ARGUMENT;
 real_localization*s=b->user;if(s->core||s->scans)return LAB_ERR_CONTEXT;s->matcher.min_range_m=minimum;s->matcher.max_range_m=maximum;return LAB_OK;
}

static void reason(real_localization *s,const char *value) {snprintf(s->reason,sizeof(s->reason),"%s",value);}
static int same_stream(const lab_real_meta *a,const lab_real_meta *b) {
    return a->provider==b->provider&&a->stream_id==b->stream_id&&a->stream_session==b->stream_session;
}
static int stale(uint64_t now,uint64_t stamp,uint64_t uncertainty,uint64_t limit) {
    return now<stamp||now-stamp>=limit||uncertainty>=limit-(now-stamp);
}
static uint64_t evaluation_time(const real_localization *s) {
    return s->have_evaluation_time&&s->evaluation_time_us>s->received_us?s->evaluation_time_us:s->received_us;
}
static int usable_pose(const real_localization *s,const real_history *h) {
    return (h->meta.flags&3u)==3u&&(h->pose.quality&LAB_POSE_VALID)&&!(h->pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))&&
        (s->config.allow_uncalibrated||(h->pose.calibrated&&(h->meta.flags&LAB_REAL_CALIBRATED)))&&
        h->meta.time_uncertainty_us<=CONT_TIME_UNCERTAINTY_US;
}
static w3_loc_context private_context(lab_real_context c) {
    if(c.source==LAB_REAL_HARDWARE)return (w3_loc_context){W3_LOC_REAL,W3_LOC_RK_MONOTONIC,c.timeline_id,c.origin_epoch};
    if(c.source==LAB_REAL_RECORDING)return (w3_loc_context){W3_LOC_RECORDED_REAL,W3_LOC_REPLAY_CLOCK,c.timeline_id,c.origin_epoch};
    return (w3_loc_context){W3_LOC_SYNTHETIC_TEST,W3_LOC_TEST_CLOCK,c.timeline_id,c.origin_epoch};
}
static w3_loc_meta private_meta(const real_localization *s,const lab_real_meta *m,int pose) {
    int fixture=s->context.source==LAB_REAL_TEST_FIXTURE;
    uint32_t provider=pose?(fixture?LAB_PROVIDER_SIM_ESTIMATE:LAB_PROVIDER_ODOMETRY):
        (fixture?LAB_PROVIDER_SIM_SENSOR:(m->provider==LAB_REAL_FPGA?LAB_PROVIDER_FPGA:LAB_PROVIDER_USB));
    return (w3_loc_meta){private_context(s->context),provider,m->stream_id,m->stream_session,m->sequence,m->flags&3u,m->time_us,m->received_us};
}
static lab_status raw_at(const real_localization *s,uint64_t t,lab_real_meta *meta,lab_pose *out) {
    static uint64_t last_raw_wait_t;
    uint32_t b=0;while(b<s->history_count&&s->history[b].meta.time_us<t)++b;
    if(b==s->history_count){if(t!=last_raw_wait_t)fprintf(stderr,"RAW_ALIGN_WAIT reason=NO_FUTURE target_us=%llu first_us=%llu last_us=%llu count=%u\n",(unsigned long long)t,(unsigned long long)(s->history_count?s->history[0].meta.time_us:0),(unsigned long long)(s->history_count?s->history[s->history_count-1].meta.time_us:0),s->history_count);last_raw_wait_t=t;return LAB_AGAIN;}uint32_t a=b;
    if(s->history[b].meta.time_us!=t){if(!b){if(t!=last_raw_wait_t)fprintf(stderr,"RAW_ALIGN_WAIT reason=NO_OLDER target_us=%llu first_us=%llu count=%u\n",(unsigned long long)t,(unsigned long long)s->history[0].meta.time_us,s->history_count);last_raw_wait_t=t;return LAB_AGAIN;}a=b-1;
        if(s->history[b].meta.time_us-s->history[a].meta.time_us>s->config.max_interpolation_gap_us){if(t!=last_raw_wait_t)fprintf(stderr,"RAW_ALIGN_WAIT reason=INTERPOLATION_GAP target_us=%llu before_us=%llu after_us=%llu gap_us=%llu limit_us=%llu\n",(unsigned long long)t,(unsigned long long)s->history[a].meta.time_us,(unsigned long long)s->history[b].meta.time_us,(unsigned long long)(s->history[b].meta.time_us-s->history[a].meta.time_us),(unsigned long long)s->config.max_interpolation_gap_us);last_raw_wait_t=t;return LAB_AGAIN;}}
    if(!usable_pose(s,&s->history[a])||!usable_pose(s,&s->history[b])){if(t!=last_raw_wait_t)fprintf(stderr,"RAW_ALIGN_WAIT reason=UNUSABLE_BRACKET target_us=%llu before_us=%llu after_us=%llu before_quality=0x%x after_quality=0x%x before_flags=0x%x after_flags=0x%x before_unc_us=%llu after_unc_us=%llu before_cal=%u after_cal=%u\n",(unsigned long long)t,(unsigned long long)s->history[a].meta.time_us,(unsigned long long)s->history[b].meta.time_us,s->history[a].pose.quality,s->history[b].pose.quality,s->history[a].meta.flags,s->history[b].meta.flags,(unsigned long long)s->history[a].meta.time_uncertainty_us,(unsigned long long)s->history[b].meta.time_uncertainty_us,s->history[a].pose.calibrated,s->history[b].pose.calibrated);last_raw_wait_t=t;return LAB_AGAIN;}
    *out=s->history[a].pose;*meta=s->history[b].meta;meta->time_us=t;
    meta->received_us=s->history[a].meta.received_us>s->history[b].meta.received_us?s->history[a].meta.received_us:s->history[b].meta.received_us;
    meta->time_uncertainty_us=s->history[a].meta.time_uncertainty_us>s->history[b].meta.time_uncertainty_us?
        s->history[a].meta.time_uncertainty_us:s->history[b].meta.time_uncertainty_us;
    if(a!=b) {
        double u=(double)(t-s->history[a].meta.time_us)/(s->history[b].meta.time_us-s->history[a].meta.time_us);lab_pose q=s->history[b].pose;
        out->x_m+=u*(q.x_m-out->x_m);out->y_m+=u*(q.y_m-out->y_m);out->yaw_rad=wl_wrap(out->yaw_rad+u*wl_wrap(q.yaw_rad-out->yaw_rad));
        out->distance_m+=u*(q.distance_m-out->distance_m);out->quality|=q.quality;out->calibrated&=q.calibrated;out->uncertainty_m=fmax(out->uncertainty_m,q.uncertainty_m);
        meta->flags|=s->history[a].meta.flags&LAB_REAL_TIME_ESTIMATED;
    }
    if(!out->calibrated)out->quality|=LAB_POSE_UNCALIBRATED;
    return LAB_OK;
}
static lab_status remember(real_localization *s,const lab_live_sample *sample) {
    const lab_real_meta *m=&sample->meta;
    if(s->have_pose_stream&&!same_stream(&s->pose_stream,m))return LAB_ERR_CONTEXT;
    if(s->have_pose_stream){uint32_t difference=m->sequence-s->pose_stream.sequence;
        if(!difference||difference>=UINT32_C(0x80000000))return LAB_ERR_TIME;}
    uint32_t at=0;while(at<s->history_count&&s->history[at].meta.time_us<m->time_us)++at;
    if(at<s->history_count&&s->history[at].meta.time_us==m->time_us)return LAB_ERR_TIME;
    if(s->history_count==LAB_REAL_HISTORY_MAX){if(!at)return LAB_ERR_TIME;
        memmove(s->history,s->history+1,(LAB_REAL_HISTORY_MAX-1)*sizeof(*s->history));--s->history_count;--at;}
    memmove(s->history+at+1,s->history+at,(s->history_count-at)*sizeof(*s->history));
    s->history[at]=(real_history){*m,sample->data.odometry};++s->history_count;s->pose_stream=*m;s->have_pose_stream=1;return LAB_OK;
}
static void trace_pose(real_localization *s,w3_loc_pose p) {
    if(s->trace_count){lab_real_xy last=s->trace[s->trace_count-1];if(hypot(last.x_m-p.x_m,last.y_m-p.y_m)<.02)return;}
    if(s->trace_count==LAB_TRACE_MAX){memmove(s->trace,s->trace+1,(LAB_TRACE_MAX-1)*sizeof(*s->trace));--s->trace_count;}
    s->trace[s->trace_count++]=(lab_real_xy){p.x_m,p.y_m};
}
static lab_status initialize_core(real_localization *s,const lab_real_meta *m,const lab_pose *raw,w3_loc_pose map_pose) {
    w3_loc_meta pm=private_meta(s,m,1);lab_status r=w3_loc_initialize(s->core,&map_pose,&pm,raw);
    if(r!=LAB_OK)return r;
    /* 当前支持数据可晚于手动初始化时刻，但须已经实际到达，不构造未来数据或回填交付时间。
 */
    for(uint32_t i=0;i<s->history_count;++i)if(s->history[i].meta.time_us>m->time_us) {
        w3_loc_meta q=private_meta(s,&s->history[i].meta,1);q.received_us=s->received_us;
        r=w3_loc_push_pose(s->core,&q,&s->history[i].pose);
        if(r<0)return r;
    }
    s->have_manual=0;return LAB_OK;
}
static lab_status try_manual(real_localization *s) {
    if(!s->core||!s->have_manual)return LAB_OK;
    lab_real_meta m;lab_pose p;lab_status r=raw_at(s,s->manual_time,&m,&p);
    if(r!=LAB_OK){reason(s,"INITIAL_POSE_WAIT_ODOMETRY");return LAB_AGAIN;}
    w3_loc_pose prior={s->manual_pose.x_m,s->manual_pose.y_m,s->manual_pose.yaw_rad};
    r=initialize_core(s,&m,&p,prior);if(r==LAB_OK)reason(s,"INITIAL_POSE_WAIT_SCAN");return r;
}
/* 恢复可在运动中使用扫描时刻对齐的真实里程计进行。 */
static uint32_t recovery_points(const real_localization*s,const struct scan_frame*f,wl_point*p){
 uint32_t n=0,cap=s->matcher.points_per_scan;double cs=cos(s->matcher.mount_yaw_rad),sn=sin(s->matcher.mount_yaw_rad);
 for(uint32_t i=0;i<cap&&i<f->count;i++){uint32_t j=f->count>cap?(uint32_t)((uint64_t)i*f->count/cap):i;const struct scan_point*q=&f->points[j];double d=q->distance_q2/4000.0;if(!scan_return_in_range(f,j,s->matcher.min_range_m,s->matcher.max_range_m))continue;double a=-q->angle_q14*(6.2831853071795864769/65536),x=d*cos(a),y=d*sin(a);p[n++]=(wl_point){s->matcher.mount_x_m+cs*x-sn*y,s->matcher.mount_y_m+sn*x+cs*y};}return n;
}
/* 候选搜索跨回调推进，共用不可变实际扫描，不直接写入接受位姿。最佳明确候选建立新先验后，仍需两次新扫描质量确认。

 */
static void recovery_work(real_localization*s,unsigned budget){
 if(!s->core||s->recovery_state!=2)return;recovery_job*j=&s->recovery_job;
 if(j->map_version!=s->core->map.map_version){s->recovery_state=1;reason(s,"RECOVERY_REFERENCE_CHANGED");return;}
 while(budget--&&j->cursor<RECOVERY_TRIALS){unsigned at=j->cursor++,angle=at/81,xy=at%81,xx=xy/9,yy=xy%9;w3_loc_pose p={j->prior.x_m+((int)xx-4)*.15,j->prior.y_m+((int)yy-4)*.15,wl_wrap(j->prior.yaw_rad+((int)angle-12)*(3.14159265358979323846/12))};
  double score=wl_recovery_score(s->core,j->points,j->point_count,p);unsigned bx=xx<3?0:xx>5?2:1,by=yy<3?0:yy>5?2:1,b=angle*9+bx*3+by;if(score<j->scores[b]){j->scores[b]=score;j->buckets[b]=p;}
 }
 if(j->cursor<RECOVERY_TRIALS)return;unsigned best=0;for(unsigned i=1;i<RECOVERY_BUCKETS;i++)if(j->scores[i]<j->scores[best])best=i;j->best=j->buckets[best];s->recovery_best_score=j->scores[best];s->recovery_second_score=1e9;
 for(unsigned i=0;i<RECOVERY_BUCKETS;i++){if(j->scores[i]>=1e8)continue;if(hypot(j->buckets[i].x_m-j->best.x_m,j->buckets[i].y_m-j->best.y_m)>.35||fabs(wl_wrap(j->buckets[i].yaw_rad-j->best.yaw_rad))>.30)s->recovery_second_score=fmin(s->recovery_second_score,j->scores[i]);}
 int ambiguous=s->recovery_second_score-s->recovery_best_score<=fmax(.00015,.12*s->recovery_best_score);
 fprintf(stderr,"RECOVERY_SEARCH_DONE attempt=%u trials=%u best_score=%.7f second_score=%.7f ambiguous=%d proposal=%.3f,%.3f,%.4f scan_time_us=%llu reference=%llu\n",s->recovery_attempts,j->cursor,s->recovery_best_score,s->recovery_second_score,ambiguous,j->best.x_m,j->best.y_m,j->best.yaw_rad,(unsigned long long)j->scan_time_us,(unsigned long long)j->map_version);
 /* 15度粗采样在局部细化前残差可能较大，此处只拒绝平坦或无参考情况，不提前判断最终位姿质量。
 */
 if(ambiguous||s->recovery_best_score>=.02239){s->recovery_state=1;j->ready=0;if(ambiguous)++s->recovery_ambiguous;reason(s,ambiguous?"RECOVERY_AMBIGUOUS":"RECOVERY_LOW_SUPPORT");return;}
 s->recovery_state=3;j->ready=1;s->recovery_failures=0;reason(s,"RECOVERY_CANDIDATE_READY");
}
static lab_status try_recover(real_localization*s,const lab_real_meta*m,const lab_pose*raw){
 if(!s->core||s->blocked)return LAB_AGAIN;
 if(!s->recovering){if(!s->core->have_success)return LAB_AGAIN;s->recovering=1;s->recovery_map=s->core->last_map;s->recovery_odom=s->core->last_odom;s->recovery_attempt_us=0;s->recovery_state=1;}
 if(s->recovery_state==3&&s->recovery_job.ready){w3_loc_pose current={raw->x_m,raw->y_m,raw->yaw_rad};w3_loc_pose prior=wl_compose(s->recovery_job.best,wl_relative(s->recovery_job.raw,current));lab_real_meta now=*m;now.received_us=s->received_us;lab_status r=initialize_core(s,&now,raw,prior);s->recovery_job.ready=0;s->recovery_candidate_time=m->time_us;s->recovery_matches=0;reason(s,r==LAB_OK?"RECOVERY_CONFIRMING":"RECOVERY_PRIOR_REJECTED");if(r!=LAB_OK)s->recovery_state=1;return r;}
 if(s->recovery_state==3){if(s->recovery_failures<3&&m->time_us-s->recovery_candidate_time<1500000)return LAB_OK;s->recovery_state=1;reason(s,"RECOVERY_CANDIDATE_REJECTED");}
 if(s->recovery_state==2){reason(s,"RECOVERY_COARSE_SEARCH");return LAB_AGAIN;}
 if(s->recovery_attempt_us&&m->time_us-s->recovery_attempt_us<1000000u){reason(s,"RECOVERY_RETRY_WAIT");return LAB_AGAIN;}
 recovery_job*j=&s->recovery_job;memset(j,0,sizeof(*j));j->point_count=recovery_points(s,s->pending,j->points);if(j->point_count<s->matcher.min_points){reason(s,"RECOVERY_TOO_FEW_POINTS");return LAB_AGAIN;}
 j->raw=(w3_loc_pose){raw->x_m,raw->y_m,raw->yaw_rad};j->prior=wl_compose(s->recovery_map,wl_relative(s->recovery_odom,j->raw));j->scan_time_us=m->time_us;j->map_version=s->core->map.map_version;for(unsigned i=0;i<RECOVERY_BUCKETS;i++)j->scores[i]=1e9;
 ++s->recovery_attempts;s->recovery_attempt_us=m->time_us;s->recovery_matches=0;s->recovery_failures=0;s->recovery_state=2;reason(s,"RECOVERY_COARSE_SEARCH");fprintf(stderr,"RECOVERY_SEARCH_BEGIN attempt=%u trials=%u xy_m=.60 yaw_rad=3.141593 budget_per_callback=16 scan_time_us=%llu\n",s->recovery_attempts,RECOVERY_TRIALS,(unsigned long long)m->time_us);return LAB_AGAIN;
}
static void rollback_seed(real_localization *s) {
    size_t n=(size_t)s->grid->view.width*s->grid->view.height;
    memset(s->grid->cells,-1,n);memset(s->grid->scores,0,n*sizeof(*s->grid->scores));
    memset(s->grid->hit_count,0,n);memset(s->grid->hit_epoch,0,n*sizeof(*s->grid->hit_epoch));s->have_seed=0;
    s->grid->view.map_version=1;s->grid->integrated=s->grid->invalid_points=s->grid->outside_endpoints=0;
    s->grid->have_scan=0;s->grid->last_scan_session=s->grid->last_scan_sequence=0;
    w3_loc_close(s->core);s->core=NULL;s->have_pending=0;
}
static lab_status process_pending(real_localization *s) {
    if(!s->have_pending)return LAB_OK;
    lab_real_meta aligned;lab_pose raw;lab_status r=raw_at(s,s->pending_meta.time_us,&aligned,&raw);
    if(r!=LAB_OK){reason(s,"SCAN_WAIT_ALIGNED_ODOMETRY");return LAB_AGAIN;}
    if(s->config.mode==LAB_REAL_BUILD_MAP&&!s->core) {
        /* 新地图在首个有支持扫描处建立明确局部参考，此时原始里程计可能已远离自身原点。
 */
        w3_loc_pose raw_pose={raw.x_m,raw.y_m,raw.yaw_rad},pose={0,0,0};
        if(!s->have_seed&&s->have_manual){
            lab_real_meta initial_meta;lab_pose initial_raw;
            r=raw_at(s,s->manual_time,&initial_meta,&initial_raw);if(r!=LAB_OK)return LAB_AGAIN;
            w3_loc_pose prior={s->manual_pose.x_m,s->manual_pose.y_m,s->manual_pose.yaw_rad};
            pose=wl_compose(prior,wl_relative((w3_loc_pose){initial_raw.x_m,initial_raw.y_m,initial_raw.yaw_rad},raw_pose));
        }
        if(!s->have_seed){s->seed_map=pose;s->seed_odom=raw_pose;s->have_seed=1;}
        pose=wl_compose(s->seed_map,wl_relative(s->seed_odom,raw_pose));
        r=wl_grid_insert(s->grid,&s->matcher,pose,s->pending);if(r!=LAB_OK){++s->rejected;s->have_pending=0;reason(s,"MAP_SEED_TOO_FEW_POINTS");return r;}
        w3_loc_context ctx=private_context(s->context);
        r=w3_loc_open(&s->core,&s->matcher,&ctx,&s->grid->view);if(r==LAB_ERR_UNAVAILABLE){s->have_pending=0;reason(s,"MAP_SEED_CONFIRMING");return LAB_AGAIN;}if(r!=LAB_OK){++s->rejected;rollback_seed(s);reason(s,"MAP_SEED_NO_REFERENCE");return r;}
        r=initialize_core(s,&aligned,&raw,pose);if(r!=LAB_OK){++s->rejected;rollback_seed(s);reason(s,"MAP_SEED_INVALID_INITIAL_POSE");return r;}
        s->last_keyframe=pose;s->last_grid_time=s->pending_meta.time_us;s->have_keyframe=1;s->have_pending=0;trace_pose(s,pose);
        reason(s,"MAP_SEEDED_WAIT_NEXT_SCAN");return LAB_OK;
    }
    if(!s->core){reason(s,"WAIT_FIXED_MAP");return LAB_AGAIN;}
    if(s->have_manual){r=try_manual(s);if(r!=LAB_OK)return r;}
    if(!s->core->initialized){reason(s,"WAIT_INITIAL_POSE");return LAB_AGAIN;}
    if(s->core->result.reason==W3_LOC_LOST||s->recovering){r=try_recover(s,&aligned,&raw);if(r!=LAB_OK){s->have_pending=0;return LAB_ERR_UNAVAILABLE;}}
    if(s->pending_meta.time_us<s->core->initial_time){s->have_pending=0;return LAB_AGAIN;}
    w3_loc_meta sm=private_meta(s,&s->pending_meta,0);
    /* 待处理扫描在支持数据实际可见后交付，内部准入用当前交接时刻；pending_meta和外部元数据保留原公开时间戳，测量时刻不改。

 */
    if(sm.received_us<s->core->received_us)sm.received_us=s->core->received_us;
    r=w3_loc_push_scan(s->core,&sm,s->pending);s->have_pending=0;
    w3_loc_result out;w3_loc_get_result(s->core,s->received_us,&out);
    if(r!=LAB_OK){if(s->recovering){s->recovery_matches=0;++s->recovery_failures;}++s->rejected;reason(s,w3_loc_reason_text(out.reason));fprintf(stderr,"LOCALIZER_MATCH_DETAIL status=%d reason=%s scan_sequence=%u scan_time_us=%llu last_success_us=%llu\n",(int)r,w3_loc_reason_text(out.reason),s->pending_meta.sequence,(unsigned long long)s->pending_meta.time_us,(unsigned long long)s->last_matched_us);return r==LAB_ERR_TIME&&out.reason==W3_LOC_LOST?LAB_ERR_UNAVAILABLE:r;}
    ++s->matched;s->last_matched_us=s->pending_meta.time_us;
    s->last_matched_uncertainty_us=s->pending_meta.time_uncertainty_us+aligned.time_uncertainty_us;
    int recovered_now=0;if(s->recovering&&++s->recovery_matches>=2){s->recovering=0;s->recovery_state=0;recovered_now=1;fprintf(stderr,"LOCALIZER_RECOVERY_CONFIRMED matches=2 coarse_attempts=%u\n",s->recovery_attempts);}
    if(s->recovering){reason(s,"RECOVERY_CONFIRMING");return LAB_OK;}
    if(s->have_fallback&&s->config.allow_odometry_fallback&&!recovered_now){
        w3_loc_pose predicted=wl_compose(s->fallback_transform,(w3_loc_pose){raw.x_m,raw.y_m,raw.yaw_rad});
        double shift=hypot(predicted.x_m-out.pose.x_m,predicted.y_m-out.pose.y_m),yaw=fabs(wl_wrap(predicted.yaw_rad-out.pose.yaw_rad));
        if(shift>.15||yaw>.35)fprintf(stderr,"LOCALIZATION_REBASE translation_m=%.4f yaw_rad=%.4f action=ACCEPT_WEIGHTED_MAP_CORRECTION odometry_continuous=1 lockout=0\n",shift,yaw);
    }
    if(!s->have_fallback||hypot(s->fallback_transform.x_m-out.map_odom.x_m,s->fallback_transform.y_m-out.map_odom.y_m)>1e-8||fabs(wl_wrap(s->fallback_transform.yaw_rad-out.map_odom.yaw_rad))>1e-8)++s->correction_version;
    s->fallback_transform=out.map_odom;s->have_fallback=1;s->fallback_time=s->last_matched_us;s->fallback_uncertainty=s->last_matched_uncertainty_us;
    trace_pose(s,out.pose);reason(s,s->recovering?"RECOVERY_CONFIRMING":s->core->correction_weight<.5?"WEAK_MATCH_ODOMETRY_CONTINUE":"TRACKING");
    if(s->grid&&s->core->correction_weight>=.5){
        double distance=hypot(out.pose.x_m-s->last_keyframe.x_m,out.pose.y_m-s->last_keyframe.y_m);
        if(distance>=.10||fabs(wl_wrap(out.pose.yaw_rad-s->last_keyframe.yaw_rad))>=.10||s->pending_meta.time_us-s->last_grid_time>=500000){
            r=wl_grid_insert(s->grid,&s->matcher,out.pose,s->pending);if(r!=LAB_OK)return r;
            r=wl_queue_reference(s->core,&s->grid->view);if(r<0)return r;
            s->last_keyframe=out.pose;s->last_grid_time=s->pending_meta.time_us;
        }
    }
    return LAB_OK;
}
static lab_status real_push_mode(void *user,const lab_live_sample *e,int control_only) {
    if(control_only&&(!e||e->type!=LAB_EVENT_POSE))return LAB_ERR_ARGUMENT;
    real_localization *s=user;if(!s||!lab_live_sample_valid(e))return LAB_ERR_ARGUMENT;
    if(!lab_real_context_equal(&s->context,&e->meta.context))return LAB_ERR_CONTEXT;
    if(e->meta.parameter_version!=s->config.parameter_version)return LAB_ERR_VERSION;
    if(!(e->meta.flags&LAB_REAL_TIME_VALID)||s->context.clock==LAB_REAL_UNMAPPED_CLOCK||
       (s->have_delivery&&e->type!=LAB_EVENT_SCAN&&e->meta.received_us<s->received_us))return LAB_ERR_TIME;
    if(s->core&&!control_only){wl_reference_work(s->core,64);if(s->grid&&!s->core->reference_job&&s->grid->view.map_version>s->core->map.map_version){lab_status q=wl_queue_reference(s->core,&s->grid->view);if(q<0)return q;}recovery_work(s,16);}
    if(e->type==LAB_EVENT_POSE){
        lab_status r=remember(s,e);if(r!=LAB_OK)return r;
        if(e->meta.received_us>s->received_us)s->received_us=e->meta.received_us;s->have_delivery=1;
        int was_manual=s->have_manual;
        if(s->core&&s->have_manual&&!control_only){r=try_manual(s);if(r<0)return r;}
        if(s->core&&s->core->initialized&&!was_manual){w3_loc_meta pm=private_meta(s,&e->meta,1);r=control_only?wl_push_control_pose(s->core,&pm,&e->data.odometry):w3_loc_push_pose(s->core,&pm,&e->data.odometry);if(r<0){if(r==LAB_ERR_TIME&&s->core->result.reason==W3_LOC_LOST){reason(s,"LOST_REINITIALIZE");return LAB_ERR_UNAVAILABLE;}return r;}}
        return control_only?LAB_OK:process_pending(s);
    }
    if(e->meta.received_us>s->received_us)s->received_us=e->meta.received_us;s->have_delivery=1;
    if(e->type==LAB_EVENT_SCAN){
        if(s->blocked)return LAB_ERR_UNAVAILABLE;
        if(e->meta.provider!=LAB_REAL_USB&&e->meta.provider!=LAB_REAL_FPGA)return LAB_ERR_ARGUMENT;
        if(s->have_scan_stream&&!same_stream(&s->scan_stream,&e->meta))return LAB_ERR_CONTEXT;
        if(s->have_scan_stream){uint32_t d=e->meta.sequence-s->scan_stream.sequence;
            if(e->meta.time_us<=s->scan_stream.time_us||!d||d>=UINT32_C(0x80000000))return LAB_ERR_TIME;}
        s->scan_stream=e->meta;s->have_scan_stream=1;++s->scans;
        if(!(e->meta.flags&LAB_REAL_DATA_VALID)||e->meta.time_uncertainty_us>CONT_TIME_UNCERTAINTY_US){
            ++s->rejected;
            if(s->core){s->core->result.quality&=~LAB_POSE_VALID;s->core->result.reason=W3_LOC_STALE;}
            reason(s,"INVALID_SCAN_OR_TIME_UNCERTAINTY");return LAB_ERR_UNAVAILABLE;
        }
        *s->pending=e->data.scan;s->pending_meta=e->meta;s->have_pending=1;return process_pending(s);
    }
    if(e->type==LAB_EVENT_RESET&&e->data.reset.reason==LAB_RESET_START&&e->data.reset.next_origin_epoch==s->context.origin_epoch&&
       !s->have_pose_stream&&!s->have_scan_stream)return LAB_OK;
    if(e->type==LAB_EVENT_GAP||e->type==LAB_EVENT_RESET){s->blocked=1;s->have_pending=0;reason(s,"SOURCE_GAP_OR_RESET_REQUIRES_NEW_CONTEXT");return LAB_ERR_CONTEXT;}
    return LAB_OK; /* 编码器和IMU由记录器保存，此处不再次融合。 */
}
static lab_status real_push(void*u,const lab_live_sample*e){return real_push_mode(u,e,0);}
static lab_status real_control_pose(void*u,const lab_live_sample*e){return real_push_mode(u,e,1);}
static int fallback_ready(const real_localization *s,uint64_t now){
 if(!s->config.allow_odometry_fallback||!s->have_fallback||s->blocked||s->have_manual||!s->history_count||now<s->fallback_time)return 0;
 const real_history*h=&s->history[s->history_count-1];
 /* 这是新的IMU与编码器估计，不重放上次雷达位姿。仍要求数值有限、数据有效、无过期或缺口或打滑、使用IMU且源时间当前；传感器故障仍停止输出。雷达失配时长不等于最近传感输入年龄。


 */
 if(!(h->pose.quality&LAB_POSE_IMU_USED))return 0;
 if(!usable_pose(s,h)||stale(now,h->meta.time_us,h->meta.time_uncertainty_us,CONT_ODOM_US))return 0;
 return 1;
}
static int fallback_snapshot(real_localization*s,uint64_t now,lab_localization_view*out){
 if(!fallback_ready(s,now))return 0;const real_history*h=&s->history[s->history_count-1];
 w3_loc_pose p=wl_compose(s->fallback_transform,(w3_loc_pose){h->pose.x_m,h->pose.y_m,h->pose.yaw_rad});
 out->has_pose=1;out->pose=h->pose;out->pose.x_m=p.x_m;out->pose.y_m=p.y_m;out->pose.yaw_rad=p.yaw_rad;
 out->quality=out->pose.quality;out->pose_time_us=h->meta.time_us;out->state=LAB_REAL_LOC_DEGRADED;out->odometry_fallback=1;
 out->map_odom_x_m=s->fallback_transform.x_m;out->map_odom_y_m=s->fallback_transform.y_m;out->map_odom_yaw_rad=s->fallback_transform.yaw_rad;
 snprintf(out->reason,sizeof(out->reason),"ODOMETRY_CONTINUE_FRESH_SENSOR");return 1;
}
static lab_status real_pose_at(void *user,uint64_t t,lab_real_meta *meta,lab_pose *pose) {
    if(meta)memset(meta,0,sizeof(*meta));if(pose)memset(pose,0,sizeof(*pose));
    real_localization *s=user;if(!s||!meta||!pose)return LAB_ERR_ARGUMENT;
    if(!s->core||s->blocked||s->have_manual)return LAB_AGAIN;
    uint64_t now=evaluation_time(s);w3_loc_result out;lab_status r=w3_loc_get_result(s->core,now,&out);
    if(r!=LAB_OK)return r;
    int fallback=s->recovering||!(out.quality&LAB_POSE_VALID)||stale(now,s->last_matched_us,s->last_matched_uncertainty_us,s->config.max_pose_age_us);
    if(fallback&&!fallback_ready(s,now))return LAB_AGAIN;
    lab_real_meta m;lab_pose p;r=raw_at(s,t,&m,&p);if(r!=LAB_OK)return r;
    w3_loc_pose corrected=wl_compose(fallback?s->fallback_transform:out.map_odom,(w3_loc_pose){p.x_m,p.y_m,p.yaw_rad});
    p.x_m=corrected.x_m;p.y_m=corrected.y_m;p.yaw_rad=corrected.yaw_rad;
    m.provider=LAB_REAL_LOCALIZER;m.stream_id=901;m.stream_session=1;m.parameter_version=s->config.parameter_version;
    if(p.calibrated)m.flags|=LAB_REAL_CALIBRATED;else m.flags&=~LAB_REAL_CALIBRATED;
    *meta=m;*pose=p;return LAB_OK;
}
static void map_view(real_localization *s,lab_real_grid_view *out) {
    memset(out,0,sizeof(*out));out->context=s->context;
    const w3_loc_map *m=s->grid?&s->grid->view:s->core?&s->core->map:NULL;if(!m)return;
    out->map_id=m->map_id;out->map_version=m->map_version;out->correction_version=s->correction_version;out->width=m->width;out->height=m->height;
    out->resolution_m=m->resolution_m;out->origin_x_m=m->origin_x_m;out->origin_y_m=m->origin_y_m;
    out->cells=m->cells;out->readonly=s->config.mode==LAB_REAL_FIXED_MAP;out->trace=s->trace;out->trace_count=s->trace_count;
}
static lab_status real_snapshot(void *user,uint64_t now,lab_localization_view *out) {
    if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));real_localization *s=user;if(!s)return LAB_ERR_ARGUMENT;
    if(s->have_delivery&&now<s->received_us)return LAB_ERR_TIME;
    if(s->have_evaluation_time&&now<s->evaluation_time_us)return LAB_ERR_TIME;
    s->evaluation_time_us=now;s->have_evaluation_time=1;
    out->context=s->context;out->parameter_version=s->config.parameter_version;out->state=LAB_REAL_LOC_WAITING;
    out->scans=s->scans;out->matched=s->matched;out->rejected=s->rejected;out->last_matched_scan_us=s->last_matched_us;map_view(s,&out->map);
    if(s->grid){out->map_expansions=s->grid->expansions;out->map_growth_limited=s->grid->growth_limited;out->map_geometry_version=s->grid->geometry_version;out->map_allocated_bytes=s->grid->allocated_bytes;}
    out->recovery_state=s->recovery_state;out->recovery_attempts=s->recovery_attempts;out->recovery_trials=s->recovery_job.cursor;out->recovery_total_trials=RECOVERY_TRIALS;out->recovery_ambiguous=s->recovery_ambiguous;out->recovery_candidate_failures=s->recovery_failures;out->recovery_best_score=s->recovery_best_score;out->recovery_second_score=s->recovery_second_score;out->recovery_search_m=.60;out->recovery_search_rad=3.14159265358979323846;
    snprintf(out->reason,sizeof(out->reason),"%s",s->reason);
    if(!s->core||s->have_manual)return LAB_OK;
    w3_loc_result result;lab_status r=w3_loc_get_result(s->core,now,&result);if(r!=LAB_OK)return r;
    out->match_input_points=result.input_points;out->match_inside_points=result.inside_points;out->match_outside_points=result.outside_points;out->match_reference_points=result.reference_points;out->match_close_points=result.close_points;out->match_sector_count=result.sector_count;
    out->match_rmse_m=result.rmse_m;out->match_robust_rmse_m=result.robust_rmse_m;out->match_ratio=result.match_ratio;out->match_coverage=result.coverage;out->map_allocated_bytes+=result.allocated_bytes;
    out->map_reference_version=s->core->map.map_version;out->map_reference_pending=s->core->reference_job!=NULL;if(s->core->reference_job)out->map_allocated_bytes+=s->core->reference_job->allocated_bytes;
    out->has_pose=result.has_pose;out->quality=result.quality;out->pose_time_us=result.scan_meta.time_us;
    out->pose=result.raw_odometry;out->pose.x_m=result.pose.x_m;out->pose.y_m=result.pose.y_m;out->pose.yaw_rad=result.pose.yaw_rad;
    out->map_odom_x_m=result.map_odom.x_m;out->map_odom_y_m=result.map_odom.y_m;out->map_odom_yaw_rad=result.map_odom.yaw_rad;
    if(s->blocked){out->quality&=~LAB_POSE_VALID;out->state=LAB_REAL_LOC_LOST;return LAB_OK;}
    if(s->recovering){if(fallback_snapshot(s,now,out))return LAB_OK;out->quality&=~LAB_POSE_VALID;out->pose.quality&=~LAB_POSE_VALID;out->state=LAB_REAL_LOC_LOST;snprintf(out->reason,sizeof(out->reason),"%s",s->reason);return LAB_OK;}
    if(result.quality&LAB_POSE_VALID){
        if(s->history_count){const real_history *h=&s->history[s->history_count-1];
            if(usable_pose(s,h)&&!stale(now,h->meta.time_us,h->meta.time_uncertainty_us,s->config.max_pose_age_us)){
                w3_loc_pose p=wl_compose(result.map_odom,(w3_loc_pose){h->pose.x_m,h->pose.y_m,h->pose.yaw_rad});out->pose=h->pose;
                out->pose.x_m=p.x_m;out->pose.y_m=p.y_m;out->pose.yaw_rad=p.yaw_rad;out->pose_time_us=h->meta.time_us;
                out->quality|=h->pose.quality;if(!h->pose.calibrated)out->quality|=LAB_POSE_UNCALIBRATED;
            }else{out->quality&=~LAB_POSE_VALID;snprintf(out->reason,sizeof(out->reason),"ODOMETRY_INVALID_OR_STALE");}
        }
        out->state=out->quality&LAB_POSE_VALID?LAB_REAL_LOC_TRACKING:LAB_REAL_LOC_DEGRADED;
    }else{
        out->state=result.has_pose?LAB_REAL_LOC_DEGRADED:LAB_REAL_LOC_WAITING;
        if(result.reason==W3_LOC_LOST||result.reason==W3_LOC_STALE)out->state=LAB_REAL_LOC_LOST;
        if(result.reason!=W3_LOC_UNINITIALIZED&&result.reason!=W3_LOC_WAIT_POSE){
            if(out->state==LAB_REAL_LOC_WAITING)out->state=LAB_REAL_LOC_DEGRADED;
            snprintf(out->reason,sizeof(out->reason),"%s",w3_loc_reason_text(result.reason));
        }
    }
    if(result.has_pose&&stale(now,s->last_matched_us,s->last_matched_uncertainty_us,s->config.max_pose_age_us)){
        static uint64_t last_timing_log=0;
        if(!last_timing_log||now<last_timing_log||now-last_timing_log>=1000000){fprintf(stderr,"MATCH_AGE_DIAG now_us=%llu match_us=%llu age_us=%llu uncertainty_us=%llu limit_us=%llu\n",(unsigned long long)now,(unsigned long long)s->last_matched_us,(unsigned long long)(now>=s->last_matched_us?now-s->last_matched_us:0),(unsigned long long)s->last_matched_uncertainty_us,(unsigned long long)s->config.max_pose_age_us);last_timing_log=now;}
        out->quality&=~LAB_POSE_VALID;out->quality|=LAB_POSE_STALE;out->state=LAB_REAL_LOC_LOST;
        if(result.reason==W3_LOC_MATCHED||result.reason==W3_LOC_STALE)
            snprintf(out->reason,sizeof(out->reason),"MATCH_AGE_PLUS_TIME_UNCERTAINTY");
    }
    if(out->state!=LAB_REAL_LOC_TRACKING)fallback_snapshot(s,now,out);
    out->pose.quality=out->quality;return LAB_OK;
}
static lab_status real_set_map(void *user,const lab_real_grid_view *map) {
    real_localization *s=user;if(!s||!lab_real_grid_valid(map))return LAB_ERR_ARGUMENT;
    if(s->config.mode!=LAB_REAL_FIXED_MAP)return LAB_ERR_READONLY;
    if(!lab_real_context_equal(&s->context,&map->context))return LAB_ERR_CONTEXT;
    if(s->core&&s->core->map.map_id==map->map_id&&map->map_version<=s->core->map.map_version)return LAB_ERR_TIME;
    w3_loc_map candidate={map->width,map->height,map->resolution_m,map->origin_x_m,map->origin_y_m,map->map_id,map->map_version,map->cells};
    w3_loc_context ctx=private_context(s->context);w3_localizer *next=NULL;lab_status r=w3_loc_open(&next,&s->matcher,&ctx,&candidate);
    if(r!=LAB_OK)return r;w3_loc_close(s->core);s->core=next;s->trace_count=0;s->have_manual=s->have_pending=0;
    s->last_matched_us=s->last_matched_uncertainty_us=0;s->have_fallback=0;reason(s,"WAIT_INITIAL_POSE");return LAB_OK;
}
static lab_status real_set_initial(void *user,const lab_pose *pose,uint64_t now) {
    real_localization *s=user;if(!s||!pose||!isfinite(pose->x_m)||!isfinite(pose->y_m)||!isfinite(pose->yaw_rad))return LAB_ERR_ARGUMENT;
    if(s->blocked)return LAB_ERR_CONTEXT;
    if(s->have_delivery&&now<s->received_us)return LAB_ERR_TIME;
    if(s->have_evaluation_time&&now<s->evaluation_time_us)return LAB_ERR_TIME;
    if(s->config.mode==LAB_REAL_BUILD_MAP&&s->core)return LAB_ERR_READONLY;
    if(s->config.mode==LAB_REAL_FIXED_MAP&&!s->core)return LAB_ERR_UNAVAILABLE;
    s->manual_pose=*pose;s->manual_time=now;s->have_manual=1;s->trace_count=0;s->have_pending=0;
    reason(s,"INITIAL_POSE_WAIT_ODOMETRY");lab_status r=try_manual(s);return r==LAB_AGAIN?LAB_OK:r;
}
static lab_status real_reset(void *user,const lab_real_context *context) {
    real_localization *s=user;if(!s||!lab_real_context_valid(context)||context->clock==LAB_REAL_UNMAPPED_CLOCK)return LAB_ERR_ARGUMENT;
    if(context->timeline_id==s->context.timeline_id&&context->origin_epoch==s->context.origin_epoch)return LAB_ERR_CONTEXT;
    if(s->grid){wl_grid *next=NULL;uint64_t id=((uint64_t)context->timeline_id<<32)|context->origin_epoch;
        lab_status r=wl_grid_open(&next,s->config.width,s->config.height,s->config.resolution_m,s->config.origin_x_m,s->config.origin_y_m,id);if(r!=LAB_OK)return r;
        wl_grid_close(s->grid);s->grid=next;w3_loc_close(s->core);s->core=NULL;
    }else if(s->core){w3_loc_context next=private_context(*context);lab_status r=w3_loc_reset(s->core,&next);if(r!=LAB_OK)return r;}
    s->context=*context;s->history_count=s->trace_count=0;s->have_pose_stream=s->have_scan_stream=s->have_pending=s->have_manual=s->have_delivery=s->blocked=s->have_keyframe=s->have_evaluation_time=s->have_seed=0;
    s->last_grid_time=s->correction_version=0;
    s->recovering=0;s->recovery_state=s->recovery_attempts=s->recovery_ambiguous=s->recovery_failures=0;memset(&s->recovery_job,0,sizeof(s->recovery_job));
    s->evaluation_time_us=0;s->have_fallback=0;
    s->scans=s->matched=s->rejected=s->received_us=s->last_matched_us=s->last_matched_uncertainty_us=0;
    reason(s,s->grid?"WAIT_MAPPING_SENSORS":s->core?"WAIT_INITIAL_POSE":"WAIT_FIXED_MAP");return LAB_OK;
}
static void real_close(void *user) {
    real_localization *s=user;if(s){w3_loc_close(s->core);wl_grid_close(s->grid);free(s->pending);free(s);}
}
size_t w3_localization_allocated_bytes(const void *user) {
    const real_localization *s=user;if(!s)return 0;
    return sizeof(*s)+sizeof(*s->pending)+(s->core?s->core->result.allocated_bytes:0)+(s->grid?s->grid->allocated_bytes:0);
}
static const lab_localization_ops operations={LAB_REAL_API_VERSION,sizeof(lab_localization_ops),
    real_push,real_snapshot,real_pose_at,real_set_map,real_set_initial,real_reset,real_close,real_control_pose};
lab_status W3_FACTORY(lab_localization_binding *out,const lab_localization_config *c,const lab_real_context *context) {
    if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));
    if(!c||!lab_real_context_valid(context)||!c->parameter_version||c->allow_uncalibrated>1||c->allow_odometry_fallback>1||
       (c->mode!=LAB_REAL_BUILD_MAP&&c->mode!=LAB_REAL_FIXED_MAP)||context->clock==LAB_REAL_UNMAPPED_CLOCK||
       !c->max_pose_age_us||c->max_pose_age_us>R8_LOC_MAX_MATCH_AGE_US||!c->max_interpolation_gap_us||c->max_interpolation_gap_us>1000000)return LAB_ERR_ARGUMENT;
    if(c->width<4||c->height<4||c->width>LAB_MAP_SIDE_MAX||c->height>LAB_MAP_SIDE_MAX)return LAB_ERR_LIMIT;
    if(!isfinite(c->resolution_m)||c->resolution_m<.01||c->resolution_m>1||!isfinite(c->origin_x_m)||!isfinite(c->origin_y_m)||
       fabs(c->origin_x_m)>1e6||fabs(c->origin_y_m)>1e6||!isfinite(c->mount_x_m)||!isfinite(c->mount_y_m)||!isfinite(c->mount_yaw_rad)||fabs(c->mount_x_m)>2||fabs(c->mount_y_m)>2)return LAB_ERR_ARGUMENT;
    real_localization *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->config=*c;s->context=*context;
    s->pending=calloc(1,sizeof(*s->pending));if(!s->pending){real_close(s);return LAB_ERR_NOMEM;}
    w3_loc_config_default(&s->matcher);s->matcher.allow_uncalibrated=c->allow_uncalibrated;s->matcher.max_pose_age_us=c->max_pose_age_us;
    s->matcher.max_pose_gap_us=c->max_interpolation_gap_us;s->matcher.mount_x_m=c->mount_x_m;s->matcher.mount_y_m=c->mount_y_m;s->matcher.mount_yaw_rad=c->mount_yaw_rad;
    if(c->mode==LAB_REAL_BUILD_MAP){uint64_t id=((uint64_t)context->timeline_id<<32)|context->origin_epoch;
        lab_status r=wl_grid_open(&s->grid,c->width,c->height,c->resolution_m,c->origin_x_m,c->origin_y_m,id);if(r!=LAB_OK){real_close(s);return r;}}
    reason(s,s->grid?"WAIT_MAPPING_SENSORS":"WAIT_FIXED_MAP");out->user=s;out->ops=&operations;return LAB_OK;
}
