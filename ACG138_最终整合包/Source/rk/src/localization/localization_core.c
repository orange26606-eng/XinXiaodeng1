#include "localization_internal.h"
#include "r8_localization_policy.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static int same_context(const w3_loc_context *a,const w3_loc_context *b) {
    return a->mode==b->mode&&a->clock==b->clock&&a->timeline_id==b->timeline_id&&a->origin_epoch==b->origin_epoch;
}
static int context_valid(const w3_loc_context *c) {
    return c&&c->timeline_id&&c->origin_epoch&&
        ((c->mode==W3_LOC_REAL&&c->clock==W3_LOC_RK_MONOTONIC)||
         (c->mode==W3_LOC_RECORDED_REAL&&c->clock==W3_LOC_REPLAY_CLOCK)||
         (c->mode==W3_LOC_SYNTHETIC_TEST&&c->clock==W3_LOC_TEST_CLOCK));
}
static int finite_pose(const lab_pose *p) {
    return p&&isfinite(p->x_m)&&isfinite(p->y_m)&&isfinite(p->yaw_rad)&&isfinite(p->distance_m)&&
        isfinite(p->uncertainty_m)&&p->uncertainty_m>=0&&p->distance_m>=0&&p->calibrated<=1;
}
static int usable(const w3_localizer *s,const wl_sample *p) {
    return (p->meta.flags&3)==3&&(p->pose.quality&LAB_POSE_VALID)&&
        !(p->pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))&&
        (s->c.allow_uncalibrated||p->pose.calibrated);
}
static int stream_equal(const w3_loc_meta *a,const w3_loc_meta *b) {
    return a->provider==b->provider&&a->stream_id==b->stream_id&&a->session==b->session;
}
static w3_loc_pose pose_of(lab_pose p) { return (w3_loc_pose){p.x_m,p.y_m,p.yaw_rad}; }
void w3_loc_config_default(w3_loc_config *c) {
    if(!c)return;memset(c,0,sizeof(*c));
    c->points_per_scan=360;c->iterations=20;c->min_points=R8_LOC_MIN_POINTS;c->occupancy_threshold=65;
    c->max_pose_gap_us=100000;c->max_scan_gap_us=R8_LOC_MAX_SCAN_GAP_US;c->max_pose_age_us=250000;
    c->min_range_m=.03;c->max_range_m=30;c->correspondence_m=R8_LOC_CORRESPONDENCE_M;c->huber_m=R8_LOC_HUBER_M;
    c->max_rmse_m=R8_LOC_MAX_ROBUST_RMSE_M;c->min_match_ratio=R8_LOC_MIN_MATCH_RATIO;c->initial_search_m=R8_LOC_INITIAL_SEARCH_M;c->initial_search_rad=R8_LOC_INITIAL_SEARCH_RAD;c->allow_uncalibrated=1;
}
static int valid_config(const w3_loc_config *c) {
    if(!c||c->points_per_scan<40||c->points_per_scan>W3_LOC_POINTS_MAX||!c->iterations||c->iterations>40||
       c->min_points<20||c->min_points>c->points_per_scan||c->occupancy_threshold<1||c->occupancy_threshold>100||
       !c->max_pose_gap_us||c->max_pose_gap_us>1000000||!c->max_scan_gap_us||c->max_scan_gap_us>5000000||
       !c->max_pose_age_us||c->max_pose_age_us>R8_LOC_MAX_MATCH_AGE_US||c->allow_uncalibrated>1)return 0;
    double values[]={c->mount_x_m,c->mount_y_m,c->mount_yaw_rad,c->min_range_m,c->max_range_m,
        c->correspondence_m,c->huber_m,c->max_rmse_m,c->min_match_ratio,c->initial_search_m,c->initial_search_rad};
    for(unsigned i=0;i<sizeof(values)/sizeof(*values);++i)if(!isfinite(values[i]))return 0;
    return fabs(c->mount_x_m)<=2&&fabs(c->mount_y_m)<=2&&c->min_range_m>0&&c->max_range_m>c->min_range_m&&c->max_range_m<=100&&
        c->correspondence_m>=.03&&c->correspondence_m<=1&&c->huber_m>0&&c->huber_m<=c->correspondence_m&&
        c->max_rmse_m>0&&c->max_rmse_m<=c->correspondence_m&&c->min_match_ratio>0&&c->min_match_ratio<=1&&
        c->initial_search_m>=0&&c->initial_search_m<=R8_LOC_INITIAL_SEARCH_M&&c->initial_search_rad>=0&&c->initial_search_rad<=R8_LOC_INITIAL_SEARCH_RAD;
}
lab_status w3_loc_open(w3_localizer **out,const w3_loc_config *c,const w3_loc_context *ctx,const w3_loc_map *map) {
    if(!out)return LAB_ERR_ARGUMENT;*out=NULL;
    if(!valid_config(c)||!context_valid(ctx)||!map||!map->cells||!map->map_id||!map->map_version)return LAB_ERR_ARGUMENT;
    if(map->width<4||map->height<4||map->width>W3_LOC_GRID_SIDE_MAX||map->height>W3_LOC_GRID_SIDE_MAX)return LAB_ERR_LIMIT;
    if(!isfinite(map->resolution_m)||map->resolution_m<.01||map->resolution_m>1||!isfinite(map->origin_x_m)||!isfinite(map->origin_y_m)||
        fabs(map->origin_x_m)>1e6||fabs(map->origin_y_m)>1e6)return LAB_ERR_ARGUMENT;
    size_t n=(size_t)map->width*map->height;uint32_t occupied=0;
    for(size_t i=0;i<n;++i) {
        if(map->cells[i]<-1||map->cells[i]>100)return LAB_ERR_FORMAT;
        occupied+=map->cells[i]>=(int)c->occupancy_threshold;
    }
    if(occupied<c->min_points)return LAB_ERR_UNAVAILABLE;
    w3_localizer *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;
    s->c=*c;s->context=*ctx;s->map=*map;
    uint32_t line=map->width>map->height?map->width:map->height;
    s->cells=malloc(n);s->distance=malloc(n*sizeof(float));s->pending=calloc(1,sizeof(*s->pending));
    s->line_in=malloc(line*sizeof(double));s->line_out=malloc(line*sizeof(double));s->breaks=malloc((line+1)*sizeof(double));s->sites=malloc(line*sizeof(int));
    if(!s->cells||!s->distance||!s->pending||!s->line_in||!s->line_out||!s->breaks||!s->sites) {w3_loc_close(s);return LAB_ERR_NOMEM;}
    memcpy(s->cells,map->cells,n);s->map.cells=s->cells;wl_build_field(s);
    s->result.reason=W3_LOC_UNINITIALIZED;s->result.map_id=map->map_id;s->result.map_version=map->map_version;
    s->result.allocated_bytes=sizeof(*s)+n*(1+sizeof(float))+sizeof(*s->pending)+(3*(size_t)line+1)*sizeof(double)+(size_t)line*sizeof(int);
    *out=s;return LAB_OK;
}
static lab_status check_meta(w3_localizer *s,const w3_loc_meta *m,int pose) {
    if(!m||!context_valid(&m->context)||!m->stream_id||!m->session||m->flags&~3u||m->provider==LAB_PROVIDER_TRUTH)return LAB_ERR_ARGUMENT;
    if(!same_context(&s->context,&m->context)) {++s->result.context_rejected;return LAB_ERR_CONTEXT;}
    int synthetic=s->context.mode==W3_LOC_SYNTHETIC_TEST;
    if(pose?(synthetic?m->provider!=LAB_PROVIDER_SIM_ESTIMATE:m->provider!=LAB_PROVIDER_ODOMETRY):
            (synthetic?m->provider!=LAB_PROVIDER_SIM_SENSOR:(m->provider!=LAB_PROVIDER_USB&&m->provider!=LAB_PROVIDER_FPGA)))return LAB_ERR_ARGUMENT;
    if(!(m->flags&LAB_TIME_VALID)||m->received_us<m->time_us||(s->have_delivery&&m->received_us<s->received_us)) {
        ++s->result.time_rejected;return LAB_ERR_TIME;
    }
    return LAB_OK;
}
static void invalid(w3_localizer *s,uint32_t reason) {
    s->result.quality&=~LAB_POSE_VALID;s->result.quality|=LAB_POSE_STALE;s->result.reason=reason;
}
lab_status w3_loc_initialize(w3_localizer *s,const w3_loc_pose *map_pose,const w3_loc_meta *m,const lab_pose *odom) {
    if(!s||!map_pose||!finite_pose(odom)||!isfinite(map_pose->x_m)||!isfinite(map_pose->y_m)||!isfinite(map_pose->yaw_rad))return LAB_ERR_ARGUMENT;
    lab_status r=check_meta(s,m,1);if(r!=LAB_OK)return r;
    wl_sample sample={*m,*odom};if(!usable(s,&sample))return LAB_ERR_UNAVAILABLE;
    if(s->have_odom&&!stream_equal(m,&s->odom_stream))return LAB_ERR_CONTEXT;
    double x=(map_pose->x_m-s->map.origin_x_m)/s->map.resolution_m,y=(map_pose->y_m-s->map.origin_y_m)/s->map.resolution_m;
    if(x<0||y<0||x>=s->map.width||y>=s->map.height)return LAB_ERR_ARGUMENT;
    int v=s->cells[(size_t)(uint32_t)y*s->map.width+(uint32_t)x];if(v<0||v>=(int)s->c.occupancy_threshold)return LAB_ERR_UNAVAILABLE;
    s->initial_map=*map_pose;s->initial_map.yaw_rad=wl_wrap(map_pose->yaw_rad);s->initial_odom=pose_of(*odom);
    s->initial_time=m->time_us;s->history_count=1;s->history[0]=sample;s->odom_stream=*m;s->have_odom=1;
    s->have_pending=s->have_success=s->have_processed=0;s->initialized=1;
    s->received_us=m->received_us;s->have_delivery=1;
    s->result.has_pose=0;s->result.quality=0;s->result.reason=W3_LOC_WAIT_POSE;
    return LAB_OK;
}
static lab_status pose_at(const w3_localizer *s,uint64_t t,lab_pose *out) {
    uint32_t b=0;while(b<s->history_count&&s->history[b].meta.time_us<t)++b;
    if(b==s->history_count)return LAB_AGAIN;
    uint32_t a=b;
    if(s->history[b].meta.time_us!=t) {
        if(!b)return LAB_ERR_TIME;a=b-1;
        if(s->history[b].meta.time_us-s->history[a].meta.time_us>s->c.max_pose_gap_us)return LAB_ERR_TIME;
    }
    if(!usable(s,&s->history[a])||!usable(s,&s->history[b]))return LAB_ERR_UNAVAILABLE;
    *out=s->history[a].pose;
    if(a!=b) {
        double u=(double)(t-s->history[a].meta.time_us)/(s->history[b].meta.time_us-s->history[a].meta.time_us);
        lab_pose q=s->history[b].pose;
        out->x_m+=u*(q.x_m-out->x_m);out->y_m+=u*(q.y_m-out->y_m);
        out->yaw_rad=wl_wrap(out->yaw_rad+u*wl_wrap(q.yaw_rad-out->yaw_rad));out->distance_m+=u*(q.distance_m-out->distance_m);
        out->quality|=q.quality;out->calibrated&=q.calibrated;out->uncertainty_m=fmax(out->uncertainty_m,q.uncertainty_m);
    }
    return LAB_OK;
}
static uint32_t extract(const w3_localizer *s,wl_point *points) {
    const struct scan_frame *f=s->pending;uint32_t n=0,cap=s->c.points_per_scan;
    double c=cos(s->c.mount_yaw_rad),v=sin(s->c.mount_yaw_rad);
    for(uint32_t i=0;i<cap&&i<f->count;++i) {
        uint32_t j=f->count>cap?(uint32_t)((uint64_t)i*f->count/cap):i;
        const struct scan_point *q=&f->points[j];double d=q->distance_q2/4000.0;
        if(!scan_return_in_range(f,j,s->c.min_range_m,s->c.max_range_m))continue;
        double angle=-(double)q->angle_q14*(3.14159265358979323846/32768.0),x=d*cos(angle),y=d*sin(angle);
        points[n++]=(wl_point){s->c.mount_x_m+c*x-v*y,s->c.mount_y_m+v*x+c*y};
    }
    return n;
}
static lab_status process(w3_localizer *s) {
    if(!s->have_pending)return LAB_OK;
    lab_pose odom={0};lab_status r=pose_at(s,s->pending_meta.time_us,&odom);
    if(r==LAB_AGAIN) {invalid(s,W3_LOC_WAIT_POSE);return r;}
    s->have_pending=0;s->have_processed=1;s->last_scan_time=s->pending_meta.time_us;s->last_scan_sequence=s->pending_meta.sequence;
    if(r!=LAB_OK) {++s->result.rejected;invalid(s,r==LAB_ERR_TIME?W3_LOC_POSE_GAP:W3_LOC_POSE_INVALID);return r;}
    /* 雷达匹配缺口不直接使航位推算过期，下方对齐位姿仍须真实可用，过期数据不刷新。
 */
    w3_loc_pose raw=pose_of(odom),predicted;
    if(s->have_success)predicted=wl_compose(s->last_map,wl_relative(s->last_odom,raw));
    else predicted=wl_compose(s->initial_map,wl_relative(s->initial_odom,raw));
    wl_point points[W3_LOC_POINTS_MAX];uint32_t n=extract(s,points);s->result.input_points=n;s->result.inside_points=s->result.outside_points=s->result.reference_points=s->result.close_points=s->result.sector_count=s->result.matched_points=0;s->result.rmse_m=s->result.robust_rmse_m=1e9;s->result.coverage=s->result.match_ratio=0;
    if(n<s->c.min_points) {++s->result.rejected;invalid(s,W3_LOC_TOO_FEW_POINTS);return LAB_ERR_UNAVAILABLE;}
    w3_loc_pose matched;double rmse,ratio;uint32_t used;int degenerate;
    int ok=wl_match(s,points,n,predicted,!s->have_success,&matched,&rmse,&ratio,&used,&degenerate);
    s->result.rmse_m=rmse;s->result.match_ratio=ratio;s->result.matched_points=used;
    if(!ok) {double nearest=1e9,farthest=0;for(uint32_t i=0;i<n;++i){double d=hypot(points[i].x,points[i].y);if(d<nearest)nearest=d;if(d>farthest)farthest=d;}fprintf(stderr,"MATCH_SCAN_DETAIL seq=%u total=%u valid=%u matched=%u ratio=%.4f rmse_m=%.4f range_min_m=%.3f range_max_m=%.3f predicted=%.3f,%.3f,%.4f map_version=%llu\n",s->pending_meta.sequence,s->pending->count,n,used,ratio,rmse,nearest,farthest,predicted.x_m,predicted.y_m,predicted.yaw_rad,(unsigned long long)s->map.map_version);++s->result.rejected;invalid(s,degenerate?W3_LOC_DEGENERATE:W3_LOC_MATCH_REJECTED);return LAB_ERR_UNAVAILABLE;}
    s->last_map=matched;s->last_odom=raw;s->last_success=s->pending_meta.time_us;s->have_success=1;
    ++s->result.accepted;s->result.has_pose=1;s->result.pose=matched;s->result.raw_odometry=odom;
    s->result.quality=odom.quality|LAB_POSE_VALID;
    if(!odom.calibrated)s->result.quality|=LAB_POSE_UNCALIBRATED;
    s->result.reason=W3_LOC_MATCHED;s->result.scan_meta=s->pending_meta;
    s->result.map_odom=wl_compose(matched,wl_relative(raw,(w3_loc_pose){0,0,0}));
    return LAB_OK;
}
static lab_status push_pose_mode(w3_localizer *s,const w3_loc_meta *m,const lab_pose *pose,int defer) {
    if(!s||!finite_pose(pose))return LAB_ERR_ARGUMENT;
    lab_status r=check_meta(s,m,1);if(r!=LAB_OK)return r;
    if(!s->initialized)return LAB_AGAIN;
    if(!stream_equal(m,&s->odom_stream)) {++s->result.context_rejected;return LAB_ERR_CONTEXT;}
    if(m->time_us<s->initial_time)return LAB_ERR_TIME;
    uint32_t at=0;while(at<s->history_count&&s->history[at].meta.time_us<m->time_us)++at;
    if(at<s->history_count&&s->history[at].meta.time_us==m->time_us)return LAB_ERR_TIME;
    if(s->history_count==W3_LOC_HISTORY_MAX) {
        if(!at)return LAB_ERR_TIME;
        memmove(s->history,s->history+1,(W3_LOC_HISTORY_MAX-1)*sizeof(*s->history));--s->history_count;--at;
    }
    memmove(s->history+at+1,s->history+at,(s->history_count-at)*sizeof(*s->history));s->history[at]=(wl_sample){*m,*pose};++s->history_count;
    s->received_us=m->received_us;s->have_delivery=1;return defer?LAB_OK:process(s);
}
lab_status w3_loc_push_pose(w3_localizer*s,const w3_loc_meta*m,const lab_pose*p){return push_pose_mode(s,m,p,0);}
lab_status wl_push_control_pose(w3_localizer*s,const w3_loc_meta*m,const lab_pose*p){return push_pose_mode(s,m,p,1);}
lab_status w3_loc_push_scan(w3_localizer *s,const w3_loc_meta *m,const struct scan_frame *f) {
    if(!s||!f||!f->count||f->count>SCAN_MAX_POINTS)return LAB_ERR_ARGUMENT;
    lab_status r=check_meta(s,m,0);if(r!=LAB_OK)return r;
    if(f->session!=m->session||f->sequence!=m->sequence)return LAB_ERR_FORMAT;
    if(!s->initialized)return LAB_AGAIN;
    if(!(m->flags&LAB_DATA_VALID)) {invalid(s,W3_LOC_TOO_FEW_POINTS);return LAB_ERR_UNAVAILABLE;}
    if(s->have_scan&&!stream_equal(m,&s->scan_stream)) {++s->result.context_rejected;return LAB_ERR_CONTEXT;}
    uint64_t previous_time=0;uint32_t previous_sequence=0;int check=0;
    if(s->have_pending) {previous_time=s->pending_meta.time_us;previous_sequence=s->pending_meta.sequence;check=1;}
    else if(s->have_processed) {previous_time=s->last_scan_time;previous_sequence=s->last_scan_sequence;check=1;}
    uint32_t diff=m->sequence-previous_sequence;
    if(m->time_us<s->initial_time||(check&&(m->time_us<=previous_time||!diff||diff>=UINT32_C(0x80000000))))return LAB_ERR_TIME;
    if(s->have_pending)++s->result.pending_replaced;
    *s->pending=*f;s->pending_meta=*m;s->scan_stream=*m;s->have_scan=s->have_pending=1;
    s->received_us=m->received_us;s->have_delivery=1;return process(s);
}
lab_status w3_loc_get_result(const w3_localizer *s,uint64_t now,w3_loc_result *out) {
    if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));if(!s)return LAB_ERR_ARGUMENT;
    if(s->have_delivery&&now<s->received_us)return LAB_ERR_TIME;
    *out=s->result;
    if(out->has_pose&&(now<s->last_success||now-s->last_success>=s->c.max_pose_age_us)) {
        out->quality=(out->quality&~LAB_POSE_VALID)|LAB_POSE_STALE;
        if(out->reason==W3_LOC_MATCHED)out->reason=W3_LOC_STALE;
    }
    return LAB_OK;
}
lab_status w3_loc_get_map(const w3_localizer *s,w3_loc_map *out) {
    if(!s||!out)return LAB_ERR_ARGUMENT;*out=s->map;return LAB_OK;
}
lab_status w3_loc_update_map(w3_localizer*s,const w3_loc_map*m){
 if(!s||!m||!m->cells||m->map_id!=s->map.map_id||m->resolution_m!=s->map.resolution_m||m->width<4||m->height<4||m->width>W3_LOC_GRID_SIDE_MAX||m->height>W3_LOC_GRID_SIDE_MAX||!isfinite(m->origin_x_m)||!isfinite(m->origin_y_m))return LAB_ERR_ARGUMENT;
 wl_reference_discard(s);size_t n=(size_t)m->width*m->height;unsigned line=m->width>m->height?m->width:m->height;
 for(size_t i=0;i<n;i++)if(m->cells[i]<-1||m->cells[i]>100)return LAB_ERR_FORMAT;
 if(m->width!=s->map.width||m->height!=s->map.height){
  int8_t*cells=malloc(n);float*distance=malloc(n*sizeof(*distance));double*in=malloc(line*sizeof(*in)),*out=malloc(line*sizeof(*out)),*breaks=malloc((line+1)*sizeof(*breaks));int*sites=malloc(line*sizeof(*sites));
  if(!cells||!distance||!in||!out||!breaks||!sites){free(cells);free(distance);free(in);free(out);free(breaks);free(sites);return LAB_ERR_NOMEM;}
  memcpy(cells,m->cells,n);free(s->cells);free(s->distance);free(s->line_in);free(s->line_out);free(s->breaks);free(s->sites);s->cells=cells;s->distance=distance;s->line_in=in;s->line_out=out;s->breaks=breaks;s->sites=sites;
 }else memcpy(s->cells,m->cells,n);
 s->map=*m;s->map.cells=s->cells;s->result.map_version=m->map_version;s->result.allocated_bytes=sizeof(*s)+n*(1+sizeof(float))+sizeof(*s->pending)+(3*(size_t)line+1)*sizeof(double)+(size_t)line*sizeof(int);wl_build_field(s);return LAB_OK;
}
lab_status w3_loc_reset(w3_localizer *s,const w3_loc_context *ctx) {
    if(!s||!context_valid(ctx))return LAB_ERR_ARGUMENT;
    if(ctx->timeline_id==s->context.timeline_id&&ctx->origin_epoch==s->context.origin_epoch)return LAB_ERR_CONTEXT;
    wl_reference_discard(s);size_t bytes=s->result.allocated_bytes;s->context=*ctx;
    s->history_count=0;s->initialized=s->have_odom=s->have_scan=s->have_pending=s->have_delivery=s->have_success=s->have_processed=0;
    s->received_us=s->initial_time=s->last_success=s->last_scan_time=0;
    memset(&s->result,0,sizeof(s->result));s->result.reason=W3_LOC_UNINITIALIZED;s->result.map_id=s->map.map_id;
    s->result.map_version=s->map.map_version;s->result.allocated_bytes=bytes;return LAB_OK;
}
const char *w3_loc_reason_text(uint32_t reason) {
    switch(reason) {
        case W3_LOC_UNINITIALIZED:return "UNINITIALIZED";case W3_LOC_WAIT_POSE:return "WAIT_POSE";
        case W3_LOC_MATCHED:return "MATCHED";case W3_LOC_TOO_FEW_POINTS:return "TOO_FEW_POINTS";
        case W3_LOC_MATCH_REJECTED:return "MATCH_REJECTED";case W3_LOC_DEGENERATE:return "DEGENERATE";
        case W3_LOC_POSE_INVALID:return "POSE_INVALID";case W3_LOC_POSE_GAP:return "POSE_GAP";
        case W3_LOC_LOST:return "LOST_REINITIALIZE";case W3_LOC_STALE:return "STALE";
        case W3_LOC_CONTEXT_REJECTED:return "CONTEXT";case W3_LOC_TIME_REJECTED:return "TIME";default:return "UNKNOWN";
    }
}
void w3_loc_close(w3_localizer *s) {
    if(!s)return;wl_reference_discard(s);free(s->cells);free(s->distance);free(s->pending);free(s->line_in);free(s->line_out);free(s->breaks);free(s->sites);free(s);
}
