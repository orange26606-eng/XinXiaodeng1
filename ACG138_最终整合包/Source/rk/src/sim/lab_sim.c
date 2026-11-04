#include "lab_sim.h"
#include "sim_engine.h"
#include "odometry.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define PI 3.14159265358979323846
#define PENDING_POSES 512u
struct pending_pose {lab_meta meta;lab_pose pose;};
struct lab_sim {
    lab_sim_config config;
    struct sim_engine engine;
    struct odom_model odometry;
    struct sim_truth estimator_truth;
    struct sim_scan scan;
    uint64_t next_index[4],current_us,estimator_time_us,last_imu_us;
    uint32_t pose_sequence,encoder_sequence,imu_sequence,truth_sequence,gap_sequence;
    uint32_t sessions[7];
    int due[4],stage,started,finished,reset_pending;
    struct pending_pose pending[PENDING_POSES];
    unsigned pending_head,pending_count;
    double imu_rate,last_imu_yaw,pos_squared,yaw_squared;
    uint64_t metric_samples;
    lab_sim_metrics metrics;
};
static unsigned rate(const lab_sim *s,unsigned index) {
    switch(index){case 0:return s->config.encoder_hz;case 1:return s->config.imu_hz;case 2:return s->config.pose_hz;default:return s->config.scan_hz;}
}
static uint64_t scheduled(const lab_sim *s,unsigned index) {return s->next_index[index]*UINT64_C(1000000)/rate(s,index);}
static uint64_t next_tick(const lab_sim *s) {
    uint64_t next=UINT64_MAX;for(unsigned i=0;i<4;++i){uint64_t t=scheduled(s,i);if(t<next)next=t;}return next;
}
void lab_sim_config_default(lab_sim_config *c) {
    if(!c)return;memset(c,0,sizeof(*c));c->seed=1;c->mode=LAB_SIM_IDEAL;c->path=LAB_PATH_STRAIGHT;c->timeline_id=1;
    c->duration_us=10000000;c->scan_hz=10;c->pose_hz=c->encoder_hz=c->imu_hz=50;c->scan_points=360;
    c->counts_per_revolution=1320;c->room_width_m=8;c->room_height_m=6;c->speed_m_s=.1;c->yaw_rate_rad_s=.3;
    c->wheel_diameter_m=.0675;c->track_m=.290;c->left_mask=5;
    for(int i=0;i<4;++i){c->wheel_scale[i]=1;c->encoder_signs[i]=(i%2)?-1:1;}
}
static lab_meta meta(const lab_sim *s,unsigned stream,unsigned provider,uint32_t sequence,uint64_t measurement,uint64_t delivery) {
    lab_meta m={0};m.mode=s->config.mode;m.provider=provider;m.clock_domain=LAB_CLOCK_SIMULATION;
    m.flags=LAB_TIME_VALID|LAB_DATA_VALID;m.timeline_id=s->config.timeline_id;m.origin_epoch=1;
    m.stream_id=stream;m.stream_session=s->sessions[stream-1];m.sequence=sequence;m.time_us=measurement;m.received_us=delivery;return m;
}
static lab_status valid_config(const lab_sim_config *c,struct sim_config *engine) {
    if(!c || !c->seed || !c->timeline_id || (c->mode!=LAB_SIM_IDEAL&&c->mode!=LAB_SIM_NOISY) ||
       c->path<LAB_PATH_STATIONARY||c->path>LAB_PATH_RECTANGLE || !c->duration_us || c->duration_us>UINT64_C(600000000) ||
       !c->scan_hz||c->scan_hz>200||!c->pose_hz||c->pose_hz>200||!c->encoder_hz||c->encoder_hz>200||!c->imu_hz||c->imu_hz>200||
       !isfinite(c->room_width_m)||c->room_width_m<2||c->room_width_m>50||!isfinite(c->room_height_m)||c->room_height_m<2||c->room_height_m>50)return LAB_ERR_ARGUMENT;
    // 限定延迟位姿内存及时间戳运算，不限制其他流。
    if(c->pose_delivery_delay_us>UINT64_C(600000000) ||
       c->pose_delivery_delay_us*c->pose_hz/1000000+2>PENDING_POSES)return LAB_ERR_LIMIT;
    enum sim_path path=c->path==LAB_PATH_STATIONARY?SIM_STATIONARY:c->path==LAB_PATH_STRAIGHT?SIM_STRAIGHT:
        c->path==LAB_PATH_ROTATE?SIM_TURN:c->path==LAB_PATH_CIRCLE?SIM_CIRCLE:SIM_RECTANGLE;
    sim_default_config(engine,path);engine->seed=c->seed;engine->period_us=1000;engine->imu_period_us=1000;
    engine->duration_us=c->duration_us;engine->scan_points=c->scan_points;
    engine->speed_m_s=c->speed_m_s;engine->yaw_rate_rad_s=c->yaw_rate_rad_s;
    engine->room_width_m=c->room_width_m;engine->room_height_m=c->room_height_m;
    engine->wheel_diameter_m=c->wheel_diameter_m;engine->track_m=c->track_m;engine->counts_per_revolution=c->counts_per_revolution;
    engine->left_mask=c->left_mask;engine->mount_x_m=c->mount_x_m;engine->mount_y_m=c->mount_y_m;engine->mount_yaw_rad=c->mount_yaw_rad;
    engine->range_noise_m=c->scan_noise_std_m;engine->encoder_noise_counts=c->wheel_noise_std_counts;
    engine->imu_noise_rad=c->imu_noise_std_rad;engine->imu_drift_rad_per_s=c->imu_bias_rad_s;
    for(int i=0;i<4;++i){engine->encoder_scale[i]=c->wheel_scale[i];engine->encoder_sign[i]=c->encoder_signs[i];}
    if(sim_validate_config(engine))return LAB_ERR_ARGUMENT;
    if(c->mode==LAB_SIM_IDEAL){
        if(c->scan_noise_std_m||c->wheel_noise_std_counts||c->imu_noise_std_rad||c->imu_bias_rad_s)return LAB_ERR_ARGUMENT;
        for(int i=0;i<4;++i)if(c->wheel_scale[i]!=1)return LAB_ERR_ARGUMENT;
    }
    return LAB_OK;
}
lab_status lab_sim_open(lab_sim **out,const lab_sim_config *config) {
    if(!out)return LAB_ERR_ARGUMENT;*out=NULL;struct sim_config engine;lab_status check=valid_config(config,&engine);if(check!=LAB_OK)return check;
    lab_sim *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->config=*config;
    if(sim_init(&s->engine,&engine)){free(s);return LAB_ERR_ARGUMENT;}
    struct odom_config od;odom_default_config(&od);od.track_mm=config->track_m*1000;od.left_mask=(int)config->left_mask;
    for(int i=0;i<4;++i){od.mm_per_count[i]=PI*config->wheel_diameter_m*1000/config->counts_per_revolution;od.signs[i]=config->encoder_signs[i];}
    odom_init(&s->odometry,&od);s->last_imu_yaw=s->engine.sample.imu_yaw_rad;
    for(unsigned i=0;i<7;++i){s->sessions[i]=config->seed^(0xa35d2401u+i*0x1020304u);if(!s->sessions[i])s->sessions[i]=i+1;}
    s->reset_pending=1;s->stage=5;*out=s;return LAB_OK;
}
static void finish_event(lab_sim *s,lab_event *out) {++s->metrics.generated_events;(void)out;}
static lab_status deliver_pose(lab_sim *s,lab_event *out) {
    struct pending_pose *p=&s->pending[s->pending_head];memset(out,0,sizeof(*out));out->type=LAB_EVENT_POSE;out->meta=p->meta;out->data.pose=p->pose;
    s->pending_head=(s->pending_head+1)%PENDING_POSES;--s->pending_count;finish_event(s,out);return LAB_OK;
}
static void make_pose(lab_sim *s,lab_pose *p,uint64_t *stamp) {
    memset(p,0,sizeof(*p));p->quality=LAB_POSE_VALID|LAB_POSE_UNCALIBRATED;
    if(s->config.mode==LAB_SIM_IDEAL){
        const struct sim_truth *t=&s->engine.truth;p->x_m=t->x_m;p->y_m=t->y_m;p->yaw_rad=t->yaw_rad;p->distance_m=t->distance_m;*stamp=s->current_us;
    } else {
        const struct odom_state *e=&s->odometry.state;p->x_m=e->x;p->y_m=e->y;p->yaw_rad=e->heading;p->distance_m=e->total_distance;p->uncertainty_m=e->uncertainty_m;
        if(e->quality==ODOM_WAIT)p->quality&=~LAB_POSE_VALID;
        if(e->quality==ODOM_GAP)p->quality=(p->quality&~LAB_POSE_VALID)|LAB_POSE_GAP;
        if(e->slip)p->quality|=LAB_POSE_SLIP;if(e->imu_used)p->quality|=LAB_POSE_IMU_USED;
        *stamp=s->estimator_time_us;if(s->current_us-*stamp>250000)p->quality=(p->quality&~LAB_POSE_VALID)|LAB_POSE_STALE;
    }
    // 估计器输出完成后，评估模块才读取真值。
    const struct sim_truth *truth=s->config.mode==LAB_SIM_IDEAL?&s->engine.truth:&s->estimator_truth;
    double pe=hypot(p->x_m-truth->x_m,p->y_m-truth->y_m),ye=sim_wrap_rad(p->yaw_rad-truth->yaw_rad);
    s->pos_squared+=pe*pe;s->yaw_squared+=ye*ye;++s->metric_samples;
    s->metrics.position_rmse_m=sqrt(s->pos_squared/s->metric_samples);s->metrics.yaw_rmse_rad=sqrt(s->yaw_squared/s->metric_samples);
    s->metrics.final_position_error_m=pe;s->metrics.final_yaw_error_rad=ye;
}
static void gap(lab_sim *s,lab_event *out,unsigned affected) {
    memset(out,0,sizeof(*out));out->type=LAB_EVENT_GAP;out->meta=meta(s,7,LAB_PROVIDER_SIM_SENSOR,++s->gap_sequence,s->current_us,s->current_us);
    out->data.gap.dropped_events=1;out->data.gap.reason=1;out->data.gap.affected_event_type=affected;
}
static lab_status prepare_tick(lab_sim *s,uint64_t t) {
    s->current_us=t;
    for(unsigned i=0;i<4;++i){s->due[i]=scheduled(s,i)==t;if(s->due[i])++s->next_index[i];}
    if(sim_advance_to(&s->engine,t))return LAB_ERR_TIME;
    double previous_imu=s->last_imu_yaw;
    if(s->started){if(sim_observe(&s->engine,s->due[0],s->due[1]))return LAB_ERR_FORMAT;}
    else s->started=1;
    if(s->due[1]){
        s->imu_rate=t>s->last_imu_us?sim_wrap_rad(s->engine.sample.imu_yaw_rad-previous_imu)/((t-s->last_imu_us)/1000000.0):0;
        s->last_imu_yaw=s->engine.sample.imu_yaw_rad;s->last_imu_us=t;
    }
    if(s->due[0]){
        (void)odom_push(&s->odometry,s->engine.sample.od01,64,t/1000);
        s->estimator_time_us=t;s->estimator_truth=s->engine.truth;
    }
    s->stage=0;return LAB_OK;
}
lab_status lab_sim_next(lab_sim *s,lab_event *out) {
    if(!s||!out)return LAB_ERR_ARGUMENT;
    if(s->reset_pending){
        memset(out,0,sizeof(*out));out->type=LAB_EVENT_RESET;out->meta=meta(s,6,LAB_PROVIDER_SIM_SENSOR,1,0,0);
        out->data.reset.reason=LAB_RESET_START;out->data.reset.next_origin_epoch=1;s->reset_pending=0;finish_event(s,out);return LAB_OK;
    }
    for(;;){
        if(s->pending_count && s->pending[s->pending_head].meta.received_us<=s->current_us)return deliver_pose(s,out);
        if(s->stage>=5){
            uint64_t next=next_tick(s);
            if(next>s->config.duration_us){s->finished=1;if(s->pending_count)return deliver_pose(s,out);return LAB_EOF;}
            if(s->pending_count&&s->pending[s->pending_head].meta.received_us<=next)return deliver_pose(s,out);
            lab_status ready=prepare_tick(s,next);if(ready!=LAB_OK)return ready;
        }
        unsigned stage=(unsigned)s->stage++;memset(out,0,sizeof(*out));
        if(stage==0&&s->due[0]){
            out->type=LAB_EVENT_ENCODER;out->meta=meta(s,3,LAB_PROVIDER_SIM_SENSOR,++s->encoder_sequence,s->current_us,s->current_us);
            for(int i=0;i<4;++i)memcpy(&out->data.encoder.counts[i],&s->engine.sample.counts[i],4);
            out->data.encoder.valid_mask=15;out->data.encoder.counts_per_revolution=s->config.counts_per_revolution;
        } else if(stage==1&&s->due[1]){
            out->type=LAB_EVENT_IMU;out->meta=meta(s,4,LAB_PROVIDER_SIM_SENSOR,++s->imu_sequence,s->current_us,s->current_us);
            out->data.imu.yaw_rad=s->engine.sample.imu_yaw_rad;out->data.imu.yaw_rate_rad_s=s->imu_rate;
            out->data.imu.valid_flags=s->engine.sample.imu_status?0:LAB_IMU_YAW_VALID|(s->current_us?LAB_IMU_RATE_VALID:0);
        } else if(stage==2&&s->due[2]){
            out->type=LAB_EVENT_TRUTH;out->meta=meta(s,5,LAB_PROVIDER_TRUTH,++s->truth_sequence,s->current_us,s->current_us);
            const struct sim_truth *t=&s->engine.truth;out->data.truth=(lab_truth){t->x_m,t->y_m,t->yaw_rad,t->distance_m};++s->metrics.truth_poses;
        } else if(stage==3&&s->due[2]){
            lab_pose pose;uint64_t measurement;make_pose(s,&pose,&measurement);++s->pose_sequence;++s->metrics.estimated_poses;
            if(s->config.drop_pose_every&&s->pose_sequence%s->config.drop_pose_every==0){++s->metrics.dropped_poses;gap(s,out,LAB_EVENT_POSE);}
            else{
                if(s->pending_count==PENDING_POSES)return LAB_ERR_LIMIT;
                struct pending_pose *p=&s->pending[(s->pending_head+s->pending_count)%PENDING_POSES];p->pose=pose;
                p->meta=meta(s,2,s->config.mode==LAB_SIM_IDEAL?LAB_PROVIDER_SIM_IDEAL:LAB_PROVIDER_SIM_ESTIMATE,s->pose_sequence,
                    measurement,s->current_us+s->config.pose_delivery_delay_us);
                if(!(pose.quality&LAB_POSE_VALID))p->meta.flags&=~LAB_DATA_VALID;
                ++s->pending_count;continue;
            }
        } else if(stage==4&&s->due[3]){
            if(sim_scan_now(&s->engine,&s->scan))return LAB_ERR_FORMAT;++s->metrics.generated_scans;
            if(s->config.drop_scan_every&&s->scan.sequence%s->config.drop_scan_every==0){++s->metrics.dropped_scans;gap(s,out,LAB_EVENT_SCAN);}
            else{
                out->type=LAB_EVENT_SCAN;out->meta=meta(s,1,LAB_PROVIDER_SIM_SENSOR,s->scan.sequence,s->current_us,s->current_us);
                struct scan_frame *f=&out->data.scan;f->session=out->meta.stream_session;f->sequence=s->scan.sequence;
                f->source=1;f->flags=0;f->timestamp_us=s->current_us;f->received_ms=s->current_us/1000;
                f->period_us=1000000u/s->config.scan_hz;f->count=s->scan.count;
                for(unsigned i=0;i<f->count;++i){const struct sim_scan_point *p=&s->scan.points[i];f->points[i]=(struct scan_point){p->angle_q14,p->distance_q2,p->quality,p->flags};}
            }
        } else continue;
        if(!lab_event_valid(out))return LAB_ERR_FORMAT;finish_event(s,out);return LAB_OK;
    }
}
void lab_sim_get_metrics(const lab_sim *s,lab_sim_metrics *out) {if(out){if(s)*out=s->metrics;else memset(out,0,sizeof(*out));}}
void lab_sim_close(lab_sim *s) {free(s);}
