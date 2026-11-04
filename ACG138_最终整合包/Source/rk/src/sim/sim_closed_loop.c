#include "sim_closed_loop.h"
#include "sim_engine.h"
#include "odometry.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define IMU_HISTORY 256u
#define PHYSICS_US UINT64_C(1000)
struct pending {
    lab_meta meta;
    uint32_t type,scan_slot,build_pose;
    union {lab_pose pose;lab_truth truth;lab_encoder encoder;lab_imu imu;lab_reset reset;lab_gap gap;} data;
    lab_truth evaluation_truth; /* 仅模拟与评估使用，不作为POSE发送。 */
};
struct imu_history {uint64_t time_us;double yaw;uint32_t sequence;};
struct sim_cl {
    sim_cl_config config;
    struct sim_engine engine;
    struct odom_model odometry;
    struct pending pending[SIM_CL_PENDING_MAX];
    unsigned pending_count;
    struct scan_frame scans[SIM_CL_SCAN_BUFFERS];
    uint32_t scan_used,scan_count;
    struct sim_scan raw_scan;
    uint32_t sequence[8],sessions[8];
    uint64_t next_index[4],now_us,estimator_time_us;
    struct imu_history imu[IMU_HISTORY];
    unsigned imu_head,imu_count;
    double previous_imu_yaw;
    uint64_t previous_imu_us;
    sim_cl_command command;
    uint32_t last_command_sequence;
    int have_command,contact;
    double ramp_v,ramp_w,left_speed,right_speed;
    sim_cl_box obstacles[SIM_CL_OBSTACLES_MAX];
    size_t obstacle_count;
    sim_cl_stats stats;
    sim_cl_evaluation evaluation;
    lab_truth estimator_truth;
    double error_position_sum,error_yaw_sum;
    uint64_t error_samples;
    lab_map_view world;
    int8_t *world_cells;
};

/* DDA每个跨越格只访问一次，未知视为遮挡但不生成障碍回波，其后格不因本查询变得可见。
 */
static double grid_ray(void *user,double x,double y,double heading){
    const sim_cl *s=user;const lab_map_view *m=&s->world;
    double gx=(x-m->origin_x_m)/m->resolution_m,gy=(y-m->origin_y_m)/m->resolution_m;
    if(gx<0||gy<0||gx>=m->width||gy>=m->height)return -1;
    int ix=(int)floor(gx),iy=(int)floor(gy);double dx=cos(heading),dy=sin(heading);
    int sx=dx>=0?1:-1,sy=dy>=0?1:-1;double inf=1e100;
    double tx=fabs(dx)<1e-14?inf:((sx>0?ix+1-gx:gx-ix)*m->resolution_m/fabs(dx));
    double ty=fabs(dy)<1e-14?inf:((sy>0?iy+1-gy:gy-iy)*m->resolution_m/fabs(dy));
    double stepx=fabs(dx)<1e-14?inf:m->resolution_m/fabs(dx),stepy=fabs(dy)<1e-14?inf:m->resolution_m/fabs(dy),distance=0;
    for(uint32_t n=0;n<m->width+m->height+2;++n){
        int cell=m->cells[(size_t)iy*m->width+(unsigned)ix];if(cell<0)return -1;if(cell>=65)return distance;
        if(fabs(tx-ty)<1e-12){
            /* 擦过格角的射线不能通过对角跳跃看穿未知或占用邻格。
 */
            int ax=ix+sx,ay=iy+sy;
            if(ax<0||ay<0||ax>=(int)m->width||ay>=(int)m->height)return -1;
            int a=m->cells[(size_t)iy*m->width+(unsigned)ax],b=m->cells[(size_t)ay*m->width+(unsigned)ix];
            if(a<0||b<0)return -1;if(a>=65||b>=65)return tx;
            distance=tx;tx+=stepx;ty+=stepy;ix=ax;iy=ay;
        }else if(tx<ty){distance=tx;tx+=stepx;ix+=sx;}
        else{distance=ty;ty+=stepy;iy+=sy;}
        if(distance>s->engine.config.range_max_m||ix<0||iy<0||ix>=(int)m->width||iy>=(int)m->height)return -1;
    }
    return -1;
}
static double grid_clearance(const sim_cl *s){
    const lab_map_view *m=&s->world;double x=s->engine.truth.x_m,y=s->engine.truth.y_m;
    double radius=hypot(s->config.vehicle_width_m,s->config.vehicle_length_m)*.5;
    double x1=m->origin_x_m+m->width*m->resolution_m,y1=m->origin_y_m+m->height*m->resolution_m;
    double result=fmin(fmin(x-m->origin_x_m,x1-x),fmin(y-m->origin_y_m,y1-y))-radius;
    /* 接触只检查半径内栅格，工作量受地图尺寸限制。 */
    double a=floor((x-radius-m->origin_x_m)/m->resolution_m),b=floor((x+radius-m->origin_x_m)/m->resolution_m);
    double c=floor((y-radius-m->origin_y_m)/m->resolution_m),d=floor((y+radius-m->origin_y_m)/m->resolution_m);
    int lo_x=(int)fmax(0,fmin(m->width,a)),hi_x=(int)fmin(m->width-1,fmax(-1,b));
    int lo_y=(int)fmax(0,fmin(m->height,c)),hi_y=(int)fmin(m->height-1,fmax(-1,d));
    for(int iy=lo_y;iy<=hi_y;++iy)for(int ix=lo_x;ix<=hi_x;++ix){
        int cell=m->cells[(size_t)iy*m->width+(unsigned)ix];if(cell>=0&&cell<65)continue;
        double bx=m->origin_x_m+ix*m->resolution_m,by=m->origin_y_m+iy*m->resolution_m;
        double dx=fmax(fmax(bx-x,0),x-bx-m->resolution_m),dy=fmax(fmax(by-y,0),y-by-m->resolution_m);
        result=fmin(result,hypot(dx,dy)-radius);
    }
    return result;
}

static int bounded(double x,double lo,double hi){return isfinite(x)&&x>=lo&&x<=hi;}
static double clamp(double x,double lo,double hi){return x<lo?lo:x>hi?hi:x;}
static unsigned rate(const sim_cl *s,unsigned i){
    return i==0?s->config.sensors.encoder_hz:i==1?s->config.sensors.imu_hz:
        i==2?s->config.sensors.pose_hz:s->config.sensors.scan_hz;
}
static uint64_t scheduled(const sim_cl *s,unsigned i){return s->next_index[i]*UINT64_C(1000000)/rate(s,i);}
static lab_truth truth(const sim_cl *s){
    const struct sim_truth *t=&s->engine.truth;
    return (lab_truth){t->x_m,t->y_m,t->yaw_rad,t->distance_m};
}
static lab_meta meta(sim_cl *s,unsigned type,unsigned provider,uint64_t time,uint64_t received){
    lab_meta m={0};m.mode=s->config.sensors.mode;m.provider=provider;m.clock_domain=LAB_CLOCK_SIMULATION;
    m.flags=LAB_TIME_VALID|LAB_DATA_VALID;m.timeline_id=s->config.sensors.timeline_id;m.origin_epoch=s->config.origin_epoch;
    m.stream_id=type;m.stream_session=s->sessions[type];m.sequence=++s->sequence[type];
    m.time_us=time;m.received_us=received;return m;
}
static unsigned priority(const struct pending *p){
    switch(p->type){case LAB_EVENT_RESET:return 0;case LAB_EVENT_IMU:return 1;
        case LAB_EVENT_ENCODER:return 2;case LAB_EVENT_TRUTH:return 3;case LAB_EVENT_POSE:return 4;
        case LAB_EVENT_SCAN:return 5;default:return 6;}
}
static int earlier(const struct pending *a,const struct pending *b){
    if(a->meta.received_us!=b->meta.received_us)return a->meta.received_us<b->meta.received_us;
    /* 构建请求使用实际生成时刻，延迟估计保留原测量时刻，两者仍先于同刻扫描。
 */
    if(a->meta.time_us!=b->meta.time_us)return a->meta.time_us<b->meta.time_us;
    if(priority(a)!=priority(b))return priority(a)<priority(b);
    if(a->build_pose!=b->build_pose)return a->build_pose>b->build_pose;
    return a->meta.sequence<b->meta.sequence;
}
static lab_status push(sim_cl *s,const struct pending *p){
    if(s->pending_count==SIM_CL_PENDING_MAX)return LAB_ERR_LIMIT;
    unsigned i=s->pending_count++;
    while(i){unsigned parent=(i-1)/2;if(!earlier(p,&s->pending[parent]))break;
        s->pending[i]=s->pending[parent];i=parent;}
    s->pending[i]=*p;
    if(s->pending_count>s->stats.pending_high_water)s->stats.pending_high_water=s->pending_count;
    if(!p->build_pose)++s->stats.events_generated;
    return LAB_OK;
}
static struct pending pop(sim_cl *s){
    struct pending result=s->pending[0],last=s->pending[--s->pending_count];
    if(s->pending_count){unsigned i=0;
        while(i*2+1<s->pending_count){unsigned child=i*2+1;
            if(child+1<s->pending_count&&earlier(&s->pending[child+1],&s->pending[child]))++child;
            if(!earlier(&s->pending[child],&last))break;s->pending[i]=s->pending[child];i=child;}
        s->pending[i]=last;
    }
    return result;
}
void sim_cl_config_default(sim_cl_config *c){
    if(!c)return;memset(c,0,sizeof(*c));lab_sim_config_default(&c->sensors);
    c->sensors.path=LAB_PATH_STATIONARY;c->sensors.duration_us=30000000;
    c->scene=SIM_CL_ROOM;c->origin_epoch=1;c->command_source_id=1;c->integration_step_us=10000;
    c->vehicle_width_m=.360;c->vehicle_length_m=.420;
    c->max_linear_m_s=.30;c->max_angular_rad_s=1.5;c->max_wheel_m_s=.60;
    c->acceleration_m_s2=.4;c->braking_m_s2=.8;c->angular_acceleration_rad_s2=2;c->angular_braking_rad_s2=3;
    c->wheel_response_tau_s=.05;c->traction_left=c->traction_right=1;
    c->max_command_valid_us=300000;c->freeze_encoder=-1;
}
static lab_status config_engine(const sim_cl_config *c,struct sim_config *e){
    if(!c)return LAB_ERR_ARGUMENT;const lab_sim_config *b=&c->sensors;
    if(!b->seed||!b->timeline_id||!c->origin_epoch||!c->command_source_id||
       (b->mode!=LAB_SIM_IDEAL&&b->mode!=LAB_SIM_NOISY)||
       !b->duration_us||b->duration_us>600000000||
       !b->scan_hz||b->scan_hz>200||!b->encoder_hz||b->encoder_hz>200||
       !b->imu_hz||b->imu_hz>200||!b->pose_hz||b->pose_hz>200||
       !bounded(b->room_width_m,2,50)||!bounded(b->room_height_m,2,50)||
       c->scene<SIM_CL_ROOM||c->scene>SIM_CL_CORRIDOR||
       c->integration_step_us<1000||c->integration_step_us>20000||
       !bounded(c->vehicle_width_m,.05,1)||!bounded(c->vehicle_length_m,.05,1)||
       !bounded(c->max_linear_m_s,.01,1)||!bounded(c->max_angular_rad_s,.01,3)||
       !bounded(c->max_wheel_m_s,.01,2)||!bounded(c->acceleration_m_s2,.01,10)||
       !bounded(c->braking_m_s2,.01,10)||!bounded(c->angular_acceleration_rad_s2,.01,20)||
       !bounded(c->angular_braking_rad_s2,.01,20)||!bounded(c->wheel_response_tau_s,0,2)||
       !bounded(c->traction_left,0,1)||!bounded(c->traction_right,0,1)||
       !c->max_command_valid_us||c->max_command_valid_us>1000000||
       c->scan_delay_us>1000000||c->encoder_delay_us>1000000||c->imu_delay_us>1000000||
       b->pose_delivery_delay_us>1000000||c->freeze_encoder<-1||c->freeze_encoder>3||
       c->freeze_start_us>600000000||c->freeze_duration_us>600000000)return LAB_ERR_ARGUMENT;
    uint64_t scan_slots=(c->scan_delay_us+c->integration_step_us)*b->scan_hz/1000000+3;
    uint64_t slots=scan_slots+(c->encoder_delay_us+c->integration_step_us)*b->encoder_hz/1000000+3+
        (c->imu_delay_us+c->integration_step_us)*b->imu_hz/1000000+3+
        (b->pose_delivery_delay_us+c->integration_step_us)*b->pose_hz/1000000+3+
        2*c->integration_step_us*b->pose_hz/1000000+12;
    if(scan_slots>SIM_CL_SCAN_BUFFERS||slots>SIM_CL_PENDING_MAX)return LAB_ERR_LIMIT;
    sim_default_config(e,SIM_STATIONARY);e->seed=b->seed;e->duration_us=b->duration_us;
    e->period_us=1000;e->imu_period_us=1000;e->scan_points=b->scan_points;e->origin_epoch=c->origin_epoch;
    e->room_width_m=b->room_width_m;e->room_height_m=b->room_height_m;
    e->wheel_diameter_m=b->wheel_diameter_m;e->track_m=b->track_m;e->counts_per_revolution=b->counts_per_revolution;
    e->left_mask=b->left_mask;e->mount_x_m=b->mount_x_m;e->mount_y_m=b->mount_y_m;e->mount_yaw_rad=b->mount_yaw_rad;
    e->range_noise_m=b->scan_noise_std_m;e->encoder_noise_counts=b->wheel_noise_std_counts;
    e->imu_noise_rad=b->imu_noise_std_rad;e->imu_drift_rad_per_s=b->imu_bias_rad_s;
    e->freeze_wheel=c->freeze_encoder;e->wheel_freeze_start_s=c->freeze_start_us/1000000.0;
    e->wheel_freeze_duration_s=c->freeze_duration_us/1000000.0;
    for(unsigned i=0;i<4;++i){e->encoder_scale[i]=b->wheel_scale[i];e->encoder_sign[i]=b->encoder_signs[i];}
    if(sim_validate_config(e))return LAB_ERR_ARGUMENT;
    if(b->mode==LAB_SIM_IDEAL){
        if(b->scan_noise_std_m||b->wheel_noise_std_counts||b->imu_noise_std_rad||b->imu_bias_rad_s)return LAB_ERR_ARGUMENT;
        for(unsigned i=0;i<4;++i)if(b->wheel_scale[i]!=1)return LAB_ERR_ARGUMENT;
    }
    return LAB_OK;
}
static void box(sim_cl *s,double x0,double y0,double x1,double y1){
    sim_cl_box *b=&s->obstacles[s->obstacle_count++];*b=(sim_cl_box){x0,y0,x1,y1};
    struct sim_segment lines[]={{x0,y0,x1,y0},{x1,y0,x1,y1},{x1,y1,x0,y1},{x0,y1,x0,y0}};
    memcpy(s->engine.segments+s->engine.segment_count,lines,sizeof(lines));s->engine.segment_count+=4;
}
static void scene(sim_cl *s){
    double x=s->config.sensors.room_width_m/8,y=s->config.sensors.room_height_m/6;
    /* 保留原模拟器四面房间墙，替换测试障碍。 */
    s->engine.segment_count=4;
    switch(s->config.scene){
    case SIM_CL_OBSTACLE:box(s,1*x,-.6*y,1.5*x,.6*y);break;
    case SIM_CL_NARROW:
        box(s,.8*x,.24,2.5*x,2*y);box(s,.8*x,-2*y,2.5*x,-.24);break;
    case SIM_CL_DEAD_END:
        box(s,1*x,.55*y,2.6*x,.75*y);box(s,1*x,-.75*y,2.6*x,-.55*y);
        box(s,2.4*x,-.55*y,2.6*x,.55*y);break;
    case SIM_CL_UNREACHABLE:
        box(s,1*x,-1*y,1.15*x,1*y);box(s,2.85*x,-1*y,3*x,1*y);
        box(s,1.15*x,.85*y,2.85*x,1*y);box(s,1.15*x,-1*y,2.85*x,-.85*y);break;
    case SIM_CL_CORRIDOR:
        box(s,-3.8*x,.65*y,3.8*x,2.8*y);box(s,-3.8*x,-2.8*y,3.8*x,-.65*y);break;
    default:break;
    }
}
static double clearance(const sim_cl *s){
    if(s->world_cells)return grid_clearance(s);
    const struct sim_truth *t=&s->engine.truth;double radius=hypot(s->config.vehicle_width_m,s->config.vehicle_length_m)*.5;
    double result=fmin(s->config.sensors.room_width_m*.5-fabs(t->x_m),s->config.sensors.room_height_m*.5-fabs(t->y_m))-radius;
    for(size_t i=0;i<s->obstacle_count;++i){const sim_cl_box *b=&s->obstacles[i];
        double dx=fmax(fmax(b->min_x_m-t->x_m,0),t->x_m-b->max_x_m);
        double dy=fmax(fmax(b->min_y_m-t->y_m,0),t->y_m-b->max_y_m);
        double d=hypot(dx,dy);
        if(dx==0&&dy==0)d=-fmin(fmin(t->x_m-b->min_x_m,b->max_x_m-t->x_m),fmin(t->y_m-b->min_y_m,b->max_y_m-t->y_m));
        result=fmin(result,d-radius);
    }
    return result;
}
static void collision(sim_cl *s,uint64_t time){
    double d=clearance(s);if(d<s->evaluation.minimum_clearance_m)s->evaluation.minimum_clearance_m=d;
    if(d<=0&&!s->contact){++s->stats.collisions;
        if(!s->evaluation.has_collision){s->evaluation.has_collision=1;s->evaluation.first_collision_us=time;
            s->evaluation.first_collision_pose=truth(s);s->evaluation.first_collision_command=s->command;}}
    s->contact=d<=0;
}
static lab_status gap(sim_cl *s,unsigned type,uint64_t time){
    struct pending p={0};p.type=LAB_EVENT_GAP;p.meta=meta(s,LAB_EVENT_GAP,LAB_PROVIDER_SIM_SENSOR,time,time);
    p.data.gap=(lab_gap){1,2101,type};return push(s,&p);
}
static lab_status generate(sim_cl *s,uint64_t t,int initial){
    int due[4];for(unsigned i=0;i<4;++i){due[i]=scheduled(s,i)==t;if(due[i])++s->next_index[i];}
    if(!initial&&sim_observe(&s->engine,due[0],due[1]))return LAB_ERR_FORMAT;
    const struct sim_sample *sample=&s->engine.sample;
    if(due[0]){
        struct pending p={0};p.type=LAB_EVENT_ENCODER;p.meta=meta(s,p.type,LAB_PROVIDER_SIM_SENSOR,t,t+s->config.encoder_delay_us);
        for(unsigned i=0;i<4;++i)memcpy(&p.data.encoder.counts[i],&sample->counts[i],4);
        p.data.encoder.valid_mask=15;p.data.encoder.counts_per_revolution=s->config.sensors.counts_per_revolution;
        p.evaluation_truth=truth(s);
        if(s->config.drop_encoder_every&&p.meta.sequence%s->config.drop_encoder_every==0){
            ++s->stats.encoders_dropped;if(gap(s,p.type,t)!=LAB_OK)return LAB_ERR_LIMIT;
        }else if(push(s,&p)!=LAB_OK)return LAB_ERR_LIMIT;
    }
    if(due[1]){
        struct pending p={0};p.type=LAB_EVENT_IMU;p.meta=meta(s,p.type,LAB_PROVIDER_SIM_SENSOR,t,t+s->config.imu_delay_us);
        p.data.imu.yaw_rad=sample->imu_yaw_rad;
        p.data.imu.yaw_rate_rad_s=t>s->previous_imu_us?sim_wrap_rad(sample->imu_yaw_rad-s->previous_imu_yaw)/((t-s->previous_imu_us)/1000000.0):0;
        p.data.imu.valid_flags=LAB_IMU_YAW_VALID|(t?LAB_IMU_RATE_VALID:0);
        s->previous_imu_yaw=sample->imu_yaw_rad;s->previous_imu_us=t;
        if(s->config.drop_imu_every&&p.meta.sequence%s->config.drop_imu_every==0){
            ++s->stats.imus_dropped;if(gap(s,p.type,t)!=LAB_OK)return LAB_ERR_LIMIT;
        }else if(push(s,&p)!=LAB_OK)return LAB_ERR_LIMIT;
    }
    if(due[2]){
        struct pending p={0};p.type=LAB_EVENT_TRUTH;p.meta=meta(s,p.type,LAB_PROVIDER_TRUTH,t,t);p.data.truth=truth(s);
        if(push(s,&p)!=LAB_OK)return LAB_ERR_LIMIT;
        p.type=LAB_EVENT_POSE;p.meta=meta(s,p.type,s->config.sensors.mode==LAB_SIM_IDEAL?LAB_PROVIDER_SIM_IDEAL:LAB_PROVIDER_SIM_ESTIMATE,t,t);
        p.build_pose=1;if(push(s,&p)!=LAB_OK)return LAB_ERR_LIMIT;
    }
    if(due[3]){
        ++s->stats.scans_generated;
        if(s->config.sensors.drop_scan_every&&(s->sequence[LAB_EVENT_SCAN]+1)%s->config.sensors.drop_scan_every==0){
            ++s->sequence[LAB_EVENT_SCAN];++s->stats.scans_dropped;return gap(s,LAB_EVENT_SCAN,t);
        }
        unsigned slot=0;while(slot<SIM_CL_SCAN_BUFFERS&&(s->scan_used&(UINT32_C(1)<<slot)))++slot;
        if(slot==SIM_CL_SCAN_BUFFERS)return LAB_ERR_LIMIT;
        if(sim_scan_now(&s->engine,&s->raw_scan))return LAB_ERR_FORMAT;
        struct pending p={0};p.type=LAB_EVENT_SCAN;p.meta=meta(s,p.type,LAB_PROVIDER_SIM_SENSOR,t,t+s->config.scan_delay_us);p.scan_slot=slot;
        struct scan_frame *f=&s->scans[slot];memset(f,0,sizeof(*f));
        f->source=1;f->session=p.meta.stream_session;f->sequence=p.meta.sequence;f->timestamp_us=t;
        f->received_ms=p.meta.received_us/1000;f->period_us=1000000/s->config.sensors.scan_hz;f->count=s->raw_scan.count;
        for(unsigned i=0;i<f->count;++i){const struct sim_scan_point *q=&s->raw_scan.points[i];
            f->points[i]=(struct scan_point){q->angle_q14,q->distance_q2,q->quality,q->flags};}
        s->scan_used|=UINT32_C(1)<<slot;++s->scan_count;
        if(s->scan_count>s->stats.scan_buffers_high_water)s->stats.scan_buffers_high_water=s->scan_count;
        if(push(s,&p)!=LAB_OK)return LAB_ERR_LIMIT;
    }
    return LAB_OK;
}
static lab_status open_world(sim_cl **out,const sim_cl_config *config,const lab_map_view *world,double x,double y,double yaw){
    if(!out)return LAB_ERR_ARGUMENT;*out=NULL;struct sim_config e;
    lab_status status=config_engine(config,&e);if(status!=LAB_OK)return status;
    sim_cl *s=calloc(1,sizeof(*s));if(!s)return LAB_ERR_NOMEM;s->config=*config;
    if(sim_init(&s->engine,&e)){free(s);return LAB_ERR_ARGUMENT;}scene(s);
    if(world){
        if(config->sensors.mode!=LAB_SIM_IDEAL||!world->cells||!world->width||!world->height||
           world->width>LAB_MAP_SIDE_MAX||world->height>LAB_MAP_SIDE_MAX||!bounded(world->resolution_m,.01,1)||
           !bounded(world->origin_x_m,-1e6,1e6)||!bounded(world->origin_y_m,-1e6,1e6)||
           !bounded(x,-1e6,1e6)||!bounded(y,-1e6,1e6)||!isfinite(yaw)){free(s);return LAB_ERR_ARGUMENT;}
        size_t n=(size_t)world->width*world->height;
        for(size_t i=0;i<n;++i)if(world->cells[i]<-1||world->cells[i]>100){free(s);return LAB_ERR_FORMAT;}
        s->world_cells=malloc(n);if(!s->world_cells){free(s);return LAB_ERR_NOMEM;}memcpy(s->world_cells,world->cells,n);
        s->world=*world;s->world.cells=s->world_cells;s->world.trace=NULL;s->world.trace_count=0;
        s->engine.world_raycast=grid_ray;s->engine.world_user=s;
        s->engine.truth.x_m=x;s->engine.truth.y_m=y;s->engine.truth.yaw_rad=sim_wrap_rad(yaw);s->engine.truth.unwrapped_yaw_rad=sim_wrap_rad(yaw);
        (void)sim_observe(&s->engine,1,1);
        if(grid_clearance(s)<=0){sim_cl_close(s);return LAB_ERR_ARGUMENT;}
    }
    struct odom_config od;odom_default_config(&od);od.track_mm=config->sensors.track_m*1000;od.left_mask=(int)config->sensors.left_mask;
    for(unsigned i=0;i<4;++i){od.mm_per_count[i]=PI*config->sensors.wheel_diameter_m*1000/config->sensors.counts_per_revolution;od.signs[i]=config->sensors.encoder_signs[i];}
    odom_init(&s->odometry,&od);
    for(unsigned i=1;i<8;++i){s->sessions[i]=config->sensors.seed^(UINT32_C(0x57240001)+i*UINT32_C(0x01020304));if(!s->sessions[i])s->sessions[i]=i;}
    uint64_t delay=config->scan_delay_us;
    if(config->encoder_delay_us>delay)delay=config->encoder_delay_us;
    if(config->imu_delay_us>delay)delay=config->imu_delay_us;
    if(config->sensors.pose_delivery_delay_us>delay)delay=config->sensors.pose_delivery_delay_us;
    s->stats.finish_us=config->sensors.duration_us+delay;s->stats.allocated_bytes=sizeof(*s)+(world?(size_t)world->width*world->height:0);
    s->evaluation.minimum_clearance_m=clearance(s);collision(s,0);
    struct pending reset={0};reset.type=LAB_EVENT_RESET;reset.meta=meta(s,LAB_EVENT_RESET,LAB_PROVIDER_SIM_SENSOR,0,0);
    reset.data.reset=(lab_reset){LAB_RESET_START,config->origin_epoch};
    status=push(s,&reset);if(status==LAB_OK)status=generate(s,0,1);
    if(status!=LAB_OK){sim_cl_close(s);return status;}*out=s;return LAB_OK;
}
lab_status sim_cl_open(sim_cl **out,const sim_cl_config *config){return open_world(out,config,NULL,0,0,0);}
lab_status sim_cl_open_map(sim_cl **out,const sim_cl_config *config,const lab_map_view *world,double x,double y,double yaw){
    if(!world){if(out)*out=NULL;return LAB_ERR_ARGUMENT;}return open_world(out,config,world,x,y,yaw);
}
lab_status sim_cl_submit(sim_cl *s,const sim_cl_command *command){
    if(!s||!command)return LAB_ERR_ARGUMENT;
    lab_status reason=LAB_OK;const lab_context *c=&command->context;
    if(c->mode!=s->config.sensors.mode||c->clock_domain!=LAB_CLOCK_SIMULATION||
       c->timeline_id!=s->config.sensors.timeline_id||c->origin_epoch!=s->config.origin_epoch||
       command->source_id!=s->config.command_source_id)reason=LAB_ERR_CONTEXT;
    else if(!command->sequence||command->sequence<=s->last_command_sequence||
       !isfinite(command->linear_m_s)||!isfinite(command->angular_rad_s))reason=LAB_ERR_ARGUMENT;
    else if(command->generated_us>s->now_us||command->valid_until_us<=s->now_us||
       command->valid_until_us-command->generated_us>s->config.max_command_valid_us||
       s->now_us>=s->config.sensors.duration_us)reason=LAB_ERR_TIME;
    if(reason!=LAB_OK){++s->stats.commands_rejected;return reason;}
    s->command=*command;s->last_command_sequence=command->sequence;s->have_command=1;
    double v=clamp(command->linear_m_s,-s->config.max_linear_m_s,s->config.max_linear_m_s);
    double w=clamp(command->angular_rad_s,-s->config.max_angular_rad_s,s->config.max_angular_rad_s);
    double l=v-w*s->config.sensors.track_m*.5,r=v+w*s->config.sensors.track_m*.5;
    double peak=fmax(fabs(l),fabs(r)),scale=peak>s->config.max_wheel_m_s?s->config.max_wheel_m_s/peak:1;
    v*=scale;w*=scale;
    if(v!=command->linear_m_s||w!=command->angular_rad_s)++s->stats.commands_clamped;
    s->command.linear_m_s=v;s->command.angular_rad_s=w;++s->stats.commands_accepted;return LAB_OK;
}
static double ramp(double current,double target,double acceleration,double braking,double dt){
    /* 反向先制动到零，再改变方向，不直接越过零速。 */
    if(current*target<0)target=0;
    double amount=(fabs(target)<fabs(current)?braking:acceleration)*dt;
    return current+clamp(target-current,-amount,amount);
}
static lab_status motion(sim_cl *s,uint64_t stop){
    double dt=(stop-s->engine.truth.elapsed_us)/1000000.0;
    if(s->have_command&&s->engine.truth.elapsed_us>=s->command.valid_until_us){
        s->have_command=0;s->ramp_v=s->ramp_w=s->left_speed=s->right_speed=0;++s->stats.command_expirations;
    }
    double target_v=s->have_command?s->command.linear_m_s:0,target_w=s->have_command?s->command.angular_rad_s:0;
    s->ramp_v=ramp(s->ramp_v,target_v,s->config.acceleration_m_s2,s->config.braking_m_s2,dt);
    s->ramp_w=ramp(s->ramp_w,target_w,s->config.angular_acceleration_rad_s2,s->config.angular_braking_rad_s2,dt);
    double l=s->ramp_v-s->ramp_w*s->config.sensors.track_m*.5,r=s->ramp_v+s->ramp_w*s->config.sensors.track_m*.5;
    double alpha=s->config.wheel_response_tau_s?1-exp(-dt/s->config.wheel_response_tau_s):1;
    s->left_speed=clamp(s->left_speed+alpha*(l-s->left_speed),-s->config.max_wheel_m_s,s->config.max_wheel_m_s);
    s->right_speed=clamp(s->right_speed+alpha*(r-s->right_speed),-s->config.max_wheel_m_s,s->config.max_wheel_m_s);
    l=s->left_speed*dt;r=s->right_speed*dt;
    if(sim_advance_wheels(&s->engine,stop,l,r,l*s->config.traction_left,r*s->config.traction_right))return LAB_ERR_TIME;
    collision(s,stop);return LAB_OK;
}
lab_status sim_cl_step(sim_cl *s,uint64_t target){
    if(!s)return LAB_ERR_ARGUMENT;
    if(target<=s->now_us||target-s->now_us>s->config.integration_step_us||target>s->stats.finish_us)return LAB_ERR_TIME;
    if(s->pending_count&&s->pending[0].meta.received_us<=s->now_us)return LAB_AGAIN;
    uint64_t end=s->config.sensors.duration_us;
    while(s->engine.truth.elapsed_us<target&&s->engine.truth.elapsed_us<end){
        uint64_t stop=target;if(stop>end)stop=end;
        uint64_t tick=(s->engine.truth.elapsed_us/PHYSICS_US+1)*PHYSICS_US;if(tick<stop)stop=tick;
        for(unsigned i=0;i<4;++i){uint64_t t=scheduled(s,i);if(t<stop)stop=t;}
        if(s->have_command&&s->command.valid_until_us>s->engine.truth.elapsed_us&&s->command.valid_until_us<stop)stop=s->command.valid_until_us;
        lab_status status=motion(s,stop);if(status!=LAB_OK)return status;
        if(s->have_command&&stop==s->command.valid_until_us){
            s->have_command=0;s->ramp_v=s->ramp_w=s->left_speed=s->right_speed=0;++s->stats.command_expirations;
        }
        int due=0;for(unsigned i=0;i<4;++i)due|=scheduled(s,i)==stop;
        if(due){status=generate(s,stop,0);if(status!=LAB_OK)return status;}
    }
    s->now_us=target;
    if(target>=end){s->have_command=0;s->ramp_v=s->ramp_w=s->left_speed=s->right_speed=0;}
    return LAB_OK;
}
static void delivered_imu(sim_cl *s,const struct pending *p){
    unsigned index=(s->imu_head+s->imu_count)%IMU_HISTORY;
    if(s->imu_count==IMU_HISTORY){index=s->imu_head;s->imu_head=(s->imu_head+1)%IMU_HISTORY;}else ++s->imu_count;
    s->imu[index]=(struct imu_history){p->meta.time_us,p->data.imu.yaw_rad,p->meta.sequence};
}
static void delivered_encoder(sim_cl *s,const struct pending *p){
    struct sim_sample sample={0};sample.sequence=p->meta.sequence;sample.timestamp_us=p->meta.time_us;
    sample.origin_epoch=s->config.origin_epoch;sample.imu_token=s->engine.config.imu_token;sample.imu_status=1;
    sample.imu_age_ms=UINT32_MAX;
    for(unsigned i=0;i<4;++i)memcpy(&sample.counts[i],&p->data.encoder.counts[i],4);
    for(unsigned n=s->imu_count;n>0;--n){
        const struct imu_history *h=&s->imu[(s->imu_head+n-1)%IMU_HISTORY];
        if(h->time_us<=p->meta.time_us){sample.imu_yaw_rad=(float)h->yaw;sample.imu_sequence=h->sequence;
            sample.imu_age_ms=(uint32_t)((p->meta.time_us-h->time_us)/1000);sample.imu_status=0;break;}
    }
    sim_pack_od01(&sample,sample.od01);
    (void)odom_push(&s->odometry,sample.od01,sizeof(sample.od01),p->meta.time_us/1000);
    s->estimator_time_us=p->meta.time_us;s->estimator_truth=p->evaluation_truth;
}
static lab_status build_pose(sim_cl *s,struct pending *p){
    uint64_t generated=p->meta.time_us;
    lab_truth expected=p->data.truth;lab_pose pose={0};pose.quality=LAB_POSE_VALID|LAB_POSE_UNCALIBRATED;
    if(s->config.sensors.mode==LAB_SIM_IDEAL){
        pose.x_m=expected.x_m;pose.y_m=expected.y_m;pose.yaw_rad=expected.yaw_rad;pose.distance_m=expected.distance_m;
    }else{
        const struct odom_state *e=&s->odometry.state;
        pose.x_m=e->x;pose.y_m=e->y;pose.yaw_rad=e->heading;pose.distance_m=e->total_distance;pose.uncertainty_m=e->uncertainty_m;
        if(e->quality==ODOM_WAIT)pose.quality&=~LAB_POSE_VALID;
        if(e->quality==ODOM_GAP)pose.quality=(pose.quality&~LAB_POSE_VALID)|LAB_POSE_GAP;
        if(e->slip)pose.quality|=LAB_POSE_SLIP;if(e->imu_used)pose.quality|=LAB_POSE_IMU_USED;
        p->meta.time_us=s->estimator_time_us;expected=s->estimator_truth;
        if(generated-p->meta.time_us>250000)pose.quality=(pose.quality&~LAB_POSE_VALID)|LAB_POSE_STALE;
    }
    /* 估计完成后才评估，这些真值不进入里程计。 */
    double pe=hypot(pose.x_m-expected.x_m,pose.y_m-expected.y_m),ye=sim_wrap_rad(pose.yaw_rad-expected.yaw_rad);
    s->error_position_sum+=pe*pe;s->error_yaw_sum+=ye*ye;++s->error_samples;
    lab_sim_metrics *m=&s->evaluation.odometry;++m->estimated_poses;
    m->position_rmse_m=sqrt(s->error_position_sum/s->error_samples);m->yaw_rmse_rad=sqrt(s->error_yaw_sum/s->error_samples);
    m->final_position_error_m=pe;m->final_yaw_error_rad=fabs(ye);
    if(s->config.sensors.drop_pose_every&&p->meta.sequence%s->config.sensors.drop_pose_every==0){
        ++s->stats.poses_dropped;return gap(s,LAB_EVENT_POSE,generated);
    }
    p->build_pose=0;p->meta.received_us=generated+s->config.sensors.pose_delivery_delay_us;p->data.pose=pose;
    if(!(pose.quality&LAB_POSE_VALID))p->meta.flags&=~LAB_DATA_VALID;
    return push(s,p);
}
lab_status sim_cl_next(sim_cl *s,lab_event *out){
    if(!s||!out)return LAB_ERR_ARGUMENT;
    for(;;){
        if(!s->pending_count||s->pending[0].meta.received_us>s->now_us)
            return s->now_us==s->stats.finish_us&&!s->pending_count?LAB_EOF:LAB_AGAIN;
        struct pending p=pop(s);
        if(p.build_pose){lab_status status=build_pose(s,&p);if(status!=LAB_OK)return status;continue;}
        if(p.type==LAB_EVENT_IMU)delivered_imu(s,&p);
        if(p.type==LAB_EVENT_ENCODER)delivered_encoder(s,&p);
        memset(out,0,sizeof(*out));out->type=p.type;out->meta=p.meta;
        switch(p.type){
        case LAB_EVENT_SCAN:out->data.scan=s->scans[p.scan_slot];s->scan_used&=~(UINT32_C(1)<<p.scan_slot);--s->scan_count;break;
        case LAB_EVENT_POSE:out->data.pose=p.data.pose;break;
        case LAB_EVENT_ENCODER:out->data.encoder=p.data.encoder;break;
        case LAB_EVENT_IMU:out->data.imu=p.data.imu;break;
        case LAB_EVENT_TRUTH:out->data.truth=p.data.truth;break;
        case LAB_EVENT_RESET:out->data.reset=p.data.reset;break;
        case LAB_EVENT_GAP:out->data.gap=p.data.gap;break;
        default:return LAB_ERR_FORMAT;
        }
        if(!lab_event_valid(out))return LAB_ERR_FORMAT;++s->stats.events_delivered;return LAB_OK;
    }
}
void sim_cl_get_stats(const sim_cl *s,sim_cl_stats *out){
    if(!out)return;if(!s){memset(out,0,sizeof(*out));return;}*out=s->stats;
    out->now_us=s->now_us;out->pending_events=s->pending_count;out->wheel_left_m_s=s->left_speed;out->wheel_right_m_s=s->right_speed;
}
void sim_cl_get_evaluation(const sim_cl *s,sim_cl_evaluation *out){
    if(!out)return;if(!s){memset(out,0,sizeof(*out));return;}*out=s->evaluation;out->truth=truth(s);
    out->odometry.generated_events=s->stats.events_generated;out->odometry.generated_scans=s->stats.scans_generated;
    out->odometry.dropped_scans=s->stats.scans_dropped;out->odometry.dropped_poses=s->stats.poses_dropped;
    out->odometry.truth_poses=s->sequence[LAB_EVENT_TRUTH];
}
lab_status sim_cl_scene_boxes(const sim_cl *s,sim_cl_box *boxes,size_t capacity,size_t *required){
    if(!s||!required||(!boxes&&capacity))return LAB_ERR_ARGUMENT;*required=s->obstacle_count;
    if(s->world_cells){*required=0;return LAB_ERR_UNAVAILABLE;}
    if(capacity<s->obstacle_count)return LAB_ERR_LIMIT;
    if(s->obstacle_count)memcpy(boxes,s->obstacles,s->obstacle_count*sizeof(*boxes));return LAB_OK;
}
void sim_cl_close(sim_cl *s){if(s){free(s->world_cells);free(s);}}
