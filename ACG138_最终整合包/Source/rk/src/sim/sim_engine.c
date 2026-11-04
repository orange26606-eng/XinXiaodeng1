#include "sim_engine.h"
#include <float.h>
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846
_Static_assert(sizeof(float)==4,"OD01 requires a 32-bit IEEE float");
double sim_wrap_rad(double value) {return atan2(sin(value),cos(value));}
static uint32_t random_u32(uint32_t *state) {
    uint32_t n=*state;n^=n<<13;n^=n>>17;n^=n<<5;*state=n;return n;
}
static double uniform(uint32_t *state) {return (random_u32(state)+1.0)/4294967297.0;}
static double gaussian(uint32_t *state,double sigma) {
    if(sigma==0)return 0;
    return sigma*sqrt(-2*log(uniform(state)))*cos(2*PI*uniform(state));
}
static void le32(uint8_t *p,uint32_t v) {for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(v>>(i*8));}
static uint32_t crc32(const uint8_t *p,size_t n) {
    uint32_t crc=~0u;for(size_t i=0;i<n;++i){crc^=p[i];for(int j=0;j<8;++j)crc=(crc>>1)^((crc&1)?0xedb88320u:0);}return ~crc;
}
void sim_pack_od01(const struct sim_sample *s,uint8_t p[SIM_OD01_BYTES]) {
    memset(p,0,SIM_OD01_BYTES);memcpy(p,"OD01",4);p[4]=1;p[6]=15;p[7]=s->imu_status;
    le32(p+8,s->sequence);le32(p+12,(uint32_t)(s->timestamp_us/1000));le32(p+16,s->origin_epoch);
    for(int i=0;i<4;++i)le32(p+20+i*4,s->counts[i]);
    uint32_t yaw;memcpy(&yaw,&s->imu_yaw_rad,4);le32(p+36,yaw);
    le32(p+40,s->imu_age_ms);le32(p+44,s->imu_sequence);le32(p+48,s->imu_token);le32(p+60,crc32(p,60));
}
const char *sim_path_name(enum sim_path path) {
    static const char *const names[]={"stationary","straight","turn","arc","circle","rectangle"};
    return path>=SIM_STATIONARY&&path<=SIM_RECTANGLE?names[path]:"invalid";
}
double sim_duration_s(enum sim_path path) {
    switch(path){case SIM_STATIONARY:return 10;case SIM_STRAIGHT:return 20;case SIM_TURN:return 12;
        case SIM_ARC:return 20;case SIM_CIRCLE:return 40;case SIM_RECTANGLE:return 100;default:return 0;}
}
void sim_default_config(struct sim_config *c,enum sim_path path) {
    memset(c,0,sizeof(*c));c->path=path;c->seed=20261026;c->period_us=20000;c->scan_period_us=100000;c->scan_points=500;
    c->scan_session=0x53494d01;c->origin_epoch=1;c->imu_token=0x494d5501;
    c->duration_us=(uint64_t)llround(sim_duration_s(path)*1000000);c->speed_m_s=.1;
    c->yaw_rate_rad_s=path==SIM_TURN?PI/6:path==SIM_CIRCLE?PI/20:path==SIM_RECTANGLE?PI/10:.1;
    c->rectangle_leg_m=2;c->room_width_m=8;c->room_height_m=6;
    c->wheel_diameter_m=.0675;c->track_m=.290;c->counts_per_revolution=1320;c->left_mask=5;c->imu_period_us=20000;
    for(int i=0;i<4;++i){c->encoder_sign[i]=(i%2)?-1:1;c->encoder_scale[i]=1;}
    c->range_min_m=.05;c->range_max_m=12;c->freeze_wheel=-1;
}
static int bounded(double n,double lo,double hi) {return isfinite(n)&&n>=lo&&n<=hi;}
int sim_validate_config(const struct sim_config *c) {
    if(!c || c->path<SIM_STATIONARY || c->path>SIM_RECTANGLE || !c->seed ||
       c->period_us<1000 || c->period_us>100000 || c->period_us%1000 ||
       c->scan_period_us<c->period_us || c->scan_period_us>1000000 ||
       c->scan_period_us%c->period_us || !c->scan_points || c->scan_points>SIM_MAX_POINTS ||
       !c->duration_us || c->duration_us>UINT64_C(3600000000) || !c->imu_period_us || c->imu_period_us>1000000 || c->imu_period_us%c->period_us ||
       !bounded(c->speed_m_s,0,1) || !bounded(c->yaw_rate_rad_s,-3,3) || !bounded(c->rectangle_leg_m,.1,10) ||
       !bounded(c->room_width_m,2,100) || !bounded(c->room_height_m,2,100) ||
       ((c->path==SIM_RECTANGLE||c->path==SIM_CIRCLE) && (c->speed_m_s<=0 || fabs(c->yaw_rate_rad_s)<.001)) ||
       !c->scan_session || !c->imu_token || !c->origin_epoch ||
       !bounded(c->wheel_diameter_m,.02,.5) || !bounded(c->track_m,.05,1) || c->counts_per_revolution<1 || c->counts_per_revolution>1000000 ||
       !bounded(c->mount_x_m,-1,1) || !bounded(c->mount_y_m,-1,1) || !bounded(c->mount_yaw_rad,-2*PI,2*PI) ||
       !bounded(c->range_min_m,.01,1) || !bounded(c->range_max_m,1,100) || c->range_min_m>=c->range_max_m ||
       !bounded(c->encoder_noise_counts,0,100) || !bounded(c->imu_noise_rad,0,.5) || !bounded(c->imu_drift_rad_per_s,-.1,.1) ||
       !bounded(c->imu_bias_rad,-PI,PI) || !bounded(c->range_noise_m,0,.5) || !bounded(c->range_dropout_probability,0,1) ||
       !bounded(c->imu_dropout_start_s,0,1000) || !bounded(c->imu_dropout_duration_s,0,1000) ||
       c->freeze_wheel<-1 || c->freeze_wheel>3 || !bounded(c->wheel_freeze_start_s,0,1000) || !bounded(c->wheel_freeze_duration_s,0,1000) ||
       !bounded(c->imu_spike_start_s,0,1000) || !bounded(c->imu_spike_duration_s,0,1000) || !bounded(c->imu_spike_rad,-PI,PI))return -1;
    unsigned left=0;for(int i=0;i<4;++i){left+=(c->left_mask>>i)&1u;if((c->encoder_sign[i]!=1&&c->encoder_sign[i]!=-1)||!bounded(c->encoder_scale[i],.5,1.5))return -1;}
    if(c->left_mask>15||left!=2)return -1;
    return 0;
}
static int in_window(double t,double start,double duration) {return duration>0&&t>=start&&t<start+duration;}
static void sample(struct sim_engine *e,int encoder_new,int imu_new) {
    const struct sim_config *c=&e->config;struct sim_sample *s=&e->sample;
    double t=e->truth.elapsed_us/1000000.0;
    s->timestamp_us=(uint64_t)c->start_ms*1000+e->truth.elapsed_us;s->origin_epoch=c->origin_epoch;s->imu_token=c->imu_token;
    for(int i=0;encoder_new&&i<4;++i){
        double measured=e->measured_wheel_counts[i]+gaussian(&e->rng_encoder,c->encoder_noise_counts);
        int64_t count=llround(measured)*c->encoder_sign[i];s->counts[i]=c->initial_counts[i]+(uint32_t)count;
    }
    if(imu_new && !in_window(t,c->imu_dropout_start_s,c->imu_dropout_duration_s)) {
        double yaw=e->truth.unwrapped_yaw_rad+c->imu_bias_rad+c->imu_drift_rad_per_s*t+gaussian(&e->rng_imu,c->imu_noise_rad);
        if(in_window(t,c->imu_spike_start_s,c->imu_spike_duration_s))yaw+=c->imu_spike_rad;
        e->last_imu_yaw=(float)sim_wrap_rad(yaw);e->last_imu_us=s->timestamp_us;++s->imu_sequence;s->imu_status=0;
    } else s->imu_status=(uint8_t)in_window(t,c->imu_dropout_start_s,c->imu_dropout_duration_s);
    s->imu_yaw_rad=e->last_imu_yaw;s->imu_age_ms=(uint32_t)((s->timestamp_us-e->last_imu_us)/1000);
    sim_pack_od01(s,s->od01);
}
int sim_init(struct sim_engine *e,const struct sim_config *c) {
    if(!e||sim_validate_config(c))return -1;memset(e,0,sizeof(*e));e->config=*c;
    e->rng_encoder=c->seed^0x8a5cd789u;e->rng_imu=c->seed^0x3b17a2c5u;e->rng_lidar=c->seed^0xc8421ef3u;
    if(!e->rng_encoder)e->rng_encoder=1;if(!e->rng_imu)e->rng_imu=1;if(!e->rng_lidar)e->rng_lidar=1;
    double w=c->room_width_m*.5,h=c->room_height_m*.5,bx0=-w*.5,bx1=-w*.375,by0=h*(.7/3),by1=h*.4;
    const struct sim_segment scene[]={
        {-w,-h,w,-h},{w,-h,w,h},{w,h,-w,h},{-w,h,-w,-h},
        {bx0,by0,bx1,by0},{bx1,by0,bx1,by1},{bx1,by1,bx0,by1},{bx0,by1,bx0,by0}
    };
    e->segment_count=sizeof(scene)/sizeof(scene[0]);memcpy(e->segments,scene,sizeof(scene));
    e->last_imu_us=(uint64_t)c->start_ms*1000;e->sample.sequence=1;sample(e,1,1);return 0;
}
static void command(const struct sim_config *c,double t,double *v,double *w,double *boundary_s) {
    *v=*w=0;*boundary_s=DBL_MAX;
    switch(c->path){
        case SIM_STRAIGHT:*v=c->speed_m_s;break;
        case SIM_TURN:*w=c->yaw_rate_rad_s;break;
        case SIM_ARC:case SIM_CIRCLE:*v=c->speed_m_s;*w=c->yaw_rate_rad_s;break;
        case SIM_RECTANGLE:{
            double straight=c->rectangle_leg_m/c->speed_m_s,turn=PI*.5/fabs(c->yaw_rate_rad_s),cycle=straight+turn;
            double start=floor((t+5e-7)/cycle)*cycle,phase=t-start;
            if(phase<straight-5e-7){*v=c->speed_m_s;*boundary_s=start+straight;}
            else{*w=c->yaw_rate_rad_s;*boundary_s=start+cycle;}
            break;
        }
        default:break;
    }
}
int sim_advance_to(struct sim_engine *e,uint64_t stop) {
    if(!e||stop<e->truth.elapsed_us||stop>e->config.duration_us)return -1;
    while(e->truth.elapsed_us<stop){
    uint64_t us=stop-e->truth.elapsed_us;double boundary,v,w,t=e->truth.elapsed_us/1000000.0;command(&e->config,t,&v,&w,&boundary);
    if(boundary!=DBL_MAX){uint64_t boundary_us=(uint64_t)llround(boundary*1000000);if(boundary_us>e->truth.elapsed_us&&boundary_us-e->truth.elapsed_us<us)us=boundary_us-e->truth.elapsed_us;}
    double dt=us/1000000.0;
    double ds=v*dt,dtheta=w*dt,half=dtheta*.5,scale=fabs(half)<1e-10?1:sin(half)/half;
    e->truth.x_m+=ds*scale*cos(e->truth.unwrapped_yaw_rad+half);e->truth.y_m+=ds*scale*sin(e->truth.unwrapped_yaw_rad+half);
    e->truth.unwrapped_yaw_rad+=dtheta;e->truth.yaw_rad=sim_wrap_rad(e->truth.unwrapped_yaw_rad);e->truth.distance_m+=fabs(ds);
    double travel[2]={(v-w*e->config.track_m*.5)*dt,(v+w*e->config.track_m*.5)*dt};
    double count_m=PI*e->config.wheel_diameter_m/e->config.counts_per_revolution;
    for(int i=0;i<4;++i){double distance=travel[(e->config.left_mask>>i)&1u?0:1];e->truth.wheel_travel_m[i]+=distance;
        if(!(e->config.freeze_wheel==i&&in_window(t,e->config.wheel_freeze_start_s,e->config.wheel_freeze_duration_s)))
            e->measured_wheel_counts[i]+=distance/count_m*e->config.encoder_scale[i];
    }
    e->truth.elapsed_us+=us;
    }
    return 0;
}
int sim_observe(struct sim_engine *e,int encoder_new,int imu_new) {
    if(!e || (encoder_new!=0&&encoder_new!=1) || (imu_new!=0&&imu_new!=1))return -1;
    if(encoder_new)++e->sample.sequence;sample(e,encoder_new,imu_new);return 0;
}
int sim_advance_wheels(struct sim_engine *e,uint64_t stop,double lw,double rw,double lg,double rg) {
    if(!e||stop<=e->truth.elapsed_us||stop>e->config.duration_us||
       !isfinite(lw)||!isfinite(rw)||!isfinite(lg)||!isfinite(rg))return -1;
    double ds=(lg+rg)*.5,turn=(rg-lg)/e->config.track_m,half=turn*.5;
    double scale=fabs(half)<1e-10?1:sin(half)/half;
    e->truth.x_m+=ds*scale*cos(e->truth.unwrapped_yaw_rad+half);
    e->truth.y_m+=ds*scale*sin(e->truth.unwrapped_yaw_rad+half);
    e->truth.unwrapped_yaw_rad+=turn;e->truth.yaw_rad=sim_wrap_rad(e->truth.unwrapped_yaw_rad);
    e->truth.distance_m+=fabs(ds);
    double count_m=PI*e->config.wheel_diameter_m/e->config.counts_per_revolution;
    double t=e->truth.elapsed_us/1000000.0;
    for(int i=0;i<4;++i){
        double d=((e->config.left_mask>>i)&1u)?lw:rw;e->truth.wheel_travel_m[i]+=d;
        if(!(e->config.freeze_wheel==i&&in_window(t,e->config.wheel_freeze_start_s,e->config.wheel_freeze_duration_s)))
            e->measured_wheel_counts[i]+=d/count_m*e->config.encoder_scale[i];
    }
    e->truth.elapsed_us=stop;return 0;
}
int sim_step(struct sim_engine *e) {
    if(!e)return -1;uint64_t end=e->config.duration_us;
    if(e->truth.elapsed_us>=end)return 0;
    uint64_t stop=e->truth.elapsed_us+e->config.period_us;if(stop>end)stop=end;
    if(sim_advance_to(e,stop))return -1;
    sim_observe(e,1,e->truth.elapsed_us%e->config.imu_period_us==0);return 1;
}
static double cross(double ax,double ay,double bx,double by) {return ax*by-ay*bx;}
double sim_raycast(const struct sim_engine *e,double x,double y,double heading) {
    double dx=cos(heading),dy=sin(heading),nearest=DBL_MAX;
    for(unsigned i=0;i<e->segment_count;++i){
        const struct sim_segment *s=&e->segments[i];double sx=s->x1_m-s->x0_m,sy=s->y1_m-s->y0_m;
        double denom=cross(dx,dy,sx,sy);if(fabs(denom)<1e-12)continue;
        double ax=s->x0_m-x,ay=s->y0_m-y,t=cross(ax,ay,sx,sy)/denom,u=cross(ax,ay,dx,dy)/denom;
        if(t>=0&&u>=-1e-10&&u<=1+1e-10&&t<nearest)nearest=t;
    }
    return nearest==DBL_MAX?-1:nearest;
}
int sim_scan_now(struct sim_engine *e,struct sim_scan *out) {
    if(!e||!out)return -1;memset(out,0,sizeof(*out));const struct sim_config *c=&e->config;
    out->session=c->scan_session;out->sequence=++e->scan_sequence;out->timestamp_us=e->sample.timestamp_us;
    out->period_us=c->scan_period_us;out->count=(uint16_t)c->scan_points;
    double yaw=e->truth.unwrapped_yaw_rad,origin_x=e->truth.x_m+cos(yaw)*c->mount_x_m-sin(yaw)*c->mount_y_m;
    double origin_y=e->truth.y_m+sin(yaw)*c->mount_x_m+cos(yaw)*c->mount_y_m;
    for(unsigned i=0;i<c->scan_points;++i){
        struct sim_scan_point *p=&out->points[i];uint32_t angle=(uint32_t)llround(i*65536.0/c->scan_points);
        p->angle_q14=(uint16_t)angle;p->flags=i==0?1:0;
        double rad=p->angle_q14*(2*PI/65536.0),heading=yaw+c->mount_yaw_rad-rad;
        double distance=e->world_raycast?e->world_raycast(e->world_user,origin_x,origin_y,heading):sim_raycast(e,origin_x,origin_y,heading);
        if(distance<c->range_min_m||distance>c->range_max_m)continue;
        if(c->range_dropout_probability>0&&uniform(&e->rng_lidar)<c->range_dropout_probability)continue;
        distance+=gaussian(&e->rng_lidar,c->range_noise_m);
        if(distance<c->range_min_m||distance>c->range_max_m)continue;
        p->distance_q2=(uint32_t)llround(distance*4000);p->quality=15;
    }
    return 0;
}
