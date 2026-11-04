#include "lab_live.h"
#include "lab_localization.h"
#include "lab_chassis.h"
#include <math.h>
#include <string.h>
int lab_real_context_valid(const lab_real_context *c){
 if(!c||!c->timeline_id||!c->origin_epoch)return 0;
 if(c->source==LAB_REAL_HARDWARE)return c->clock==LAB_REAL_MONOTONIC||c->clock==LAB_REAL_UNMAPPED_CLOCK;
 if(c->source==LAB_REAL_RECORDING)return c->clock==LAB_REAL_REPLAY_CLOCK;
 return c->source==LAB_REAL_TEST_FIXTURE&&(c->clock==LAB_REAL_MONOTONIC||c->clock==LAB_REAL_REPLAY_CLOCK);
}
int lab_real_context_equal(const lab_real_context *a,const lab_real_context *b){
 return a&&b&&a->source==b->source&&a->clock==b->clock&&a->timeline_id==b->timeline_id&&a->origin_epoch==b->origin_epoch;
}
int lab_real_meta_valid(const lab_real_meta *m){
 return m&&lab_real_context_valid(&m->context)&&m->provider>=1&&m->provider<=4&&m->stream_id&&m->stream_session&&
 !m->reserved&&!(m->flags&~15u)&&m->parameter_version&&m->received_us>=m->time_us&&
 (!(m->flags&LAB_REAL_TIME_ESTIMATED)||m->time_uncertainty_us>0)&&
 !(m->context.clock==LAB_REAL_UNMAPPED_CLOCK&&(m->flags&LAB_REAL_TIME_VALID));
}
int lab_live_sample_valid(const lab_live_sample *e){
 if(!e||e->reserved||!lab_real_meta_valid(&e->meta))return 0;
 switch(e->type){
 case LAB_EVENT_SCAN:{const struct scan_frame *s=&e->data.scan;return s->count&&s->count<=SCAN_MAX_POINTS&&s->source>=1&&s->source<=2&&
  !(s->flags&~1u)&&s->session==e->meta.stream_session&&s->sequence==e->meta.sequence;}
 case LAB_EVENT_POSE:{const lab_pose *p=&e->data.odometry;return e->meta.provider==LAB_REAL_ODOMETRY&&isfinite(p->x_m)&&isfinite(p->y_m)&&
  isfinite(p->yaw_rad)&&isfinite(p->distance_m)&&p->distance_m>=0&&isfinite(p->uncertainty_m)&&p->uncertainty_m>=0&&p->calibrated<=1&&!(p->quality&~63u);}
 case LAB_EVENT_ENCODER:return !(e->data.encoder.valid_mask&~15u)&&e->data.encoder.counts_per_revolution&&e->data.encoder.counts_per_revolution<=1000000;
 case LAB_EVENT_IMU:return isfinite(e->data.imu.yaw_rad)&&isfinite(e->data.imu.yaw_rate_rad_s)&&!(e->data.imu.valid_flags&~3u);
 case LAB_EVENT_RESET:return e->data.reset.reason>=1&&e->data.reset.reason<=5&&e->data.reset.next_origin_epoch;
 case LAB_EVENT_GAP:return e->data.gap.dropped_events&&e->data.gap.reason&&e->data.gap.affected_event_type<=7;
 default:return 0; /* 真实输入接口拒绝TRUTH事件。 */
 }
}
void lab_live_source_config_default(lab_live_source_config *c){
 if(!c)return;memset(c,0,sizeof(*c));strcpy(c->lidar_device,"/dev/ttyUSB0");c->ls01_port=6210;c->read_existing_odometry=1;
 c->max_scan_age_us=c->max_odometry_age_us=250000;c->parameter_version=1;
}
void lab_localization_config_default(lab_localization_config *c){
 if(!c)return;memset(c,0,sizeof(*c));c->mode=LAB_REAL_BUILD_MAP;c->width=c->height=400;c->resolution_m=.05;
 c->origin_x_m=c->origin_y_m=-10;c->allow_uncalibrated=1;c->max_pose_age_us=250000;c->max_interpolation_gap_us=100000;c->parameter_version=1;
}
int lab_real_grid_valid(const lab_real_grid_view *m){
 if(!m||!lab_real_context_valid(&m->context)||!m->map_id||!m->map_version||!m->cells||!m->width||!m->height||
 m->width>LAB_MAP_SIDE_MAX||m->height>LAB_MAP_SIDE_MAX||m->readonly>1||m->trace_count>LAB_TRACE_MAX||
 (m->trace_count&&!m->trace)||!isfinite(m->resolution_m)||m->resolution_m<.01||m->resolution_m>1||
 !isfinite(m->origin_x_m)||!isfinite(m->origin_y_m)||fabs(m->origin_x_m)>1e6||fabs(m->origin_y_m)>1e6)return 0;
 for(size_t i=0;i<(size_t)m->width*m->height;++i)if(m->cells[i]<-1||m->cells[i]>100)return 0;
 for(uint32_t i=0;i<m->trace_count;++i)if(!isfinite(m->trace[i].x_m)||!isfinite(m->trace[i].y_m))return 0;return 1;
}
void lab_chassis_config_default(lab_chassis_config *c){
 if(!c)return;memset(c,0,sizeof(*c));c->backend=LAB_CHASSIS_DRY_RUN;c->left_mask=5;c->track_m=.290;
 for(int i=0;i<4;++i){c->wheel_diameter_m[i]=.0675;c->drive_sign[i]=1;}
 c->max_wheel_rpm=40;c->max_linear_m_s=.10;c->max_angular_rad_s=.4;c->receive_watchdog_us=200000;c->max_command_age_us=150000;
}
int lab_real_command_valid(const lab_real_command *c,uint64_t now){
 return c&&lab_real_context_valid(&c->context)&&isfinite(c->linear_m_s)&&isfinite(c->angular_rad_s)&&c->enabled<=1&&
 c->generated_us<=now&&c->expires_us>now&&c->expires_us>c->generated_us&&c->expires_us-c->generated_us<=500000&&
 (!c->enabled||c->control_session)&&fabs(c->linear_m_s)<=1&&fabs(c->angular_rad_s)<=3;
}
const char *lab_real_source_text(uint32_t s){switch(s){case LAB_REAL_HARDWARE:return "REAL_HARDWARE";case LAB_REAL_RECORDING:return "REAL_RECORDING";case LAB_REAL_TEST_FIXTURE:return "TEST_FIXTURE";default:return "INVALID_SOURCE";}}
const char *lab_real_mode_text(uint32_t m){switch(m){case LAB_REAL_READONLY:return "READONLY";case LAB_REAL_MAPPING:return "MAPPING";case LAB_REAL_LOCALIZATION:return "LOCALIZATION";case LAB_REAL_SHADOW:return "SHADOW";case LAB_REAL_NAVIGATION:return "NAVIGATION";default:return "INVALID_MODE";}}
