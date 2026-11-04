#include "lab_types.h"
#include <math.h>
int lab_meta_valid(const lab_meta *m) {
 return m && m->mode>=1 && m->mode<=4 && m->provider>=1 && m->provider<=8 &&
  m->clock_domain>=1 && m->clock_domain<=4 && !(m->flags&~3u) && m->timeline_id &&
  m->stream_id && m->stream_session && m->origin_epoch &&
  !(m->clock_domain==LAB_CLOCK_REMOTE_UNMAPPED && (m->flags&LAB_TIME_VALID));
}
int lab_event_valid(const lab_event *e) {
 if(!e || !lab_meta_valid(&e->meta))return 0;
 if(e->type==LAB_EVENT_TRUTH) {
  const lab_truth *p=&e->data.truth;
  return e->meta.provider==LAB_PROVIDER_TRUTH && isfinite(p->x_m)&&isfinite(p->y_m)&&isfinite(p->yaw_rad)&&isfinite(p->distance_m)&&p->distance_m>=0;
 }
 if(e->meta.provider==LAB_PROVIDER_TRUTH)return 0;
 switch(e->type) {
 case LAB_EVENT_SCAN: {
  const struct scan_frame *f=&e->data.scan;
  return f->count>0 && f->count<=SCAN_MAX_POINTS && f->source>=1 && f->source<=2 &&
   scan_geometry_valid(f) && !(f->flags&~1u) && f->session==e->meta.stream_session && f->sequence==e->meta.sequence;
 }
 case LAB_EVENT_POSE: {
  const lab_pose *p=&e->data.pose;
  return isfinite(p->x_m)&&isfinite(p->y_m)&&isfinite(p->yaw_rad)&&isfinite(p->distance_m)&&p->distance_m>=0 &&
   isfinite(p->uncertainty_m)&&p->uncertainty_m>=0 && p->calibrated<=1 && !(p->quality&~63u) &&
   !(e->meta.mode==LAB_SIM_NOISY && e->meta.provider==LAB_PROVIDER_SIM_IDEAL);
 }
 case LAB_EVENT_ENCODER:return !(e->data.encoder.valid_mask&~15u) && e->data.encoder.counts_per_revolution>0 && e->data.encoder.counts_per_revolution<=1000000;
 case LAB_EVENT_IMU:return isfinite(e->data.imu.yaw_rad)&&isfinite(e->data.imu.yaw_rate_rad_s)&&!(e->data.imu.valid_flags&~3u);
 case LAB_EVENT_RESET:return e->data.reset.reason>=1 && e->data.reset.reason<=5 && e->data.reset.next_origin_epoch>0;
 case LAB_EVENT_GAP:return e->data.gap.dropped_events>0 && e->data.gap.reason>0 && e->data.gap.affected_event_type<=7;
 default:return 0;
 }
}
int lab_context_matches(const lab_context *c,const lab_meta *m) {
 return c && m && c->mode==m->mode && c->clock_domain==m->clock_domain &&
  c->timeline_id==m->timeline_id && c->origin_epoch==m->origin_epoch;
}
const char *lab_status_text(lab_status s) {
 switch(s){
 case LAB_OK:return "OK";case LAB_AGAIN:return "AGAIN";case LAB_EOF:return "EOF";
 case LAB_ERR_ARGUMENT:return "ARGUMENT";case LAB_ERR_IO:return "IO";case LAB_ERR_FORMAT:return "FORMAT";
 case LAB_ERR_CRC:return "CRC";case LAB_ERR_TRUNCATED:return "TRUNCATED";case LAB_ERR_VERSION:return "VERSION";
 case LAB_ERR_LIMIT:return "LIMIT";case LAB_ERR_TIME:return "TIME";case LAB_ERR_CONTEXT:return "CONTEXT";
 case LAB_ERR_READONLY:return "READONLY";case LAB_ERR_NOMEM:return "NOMEM";case LAB_ERR_UNAVAILABLE:return "UNAVAILABLE";
 default:return "UNKNOWN_STATUS";}
}
const char *lab_mode_text(uint32_t mode) {
 switch(mode){case LAB_LIVE_STATIC:return "LIVE_STATIC";case LAB_REPLAY:return "REPLAY";
 case LAB_SIM_IDEAL:return "SIM_IDEAL";case LAB_SIM_NOISY:return "SIM_NOISY";default:return "UNKNOWN_MODE";}
}
