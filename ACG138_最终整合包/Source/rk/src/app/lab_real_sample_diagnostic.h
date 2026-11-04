#ifndef ACG_REAL_SAMPLE_DIAGNOSTIC_H
#define ACG_REAL_SAMPLE_DIAGNOSTIC_H
/* 仅作诊断判定，实际准入仍由lab_live_sample_valid及运行对象决定，状态和滤波策略不变。
 */
#include "lab_live.h"
#include <math.h>
static inline const char *lab_real_sample_invalid_reason(const lab_live_sample *e){
 if(!e)return "NULL_SAMPLE";
 if(e->reserved)return "SAMPLE_RESERVED_NONZERO";
 const lab_real_meta*m=&e->meta;
 if(!lab_real_context_valid(&m->context))return "META_CONTEXT_INVALID";
 if(m->provider<1||m->provider>4)return "META_PROVIDER_INVALID";
 if(!m->stream_id)return "META_STREAM_ID_ZERO";
 if(!m->stream_session)return "META_STREAM_SESSION_ZERO";
 if(m->reserved)return "META_RESERVED_NONZERO";
 if(m->flags&~15u)return "META_UNKNOWN_FLAGS";
 if(!m->parameter_version)return "META_PARAMETER_VERSION_ZERO";
 if(m->received_us<m->time_us)return "META_SOURCE_AFTER_RECEIPT";
 if((m->flags&LAB_REAL_TIME_ESTIMATED)&&!m->time_uncertainty_us)return "META_ESTIMATED_ZERO_UNCERTAINTY";
 if(m->context.clock==LAB_REAL_UNMAPPED_CLOCK&&(m->flags&LAB_REAL_TIME_VALID))return "META_UNMAPPED_CLOCK_MARKED_VALID";
 switch(e->type){
 case LAB_EVENT_SCAN:{const struct scan_frame*s=&e->data.scan;
  if(!s->count||s->count>SCAN_MAX_POINTS)return "SCAN_COUNT_OUT_OF_RANGE";
  if(s->source<1||s->source>2)return "SCAN_SOURCE_INVALID";
  if(!scan_geometry_valid(s))return "SCAN_GEOMETRY_INVALID";
  if(s->flags&~1u)return "SCAN_UNKNOWN_FLAGS";
  if(s->session!=m->stream_session)return "SCAN_SESSION_META_MISMATCH";
  if(s->sequence!=m->sequence)return "SCAN_SEQUENCE_META_MISMATCH";
  break;}
 case LAB_EVENT_POSE:{const lab_pose*p=&e->data.odometry;
  if(m->provider!=LAB_REAL_ODOMETRY)return "POSE_PROVIDER_NOT_ODOMETRY";
  if(!isfinite(p->x_m)||!isfinite(p->y_m))return "POSE_POSITION_NONFINITE";
  if(!isfinite(p->yaw_rad))return "POSE_YAW_NONFINITE";
  if(!isfinite(p->distance_m)||p->distance_m<0)return "POSE_DISTANCE_INVALID";
  if(!isfinite(p->uncertainty_m)||p->uncertainty_m<0)return "POSE_UNCERTAINTY_INVALID";
  if(p->calibrated>1)return "POSE_CALIBRATED_INVALID";
  if(p->quality&~63u)return "POSE_UNKNOWN_QUALITY";
  break;}
 case LAB_EVENT_ENCODER:
  if(e->data.encoder.valid_mask&~15u)return "ENCODER_MASK_INVALID";
  if(!e->data.encoder.counts_per_revolution||e->data.encoder.counts_per_revolution>1000000)return "ENCODER_CPR_INVALID";
  break;
 case LAB_EVENT_IMU:
  if(!isfinite(e->data.imu.yaw_rad))return "IMU_YAW_NONFINITE";
  if(!isfinite(e->data.imu.yaw_rate_rad_s))return "IMU_RATE_NONFINITE";
  if(e->data.imu.valid_flags&~3u)return "IMU_UNKNOWN_VALID_FLAGS";
  break;
 case LAB_EVENT_RESET:
  if(e->data.reset.reason<1||e->data.reset.reason>5)return "RESET_REASON_INVALID";
  if(!e->data.reset.next_origin_epoch)return "RESET_EPOCH_ZERO";
  break;
 case LAB_EVENT_GAP:
  if(!e->data.gap.dropped_events)return "GAP_DROPPED_ZERO";
  if(!e->data.gap.reason)return "GAP_REASON_ZERO";
  if(e->data.gap.affected_event_type>7)return "GAP_EVENT_TYPE_INVALID";
  break;
 default:return "EVENT_TYPE_UNSUPPORTED";
 }
 return "SAMPLE_INVALID_UNCLASSIFIED";
}
#endif
