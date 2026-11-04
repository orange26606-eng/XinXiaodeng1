#include "live_real_convert.h"
#include <string.h>
static lab_status convert(const li_meta *m,const lab_real_context *c,uint64_t parameters,
                          uint32_t provider,uint32_t stream,lab_real_meta *out){
 uint32_t source=m->context.run_kind==LI_REAL_READONLY?LAB_REAL_HARDWARE:LAB_REAL_TEST_FIXTURE;
 if((m->context.run_kind!=LI_REAL_READONLY&&m->context.run_kind!=LI_OFFLINE_TEST)||
  ((m->flags&LI_SYNTHETIC_TEST)!=0)!=(m->context.run_kind==LI_OFFLINE_TEST)||
  !lab_real_context_valid(c)||c->clock!=LAB_REAL_MONOTONIC||c->source!=source||
  c->timeline_id!=m->context.timeline_id||c->origin_epoch!=m->context.origin_epoch||!parameters)return LAB_ERR_CONTEXT;
 *out=(lab_real_meta){.context=*c,.provider=provider,.stream_id=stream,.stream_session=m->producer_session,
  .sequence=m->sequence,.time_us=m->time_us,.received_us=m->received_us,
  .time_uncertainty_us=m->time_error_us,.parameter_version=parameters};
 if(m->flags&LI_HOST_TIME)out->flags|=LAB_REAL_TIME_VALID;
 if(m->flags&LI_ESTIMATED_MEASUREMENT_TIME)out->flags|=LAB_REAL_TIME_ESTIMATED;
 return LAB_OK;
}
lab_status li_real_pose_sample(const li_pose_sample *p,const lab_real_context *ctx,uint64_t version,uint32_t type,lab_live_sample *out){
 if(out)memset(out,0,sizeof(*out));if(!p||!ctx||!out||(type!=LAB_EVENT_POSE&&type!=LAB_EVENT_ENCODER))return LAB_ERR_ARGUMENT;
 lab_status r=convert(&p->meta,ctx,version,LAB_REAL_ODOMETRY,type==LAB_EVENT_POSE?2:3,&out->meta);if(r!=LAB_OK)return r;
 out->type=type;
 if(type==LAB_EVENT_POSE){out->data.odometry=p->pose;
  if((p->pose.quality&LAB_POSE_VALID)&&!(p->pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP)))out->meta.flags|=LAB_REAL_DATA_VALID;
  if(p->pose.calibrated)out->meta.flags|=LAB_REAL_CALIBRATED;
 }else{memcpy(out->data.encoder.counts,p->counts,sizeof(p->counts));out->data.encoder.valid_mask=p->counts_valid_mask;
  out->data.encoder.counts_per_revolution=p->counts_per_revolution;if(p->counts_valid_mask==15)out->meta.flags|=LAB_REAL_DATA_VALID;}
 if(!lab_live_sample_valid(out)){memset(out,0,sizeof(*out));return LAB_ERR_FORMAT;}return LAB_OK;
}
lab_status li_real_scan_sample(const li_scan_view *s,const lab_real_context *ctx,uint64_t version,lab_live_sample *out){
 if(out)memset(out,0,sizeof(*out));if(!s||!ctx||!out||!s->frame)return LAB_ERR_ARGUMENT;
 lab_status r=convert(&s->meta,ctx,version,s->frame->source==1?LAB_REAL_USB:LAB_REAL_FPGA,1,&out->meta);if(r!=LAB_OK)return r;
 out->type=LAB_EVENT_SCAN;out->data.scan=*s->frame;
 if(s->frame->count>SCAN_MAX_POINTS){memset(out,0,sizeof(*out));return LAB_ERR_FORMAT;}
 for(unsigned i=0;i<s->frame->count;++i)if(s->frame->points[i].quality&&s->frame->points[i].distance_q2){out->meta.flags|=LAB_REAL_DATA_VALID;break;}
 if(!lab_live_sample_valid(out)){memset(out,0,sizeof(*out));return LAB_ERR_FORMAT;}return LAB_OK;
}
