#include "lab_real_time_source.h"
#include <math.h>
#include <string.h>
static uint32_t le32(const uint8_t *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
void lab_real_timed_odom_init(lab_real_timed_odom_state *s,const lab_real_time_profile *p){if(s){memset(s,0,sizeof(*s));lab_real_clock_init(&s->clock,p);}}
lab_status lab_real_timed_odom_convert(lab_real_timed_odom_state *s,const lab_real_odom_time *c,const lab_real_context *ctx,uint64_t now,lab_live_sample out[3],uint32_t *count){
 if(count)*count=0;if(!s||!c||!count||!out||!lab_real_context_valid(ctx)||ctx->clock!=LAB_REAL_MONOTONIC||!lab_real_od01_valid(c->raw_od01,64))return LAB_ERR_ARGUMENT;
 if(c->parameter_version!=s->clock.profile.parameter_version||c->counts_per_revolution!=s->clock.profile.counts_per_revolution)return LAB_ERR_VERSION;
 if(s->clock.profile.odometry_config_crc32&&s->clock.profile.odometry_config_crc32!=c->config_crc32)return LAB_ERR_VERSION;
 if(c->received_us>c->published_us||c->published_us>now)return LAB_ERR_TIME;
 if(s->producer_id&&(s->producer_id!=c->producer_id||s->session!=c->session||s->config_crc!=c->config_crc32))return LAB_ERR_CONTEXT;
 if(s->producer_id){uint32_t delta=c->accepted_sequence-s->accepted;if(!delta)return LAB_AGAIN;if(delta>=UINT32_C(0x80000000))return LAB_ERR_CONTEXT;}
 s->producer_id=c->producer_id;s->session=c->session;s->config_crc=c->config_crc32;s->accepted=c->accepted_sequence;
 uint64_t t,error;lab_status r=lab_real_clock_observe(&s->clock,c->raw_od01,c->received_us,&t,&error);if(r!=LAB_OK)return r;
 memset(out,0,3*sizeof(*out));lab_real_meta m={.context=*ctx,.provider=LAB_REAL_ODOMETRY,.stream_id=2,.stream_session=c->session,.sequence=c->accepted_sequence,.flags=LAB_REAL_TIME_VALID|LAB_REAL_TIME_ESTIMATED,.time_us=t,.received_us=now,.time_uncertainty_us=error,.parameter_version=c->parameter_version};
 out[0].meta=m;out[0].type=LAB_EVENT_POSE;out[0].data.odometry=c->pose;if((c->pose.quality&LAB_POSE_VALID)&&!(c->pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP)))out[0].meta.flags|=LAB_REAL_DATA_VALID;if(c->pose.calibrated)out[0].meta.flags|=LAB_REAL_CALIBRATED;
 out[1].meta=m;out[1].meta.stream_id=3;out[1].type=LAB_EVENT_ENCODER;for(unsigned i=0;i<4;++i)out[1].data.encoder.counts[i]=(int32_t)le32(c->raw_od01+20+4*i);out[1].data.encoder.valid_mask=c->raw_od01[5]?0:c->raw_od01[6];out[1].data.encoder.counts_per_revolution=c->counts_per_revolution;if(out[1].data.encoder.valid_mask==15)out[1].meta.flags|=LAB_REAL_DATA_VALID;*count=2;
 uint32_t seq=le32(c->raw_od01+44),age=le32(c->raw_od01+40),bits=le32(c->raw_od01+36);float yaw;memcpy(&yaw,&bits,4);
 if(!s->have_imu||((uint32_t)(seq-s->imu_sequence)<UINT32_C(0x80000000)&&seq!=s->imu_sequence)){
  s->have_imu=1;s->imu_sequence=seq;out[2].meta=m;out[2].meta.provider=LAB_REAL_FPGA;out[2].meta.stream_id=4;out[2].meta.sequence=seq;out[2].type=LAB_EVENT_IMU;
  /* 数据年龄以 FPGA 上报时间为基准，分辨率为毫秒。OD01 不含角速度，valid_flags 中不设置 RATE_VALID。 */
  uint64_t age_us=(uint64_t)age*1000;int valid=!c->raw_od01[7]&&age<=150&&isfinite(yaw)&&fabs(yaw)<1e6&&t>=age_us;
  out[2].data.imu.yaw_rad=isfinite(yaw)?yaw:0;out[2].data.imu.valid_flags=valid?LAB_IMU_YAW_VALID:0;
  if(valid){out[2].meta.time_us=t-age_us;out[2].meta.flags|=LAB_REAL_DATA_VALID;if(error!=UINT64_MAX)out[2].meta.time_uncertainty_us=error+1000;}
  if(valid&&out[2].meta.time_us>=s->last_imu_time_us){s->last_imu_time_us=out[2].meta.time_us;*count=3;}
 }
 for(uint32_t i=0;i<*count;++i)if(!lab_live_sample_valid(out+i)){*count=0;return LAB_ERR_FORMAT;}return LAB_OK;
}
