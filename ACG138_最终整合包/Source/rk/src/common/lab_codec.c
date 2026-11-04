#include "lab_codec.h"
#include <string.h>
#include <stdio.h>
_Static_assert(sizeof(double)==8,"File v1 requires IEEE754 binary64");
static uint16_t r16(const uint8_t *p){return (uint16_t)((p[0]<<8)|p[1]);}
static uint32_t r32(const uint8_t *p){return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];}
static uint64_t r64(const uint8_t *p){return ((uint64_t)r32(p)<<32)|r32(p+4);}
static void w16(uint8_t *p,uint16_t n){p[0]=(uint8_t)(n>>8);p[1]=(uint8_t)n;}
static void w32(uint8_t *p,uint32_t n){p[0]=(uint8_t)(n>>24);p[1]=(uint8_t)(n>>16);p[2]=(uint8_t)(n>>8);p[3]=(uint8_t)n;}
static void w64(uint8_t *p,uint64_t n){w32(p,(uint32_t)(n>>32));w32(p+4,(uint32_t)n);}
static double rd(const uint8_t *p){uint64_t u=r64(p);double d;memcpy(&d,&u,8);return d;}
static void wd(uint8_t *p,double d){uint64_t u;memcpy(&u,&d,8);w64(p,u);}
static int zeros(const uint8_t *p,size_t n){for(size_t i=0;i<n;++i)if(p[i])return 0;return 1;}
uint32_t lab_crc32_extend(uint32_t previous,const void *data,size_t bytes){
 const uint8_t *p=data;uint32_t c=~previous;
 for(size_t i=0;i<bytes;++i){c^=p[i];for(int k=0;k<8;++k)c=(c>>1)^(0xedb88320u&(uint32_t)-(int32_t)(c&1));}return ~c;
}
uint32_t lab_crc32(const void *data,size_t bytes){return lab_crc32_extend(0,data,bytes);}
lab_status lab_file_header_encode(uint8_t out[64],const lab_file_header *h){
 if(!out||!h||h->metadata_bytes>LAB_METADATA_MAX||h->mode<1||h->mode>4||h->flags)return LAB_ERR_ARGUMENT;
 memset(out,0,64);memcpy(out,"ACGLAB1",7);w16(out+8,1);w16(out+10,64);w32(out+12,h->metadata_bytes);w32(out+16,h->metadata_crc32);
 w32(out+20,h->flags);w64(out+24,h->created_unix_us);w64(out+32,h->recording_id);w32(out+40,h->mode);w32(out+44,LAB_API_VERSION);w64(out+48,h->timeline_start_us);
 w32(out+60,lab_crc32(out,60));return LAB_OK;
}
lab_status lab_file_header_decode(lab_file_header *out,const uint8_t *p,size_t n){
 if(!out||!p)return LAB_ERR_ARGUMENT;if(n<64)return LAB_ERR_TRUNCATED;if(n!=64)return LAB_ERR_FORMAT;
 if(memcmp(p,"ACGLAB1\0",8))return LAB_ERR_FORMAT;
 if(r16(p+8)!=1||r32(p+44)!=LAB_API_VERSION)return LAB_ERR_VERSION;
 if(r16(p+10)!=64||!zeros(p+56,4)||r32(p+20))return LAB_ERR_FORMAT;
 if(r32(p+60)!=lab_crc32(p,60))return LAB_ERR_CRC;
 if(r32(p+12)>LAB_METADATA_MAX)return LAB_ERR_LIMIT;
 if(r32(p+40)<1||r32(p+40)>4)return LAB_ERR_FORMAT;
 *out=(lab_file_header){r32(p+12),r32(p+16),r32(p+20),r32(p+40),r64(p+24),r64(p+32),r64(p+48)};return LAB_OK;
}
lab_status lab_file_footer_encode(uint8_t out[32],const lab_file_footer *f){
 if(!out||!f||!(f->flags&LAB_FOOTER_CLOSED)||(f->flags&~3u))return LAB_ERR_ARGUMENT;
 memset(out,0,32);memcpy(out,"END1",4);w32(out+4,32);w64(out+8,f->event_count);w32(out+16,f->stream_crc32);w32(out+20,f->flags);w32(out+28,lab_crc32(out,28));return LAB_OK;
}
lab_status lab_file_footer_decode(lab_file_footer *out,const uint8_t *p,size_t n){
 if(!out||!p)return LAB_ERR_ARGUMENT;if(n<32)return LAB_ERR_TRUNCATED;if(n!=32||memcmp(p,"END1",4)||r32(p+4)!=32||!zeros(p+24,4))return LAB_ERR_FORMAT;
 if(r32(p+28)!=lab_crc32(p,28))return LAB_ERR_CRC;
 if(!(r32(p+20)&LAB_FOOTER_CLOSED)||(r32(p+20)&~3u))return LAB_ERR_FORMAT;
 *out=(lab_file_footer){r64(p+8),r32(p+16),r32(p+20)};return LAB_OK;
}
size_t lab_event_payload_size(const lab_event *e){
 if(!lab_event_valid(e)|| (e->type==LAB_EVENT_SCAN&&e->data.scan.geometry_kind!=SCAN_GEOMETRY_RAW))return 0;
 switch(e->type){case LAB_EVENT_SCAN:return 32+(size_t)e->data.scan.count*8;case LAB_EVENT_POSE:return 48;
 case LAB_EVENT_ENCODER:return 24;case LAB_EVENT_IMU:return 24;case LAB_EVENT_TRUTH:return 32;
 case LAB_EVENT_RESET:return 8;case LAB_EVENT_GAP:return 16;default:return 0;}
}
lab_status lab_event_encode(uint8_t h[80],uint8_t *p,size_t cap,size_t *bytes,const lab_event *e){
 if(!h||!p||!bytes||!e)return LAB_ERR_ARGUMENT;
 if(e->type==LAB_EVENT_SCAN&&e->data.scan.geometry_kind!=SCAN_GEOMETRY_RAW){
  /* 不支持记录去畸变数据的v1格式，不能静默丢弃射线原点。 */
  fprintf(stderr,"UNSUPPORTED_DESKEW_RECORD_FORMAT: ACGLAB-v1 cannot preserve actual ray origins; record raw acquisition instead\n");
  *bytes=0;return LAB_ERR_UNAVAILABLE;
 }
 size_t size=lab_event_payload_size(e);
 if(!size)return LAB_ERR_ARGUMENT;if(cap<size)return LAB_ERR_LIMIT;
 memset(h,0,80);memset(p,0,size);memcpy(h,"EV01",4);w16(h+4,80);w16(h+6,(uint16_t)e->type);w32(h+8,(uint32_t)size);
 const lab_meta *m=&e->meta;w16(h+12,(uint16_t)m->mode);w16(h+14,(uint16_t)m->provider);w16(h+16,(uint16_t)m->clock_domain);w16(h+18,(uint16_t)m->flags);
 w32(h+20,m->timeline_id);w32(h+24,m->stream_id);w32(h+28,m->stream_session);w32(h+32,m->origin_epoch);w32(h+36,m->sequence);w64(h+40,m->time_us);w64(h+48,m->received_us);
 switch(e->type){
 case LAB_EVENT_SCAN:{const struct scan_frame *f=&e->data.scan;p[0]=f->source;w16(p+2,f->flags);w32(p+4,f->session);w32(p+8,f->sequence);w32(p+12,f->period_us);w64(p+16,f->timestamp_us);w16(p+24,f->count);
  for(unsigned i=0;i<f->count;++i){uint8_t *d=p+32+i*8;w16(d,f->points[i].angle_q14);w32(d+2,f->points[i].distance_q2);d[6]=f->points[i].quality;d[7]=f->points[i].flags;}break;}
 case LAB_EVENT_POSE:{const lab_pose *v=&e->data.pose;wd(p,v->x_m);wd(p+8,v->y_m);wd(p+16,v->yaw_rad);wd(p+24,v->distance_m);w32(p+32,v->quality);w32(p+36,v->calibrated);wd(p+40,v->uncertainty_m);break;}
 case LAB_EVENT_ENCODER:for(int i=0;i<4;++i)w32(p+i*4,(uint32_t)e->data.encoder.counts[i]);w32(p+16,e->data.encoder.valid_mask);w32(p+20,e->data.encoder.counts_per_revolution);break;
 case LAB_EVENT_IMU:wd(p,e->data.imu.yaw_rad);wd(p+8,e->data.imu.yaw_rate_rad_s);w32(p+16,e->data.imu.valid_flags);break;
 case LAB_EVENT_TRUTH:wd(p,e->data.truth.x_m);wd(p+8,e->data.truth.y_m);wd(p+16,e->data.truth.yaw_rad);wd(p+24,e->data.truth.distance_m);break;
 case LAB_EVENT_RESET:w32(p,e->data.reset.reason);w32(p+4,e->data.reset.next_origin_epoch);break;
 case LAB_EVENT_GAP:w64(p,e->data.gap.dropped_events);w32(p+8,e->data.gap.reason);w32(p+12,e->data.gap.affected_event_type);break;
 }
 w32(h+56,lab_crc32(p,size));uint32_t crc=lab_crc32(h,60);crc=lab_crc32_extend(crc,h+64,16);w32(h+60,crc);*bytes=size;return LAB_OK;
}
lab_status lab_event_header_check(const uint8_t *h,size_t n,size_t *bytes){
 if(!h||!bytes)return LAB_ERR_ARGUMENT;if(n<80)return LAB_ERR_TRUNCATED;if(n!=80||memcmp(h,"EV01",4)||r16(h+4)!=80||!zeros(h+64,16))return LAB_ERR_FORMAT;
 uint32_t crc=lab_crc32(h,60);crc=lab_crc32_extend(crc,h+64,16);if(crc!=r32(h+60))return LAB_ERR_CRC;
 size_t size=r32(h+8);if(size>LAB_RECORD_PAYLOAD_MAX)return LAB_ERR_LIMIT;
 unsigned type=r16(h+6);size_t expected=0;
 switch(type){case LAB_EVENT_SCAN:if(size<40||(size-32)%8||(size-32)/8>SCAN_MAX_POINTS)return LAB_ERR_FORMAT;expected=size;break;
 case LAB_EVENT_POSE:expected=48;break;case LAB_EVENT_ENCODER:case LAB_EVENT_IMU:expected=24;break;
 case LAB_EVENT_TRUTH:expected=32;break;case LAB_EVENT_RESET:expected=8;break;case LAB_EVENT_GAP:expected=16;break;default:return LAB_ERR_FORMAT;}
 if(size!=expected)return LAB_ERR_FORMAT;
 lab_meta m={r16(h+12),r16(h+14),r16(h+16),r16(h+18),r32(h+20),r32(h+24),r32(h+28),r32(h+32),r32(h+36),r64(h+40),r64(h+48)};
 if(!lab_meta_valid(&m))return LAB_ERR_FORMAT;*bytes=size;return LAB_OK;
}
lab_status lab_event_decode(lab_event *out,const uint8_t *h,size_t hn,const uint8_t *p,size_t n){
 if(!out||!p)return LAB_ERR_ARGUMENT;size_t expected=0;lab_status s=lab_event_header_check(h,hn,&expected);if(s!=LAB_OK)return s;
 if(n<expected)return LAB_ERR_TRUNCATED;if(n!=expected)return LAB_ERR_FORMAT;if(r32(h+56)!=lab_crc32(p,n))return LAB_ERR_CRC;
 memset(out,0,sizeof(*out));out->type=r16(h+6);out->meta=(lab_meta){r16(h+12),r16(h+14),r16(h+16),r16(h+18),r32(h+20),r32(h+24),r32(h+28),r32(h+32),r32(h+36),r64(h+40),r64(h+48)};
 switch(out->type){
 case LAB_EVENT_SCAN:{struct scan_frame *f=&out->data.scan;if(p[1]||!zeros(p+26,6))return LAB_ERR_FORMAT;f->source=p[0];f->flags=r16(p+2);f->session=r32(p+4);f->sequence=r32(p+8);f->period_us=r32(p+12);f->timestamp_us=r64(p+16);f->count=r16(p+24);f->received_ms=out->meta.received_us/1000;
  if(!f->count||f->count>SCAN_MAX_POINTS||n!=32+(size_t)f->count*8)return LAB_ERR_FORMAT;
  for(unsigned i=0;i<f->count;++i){const uint8_t *d=p+32+i*8;f->points[i]=(struct scan_point){r16(d),r32(d+2),d[6],d[7]};}break;}
 case LAB_EVENT_POSE:out->data.pose=(lab_pose){rd(p),rd(p+8),rd(p+16),rd(p+24),r32(p+32),r32(p+36),rd(p+40)};break;
 case LAB_EVENT_ENCODER:for(int i=0;i<4;++i){uint32_t v=r32(p+i*4);memcpy(&out->data.encoder.counts[i],&v,4);}out->data.encoder.valid_mask=r32(p+16);out->data.encoder.counts_per_revolution=r32(p+20);break;
 case LAB_EVENT_IMU:if(!zeros(p+20,4))return LAB_ERR_FORMAT;out->data.imu=(lab_imu){rd(p),rd(p+8),r32(p+16)};break;
 case LAB_EVENT_TRUTH:out->data.truth=(lab_truth){rd(p),rd(p+8),rd(p+16),rd(p+24)};break;
 case LAB_EVENT_RESET:out->data.reset=(lab_reset){r32(p),r32(p+4)};break;
 case LAB_EVENT_GAP:out->data.gap=(lab_gap){r64(p),r32(p+8),r32(p+12)};break;
 }
 return lab_event_valid(out)?LAB_OK:LAB_ERR_FORMAT;
}
