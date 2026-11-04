#include "h264_encoder.h"
#include "mpp/rk_mpi.h"
#include "mpp/rk_venc_cfg.h"
#include "mpp/mpp_buffer.h"
#include "mpp/mpp_frame.h"
#include "mpp/mpp_packet.h"
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
struct h264_encoder {
 void *library;MppCtx ctx;MppApi *api;MppEncCfg cfg;MppBufferGroup group;MppBuffer input;
 MPP_RET (*create)(MppCtx*,MppApi**),(*init)(MppCtx,MppCtxType,MppCodingType),(*destroy)(MppCtx);
 MPP_RET (*cfg_init)(MppEncCfg*),(*cfg_deinit)(MppEncCfg),(*cfg_set)(MppEncCfg,const char*,RK_S32);
 MPP_RET (*group_get)(MppBufferGroup*,MppBufferType,MppBufferMode,const char*,const char*),(*group_put)(MppBufferGroup);
 MPP_RET (*buffer_get)(MppBufferGroup,MppBuffer*,size_t,const char*,const char*),(*buffer_put)(MppBuffer,const char*);
 void *(*buffer_ptr)(MppBuffer,const char*);
 MPP_RET (*frame_init)(MppFrame*),(*frame_deinit)(MppFrame*),(*packet_deinit)(MppPacket*);
 void (*frame_width)(MppFrame,RK_U32),(*frame_height)(MppFrame,RK_U32),(*frame_hstride)(MppFrame,RK_U32),(*frame_vstride)(MppFrame,RK_U32);
 void (*frame_fmt)(MppFrame,MppFrameFormat),(*frame_buffer)(MppFrame,MppBuffer),(*frame_pts)(MppFrame,RK_S64);
 void *(*packet_pos)(const MppPacket);size_t (*packet_length)(const MppPacket);
 RK_U32 (*packet_partition)(const MppPacket),(*packet_eoi)(const MppPacket);
 unsigned char *output;
};
unsigned h264_annexb_flags(const unsigned char *p,size_t n){
 unsigned types=0;
 for(size_t i=0;i+3<n;i++){
  size_t k=0;if(!p[i]&&!p[i+1]&&p[i+2]==1)k=i+3;
  else if(i+4<n&&!p[i]&&!p[i+1]&&!p[i+2]&&p[i+3]==1)k=i+4;
  if(k){unsigned type=p[k]&31;if(type==5)types|=2;else if(type==7)types|=4;else if(type==8)types|=8;i=k;}
 }
 return (types&2)|((types&12)==12?1:0);
}
int video_scale_nv12_720(unsigned char *dst,const unsigned char *src,size_t n,unsigned w,unsigned h,unsigned stride){
 if(!dst||!src||w!=1920||h!=1080||stride<w||stride>8192||n<(size_t)stride*h*3/2)return -1;
 /* 精确的3比2下采样几何，最近邻采样保留双眼分界。 */
 for(unsigned y=0;y<720;y++){
  const unsigned char *row=src+(size_t)(y*3/2)*stride;unsigned char *out=dst+y*1280;
  for(unsigned x=0;x<1280;x+=2){out[x]=row[x*3/2];out[x+1]=row[(x+1)*3/2];}
 }
 const unsigned char *uv=src+(size_t)stride*h;
 for(unsigned y=0;y<360;y++){
  const unsigned char *row=uv+(size_t)(y*3/2)*stride;unsigned char *out=dst+1280*720+y*1280;
  for(unsigned x=0;x<640;x++){unsigned s=(x*3/2)*2;out[x*2]=row[s];out[x*2+1]=row[s+1];}
 }
 return 0;
}
void h264_encoder_close(struct h264_encoder *e){
 if(!e)return;
 if(e->ctx&&e->destroy)e->destroy(e->ctx);
 if(e->cfg&&e->cfg_deinit)e->cfg_deinit(e->cfg);
 if(e->input&&e->buffer_put)e->buffer_put(e->input,"acg138-video");
 if(e->group&&e->group_put)e->group_put(e->group);
 free(e->output);if(e->library)dlclose(e->library);free(e);
}
int h264_encoder_open(struct h264_encoder **out,const char *library){
 *out=NULL;struct h264_encoder *e=calloc(1,sizeof(*e));if(!e)return -1;
 e->library=dlopen(library,RTLD_NOW|RTLD_LOCAL);
 if(!e->library){fprintf(stderr,"H264 MPP load failed: %s\n",dlerror());goto fail;}
#define LOAD(member,name) do{void *p=dlsym(e->library,name);if(!p){fprintf(stderr,"Missing MPP symbol: %s\n",name);goto fail;}memcpy(&e->member,&p,sizeof(p));}while(0)
 LOAD(create,"mpp_create");LOAD(init,"mpp_init");LOAD(destroy,"mpp_destroy");
 LOAD(cfg_init,"mpp_enc_cfg_init");LOAD(cfg_deinit,"mpp_enc_cfg_deinit");LOAD(cfg_set,"mpp_enc_cfg_set_s32");
 LOAD(group_get,"mpp_buffer_group_get");LOAD(group_put,"mpp_buffer_group_put");
 LOAD(buffer_get,"mpp_buffer_get_with_tag");LOAD(buffer_put,"mpp_buffer_put_with_caller");LOAD(buffer_ptr,"mpp_buffer_get_ptr_with_caller");
 LOAD(frame_init,"mpp_frame_init");LOAD(frame_deinit,"mpp_frame_deinit");
 LOAD(frame_width,"mpp_frame_set_width");LOAD(frame_height,"mpp_frame_set_height");LOAD(frame_hstride,"mpp_frame_set_hor_stride");LOAD(frame_vstride,"mpp_frame_set_ver_stride");
 LOAD(frame_fmt,"mpp_frame_set_fmt");LOAD(frame_buffer,"mpp_frame_set_buffer");LOAD(frame_pts,"mpp_frame_set_pts");
 LOAD(packet_pos,"mpp_packet_get_pos");LOAD(packet_length,"mpp_packet_get_length");LOAD(packet_deinit,"mpp_packet_deinit");
 LOAD(packet_partition,"mpp_packet_is_partition");LOAD(packet_eoi,"mpp_packet_is_eoi");
#undef LOAD
 if(e->create(&e->ctx,&e->api)||!e->api||!e->api->control||!e->api->encode_put_frame||!e->api->encode_get_packet||e->init(e->ctx,MPP_CTX_ENC,MPP_VIDEO_CodingAVC))goto fail;
 RK_S64 timeout=200;
 if(e->api->control(e->ctx,MPP_SET_INPUT_TIMEOUT,&timeout)||e->api->control(e->ctx,MPP_SET_OUTPUT_TIMEOUT,&timeout))goto fail;
 if(e->cfg_init(&e->cfg)||e->api->control(e->ctx,MPP_ENC_GET_CFG,e->cfg))goto fail;
 const struct {const char *key;int value;} settings[]={
  {"prep:width",1280},{"prep:height",720},{"prep:hor_stride",1280},{"prep:ver_stride",720},{"prep:format",MPP_FMT_YUV420SP},
  {"rc:mode",MPP_ENC_RC_MODE_CBR},{"rc:fps_in_flex",0},{"rc:fps_in_num",30},{"rc:fps_in_denom",1},
  {"rc:fps_out_flex",0},{"rc:fps_out_num",30},{"rc:fps_out_denom",1},{"rc:gop",30},
  {"rc:bps_target",4000000},{"rc:bps_max",5000000},{"rc:bps_min",3000000},
  {"codec:type",MPP_VIDEO_CodingAVC},{"h264:profile",66},{"h264:level",31},{"h264:cabac_en",0},{"h264:trans8x8",0}};
 for(size_t i=0;i<sizeof(settings)/sizeof(settings[0]);i++)if(e->cfg_set(e->cfg,settings[i].key,settings[i].value)){fprintf(stderr,"MPP rejected setting: %s\n",settings[i].key);goto fail;}
 if(e->api->control(e->ctx,MPP_ENC_SET_CFG,e->cfg))goto fail;
 MppEncHeaderMode headers=MPP_ENC_HEADER_MODE_EACH_IDR;
 if(e->api->control(e->ctx,MPP_ENC_SET_HEADER_MODE,&headers))goto fail;
 if(e->group_get(&e->group,MPP_BUFFER_TYPE_DRM,MPP_BUFFER_INTERNAL,"acg138-video","open")||
    e->buffer_get(e->group,&e->input,1280*720*3/2,"acg138-video","open"))goto fail;
 e->output=malloc(VIDEO_MAX_PACKET);if(!e->output||!h264_encoder_pixels(e))goto fail;
 *out=e;return 0;
fail:fprintf(stderr,"H264 hardware encoder initialization failed\n");h264_encoder_close(e);return -1;
}
unsigned char *h264_encoder_pixels(struct h264_encoder *e){return e?e->buffer_ptr(e->input,"acg138-video"):NULL;}
int h264_encoder_encode(struct h264_encoder *e,uint64_t pts_ms,int force,unsigned char **data,size_t *length,unsigned *flags){
 *data=NULL;*length=0;*flags=0;
 if(force&&e->api->control(e->ctx,MPP_ENC_SET_IDR_FRAME,NULL))return -1;
 MppFrame frame=NULL;if(e->frame_init(&frame))return -1;
 e->frame_width(frame,1280);e->frame_height(frame,720);e->frame_hstride(frame,1280);e->frame_vstride(frame,720);
 e->frame_fmt(frame,MPP_FMT_YUV420SP);e->frame_buffer(frame,e->input);e->frame_pts(frame,(RK_S64)(pts_ms*1000));
 MPP_RET ret=e->api->encode_put_frame(e->ctx,frame);e->frame_deinit(&frame);if(ret)return -1;
 size_t total=0;
 for(unsigned count=0;count<64;count++){
  MppPacket packet=NULL;ret=e->api->encode_get_packet(e->ctx,&packet);
  if(ret||!packet)return -1;
  size_t n=e->packet_length(packet);void *pos=e->packet_pos(packet);
  int end=!e->packet_partition(packet)||e->packet_eoi(packet);
  if(!n||!pos||n>VIDEO_MAX_PACKET-total){e->packet_deinit(&packet);return -1;}
  memcpy(e->output+total,pos,n);total+=n;e->packet_deinit(&packet);
  if(end){*data=e->output;*length=total;*flags=h264_annexb_flags(e->output,total);return 0;}
 }
 return -1;
}
