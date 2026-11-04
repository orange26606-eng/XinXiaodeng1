#ifndef ACG_H264_ENCODER_H
#define ACG_H264_ENCODER_H
#include <stddef.h>
#include <stdint.h>
enum { VIDEO_WIDTH=1280, VIDEO_HEIGHT=720, VIDEO_FPS=30, VIDEO_MAX_PACKET=2*1024*1024 };
struct h264_encoder;
int h264_encoder_open(struct h264_encoder **out,const char *library);
void h264_encoder_close(struct h264_encoder *encoder);
unsigned char *h264_encoder_pixels(struct h264_encoder *encoder);
int h264_encoder_encode(struct h264_encoder *encoder,uint64_t pts_ms,int force_idr,
                        unsigned char **data,size_t *length,unsigned *flags);
/* 有限范围NV12，不修改DSI显示所用的源图。 */
int video_scale_nv12_720(unsigned char *dst,const unsigned char *src,size_t bytes,
                         unsigned width,unsigned height,unsigned stride);
unsigned h264_annexb_flags(const unsigned char *data,size_t length);
#endif
