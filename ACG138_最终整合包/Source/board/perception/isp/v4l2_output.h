#ifndef CAMERA_V4L2_OUTPUT_H
#define CAMERA_V4L2_OUTPUT_H
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include "pixels.h"
#include "../real_frames.h"
struct camera_output { int fd; unsigned w,h,x; uint8_t *pixels; size_t size; };
static int camera_output_open(struct camera_output *o,const char *path,unsigned w,unsigned h,unsigned x) {
    o->fd=open(path,O_WRONLY|O_CLOEXEC);
    if(o->fd<0){perror(path);return -1;}
    struct v4l2_capability cap={0};
    if(ioctl(o->fd,VIDIOC_QUERYCAP,&cap) || strncmp((char*)cap.card,"FPGA_",5)) {
        fprintf(stderr,"refusing non-FPGA output %s\n",path);return -1;
    }
    struct v4l2_format f={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT};
    f.fmt.pix.width=w;f.fmt.pix.height=h;f.fmt.pix.pixelformat=V4L2_PIX_FMT_NV12;
    f.fmt.pix.field=V4L2_FIELD_NONE;f.fmt.pix.bytesperline=w;f.fmt.pix.sizeimage=w*h*3/2;
    f.fmt.pix.colorspace=V4L2_COLORSPACE_SMPTE170M;
    f.fmt.pix.ycbcr_enc=V4L2_YCBCR_ENC_601;f.fmt.pix.quantization=V4L2_QUANTIZATION_LIM_RANGE;
    if(ioctl(o->fd,VIDIOC_S_FMT,&f) || f.fmt.pix.width!=w || f.fmt.pix.height!=h ||
        f.fmt.pix.pixelformat!=V4L2_PIX_FMT_NV12 || f.fmt.pix.bytesperline!=w || f.fmt.pix.sizeimage!=w*h*3/2) {
        fprintf(stderr,"V4L2 output format mismatch %s\n",path);return -1;
    }
    struct v4l2_streamparm p={.type=V4L2_BUF_TYPE_VIDEO_OUTPUT};
    p.parm.output.timeperframe.numerator=1;p.parm.output.timeperframe.denominator=30;
    if(ioctl(o->fd,VIDIOC_S_PARM,&p)){perror("V4L2 output fps");return -1;}
    if(real_frames_configure(o->fd))return -1;
    o->w=w;o->h=h;o->x=x;o->size=w*h*3/2;o->pixels=malloc(o->size);
    if(!o->pixels)return -1;
    fprintf(stderr,"V4L2 %s NV12 %ux%u crop_x=%u\n",path,w,h,x);
    return 0;
}
static int camera_output_frame(struct camera_output *o,const uint8_t *nv12,unsigned stride) {
    const uint8_t *data=nv12;
    if(o->w!=stride) { nv12_crop(nv12,stride,o->h,o->x,o->w,o->pixels);data=o->pixels; }
    ssize_t n;do n=write(o->fd,data,o->size);while(n<0 && errno==EINTR);
    if(n!=(ssize_t)o->size){perror("V4L2 frame write");return -1;}
    return 0;
}
static void camera_output_close(struct camera_output *o) {
    if(o->fd>=0)close(o->fd);
    free(o->pixels);
}
#endif
