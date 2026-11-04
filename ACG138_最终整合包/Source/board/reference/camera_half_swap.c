/* 1.1.0在YOLO前交换NV12两眼图像；1.1.1核对输出颜色元数据和驱动默认值。读取ISP的video60，将两个960像素宽的图像交换后写入video62。YOLO接收修正顺序的图像，固定界面布局保持不变，设备打开与关闭沿用已验证的V4L2 MMAP实现。



 */
#define _GNU_SOURCE
#include "camera_swap_nv12.h"
#include "dashboard_video.h"
#include "../perception/real_frames.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
static volatile sig_atomic_t stopped;
static void stop(int sig) { stopped=sig; }
static uint64_t now_ms(void) {
    struct timespec t;
    if(clock_gettime(CLOCK_MONOTONIC,&t)) return 0;
    return (uint64_t)t.tv_sec*1000+(uint64_t)t.tv_nsec/1000000;
}
static int call_ioctl(int fd,unsigned long op,void *arg) {
    int rc; do { rc=ioctl(fd,op,arg); } while(rc<0 && errno==EINTR && !stopped);
    return rc;
}
static int wait_capture(const char *path) {
    uint64_t end=now_ms()+10000;
    while(!stopped && now_ms()<end) {
        int fd=open(path,O_RDWR|O_NONBLOCK|O_CLOEXEC);
        if(fd>=0) {
            struct v4l2_capability cap={0};
            int rc=call_ioctl(fd,VIDIOC_QUERYCAP,&cap); close(fd);
            uint32_t caps=(cap.capabilities&V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
            if(!rc && (caps&V4L2_CAP_VIDEO_CAPTURE) && (caps&V4L2_CAP_STREAMING)) return 0;
        }
        (void)poll(NULL,0,100);
    }
    fprintf(stderr,"Camera swap: no ISP capture capability within 10 seconds on %s\n",path);
    return -1;
}
int main(int argc,char **argv) {
    if(argc!=4 || !strcmp(argv[1],argv[2]) || strncmp(argv[1],"/dev/video",10) ||
       strncmp(argv[2],"/dev/video",10) || strlen(argv[3])>3900) {
        fprintf(stderr,"Usage: %s /dev/video60 /dev/video62 READY_FILE\n",argv[0]); return 2;
    }
    struct stat st;
    if(!lstat(argv[3],&st) || errno!=ENOENT) {
        fprintf(stderr,"Camera swap readiness path must not already exist\n"); return 2;
    }
    struct sigaction action={0}; action.sa_handler=stop; sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,NULL);sigaction(SIGTERM,&action,NULL);sigaction(SIGHUP,&action,NULL);
    struct dashboard_video input={.fd=-1};
    int output=-1,ready_owned=0,result=1;
    uint8_t *pixels=NULL;
    const size_t bytes=(size_t)1920*1080*3/2;
    char temporary[4096];
    snprintf(temporary,sizeof(temporary),"%s.%ld.tmp",argv[3],(long)getpid());
    int temporary_owned=0;
    if(wait_capture(argv[1]) || stopped) goto done;
    if(dashboard_video_open(&input,argv[1],1,1)) goto done;
    if(input.layout.fourcc!=DASHBOARD_NV12 || input.layout.width!=1920 || input.layout.height!=1080) {
        fprintf(stderr,"Camera swap expects ISP NV12 1920x1080; actual %.4s %ux%u\n",
                input.format_name,input.layout.width,input.layout.height); goto done;
    }
    output=open(argv[2],O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if(output<0) { perror("Camera swap output open"); goto done; }
    struct stat in_stat,out_stat;
    if(fstat(input.fd,&in_stat) || fstat(output,&out_stat) || !S_ISCHR(out_stat.st_mode) ||
       in_stat.st_rdev==out_stat.st_rdev) {
        fprintf(stderr,"Camera swap input/output must be distinct video devices\n"); goto done;
    }
    struct v4l2_capability cap={0};
    if(call_ioctl(output,VIDIOC_QUERYCAP,&cap)) { perror("Camera swap output QUERYCAP"); goto done; }
    uint32_t caps=(cap.capabilities&V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    if(!(caps&V4L2_CAP_VIDEO_OUTPUT) || !(caps&V4L2_CAP_READWRITE)) {
        fprintf(stderr,"Camera swap output is busy or does not support video write\n"); goto done;
    }
    struct v4l2_format fmt={0}; fmt.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width=1920;fmt.fmt.pix.height=1080;fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_NV12;
    fmt.fmt.pix.field=V4L2_FIELD_NONE;fmt.fmt.pix.bytesperline=1920;fmt.fmt.pix.sizeimage=(unsigned)bytes;
    fmt.fmt.pix.colorspace=input.layout.matrix==DASHBOARD_BT709 ? V4L2_COLORSPACE_REC709 : V4L2_COLORSPACE_SMPTE170M;
    fmt.fmt.pix.ycbcr_enc=input.layout.matrix==DASHBOARD_BT709 ? V4L2_YCBCR_ENC_709 : V4L2_YCBCR_ENC_601;
    fmt.fmt.pix.quantization=input.layout.range==DASHBOARD_FULL ? V4L2_QUANTIZATION_FULL_RANGE : V4L2_QUANTIZATION_LIM_RANGE;
    fmt.fmt.pix.xfer_func=V4L2_XFER_FUNC_709;
    fmt.fmt.pix.priv=V4L2_PIX_FMT_PRIV_MAGIC;
    if(call_ioctl(output,VIDIOC_S_FMT,&fmt)) { perror("Camera swap output S_FMT (stop other video62 users)"); goto done; }
    if(call_ioctl(output,VIDIOC_G_FMT,&fmt)) { perror("Camera swap output G_FMT"); goto done; }
    if(fmt.fmt.pix.width!=1920 || fmt.fmt.pix.height!=1080 || fmt.fmt.pix.pixelformat!=V4L2_PIX_FMT_NV12 ||
       fmt.fmt.pix.bytesperline!=1920 || fmt.fmt.pix.sizeimage!=bytes) {
        fprintf(stderr,"Camera swap output refused the required NV12 layout\n"); goto done;
    }
    /* S_FMT尺寸不变时，回环设备可能保留旧颜色信息；像素保持不变，但拒绝明确不兼容的元数据。
 */
    unsigned output_range=fmt.fmt.pix.quantization ? fmt.fmt.pix.quantization :
        (fmt.fmt.pix.colorspace==V4L2_COLORSPACE_JPEG ? V4L2_QUANTIZATION_FULL_RANGE : V4L2_QUANTIZATION_LIM_RANGE);
    unsigned output_matrix=fmt.fmt.pix.ycbcr_enc ? fmt.fmt.pix.ycbcr_enc : V4L2_MAP_YCBCR_ENC_DEFAULT(fmt.fmt.pix.colorspace);
    if(output_range!=(input.layout.range==DASHBOARD_FULL ? V4L2_QUANTIZATION_FULL_RANGE : V4L2_QUANTIZATION_LIM_RANGE) ||
       output_matrix!=(input.layout.matrix==DASHBOARD_BT709 ? V4L2_YCBCR_ENC_709 : V4L2_YCBCR_ENC_601)) {
        fprintf(stderr,"Camera swap output color metadata disagrees with ISP input\n"); goto done;
    }
    if(real_frames_configure(output))goto done;
    struct v4l2_streamparm cadence={0};cadence.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;
    cadence.parm.output.timeperframe.numerator=1;cadence.parm.output.timeperframe.denominator=30;
    if(call_ioctl(output,VIDIOC_S_PARM,&cadence))goto done;
    pixels=malloc(bytes); if(!pixels) { perror("Camera swap allocation"); goto done; }
    uint64_t last=now_ms(),frames=0;
    while(!stopped) {
        struct pollfd p={.fd=input.fd,.events=POLLIN};
        int rc=poll(&p,1,200);
        if(rc<0) { if(errno==EINTR)continue; perror("Camera swap poll"); goto done; }
        if(p.revents&(POLLERR|POLLHUP|POLLNVAL)) { fprintf(stderr,"Camera swap input stopped\n"); goto done; }
        if(now_ms()-last>=10000) { fprintf(stderr,"Camera swap: no new ISP frame for 10 seconds\n"); goto done; }
        if(!(p.revents&POLLIN))continue;
        struct v4l2_buffer b={0}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
        if(call_ioctl(input.fd,VIDIOC_DQBUF,&b)) {
            if(errno==EAGAIN || (errno==EINTR && stopped))continue;
            perror("Camera swap DQBUF"); goto done;
        }
        if(b.index>=input.buffer_count || b.bytesused>input.map_lengths[b.index]) {
            fprintf(stderr,"Camera swap invalid capture buffer\n"); goto done;
        }
        if(b.flags&V4L2_BUF_FLAG_ERROR) {
            if(call_ioctl(input.fd,VIDIOC_QBUF,&b))goto done;
            continue;
        }
        if(camera_swap_nv12(pixels,bytes,input.maps[b.index],b.bytesused,1920,1080,input.layout.stride)) {
            fprintf(stderr,"Camera swap rejected a short/invalid NV12 frame\n"); goto done;
        }
        if(call_ioctl(input.fd,VIDIOC_QBUF,&b)) { perror("Camera swap QBUF"); goto done; }
        uint64_t deadline=now_ms()+2000;
        ssize_t written=-1;
        while(!stopped) {
            written=write(output,pixels,bytes);
            if(written>=0)break;
            if(errno==EINTR)continue;
            if(errno!=EAGAIN || now_ms()>=deadline)break;
            (void)poll(NULL,0,10);
        }
        if(stopped)break;
        if(written!=(ssize_t)bytes) { fprintf(stderr,"Camera swap incomplete output write: %zd/%zu, errno=%d\n",written,bytes,errno); goto done; }
        last=now_ms();++frames;
        if(!ready_owned) {
            int fd=open(temporary,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
            if(fd<0) { perror("Camera swap readiness create"); goto done; }
            temporary_owned=1;
            int failed=dprintf(fd,"%s\n",argv[2])<0;
            if(close(fd))failed=1;
            if(failed || rename(temporary,argv[3])) { perror("Camera swap readiness publish"); goto done; }
            temporary_owned=0;ready_owned=1;
            fprintf(stderr,"CAMERA_SWAP_READY input=%s output=%s NV12=1920x1080 order=right,left; before YOLO\n",argv[1],argv[2]);
        }
        if(frames%300==0)fprintf(stderr,"CAMERA_SWAP frames=%" PRIu64 "\n",frames);
    }
    result=0;
done:
    if(ready_owned)unlink(argv[3]);
    if(temporary_owned)unlink(temporary);
    if(output>=0)close(output);
    dashboard_video_close(&input);free(pixels);
    return stopped ? 128+stopped : result;
}
