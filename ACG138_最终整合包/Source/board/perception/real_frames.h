#ifndef ACG_REAL_FRAMES_H
#define ACG_REAL_FRAMES_H
#include <errno.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
/* 仅修改本会话的回环写入端，不重复输出旧帧或超时替代图。 */
static int real_frames_configure(int fd) {
    struct v4l2_queryctrl query;memset(&query,0,sizeof(query));query.id=V4L2_CTRL_FLAG_NEXT_CTRL;
    unsigned seen=0;
    while(ioctl(fd,VIDIOC_QUERYCTRL,&query)==0){
        if(!strcmp((char*)query.name,"sustain_framerate")||!strcmp((char*)query.name,"timeout")){
            struct v4l2_control control;memset(&control,0,sizeof(control));control.id=query.id;control.value=0;
            if(ioctl(fd,VIDIOC_S_CTRL,&control)<0)return -1;
            if(ioctl(fd,VIDIOC_G_CTRL,&control)<0||control.value!=0)return -1;
            seen|=!strcmp((char*)query.name,"timeout")?2u:1u;
        }
        query.id|=V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    if(seen!=3){fprintf(stderr,"Loopback controls unavailable; cannot verify real-frame-only mode\n");return -1;}
    return 0;
}
#endif
