/* 复用显示程序已有的结果与采集缓存，保持V4L2单接收者。 */
#define _GNU_SOURCE
#include "phone_export.h"
#include "phone_shared.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
static struct phone_shared *shared;
static int attempted;
static uint64_t last_frame,last_state;
static struct phone_shared *open_shared(void){
    if(attempted)return shared;
    attempted=1;const char *dir=getenv("ACG_PHONE_DIR");if(!dir||!*dir)return NULL;
    char path[PATH_MAX];if(snprintf(path,sizeof(path),"%s/frames.shm",dir)>=(int)sizeof(path))return NULL;
    if(mkdir(dir,0700)&&errno!=EEXIST){perror("phone export mkdir");return NULL;}
    int fd=open(path,O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd<0){perror("phone export open");return NULL;}
    if(ftruncate(fd,sizeof(*shared))){perror("phone export size");close(fd);return NULL;}
    shared=mmap(NULL,sizeof(*shared),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);close(fd);
    if(shared==MAP_FAILED){shared=NULL;perror("phone export mmap");return NULL;}
    shared->magic=0;memset(shared,0,offsetof(struct phone_shared,pixels));
    shared->version=1;__atomic_store_n(&shared->magic,PHONE_MAGIC,__ATOMIC_RELEASE);
    fprintf(stderr,"Phone export enabled: %s (existing video and telemetry only)\n",path);return shared;
}
void phone_export_raw(const struct dashboard_video_layout *layout,const void *data,size_t bytes,uint64_t now){
    struct phone_shared *s=open_shared();if(!s||!layout||!data||bytes>PHONE_MAX_FRAME||now-last_frame<33)return;
    phone_begin_write(&s->frame_sequence);
    s->frame=(struct phone_frame_meta){.stamp_ms=now,.width=layout->width,.height=layout->height,
        .stride=layout->stride,.fourcc=layout->fourcc,.matrix=layout->matrix,.range=layout->range,.bytes=(uint32_t)bytes};
    memcpy(s->pixels,data,bytes);phone_end_write(&s->frame_sequence);last_frame=now;
}
void phone_export_state(const struct imu_model *imu,const struct robot_model *robot,uint64_t now){
    struct phone_shared *s=open_shared();if(!s||!imu||!robot||now-last_state<80)return;
    struct phone_sensor_state state={.stamp_ms=now,.imu_ms=imu->last_good_ms,.robot_ms=robot->last_ms,
        .roll=imu->degrees[0],.pitch=imu->degrees[1],.yaw=imu->degrees[2],.imu_state=imu_model_state(imu,now),
        .robot_seen=robot->seen,.robot_bad=robot->bad_packet,.range_valid=robot->valid,.auto_mode=robot->auto_mode,
        .actual=robot->actual,.feedback=robot->feedback,.stop_reason=robot->stop_reason,
        .remote_supported=robot->remote_supported,.remote_flags=robot->remote_flags,.remote_sequence=robot->remote_sequence,
        .nav_supported=robot->nav_supported,.nav_state=robot->state,.near_mask=robot->near_mask,.feedback_supported=robot->feedback_supported,
        .feedback_phase=robot->feedback_phase,.feedback_retries=robot->feedback_retries};
    for(unsigned i=0;i<3;i++){state.mm[i]=robot->mm[i];state.error[i]=robot->error[i];}
    phone_begin_write(&s->state_sequence);s->state=state;phone_end_write(&s->state_sequence);last_state=now;
}
