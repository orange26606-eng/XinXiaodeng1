/* 1.1.0：原子发布YOLO_FPS_V1快照，隐藏无效或过期数值。 */
#define _POSIX_C_SOURCE 200809L
#include "dashboard_yolo_fps.h"
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static int unsigned_token(const char *token,uint64_t *value) {
    if(!token || !*token)return 0;
    for(const unsigned char *p=(const unsigned char *)token;*p;p++)if(!isdigit(*p))return 0;
    char *end;
    errno=0;
    unsigned long long parsed=strtoull(token,&end,10);
    if(errno || *end || parsed>UINT64_MAX)return 0;
    *value=(uint64_t)parsed;
    return 1;
}
int dashboard_yolo_fps_parse(const char *bytes,size_t count,struct dashboard_yolo_fps *out) {
    struct dashboard_yolo_fps next={0};
    memset(out,0,sizeof(*out));
    if(!bytes || !count || count>=256 || bytes[count-1]!='\n' || memchr(bytes,0,count))return -1;
    for(size_t i=0;i+1<count;i++)if(bytes[i]=='\n' || bytes[i]=='\r')return -1;
    char line[256],*save=NULL,*tokens[5];
    memcpy(line,bytes,count);line[count-1]=0;
    for(unsigned i=0;i<5;i++) {
        tokens[i]=strtok_r(i?NULL:line," \t",&save);
        if(!tokens[i])return -1;
    }
    if(strtok_r(NULL," \t",&save) || strcmp(tokens[0],"YOLO_FPS_V1") ||
       !unsigned_token(tokens[1],&next.pid) || !next.pid || next.pid>INT_MAX ||
       !unsigned_token(tokens[2],&next.timestamp_ms) || !next.timestamp_ms ||
       !unsigned_token(tokens[3],&next.total_frames) || !next.total_frames)return -1;
    char *end;
    errno=0;
    next.fps=strtod(tokens[4],&end);
    if(errno || end==tokens[4] || *end || !isfinite(next.fps) || next.fps<0)return -1;
    next.seen=1;*out=next;
    return 1;
}
int dashboard_yolo_fps_read(const char *path,struct dashboard_yolo_fps *out) {
    memset(out,0,sizeof(*out));
    if(!path || !*path)return 0;
    int fd=open(path,O_RDONLY|O_CLOEXEC|O_NONBLOCK|O_NOFOLLOW);
    if(fd<0)return errno==ENOENT?0:-1;
    struct stat st;
    if(fstat(fd,&st) || !S_ISREG(st.st_mode) || st.st_size<1 || st.st_size>=256) {
        close(fd);return -1;
    }
    char bytes[256];size_t used=0;
    for(;;) {
        ssize_t n=read(fd,bytes+used,sizeof(bytes)-used);
        if(n<0 && errno==EINTR)continue;
        if(n<0 || (n>0 && used+(size_t)n>=sizeof(bytes))) { close(fd);return -1; }
        if(!n)break;
        used+=(size_t)n;
    }
    if(close(fd))return -1;
    return dashboard_yolo_fps_parse(bytes,used,out);
}
int dashboard_yolo_fps_value(const struct dashboard_yolo_fps *sample,uint64_t now_ms,double *fps) {
    if(fps)*fps=0;
    if(!sample || !sample->seen || !fps || !sample->pid || !sample->total_frames ||
       !isfinite(sample->fps) || sample->fps<0 || sample->timestamp_ms>now_ms ||
       now_ms-sample->timestamp_ms>2500)return 0;
    *fps=sample->fps;
    return 1;
}
