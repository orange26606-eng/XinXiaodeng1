/* 解析 YOLO FRAME/detections 日志中的名称、类别、分数和检测框。只发布完整帧，无目标帧清空上次结果；置信度沿用模型输出。 */
/* 1.1.1，2026-10-15：显示已过时间，保留低频日志结果供查看。 */
#define _POSIX_C_SOURCE 200809L
#include "dashboard_detections.h"
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
static int valid_name(const char *s)
{
    size_t n=strlen(s);
    if(!n || n>=64) return 0;
    for(size_t i=0;i<n;++i) if((unsigned char)s[i]<32 || (unsigned char)s[i]==127) return 0;
    return 1;
}
static void add_object(struct dashboard_detections *d,const char *name,double score)
{
    unsigned at=0;
    while(at<d->count && d->objects[at].score>=score) ++at;
    if(at>=DASHBOARD_OBJECTS) return;
    unsigned end=d->count<DASHBOARD_OBJECTS ? d->count : DASHBOARD_OBJECTS-1;
    for(unsigned i=end;i>at;--i) d->objects[i]=d->objects[i-1];
    snprintf(d->objects[at].name,sizeof(d->objects[at].name),"%s",name);
    d->objects[at].score=score;
    if(d->count<DASHBOARD_OBJECTS) ++d->count;
}
void dashboard_detections_init(struct dashboard_detection_parser *p)
{ memset(p,0,sizeof(*p)); }
static int complete(struct dashboard_detection_parser *p,uint64_t now)
{
    p->pending.timestamp_ms=now; p->pending.seen=1; p->latest=p->pending;
    p->active=0; return 1;
}
static int parse_line(struct dashboard_detection_parser *p,char *line,uint64_t now)
{
    if(!strncmp(line,"FRAME ",6)) {
        uint64_t frame; unsigned total; int used=0;
        p->active=0;
        if(sscanf(line,"FRAME %" SCNu64 " detections=%u%n",&frame,&total,&used)!=2 ||
           total>512 || (line[used] && !isspace((unsigned char)line[used]))) return 0;
        memset(&p->pending,0,sizeof(p->pending));
        p->pending.frame=frame; p->pending.total=total; p->expected=total; p->received=0;
        p->active=1;
        return total ? 0 : complete(p,now);
    }
    if(!p->active) return 0;
    char *mark=strstr(line," class=");
    if(!mark) return 0; /* 非检测诊断也可能写入stderr。 */
    unsigned cls; double score,x1,y1,x2,y2; int used=0;
    if(sscanf(mark," class=%u score=%lf box=(%lf,%lf,%lf,%lf)%n",&cls,&score,&x1,&y1,&x2,&y2,&used)!=6 ||
       !isfinite(score) || score<0 || score>1 || !isfinite(x1) || !isfinite(y1) ||
       !isfinite(x2) || !isfinite(y2) || cls>65535) { p->active=0; return 0; }
    for(const char *tail=mark+used;*tail;++tail)
        if(!isspace((unsigned char)*tail)) { p->active=0; return 0; }
    *mark=0;
    if(!valid_name(line)) { p->active=0; return 0; }
    add_object(&p->pending,line,score);
    ++p->received;
    return p->received==p->expected ? complete(p,now) : 0;
}
int dashboard_detections_feed(struct dashboard_detection_parser *p,const char *bytes,size_t size,uint64_t now)
{
    int changed=0;
    for(size_t i=0;i<size;++i) {
        char ch=bytes[i];
        if(ch=='\r') continue;
        if(ch=='\n') {
            if(!p->overflow) { p->line[p->used]=0; changed+=parse_line(p,p->line,now); }
            else p->active=0;
            p->used=0;p->overflow=0;
        } else if(ch==0 || p->used+1>=sizeof(p->line)) p->overflow=1;
        else if(!p->overflow) p->line[p->used++]=ch;
    }
    return changed;
}
int dashboard_detections_write(const char *path,const struct dashboard_detections *d)
{
    char temp[4096];
    if(!path || strlen(path)+5>=sizeof(temp) || !d->seen || d->count>DASHBOARD_OBJECTS) return -1;
    snprintf(temp,sizeof(temp),"%s.tmp",path);
    FILE *f=fopen(temp,"w"); if(!f) return -1;
    int bad=fprintf(f,"YDET1 %" PRIu64 " %" PRIu64 " %u %u\n",d->timestamp_ms,d->frame,d->total,d->count)<0;
    for(unsigned i=0;i<d->count;++i)
        if(fprintf(f,"%s\t%.9g\n",d->objects[i].name,d->objects[i].score)<0) bad=1;
    if(fclose(f)) bad=1;
    if(bad || rename(temp,path)) { remove(temp); return -1; }
    return 0;
}
int dashboard_detections_read(const char *path,struct dashboard_detections *d)
{
    struct dashboard_detections next={0};
    if(!path) { memset(d,0,sizeof(*d)); return 0; }
    FILE *f=NULL;
#ifndef _WIN32
    int fd=open(path,O_RDONLY|O_CLOEXEC|O_NONBLOCK|O_NOFOLLOW);
    if(fd<0) { memset(d,0,sizeof(*d)); return errno==ENOENT ? 0 : -1; }
    struct stat st;
    if(fstat(fd,&st) || !S_ISREG(st.st_mode) || st.st_size>2048) { close(fd); memset(d,0,sizeof(*d)); return -1; }
    f=fdopen(fd,"r"); if(!f) { close(fd); memset(d,0,sizeof(*d)); return -1; }
#else
    f=fopen(path,"r"); if(!f) { memset(d,0,sizeof(*d)); return errno==ENOENT ? 0 : -1; }
#endif
    char line[256]; int used=0;
    if(!fgets(line,sizeof(line),f) || sscanf(line,"YDET1 %" SCNu64 " %" SCNu64 " %u %u%n",
       &next.timestamp_ms,&next.frame,&next.total,&next.count,&used)!=4 || next.total>512 ||
       next.count!=(next.total<DASHBOARD_OBJECTS ? next.total : DASHBOARD_OBJECTS)) goto invalid;
    for(const char *tail=line+used;*tail;++tail) if(!isspace((unsigned char)*tail)) goto invalid;
    for(unsigned i=0;i<next.count;++i) {
        if(!fgets(line,sizeof(line),f)) goto invalid;
        char *tab=strchr(line,'\t'); if(!tab) goto invalid;
        *tab++=0; if(!valid_name(line)) goto invalid;
        char *end; errno=0; double score=strtod(tab,&end);
        if(errno || end==tab || !isfinite(score) || score<0 || score>1) goto invalid;
        for(;*end;++end) if(!isspace((unsigned char)*end)) goto invalid;
        snprintf(next.objects[i].name,sizeof(next.objects[i].name),"%s",line);
        next.objects[i].score=score;
    }
    if(fgetc(f)!=EOF || ferror(f)) goto invalid;
    if(fclose(f)) { memset(d,0,sizeof(*d)); return -1; }
    next.seen=1; *d=next; return 1;
invalid:
    fclose(f); memset(d,0,sizeof(*d)); return -1;
}
int dashboard_detections_age_ms(const struct dashboard_detections *d,uint64_t now,uint64_t *age)
{
    if(!d || !d->seen || !age) return 0;
    if(d->timestamp_ms>now) {
        /* 界面采样now后、读取文件前，仍可能发布更新的快照。 */
        if(d->timestamp_ms-now>1000) return 0;
        *age=0;
    } else *age=now-d->timestamp_ms;
    return 1;
}
const char *dashboard_object_label(const char *name)
{
    static const char *const labels[][2]={
        {"person","人"},{"bicycle","自行车"},{"car","汽车"},{"motorcycle","摩托车"},
        {"airplane","飞机"},{"bus","公交车"},{"train","火车"},{"truck","卡车"},{"boat","船"},
        {"traffic light","交通灯"},{"fire hydrant","消防栓"},{"stop sign","停车标志"},
        {"parking meter","停车计时器"},{"bench","长椅"},{"bird","鸟"},{"cat","猫"},{"dog","狗"},
        {"horse","马"},{"sheep","羊"},{"cow","牛"},{"elephant","大象"},{"bear","熊"},
        {"zebra","斑马"},{"giraffe","长颈鹿"},{"backpack","背包"},{"umbrella","雨伞"},
        {"handbag","手提包"},{"tie","领带"},{"suitcase","行李箱"},{"frisbee","飞盘"},
        {"skis","滑雪板"},{"snowboard","单板滑雪板"},{"sports ball","球"},{"kite","风筝"},
        {"baseball bat","棒球棒"},{"baseball glove","棒球手套"},{"skateboard","滑板"},
        {"surfboard","冲浪板"},{"tennis racket","网球拍"},{"bottle","瓶子"},
        {"wine glass","酒杯"},{"cup","杯子"},{"fork","叉子"},{"knife","刀"},{"spoon","勺子"},
        {"bowl","碗"},{"banana","香蕉"},{"apple","苹果"},{"sandwich","三明治"},
        {"orange","橙子"},{"broccoli","西兰花"},{"carrot","胡萝卜"},{"hot dog","热狗"},
        {"pizza","披萨"},{"donut","甜甜圈"},{"cake","蛋糕"},{"chair","椅子"},
        {"couch","沙发"},{"potted plant","盆栽"},{"bed","床"},{"dining table","餐桌"},
        {"toilet","马桶"},{"tv","电视"},{"laptop","笔记本电脑"},{"mouse","鼠标"},
        {"remote","遥控器"},{"keyboard","键盘"},{"cell phone","手机"},{"microwave","微波炉"},
        {"oven","烤箱"},{"toaster","烤面包机"},{"sink","水槽"},{"refrigerator","冰箱"},
        {"book","书"},{"clock","钟"},{"vase","花瓶"},{"scissors","剪刀"},
        {"teddy bear","玩具熊"},{"hair drier","吹风机"},{"toothbrush","牙刷"}
    };
    for(size_t i=0;i<sizeof(labels)/sizeof(labels[0]);++i) if(!strcmp(name,labels[i][0])) return labels[i][1];
    return name;
}
