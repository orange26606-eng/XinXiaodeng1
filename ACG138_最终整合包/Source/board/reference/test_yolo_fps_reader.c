/* 1.1.0：独立测试快照解析、文件和时效，不访问设备。 */
#define _POSIX_C_SOURCE 200809L
#include "dashboard_yolo_fps.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static unsigned checks;
#define CHECK(expression) do { ++checks; if(!(expression)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#expression); return 1; } } while(0)
static int write_file(const char *path,const char *bytes) {
    FILE *f=fopen(path,"w");if(!f)return -1;
    int bad=fputs(bytes,f)<0;
    return fclose(f) || bad?-1:0;
}
int main(void) {
    struct dashboard_yolo_fps sample={0};double fps=99;
    static const char valid[]="YOLO_FPS_V1 1234 10000 120 14.875\n";
    CHECK(dashboard_yolo_fps_parse(valid,sizeof(valid)-1,&sample)==1);
    CHECK(sample.pid==1234 && sample.timestamp_ms==10000 && sample.total_frames==120 && sample.fps==14.875);
    CHECK(dashboard_yolo_fps_value(&sample,10000,&fps)==1 && fps==14.875);
    CHECK(dashboard_yolo_fps_value(&sample,12500,&fps)==1 && fps==14.875);
    CHECK(dashboard_yolo_fps_value(&sample,12501,&fps)==0 && fps==0);
    CHECK(dashboard_yolo_fps_value(&sample,9999,&fps)==0 && fps==0);
    static const char *invalid[]={
        "", "YOLO_FPS_V1 1234 10000 120 14.875", "YOLO_FPS_V2 1234 10000 120 14.8\n",
        "YOLO_FPS_V1 0 10000 120 14.8\n", "YOLO_FPS_V1 -1 10000 120 14.8\n",
        "YOLO_FPS_V1 2147483648 10000 120 14.8\n", "YOLO_FPS_V1 1234 0 120 14.8\n",
        "YOLO_FPS_V1 1234 -100 120 14.8\n", "YOLO_FPS_V1 1234 18446744073709551616 120 14.8\n",
        "YOLO_FPS_V1 1234 10000 0 14.8\n", "YOLO_FPS_V1 1234 10000 -1 14.8\n",
        "YOLO_FPS_V1 1234 10000 120 nan\n", "YOLO_FPS_V1 1234 10000 120 inf\n",
        "YOLO_FPS_V1 1234 10000 120 -0.1\n", "YOLO_FPS_V1 1234 10000 120 14.8junk\n",
        "YOLO_FPS_V1 1234 10000 120 14.8 extra\n", "YOLO_FPS_V1 1234 10000 120 14.8\n\n",
        "YOLO_FPS_V1 1234 10000 120\n"
    };
    for(size_t i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        sample.seen=1;sample.fps=88;
        CHECK(dashboard_yolo_fps_parse(invalid[i],strlen(invalid[i]),&sample)<0);
        CHECK(!sample.seen && !sample.fps);
    }
    char binary[sizeof(valid)];memcpy(binary,valid,sizeof(valid));binary[12]=0;
    CHECK(dashboard_yolo_fps_parse(binary,sizeof(valid)-1,&sample)<0 && !sample.seen);
    const char zero[]="YOLO_FPS_V1 1234 10000 120 0.000\n";
    CHECK(dashboard_yolo_fps_parse(zero,sizeof(zero)-1,&sample)==1);
    CHECK(dashboard_yolo_fps_value(&sample,10001,&fps)==1 && fps==0);
    char directory[]="/tmp/yolo-fps-reader.XXXXXX",path[256],other[256],link[256],fifo[256];
    CHECK(mkdtemp(directory)!=NULL);
    snprintf(path,sizeof(path),"%s/latest",directory);snprintf(other,sizeof(other),"%s/new",directory);
    snprintf(link,sizeof(link),"%s/link",directory);snprintf(fifo,sizeof(fifo),"%s/fifo",directory);
    CHECK(dashboard_yolo_fps_read(path,&sample)==0 && !sample.seen);
    CHECK(write_file(path,valid)==0);
    CHECK(dashboard_yolo_fps_read(path,&sample)==1 && sample.fps==14.875);
    CHECK(write_file(other,"YOLO_FPS_V1 2345 11000 135 15.000\n")==0);
    CHECK(rename(other,path)==0);
    CHECK(dashboard_yolo_fps_read(path,&sample)==1 && sample.pid==2345 && sample.fps==15);
    CHECK(write_file(path,"invalid\n")==0);
    CHECK(dashboard_yolo_fps_read(path,&sample)<0 && !sample.seen && !sample.fps);
    CHECK(write_file(path,valid)==0);
    CHECK(symlink(path,link)==0);
    CHECK(dashboard_yolo_fps_read(link,&sample)<0 && !sample.seen);
    CHECK(mkfifo(fifo,0600)==0);
    CHECK(dashboard_yolo_fps_read(fifo,&sample)<0 && !sample.seen);
    CHECK(dashboard_yolo_fps_read(directory,&sample)<0 && !sample.seen);
    CHECK(dashboard_yolo_fps_read(NULL,&sample)==0 && !sample.seen);
    CHECK(unlink(path)==0 && unlink(link)==0 && unlink(fifo)==0 && rmdir(directory)==0);
    printf("YOLO_FPS_READER checks=%u parser=PASS atomic_replace=PASS stale_2500ms=PASS invalid_clear=PASS\n",checks);
    return 0;
}
