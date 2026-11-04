/* 独立检查更新率与检测结果接口约定。 */
#include "dashboard_detections.h"
#include "dashboard_rate.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do {++checks;if(!(c)){fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
int main(int argc,char **argv)
{
    if(argc!=2)return 2;
    unsigned checks=0;
    struct dashboard_rate r;dashboard_rate_init(&r,0,100);
    dashboard_rate_update(&r,49,1080);CHECK(!r.ready);
    dashboard_rate_update(&r,50,1100);CHECK(r.ready && fabs(r.hz-50)<.001);
    dashboard_rate_update(&r,150,3100);CHECK(fabs(r.hz-50)<.001);
    dashboard_rate_update(&r,150,4100);CHECK(r.hz==0);
    dashboard_rate_update(&r,145,5100);CHECK(!r.ready);
    dashboard_rate_update(&r,195,6100);CHECK(r.ready && fabs(r.hz-50)<.001);
    dashboard_rate_update(&r,195,100);CHECK(!r.ready);
    dashboard_rate_update(&r,220,1100);CHECK(r.ready && fabs(r.hz-25)<.001);
    struct dashboard_detection_parser p;dashboard_detections_init(&p);
    const char *a="FRAME 210 detections=2 inference_decode_ms=80.2044\r\nperson class=0 score=0.923 box=(1,2,3,4)\r\ncell phone class=67 score=0.84 box=(5,6,7,8)\r\n";
    for(size_t i=0;i<strlen(a)-1;++i)CHECK(!dashboard_detections_feed(&p,a+i,1,2000));
    CHECK(!p.latest.seen);CHECK(dashboard_detections_feed(&p,a+strlen(a)-1,1,2001)==1);
    CHECK(p.latest.seen && p.latest.total==2 && p.latest.count==2 && p.latest.frame==210 && p.latest.timestamp_ms==2001);
    CHECK(!strcmp(p.latest.objects[1].name,"cell phone") && fabs(p.latest.objects[0].score-.923)<.000001);
    CHECK(!strcmp(dashboard_object_label("person"),"人"));CHECK(!strcmp(dashboard_object_label("custom item"),"custom item"));
    char path[2048];snprintf(path,sizeof(path),"%s/detection-test.latest",argv[1]);
    CHECK(!dashboard_detections_write(path,&p.latest));
    struct dashboard_detections d={0};CHECK(dashboard_detections_read(path,&d)==1);
    CHECK(d.count==2 && d.timestamp_ms==2001 && !strcmp(d.objects[1].name,"cell phone"));
    const char *empty="FRAME 211 detections=0 inference_decode_ms=75\n";
    CHECK(dashboard_detections_feed(&p,empty,strlen(empty),2100)==1 && p.latest.count==0 && p.latest.total==0);
    CHECK(!dashboard_detections_write(path,&p.latest));CHECK(dashboard_detections_read(path,&d)==1 && d.count==0);
    const char *bad="FRAME 212 detections=1 inference_decode_ms=80\nperson class=0 score=nan box=(1,2,3,4)\n";
    CHECK(!dashboard_detections_feed(&p,bad,strlen(bad),2200));CHECK(p.latest.frame==211);
    const char *incomplete="FRAME 213 detections=2\nchair class=56 score=0.2 box=(0,0,1,1)\nFRAME 214 detections=0\n";
    CHECK(dashboard_detections_feed(&p,incomplete,strlen(incomplete),2300)==1 && p.latest.frame==214 && p.latest.total==0);
    const char *header="FRAME 215 detections=8\n";
    CHECK(!dashboard_detections_feed(&p,header,strlen(header),2400));
    for(unsigned i=0;i<8;++i){
        char line[128];snprintf(line,sizeof(line),"item%u class=%u score=0.%u box=(0,0,10,10)\n",i,i,i+1);
        CHECK(dashboard_detections_feed(&p,line,strlen(line),2400)==(i==7));
    }
    CHECK(p.latest.count==6 && p.latest.total==8);
    CHECK(!strcmp(p.latest.objects[0].name,"item7") && !strcmp(p.latest.objects[5].name,"item2"));
    const char *bogus="FRAME 216 detections=513\n";CHECK(!dashboard_detections_feed(&p,bogus,strlen(bogus),2500));
    CHECK(p.latest.frame==215);
    char longline[5000];memset(longline,'x',sizeof(longline));longline[4999]='\n';
    CHECK(!dashboard_detections_feed(&p,longline,sizeof(longline),2600));
    CHECK(dashboard_detections_feed(&p,empty,strlen(empty),2700)==1);
    FILE *f=fopen(path,"w");CHECK(f!=NULL);fputs("YDET1 2000 1 1 1\nperson\t1.2\n",f);fclose(f);
    CHECK(dashboard_detections_read(path,&d)==-1 && !d.seen);
    f=fopen(path,"w");CHECK(f!=NULL);fputs("YDET1 2000 1 2 1\nperson\t0.9\n",f);fclose(f);
    CHECK(dashboard_detections_read(path,&d)==-1 && !d.seen);
    CHECK(!remove(path));CHECK(dashboard_detections_read(path,&d)==0 && !d.seen);
    printf("PASS rate and detections: %u checks; elapsed-time Hz, gaps/reset, fragmented logs, complete/empty frames, top6, malformed input and snapshots\n",checks);
    return 0;
}
