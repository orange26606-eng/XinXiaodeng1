/* 回放600帧运行日志，时间戳使用主机测试时钟；检查整组替换、最后结果保留、空结果及计时语义。
 */
#include "dashboard_detections.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(c) do { ++checks; if(!(c)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c); return 1; } } while(0)
int main(int argc,char **argv)
{
    if(argc!=2) return 2;
    unsigned checks=0,groups=0,empty=0;
    uint64_t age=999,complete_ms=0;
    struct dashboard_detection_parser parser;
    dashboard_detections_init(&parser);
    CHECK(!dashboard_detections_age_ms(NULL,1000,&age));
    CHECK(!dashboard_detections_age_ms(&parser.latest,1000,&age));
    FILE *f=fopen(argv[1],"rb"); CHECK(f!=NULL);
    char chunk[13]; size_t n;
    while((n=fread(chunk,1,sizeof(chunk),f))!=0) {
        uint64_t now=100000+(uint64_t)groups*3000;
        int changed=dashboard_detections_feed(&parser,chunk,n,now);
        if(!changed) continue;
        CHECK(changed==1);
        CHECK(parser.latest.frame==(groups ? (uint64_t)groups*30 : 1));
        CHECK(parser.latest.timestamp_ms==now);
        CHECK(dashboard_detections_age_ms(&parser.latest,now,&age) && age==0);
        if(!parser.latest.total) {
            ++empty;
            CHECK((parser.latest.frame==90 || parser.latest.frame==180) && parser.latest.count==0);
        }
        complete_ms=now; ++groups;
    }
    CHECK(!ferror(f)); CHECK(!fclose(f));
    CHECK(groups==20 && empty==2);
    CHECK(parser.latest.frame==570 && parser.latest.total==4 && parser.latest.count==4);
    CHECK(!strcmp(parser.latest.objects[0].name,"person") && fabs(parser.latest.objects[0].score-.85498)<1e-8);
    CHECK(!strcmp(parser.latest.objects[1].name,"person") && fabs(parser.latest.objects[1].score-.807617)<1e-8);
    CHECK(!strcmp(parser.latest.objects[2].name,"car") && !strcmp(parser.latest.objects[3].name,"chair"));
    CHECK(!strcmp(dashboard_object_label(parser.latest.objects[2].name),"汽车"));
    struct dashboard_detections held=parser.latest;
    const uint64_t intervals[]={1,1499,1500,3000,9999,10000,60000,86400123};
    for(unsigned i=0;i<sizeof(intervals)/sizeof(intervals[0]);++i) {
        CHECK(dashboard_detections_age_ms(&parser.latest,complete_ms+intervals[i],&age));
        CHECK(age==intervals[i]);
        CHECK(!memcmp(&held,&parser.latest,sizeof(held)));
    }
    CHECK(dashboard_detections_age_ms(&held,complete_ms-1,&age) && age==0);
    CHECK(dashboard_detections_age_ms(&held,complete_ms-1000,&age) && age==0);
    CHECK(!dashboard_detections_age_ms(&held,complete_ms-1001,&age));
    CHECK(!dashboard_detections_age_ms(&held,complete_ms,NULL));
    const char *partial="FRAME 600 detections=2\nperson class=0 score=0.95 box=(1,2,3,4)\n";
    CHECK(!dashboard_detections_feed(&parser,partial,strlen(partial),complete_ms+3000));
    CHECK(!memcmp(&held,&parser.latest,sizeof(held)));
    const char *last="car class=2 score=0.88 box=(1,2,3,4)\n";
    CHECK(dashboard_detections_feed(&parser,last,strlen(last),complete_ms+3100)==1);
    CHECK(parser.latest.frame==600 && parser.latest.count==2);
    CHECK(dashboard_detections_age_ms(&parser.latest,complete_ms+3100,&age) && age==0);
    const char *zero="FRAME 630 detections=0 inference_decode_ms=80\n";
    CHECK(dashboard_detections_feed(&parser,zero,strlen(zero),complete_ms+6100)==1);
    CHECK(parser.latest.frame==630 && parser.latest.total==0 && parser.latest.count==0);
    CHECK(dashboard_detections_age_ms(&parser.latest,complete_ms+6200,&age) && age==100);
    printf("PASS sampled-log age: %u checks; 20 real log groups, two empty groups, partial/complete replacement, held results and monotonic age\n",checks);
    return 0;
}
