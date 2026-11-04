/* 静态绘制样例，不代表传感器输出。覆盖等待、实时、过期、错误、实测更新率、目标为空或禁用、日志年龄及溢出列表。通过真实缩放器检查等比例与拉伸，通过不对称左右目标和向右箭头检查交换而非镜像，并覆盖三角度、参考航向、双语长名称及两位置信度布局。







 */
#include "dashboard_canvas.h"
#include "dashboard_video.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void write_ppm(const char *directory,const char *name,const uint32_t *p)
{
    char path[2048]; snprintf(path,sizeof(path),"%s/%s.ppm",directory,name);
    FILE *f=fopen(path,"wb"); if(!f) { perror(path); exit(1); }
    fprintf(f,"P6\n%d %d\n255\n",DASHBOARD_WIDTH,DASHBOARD_HEIGHT);
    for(unsigned i=0;i<DASHBOARD_WIDTH*DASHBOARD_HEIGHT;++i) {
        unsigned char rgb[3]={(unsigned char)(p[i]>>16),(unsigned char)(p[i]>>8),(unsigned char)p[i]};
        if(fwrite(rgb,1,3,f)!=3) exit(1);
    }
    if(fclose(f)) exit(1);
}
int main(int argc,char **argv)
{
    if(argc!=2) return 2;
    uint32_t *p=calloc(DASHBOARD_WIDTH*DASHBOARD_HEIGHT,sizeof(*p));
    uint32_t *v=calloc(DASHBOARD_VIDEO_WIDTH*DASHBOARD_VIDEO_HEIGHT,sizeof(*v));
    uint8_t *source=malloc(640*640*3);
    if(!p || !v || !source) return 1;
    for(unsigned y=0;y<640;++y) for(unsigned x=0;x<640;++x) {
        unsigned local=x%320;
        unsigned r=x<320 ? 35 : 45,g=x<320 ? 70 : 130,b=x<320 ? 150 : 65;
        if(local%64==0 || y%64==0) { r+=25;g+=25;b+=25; }
        const char *letter=x<320 ? "100100100100111" : "110101110101101";
        if(local>=112 && local<184 && y>=160 && y<280 &&
           letter[((y-160)/24)*3+(local-112)/24]=='1') r=g=b=245;
        if(((local>=80 && local<240) && ((y>=110 && y<114) || (y>=326 && y<330))) ||
           ((y>=110 && y<330) && ((local>=80 && local<84) || (local>=236 && local<240)))) r=g=b=235;
        int arrow_y=abs((int)y-455);
        if((local>=70 && local<235 && arrow_y<4) ||
           (local>=220 && local<=260 && arrow_y<=(int)(260-local))) { r=250;g=220;b=90; }
        size_t at=((size_t)y*640+x)*3;
        source[at]=(uint8_t)r;source[at+1]=(uint8_t)g;source[at+2]=(uint8_t)b;
    }
    struct dashboard_video_layout layout={DASHBOARD_RGB24,640,640,1920,DASHBOARD_BT601,DASHBOARD_FULL};
    if(dashboard_video_convert(&layout,source,640*640*3,v,DASHBOARD_VIDEO_WIDTH*DASHBOARD_VIDEO_HEIGHT,
                                     DASHBOARD_VIDEO_WIDTH,DASHBOARD_VIDEO_HEIGHT)) return 1;
    struct imu_model m; imu_model_init(&m,1000); m.simulated=1;
    struct dashboard_rate rate;dashboard_rate_init(&rate,0,1000);
    struct dashboard_heading heading; dashboard_heading_init(&heading);
    struct dashboard_detections det={0};
    dashboard_canvas_draw(p,&m,NULL,0,0,1000,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-waiting",p);
    m.have_angles=m.have_sequence=m.seen_packet=1;
    m.state=IMU_LIVE; m.last_good_ms=m.last_packet_ms=2000; m.sequence=260; m.good_count=260;
    m.degrees[0]=12.34; m.degrees[1]=-5.67; m.degrees[2]=178.90;
    dashboard_heading_update(&heading,&m,2020);
    m.degrees[0]=0.15; m.degrees[1]=-0.21; m.degrees[2]=-136.10;
    ++m.good_count; dashboard_heading_update(&heading,&m,2020);
    dashboard_rate_update(&rate,51,2020);
    det.seen=1;det.timestamp_ms=2000;det.total=det.count=6;
    const char *names[]={"person","cell phone","bottle","chair","tv","keyboard"};
    for(unsigned i=0;i<6;++i){snprintf(det.objects[i].name,64,"%s",names[i]);det.objects[i].score=.963-i*.079;}
    dashboard_canvas_draw(p,&m,v,80,2000,2020,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-native",p);
    if(dashboard_video_convert(&layout,source,640*640*3,v,DASHBOARD_VIDEO_WIDTH*DASHBOARD_VIDEO_HEIGHT,
                                     DASHBOARD_VIDEO_WIDTH,DASHBOARD_VIDEO_HEIGHT)) return 1;
    dashboard_canvas_draw(p,&m,v,80,2000,2020,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-live",p);
    m.last_good_ms=m.last_packet_ms=5200;
    dashboard_canvas_draw(p,&m,v,112,5200,5200,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-held",p);
    m.last_good_ms=m.last_packet_ms=14200;
    dashboard_canvas_draw(p,&m,v,202,14200,14200,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-overdue",p);
    m.last_good_ms=m.last_packet_ms=73200;
    dashboard_canvas_draw(p,&m,v,792,73200,73200,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-minute",p);
    m.last_good_ms=m.last_packet_ms=2200;
    det.total=12;
    dashboard_canvas_draw(p,&m,v,82,2200,2200,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-overflow",p);
    det.total=6;
    m.last_good_ms=m.last_packet_ms=2000;
    dashboard_canvas_draw(p,&m,v,80,2000,4500,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-stale",p);
    m.state=IMU_DEVICE_ERROR; m.status=3; m.error_count=1; m.last_packet_ms=2300;
    dashboard_canvas_draw(p,&m,v,81,2300,2300,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-error",p);
    det.total=det.count=0;
    dashboard_canvas_draw(p,&m,v,81,2300,2300,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-empty",p);
    dashboard_canvas_draw(p,&m,v,81,2300,2300,&rate,NULL,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-disabled",p);
    dashboard_canvas_draw(p,&m,v,90,4500,4500,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],"dashboard-detection-stale",p);
    m.state=IMU_LIVE; m.last_good_ms=m.last_packet_ms=9000;
    const double turns[]={0,90,-90,179.9,-179.9};
    const char *files[]={"heading-zero","heading-left","heading-right","heading-wrap-positive","heading-wrap-negative"};
    for(unsigned i=0;i<5;++i) {
        m.degrees[2]=heading.reference_degrees+turns[i];
        if(m.degrees[2]>=180) m.degrees[2]-=360;
        if(m.degrees[2]<-180) m.degrees[2]+=360;
        ++m.good_count; dashboard_heading_update(&heading,&m,9000);
        dashboard_canvas_draw(p,&m,v,90,9000,9000,&rate,&det,&heading,NULL,NULL); write_ppm(argv[1],files[i],p);
    }
    const char *long_names[]={"parking meter","refrigerator","baseball glove","snowboard","traffic light","custom item"};
    det.total=det.count=6; det.timestamp_ms=8000;
    for(unsigned i=0;i<6;++i) snprintf(det.objects[i].name,64,"%s",long_names[i]);
    dashboard_canvas_draw(p,&m,v,90,9000,9000,&rate,&det,&heading,NULL,NULL);
    write_ppm(argv[1],"dashboard-bilingual-long",p);
    free(source); free(p); free(v); return 0;
}
