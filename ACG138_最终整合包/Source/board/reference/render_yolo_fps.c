/* 1.1.0：使用实际C绘制器生成静态样例，不访问板卡。 */
#include "dashboard_canvas.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void write_ppm(const char *directory,const char *name,const uint32_t *pixels) {
    char path[2048];snprintf(path,sizeof(path),"%s/%s.ppm",directory,name);
    FILE *f=fopen(path,"wb");if(!f){perror(path);exit(1);}
    fprintf(f,"P6\n%d %d\n255\n",DASHBOARD_WIDTH,DASHBOARD_HEIGHT);
    for(size_t i=0;i<DASHBOARD_WIDTH*DASHBOARD_HEIGHT;i++) {
        const unsigned char rgb[]={(unsigned char)(pixels[i]>>16),(unsigned char)(pixels[i]>>8),(unsigned char)pixels[i]};
        if(fwrite(rgb,1,3,f)!=3)exit(1);
    }
    if(fclose(f))exit(1);
}
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    uint32_t *pixels=calloc(DASHBOARD_WIDTH*DASHBOARD_HEIGHT,sizeof(*pixels));
    uint32_t *base=calloc(DASHBOARD_WIDTH*DASHBOARD_HEIGHT,sizeof(*base));
    uint32_t *video=calloc(DASHBOARD_VIDEO_WIDTH*DASHBOARD_VIDEO_HEIGHT,sizeof(*video));
    if(!pixels || !base || !video)return 1;
    for(unsigned y=0;y<DASHBOARD_VIDEO_HEIGHT;y++)for(unsigned x=0;x<DASHBOARD_VIDEO_WIDTH;x++) {
        unsigned r=28+x/20,g=55+y/12,b=x<DASHBOARD_VIDEO_WIDTH/2?112:67;
        if(x%90<2 || y%90<2){r+=22;g+=22;b+=22;}
        video[(size_t)y*DASHBOARD_VIDEO_WIDTH+x]=(r<<16)|(g<<8)|b;
    }
    struct imu_model model;imu_model_init(&model,1000);
    model.simulated=1;model.have_angles=model.have_sequence=model.seen_packet=1;
    model.state=IMU_LIVE;model.last_good_ms=model.last_packet_ms=10000;
    model.good_count=model.sequence=120;model.degrees[0]=.15;model.degrees[1]=-.21;model.degrees[2]=45;
    struct dashboard_heading heading;dashboard_heading_init(&heading);dashboard_heading_update(&heading,&model,10000);
    struct dashboard_rate rate;dashboard_rate_init(&rate,0,9000);dashboard_rate_update(&rate,50,10000);
    struct dashboard_detections detections={.seen=1,.timestamp_ms=10000,.count=3,.total=3};
    const char *names[]={"person","bottle","chair"};
    for(unsigned i=0;i<3;i++){snprintf(detections.objects[i].name,64,"%s",names[i]);detections.objects[i].score=.94-i*.1;}
    struct robot_model robot={.seen=1,.last_ms=10000,.valid=7,.mm={800,1200,4500},.auto_mode=1,.state=4,.actual=6};
    struct dashboard_yolo_fps fps={.pid=1234,.timestamp_ms=10000,.total_frames=120,.fps=14.8,.seen=1};
    dashboard_canvas_draw(base,&model,video,120,10000,10000,&rate,&detections,&heading,&robot,NULL);
    dashboard_canvas_draw(pixels,&model,video,120,10000,10000,&rate,&detections,&heading,&robot,&fps);
    for(unsigned y=0;y<DASHBOARD_HEIGHT;y++)for(unsigned x=0;x<DASHBOARD_WIDTH;x++)
        if(pixels[(size_t)y*DASHBOARD_WIDTH+x]!=base[(size_t)y*DASHBOARD_WIDTH+x] &&
           !(x>=20 && x<920 && y>=104 && y<142)) {
            fprintf(stderr,"FPS overlay escaped its camera-top region at %u,%u\n",x,y);return 1;
        }
    write_ppm(argv[1],"yolo-fps-live",pixels);
    fps.seen=0;
    dashboard_canvas_draw(pixels,&model,video,120,10000,10000,&rate,&detections,&heading,&robot,&fps);
    write_ppm(argv[1],"yolo-fps-waiting",pixels);
    fps.seen=1;fps.timestamp_ms=7499;
    dashboard_canvas_draw(pixels,&model,video,120,10000,10000,&rate,&detections,&heading,&robot,&fps);
    write_ppm(argv[1],"yolo-fps-stale",pixels);
    dashboard_canvas_draw(pixels,&model,video,120,10000,10000,&rate,NULL,&heading,&robot,NULL);
    write_ppm(argv[1],"yolo-fps-disabled",pixels);
    dashboard_canvas_draw(pixels,&model,video,120,7000,10000,&rate,&detections,&heading,&robot,&fps);
    write_ppm(argv[1],"yolo-fps-stale-camera",pixels);
    free(pixels);free(base);free(video);
    puts("YOLO_FPS_RENDER fixtures=5 overlay_bounds=PASS unchanged_imu_and_object_regions=PASS");
    return 0;
}
