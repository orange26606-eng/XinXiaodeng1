#include "dashboard_canvas.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void ppm(const char *dir,const char *name,uint32_t *p)
{
    char path[2048];snprintf(path,sizeof(path),"%s/%s.ppm",dir,name);
    FILE *f=fopen(path,"wb");if(!f)exit(1);
    fprintf(f,"P6\n%d %d\n255\n",DASHBOARD_WIDTH,DASHBOARD_HEIGHT);
    for(unsigned i=0;i<DASHBOARD_WIDTH*DASHBOARD_HEIGHT;++i) {
        unsigned char rgb[3]={(unsigned char)(p[i]>>16),(unsigned char)(p[i]>>8),(unsigned char)p[i]};
        if(fwrite(rgb,1,3,f)!=3)exit(1);
    }
    if(fclose(f))exit(1);
}
int main(int argc,char **argv)
{
    if(argc!=2)return 2;
    uint32_t *p=calloc(DASHBOARD_WIDTH*DASHBOARD_HEIGHT,sizeof(*p));
    uint32_t *video=calloc(DASHBOARD_VIDEO_WIDTH*DASHBOARD_VIDEO_HEIGHT,sizeof(*video));
    if(!p || !video)return 1;
    for(unsigned y=0;y<DASHBOARD_VIDEO_HEIGHT;++y)for(unsigned x=0;x<DASHBOARD_VIDEO_WIDTH;++x)
        video[(size_t)y*DASHBOARD_VIDEO_WIDTH+x]=x<DASHBOARD_VIDEO_WIDTH/2?0x002b6090:0x00356e4a;
    struct imu_model imu;imu_model_init(&imu,1000);imu.simulated=1;
    imu.have_angles=imu.have_sequence=imu.seen_packet=1;imu.state=IMU_LIVE;
    imu.last_good_ms=imu.last_packet_ms=2000;imu.good_count=1;imu.degrees[0]=1.23;imu.degrees[1]=-2.34;imu.degrees[2]=15;
    struct dashboard_heading heading;dashboard_heading_init(&heading);dashboard_heading_update(&heading,&imu,2000);
    struct dashboard_rate rate;dashboard_rate_init(&rate,0,1000);dashboard_rate_update(&rate,100,2000);
    struct dashboard_detections det={.seen=1,.timestamp_ms=2000};
    struct robot_model robot={.seen=1,.last_ms=2000,.valid=1,.mm={1234,4500,0},.error={0,0,2}};
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-live-max-error",p);
    robot.valid=7;robot.mm[0]=300;robot.mm[1]=1500;robot.mm[2]=4500;memset(robot.error,0,3);
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-three-distances",p);
    robot.auto_mode=1;robot.actual=6;robot.state=4;
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-mode-auto",p);
    robot.auto_mode=0;robot.actual=0;robot.state=0;
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-30cm-green",p);
    robot.mm[1]=299;
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-below30cm-red",p);
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-mode-fallback",p);
    robot.last_ms=1000;
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-stale",p);
    memset(&robot,0,sizeof(robot));
    dashboard_canvas_draw(p,&imu,video,10,2000,2020,&rate,&det,&heading,&robot,NULL);ppm(argv[1],"robot-waiting",p);
    free(video);free(p);return 0;
}
