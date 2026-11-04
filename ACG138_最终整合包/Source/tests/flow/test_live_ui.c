#include "live_nav_ui.h"
#include "phone_nav_snapshot.h"
#include "lab_real_session.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
int main(int argc,char**argv){assert(argc==2);lab_real_runtime_view v={0};v.context=(lab_real_context){LAB_REAL_TEST_FIXTURE,LAB_REAL_MONOTONIC,1,1};v.has_localization=1;v.localization.state=LAB_REAL_LOC_TRACKING;v.localization.map=(lab_real_grid_view){.context=v.context,.map_id=1,.map_version=1,.width=100,.height=100,.resolution_m=.1,.origin_x_m=-5,.origin_y_m=-5};int8_t*cells=malloc(10000);assert(cells);memset(cells,0,10000);v.localization.map.cells=cells;
 live_nav_ui u;live_nav_ui_init(&u);uint32_t*p=calloc(1280*720,sizeof(*p));assert(p);live_nav_ui_draw(&u,p,1280,720,&v,0);
 char *json=malloc(300000);size_t written=0;assert(json);
 v.navigation.body_length_m=.333;v.navigation.body_width_m=.32;v.navigation.margin_m=.015;v.navigation.local_resolution_m=.025;
 assert(phone_nav_snapshot_json(&v,json,300000,&written));assert(written>13000&&strstr(json,"\"map_id\":\"1\"")&&strstr(json,"\"cells\":\"AAAA"));assert(strstr(json,"\"length_m\":0.333")&&strstr(json,"\"local_resolution_m\":0.025")&&strstr(json,"\"lidar_offset_verified\":false"));
 assert(!phone_nav_snapshot_json(&v,json,16,&written)&&written==0&&json[0]==0);
 v.localization.pose.x_m=NAN;assert(phone_nav_snapshot_json(&v,json,300000,&written)&&strstr(json,"\"pose_valid\":false")&&strstr(json,"\"pose\":null"));v.localization.pose.x_m=0;free(json);
 assert(live_nav_ui_pick(&u,&v,421,420)==1);assert(fabs(u.goal_x-1)<.02&&fabs(u.goal_y)<.02);
 assert(live_nav_ui_select_world(&u,&v,2,1,0,0)==-1&&strstr(u.message,"STALE_MAP"));
 assert(live_nav_ui_select_world(&u,&v,1,2,0,0)==-1&&strstr(u.message,"STALE_MAP"));
 assert(live_nav_ui_select_world(&u,&v,1,1,NAN,0)==-1&&strstr(u.message,"INVALID_COORDINATES"));
 assert(live_nav_ui_select_world(&u,&v,1,1,6,0)==-1&&strstr(u.message,"OUTSIDE"));
 assert(live_nav_ui_select_world(&u,&v,1,1,1,2)==1&&u.goal_x==1&&u.goal_y==2);
 v.localization.state=LAB_REAL_LOC_LOST;assert(live_nav_ui_select_world(&u,&v,1,1,1,2)==1);assert(!v.enabled);v.localization.state=LAB_REAL_LOC_TRACKING;
 assert(live_nav_ui_button(20,100)==1&&live_nav_ui_button(200,100)==2&&live_nav_ui_button(400,100)==3&&live_nav_ui_button(500,100)==4);
 memset(cells,-1,10000);assert(live_nav_ui_pick(&u,&v,421,420)==1);assert(u.target_observation_pending);
 memset(cells,100,10000);assert(live_nav_ui_pick(&u,&v,421,420)==-1);assert(strstr(u.message,"OCCUPIED"));
 assert(live_nav_ui_pick(&u,&v,0,0)==0);v.has_localization=0;assert(live_nav_ui_pick(&u,&v,421,420)==-1);
 v.has_localization=1;memset(cells,0,10000);assert(live_nav_ui_pick(&u,&v,421,285)==1&&u.goal_y>.99&&u.goal_y<1.02); /* up is +Y */
 live_nav_ui_draw(&u,p,1280,720,&v,0);FILE*f=fopen(argv[1],"wb");assert(f);fprintf(f,"P6\n1280 720\n255\n");for(int i=0;i<1280*720;++i){unsigned char rgb[3]={(unsigned char)(p[i]>>16),(unsigned char)(p[i]>>8),(unsigned char)p[i]};assert(fwrite(rgb,1,3,f)==3);}assert(!fclose(f));
 free(cells);cells=calloc(400u*400u,1);assert(cells);v.localization.map.cells=cells;v.localization.map.width=v.localization.map.height=400;v.localization.map.resolution_m=.05;v.localization.map.origin_x_m=v.localization.map.origin_y_m=-10;
 clock_t started=clock();for(int i=0;i<10;++i)live_nav_ui_draw(&u,p,1280,720,&v,0);double seconds=(double)(clock()-started)/CLOCKS_PER_SEC;assert(seconds<3.0);printf("PASS 400x400 map: ten 1280x720 renders CPU time %.3fs\n",seconds);
 free(p);free(cells);
 puts("PASS screen coordinate inverse, +Y up, provisional unknown goal, occupied/no-localization rejection, buttons, rendered preview");return 0;
}
