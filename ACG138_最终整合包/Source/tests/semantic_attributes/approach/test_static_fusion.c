#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rk/src/app/object_scene_fusion.c"
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);assert(x);}}while(0)
static lab_real_runtime_view view(uint64_t now){lab_real_runtime_view v={0};v.context=(lab_real_context){LAB_REAL_TEST_FIXTURE,LAB_REAL_MONOTONIC,44,1};v.has_odometry=1;v.raw_odometry.quality=LAB_POSE_VALID;v.odometry_time_us=v.scan_time_us=now;return v;}
static obj_frame frame(uint64_t now,const char*label,double z){obj_frame f={0};f.epoch=11;f.frame_id=1;f.frame_us=f.generated_us=now;f.count=1;f.selected=10;obj_detection*d=f.objects;d->track_id=10;d->class_id=d->task_class_id=d->observed_class_id=4;d->observed=d->bbox_valid=d->bearing_valid=d->depth_valid=1;d->binding_generation=1;d->depth_eye=d->geometry_eye=d->eye=0;d->observation_frame_id=d->depth_frame_id=1;d->observation_time_us=d->depth_time_us=now;d->bbox[0]=400;d->bbox[1]=300;d->bbox[2]=560;d->bbox[3]=800;d->z=z;d->error=.02;strcpy(d->label,label);strcpy(d->tracking,"TRACKING");return f;}
static void wall(struct scan_frame*s,uint64_t now,double x){memset(s,0,sizeof(*s));s->session=1;s->sequence=1;s->timestamp_us=now;s->period_us=100000;for(int i=0;i<161;i++){double y=-2.+i*.025,r=hypot(x,y),angle=atan2(-y,x);if(angle<0)angle+=6.283185307179586;s->points[s->count++]=(struct scan_point){.angle_q14=(uint16_t)llround(angle*65536/6.283185307179586),.distance_q2=(uint32_t)llround(r*4000),.quality=30};}}
int main(void){uint64_t n=1000000;lab_real_runtime_view v=view(n);obj_scene_config cfg;obj_scene_config_default(&cfg);cfg.mode=1;cfg.mount_configured=1;cfg.mount_approximate=0;cfg.camera.enabled=cfg.camera.verified=1;cfg.camera.x=.1265;passage_config p;passage_config_default(&p);p.mount_x_m=p.mount_y_m=p.mount_yaw_rad=0;
 obj_frame f=frame(n,"box",.8);CHECK(sf_static_depth_available(f.objects,&f,n));CHECK(sf_static_box_depth_support(f.objects,&cfg,&f,NULL,&v,n,.1265,0,.8));CHECK(!sf_static_box_depth_support(f.objects,&cfg,&f,NULL,&v,n,.1265,0,2));strcpy(f.objects[0].label,"bottle");CHECK(!sf_static_box_depth_support(f.objects,&cfg,&f,NULL,&v,n,.1265,0,.8));strcpy(f.objects[0].label,"box");f.objects[0].depth_time_us=1;CHECK(!sf_static_box_depth_support(f.objects,&cfg,&f,NULL,&v,n,.1265,0,.8));f=frame(n,"box",.8);v.scan_time_us=n+400000;CHECK(!sf_static_box_depth_support(f.objects,&cfg,&f,NULL,&v,n+400000,.1265,0,.8));v=view(n);
 sf_hit hits[3]={{.x=1,.y=.3,.angle=.1,.full_track=-1},{.x=1.2,.y=0,.angle=.2,.full_track=-1},{.x=1,.y=.6,.angle=.3,.full_track=-1}};sf_cluster c[3];unsigned count=0;sf_add_cluster_min(c,&count,hits,0,3,0,0,0);CHECK(count==1);double oldx=c[0].x,oldy=c[0].y;sf_static_surface_point(c,hits,3,0,0);CHECK(c[0].x==hits[0].x&&c[0].y==hits[0].y);CHECK(c[0].x!=oldx||c[0].y!=oldy);CHECK(c[0].n==3);
 count=0;sf_add_cluster_min(c,&count,hits,0,1,0,0,0);CHECK(count==1);count=0;sf_add_cluster_min(c,&count,hits,0,1,0,0,1);CHECK(count==0);
 static obj_scene scene;obj_scene_fuse(&scene,&cfg,&f,NULL,NULL,&v,&p,n);CHECK(scene.target_odom_valid&&scene.measurement_v2.valid);CHECK(!strcmp(scene.source,"STEREO_COARSE"));CHECK(fabs(scene.target_odom_x-(.1265+.8))<1e-8);CHECK(scene.measurement_v2.source_us==n&&!scene.measurement_v2.scan_us);
 /* 当前双目支持的长箱面不能仅按墙面否决。 */
 static struct scan_frame scan;wall(&scan,n,1);v.latest_scan=&scan;v.has_scan=1;f=frame(n,"box",.8735);memset(&scene,0,sizeof scene);obj_scene_fuse(&scene,&cfg,&f,NULL,NULL,&v,&p,n);CHECK(scene.target_odom_valid&&scene.measurement_v2.valid);CHECK(!strcmp(scene.source,"RADAR_VISUAL_STEREO"));CHECK(fabs(scene.target_odom_x-1)<.01&&fabs(scene.target_odom_y)<.05);
 /* 同一墙面不能替代低水瓶的近距双目。 */
 f=frame(n,"bottle",.3);memset(&scene,0,sizeof scene);obj_scene_fuse(&scene,&cfg,&f,NULL,NULL,&v,&p,n);CHECK(scene.target_odom_valid&&scene.measurement_v2.valid);CHECK(!strcmp(scene.source,"STEREO_COARSE"));CHECK(fabs(scene.target_odom_x-.4265)<.01);
 /* 缺少前景深度时仍排除墙面，不全局关闭该判断。 */
 f=frame(n,"box",.8735);f.objects[0].depth_valid=0;memset(&scene,0,sizeof scene);obj_scene_fuse(&scene,&cfg,&f,NULL,NULL,&v,&p,n);CHECK(!scene.target_odom_valid&&!scene.measurement_v2.valid);
 printf("PASS %u actual STATIC fusion checks: no-lidar stereo, current-depth-supported long box face, real surface return instead of centroid, sparse static admission, background wall retained, low-bottle wall fallback, source ages; FOLLOW algorithms unchanged\n",checks);return 0;}
