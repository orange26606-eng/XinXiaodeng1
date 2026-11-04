#include "object_dual_task.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"CHECK line %d: %s\n",__LINE__,#x);assert(x);}}while(0)
static lab_real_runtime_view view(uint64_t now,double x){lab_real_runtime_view v={0};v.context=(lab_real_context){LAB_REAL_TEST_FIXTURE,LAB_REAL_MONOTONIC,44,1};v.has_odometry=1;v.odometry_time_us=now;v.raw_odometry.quality=LAB_POSE_VALID;v.raw_odometry.x_m=x;return v;}
static obj_frame frame(uint64_t now,unsigned id,int cls,const char*label){obj_frame f={0};f.epoch=11;f.frame_id=id;f.frame_us=f.generated_us=now;f.count=1;f.selected=10;obj_detection*d=f.objects;d->track_id=10;d->class_id=d->task_class_id=d->observed_class_id=cls;d->observed=d->bbox_valid=d->bearing_valid=1;d->eye=d->geometry_eye=d->depth_eye=0;d->binding_generation=1;d->observation_frame_id=id;d->observation_time_us=now;d->bbox[0]=430;d->bbox[1]=400;d->bbox[2]=530;d->bbox[3]=700;d->score=.11;strcpy(d->label,label);strcpy(d->tracking,"TRACKING");return f;}
static void color(obj_detection*d,const char*c,uint64_t now,unsigned id){d->attributes.color_valid=1;strcpy(d->attributes.color,c);d->attributes.color_source_us=now;d->attributes.color_frame_id=id;}
static void geometry(object_dual_task*t,int verified){t->visual_geometry.loaded=t->visual_geometry.configured=1;t->visual_geometry.verified=verified;t->visual_geometry.x=.1265;}
static void start(object_dual_task*t,unsigned mode,uint64_t now){CHECK(object_dual_task_prepare(t,mode,11,10,.20,1));geometry(t,1);CHECK(object_dual_task_start(t,mode,11,10,now));}
static void depth(obj_detection*d,uint64_t now,unsigned id,double z){d->depth_valid=1;d->depth_time_us=now;d->depth_frame_id=id;d->depth_eye=d->geometry_eye=0;d->x=d->y=0;d->z=z;d->error=.02;}
static void big(obj_detection*d){d->bbox[0]=0;d->bbox[1]=0;d->bbox[2]=960;d->bbox[3]=1080;}
static void tick(object_dual_task*t,obj_frame*f,lab_real_runtime_view*v,uint64_t now,lab_real_command*c){CHECK(object_dual_task_tick(t,NULL,f,v,now,c));}
int main(int argc,char**argv){(void)argc;(void)argv;object_dual_task t;lab_real_command c;uint64_t n=1000000;lab_real_runtime_view v=view(n,0);obj_frame f=frame(n,1,3,"bottle");
 CHECK(!object_dual_task_prepare_search_attributes(&t,"bottle","bogus","",.2,1));CHECK(!object_dual_task_prepare_search_attributes(&t,"bottle","red","giant",.2,1));CHECK(!object_dual_task_prepare_search_attributes(&t,"bottle|box","red","",.2,1));
 CHECK(object_dual_task_prepare_search_attributes(&t,"bottle","red","",.2,1));CHECK(object_dual_task_start(&t,1,0,0,n));f.count=2;f.objects[1]=f.objects[0];f.objects[1].track_id=20;color(f.objects,"blue",n,1);color(f.objects+1,"red",n,1);tick(&t,&f,&v,n,&c);CHECK(t.track_id==20&&c.linear_m_s>0&&t.search_bound);f.objects[1].attributes.color_valid=0;tick(&t,&f,&v,n+10000,&c);CHECK(t.track_id==20);f.objects[1].observed=0;tick(&t,&f,&v,n+20000,&c);CHECK(t.track_id==20);
 CHECK(object_dual_task_prepare_search_attributes(&t,"box","red","large",.2,1));CHECK(object_dual_task_start(&t,1,0,0,n));f=frame(n,1,4,"box");f.count=2;f.objects[1]=f.objects[0];f.objects[1].track_id=20;for(int i=0;i<2;i++){color(f.objects+i,"red",n,1);f.objects[i].attributes.size_valid=1;strcpy(f.objects[i].attributes.size,"medium");f.objects[i].attributes.width_m=.25+i*.1;f.objects[i].attributes.height_m=.2;}tick(&t,&f,&v,n,&c);CHECK(t.track_id==20&&c.linear_m_s>0);
 /* 没有扫描、地图或超声波时，低水瓶仍可触发双目视觉保持。 */
 start(&t,1,n);f=frame(n,1,3,"bottle");depth(f.objects,n,1,.24);tick(&t,&f,&v,n,&c);CHECK(t.near_hold&&t.near_metric&&c.linear_m_s==0&&c.angular_rad_s==0&&!t.arrived);CHECK(t.visual_dock.confirmations==1);tick(&t,&f,&v,n,&c);CHECK(t.visual_dock.confirmations==1&&!t.arrived);object_route_target_v2 route;CHECK(object_dual_task_route_v2(&t,&v,n,&route)&&route.zero_required);
 f=frame(n+50000,2,3,"bottle");depth(f.objects,n+50000,2,.24);v=view(n+50000,0);tick(&t,&f,&v,n+50000,&c);CHECK(t.arrived&&t.near_hold&&!t.active&&!strcmp(t.reason,"STATIC_VISUAL_SURFACE_ARRIVAL"));
 /* 单次错误近距后，两笔真实远距可解除未完成任务的暂定保持。 */
 start(&t,1,n);f=frame(n,1,4,"box");depth(f.objects,n,1,.24);v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.near_hold&&!t.arrived);
 f=frame(n+50000,2,4,"box");depth(f.objects,n+50000,2,1.2);v=view(n+50000,0);tick(&t,&f,&v,n+50000,&c);CHECK(t.near_hold&&!t.arrived);
 f=frame(n+100000,3,4,"box");depth(f.objects,n+100000,3,1.2);v=view(n+100000,0);tick(&t,&f,&v,n+100000,&c);CHECK(!t.near_hold&&!t.near_latch&&!t.arrived&&c.linear_m_s>0);
 /* 近期近表面优先于较新但更远的双目发布，不重标两数据源时刻，也不重复计数同一扫描。
 */
 start(&t,1,n);f=frame(n+100000,1,4,"box");depth(f.objects,n+100000,1,2);v=view(n+100000,0);tick(&t,&f,&v,n+100000,&c);CHECK(!t.near_hold);
 static obj_scene near_scene;memset(&near_scene,0,sizeof near_scene);object_target_observation_v2*near=&near_scene.measurement_v2;near->valid=near->surface_valid=near->mount_verified=1;near->kind=OBJECT_MEASURE_RADAR_VISUAL;near->epoch=11;near->track_id=10;near->binding_generation=1;near->context=v.context;near->source_us=near->scan_us=n+50000;near->image_us=n+100000;near->frame_id=1;near->body_x_m=near->odom_x_m=.3;near->surface_gap_m=.1335;near->error_m=.03;strcpy(near->source,"RADAR_VISUAL");
 CHECK(object_dual_task_tick(&t,&near_scene,&f,&v,n+100000,&c));CHECK(t.near_hold&&c.linear_m_s==0&&t.visual_dock.confirmations==1&&t.visual_dock.metric_source_us==n+50000);
 CHECK(object_dual_task_tick(&t,&near_scene,&f,&v,n+110000,&c));CHECK(!t.arrived&&t.visual_dock.confirmations==1);
 /* 右眼深度使用右眼的物理坐标原点。 */
 start(&t,1,n);f=frame(n,1,3,"bottle");depth(f.objects,n,1,.18);f.objects[0].geometry_eye=f.objects[0].depth_eye=f.objects[0].eye=1;v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.near_hold&&!t.near_metric&&c.linear_m_s==0&&!strcmp(t.visual_dock.source,"STEREO_RIGHT_ORIGIN_UNKNOWN_COARSE"));
 start(&t,1,n);t.visual_geometry.extrinsics_valid=1;t.visual_geometry.stereo_R[0]=t.visual_geometry.stereo_R[4]=t.visual_geometry.stereo_R[8]=1;t.visual_geometry.stereo_T[0]=-.035;
 f=frame(n,1,3,"bottle");depth(f.objects,n,1,.24);f.objects[0].geometry_eye=f.objects[0].depth_eye=f.objects[0].eye=1;v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.near_hold&&t.near_metric&&t.visual_dock.metric_world_y<-.03);CHECK(fabs(t.visual_dock.metric_world_x-.3665)<.001);
 /* 加载实际随包右眼到左眼几何，不修改测试输入。 */
 start(&t,1,n);CHECK(object_dual_task_geometry_load(&t,"runtime/objects.conf"));CHECK(t.visual_geometry.extrinsics_valid&&!t.visual_geometry.verified&&t.visual_geometry.mono_valid[0]&&t.visual_geometry.mono_valid[1]);
 f=frame(n,1,3,"bottle");depth(f.objects,n,1,.18);f.objects[0].geometry_eye=f.objects[0].depth_eye=f.objects[0].eye=1;v=view(n,0);tick(&t,&f,&v,n,&c);
 double expected_left_z=t.visual_geometry.stereo_R[2]*(-t.visual_geometry.stereo_T[0])+t.visual_geometry.stereo_R[5]*(-t.visual_geometry.stereo_T[1])+t.visual_geometry.stereo_R[8]*(.18-t.visual_geometry.stereo_T[2]);
 CHECK(t.near_hold&&!t.near_metric&&fabs(t.visual_dock.metric_world_x-(t.visual_geometry.x+expected_left_z))<1e-8);
 /* 首次近距后箱框部分裁边或丢失时，不应开始旋转搜索。 */
 start(&t,1,n);f=frame(n,1,4,"box");depth(f.objects,n,1,.24);v=view(n,0);tick(&t,&f,&v,n,&c);f.objects[0].observed=0;f.frame_us=f.generated_us=n+50000;v=view(n+50000,0);tick(&t,&f,&v,n+50000,&c);CHECK(t.near_hold&&c.linear_m_s==0&&c.angular_rad_s==0&&!strcmp(t.state,"VERIFY_ARRIVAL"));
 object_dual_task_cancel(&t,"USER_CANCEL");CHECK(!t.near_hold&&!object_dual_task_tick(&t,NULL,&f,&v,n+60000,&c));
 /* 远处大箱子即使满屏或裁边，也不当作近目标。 */
 start(&t,1,n);f=frame(n,1,4,"box");big(f.objects);depth(f.objects,n,1,2);v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(!t.near_hold&&!t.arrived&&c.linear_m_s>0);f=frame(n+100000,2,4,"box");big(f.objects);depth(f.objects,n+100000,2,1.9);v=view(n+100000,.1);tick(&t,&f,&v,n+100000,&c);CHECK(!t.near_hold&&!t.arrived);
 /* 安装参数未核验时，结果标为粗略停靠。 */
 start(&t,1,n);geometry(&t,0);f=frame(n,1,3,"bottle");depth(f.objects,n,1,.24);v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.near_hold&&!t.near_metric);f=frame(n+50000,2,3,"bottle");depth(f.objects,n+50000,2,.24);v=view(n+50000,0);tick(&t,&f,&v,n+50000,&c);CHECK(t.arrived&&!strcmp(t.reason,"STATIC_VISUAL_COARSE_DOCKED"));
 /* 经标定的里程计与图像放大可在无深度和雷达时使水瓶停车。 */
 start(&t,1,n);f=frame(n,1,3,"bottle");v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(!t.near_hold&&c.linear_m_s>0);f=frame(n+300000,2,3,"bottle");f.objects[0].bbox[0]=405;f.objects[0].bbox[2]=555;f.objects[0].bbox[1]=325;f.objects[0].bbox[3]=775;v=view(n+300000,.1);tick(&t,&f,&v,n+300000,&c);CHECK(t.near_hold&&!t.near_metric&&c.linear_m_s==0&&!strcmp(t.visual_dock.source,"MONOCULAR_SCALE_ODOM_COARSE"));CHECK(fabs(t.visual_dock.gap_m-.16)<.01);
 /* 同一旧图像重复读取时，框扩张证据保持原值。 */
 start(&t,1,n);f=frame(n,1,3,"bottle");v=view(n,0);tick(&t,&f,&v,n,&c);big(f.objects);v=view(n+300000,.1);tick(&t,&f,&v,n+300000,&c);CHECK(!t.near_hold&&t.visual_dock.count==1);
 /* 换眼重置尺度历史，不将另一眼相同像素尺寸当作目标变大。 */
 start(&t,1,n);f=frame(n,1,4,"box");v=view(n,0);tick(&t,&f,&v,n,&c);f=frame(n+300000,2,4,"box");big(f.objects);f.objects[0].eye=1;v=view(n+300000,.1);tick(&t,&f,&v,n+300000,&c);CHECK(!t.near_hold&&t.visual_dock.count==1);
 /* 忽略超声波零值，新的正近距值支持视觉接近证据。 */
 start(&t,1,n);f=frame(n,1,4,"box");big(f.objects);v=view(n,0);v.feedback.time_us=n;v.feedback.sonar_valid=v.feedback.sonar_fresh=1;v.feedback.sonar_mm[0]=0;tick(&t,&f,&v,n,&c);CHECK(!t.near_hold);f.frame_id=f.objects[0].observation_frame_id=2;f.frame_us=f.objects[0].observation_time_us=n+50000;v=view(n+50000,0);v.feedback.time_us=n+50000;v.feedback.sonar_valid=v.feedback.sonar_fresh=1;v.feedback.sonar_mm[0]=150;tick(&t,&f,&v,n+50000,&c);CHECK(t.near_hold&&!t.near_metric&&c.linear_m_s==0);
 /* 没有采集位姿的旧深度不能重标为当前。 */
 start(&t,1,n);f=frame(n,5,4,"box");depth(f.objects,n-300000,1,.24);v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(!t.near_hold&&!t.visual_dock.gap_valid);
 /* 真实历史帧深度用实测行程补偿。 */
 start(&t,1,n);f=frame(n,1,4,"box");v=view(n,0);tick(&t,&f,&v,n,&c);f=frame(n+300000,2,4,"box");depth(f.objects,n,1,.5);v=view(n+300000,.1);tick(&t,&f,&v,n+300000,&c);CHECK(t.visual_dock.gap_valid&&fabs(t.visual_dock.gap_m-.36)<.01&&t.visual_dock.metric_source_us==n);
 /* 缺少地面几何参数时，全屏框只能提供图像证据。 */
 start(&t,1,n);CHECK(!object_dual_task_geometry_load(&t,NULL));f=frame(n,1,4,"box");big(f.objects);v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(!t.near_hold&&!t.visual_dock.gap_valid);
 /* 地面求交使用已核验的相机安装高度。 */
 start(&t,1,n);t.visual_geometry.verified=1;t.visual_geometry.z=.10;t.visual_geometry.mono_valid[0]=1;t.visual_geometry.fx[0]=t.visual_geometry.fy[0]=800;t.visual_geometry.cx[0]=480;t.visual_geometry.cy[0]=540;
 f=frame(n,1,3,"bottle");f.objects[0].bbox[3]=940;v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.near_hold&&!t.near_metric&&!strcmp(t.visual_dock.source,"GROUND_BOX_CONTACT_APPROX"));
 /* 目标底部裁边时仍可按实际行程推进之前的地面点。 */
 start(&t,1,n);t.visual_geometry.verified=1;t.visual_geometry.z=.20;t.visual_geometry.mono_valid[0]=1;t.visual_geometry.fx[0]=t.visual_geometry.fy[0]=800;t.visual_geometry.cx[0]=480;t.visual_geometry.cy[0]=540;
 f=frame(n,1,3,"bottle");f.objects[0].bbox[3]=940;v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(!t.near_hold&&t.visual_dock.gap_valid);
 f=frame(n+300000,2,3,"bottle");f.objects[0].bbox[3]=1080;v=view(n+300000,.15);tick(&t,&f,&v,n+300000,&c);CHECK(t.near_hold&&c.linear_m_s==0&&c.angular_rad_s==0);
 /* 下方恢复无距离箱子的诊断输入。 */
 start(&t,1,n);f=frame(n,1,4,"box");big(f.objects);v=view(n,0);tick(&t,&f,&v,n,&c);
 FILE*geo=fopen("fixture-geometry.conf","wb");CHECK(geo);fputs("camera_to_body_verified=1\ncamera_x_m=.1265\ncamera_y_m=0\ncamera_z_m=.10\ncamera_roll_deg=0\ncamera_pitch_deg=0\ncamera_yaw_deg=0\nmono_eye0_file=fixture-mono.conf\n",geo);fclose(geo);
 geo=fopen("fixture-mono.conf","wb");CHECK(geo);fputs("calibration_enabled=1\neye=0\nimage_width=960\nimage_height=1080\nfx=800\nfy=800\ncx=480\ncy=540\n",geo);fclose(geo);
 CHECK(object_dual_task_geometry_load(&t,"fixture-geometry.conf"));CHECK(t.visual_geometry.verified&&t.visual_geometry.z==.10&&t.visual_geometry.mono_valid[0]);
 geo=fopen("fixture-geometry.conf","wb");CHECK(geo);fputs("camera_to_body_verified=1\ncamera_x_m=.1265\ncamera_y_m=0\ncamera_z_m=0\ncamera_roll_deg=0\ncamera_pitch_deg=0\ncamera_yaw_deg=0\nmono_eye0_file=fixture-mono.conf\n",geo);fclose(geo);CHECK(object_dual_task_geometry_load(&t,"fixture-geometry.conf"));CHECK(!strcmp(t.visual_geometry.reason,"GROUND_HEIGHT_UNAVAILABLE"));
 CHECK(object_dual_task_prepare_search_attributes(&t,"bottle","red","",.2,1));CHECK(object_dual_task_start(&t,1,0,0,n));f=frame(n,1,3,"bottle");v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(!strcmp(t.reason,"STATIC_SEARCH_COLOR_UNAVAILABLE")&&c.angular_rad_s==.30&&t.search_class_candidates==1);
 color(f.objects,"blue",n,1);tick(&t,&f,&v,n,&c);CHECK(!strcmp(t.reason,"STATIC_SEARCH_COLOR_NOT_MATCHED")&&t.search_color_candidates==0);
 CHECK(object_dual_task_prepare_search_attributes(&t,"bottle","red","large",.2,1));CHECK(object_dual_task_start(&t,1,0,0,n));color(f.objects,"red",n,1);tick(&t,&f,&v,n,&c);CHECK(!strcmp(t.reason,"SIZE_PROBE_ALIGN_CURRENT_CANDIDATE")&&t.probe_active&&!t.search_bound&&c.linear_m_s==0&&c.angular_rad_s==0);f.objects[0].attributes.size_valid=1;f.objects[0].attributes.height_m=.2;strcpy(f.objects[0].attributes.size,"medium");tick(&t,&f,&v,n,&c);CHECK(!strcmp(t.reason,"STATIC_SEARCH_SINGLE_SIZE_NOT_MATCHED"));
 /* 右侧大小未知的真实候选仍应被观测，不盲目向左搜索，也不误标为尺寸匹配。
 */
 CHECK(object_dual_task_prepare_search_attributes(&t,"box","red","large",.2,1));CHECK(object_dual_task_start(&t,1,0,0,n));f=frame(n,1,4,"box");color(f.objects,"red",n,1);f.objects[0].bearing=-.5;v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.probe_active&&!t.search_bound&&t.probe_track_id==10&&c.linear_m_s==0&&c.angular_rad_s<0);CHECK(object_dual_task_route_v2(&t,&v,n,&route));CHECK(!route.static_search_left&&route.static_bearing_approach&&route.epoch==11&&route.track_id==10&&!route.valid&&!route.goal_valid);
 f.objects[0].observed=0;v=view(n+100000,0);tick(&t,&f,&v,n+100000,&c);CHECK(t.probe_active&&!t.probe_current&&c.linear_m_s==0&&c.angular_rad_s<0&&t.probe_source_us==n);
 f=frame(n+150000,2,4,"box");color(f.objects,"red",n+150000,2);f.objects[0].attributes.size_valid=1;f.objects[0].attributes.width_m=.5;f.objects[0].attributes.height_m=.4;strcpy(f.objects[0].attributes.size,"large");v=view(n+150000,0);tick(&t,&f,&v,n+150000,&c);CHECK(t.search_bound&&!t.probe_active&&t.track_id==10&&c.linear_m_s>0);
 /* 大小未知的近候选应保持停车，不将尺寸任务标作完成。 */
 CHECK(object_dual_task_prepare_search_attributes(&t,"bottle","red","large",.2,1));CHECK(object_dual_task_start(&t,1,0,0,n));geometry(&t,0);f=frame(n,1,3,"bottle");color(f.objects,"red",n,1);depth(f.objects,n,1,.24);v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.probe_active&&t.near_hold&&!t.search_bound&&!t.arrived&&c.linear_m_s==0&&c.angular_rad_s==0);
 /* 恢复期望的无距离诊断。 */start(&t,1,n);f=frame(n,1,4,"box");big(f.objects);v=view(n,0);tick(&t,&f,&v,n,&c);
 char out[24000];size_t written;CHECK(object_dual_task_json(&t,n,out,sizeof out,&written));CHECK(strstr(out,"\"near_hold\":false")&&strstr(out,"LARGE_IMAGE_ALONE_NOT_ARRIVAL"));
 /* 新里程计上下文使历史世界位置、试探偏航及近距证据失效。 */
 start(&t,1,n);f=frame(n,1,3,"bottle");depth(f.objects,n,1,.24);v=view(n,0);tick(&t,&f,&v,n,&c);CHECK(t.near_hold);v=view(n+50000,0);v.context.origin_epoch++;tick(&t,&f,&v,n+50000,&c);CHECK(!t.near_hold&&!t.probe_active&&!t.visual_dock.initialized&&c.linear_m_s==0&&c.angular_rad_s==0);
 printf("PASS %u production static-control checks: attributes, relative metric size, no-lidar bottle, visible box surface, far large box, clipped loss hold, source/eye/stereo-pose continuity, zero-sonar, explicit cancel; no device or motor IO\n",checks);return 0;}
