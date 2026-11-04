/* 用回调统计传输调用次数，测试期间使用设备桩。 */
#define main previous_contract_main
#include "test_real_contract.c"
#undef main
#include "lab_real_session.h"
#include "continuous_policy.h"
static void commissioning_limits(void){
 fixture f;init(&f,0);lab_real_options o;lab_real_options_default(&o);o.mode=LAB_REAL_SHADOW;o.mapping_shadow=1;
 lab_live_source_binding in={&f,&source_ops};lab_real_session session;
 CHECK(lab_real_session_open(&session,&o,&f.context,&in)==LAB_OK);
 CHECK(session.options.localization.max_pose_age_us==800000&&session.options.localization.max_interpolation_gap_us==350000);
 CHECK(CONT_TIME_UNCERTAINTY_US==250000);
 CHECK(!session.options.localization.allow_odometry_fallback); /* 仿真默认条件保持严格。 */
 CHECK(o.source.max_scan_age_us==250000&&o.source.max_odometry_age_us==250000);
 CHECK(!session.options.enable_drive);lab_real_session_close(&session);
 puts("PASS commissioning match-age=800ms interpolation=350ms uncertainty=250ms; no drive enabled, simulation fallback opt-in only");
}
static void calibration_gate(void){fixture f;init(&f,1);events(&f,0,1);f.events[1].data.odometry.calibrated=0;
 lab_real_runtime*r=open_run(&f,LAB_REAL_NAVIGATION,1);CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);
 CHECK(lab_real_runtime_enable(r,1,50000)==LAB_AGAIN);CHECK(!f.arm&&!f.apply&&!strcmp(view(r).reason,"UNCALIBRATED_ARM_DENIED"));
 events(&f,100000,2);CHECK(lab_real_runtime_tick(r,100000)==LAB_AGAIN);CHECK(!f.arm&&!f.apply);
 CHECK(lab_real_runtime_enable(r,1,100000)==LAB_OK);CHECK(lab_real_runtime_tick(r,150000)==LAB_OK&&f.apply==1);
 CHECK(lab_real_runtime_tick(r,500000)==LAB_AGAIN);CHECK(!view(r).enabled&&f.stop==1);
 events(&f,550000,3);CHECK(lab_real_runtime_tick(r,550000)==LAB_AGAIN);CHECK(f.apply==1&&!view(r).enabled);
 CHECK(lab_real_runtime_cancel(r,550000)==LAB_OK);lab_real_runtime_close(r);free(f.events);
 puts("PASS uncalibrated ARM denied, no implicit arming, stale data stop and no automatic restart");
}
static void mount_gate(void){fixture f;init(&f,1);events(&f,0,1);lab_real_runtime_config c;lab_real_runtime_config_default(&c);c.mode=LAB_REAL_NAVIGATION;c.allow_hardware_output=1;c.geometry_verification_required=1;c.geometry_verified=0;
 lab_real_bindings b={.sensors={&f,&source_ops},.localization={&f,&loc_ops},.chassis={&f,&chassis_ops},.navigation={&f,goal,cancel,navstep,reset}};lab_real_runtime*r=NULL;CHECK(lab_real_runtime_open(&r,&c,&f.context,&b)==LAB_OK);
 CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);CHECK(lab_real_runtime_enable(r,1,50000)==LAB_AGAIN);CHECK(!strcmp(view(r).reason,"LIDAR_OFFSET_NOT_VERIFIED")&&!f.arm&&!f.apply);lab_real_runtime_close(r);free(f.events);puts("PASS unverified lidar offsets block ARM at runtime independently of phone UI");
 init(&f,1);events(&f,0,1);c.geometry_verified=1;CHECK(lab_real_runtime_open(&r,&c,&f.context,&b)==LAB_OK);CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);CHECK(lab_real_runtime_enable(r,1,50000)==LAB_OK&&f.arm==1);CHECK(lab_real_runtime_tick(r,100000)==LAB_OK&&f.apply==1);CHECK(view(r).enabled);lab_real_runtime_close(r);free(f.events);puts("PASS verified mount plus calibrated fresh localization enables hardware backend path; transport callbacks are counters only");
}
static void cli(int argc,char**argv){CHECK(argc==5);lab_real_options o;char error[256];
 char*args[]={"test","--mode","mapping-navigation","--mpu-snapshot","TEST_FIXTURE","--timing-profile",argv[1],"--mpu-odometry",argv[2],"--mpu-imu",argv[3],"--duration-s","0"};
 CHECK(lab_real_options_parse(&o,13,args,error,sizeof(error)));CHECK(o.mode==LAB_REAL_NAVIGATION&&o.mapping_shadow&&!o.enable_drive&&o.duration_us==UINT64_MAX);
 char*enabled[]={"test","--mode","mapping-navigation","--mpu-snapshot","TEST_FIXTURE","--timing-profile",argv[1],"--mpu-odometry",argv[2],"--mpu-imu",argv[3],"--enable-drive","--chassis-endpoint","ncv1://eth0/192.168.0.2:5012?config_id=806367234"};
 CHECK(lab_real_options_parse(&o,14,enabled,error,sizeof(error)));CHECK(o.enable_drive&&o.mapping_shadow);
 char*measured[16];for(unsigned i=0;i<14;i++)measured[i]=enabled[i];measured[14]="--passage-profile";measured[15]=argv[4];CHECK(lab_real_options_parse(&o,16,measured,error,sizeof(error)));CHECK(o.enable_drive&&o.passage.mount_verified&&fabs(o.passage.length_m-.333)<1e-12&&fabs(o.localization.mount_x_m+.0135)<1e-12&&o.localization.mount_y_m==0&&o.localization.mount_yaw_rad==0);
 char*bad[]={"test","--mode","mapping-navigation","--enable-drive"};CHECK(!lab_real_options_parse(&o,4,bad,error,sizeof(error)));
 puts("PASS snapshot live-navigation CLI, preview default, unlimited mapping and explicit hardware configuration");
}
static void continuous_drive(void){
 fixture f;init(&f,1);events(&f,0,1);lab_real_runtime*r=open_run(&f,LAB_REAL_NAVIGATION,1);
 CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);
 f.loc.state=LAB_REAL_LOC_DEGRADED;f.loc.odometry_fallback=1;f.push_wait=1;
 events(&f,300000,2);f.loc.pose_time_us=300000;f.loc.last_matched_scan_us=0;
 CHECK(lab_real_runtime_tick(r,300000)==LAB_AGAIN);CHECK(!strcmp(view(r).reason,"HARDWARE_DISABLED"));CHECK(!f.arm&&!f.apply);
 CHECK(lab_real_runtime_enable(r,1,300000)==LAB_OK);CHECK(lab_real_runtime_tick(r,350000)==LAB_OK&&f.apply==1);
 CHECK(view(r).localization.state==LAB_REAL_LOC_DEGRADED);
 f.loc.odometry_fallback=0;CHECK(lab_real_runtime_tick(r,360000)==LAB_AGAIN);CHECK(f.stop==1&&!view(r).enabled);
 f.loc.odometry_fallback=1;events(&f,400000,3);f.loc.pose_time_us=400000;CHECK(lab_real_runtime_tick(r,400000)==LAB_AGAIN);
 CHECK(lab_real_runtime_enable(r,1,400000)==LAB_OK);CHECK(lab_real_runtime_tick(r,450000)==LAB_OK&&f.apply==2);
 CHECK(lab_real_runtime_tick(r,700000)==LAB_AGAIN);CHECK(f.stop==2&&!view(r).enabled);
 lab_real_runtime_close(r);free(f.events);puts("PASS bounded degraded-mode marker permits explicit navigation; invalid state and stale raw data still stop; no physical motors");
}
int main(int argc,char**argv){CHECK(!previous_contract_main());commissioning_limits();calibration_gate();mount_gate();continuous_drive();cli(argc,argv);printf("PASS navigation drive gates checks=%u; SYNTHETIC test counters only\n",checks);return 0;}
