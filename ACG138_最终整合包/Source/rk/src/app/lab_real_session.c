#include "lab_real_session.h"
#include "lab_real_navigation.h"
#include "lab_real_mapfile.h"
#include "lab_real_screen.h"
#include "lab_real_snapshot.h"
#include "localization_range.h"
#include <errno.h>
#include <math.h>
#include "continuous_policy.h"
#include "s2_navigation_limits.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
void lab_real_options_default(lab_real_options *o){if(!o)return;memset(o,0,sizeof(*o));lab_live_source_config_default(&o->source);lab_real_mpu_config_default(&o->mpu);lab_localization_config_default(&o->localization);o->mode=LAB_REAL_READONLY;o->duration_us=60000000;snprintf(o->output,sizeof(o->output),"HDMI-A-2");o->source.read_existing_odometry=0;snprintf(o->source.allowed_source_ip,sizeof(o->source.allowed_source_ip),"127.0.0.1");o->goal.id=1;passage_config_default(&o->passage);lab_real_time_profile_default(&o->timing);o->scan_time_port=6211;}
static int copy(char *dst,size_t size,const char *s){if(!s||strlen(s)>=size||!s[0])return 0;memcpy(dst,s,strlen(s)+1);return 1;}
static int number(const char *s,double *out){char *end=NULL;errno=0;double x=strtod(s,&end);if(errno||end==s||*end||!isfinite(x))return 0;*out=x;return 1;}
static int integer(const char *s,uint64_t *out){if(!s||s[0]=='-'||s[0]=='+')return 0;char *e=NULL;errno=0;unsigned long long v=strtoull(s,&e,10);if(errno||e==s||*e)return 0;*out=v;return 1;}
static uint64_t snapshot_identity(const lab_real_mpu_config *c){
#ifdef LAB_REAL_SNAPSHOT_SOURCE
 return lab_real_snapshot_identity(c);
#else
 (void)c;return 0;
#endif
}
int lab_real_options_parse(lab_real_options *o,int argc,char **argv,char *error,size_t bytes){
 if(!o)return 0;lab_real_options_default(o);const char *bad="invalid arguments";int port_set=0,raw_mpu=0,mount_set=0;
 for(int i=1;i<argc;++i){const char *a=argv[i];
  if(!strcmp(a,"--mpu9250")){o->use_mpu9250=1;raw_mpu=1;continue;}
  if(!strcmp(a,"--enable-drive")){o->enable_drive=1;continue;}
  if(!strcmp(a,"--run")){o->run=1;continue;}if(!strcmp(a,"--preflight")){o->preflight=1;continue;}
  if(i+1>=argc){bad="missing option value";goto fail;}const char *v=argv[++i];uint64_t u=0;double d=0;
  if(!strcmp(a,"--mode")){o->mapping_shadow=0;if(!strcmp(v,"readonly"))o->mode=1;else if(!strcmp(v,"mapping"))o->mode=2;else if(!strcmp(v,"localization"))o->mode=3;else if(!strcmp(v,"shadow")){o->mode=4;o->mapping_shadow=0;}else if(!strcmp(v,"mapping-navigation")){o->mode=5;o->mapping_shadow=1;}else if(!strcmp(v,"mapping-shadow")){o->mode=4;o->mapping_shadow=1;}else{bad="mode must be readonly/mapping/localization/shadow/mapping-shadow; motor output not exposed";goto fail;}}
  else if(!strcmp(a,"--chassis-endpoint")){if(!copy(o->chassis_endpoint,sizeof(o->chassis_endpoint),v))goto fail;}
  else if(!strcmp(a,"--odom-shm")){if(!copy(o->source.odometry_path,sizeof(o->source.odometry_path),v))goto fail;o->source.read_existing_odometry=1;}
  else if(!strcmp(a,"--time-chain-profile")){if(!copy(o->timing_profile,sizeof(o->timing_profile),v))goto fail;o->timed=1;}
  else if(!strcmp(a,"--timed-odom")){if(!copy(o->timed_odometry_path,sizeof(o->timed_odometry_path),v))goto fail;o->timed=1;}
  else if(!strcmp(a,"--scan-time-port")){if(!integer(v,&u)||u>65535)goto fail;o->scan_time_port=(uint32_t)u;o->timed=1;}
  else if(!strcmp(a,"--mpu-snapshot")){
#ifdef LAB_REAL_SNAPSHOT_SOURCE
   if(!copy(o->mpu_snapshot,sizeof(o->mpu_snapshot),v))goto fail;o->use_mpu9250=1;
#else
   bad="this older build has no snapshot source; use snapshot-map release";goto fail;
#endif
  }
  else if(!strcmp(a,"--passage-profile")){if(!copy(o->passage_profile,sizeof(o->passage_profile),v))goto fail;}
  else if(!strcmp(a,"--mpu-bind")){if(!copy(o->mpu.bind_ip,sizeof(o->mpu.bind_ip),v))goto fail;}
  else if(!strcmp(a,"--mpu-peer")){if(!copy(o->mpu.peer_ip,sizeof(o->mpu.peer_ip),v))goto fail;}
  else if(!strcmp(a,"--mpu-odometry")){if(!copy(o->mpu.odom_config,sizeof(o->mpu.odom_config),v))goto fail;}
  else if(!strcmp(a,"--mpu-imu")){if(!copy(o->mpu.imu_config,sizeof(o->mpu.imu_config),v))goto fail;}
  else if(!strcmp(a,"--timing-profile")){if(!copy(o->mpu.timing_profile,sizeof(o->mpu.timing_profile),v))goto fail;}
  else if(!strcmp(a,"--calibration")){if(!copy(o->source.calibration_path,sizeof(o->source.calibration_path),v))goto fail;}
  else if(!strcmp(a,"--ls01-port")){if(!integer(v,&u)||u>65535)goto fail;o->source.ls01_port=(uint32_t)u;port_set=1;}
  else if(!strcmp(a,"--source-ip")){if(!copy(o->source.allowed_source_ip,sizeof(o->source.allowed_source_ip),v))goto fail;}
  else if(!strcmp(a,"--output")){if(!copy(o->output,sizeof(o->output),v))goto fail;}
  else if(!strcmp(a,"--load-map")){if(!copy(o->load_map,sizeof(o->load_map),v))goto fail;}
  else if(!strcmp(a,"--save-map")){if(!copy(o->save_map,sizeof(o->save_map),v))goto fail;}
  else if(!strcmp(a,"--preview")){if(!copy(o->preview,sizeof(o->preview),v))goto fail;}
  else if(!strcmp(a,"--parameter-version")){if(!integer(v,&u)||!u)goto fail;o->source.parameter_version=o->localization.parameter_version=u;}
  else if(!strcmp(a,"--duration-s")){if(!number(v,&d)||d<0||(d>0&&d<.1)||d>3600)goto fail;o->duration_us=d==0?UINT64_MAX:(uint64_t)(d*1e6);}
  else if(!strcmp(a,"--mount-x-m")){if(!number(v,&d)||fabs(d)>2)goto fail;mount_set=1;o->localization.mount_x_m=d;}
  else if(!strcmp(a,"--mount-y-m")){if(!number(v,&d)||fabs(d)>2)goto fail;mount_set=1;o->localization.mount_y_m=d;}
  else if(!strcmp(a,"--mount-yaw-deg")){if(!number(v,&d)||fabs(d)>360)goto fail;mount_set=1;o->localization.mount_yaw_rad=d*.017453292519943295;}
  else if(!strcmp(a,"--initial-x-m")){if(!number(v,&d))goto fail;o->has_initial|=1;o->initial.x_m=d;}
  else if(!strcmp(a,"--initial-y-m")){if(!number(v,&d))goto fail;o->has_initial|=2;o->initial.y_m=d;}
  else if(!strcmp(a,"--initial-yaw-deg")){if(!number(v,&d))goto fail;o->has_initial|=4;o->initial.yaw_rad=d*.017453292519943295;}
  else if(!strcmp(a,"--goal-x-m")){if(!number(v,&d))goto fail;o->has_goal|=1;o->goal.x_m=d;}
  else if(!strcmp(a,"--goal-y-m")){if(!number(v,&d))goto fail;o->has_goal|=2;o->goal.y_m=d;}
  else{bad="unknown option";goto fail;}
 }
 if(o->passage_profile[0]){char detail[160];if(mount_set){bad="use passage profile or mount CLI options, not both";goto fail;}if(!passage_config_load(&o->passage,o->passage_profile,detail,sizeof(detail))){bad="invalid or unreadable passage profile";goto fail;}
  o->localization.mount_x_m=o->passage.mount_x_m;o->localization.mount_y_m=o->passage.mount_y_m;o->localization.mount_yaw_rad=o->passage.mount_yaw_rad;
 }else{o->passage.mount_x_m=o->localization.mount_x_m;o->passage.mount_y_m=o->localization.mount_y_m;o->passage.mount_yaw_rad=o->localization.mount_yaw_rad;}
 if(o->use_mpu9250){
  if(raw_mpu&&o->mpu_snapshot[0]){bad="choose --mpu-snapshot FILE or --mpu9250 raw UDP, never both";goto fail;}
  if(!port_set)o->source.ls01_port=6212;
  if(o->source.ls01_port==5016){bad="timed scan cannot use FPGA port 5016";goto fail;}
  if(o->source.read_existing_odometry||o->source.calibration_path[0]){bad="MPU raw source cannot also use legacy shared odometry/calibration";goto fail;}
  if(o->mode>LAB_REAL_MAPPING&&!(o->mpu_snapshot[0]&&o->mapping_shadow)){bad="MPU candidate supports readonly/mapping only; autonomous output remains unavailable";goto fail;}
  if(o->mpu.timing_profile[0]){char profile[256];strcpy(profile,o->mpu.timing_profile);if(lab_real_mpu_profile(&o->mpu,profile)!=LAB_OK){bad="invalid timing assumption profile";goto fail;}}
  if((o->mode==LAB_REAL_MAPPING||o->mapping_shadow)&&!o->mpu.experimental){bad="MPU mapping requires explicit experimental timing profile; unknown errors are not silently overridden";goto fail;}
 }else if(o->mpu.timing_profile[0]||o->mpu.odom_config[0]||o->mpu.imu_config[0]){bad="MPU options require --mpu9250";goto fail;}
 if(o->duration_us==UINT64_MAX&&o->mode!=LAB_REAL_MAPPING&&o->mode!=LAB_REAL_READONLY&&!o->mapping_shadow){bad="unlimited duration is limited to read-only and mapping";goto fail;}
 if(o->run&&o->preflight){bad="--run and --preflight are exclusive";goto fail;}
 if(o->has_initial&&o->has_initial!=7){bad="provide all initial x/y/yaw fields";goto fail;}
 if(o->has_goal&&o->has_goal!=3){bad="provide both goal x/y fields";goto fail;}
 if(o->has_goal&&o->mode<LAB_REAL_SHADOW){bad="goals require shadow mode";goto fail;}
 if(!o->mapping_shadow&&o->mode>=LAB_REAL_LOCALIZATION&&(!o->load_map[0]||o->has_initial!=7)){bad="localization/shadow requires REAL map and explicit initial x/y/yaw";goto fail;}
 if(o->mapping_shadow&&(o->load_map[0]||o->has_initial)){bad="mapping-shadow starts a new local map; do not load map or supply fixed-map initial pose";goto fail;}
 if(o->mode==LAB_REAL_READONLY&&(o->load_map[0]||o->save_map[0]||o->has_initial)){bad="readonly does not map or accept initial pose";goto fail;}
 if(o->mode==LAB_REAL_MAPPING&&o->load_map[0]){bad="mapping starts a new map; use localization for loaded map";goto fail;}
 if(o->timed){
  if(o->use_mpu9250){bad="ODT1 timing chain and MPU9250 source are exclusive";goto fail;}
  if(o->mode>LAB_REAL_MAPPING){bad="declared time bounds are experimental: timed source currently permits readonly/mapping only";goto fail;}
  if(!o->timing_profile[0]){bad="new timing chain requires explicit --time-chain-profile";goto fail;}
  if(o->source.read_existing_odometry||o->source.calibration_path[0]){bad="timed source cannot mix legacy --odom-shm/--calibration; producer loads its own config";goto fail;}
  if(lab_real_time_profile_load(o->timing_profile,&o->timing)!=LAB_OK){bad="timing profile invalid or unreadable";goto fail;}
  if(o->timing.parameter_version!=o->source.parameter_version){bad="timing and runtime parameter_version mismatch";goto fail;}
  if(o->source.ls01_port&&(!o->scan_time_port||o->scan_time_port==o->source.ls01_port)){bad="LS01 and timing ports must be distinct and nonzero";goto fail;}
 }
 if(o->enable_drive&&(o->mode!=LAB_REAL_NAVIGATION||!o->chassis_endpoint[0]||!o->mpu_snapshot[0])){bad="drive requires mapping-navigation, explicit chassis endpoint and single-owner MPU snapshot";goto fail;}
 if(o->chassis_endpoint[0]&&!o->enable_drive){bad="endpoint requires explicit --enable-drive";goto fail;}
 if(!o->use_mpu9250&&!o->source.read_existing_odometry&&!o->source.ls01_port&&!o->timed_odometry_path[0]){bad="no explicit sensor input";goto fail;}
 if(o->use_mpu9250){o->mpu.declared_version=o->source.parameter_version;o->mpu.mount[0]=o->localization.mount_x_m;o->mpu.mount[1]=o->localization.mount_y_m;o->mpu.mount[2]=o->localization.mount_yaw_rad;
  uint64_t identity=o->mpu_snapshot[0]?snapshot_identity(&o->mpu):lab_real_mpu_identity(&o->mpu);if(!identity){bad="MPU parameter files missing or oversized";goto fail;}o->source.parameter_version=o->localization.parameter_version=identity;}
 if(error&&bytes)*error=0;return 1;
fail:if(error&&bytes)snprintf(error,bytes,"%s",bad);return 0;
}
void lab_real_usage(void){puts("lab-real[ -panel] --info | --help | [--preflight | --run] [options]\n --mode readonly|mapping|localization|shadow|mapping-shadow (default readonly)\n --odom-shm EXACT_SHARED_FILE --calibration OPTIONAL_ODOM_CONF\n --ls01-port 6210 --source-ip 127.0.0.1\n --time-chain-profile FILE --timed-odom NEW.odt --scan-time-port 6211\n --duration-s 60 --output HDMI-A-2 --preview NEW.ppm\n --save-map NEW.rmap | --load-map SAVED.rmap\n --initial-x-m X --initial-y-m Y --initial-yaw-deg YAW\n --goal-x-m X --goal-y-m Y (shadow or mapping-shadow)\n --passage-profile FILE (body, margins, local grid, measured lidar offsets)\n --mount-x-m X --mount-y-m Y --mount-yaw-deg YAW (exclusive with profile)\n --parameter-version N\n --mpu9250 [--mpu-bind 192.168.0.3 --mpu-peer 192.168.0.2]\n --mpu-odometry FILE --mpu-imu FILE --timing-profile CANDIDATE.conf\n --mpu-snapshot /run/acg138-mpu-service/mpu9250.snapshot (snapshot-map build)\n Snapshot source reads files, NEVER binds UDP5016; LT01 scan port6212.\n MPU raw UDP5016 + LT01 USB timing UDP6212; mapping-only experimental.\n--mode mapping-navigation: live map + screen goals; --enable-drive --chassis-endpoint ncv1://eth0/192.168.0.2:5012?config_id=655369985\n Explicit screen ARM additionally requires calibrated odometry, TRACKING and fresh sensors. No automatic arm or restart.\nNo --run means preflight, no source/socket/device opened.");}
lab_status lab_real_session_open(lab_real_session *s,const lab_real_options *o,const lab_real_context *ctx,const lab_live_source_binding *injected){
 if(!s||!o||!lab_real_context_valid(ctx)||o->mode<1||o->mode>5||o->mapping_shadow>1||(o->mapping_shadow&&(o->mode<LAB_REAL_SHADOW||o->load_map[0]||o->has_initial)))return LAB_ERR_ARGUMENT;memset(s,0,sizeof(*s));s->options=*o;lab_status r=LAB_OK;const char *open_stage="SOURCE_CREATE";
 if(injected){if(!injected->ops)return LAB_ERR_ARGUMENT;s->bindings.sensors=*injected;}
 else{
#ifdef LAB_REAL_POSIX_SOURCE
  if(o->timed){lab_real_timed_source_config tc={0};tc.legacy=o->source;tc.profile=o->timing;tc.scan_time_port=o->scan_time_port;snprintf(tc.timed_odometry_path,sizeof(tc.timed_odometry_path),"%s",o->timed_odometry_path);r=lab_real_timed_source_create(&s->bindings.sensors,&tc,ctx);}
  else if(o->mpu_snapshot[0]){
#ifdef LAB_REAL_SNAPSHOT_SOURCE
   r=lab_real_snapshot_source_create(&s->bindings.sensors,o->mpu_snapshot,&o->mpu,&o->source,ctx);
#else
   r=LAB_ERR_UNAVAILABLE;
#endif
  }
  else r=o->use_mpu9250?lab_real_mpu_source_create(&s->bindings.sensors,&o->mpu,&o->source,ctx):lab_live_source_create(&s->bindings.sensors,&o->source,ctx);
  s->own_source=r==LAB_OK;
#else
  r=LAB_ERR_UNAVAILABLE;
#endif
  if(r!=LAB_OK)goto fail;
 }
 lab_localization_config lc=o->localization;lc.mode=!o->mapping_shadow&&o->mode>=LAB_REAL_LOCALIZATION?LAB_REAL_FIXED_MAP:LAB_REAL_BUILD_MAP;
 /* 调试配置只放宽匹配年龄，不放宽原始传感器看门狗。 */
 lc.max_pose_age_us=CONT_MATCH_US;lc.max_interpolation_gap_us=CONT_INTERPOLATION_US;lc.allow_odometry_fallback=ctx->source==LAB_REAL_HARDWARE;s->options.localization=lc;
 fprintf(stderr,"CONTINUITY_PROFILE pose_us=%llu scan_us=%llu feedback_us=%llu command_ttl_us=%llu phone_lease_us=%llu match_quality=WEIGHTED deadreckon=WHILE_REAL_IMU_ENCODER_FRESH context_restart=AUTO timing=EXPERIMENTAL\n",(unsigned long long)CONT_ODOM_US,(unsigned long long)CONT_SCAN_US,(unsigned long long)CONT_FEEDBACK_US,(unsigned long long)CONT_COMMAND_US,(unsigned long long)CONT_PHONE_US);
 if(o->mode>=LAB_REAL_MAPPING){open_stage="LOCALIZATION_CREATE";r=lab_localization_create(&s->bindings.localization,&lc,ctx);if(r!=LAB_OK)goto fail;
  open_stage="LOCALIZATION_SENSOR_RANGE";r=lab_real_localization_set_sensor_range(&s->bindings.localization,o->passage.lidar_min_m,o->passage.lidar_max_m);if(r!=LAB_OK)goto fail;
  if(o->load_map[0]){lab_real_map_file f;r=lab_real_map_load(o->load_map,ctx,&lc,&f);if(r!=LAB_OK)goto fail;if(f.experimental_timing){lab_real_map_file_close(&f);r=LAB_ERR_READONLY;goto fail;}r=s->bindings.localization.ops->set_map(s->bindings.localization.user,&f.view);lab_real_map_file_close(&f);if(r!=LAB_OK)goto fail;}
 }
 lab_chassis_config cc;lab_chassis_config_default(&cc);cc.backend=LAB_CHASSIS_DRY_RUN;cc.transport=LAB_CHASSIS_TRANSPORT_NONE;cc.endpoint[0]=0;
 cc.max_linear_m_s=.40;cc.max_angular_rad_s=S2_MAX_YAW_RAD_S;cc.max_wheel_rpm=120;cc.direct_targets=1;cc.max_command_age_us=CONT_COMMAND_US;cc.receive_watchdog_us=CONT_COMMAND_US;
 cc.drive_sign[0]=1;cc.drive_sign[1]=-1;cc.drive_sign[2]=1;cc.drive_sign[3]=-1;
 if(o->enable_drive){cc.backend=LAB_CHASSIS_HARDWARE;cc.transport=LAB_CHASSIS_TRANSPORT_UDP;cc.protocol_version=1;snprintf(cc.endpoint,sizeof(cc.endpoint),"%s",o->chassis_endpoint);}
 open_stage="CHASSIS_CREATE";r=lab_chassis_create(&s->bindings.chassis,&cc,ctx);if(r!=LAB_OK)goto fail;
 if(o->mode>=LAB_REAL_SHADOW){open_stage="NAVIGATION_CREATE";r=lab_real_navigation_create_profile(&s->bindings.navigation,ctx,&s->bindings.localization,&lc,&o->passage);if(r!=LAB_OK)goto fail;}
 lab_real_runtime_config rc;lab_real_runtime_config_default(&rc);rc.mode=o->mode==LAB_REAL_NAVIGATION&&!o->enable_drive?LAB_REAL_SHADOW:o->mode;rc.allow_hardware_output=o->enable_drive;rc.geometry_verification_required=1;rc.geometry_verified=o->passage.mount_verified;rc.max_linear_m_s=.40;rc.max_angular_rad_s=S2_MAX_YAW_RAD_S;rc.max_scan_age_us=CONT_SCAN_US;rc.max_pose_age_us=CONT_ODOM_US;rc.control_session=((uint64_t)ctx->timeline_id<<32)|ctx->origin_epoch;open_stage="RUNTIME_CREATE";r=lab_real_runtime_open(&s->runtime,&rc,ctx,&s->bindings);if(r!=LAB_OK)goto fail;
 lab_real_runtime_view_get(s->runtime,&s->view);s->view.experimental_timing=(o->use_mpu9250&&o->mpu.experimental)||(o->timed&&(o->timing.fpga_delay_declared||o->timing.sdk_time_declared));return LAB_OK;
fail:fprintf(stderr,"SESSION_OPEN_DIAG stage=%s status=%d match_age_us=%llu control_pose_age_us=%llu scan_age_us=%llu enable_drive=%u mapping_shadow=%u\n",open_stage,(int)r,(unsigned long long)CONT_MATCH_US,(unsigned long long)CONT_ODOM_US,(unsigned long long)CONT_SCAN_US,o->enable_drive,o->mapping_shadow);lab_real_session_close(s);return r;
}
lab_status lab_real_session_tick(lab_real_session *s,uint64_t now){if(!s||!s->runtime)return LAB_ERR_ARGUMENT;if(!s->started_us)s->started_us=now;
 lab_status r=lab_real_runtime_tick(s->runtime,now);lab_real_runtime_view_get(s->runtime,&s->view);if(s->view.time_us>now)now=s->view.time_us;s->view.experimental_timing=(s->options.use_mpu9250&&s->options.mpu.experimental)||(s->options.timed&&(s->options.timing.fpga_delay_declared||s->options.timing.sdk_time_declared));
 if(s->options.has_initial&&!s->initial_sent&&s->view.has_odometry){lab_status a=s->bindings.localization.ops->set_initial_pose(s->bindings.localization.user,&s->options.initial,now);if(a!=LAB_OK)return a;s->initial_sent=1;}
 if(s->options.has_goal&&!s->goal_sent&&s->view.has_localization&&s->view.localization.state==LAB_REAL_LOC_TRACKING){lab_real_goal g=s->options.goal;g.map_id=s->view.localization.map.map_id;lab_status a=lab_real_runtime_set_goal(s->runtime,&g,now);if(a!=LAB_OK)return a;s->goal_sent=1;}
 if(s->bindings.navigation.view)s->bindings.navigation.view(s->bindings.navigation.user,&s->view.navigation);
 return r;
}
lab_status lab_real_session_save(lab_real_session *s){if(!s||!s->runtime)return LAB_ERR_ARGUMENT;if(!s->options.save_map[0])return LAB_OK;
 const lab_real_grid_view *m=&s->view.localization.map;int observed=0;if(lab_real_grid_valid(m))for(size_t i=0;i<(size_t)m->width*m->height;++i)if(m->cells[i]>=0){observed=1;break;}
 if(!observed)return LAB_ERR_UNAVAILABLE;return lab_real_map_save_ex(s->options.save_map,m,&s->options.localization,(s->options.use_mpu9250&&s->options.mpu.experimental)||(s->options.timed&&(s->options.timing.fpga_delay_declared||s->options.timing.sdk_time_declared)));
}
void lab_real_session_close(lab_real_session *s){if(!s)return;lab_real_runtime_close(s->runtime);s->runtime=NULL;if(s->bindings.navigation.close)s->bindings.navigation.close(s->bindings.navigation.user);if(s->bindings.chassis.ops)s->bindings.chassis.ops->close(s->bindings.chassis.user);if(s->bindings.localization.ops)s->bindings.localization.ops->close(s->bindings.localization.user);if(s->own_source&&s->bindings.sensors.ops)s->bindings.sensors.ops->close(s->bindings.sensors.user);memset(&s->bindings,0,sizeof(s->bindings));s->own_source=0;}
void lab_real_print_view(const lab_real_runtime_view *v){printf("mode=%s source=%s accepted=%llu rejected=%llu loc=%u matched=%llu aliases=%llu output=%s reason=%s sensor=%s localizer=%s nav=%s phase=%s scan_unc_us=%llu odom_unc_us=%llu\n",(v->mode==LAB_REAL_SHADOW&&v->localization.map.cells&&!v->localization.map.readonly)?"MAPPING_SHADOW":lab_real_mode_text(v->mode),lab_real_source_text(v->context.source),(unsigned long long)v->accepted,(unsigned long long)v->rejected,v->localization.state,(unsigned long long)v->localization.matched,(unsigned long long)v->coalesced_pose_aliases,v->enabled?"ARMED":"DISABLED",v->reason,v->sensor_health.reason,v->localization.reason,v->navigation.reason,v->navigation.phase_text,(unsigned long long)v->scan_uncertainty_us,(unsigned long long)v->odometry_uncertainty_us);fflush(stdout);}
int lab_real_write_preview(const char *path,const lab_real_runtime_view *v){FILE *f=fopen(path,"wbx");if(!f)return 0;int w=1280,h=720;uint32_t *p=calloc((size_t)w*h,sizeof(*p));if(!p){fclose(f);return 0;}lab_real_screen_draw(p,w,h,v);fprintf(f,"P6\n%d %d\n255\n",w,h);int ok=1;for(size_t i=0;i<(size_t)w*h;++i){unsigned char rgb[3]={(unsigned char)(p[i]>>16),(unsigned char)(p[i]>>8),(unsigned char)p[i]};if(fwrite(rgb,1,3,f)!=3){ok=0;break;}}free(p);if(fclose(f))ok=0;return ok;}
