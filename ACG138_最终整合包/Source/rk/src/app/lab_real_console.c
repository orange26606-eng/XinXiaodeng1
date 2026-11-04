#define _POSIX_C_SOURCE 200809L
#include "lab_real_session.h"
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#ifdef LAB_REAL_POSIX_SOURCE
#define LAB_CAPTURE_AVAILABLE "true"
#else
#define LAB_CAPTURE_AVAILABLE "false"
#endif
#ifdef LAB_REAL_SNAPSHOT_SOURCE
#define LAB_SNAPSHOT_AVAILABLE "true"
#else
#define LAB_SNAPSHOT_AVAILABLE "false"
#endif
static volatile sig_atomic_t running=1;
static void stop_run(int s){(void)s;running=0;}
static uint64_t now_us(void){
#ifdef LAB_REAL_POSIX_SOURCE
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;
#else
 return (uint64_t)clock()*1000000/CLOCKS_PER_SEC;
#endif
}
int main(int argc,char **argv){
 if(argc==2&&!strcmp(argv[1],"--info")){puts("{\"api\":\"1.0.0\",\"milestone\":\"RK_MAPPING_SHADOW_OFFLINE\",\"sensors_linked\":true,\"posix_capture_available\":" LAB_CAPTURE_AVAILABLE ",\"localization_linked\":true,\"planner_linked\":true,\"chassis_linked\":true,\"hardware_output\":false,\"board_tested\":false,\"real_loop_closure\":false,\"mapping_shadow_supported\":true,\"time_chain\":\"SDK_ST01_ODT1_OPTIONAL\",\"snapshot_file_source\":" LAB_SNAPSHOT_AVAILABLE "}");return 0;}
 if(argc==2&&!strcmp(argv[1],"--help")){lab_real_usage();return 0;}
 lab_real_options o;char error[256];if(!lab_real_options_parse(&o,argc,argv,error,sizeof(error))){fprintf(stderr,"%s\n",error);return 2;}
 if(!o.run){printf("PREFLIGHT ONLY mode=%s modules=IO,LC,MC,planner hardware_output=DISABLED board_tested=false\nls01_port=%u source_ip=%s odom=%s map=%s\nUnknown time bounds remain UNKNOWN; declared profiles never imply board validation.\n",o.mapping_shadow?"MAPPING_SHADOW":lab_real_mode_text(o.mode),o.source.ls01_port,o.source.allowed_source_ip,o.source.odometry_path,o.load_map);
  printf("time_chain=%s profile_crc=%08x fpga_delay_declared=%u fpga_stamp_declared=%u sdk_time_declared=%u\n",o.mpu_snapshot[0]?"MPUSVC_FILE_LT01_MIDPOINT":o.use_mpu9250?"MPU9250_I9E1_V2_LT01":o.timed?"SDK_ST01_ODT1":"LEGACY_RECEIPT_PROXY",o.timing.profile_crc32,o.timing.fpga_delay_declared,o.timing.fpga_stamp_declared,o.timing.sdk_time_declared);
  if(o.mpu_snapshot[0])printf("snapshot=%s receiver=FILE_ONLY udp5016=NOT_OPENED\n",o.mpu_snapshot);
  if(o.use_mpu9250&&!o.mpu_snapshot[0])printf("MPU source=%s:%u peer=%s timing_profile=%s experimental=%u; NO MOTOR OUTPUT\n",o.mpu.bind_ip,o.mpu.port,o.mpu.peer_ip,o.mpu.timing_profile,o.mpu.experimental);
  if(o.preview[0]){lab_real_runtime_view v={0};v.context=(lab_real_context){LAB_REAL_HARDWARE,LAB_REAL_MONOTONIC,1,1};v.mode=o.mode;snprintf(v.reason,sizeof(v.reason),"PREFLIGHT_NO_INPUT_OPENED");if(!lab_real_write_preview(o.preview,&v))return 4;}return 0;}
 uint64_t started=now_us();lab_real_context ctx={LAB_REAL_HARDWARE,LAB_REAL_MONOTONIC,(uint32_t)(started^(started>>32))|1u,1};lab_real_session s;lab_status r=lab_real_session_open(&s,&o,&ctx,NULL);if(r!=LAB_OK){fprintf(stderr,"SOURCE_OR_MODULE_OPEN_FAILED status=%s; no fallback\n",lab_status_text(r));return 3;}
 signal(SIGINT,stop_run);signal(SIGTERM,stop_run);uint64_t last_log=0,last_tick=0;int code=0;
 while(running){uint64_t now=now_us();if(now-started>=o.duration_us)break;if(now<=last_tick)continue;last_tick=now;r=lab_real_session_tick(&s,now);if(!last_log||now-last_log>=1000000){lab_real_print_view(&s.view);last_log=now;}
  if(r<0&&r!=LAB_ERR_UNAVAILABLE){lab_real_print_view(&s.view);code=5;break;}
#ifdef LAB_REAL_POSIX_SOURCE
  struct timespec delay={0,20000000};nanosleep(&delay,NULL);
#endif
 }
 if(o.save_map[0]&&(r=lab_real_session_save(&s))!=LAB_OK){fprintf(stderr,"MAP_NOT_SAVED status=%s; empty maps are not success\n",lab_status_text(r));code=6;}
 if(o.preview[0]&&!lab_real_write_preview(o.preview,&s.view))code=4;lab_real_print_view(&s.view);lab_real_session_close(&s);return code;
}
