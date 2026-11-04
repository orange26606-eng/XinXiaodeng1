#include "lab_real_mpusvc_posix.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
static int copy(char *out,size_t n,const char *s){if(!*s||strlen(s)>=n)return 0;memcpy(out,s,strlen(s)+1);return 1;}
int main(int argc,char **argv){
 if(argc==2&&!strcmp(argv[1],"--info")){puts("{\"role\":\"MPU9250_HEADLESS_SENSOR_SERVICE\",\"wire\":\"I9E1 v2 UDP5016 + F6A1 v1 UDP5017\",\"snapshot\":\"MPUSVC1 768-byte BE+CRC\",\"legacy_bytes\":4224,\"wayland_required\":false,\"motor_sender\":false,\"time_error\":\"UNKNOWN\",\"board_tested\":false}");return 0;}
 if(argc==2&&!strcmp(argv[1],"--help")){puts("lab-mpu9250-service --run --odom-config FILE --imu-config FILE [--state-dir /run/acg138-mpu-service] [--bind 192.168.0.3] [--peer 192.168.0.2] [--duration-s 60] [--record NEW.i9cap] [--parameter-version 1]\nOffline: --replay FILE --snapshot NEW.mpusnap --odom-config FILE --imu-config FILE\nPreflight is the default without --run. Never configures a network, sends motor commands or starts cameras. Ports5016/5017 have one service owner. --attitude-source auto|rk|fpga (defaultauto); FPGA paired by epoch/ag_sequence/source_us. Raw-only replay remains RK-reference.");return 0;}
 mpusvc_options o={.parameter_version=1,.attitude_mode=F6_MODE_AUTO};strcpy(o.state_dir,"/run/acg138-mpu-service");strcpy(o.bind_ip,"192.168.0.3");strcpy(o.peer_ip,"192.168.0.2");const char *replay=NULL,*snapshot=NULL;int run=0;
 for(int i=1;i<argc;++i){const char *a=argv[i];if(!strcmp(a,"--run")){run=1;continue;}if(i+1>=argc){fprintf(stderr,"missing value: %s\n",a);return 2;}const char *v=argv[++i];
  if(!strcmp(a,"--state-dir")){if(!copy(o.state_dir,sizeof(o.state_dir),v))return 2;}
  else if(!strcmp(a,"--bind")){if(!copy(o.bind_ip,sizeof(o.bind_ip),v))return 2;}
  else if(!strcmp(a,"--peer")){if(!copy(o.peer_ip,sizeof(o.peer_ip),v))return 2;}
  else if(!strcmp(a,"--odom-config")){if(!copy(o.odom_path,sizeof(o.odom_path),v))return 2;}
  else if(!strcmp(a,"--imu-config")){if(!copy(o.imu_path,sizeof(o.imu_path),v))return 2;}
  else if(!strcmp(a,"--attitude-source")){if(!strcmp(v,"auto"))o.attitude_mode=F6_MODE_AUTO;else if(!strcmp(v,"rk"))o.attitude_mode=F6_MODE_RK;else if(!strcmp(v,"fpga"))o.attitude_mode=F6_MODE_FPGA;else return 2;}
  else if(!strcmp(a,"--record")){if(!copy(o.record_path,sizeof(o.record_path),v))return 2;}
  else if(!strcmp(a,"--replay"))replay=v;
  else if(!strcmp(a,"--snapshot"))snapshot=v;
  else if(!strcmp(a,"--duration-s")){char *end;errno=0;double n=strtod(v,&end);if(errno||end==v||*end||!isfinite(n)||n<.1||n>3600)return 2;o.duration_us=(uint64_t)(n*1e6);}
  else if(!strcmp(a,"--parameter-version")){char *end;errno=0;if(*v=='-'||*v=='+')return 2;unsigned long long n=strtoull(v,&end,10);if(errno||end==v||*end||!n)return 2;o.parameter_version=n;}
  else{fprintf(stderr,"unknown option: %s\n",a);return 2;}
 }
 if(!o.odom_path[0]||!o.imu_path[0]||(run&&replay)||((replay!=NULL)!=(snapshot!=NULL))||(replay&&o.record_path[0])){fputs("explicit odom/imu configs required; replay and live are exclusive\n",stderr);return 2;}
 struct odom_config odom;odom_default_config(&odom);struct i9_config imu;i9_default_config(&imu);if(odom_load_config(&odom,o.odom_path)||mpu_config_load(&imu,o.imu_path)){fputs("CONFIG_INVALID\n",stderr);return 3;}
 if(replay){mpusvc_view v;lab_status r=mpusvc_reprocess(replay,&odom,&imu,&v);if(r!=LAB_OK){fprintf(stderr,"CAPTURE_REJECTED status=%d\n",r);return 4;}uint8_t b[768];if(mpusvc_encode(b,&v)!=LAB_OK||mpusvc_write_new(snapshot,b,sizeof(b))!=LAB_OK)return 5;mpusvc_print(&v,v.published_us);puts("OFFLINE ONLY: source preserved/relabelled RECORDING; no legacy hardware shared file created.");return 0;}
 if(!run){mpusvc_core *s=NULL;lab_status r=mpusvc_open(&s,LAB_REAL_HARDWARE,1,o.parameter_version,&odom,&imu);if(r!=LAB_OK)return 3;mpusvc_close(s);printf("PREFLIGHT_CONFIG_ONLY bind=%s:5016 peer=%s state=%s; no socket/output/display opened; no motor sender\n",o.bind_ip,o.peer_ip,o.state_dir);return 0;}
 return mpusvc_run_posix(&o,&odom,&imu);
}
