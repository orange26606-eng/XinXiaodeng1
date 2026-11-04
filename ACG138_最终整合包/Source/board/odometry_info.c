/* 只读本地诊断，不打开电机或控制套接字。 */
#define _POSIX_C_SOURCE 200809L
#include "odometry_runtime.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
int main(void){
 const char*dir=getenv("ACG_PHONE_DIR");if(!dir||!*dir)dir="/run/acg138-phone-full9250";
 struct odom_state s={0};if(odom_shared_read(dir,&s)){fprintf(stderr,"No valid odometry shared snapshot in %s; start matching full MPU9250 runtime first.\n",dir);return 2;}
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
 char out[16384];if(!odom_json(out,sizeof(out),&s,(uint64_t)t.tv_sec*1000+t.tv_nsec/1000000))return 3;
 puts(out);return 0;
}
