#include "object_turn_smooth.h"
#include "object_dual_task.h"
#include <stdio.h>
#include <stdlib.h>
#define C(x) do{checks++;if(!(x)){fprintf(stderr,"TURN_FAIL line=%d %s\n",__LINE__,#x);exit(1);}}while(0)
static unsigned checks;
static void ramps(void){obj_turn_smoother s={0};uint64_t now=1000000;double old=0;
 for(int k=0;k<20;k++){double w=obj_turn_shape(&s,now,.2,0);C(w>=old&&w<=.200000001);C(w-old<=.020000001);old=w;now+=50000;}
 C(fabs(old-.2)<1e-8);double w=obj_turn_shape(&s,now+5000000,.3,0);C(w-old<=.040000001);
 C(obj_turn_shape(&s,now+5050000,0,1)==0);C(obj_turn_shape(&s,now+5050000,.2,0)==0);
 obj_turn_reset(&s);C(obj_turn_shape(&s,now,.2,0)<=.020000001);
}
static void reversal(void){obj_turn_smoother s={0};uint64_t now=1000000;double yaw=0;
 for(int i=0;i<15;i++){obj_turn_observe(&s.motion,now,yaw);obj_turn_shape(&s,now,.2,0);now+=50000;}
 int negative=0;for(int i=0;i<30;i++){yaw+=i<14?.025:0;obj_turn_observe(&s.motion,now,yaw);double w=obj_turn_shape(&s,now,-.2,0);if(i<14)C(w>=0);if(w<0)negative=1;C(fabs(w)<=.200000001);now+=50000;}C(negative);
 obj_turn_measure m=s.motion;obj_turn_observe(&s.motion,m.stamp,m.yaw+2);C(s.motion.rate==m.rate&&s.motion.yaw==m.yaw);C(!obj_turn_measured(&s.motion,now+400000));
 obj_turn_shape(&s,now,0,1);C(s.output==0);
}
static void endpoint(void){C(fabs(obj_turn_endpoint(.04,.2))<fabs(obj_turn_endpoint(.4,.2)));C(obj_turn_endpoint(-.04,.2)<0);
 object_dual_task t;object_dual_task_init(&t);C(object_dual_task_prepare(&t,1,9,3,.2,1));uint64_t now=10000000;C(object_dual_task_start(&t,1,9,3,now));t.last_visual_us=now-2000000;t.bearing_valid=1;t.bearing_rad=.1;t.search_center_yaw=0;
 lab_real_runtime_view v={.context={LAB_REAL_HARDWARE,LAB_REAL_MONOTONIC,1,1},.has_odometry=1,.odometry_time_us=now,.raw_odometry={.quality=LAB_POSE_VALID}};lab_real_command out;
 C(object_dual_task_tick(&t,NULL,NULL,&v,now,&out));C(t.static_lost&&out.angular_rad_s>0);
 now+=50000;v.odometry_time_us=now;v.raw_odometry.yaw_rad=.42;C(object_dual_task_tick(&t,NULL,NULL,&v,now,&out));C(out.angular_rad_s==0&&t.static_search_leg==0);
 now+=50000;v.odometry_time_us=now;v.raw_odometry.yaw_rad=.45;C(object_dual_task_tick(&t,NULL,NULL,&v,now,&out));C(out.angular_rad_s==0&&t.static_search_leg==0);
 now+=50000;v.odometry_time_us=now;C(object_dual_task_tick(&t,NULL,NULL,&v,now,&out));C(t.static_search_leg==1&&out.angular_rad_s<0);
 object_dual_task_cancel(&t,"USER_CANCEL");C(!object_dual_task_tick(&t,NULL,NULL,&v,now+1,&out)&&out.angular_rad_s==0);
}
int main(void){ramps();reversal();endpoint();printf("PASS smooth turn checks=%u ramp/reversal/measured-endpoint/STOP no-device\n",checks);return 0;}
