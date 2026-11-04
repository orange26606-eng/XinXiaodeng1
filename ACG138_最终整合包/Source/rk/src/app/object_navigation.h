#ifndef ACG_OBJECT_NAVIGATION_H
#define ACG_OBJECT_NAVIGATION_H
#include "object_lidar.h"
#include "phone_goal_ipc.h"
typedef struct object_navigation {
 int active,blocked,track_id;uint64_t epoch,map_id,map_version,request_us,created_us,frame_us,reached_us;
 double target_x,target_y,goal_x,goal_y,goal_yaw,standoff_m,front_m;
 char association_id[96],state[32],reason[96];
} object_navigation;
void object_navigation_cancel(object_navigation*,const char*);
void object_navigation_reject(object_navigation*,const char*);
int object_navigation_preview(object_navigation*,const phone_goal_message*,const obj_lidar_target*,const lab_real_runtime_view*,double,uint64_t);
int object_navigation_matches(const object_navigation*,const obj_lidar_target*,const lab_real_runtime_view*,uint64_t);
int object_navigation_can_start(const object_navigation*,const obj_lidar_target*,const lab_real_runtime_view*,uint64_t);
/* 面板需取消运行任务或使能时返回1，不自动重启。 */
int object_navigation_tick(object_navigation*,const obj_lidar_target*,const lab_real_runtime_view*,int,uint64_t);
#endif
