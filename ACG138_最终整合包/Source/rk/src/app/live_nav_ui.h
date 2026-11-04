#ifndef LIVE_NAV_UI_H
#define LIVE_NAV_UI_H
#include "lab_real_runtime.h"
#include "object_dual_task.h"
typedef struct live_nav_ui {double center_x,center_y,span_m,goal_x,goal_y,requested_x,requested_y;int width,height,selected,goal_adjusted;char message[128];double last_pose_x,last_pose_y,last_pose_yaw;uint64_t last_pose_time,last_pose_map_id;uint32_t have_last_pose,target_observation_pending;} live_nav_ui;
void live_nav_ui_init(live_nav_ui *u);
int live_nav_ui_pick(live_nav_ui *u,const lab_real_runtime_view *v,int x,int y);
/* 同一map_id允许较旧内容修订，但远程世界坐标目标始终按当前几何与栅格核对，拒绝未来版本。
 */
int live_nav_ui_select_world(live_nav_ui *u,const lab_real_runtime_view *v,uint64_t map_id,uint64_t map_version,double x_m,double y_m);
int live_nav_ui_button(int x,int y); /* 操作1启动、2停止、3放大、4缩小。 */
void live_nav_ui_draw(live_nav_ui *u,uint32_t *pixels,int w,int h,const lab_real_runtime_view *v,int drive_available);
void live_nav_ui_draw_scene(live_nav_ui*,uint32_t*,int,int,const lab_real_runtime_view*,const obj_scene*);
void live_nav_ui_draw_dual(live_nav_ui*,uint32_t*,int,int,const lab_real_runtime_view*,const obj_scene*,const object_dual_task*,const obj_frame*,double,int);
#endif
