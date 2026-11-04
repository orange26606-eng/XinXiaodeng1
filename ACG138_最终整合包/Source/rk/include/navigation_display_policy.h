#ifndef ACG_NAVIGATION_DISPLAY_POLICY_H
#define ACG_NAVIGATION_DISPLAY_POLICY_H
/* 导航短暂等待时保留地图；普通预览使用原摄像头与IMU画面。 */

static inline int navigation_display_policy(int previous,int drive,int intent,int goal,int armed){
 if(!drive||!intent||!goal)return 0;
 return armed||previous;
}
#endif
