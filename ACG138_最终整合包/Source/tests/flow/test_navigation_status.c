#include "navigation_status.h"
#include <assert.h>
#include <stdio.h>
int main(void){lab_real_runtime_view v={0};v.mode=LAB_REAL_NAVIGATION;v.has_goal=1;
 assert(!strcmp(navigation_vehicle_status(&v),"WAIT_START"));v.pending_enable=1;assert(!strcmp(navigation_vehicle_status(&v),"ARMING"));
 v.pending_enable=0;v.enabled=1;v.request.linear_m_s=.14;assert(!strcmp(navigation_vehicle_status(&v),"WAIT_FIRST_FEEDBACK"));
 v.time_us=1000000;v.feedback.time_us=1000000;v.feedback.actual_rpm[0]=20;assert(!strcmp(navigation_vehicle_status(&v),"RUNNING"));v.localization.odometry_fallback=1;assert(!strcmp(navigation_vehicle_status(&v),"ODOMETRY_RUNNING"));
 v.enabled=0;v.request.linear_m_s=0;snprintf(v.reason,sizeof(v.reason),"STALE_OR_INVALID_LOCALIZATION");assert(!strcmp(navigation_vehicle_status(&v),"SENSOR_OR_LOCALIZATION_WAIT"));
 snprintf(v.reason,sizeof(v.reason),"GOAL_REACHED");assert(!strcmp(navigation_vehicle_status(&v),"GOAL_REACHED"));
 v.reason[0]=0;v.navigation.state=2;snprintf(v.navigation.reason,sizeof(v.navigation.reason),"REPLANNING");assert(!strcmp(navigation_vehicle_status(&v),"PLANNING"));
 puts("PASS motion status separates request acceptance, measured wheels, odometry continuation, planning, data wait and arrival");return 0;}
