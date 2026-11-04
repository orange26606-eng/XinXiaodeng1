#ifndef ACG_TAG_GOAL_H
#define ACG_TAG_GOAL_H
#include "lab_real_runtime.h"
#define TAG_JSON_MAX 32768
#define TAG_MAX 16
typedef struct tag_observation {int id,confirmed,pose_valid;double x,y,z,nx,ny,nz,error,margin;} tag_observation;
typedef struct tag_snapshot {uint64_t sequence,frame_us,generated_us;int calibrated,eye,count;tag_observation tags[TAG_MAX];} tag_snapshot;
typedef struct tag_mount {int calibrated,verified,eye;uint64_t max_age_us,delay_us;double x,y,z,roll,pitch,yaw;} tag_mount;
typedef struct tag_task {int active,id;uint64_t map_id,frame_us,last_poll_us,reached_us,created_us,request_us;double standoff,target_x,target_y,goal_x,goal_y,goal_yaw;char state[32],reason[80];} tag_task;
int tag_snapshot_parse(const char*,size_t,tag_snapshot*);
int tag_snapshot_load(const char*,tag_snapshot*);
int tag_mount_load(const char*,tag_mount*);
int tag_capture_pose(const tag_snapshot*,const tag_mount*,const lab_localization_binding*,const lab_real_context*,lab_pose*,char reason[80]);
int tag_goal_compute(const tag_snapshot*,const tag_mount*,int,double,uint64_t,const lab_pose*,double,tag_task*);
int tag_task_reobserve(tag_task*,const tag_snapshot*,const tag_mount*,uint64_t,const lab_pose*);
void tag_task_cancel(tag_task*,const char*);
#endif
