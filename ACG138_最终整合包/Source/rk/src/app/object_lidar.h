#ifndef ACG_OBJECT_LIDAR_H
#define ACG_OBJECT_LIDAR_H
#include "lab_real_runtime.h"
#include "passage.h"
#include <stddef.h>
#include <stdint.h>
#define OBJ_LIDAR_JSON_MAX 65536u
#define OBJ_LIDAR_SNAPSHOT_MAX (3u*1024u*1024u)
typedef struct obj_lidar_state obj_lidar_state;
typedef struct obj_lidar_target {
 int valid,track_id;
 uint64_t epoch,frame_id,frame_us,scan_us,confirmed_us,published_us,map_id,map_version;
 lab_real_context context;
 double map_x,map_y,range_m,error_m;
 char association_id[96],status[32],reason[96];
} obj_lidar_target;
obj_lidar_state *obj_lidar_create(void);
void obj_lidar_free(obj_lidar_state*);
/* 不持有传感器或传输通道，由建图事件循环调用，更新工作量有界。 */
int obj_lidar_update(obj_lidar_state*,const char*dir,const char*config,
 const lab_real_runtime_view*,const lab_localization_binding*,const passage_config*,uint64_t now_us);
int obj_lidar_get_target(const obj_lidar_state*,obj_lidar_target*);
int obj_lidar_json(const obj_lidar_state*,uint64_t now_us,char*,size_t,size_t*);
#endif
