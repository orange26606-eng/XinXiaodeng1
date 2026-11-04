#ifndef ACG_OBJECT_READONLY_H
#define ACG_OBJECT_READONLY_H
#include "lab_localization.h"
#include <stddef.h>
#include <stdint.h>
#include "object_follow_v5.h"
#include "../../../board/object_attributes.h"
#define OBJ_MAX 65
#define OBJ_STATE_MAX 262144u
#define OBJ_SNAPSHOT_MAX (2u*1024u*1024u)
typedef struct obj_detection {int track_id,class_id,observed,confirmed,depth_valid,bbox_valid;double x,y,z,error,bbox[4],score;int coarse_valid,bearing_valid;double coarse_range,coarse_error,bearing;char coarse_reason[96],coarse_quality[24];char label[64],tracking[24],reason[80];int eye,geometry_eye,depth_eye,bearing_edges_valid;int depth_bearing_valid;double depth_bearing;unsigned depth_candidate_count;double depth_candidate_range[3],depth_candidate_error[3];unsigned depth_candidate_points[3];double bearing_left,bearing_right,depth_bbox[4];uint64_t depth_frame_id,depth_time_us;int ray_origin_valid;double ray_origin[3];int ray_direction_valid;double ray_direction[3],ray_left[3],ray_right[3];int reacquire_candidate_valid,reacquire_candidate_bearing_valid,reacquire_candidate_eye;double reacquire_candidate_bearing_rad;uint64_t reacquire_candidate_time_us;int reacquire_candidate_track_id;char reacquire_candidate_reason[96];
 /* 语义意图与实际检测类别分开，绑定改变只使旧距离和速度失效，不形成新观测。原观测时间不等于JSON生成时刻。

 */
 int task_class_id,observed_class_id;uint32_t binding_generation;
 char binding_reason[96];uint64_t observation_frame_id,observation_time_us;
 object_follow_exit_v5 follow_exit;
 obj_attributes attributes;
} obj_detection;
typedef struct obj_frame {uint64_t epoch,frame_id,frame_us,generated_us;int eye,calibrated,selected,count;obj_detection objects[OBJ_MAX];} obj_frame;
typedef struct obj_selection {uint64_t seq,epoch;int track_id;char snapshot_id[64];} obj_selection;
typedef struct obj_mount {int enabled,verified,eye;uint64_t max_age_us,delay_us;double x,y,z,roll,pitch,yaw;} obj_mount;
typedef struct obj_map_entry {uint64_t epoch,frame_us;int track_id,selected,seen;double odom_x,odom_y,error;char label[64];} obj_map_entry;
typedef struct obj_map_state {int initialized,count;lab_real_context context;uint64_t map_id,epoch,last_frame;obj_map_entry entries[OBJ_MAX];char reason[80];} obj_map_state;
/* 解析器不产生传输或控制副作用，也用于Java到C的跨语言HTTP请求格式测试。
 */
int obj_select_parse(const char*,size_t,int clear,obj_selection*);
int obj_frame_parse(const char*,size_t,obj_frame*);
/* 含PNG快照独立上限2MiB，状态文件保持128KiB。 */
int obj_snapshot_parse(const char*,size_t,obj_frame*);
/* 严格配对同帧图像标识及检测结果，保留原始字节。 */
int obj_snapshot_matches(const char*,size_t,const obj_frame*);
int obj_motion_meta(const char*,size_t,uint64_t*);
int obj_json_time(const char*,size_t,const char*key,uint64_t*);
int obj_approach_u64(const char*,size_t,const char*key,uint64_t*);
int obj_mount_load(const char*,obj_mount*);
int obj_frame_load(const char*,obj_frame*);
int obj_file_read(const char*,size_t,char**,size_t*);
int obj_atomic_write(const char*,const char*,size_t);
void obj_map_update(obj_map_state*,const obj_frame*,const obj_mount*,const lab_localization_view*,const lab_localization_binding*,uint64_t);
int obj_map_json(const obj_map_state*,const lab_localization_view*,uint64_t,char*,size_t,size_t*);
void obj_map_export(obj_map_state*,const char*dir,const char*config,const lab_localization_view*,const lab_localization_binding*,uint64_t);
#endif
