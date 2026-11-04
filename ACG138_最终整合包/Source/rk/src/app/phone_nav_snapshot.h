#ifndef ACG_PHONE_NAV_SNAPSHOT_H
#define ACG_PHONE_NAV_SNAPSHOT_H
#include "lab_real_runtime.h"
#include <stddef.h>
/* 每个面板独立持有显示缓存，坐标保留原观测时间。 */

typedef struct phone_nav_snapshot_state {
 lab_real_context context;uint64_t map_id,pose_time_us;lab_pose pose;uint32_t have_pose;
} phone_nav_snapshot_state;
int phone_nav_snapshot_json_cached(phone_nav_snapshot_state *state,const lab_real_runtime_view *v,char *out,size_t cap,size_t *written);
/* 输出归调用方所有。255为未知，0..100为占用概率，首行对应最小Y；无效或不完整快照返回0。 */


int phone_nav_snapshot_json(const lab_real_runtime_view *v,char *out,size_t cap,size_t *written);
/* 仅保留最新绘图输出。 */
int phone_nav_motion_json(const lab_real_runtime_view *v,uint64_t now,char *out,size_t cap,size_t *written);
int phone_nav_motion_export(const lab_real_runtime_view *v,const char *directory,uint64_t now);
#endif
