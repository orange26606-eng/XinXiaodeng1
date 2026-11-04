#ifndef ACG_LAB_ROUTER_H
#define ACG_LAB_ROUTER_H
#include "lab_types.h"
typedef lab_status (*lab_sink_push)(void *,const lab_event *);
typedef lab_status (*lab_sink_reset)(void *,const lab_context *);
typedef struct lab_router {
 lab_context context;
 struct map_config local_config;
 struct scan_frame latest_scan;
 struct local_map local_map;
 lab_pose latest_pose;
 lab_truth latest_truth;
 lab_meta scan_meta,pose_meta,truth_meta;
 uint64_t accepted[8],rejected,record_failures,map_waits,map_failures,truth_only,resets;
 int have_scan,have_pose,have_truth;
 lab_sink_push map_push,record_push;
 lab_sink_reset map_reset;
 void *map_user,*record_user;
} lab_router;
void lab_router_init(lab_router *router,const lab_context *context);
lab_status lab_router_reset(lab_router *router,const lab_context *context);
lab_status lab_router_dispatch(lab_router *router,const lab_event *event);
#endif
