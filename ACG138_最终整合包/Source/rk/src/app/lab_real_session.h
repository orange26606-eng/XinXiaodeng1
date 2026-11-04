#ifndef W1_REAL_SESSION_H
#define W1_REAL_SESSION_H
#include "lab_real_runtime.h"
#include "lab_real_mpu.h"
#include "lab_real_time_source.h"
#include "passage.h"
typedef struct lab_real_options {
 uint32_t use_mpu9250;lab_real_mpu_config mpu;
 char mpu_snapshot[256]; /* 完整MPUSVC1文件，与原始UDP数据源互斥。 */
 lab_live_source_config source;lab_localization_config localization;
 lab_real_time_profile timing;uint32_t timed,scan_time_port;
 char timing_profile[256],timed_odometry_path[256];
 uint32_t mode,run,preflight,has_initial,has_goal;uint64_t duration_us;
 char output[64],load_map[256],save_map[256],preview[256];
 lab_pose initial;lab_real_goal goal;
 /* 命令行组合SHADOW与BUILD_MAP，不新增冻结的公共模式。 */
 uint32_t mapping_shadow,enable_drive;char chassis_endpoint[192];
 passage_config passage;char passage_profile[256];
} lab_real_options;
typedef struct lab_real_session {lab_real_options options;lab_real_bindings bindings;lab_real_runtime *runtime;lab_real_runtime_view view;int own_source,initial_sent,goal_sent;uint64_t started_us;} lab_real_session;
void lab_real_options_default(lab_real_options *out);
int lab_real_options_parse(lab_real_options *out,int argc,char **argv,char *error,size_t error_bytes);
void lab_real_usage(void);
lab_status lab_real_session_open(lab_real_session *session,const lab_real_options *options,const lab_real_context *context,const lab_live_source_binding *injected_source);
lab_status lab_real_session_tick(lab_real_session *session,uint64_t now_us);
lab_status lab_real_session_save(lab_real_session *session);
void lab_real_session_close(lab_real_session *session);
void lab_real_print_view(const lab_real_runtime_view *view);
int lab_real_write_preview(const char *new_path,const lab_real_runtime_view *view);
#endif
