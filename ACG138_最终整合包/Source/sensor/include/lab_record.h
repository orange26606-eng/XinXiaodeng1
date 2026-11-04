#ifndef ACG_MAPPING_LAB_RECORD_H
#define ACG_MAPPING_LAB_RECORD_H
#include "lab_codec.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct lab_record_writer lab_record_writer;
typedef struct lab_record_reader lab_record_reader;
typedef struct lab_replay lab_replay;
typedef struct lab_record_options {
 uint64_t max_file_bytes; /* 默认256MiB，硬上限1GiB。 */
 uint32_t allow_truncated; /* 只读端可返回有效前缀，但末端状态仍为TRUNCATED。 */
} lab_record_options;
typedef struct lab_record_stats {
 uint64_t events,bytes,dropped_events;
 uint32_t clean_footer,capture_loss;
 lab_status terminal_status;
} lab_record_stats;
void lab_record_options_default(lab_record_options *options);
lab_status lab_record_writer_open(lab_record_writer **out,const char *new_path,
 const lab_record_options *options,const lab_file_header *header,const char *metadata_utf8);
lab_status lab_record_writer_append(lab_record_writer *writer,const lab_event *event);
void lab_record_writer_stats(const lab_record_writer *writer,lab_record_stats *out);
/* 写入结束记录并释放写入器，采集丢失需反映在footer_flags中。 */
lab_status lab_record_writer_close(lab_record_writer *writer,uint32_t footer_flags);
lab_status lab_record_reader_open(lab_record_reader **out,const char *path,const lab_record_options *options);
lab_status lab_record_reader_info(const lab_record_reader *reader,lab_file_header *header,
 char *metadata_utf8,size_t capacity,size_t *required_bytes);
lab_status lab_record_reader_next(lab_record_reader *reader,lab_event *out);
void lab_record_reader_stats(const lab_record_reader *reader,lab_record_stats *out);
void lab_record_reader_close(lab_record_reader *reader);

typedef struct lab_replay_options {
 double speed; /* 回放速度0.1至8.0倍，暂停为独立状态。 */
 uint32_t timeline_id,start_paused,allow_truncated;
 uint64_t lookahead_us; /* 默认100000项，最多200000项。 */
} lab_replay_options;
void lab_replay_options_default(lab_replay_options *options);
lab_status lab_replay_open(lab_replay **out,const char *path,const lab_replay_options *options,uint64_t host_now_us);
lab_status lab_replay_poll(lab_replay *replay,uint64_t host_now_us,lab_event *out);
lab_status lab_replay_pause(lab_replay *replay,uint64_t host_now_us);
lab_status lab_replay_resume(lab_replay *replay,uint64_t host_now_us);
lab_status lab_replay_set_speed(lab_replay *replay,double speed,uint64_t host_now_us);
/* 暂停时推进到下一SCAN，再最多带出一笔后续POSE，受lookahead_us和128个事件限制，遇到再下一SCAN前停止。
 */
lab_status lab_replay_step_scan(lab_replay *replay);
/* 新时间线必须不同，清空预读、回卷状态及相对时钟。 */
lab_status lab_replay_restart(lab_replay *replay,uint32_t new_timeline_id,uint64_t host_now_us);
uint64_t lab_replay_time_us(const lab_replay *replay,uint64_t host_now_us);
void lab_replay_close(lab_replay *replay);
#ifdef __cplusplus
}
#endif
#endif
