#ifndef ACG_VIDEO_SERVICE_H
#define ACG_VIDEO_SERVICE_H
#include <stddef.h>
#include <stdint.h>
struct video_source {uint32_t sequence;uint64_t capture_ms;unsigned width,height,stride;};
typedef int (*video_source_reader)(unsigned char*,size_t,uint32_t,struct video_source*);
typedef int (*video_session_check)(const char*);
struct video_stats {uint64_t encoded,dropped,resyncs,last_capture_ms;unsigned fps_milli,encode_us;int ready,failed,connected;};
int video_service_start(const char *mpp_library,video_source_reader reader);
void video_service_stop(void);
void video_service_stats(struct video_stats *stats);
/* 进入本函数前，已有HTTP处理器先完成认证。 */
void video_service_stream(int fd,const char *session,video_session_check valid);
#endif
