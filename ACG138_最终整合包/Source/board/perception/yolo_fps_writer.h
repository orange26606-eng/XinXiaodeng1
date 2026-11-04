/* 统计完整帧吞吐，包含推理之外的阶段。在适配器状态锁内调用，每秒最多发布一次，YOLO_FPS_FILE位于本会话私有临时目录。

 */
#ifndef YOLO_FPS_WRITER_H
#define YOLO_FPS_WRITER_H
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

struct yolo_fps_writer {
    char path[PATH_MAX];
    uint64_t start_ns, start_frames;
    int enabled, started;
};

static void yolo_fps_writer_init(struct yolo_fps_writer *writer)
{
    memset(writer, 0, sizeof(*writer));
    const char *path = getenv("YOLO_FPS_FILE");
    if (!path || !*path) return;
    if (*path != '/' || strlen(path) > sizeof(writer->path) - 40) {
        fputs("YOLO_FPS: invalid absolute snapshot path; display metric disabled\n", stderr);
        return;
    }
    memcpy(writer->path, path, strlen(path) + 1);
    writer->enabled = 1;
}

static int yolo_fps_writer_publish(struct yolo_fps_writer *writer,
                                  uint64_t stamp_ns, uint64_t frames, double fps)
{
    char temporary[PATH_MAX], record[192];
    const int temp_length = snprintf(temporary, sizeof(temporary), "%s.%ld.tmp",
                                     writer->path, (long)getpid());
    const int length = snprintf(record, sizeof(record),
        "YOLO_FPS_V1 %ld %" PRIu64 " %" PRIu64 " %.3f\n",
        (long)getpid(), stamp_ns / 1000000u, frames, fps);
    if (temp_length < 0 || (size_t)temp_length >= sizeof(temporary) ||
        length < 0 || (size_t)length >= sizeof(record)) { errno = ENAMETOOLONG; return -1; }
    const int fd = open(temporary, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) return -1;
    size_t sent = 0;
    int failure = 0;
    while (sent < (size_t)length) {
        const ssize_t n = write(fd, record + sent, (size_t)length - sent);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) { failure = n < 0 ? errno : EIO; break; }
        sent += (size_t)n;
    }
    if (close(fd) && !failure) failure = errno;
    if (!failure && rename(temporary, writer->path)) failure = errno;
    if (failure) { (void)unlink(temporary); errno = failure; return -1; }
    return 0;
}

static void yolo_fps_writer_frame(struct yolo_fps_writer *writer,
                                 uint64_t stamp_ns, uint64_t completed_frames)
{
    if (!writer->enabled || !stamp_ns) return;
    if (!writer->started || stamp_ns < writer->start_ns || completed_frames < writer->start_frames) {
        writer->started = 1;
        writer->start_ns = stamp_ns;
        writer->start_frames = completed_frames;
        return;
    }
    const uint64_t elapsed = stamp_ns - writer->start_ns;
    if (elapsed < 1000000000u) return;
    const double fps = (completed_frames - writer->start_frames) * 1e9 / (double)elapsed;
    if (yolo_fps_writer_publish(writer, stamp_ns, completed_frames, fps)) {
        fprintf(stderr, "YOLO_FPS: snapshot unavailable (%s); inference continues\n", strerror(errno));
        writer->enabled = 0;
    }
    writer->start_ns = stamp_ns;
    writer->start_frames = completed_frames;
}
#endif
