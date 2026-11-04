/* V4L2 视频接口，使用有界 MMAP 缓存。采集格式由相机、ISP 和 YOLO 生产进程协商，此层负责读取和像素转换。 */
#ifndef DASHBOARD_VIDEO_H
#define DASHBOARD_VIDEO_H

#include <stddef.h>
#include <stdint.h>

#define DASHBOARD_FOURCC(a,b,c,d) ((uint32_t)(a) | ((uint32_t)(b) << 8) | \
                                 ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#define DASHBOARD_NV12 DASHBOARD_FOURCC('N','V','1','2')
#define DASHBOARD_NV21 DASHBOARD_FOURCC('N','V','2','1')
#define DASHBOARD_YUV420 DASHBOARD_FOURCC('Y','U','1','2')
#define DASHBOARD_YVU420 DASHBOARD_FOURCC('Y','V','1','2')
#define DASHBOARD_YUYV DASHBOARD_FOURCC('Y','U','Y','V')
#define DASHBOARD_UYVY DASHBOARD_FOURCC('U','Y','V','Y')
#define DASHBOARD_RGB24 DASHBOARD_FOURCC('R','G','B','3')
#define DASHBOARD_BGR24 DASHBOARD_FOURCC('B','G','R','3')

enum dashboard_matrix { DASHBOARD_BT601 = 1, DASHBOARD_BT709 = 2 };
enum dashboard_range { DASHBOARD_LIMITED = 1, DASHBOARD_FULL = 2 };

struct dashboard_video_layout {
    uint32_t fourcc;
    unsigned width, height, stride;
    enum dashboard_matrix matrix;
    enum dashboard_range range;
};

struct dashboard_video {
    int fd;
    uint64_t frames, last_frame_ms;
    uint32_t *pixels;                /* 输出缓冲为output_width×output_height个XRGB8888像素。 */
    unsigned output_width, output_height;
    char format_name[8];
    struct dashboard_video_layout layout;
    size_t required_bytes;
    void *maps[8];
    size_t map_lengths[8];
    unsigned buffer_count;
    int streaming;
};

/* 这些纯函数只转换数据，调用方管理输入和存储。 */
int dashboard_video_required_bytes(const struct dashboard_video_layout *layout,
                                   size_t *required);
/* 成功返回0，输入无效返回-1且不改输出。source_bytes包含步长填充及完整图像平面，destination_pixels按元素计数。等比例黑边适配，最近邻取样，像素字为0x00RRGGBB。源与目标不得重叠，拒绝超过8192×8192的尺寸。




 */
int dashboard_video_convert(const struct dashboard_video_layout *layout,
                            const uint8_t *source, size_t source_bytes,
                            uint32_t *destination, size_t destination_pixels,
                            unsigned output_width, unsigned output_height);

/* v7 YOLO输出640×640的RGB画布，其中双眼图像为位于(0,140)的640×360区域。显式启用时完整适配该区域并叠加原状态文字；其他格式使用全图。这里不按像素内容猜测、不裁场景、不执行推理。


 */
int dashboard_video_preview(const struct dashboard_video_layout *layout,
                            const uint8_t *source, size_t source_bytes,
                            uint32_t *destination, size_t destination_pixels,
                            unsigned output_width, unsigned output_height,
                            int yolo640);

/* open()初始化未打开的结构，open失败后也可安全close。生产者应已运行，不修改S_FMT或生产设置。update返回0表示无可用帧，1表示新帧，-1表示致命错误，并提供errno及stderr。


 */
int dashboard_video_open(struct dashboard_video *video, const char *path,
                         unsigned output_width, unsigned output_height);
int dashboard_video_update(struct dashboard_video *video, uint64_t now_ms);
void dashboard_video_close(struct dashboard_video *video);

#endif
