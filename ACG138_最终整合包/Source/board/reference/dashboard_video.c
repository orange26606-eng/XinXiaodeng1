/* V4L2 视频采集与显示：格式协商、有界缓存、按行跨度转换 YUV/RGB、颜色范围选择和等比显示。
 * 参考：
 * https://docs.kernel.org/userspace-api/media/v4l/pixfmt-v4l2.html
 * https://docs.kernel.org/userspace-api/media/v4l/pixfmt-yuv-planar.html
 * https://docs.kernel.org/userspace-api/media/v4l/colorspaces-defs.html */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "dashboard_video.h"
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int is_rgb(uint32_t f)
{
    return f == DASHBOARD_RGB24 || f == DASHBOARD_BGR24;
}

int dashboard_video_required_bytes(const struct dashboard_video_layout *l,
                                   size_t *required)
{
    size_t row, size;
    unsigned bytes_per_pixel = 1;
    if (!l || !required || !l->width || !l->height || l->width > 8192 ||
        l->height > 8192 || !l->stride ||
        (l->matrix != DASHBOARD_BT601 && l->matrix != DASHBOARD_BT709) ||
        (l->range != DASHBOARD_LIMITED && l->range != DASHBOARD_FULL))
        return -1;
    switch (l->fourcc) {
    case DASHBOARD_NV12: case DASHBOARD_NV21:
    case DASHBOARD_YUV420: case DASHBOARD_YVU420:
        if ((l->width | l->height | l->stride) & 1U) return -1;
        break;
    case DASHBOARD_YUYV: case DASHBOARD_UYVY:
        if (l->width & 1U) return -1;
        bytes_per_pixel = 2;
        break;
    case DASHBOARD_RGB24: case DASHBOARD_BGR24:
        bytes_per_pixel = 3;
        break;
    default:
        return -1;
    }
    row = (size_t)l->width * bytes_per_pixel;
    if (l->stride < row || l->stride > SIZE_MAX / l->height) return -1;
    size = (size_t)l->stride * l->height;
    if (bytes_per_pixel == 1) {
        if (size > SIZE_MAX - size / 2) return -1;
        size += size / 2;
    }
    /* 限制映射大小，避免错误生产者元数据造成无界分配。 */
    if (size > (size_t)256 * 1024 * 1024) return -1;
    *required = size;
    return 0;
}

static unsigned clamp8(int value)
{
    return value < 0 ? 0U : value > 255 ? 255U : (unsigned)value;
}

static uint32_t yuv_pixel(unsigned y, unsigned u, unsigned v,
                          enum dashboard_matrix matrix,
                          enum dashboard_range range)
{
    int c = (int)y, d = (int)u - 128, e = (int)v - 128;
    int r, g, b;
    if (range == DASHBOARD_LIMITED) {
        c = 298 * (c - 16);
        if (matrix == DASHBOARD_BT709) {
            r = c + 459 * e; g = c - 55 * d - 136 * e; b = c + 541 * d;
        } else {
            r = c + 409 * e; g = c - 100 * d - 208 * e; b = c + 516 * d;
        }
    } else {
        c *= 256;
        if (matrix == DASHBOARD_BT709) {
            r = c + 403 * e; g = c - 48 * d - 120 * e; b = c + 475 * d;
        } else {
            r = c + 359 * e; g = c - 88 * d - 183 * e; b = c + 454 * d;
        }
    }
    return (clamp8((r + 128) / 256) << 16) |
           (clamp8((g + 128) / 256) << 8) | clamp8((b + 128) / 256);
}

static uint32_t get_pixel(const struct dashboard_video_layout *l,
                          const uint8_t *s, unsigned x, unsigned y)
{
    size_t row = (size_t)y * l->stride, plane = (size_t)l->stride * l->height;
    unsigned yy, u, v;
    if (is_rgb(l->fourcc)) {
        const uint8_t *p = s + row + (size_t)x * 3;
        int r = p[l->fourcc == DASHBOARD_RGB24 ? 0 : 2];
        int g = p[1], b = p[l->fourcc == DASHBOARD_RGB24 ? 2 : 0];
        if (l->range == DASHBOARD_LIMITED) {
            r = (int)clamp8(((r - 16) * 255 + 109) / 219);
            g = (int)clamp8(((g - 16) * 255 + 109) / 219);
            b = (int)clamp8(((b - 16) * 255 + 109) / 219);
        }
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
    }
    if (l->fourcc == DASHBOARD_YUYV || l->fourcc == DASHBOARD_UYVY) {
        const uint8_t *p = s + row + (size_t)(x / 2) * 4;
        unsigned uyvy = l->fourcc == DASHBOARD_UYVY;
        yy = p[(x & 1U) * 2 + uyvy];
        u = p[uyvy ? 0 : 1]; v = p[uyvy ? 2 : 3];
    } else {
        yy = s[row + x];
        if (l->fourcc == DASHBOARD_NV12 || l->fourcc == DASHBOARD_NV21) {
            const uint8_t *p = s + plane + (size_t)(y / 2) * l->stride + (x & ~1U);
            u = p[l->fourcc == DASHBOARD_NV12 ? 0 : 1];
            v = p[l->fourcc == DASHBOARD_NV12 ? 1 : 0];
        } else {
            size_t offset = (size_t)(y / 2) * (l->stride / 2) + x / 2;
            const uint8_t *p1 = s + plane + offset;
            const uint8_t *p2 = p1 + plane / 4;
            u = *(l->fourcc == DASHBOARD_YUV420 ? p1 : p2);
            v = *(l->fourcc == DASHBOARD_YUV420 ? p2 : p1);
        }
    }
    return yuv_pixel(yy, u, v, l->matrix, l->range);
}

static int convert_region(const struct dashboard_video_layout *l,
                            const uint8_t *s, size_t source_bytes,
                            uint32_t *dst, size_t destination_pixels,
                            unsigned ow, unsigned oh,
                            unsigned rx, unsigned ry, unsigned rw, unsigned rh)
{
    size_t required, pixels;
    unsigned width, height, left, top, x, y;
    if (!s || !dst || !ow || !oh || ow > 8192 || oh > 8192 ||
        dashboard_video_required_bytes(l, &required) || source_bytes < required ||
        !rw || !rh || rx > l->width || ry > l->height ||
        rw > l->width-rx || rh > l->height-ry ||
        ow > SIZE_MAX / oh) return -1;
    pixels = (size_t)ow * oh;
    if (pixels > destination_pixels || pixels > SIZE_MAX / sizeof(*dst)) return -1;
    width = ow;
    height = (unsigned)((uint64_t)ow * rh / rw);
    if (height > oh) {
        height = oh;
        width = (unsigned)((uint64_t)oh * rw / rh);
    }
    if (!width) width = 1;
    if (!height) height = 1;
    left = (ow - width) / 2; top = (oh - height) / 2;
    memset(dst, 0, pixels * sizeof(*dst));
    for (y = 0; y < height; ++y) {
        unsigned sy = ry + (unsigned)((uint64_t)y * rh / height);
        for (x = 0; x < width; ++x) {
            unsigned sx = rx + (unsigned)((uint64_t)x * rw / width);
            dst[(size_t)(top + y) * ow + left + x] = get_pixel(l, s, sx, sy);
        }
    }
    return 0;
}

int dashboard_video_convert(const struct dashboard_video_layout *l,
                            const uint8_t *s, size_t bytes, uint32_t *dst,
                            size_t pixels, unsigned ow, unsigned oh)
{
    if (!l) return -1;
    return convert_region(l,s,bytes,dst,pixels,ow,oh,0,0,l->width,l->height);
}

int dashboard_video_preview(const struct dashboard_video_layout *l,
                            const uint8_t *s, size_t bytes, uint32_t *dst,
                            size_t pixels, unsigned ow, unsigned oh, int yolo640)
{
    if (!l) return -1;
    if (!yolo640 || l->width!=640 || l->height!=640 || !is_rgb(l->fourcc))
        return dashboard_video_convert(l,s,bytes,dst,pixels,ow,oh);
    if (convert_region(l,s,bytes,dst,pixels,ow,oh,0,140,640,360)) return -1;
    unsigned w=ow,h=(unsigned)((uint64_t)ow*360/640);
    if (h>oh) { h=oh;w=(unsigned)((uint64_t)oh*640/360); }
    if (!w) w=1;
    if (!h) h=1;
    unsigned left=(ow-w)/2,top=(oh-h)/2;
    unsigned header=(unsigned)((uint64_t)40*w/640);
    if (header>h) header=h;
    /* 图像叠加仅绘制亮色字形及阴影，黑色填充保持透明。 */


    for (unsigned pass=0;pass<2;++pass) for (unsigned y=0;y<header;++y)
        for (unsigned x=0;x<w;++x) {
            uint32_t c=get_pixel(l,s,(unsigned)((uint64_t)x*640/w),
                                   (unsigned)((uint64_t)y*640/w));
            if (((c>>16)&255)<160 || ((c>>8)&255)<160 || (c&255)<160) continue;
            int dx=(int)(left+x),dy=(int)(top+y);
            if (pass) dst[(size_t)dy*ow+(unsigned)dx]=c;
            else for (int yy=dy-1;yy<=dy+1;++yy) for (int xx=dx-1;xx<=dx+1;++xx)
                if (xx>=0 && yy>=0 && (unsigned)xx<ow && (unsigned)yy<oh)
                    dst[(size_t)yy*ow+(unsigned)xx]=0;
        }
    return 0;
}

#ifdef __linux__
#include <fcntl.h>
#include <time.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

static int video_ioctl(int fd, unsigned long request, void *arg)
{
    int rc;
    do { rc = ioctl(fd, request, arg); } while (rc < 0 && errno == EINTR);
    return rc;
}

static int resolve_colorimetry(const struct v4l2_pix_format *p, int extended,
                               struct dashboard_video_layout *l)
{
    unsigned enc = extended ? p->ycbcr_enc : V4L2_YCBCR_ENC_DEFAULT;
    unsigned range = extended ? p->quantization : V4L2_QUANTIZATION_DEFAULT;
    if (!enc) enc = V4L2_MAP_YCBCR_ENC_DEFAULT(p->colorspace);
    if (is_rgb(l->fourcc)) {
        l->matrix = DASHBOARD_BT601; /* RGB不使用YCbCr矩阵。 */
    } else if (enc == V4L2_YCBCR_ENC_601) {
        l->matrix = DASHBOARD_BT601;
    } else if (enc == V4L2_YCBCR_ENC_709) {
        l->matrix = DASHBOARD_BT709;
    } else {
        fprintf(stderr, "Camera unsupported YCbCr encoding=%u colorspace=%u\n",
                enc, p->colorspace);
        return -1;
    }
    if (!range) range = is_rgb(l->fourcc) || p->colorspace == V4L2_COLORSPACE_JPEG
                       ? V4L2_QUANTIZATION_FULL_RANGE : V4L2_QUANTIZATION_LIM_RANGE;
    if (range == V4L2_QUANTIZATION_FULL_RANGE) l->range = DASHBOARD_FULL;
    else if (range == V4L2_QUANTIZATION_LIM_RANGE) l->range = DASHBOARD_LIMITED;
    else { fprintf(stderr, "Camera unsupported quantization=%u\n", range); return -1; }
    return 0;
}

int dashboard_video_open(struct dashboard_video *v, const char *path,
                         unsigned ow, unsigned oh)
{
    struct v4l2_capability cap;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    unsigned capabilities, i;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE, saved_errno;
    const char *operation = "initialization";
    if (!v) { errno = EINVAL; return -1; }
    memset(v, 0, sizeof(*v)); v->fd = -1;
    if (!path || !ow || !oh || ow > 8192 || oh > 8192 ||
        ow > SIZE_MAX / oh || (size_t)ow * oh > SIZE_MAX / sizeof(uint32_t)) {
        errno = EINVAL; return -1;
    }
    v->output_width = ow; v->output_height = oh;
    operation = "open";
    v->fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (v->fd < 0) goto fail;
    memset(&cap, 0, sizeof(cap)); operation = "VIDIOC_QUERYCAP";
    if (video_ioctl(v->fd, VIDIOC_QUERYCAP, &cap) < 0) goto fail;
    capabilities = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    if (!(capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "Camera %s must support single-planar streaming capture; producer ready?\n", path);
        errno = ENOTSUP; goto fail;
    }
    memset(&format, 0, sizeof(format)); format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (capabilities & V4L2_CAP_EXT_PIX_FORMAT) format.fmt.pix.priv = V4L2_PIX_FMT_PRIV_MAGIC;
    operation = "VIDIOC_G_FMT";
    if (video_ioctl(v->fd, VIDIOC_G_FMT, &format) < 0) goto fail;
    v->layout.width = format.fmt.pix.width; v->layout.height = format.fmt.pix.height;
    v->layout.stride = format.fmt.pix.bytesperline; v->layout.fourcc = format.fmt.pix.pixelformat;
    for (i = 0; i < 4; ++i) {
        unsigned c = (v->layout.fourcc >> (8 * i)) & 255U;
        v->format_name[i] = c >= 32 && c <= 126 ? (char)c : '?';
    }
    operation = "format validation";
    if (format.fmt.pix.field != V4L2_FIELD_NONE && format.fmt.pix.field != V4L2_FIELD_ANY) {
        fprintf(stderr, "Camera interlaced/alternate field format=%u is unsupported\n", format.fmt.pix.field);
        errno = ENOTSUP; goto fail;
    }
    if (resolve_colorimetry(&format.fmt.pix,
                            !!(capabilities & V4L2_CAP_EXT_PIX_FORMAT) &&
                            format.fmt.pix.priv == V4L2_PIX_FMT_PRIV_MAGIC, &v->layout) < 0 ||
        dashboard_video_required_bytes(&v->layout, &v->required_bytes) < 0 ||
        format.fmt.pix.sizeimage < v->required_bytes ||
        format.fmt.pix.sizeimage > (size_t)256 * 1024 * 1024) {
        fprintf(stderr, "Camera unsupported/invalid format: %s %ux%u stride=%u sizeimage=%u\n",
                v->format_name, v->layout.width, v->layout.height, v->layout.stride,
                format.fmt.pix.sizeimage);
        errno = EINVAL; goto fail;
    }
    v->pixels = calloc((size_t)ow * oh, sizeof(*v->pixels));
    if (!v->pixels) { operation = "output allocation"; goto fail; }
    memset(&req, 0, sizeof(req)); req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    operation = "VIDIOC_REQBUFS";
    if (video_ioctl(v->fd, VIDIOC_REQBUFS, &req) < 0) goto fail;
    if (!req.count || req.count > 8) { errno = ENOBUFS; goto fail; }
    v->buffer_count = req.count;
    for (i = 0; i < v->buffer_count; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf)); buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP; buf.index = i;
        operation = "VIDIOC_QUERYBUF";
        if (video_ioctl(v->fd, VIDIOC_QUERYBUF, &buf) < 0) goto fail;
        if (buf.length < format.fmt.pix.sizeimage || buf.length > (size_t)256 * 1024 * 1024) {
            errno = EOVERFLOW; goto fail;
        }
        operation = "mmap";
        v->maps[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v->fd, buf.m.offset);
        if (v->maps[i] == MAP_FAILED) { v->maps[i] = NULL; goto fail; }
        v->map_lengths[i] = buf.length;
        operation = "VIDIOC_QBUF";
        if (video_ioctl(v->fd, VIDIOC_QBUF, &buf) < 0) goto fail;
    }
    operation = "VIDIOC_STREAMON";
    if (video_ioctl(v->fd, VIDIOC_STREAMON, &type) < 0) goto fail;
    v->streaming = 1;
    fprintf(stderr, "Dashboard camera=%s format=%s %ux%u stride=%u matrix=BT.%s range=%s buffers=%u\n",
            path, v->format_name, v->layout.width, v->layout.height, v->layout.stride,
            v->layout.matrix == DASHBOARD_BT709 ? "709" : "601",
            v->layout.range == DASHBOARD_FULL ? "full" : "limited", v->buffer_count);
    return 0;
fail:
    saved_errno = errno;
    fprintf(stderr, "Camera %s %s: %s\n", path, operation, strerror(saved_errno));
    dashboard_video_close(v); errno = saved_errno;
    return -1;
}

int dashboard_video_update(struct dashboard_video *v, uint64_t now_ms)
{
    struct v4l2_buffer latest;
    unsigned attempt;
    int have_latest = 0;
    if (!v || v->fd < 0 || !v->streaming) { errno = EINVAL; return -1; }
    memset(&latest, 0, sizeof(latest));
    /* 持有最新出队缓冲直到转换完成，旧缓冲重新入队。若回环驱动在write()路径独立回收内存，仍需上板检查撕裂。限制排空数量，避免高速生产者挤占IMU、控制和事件处理。



 */
    for (attempt = 0; attempt < 8; ++attempt) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf)); buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (video_ioctl(v->fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) break;
            fprintf(stderr, "Camera VIDIOC_DQBUF: %s\n", strerror(errno));
            return -1;
        }
        if (buf.index >= v->buffer_count || !v->maps[buf.index] ||
            buf.bytesused > v->map_lengths[buf.index] || buf.bytesused < v->required_bytes) {
            fprintf(stderr, "Camera invalid capture buffer index=%u bytesused=%u required=%zu\n",
                    buf.index, buf.bytesused, v->required_bytes);
            errno = EPROTO; return -1;
        }
        if (buf.flags & V4L2_BUF_FLAG_ERROR) {
            if (video_ioctl(v->fd, VIDIOC_QBUF, &buf) < 0) return -1;
            continue;
        }
        if (have_latest && video_ioctl(v->fd, VIDIOC_QBUF, &latest) < 0) {
            fprintf(stderr, "Camera VIDIOC_QBUF: %s\n", strerror(errno)); return -1;
        }
        latest = buf; have_latest = 1;
    }
    if (!have_latest) return 0;
    const char *fit=getenv("ACG_PHONE_VIDEO_FIT");
    if (dashboard_video_preview(&v->layout, v->maps[latest.index], latest.bytesused,
                                v->pixels, (size_t)v->output_width * v->output_height,
                                v->output_width, v->output_height,
                                fit && !strcmp(fit,"yolo640")) < 0) {
        fprintf(stderr, "Camera frame conversion failed validation\n"); errno = EPROTO; return -1;
    }
    extern void phone_export_raw(const struct dashboard_video_layout*, const void*, size_t, uint64_t) __attribute__((weak));
    if (phone_export_raw) phone_export_raw(&v->layout,v->maps[latest.index],latest.bytesused,now_ms);
    if (video_ioctl(v->fd, VIDIOC_QBUF, &latest) < 0) {
        fprintf(stderr, "Camera VIDIOC_QBUF: %s\n", strerror(errno)); return -1;
    }
    ++v->frames; v->last_frame_ms = now_ms;
    return 1;
}

void dashboard_video_close(struct dashboard_video *v)
{
    unsigned i;
    if (!v) return;
    if (v->fd >= 0 && v->streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)video_ioctl(v->fd, VIDIOC_STREAMOFF, &type);
    }
    for (i = 0; i < v->buffer_count && i < 8; ++i)
        if (v->maps[i]) (void)munmap(v->maps[i], v->map_lengths[i]);
    if (v->fd >= 0) (void)close(v->fd);
    free(v->pixels);
    memset(v, 0, sizeof(*v)); v->fd = -1;
}
#else
int dashboard_video_open(struct dashboard_video *v, const char *path,
                         unsigned ow, unsigned oh)
{
    (void)path; (void)ow; (void)oh;
    if (v) { memset(v, 0, sizeof(*v)); v->fd = -1; }
    errno = ENOSYS; return -1;
}
int dashboard_video_update(struct dashboard_video *v, uint64_t now_ms)
{
    (void)v; (void)now_ms; errno = ENOSYS; return -1;
}
void dashboard_video_close(struct dashboard_video *v)
{
    if (v) { free(v->pixels); memset(v, 0, sizeof(*v)); v->fd = -1; }
}
#endif
