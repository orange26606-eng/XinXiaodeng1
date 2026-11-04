/* 显示像素路径主机回归，独立核对颜色公式和缓冲边界，不模拟真实V4L2内核或设备，也不作为硬件显示验证。



 */
#include "dashboard_video.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

static unsigned reference_clamp(double value)
{
    return value <= 0.0 ? 0U : value >= 255.0 ? 255U : (unsigned)(value + 0.5);
}

/* 按标称亮度和色度范围归一化的独立BT.601及BT.709公式。 */
static uint32_t reference_yuv(unsigned y, unsigned u, unsigned v,
                               enum dashboard_matrix matrix,
                               enum dashboard_range range)
{
    double kr = matrix == DASHBOARD_BT709 ? 0.2126 : 0.299;
    double kb = matrix == DASHBOARD_BT709 ? 0.0722 : 0.114;
    double kg = 1.0 - kr - kb;
    double luma = range == DASHBOARD_FULL ? y / 255.0 : ((double)y - 16) / 219.0;
    double cb = ((double)u - 128) / (range == DASHBOARD_FULL ? 255.0 : 224.0);
    double cr = ((double)v - 128) / (range == DASHBOARD_FULL ? 255.0 : 224.0);
    unsigned r = reference_clamp(255 * (luma + 2 * (1 - kr) * cr));
    unsigned b = reference_clamp(255 * (luma + 2 * (1 - kb) * cb));
    unsigned g = reference_clamp(255 * (luma - 2 * kb * (1 - kb) * cb / kg -
                                      2 * kr * (1 - kr) * cr / kg));
    return (r << 16) | (g << 8) | b;
}

static void close_color(uint32_t got, uint32_t expected)
{
    unsigned shift;
    CHECK((got & 0xff000000U) == 0);
    for (shift = 0; shift < 24; shift += 8) {
        int difference = (int)((got >> shift) & 255) - (int)((expected >> shift) & 255);
        CHECK(difference >= -2 && difference <= 2);
    }
}

static void fill_yuv(const struct dashboard_video_layout *l, uint8_t *s,
                      unsigned y, unsigned u, unsigned v)
{
    unsigned row, x;
    size_t plane = (size_t)l->stride * l->height;
    memset(s, 0xa5, 128);
    for (row = 0; row < l->height; ++row) {
        if (l->fourcc == DASHBOARD_YUYV || l->fourcc == DASHBOARD_UYVY) {
            for (x = 0; x < l->width; x += 2) {
                uint8_t *p = s + row * l->stride + x * 2;
                if (l->fourcc == DASHBOARD_YUYV) {
                    p[0] = (uint8_t)y; p[1] = (uint8_t)u;
                    p[2] = (uint8_t)y; p[3] = (uint8_t)v;
                } else {
                    p[0] = (uint8_t)u; p[1] = (uint8_t)y;
                    p[2] = (uint8_t)v; p[3] = (uint8_t)y;
                }
            }
        } else memset(s + row * l->stride, (int)y, l->width);
    }
    if (l->fourcc == DASHBOARD_NV12 || l->fourcc == DASHBOARD_NV21) {
        for (row = 0; row < l->height / 2; ++row)
            for (x = 0; x < l->width; x += 2) {
                uint8_t *p = s + plane + row * l->stride + x;
                p[0] = (uint8_t)(l->fourcc == DASHBOARD_NV12 ? u : v);
                p[1] = (uint8_t)(l->fourcc == DASHBOARD_NV12 ? v : u);
            }
    } else if (l->fourcc == DASHBOARD_YUV420 || l->fourcc == DASHBOARD_YVU420) {
        for (row = 0; row < l->height / 2; ++row)
            for (x = 0; x < l->width / 2; ++x) {
                size_t offset = plane + row * (l->stride / 2) + x;
                s[offset] = (uint8_t)(l->fourcc == DASHBOARD_YUV420 ? u : v);
                s[offset + plane / 4] = (uint8_t)(l->fourcc == DASHBOARD_YUV420 ? v : u);
            }
    }
}

static void test_yuv_color_and_padding(void)
{
    const uint32_t formats[] = { DASHBOARD_NV12, DASHBOARD_NV21, DASHBOARD_YUV420,
                                 DASHBOARD_YVU420, DASHBOARD_YUYV, DASHBOARD_UYVY };
    const unsigned samples[] = { 0, 1, 16, 32, 64, 81, 90, 128, 145, 160, 192, 235, 240, 254, 255 };
    unsigned format, matrix, range, y, u, v, k;
    for (format = 0; format < sizeof(formats) / sizeof(formats[0]); ++format) {
        struct dashboard_video_layout l = { formats[format], 2, 2,
                                            format >= 4 ? 8U : 6U,
                                            DASHBOARD_BT601, DASHBOARD_LIMITED };
        uint8_t source[128], saved[128];
        uint32_t dst[6];
        size_t required;
        CHECK(dashboard_video_required_bytes(&l, &required) == 0);
        CHECK(required == (format >= 4 ? 16U : 18U));
        for (matrix = DASHBOARD_BT601; matrix <= DASHBOARD_BT709; ++matrix)
            for (range = DASHBOARD_LIMITED; range <= DASHBOARD_FULL; ++range) {
                l.matrix = (enum dashboard_matrix)matrix;
                l.range = (enum dashboard_range)range;
                for (y = 0; y < sizeof(samples) / sizeof(samples[0]); ++y)
                    for (u = 0; u < sizeof(samples) / sizeof(samples[0]); ++u)
                        for (v = 0; v < sizeof(samples) / sizeof(samples[0]); ++v) {
                            uint32_t expected = reference_yuv(samples[y], samples[u], samples[v],
                                                               l.matrix, l.range);
                            fill_yuv(&l, source, samples[y], samples[u], samples[v]);
                            memcpy(saved, source, sizeof(saved));
                            dst[0] = 0xabcddcba; dst[5] = 0x12345678;
                            CHECK(dashboard_video_convert(&l, source, required, dst + 1, 4, 2, 2) == 0);
                            for (k = 1; k <= 4; ++k) close_color(dst[k], expected);
                            CHECK(dst[0] == 0xabcddcba && dst[5] == 0x12345678);
                            CHECK(memcmp(source, saved, sizeof(saved)) == 0);
                        }
            }
    }
}

static void test_chroma_plane_offsets(void)
{
    const uint32_t formats[] = { DASHBOARD_NV12, DASHBOARD_NV21, DASHBOARD_YUV420, DASHBOARD_YVU420 };
    const unsigned uv[4][2] = { { 90, 240 }, { 54, 34 }, { 240, 110 }, { 128, 128 } };
    unsigned f, x, y;
    for (f = 0; f < 4; ++f) {
        struct dashboard_video_layout l = { formats[f], 4, 4, 8, DASHBOARD_BT601, DASHBOARD_LIMITED };
        uint8_t s[48]; uint32_t out[16];
        memset(s, 0xee, sizeof(s));
        for (y = 0; y < 4; ++y) for (x = 0; x < 4; ++x) s[y * 8 + x] = (uint8_t)(40 + y * 20 + x);
        for (y = 0; y < 2; ++y) for (x = 0; x < 2; ++x) {
            unsigned u = uv[y * 2 + x][0], v = uv[y * 2 + x][1];
            if (f < 2) {
                s[32 + y * 8 + x * 2] = (uint8_t)(f == 0 ? u : v);
                s[33 + y * 8 + x * 2] = (uint8_t)(f == 0 ? v : u);
            } else {
                s[32 + y * 4 + x] = (uint8_t)(f == 2 ? u : v);
                s[40 + y * 4 + x] = (uint8_t)(f == 2 ? v : u);
            }
        }
        CHECK(dashboard_video_convert(&l, s, sizeof(s), out, 16, 4, 4) == 0);
        for (y = 0; y < 4; ++y) for (x = 0; x < 4; ++x)
            close_color(out[y * 4 + x], reference_yuv(40 + y * 20 + x,
                        uv[(y / 2) * 2 + x / 2][0], uv[(y / 2) * 2 + x / 2][1],
                        DASHBOARD_BT601, DASHBOARD_LIMITED));
    }
}

static void test_rgb_and_letterbox(void)
{
    struct dashboard_video_layout l = { DASHBOARD_RGB24, 4, 2, 16, DASHBOARD_BT601, DASHBOARD_FULL };
    uint8_t source[32]; uint32_t out[18]; unsigned i;
    const uint32_t colors[] = { 0xff0000, 0x00ff00, 0x0000ff, 0xffffff,
                                0x102030, 0x405060, 0x708090, 0xa0b0c0 };
    memset(source, 0xe9, sizeof(source));
    for (i = 0; i < 8; ++i) {
        unsigned offset = (i / 4) * 16 + (i % 4) * 3;
        source[offset] = (uint8_t)(colors[i] >> 16);
        source[offset + 1] = (uint8_t)(colors[i] >> 8);
        source[offset + 2] = (uint8_t)colors[i];
    }
    out[0] = 0x12345678; out[17] = 0x87654321;
    CHECK(dashboard_video_convert(&l, source, sizeof(source), out + 1, 16, 4, 4) == 0);
    for (i = 1; i <= 4; ++i) CHECK(out[i] == 0);
    for (i = 0; i < 8; ++i) CHECK(out[i + 5] == colors[i]);
    for (i = 13; i <= 16; ++i) CHECK(out[i] == 0);
    CHECK(out[0] == 0x12345678 && out[17] == 0x87654321);
    CHECK(dashboard_video_convert(&l, source, sizeof(source), out, 2, 2, 1) == 0);
    CHECK(out[0] == colors[0] && out[1] == colors[2]);
    /* BGR字节顺序、带填充行及有限范围RGB端点。 */
    l.fourcc = DASHBOARD_BGR24; l.width = 2; l.height = 1; l.stride = 8;
    source[0] = 255; source[1] = 100; source[2] = 20;
    source[3] = 0; source[4] = 64; source[5] = 128;
    CHECK(dashboard_video_convert(&l, source, 8, out, 2, 2, 1) == 0);
    CHECK(out[0] == 0x1464ff && out[1] == 0x804000);
    l.range = DASHBOARD_LIMITED;
    memset(source, 16, 3); memset(source + 3, 235, 3);
    CHECK(dashboard_video_convert(&l, source, 8, out, 2, 2, 1) == 0);
    CHECK(out[0] == 0 && out[1] == 0xffffff);
    /* 竖向源图适配横向输出，并检查单像素输出。 */
    l.range = DASHBOARD_FULL; l.width = 1; l.height = 2; l.stride = 4;
    memset(source, 255, 8);
    CHECK(dashboard_video_convert(&l, source, 8, out, 8, 4, 2) == 0);
    CHECK(out[0] == 0 && out[1] == 0xffffff && out[2] == 0 && out[3] == 0);
    CHECK(out[4] == 0 && out[5] == 0xffffff && out[6] == 0 && out[7] == 0);
    CHECK(dashboard_video_convert(&l, source, 8, out, 1, 1, 1) == 0);
    CHECK(out[0] == 0xffffff);
}

static void test_packed_luma_positions(void)
{
    uint8_t yuyv[24] = { 16, 90, 81, 240, 145, 54, 235, 34, 0xee, 0xee, 0xee, 0xee,
                         235, 240, 145, 110, 81, 128, 16, 128, 0xee, 0xee, 0xee, 0xee };
    uint8_t uyvy[24];
    const unsigned luma[] = { 16, 81, 145, 235, 235, 145, 81, 16 };
    const unsigned u[] = { 90, 90, 54, 54, 240, 240, 128, 128 };
    const unsigned v[] = { 240, 240, 34, 34, 110, 110, 128, 128 };
    struct dashboard_video_layout l = { DASHBOARD_YUYV, 4, 2, 12, DASHBOARD_BT709, DASHBOARD_FULL };
    uint32_t out[8]; unsigned i;
    for (i = 0; i < sizeof(yuyv); i += 2) { uyvy[i] = yuyv[i + 1]; uyvy[i + 1] = yuyv[i]; }
    CHECK(dashboard_video_convert(&l, yuyv, sizeof(yuyv), out, 8, 4, 2) == 0);
    for (i = 0; i < 8; ++i) close_color(out[i], reference_yuv(luma[i], u[i], v[i], l.matrix, l.range));
    l.fourcc = DASHBOARD_UYVY;
    CHECK(dashboard_video_convert(&l, uyvy, sizeof(uyvy), out, 8, 4, 2) == 0);
    for (i = 0; i < 8; ++i) close_color(out[i], reference_yuv(luma[i], u[i], v[i], l.matrix, l.range));
}

static void test_rejections(void)
{
    struct dashboard_video_layout good = { DASHBOARD_NV12, 4, 4, 8, DASHBOARD_BT601, DASHBOARD_LIMITED };
    struct dashboard_video_layout l;
    uint8_t source[48] = {0}; uint32_t out[16], saved[16]; size_t required; unsigned i;
    for (i = 0; i < 16; ++i) out[i] = saved[i] = 0xdeadbeef;
    CHECK(dashboard_video_convert(&good, source, sizeof(source) - 1, out, 16, 4, 4) == -1);
    CHECK(memcmp(out, saved, sizeof(out)) == 0);
    CHECK(dashboard_video_convert(&good, source, sizeof(source), out, 15, 4, 4) == -1);
    CHECK(memcmp(out, saved, sizeof(out)) == 0);
    CHECK(dashboard_video_convert(NULL, source, sizeof(source), out, 16, 4, 4) == -1);
    CHECK(dashboard_video_convert(&good, NULL, sizeof(source), out, 16, 4, 4) == -1);
    CHECK(dashboard_video_convert(&good, source, sizeof(source), NULL, 16, 4, 4) == -1);
    CHECK(dashboard_video_convert(&good, source, sizeof(source), out, SIZE_MAX, UINT_MAX, UINT_MAX) == -1);
    CHECK(dashboard_video_convert(&good, source, sizeof(source), out, 16, 0, 4) == -1);
    CHECK(dashboard_video_required_bytes(NULL, &required) == -1);
    CHECK(dashboard_video_required_bytes(&good, NULL) == -1);
    l = good; l.width = 3; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.height = 3; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.stride = 7; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.stride = 2; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.stride = UINT_MAX - 1; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.width = UINT_MAX; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.height = 0; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.fourcc = 0; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.matrix = (enum dashboard_matrix)9; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.range = (enum dashboard_range)9; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.fourcc = DASHBOARD_YUYV; l.width = 3; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    l = good; l.fourcc = DASHBOARD_RGB24; l.stride = 11; CHECK(dashboard_video_required_bytes(&l, &required) == -1);
    CHECK(memcmp(out, saved, sizeof(out)) == 0);
}

int main(void)
{
    test_yuv_color_and_padding();
    test_chroma_plane_offsets();
    test_rgb_and_letterbox();
    test_packed_luma_positions();
    test_rejections();
    printf("PASS dashboard_video: %u checks; 8 formats, stride/planes, color/range, fit and buffer bounds\n", checks);
    return 0;
}
