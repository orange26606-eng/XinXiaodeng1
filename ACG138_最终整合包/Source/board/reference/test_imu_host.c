/* 主机协议与状态测试，包含独立CRC标准值、故障及画布边界检查；1.1.1按浅色绘制器更新颜色断言。
 */
#include "imu_canvas.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(expression) do { ++checks; if (!(expression)) { ++failures; \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expression); } } while (0)
/* 小端序号0x12345678；横滚pi/2、俯仰-pi/4、偏航pi；CRC由Python zlib独立计算。 */
static const uint8_t golden[32] = {
    0x49,0x4D,0x55,0x31,0x01,0x00,0x00,0x00,0x78,0x56,0x34,0x12,
    0xDB,0x0F,0xC9,0x3F,0xDB,0x0F,0x49,0xBF,0xDB,0x0F,0x49,0x40,
    0x00,0x00,0x00,0x00,0xD4,0x9E,0x9D,0x8B
};

static void put_u32(uint8_t *p, uint32_t value)
{ for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (i * 8)); }
static void fix_crc(uint8_t *p) { put_u32(p + 28, imu_crc32(p, 28)); }
static void packet(uint8_t *p, uint32_t sequence, uint8_t status)
{
    memcpy(p, golden, 32); put_u32(p + 8, sequence); p[5] = status;
    if (status) memset(p + 12, 0, 12);
    fix_crc(p);
}

static void test_crc(void)
{
    CHECK(imu_crc32((const uint8_t *)"123456789", 9) == UINT32_C(0xCBF43926));
    CHECK(imu_crc32((const uint8_t *)"", 0) == 0);
    CHECK(imu_crc32(golden, 28) == UINT32_C(0x8B9D9ED4));
}

static void test_decode(void)
{
    struct imu_frame frame;
    CHECK(imu_decode(golden, 32, &frame) == IMU_DECODE_OK);
    CHECK(frame.sequence == UINT32_C(0x12345678) && frame.status == 0);
    CHECK(fabs((double)frame.radians[0] - 1.5707963267948966) < 1e-6);
    CHECK(fabs((double)frame.radians[1] + 0.7853981633974483) < 1e-6);
    CHECK(fabs((double)frame.radians[2] - 3.1415926535897932) < 1e-6);
    uint8_t p[32];
    CHECK(imu_decode(golden, 0, &frame) == IMU_BAD_LENGTH);
    CHECK(imu_decode(golden, 31, &frame) == IMU_BAD_LENGTH);
    CHECK(imu_decode(golden, 33, &frame) == IMU_BAD_LENGTH);
    memcpy(p, golden, 32); p[0] = 'X'; CHECK(imu_decode(p, 32, &frame) == IMU_BAD_MAGIC);
    memcpy(p, golden, 32); p[4] = 2; CHECK(imu_decode(p, 32, &frame) == IMU_BAD_VERSION);
    for (unsigned i = 6; i < 28; ++i) if (i < 8 || i >= 24) {
        memcpy(p, golden, 32); p[i] = 1; fix_crc(p);
        CHECK(imu_decode(p, 32, &frame) == IMU_BAD_RESERVED);
    }
    memcpy(p, golden, 32); p[5] = 6; fix_crc(p);
    CHECK(imu_decode(p, 32, &frame) == IMU_BAD_STATUS);
    for (unsigned byte = 8; byte < 24; ++byte) {
        memcpy(p, golden, 32); p[byte] ^= 1;
        CHECK(imu_decode(p, 32, &frame) == IMU_BAD_CRC);
    }
    memcpy(p, golden, 32); p[31] ^= 1;
    CHECK(imu_decode(p, 32, &frame) == IMU_BAD_CRC);
    uint32_t nonfinite[] = {UINT32_C(0x7FC00000), UINT32_C(0x7F800000), UINT32_C(0xFF800000)};
    for (unsigned axis = 0; axis < 3; ++axis) for (unsigned n = 0; n < 3; ++n) {
        memcpy(p, golden, 32); put_u32(p + 12 + axis * 4, nonfinite[n]); fix_crc(p);
        CHECK(imu_decode(p, 32, &frame) == IMU_BAD_FLOAT);
    }
    for (unsigned status = 1; status <= 5; ++status) {
        packet(p, 1, (uint8_t)status);
        CHECK(imu_decode(p, 32, &frame) == IMU_DECODE_OK);
        CHECK(frame.status == status && frame.radians[0] == 0 &&
              frame.radians[1] == 0 && frame.radians[2] == 0);
        p[12] = 1; fix_crc(p);
        CHECK(imu_decode(p, 32, &frame) == IMU_BAD_ERROR_PAYLOAD);
    }
    memset(&frame, 0xA5, sizeof(frame));
    struct imu_frame untouched;
    memcpy(&untouched, &frame, sizeof(frame));
    memcpy(p, golden, 32); p[31] ^= 1;
    CHECK(imu_decode(p, 32, &frame) == IMU_BAD_CRC);
    CHECK(!memcmp(&frame, &untouched, sizeof(frame)));
}

static void test_model(void)
{
    struct imu_model model;
    uint8_t p[32];
    imu_model_init(&model, 0);
    CHECK(imu_model_state(&model, 0) == IMU_WAITING);
    CHECK(imu_model_state(&model, 5999) == IMU_WAITING);
    CHECK(imu_model_state(&model, 6000) == IMU_STALE);
    packet(p, 1, 1); imu_model_push(&model, p, 32, 100);
    CHECK(!model.have_angles && model.error_count == 1);
    CHECK(imu_model_state(&model, 999) == IMU_DEVICE_ERROR);
    CHECK(imu_model_state(&model, 1000) == IMU_STALE);
    packet(p, 2, 0); CHECK(imu_model_push(&model, p, 32, 1100) == 1);
    CHECK(fabs(model.degrees[0] - 90.0) < 0.00001);
    CHECK(fabs(model.degrees[1] + 45.0) < 0.00001);
    CHECK(fabs(model.degrees[2] - 180.0) < 0.00001);
    double previous[3]; memcpy(previous, model.degrees, sizeof(previous));
    for (unsigned status = 1; status <= 5; ++status) {
        packet(p, 2 + status, (uint8_t)status);
        CHECK(imu_model_push(&model, p, 32, 1100 + status * 40) == 0);
        CHECK(!memcmp(previous, model.degrees, sizeof(previous)));
        CHECK(model.good_count == 1 && model.status == status);
    }
    CHECK(imu_model_state(&model, 2099) == IMU_DEVICE_ERROR);
    CHECK(imu_model_state(&model, 2100) == IMU_STALE);
    packet(p, 8, 0); p[31] ^= 1; imu_model_push(&model, p, 32, 1400);
    CHECK(model.invalid_count == 1 && model.sequence == 7);
    CHECK(imu_model_state(&model, 1400) == IMU_INVALID);
    CHECK(!memcmp(previous, model.degrees, sizeof(previous)));
    packet(p, 8, 0); CHECK(imu_model_push(&model, p, 32, 1500) == 1);
    CHECK(imu_model_push(&model, p, 32, 1540) == 0);
    packet(p, 7, 0); CHECK(imu_model_push(&model, p, 32, 1580) == 0);
    CHECK(model.sequence == 8 && model.order_count == 2);
    packet(p, 11, 0); CHECK(imu_model_push(&model, p, 32, 1600) == 1);
    CHECK(model.missing_count == 2);
    packet(p, 0, 0); CHECK(imu_model_push(&model, p, 32, 1640) == 0);
    CHECK(model.sequence == 11); /* 发送端复位后显式重启接收器。 */
    imu_model_init(&model, 2000);
    packet(p, UINT32_C(0xFFFFFFFE), 0); CHECK(imu_model_push(&model, p, 32, 2000) == 1);
    packet(p, UINT32_C(0xFFFFFFFF), 0); CHECK(imu_model_push(&model, p, 32, 2040) == 1);
    packet(p, 0, 0); CHECK(imu_model_push(&model, p, 32, 2080) == 1);
    packet(p, 1, 0); CHECK(imu_model_push(&model, p, 32, 2120) == 1);
    CHECK(model.missing_count == 0 && model.order_count == 0);
    memset(p + 12, 0, 12); put_u32(p + 12, UINT32_C(0x80000000));
    put_u32(p + 8, 2); fix_crc(p); CHECK(imu_model_push(&model, p, 32, 2200) == 1);
    CHECK(signbit(model.degrees[0]) && model.degrees[1] == 0 && model.degrees[2] == 0);
    CHECK(imu_model_state(&model, 3199) == IMU_LIVE);
    CHECK(imu_model_state(&model, 3200) == IMU_STALE);
}

static int write_ppm(const char *name, const uint32_t *pixels)
{
    FILE *out = fopen(name, "wb");
    if (!out) return -1;
    fprintf(out, "P6\n%d %d\n255\n", IMU_CANVAS_WIDTH, IMU_CANVAS_HEIGHT);
    for (size_t i = 0; i < (size_t)IMU_CANVAS_WIDTH * IMU_CANVAS_HEIGHT; ++i) {
        unsigned char rgb[] = {(unsigned char)(pixels[i] >> 16),
                              (unsigned char)(pixels[i] >> 8), (unsigned char)pixels[i]};
        if (fwrite(rgb, 1, 3, out) != 3) { fclose(out); return -1; }
    }
    return fclose(out);
}

static void test_canvas(int render)
{
    const size_t count = (size_t)IMU_CANVAS_WIDTH * IMU_CANVAS_HEIGHT;
    uint32_t *storage = malloc((count + 2) * sizeof(uint32_t));
    CHECK(storage != NULL);
    if (!storage) return;
    uint32_t *pixels = storage + 1;
    storage[0] = storage[count + 1] = UINT32_C(0xDEADBEEF);
    struct imu_model model;
    imu_model_init(&model, 0);
    CHECK(imu_model_push(&model, golden, 32, 100) == 1);
    model.simulated = 1;
    imu_canvas_draw(pixels, &model, 140);
    CHECK(storage[0] == UINT32_C(0xDEADBEEF) && storage[count + 1] == UINT32_C(0xDEADBEEF));
    size_t white = 0, green = 0, red = 0;
    for (size_t i = 0; i < count; ++i) {
        white += pixels[i] == 0x00FCFCFD;
        green += pixels[i] == 0x0024764B;
    }
    CHECK(white > 100 && green > 100);
    if (render) CHECK(write_ppm("panel_live.ppm", pixels) == 0);
    imu_canvas_draw(pixels, &model, 1100);
    for (size_t i = 0; i < count; ++i) red += pixels[i] == 0x00915E09;
    CHECK(red > 100);
    if (render) CHECK(write_ppm("panel_stale.ppm", pixels) == 0);
    model.degrees[0] = 1e30; model.degrees[1] = -1e30; model.degrees[2] = 1e100;
    model.good_count = model.error_count = model.invalid_count = UINT64_MAX;
    imu_canvas_draw(pixels, &model, 1140);
    CHECK(storage[0] == UINT32_C(0xDEADBEEF) && storage[count + 1] == UINT32_C(0xDEADBEEF));
    free(storage);
}

static void test_rtl_capture(const char *filename)
{
    /* 期望值来自IIC到GMII仿真的固定sensor_pose输入，不由抓取输出反推。
 */
    static const uint32_t expected_bits[6][3] = {
        {0x3F800000,0xBF000000,0x00000000},
        {0x3F800000,0xBF000000,0x00000000},
        {0xC0000000,0x3F000000,0x3FC90FDB},
        {0,0,0}, {0,0,0}, {0x3F000000,0xC0400000,0xBFC90FDB}
    };
    FILE *source = fopen(filename, "r");
    CHECK(source != NULL);
    if (!source) return;
    uint8_t payloads[6][32] = {{0}};
    uint32_t masks[6] = {0};
    char line[256];
    CHECK(fgets(line, sizeof(line), source) != NULL);
    while (fgets(line, sizeof(line), source)) {
        unsigned instance, epoch, frame, index, value;
        int parsed = sscanf(line, "%u,%u,%u,%u,%x", &instance, &epoch, &frame, &index, &value);
        CHECK(parsed == 5);
        if (parsed != 5) continue;
        if (instance || epoch || frame >= 6 || index < 50 || index >= 82) continue;
        CHECK(value <= 255);
        unsigned offset = index - 50;
        CHECK(!(masks[frame] & (UINT32_C(1) << offset)));
        masks[frame] |= UINT32_C(1) << offset;
        payloads[frame][offset] = (uint8_t)value;
    }
    CHECK(!ferror(source));
    fclose(source);
    struct imu_model model;
    imu_model_init(&model, 0);
    double previous[3] = {0};
    for (unsigned i = 0; i < 6; ++i) {
        CHECK(masks[i] == UINT32_MAX);
        struct imu_frame frame = {0};
        CHECK(imu_decode(payloads[i], 32, &frame) == IMU_DECODE_OK);
        CHECK(frame.sequence == i);
        unsigned status = i == 3 || i == 4 ? 1 : 0;
        CHECK(frame.status == status);
        for (unsigned axis = 0; axis < 3; ++axis) {
            uint32_t actual_bits;
            memcpy(&actual_bits, &frame.radians[axis], sizeof(actual_bits));
            CHECK(actual_bits == expected_bits[i][axis]);
        }
        CHECK(imu_model_push(&model, payloads[i], 32, 100 + i * 40) == !status);
        if (status) CHECK(!memcmp(previous, model.degrees, sizeof(previous)));
        else memcpy(previous, model.degrees, sizeof(previous));
    }
    CHECK(model.good_count == 4 && model.error_count == 2);
    CHECK(model.invalid_count == 0 && model.missing_count == 0 && model.order_count == 0);
    CHECK(fabs(model.degrees[0] - 28.6478897565) < 0.0001);
    CHECK(fabs(model.degrees[1] + 171.887338539) < 0.0001);
    CHECK(fabs(model.degrees[2] + 90) < 0.0001);
    CHECK(imu_model_state(&model, 1300) == IMU_STALE);
}

int main(int argc, char **argv)
{
    int render = 0;
    const char *capture = NULL;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--render")) render = 1;
        else if (!strcmp(argv[i], "--capture") && i + 1 < argc) capture = argv[++i];
        else return 2;
    }
    test_crc(); test_decode(); test_model(); test_canvas(render);
    if (capture) test_rtl_capture(capture);
    printf("IMU HOST TEST: checks=%u failures=%u\n", checks, failures);
    return failures ? 1 : 0;
}
