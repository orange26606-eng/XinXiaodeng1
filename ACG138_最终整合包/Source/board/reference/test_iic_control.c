/* 主机端协议、会话和画布测试，设备使用桩。 */

#include "imu_iic_control.h"
#include "imu_protocol.h"
#include "imu_canvas.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks, failures;
#define CHECK(e) do { ++checks; if (!(e)) { ++failures; fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #e); } } while (0)
static const char *start_hex = "494d43310101000078563412000000000000000000000000000000007cbf3fc5";
static const char *confirm_hex = "494d43310102000078563412000000000000000000000000000000004186dab3";
static const char *ack_hex[4] = {
    "494d41310101000078563412093200000000000000000000000000008dea2f69",
    "494d4131010200007856341209320000000000000000000000000000b0d3ca1f",
    "494d41310103060078563412000000000000000000000000000000007a7d124f",
    "494d413101040000785634120000000000000000000000000000000054bf5a4b"
};
static void unhex(const char *hex, uint8_t *bytes, size_t length)
{
    CHECK(strlen(hex) == 2 * length);
    for (size_t i = 0; i < length; ++i) {
        unsigned value = 0;
        CHECK(sscanf(hex + i * 2, "%2x", &value) == 1); bytes[i] = (uint8_t)value;
    }
}
static void put_le32(uint8_t *p, uint32_t value)
{ for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8 * i)); }
static void crc(uint8_t p[32]) { put_le32(p + 28, imu_crc32(p, 28)); }
static void test_protocol(void)
{
    uint8_t expected[32], output[32], ack[32];
    unhex(start_hex, expected, 32);
    CHECK(imu_control_encode_command(IMU_CONTROL_START, 0x12345678, output) == 0);
    CHECK(!memcmp(output, expected, 32));
    unhex(confirm_hex, expected, 32);
    CHECK(imu_control_encode_command(IMU_CONTROL_CONFIRM_LED_STOPPED, 0x12345678, output) == 0);
    CHECK(!memcmp(output, expected, 32));
    memset(output, 0xA5, 32);
    CHECK(imu_control_encode_command(0, 1, output) == -1);
    CHECK(imu_control_encode_command(3, 1, output) == -1);
    for (unsigned i = 0; i < 32; ++i) CHECK(output[i] == 0xA5);
    CHECK(imu_control_encode_command(1, 1, NULL) == -1);
    for (unsigned state = 1; state <= 4; ++state) {
        unhex(ack_hex[state - 1], ack, 32);
        struct imu_control_ack decoded;
        CHECK(imu_control_decode_ack(ack, 32, &decoded) == IMU_CTRL_OK);
        CHECK(decoded.state == state && decoded.token == 0x12345678);
        CHECK(decoded.algorithm == (state <= 2 ? 9 : 0));
        CHECK(decoded.polling_hz == (state <= 2 ? 50 : 0));
    }
    struct imu_control_ack decoded = {0};
    unhex(ack_hex[0], ack, 32);
    CHECK(imu_control_decode_ack(ack, 31, &decoded) == IMU_CTRL_LENGTH);
    CHECK(imu_control_decode_ack(ack, 33, &decoded) == IMU_CTRL_LENGTH);
    CHECK(imu_control_decode_ack(NULL, 32, &decoded) == IMU_CTRL_LENGTH);
    CHECK(imu_control_decode_ack(ack, 32, NULL) == IMU_CTRL_LENGTH);
    uint8_t changed[32];
    memcpy(changed, ack, 32); changed[0] = 'X'; CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_MAGIC);
    memcpy(changed, ack, 32); changed[4] = 2; CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_VERSION);
    for (unsigned i = 7; i < 28; ++i) if (i == 7 || i >= 14) {
        memcpy(changed, ack, 32); changed[i] = 1; crc(changed);
        CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_RESERVED);
    }
    memcpy(changed, ack, 32); changed[5] = 0; crc(changed);
    CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_STATE);
    changed[5] = 5; crc(changed); CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_STATE);
    memcpy(changed, ack, 32); changed[6] = 7; crc(changed);
    CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_ERROR);
    changed[6] = 1; crc(changed); CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_ERROR);
    memcpy(changed, ack, 32); changed[12] = 6; crc(changed);
    CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_CONFIG);
    memcpy(changed, ack, 32); changed[13] = 25; crc(changed);
    CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_CONFIG);
    unhex(ack_hex[2], changed, 32); changed[6] = 0; crc(changed);
    CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_ERROR);
    unhex(ack_hex[3], changed, 32); changed[6] = 1; crc(changed);
    CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_ERROR);
    for (unsigned state = 3; state <= 4; ++state) {
        unhex(ack_hex[state - 1], changed, 32); changed[12] = 9; changed[13] = 50; crc(changed);
        CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_CONFIG);
    }
    for (unsigned i = 8; i < 12; ++i) {
        memcpy(changed, ack, 32); changed[i] ^= 1;
        CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_CRC);
    }
    memcpy(changed, ack, 32); changed[31] ^= 1;
    memset(&decoded, 0xA5, sizeof(decoded)); struct imu_control_ack untouched;
    memcpy(&untouched, &decoded, sizeof(decoded));
    CHECK(imu_control_decode_ack(changed, 32, &decoded) == IMU_CTRL_CRC);
    CHECK(!memcmp(&decoded, &untouched, sizeof(decoded)));
}

static void test_session(void)
{
    struct imu_control_session s;
    uint8_t command[32], first[32], wait[32], ready[32], fail[32], busy[32];
    unhex(ack_hex[0], wait, 32); unhex(ack_hex[1], ready, 32);
    unhex(ack_hex[2], fail, 32); unhex(ack_hex[3], busy, 32);
    imu_control_session_init(&s, 0x12345678, 0);
    CHECK(s.phase == IMU_PHASE_STARTING && !imu_control_session_user_confirm(&s, 0));
    CHECK(imu_control_session_next_tx(&s, 0, first) == 1 && s.attempts == 1);
    CHECK(imu_control_session_next_tx(&s, 999, command) == 0);
    CHECK(imu_control_session_next_tx(&s, 1000, command) == 1);
    CHECK(!memcmp(first, command, 32) && s.attempts == 2);
    CHECK(imu_control_session_receive(&s, ready, 32) == 0 && s.phase == IMU_PHASE_STARTING);
    uint8_t unrelated[32]; memcpy(unrelated, wait, 32); put_le32(unrelated + 8, 1); crc(unrelated);
    CHECK(imu_control_session_receive(&s, unrelated, 32) == 0);
    CHECK(s.unrelated_acks == 1 && s.phase == IMU_PHASE_STARTING);
    unrelated[31] ^= 1; CHECK(imu_control_session_receive(&s, unrelated, 32) == 0 && s.invalid_acks == 1);
    CHECK(imu_control_session_receive(&s, wait, 32) == 0 && s.phase == IMU_PHASE_WAIT_LED);
    CHECK(imu_control_session_next_tx(&s, 3600000, command) == 0 && s.phase == IMU_PHASE_WAIT_LED);
    CHECK(imu_control_session_receive(&s, ready, 32) == 0 && s.phase == IMU_PHASE_WAIT_LED);
    CHECK(imu_control_session_user_confirm(&s, 3600000) == 1 && s.phase == IMU_PHASE_CONFIRMING);
    CHECK(imu_control_session_user_confirm(&s, 3600000) == 0);
    CHECK(imu_control_session_next_tx(&s, 3600000, command) == 1 && command[5] == 2);
    CHECK(imu_control_session_receive(&s, wait, 32) == 0 && s.phase == IMU_PHASE_CONFIRMING);
    CHECK(imu_control_session_receive(&s, ready, 32) == 1 && s.phase == IMU_PHASE_STREAMING);
    CHECK(imu_control_session_receive(&s, ready, 32) == 0);
    CHECK(imu_control_session_receive(&s, wait, 32) == 0 && s.phase == IMU_PHASE_STREAMING);
    CHECK(imu_control_session_next_tx(&s, 99999999, command) == 0);
    CHECK(!imu_control_session_user_confirm(&s, 99999999));
    imu_control_session_init(&s, 0x12345678, 0);
    for (unsigned i = 0; i < 10; ++i) {
        CHECK(imu_control_session_next_tx(&s, i * 1000, command) == 1);
        CHECK(command[5] == 1 && !memcmp(command, first, 32));
    }
    CHECK(s.attempts == 10);
    CHECK(imu_control_session_next_tx(&s, 13999, command) == 0 && s.phase == IMU_PHASE_STARTING);
    CHECK(imu_control_session_next_tx(&s, 14000, command) == 0 && s.phase == IMU_PHASE_FAILED && s.timed_out);
    CHECK(imu_control_session_next_tx(&s, 30000, command) == 0 && s.attempts == 10);
    CHECK(imu_control_session_receive(&s, wait, 32) == 0 && s.phase == IMU_PHASE_FAILED);
    imu_control_session_init(&s, 0x12345678, 0);
    CHECK(imu_control_session_receive(&s, fail, 32) == 0 && s.phase == IMU_PHASE_FAILED && s.error == 6);
    imu_control_session_init(&s, 0x12345678, 0);
    CHECK(imu_control_session_receive(&s, busy, 32) == 0 && s.phase == IMU_PHASE_FAILED && s.busy);
    imu_control_session_init(&s, 1, 0);
    CHECK(imu_control_session_receive(&s, wait, 32) == 0 && s.phase == IMU_PHASE_STARTING);
    CHECK(imu_control_session_next_tx(&s, 0, command) == 1 && command[8] == 1);
    imu_control_session_io_failed(&s); CHECK(s.phase == IMU_PHASE_FAILED && s.local_error);
    imu_control_session_resume(&s, 0x12345678, 0);
    CHECK(s.resume_existing && s.phase == IMU_PHASE_STARTING);
    CHECK(imu_control_session_next_tx(&s, 0, command) == 1 && !memcmp(command, first, 32));
    CHECK(imu_control_session_receive(&s, ready, 32) == 1 && s.phase == IMU_PHASE_STREAMING);
    imu_control_session_resume(&s, 0x12345678, 0);
    CHECK(imu_control_session_receive(&s, wait, 32) == 0 && s.phase == IMU_PHASE_WAIT_LED);
    CHECK(imu_control_session_next_tx(&s, 999999, command) == 0 && s.phase == IMU_PHASE_WAIT_LED);
    CHECK(imu_control_session_user_confirm(&s, 999999) == 1);
}

static void test_ethernet(void)
{
    static const char *frame_hex =
      "000a3501fec0aabbccddeeff08004500003c000040004011b95bc0a80003c0a80002"
      "138f138f00280000494d43310101000078563412000000000000000000000000000000007cbf3fc5";
    uint8_t expected[74], frame[74], payload[32];
    unhex(frame_hex, expected, 74); unhex(start_hex, payload, 32);
    const uint8_t source[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
    const uint8_t destination[6] = {0,0x0A,0x35,1,0xFE,0xC0};
    const uint8_t source_ip[4] = {192,168,0,3}, destination_ip[4] = {192,168,0,2};
    CHECK(imu_control_build_ethernet(frame, source, destination, source_ip, destination_ip, payload) == 74);
    CHECK(!memcmp(frame, expected, 74));
    unsigned sum = 0; for (unsigned i = 14; i < 34; i += 2) sum += ((unsigned)frame[i] << 8) | frame[i + 1];
    while (sum >> 16) sum = (sum & 65535u) + (sum >> 16);
    CHECK(sum == 65535u);
    CHECK(frame[20] == 0x40 && frame[21] == 0 && frame[22] == 64);
    CHECK(frame[40] == 0 && frame[41] == 0 && !memcmp(frame + 42, payload, 32));
    CHECK(imu_control_build_ethernet(NULL, source, destination, source_ip, destination_ip, payload) == 0);
    FILE *binary = fopen("control_start_ethernet.bin", "wb"); CHECK(binary != NULL);
    if (binary) { CHECK(fwrite(frame, 1, 74, binary) == 74); CHECK(fclose(binary) == 0); }
}

static int ppm(const char *name, const uint32_t *p)
{
    FILE *f = fopen(name, "wb"); if (!f) return -1;
    fprintf(f, "P6\n%d %d\n255\n", IMU_CANVAS_WIDTH, IMU_CANVAS_HEIGHT);
    for (size_t i = 0; i < (size_t)IMU_CANVAS_WIDTH * IMU_CANVAS_HEIGHT; ++i) {
        uint8_t b[3] = {(uint8_t)(p[i] >> 16), (uint8_t)(p[i] >> 8), (uint8_t)p[i]};
        if (fwrite(b, 1, 3, f) != 3) { fclose(f); return -1; }
    }
    return fclose(f);
}
static void test_canvas(void)
{
    const size_t count = (size_t)IMU_CANVAS_WIDTH * IMU_CANVAS_HEIGHT;
    uint32_t *storage = malloc((count + 2) * sizeof(uint32_t)); CHECK(storage != NULL);
    if (!storage) return;
    storage[0] = storage[count + 1] = 0xDEADBEEF; uint32_t *p = storage + 1;
    struct imu_model model; imu_model_init(&model, 0);
    model.have_angles = model.seen_packet = model.simulated = 1;
    model.state = IMU_LIVE; model.last_good_ms = model.last_packet_ms = 100;
    model.degrees[0] = 30; model.degrees[1] = -45; model.degrees[2] = 90;
    struct imu_control_session session; imu_control_session_init(&session, 1, 0);
    session.phase = IMU_PHASE_WAIT_LED;
    imu_canvas_draw_control(p, &model, &session, 140);
    uint32_t *hidden = malloc(count * sizeof(*hidden)); CHECK(hidden != NULL);
    if (hidden) {
        memcpy(hidden, p, count * sizeof(*hidden));
        model.degrees[0] = -120; model.degrees[1] = 13; model.degrees[2] = 171;
        imu_canvas_draw_control(p, &model, &session, 140);
        CHECK(!memcmp(hidden, p, count * sizeof(*hidden)));
        free(hidden); /* 缓存角度不应影响READY前的显示像素。 */
    }
    CHECK(storage[0] == 0xDEADBEEF && storage[count + 1] == 0xDEADBEEF);
    CHECK(ppm("panel_wait_led.ppm", p) == 0);
    CHECK(imu_canvas_confirm_hit(24, 336)); CHECK(imu_canvas_confirm_hit(743.99, 383.99));
    CHECK(!imu_canvas_confirm_hit(23.99, 336)); CHECK(!imu_canvas_confirm_hit(744, 350));
    CHECK(!imu_canvas_confirm_hit(30, 384));
    session.phase = IMU_PHASE_STREAMING;
    imu_canvas_draw_control(p, &model, &session, 140);
    size_t green = 0; for (size_t i = 0; i < count; ++i) green += p[i] == 0x0024764B;
    CHECK(green > 100); CHECK(ppm("panel_live.ppm", p) == 0);
    imu_canvas_draw_control(p, &model, &session, 1100); CHECK(ppm("panel_stale.ppm", p) == 0);
    session.phase = IMU_PHASE_CONFIRMING;
    imu_canvas_draw_control(p, &model, &session, 1100); CHECK(ppm("panel_confirming.ppm", p) == 0);
    session.phase = IMU_PHASE_FAILED; session.timed_out = 1;
    imu_canvas_draw_control(p, &model, &session, 1100); CHECK(ppm("panel_timeout.ppm", p) == 0);
    CHECK(storage[0] == 0xDEADBEEF && storage[count + 1] == 0xDEADBEEF);
    free(storage);
}
int main(void)
{
    test_protocol(); test_session(); test_ethernet(); test_canvas();
    printf("IMU IIC CONTROL HOST TEST: checks=%u failures=%u\n", checks, failures);
    return failures ? 1 : 0;
}
