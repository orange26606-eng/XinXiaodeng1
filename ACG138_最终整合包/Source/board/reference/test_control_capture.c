/* 离线互操作测试，将RTL实际GMII CSV交给运行用C解码器，不打开网络套接字或硬件。
 */
#include "imu_iic_control.h"
#include "imu_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct captured { unsigned instance, epoch, frame, size; unsigned char bytes[2048]; };
static struct captured frames[64];
static unsigned count, checks, failures;
#define CHECK(test) do { ++checks; if (!(test)) { ++failures; \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #test); } } while (0)
static unsigned read_hex32(const char *text)
{
    char part[9]; memcpy(part, text, 8); part[8] = 0;
    return (unsigned)strtoul(part, NULL, 16);
}

int main(int argc, char **argv)
{
    if (argc != 3) { fprintf(stderr, "Usage: %s capture.csv expected.csv\n", argv[0]); return 2; }
    FILE *input = fopen(argv[1], "rb");
    if (!input) { perror("capture CSV"); return 2; }
    char line[512];
    if (!fgets(line, sizeof(line), input)) { fclose(input); return 2; }
    while (fgets(line, sizeof(line), input)) {
        unsigned instance, epoch, frame, index, value;
        if (sscanf(line, "%u,%u,%u,%u,%x", &instance, &epoch, &frame, &index, &value) != 5)
            { fprintf(stderr, "Invalid capture row\n"); fclose(input); return 2; }
        unsigned found = count;
        for (unsigned i = 0; i < count; ++i)
            if (frames[i].instance == instance && frames[i].epoch == epoch && frames[i].frame == frame)
                { found = i; break; }
        if (found == count) {
            if (count == 64) { fclose(input); return 2; }
            frames[count].instance = instance; frames[count].epoch = epoch;
            frames[count++].frame = frame;
        }
        if (index != frames[found].size || index >= 2048 || value > 255)
            { fprintf(stderr, "Non-contiguous or oversized capture frame\n"); fclose(input); return 2; }
        frames[found].bytes[frames[found].size++] = (unsigned char)value;
    }
    fclose(input);
    input = fopen(argv[2], "rb");
    if (!input) { perror("expected CSV"); return 2; }
    if (!fgets(line, sizeof(line), input)) { fclose(input); return 2; }
    unsigned seen = 0, acknowledgements = 0, poses = 0;
    while (fgets(line, sizeof(line), input)) {
        unsigned instance, epoch, frame, is_ack, state, error, sequence;
        char angles[25], extra;
        if (sscanf(line, "%u,%u,%u,%u,%u,%u,%x,%24[0-9a-fA-F]%c", &instance, &epoch,
                   &frame, &is_ack, &state, &error, &sequence, angles, &extra) != 9 || strlen(angles) != 24)
            { fprintf(stderr, "Invalid expected row\n"); fclose(input); return 2; }
        CHECK(seen < count);
        if (seen >= count) break;
        const struct captured *data = &frames[seen++];
        CHECK(data->instance == instance && data->epoch == epoch && data->frame == frame);
        CHECK(data->size >= 82);
        if (data->size < 82) continue;
        /* GMII前8字节为前导码与SFD，之后为14字节以太网头、20字节IPv4头及8字节UDP头，独立核验FCS、IP头和应用CRC。
 */
        const unsigned char *payload = data->bytes + 50;
        if (is_ack) {
            struct imu_control_ack ack = {0};
            CHECK(imu_control_decode_ack(payload, 32, &ack) == IMU_CTRL_OK);
            CHECK(ack.token == sequence); CHECK(ack.state == state); CHECK(ack.error == error);
            CHECK(ack.algorithm == (state == 1 || state == 2 ? 9 : 0));
            CHECK(ack.polling_hz == (state == 1 || state == 2 ? 50 : 0));
            printf("RTL ACK instance=%u epoch=%u frame=%u token=%08X state=%u error=%u algorithm=%u polling_hz=%u\n",
                   instance, epoch, frame, ack.token, ack.state, ack.error, ack.algorithm, ack.polling_hz);
            ++acknowledgements;
        } else {
            struct imu_frame pose = {0};
            CHECK(imu_decode(payload, 32, &pose) == IMU_DECODE_OK);
            CHECK(pose.sequence == sequence); CHECK(pose.status == error);
            for (unsigned axis = 0; axis < 3; ++axis) {
                unsigned actual; memcpy(&actual, &pose.radians[axis], sizeof(actual));
                CHECK(actual == read_hex32(angles + (2 - axis) * 8));
            }
            printf("RTL POSE instance=%u epoch=%u frame=%u sequence=%08X radians=%.9g,%.9g,%.9g\n",
                   instance, epoch, frame, pose.sequence, pose.radians[0], pose.radians[1], pose.radians[2]);
            ++poses;
        }
    }
    fclose(input);
    CHECK(seen == count); CHECK(count == 9); CHECK(acknowledgements == 7); CHECK(poses == 2);
    printf("RTL capture interoperability: %u frames (%u ACK, %u POSE), %u checks, %u failures\n",
           seen, acknowledgements, poses, checks, failures);
    return failures ? 1 : 0;
}
