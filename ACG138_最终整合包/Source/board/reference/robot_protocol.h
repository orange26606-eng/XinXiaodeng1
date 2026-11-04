#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#define ROBOT_PACKET_SIZE 32
#define ROBOT_UDP_PORT 5008
#define ROBOT_STALE_MS 1000
enum robot_distance_state { ROBOT_WAITING, ROBOT_DISTANCE, ROBOT_MAXIMUM, ROBOT_INVALID, ROBOT_STALE, ROBOT_HOLD, ROBOT_BLIND, ROBOT_LOST };
enum robot_range_state { ROBOT_RANGE_UNKNOWN=-1, ROBOT_RANGE_BLOCKED=0, ROBOT_RANGE_ALLOWED=1 };
enum robot_drive_mode { ROBOT_MODE_UNKNOWN, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTO };
struct robot_model {
    uint64_t last_ms, received, rejected;
    uint32_t sequence;
    uint16_t mm[3];
    uint8_t valid, error[3], auto_mode, actual, feedback, state, stop_reason;
    int seen, bad_packet;
    uint8_t remote_supported, remote_flags;
    uint16_t remote_sequence;
    uint8_t nav_supported, near_mask;
    uint8_t feedback_supported, feedback_phase, feedback_retries;
};
uint32_t robot_crc32(const uint8_t *data, size_t size);
int robot_model_push(struct robot_model *model, const uint8_t *data, size_t size, uint64_t now_ms);
enum robot_distance_state robot_distance_state(const struct robot_model *model, unsigned channel, uint64_t now_ms);
enum robot_range_state robot_auto_range_state(const struct robot_model *model, unsigned channel, uint64_t now_ms);
enum robot_drive_mode robot_drive_mode(const struct robot_model *model, uint64_t now_ms);
const char *robot_reason_text(unsigned reason);
const char *robot_feedback_phase_text(unsigned phase);
const char *robot_action_text(unsigned action);
#endif
