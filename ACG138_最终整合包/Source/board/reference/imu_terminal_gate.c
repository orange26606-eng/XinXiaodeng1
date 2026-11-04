#include "imu_terminal_gate.h"

#include <string.h>

void imu_terminal_gate_init(struct imu_terminal_gate *gate)
{
    if (gate != NULL) {
        memset(gate, 0, sizeof(*gate));
    }
}

void imu_terminal_gate_prompt(struct imu_terminal_gate *gate)
{
    int partial;

    if (gate == NULL || gate->disconnected || gate->confirmed) {
        return;
    }
    partial = gate->used != 0U || gate->discard_line;
    gate->used = 0U;
    gate->line[0] = '\0';
    gate->discard_line = partial != 0;
    gate->prompted = 1;
}

static int line_is_confirmation(const struct imu_terminal_gate *gate)
{
    return (gate->used == 1U ||
            (gate->used == 2U && gate->line[1] == '\r')) &&
           (gate->line[0] == 'y' || gate->line[0] == 'Y');
}

int imu_terminal_gate_feed(struct imu_terminal_gate *gate,
                           const char *bytes, size_t count)
{
    size_t index;

    if (gate == NULL || bytes == NULL || gate->disconnected ||
        gate->confirmed) {
        return 0;
    }

    for (index = 0U; index < count; ++index) {
        char byte = bytes[index];

        if (byte == '\n') {
            int accepted = gate->prompted && !gate->discard_line &&
                           line_is_confirmation(gate);
            gate->used = 0U;
            gate->line[0] = '\0';
            gate->discard_line = 0;
            if (accepted) {
                gate->confirmed = 1;
                return 1;
            }
        } else if (!gate->discard_line) {
            if (gate->used == sizeof(gate->line) - 1U) {
                gate->used = 0U;
                gate->line[0] = '\0';
                gate->discard_line = 1;
            } else {
                gate->line[gate->used++] = byte;
                gate->line[gate->used] = '\0';
            }
        }
    }
    return 0;
}

int imu_terminal_gate_disconnect(struct imu_terminal_gate *gate)
{
    if (gate == NULL) {
        return 0;
    }
    gate->disconnected = 1;
    gate->used = 0U;
    gate->line[0] = '\0';
    gate->discard_line = 0;
    return 1;
}
