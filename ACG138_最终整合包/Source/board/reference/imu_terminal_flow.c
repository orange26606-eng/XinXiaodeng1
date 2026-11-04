#include "imu_terminal_flow.h"
#include <string.h>
void imu_terminal_flow_init(struct imu_terminal_flow *f, uint32_t token,
                             int resume, uint64_t now_ms)
{
    memset(f, 0, sizeof(*f)); imu_terminal_gate_init(&f->gate);
    if (resume) imu_control_session_resume(&f->control, token, now_ms);
    else imu_control_session_init(&f->control, token, now_ms);
}
int imu_terminal_flow_prompt_needed(const struct imu_terminal_flow *f)
{ return !f->stopped && f->control.phase == IMU_PHASE_WAIT_LED && !f->gate.prompted; }
int imu_terminal_flow_after_input_flush(struct imu_terminal_flow *f)
{
    if (!imu_terminal_flow_prompt_needed(f)) return 0;
    imu_terminal_gate_init(&f->gate); imu_terminal_gate_prompt(&f->gate);
    return 1;
}
int imu_terminal_flow_input(struct imu_terminal_flow *f, const char *bytes,
                            size_t length, uint64_t now_ms)
{
    if (f->stopped) return 0;
    int confirmed = imu_terminal_gate_feed(&f->gate, bytes, length);
    if (f->control.phase != IMU_PHASE_WAIT_LED || !f->gate.prompted || !confirmed) return 0;
    return imu_control_session_user_confirm(&f->control, now_ms);
}
int imu_terminal_flow_receive(struct imu_terminal_flow *f,
                              const uint8_t *bytes, size_t length)
{ return f->stopped ? 0 : imu_control_session_receive(&f->control, bytes, length); }
int imu_terminal_flow_next_tx(struct imu_terminal_flow *f, uint64_t now_ms,
                              uint8_t output[32])
{ return f->stopped ? 0 : imu_control_session_next_tx(&f->control, now_ms, output); }
void imu_terminal_flow_disconnect(struct imu_terminal_flow *f)
{ f->stopped = 1; (void)imu_terminal_gate_disconnect(&f->gate); }
int imu_terminal_flow_may_open_window(const struct imu_terminal_flow *f)
{
    return !f->stopped && f->control.phase == IMU_PHASE_STREAMING &&
           (f->gate.confirmed || f->control.resume_existing);
}
int imu_terminal_flow_failed(const struct imu_terminal_flow *f)
{ return f->stopped || f->control.phase == IMU_PHASE_FAILED; }
