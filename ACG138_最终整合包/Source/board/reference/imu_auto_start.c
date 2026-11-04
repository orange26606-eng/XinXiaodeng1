#include "imu_auto_start.h"
#include <string.h>

void imu_auto_start_init(struct imu_auto_start *s,uint32_t token,int resume,uint64_t now)
{
    memset(s,0,sizeof(*s));
    if(resume)imu_control_session_resume(&s->control,token,now);
    else imu_control_session_init(&s->control,token,now);
    imu_model_init(&s->model,now);
}
int imu_auto_start_failed(const struct imu_auto_start *s)
{ return s->cancelled || s->data_timed_out || s->control.phase==IMU_PHASE_FAILED; }
void imu_auto_start_tick(struct imu_auto_start *s,uint64_t now)
{
    if(imu_auto_start_failed(s))return;
    if(s->control.phase==IMU_PHASE_WAIT_LED) {
        if(!s->waiting) { s->waiting=1;s->wait_ms=now; }
        if(now>=s->wait_ms && now-s->wait_ms>=IMU_AUTO_WAIT_MS)
            (void)imu_control_session_confirm(&s->control,now);
    }
    if(s->control.phase==IMU_PHASE_STREAMING) {
        if(!s->ready) { s->ready=1;s->ready_ms=now;imu_model_init(&s->model,now); }
        if(s->complete && (now<s->last_good_ms || now-s->last_good_ms>IMU_AUTO_MAX_GAP_MS)) {
            s->complete=0;s->consecutive_good=0;
        }
        if(!s->complete && now>=s->ready_ms && now-s->ready_ms>=IMU_AUTO_DATA_TIMEOUT_MS)
            s->data_timed_out=1;
    }
}
int imu_auto_start_next_tx(struct imu_auto_start *s,uint64_t now,uint8_t output[32])
{
    imu_auto_start_tick(s,now);
    return imu_auto_start_failed(s)?0:imu_control_session_next_tx(&s->control,now,output);
}
int imu_auto_start_angles(struct imu_auto_start *s,const uint8_t *packet,size_t size,uint64_t now)
{
    imu_auto_start_tick(s,now);
    if(imu_auto_start_failed(s) || !s->ready)return 0;
    if(!imu_model_push(&s->model,packet,size,now)) {
        s->consecutive_good=0;s->complete=0;return 0;
    }
    if(!s->consecutive_good || now<s->last_good_ms || now-s->last_good_ms>IMU_AUTO_MAX_GAP_MS) {
        s->consecutive_good=0;s->good_since_ms=now;s->complete=0;
    }
    ++s->consecutive_good;s->last_good_ms=now;
    if(s->consecutive_good>=IMU_AUTO_GOOD_FRAMES && now>=s->good_since_ms &&
       now-s->good_since_ms>=IMU_AUTO_GOOD_SPAN_MS)s->complete=1;
    return 1;
}
int imu_auto_start_may_open(const struct imu_auto_start *s,uint64_t now)
{
    return !imu_auto_start_failed(s) && s->control.phase==IMU_PHASE_STREAMING && s->complete &&
           now>=s->last_good_ms && now-s->last_good_ms<=IMU_AUTO_MAX_GAP_MS &&
           imu_model_state(&s->model,now)==IMU_LIVE;
}
void imu_auto_start_cancel(struct imu_auto_start *s) { s->cancelled=1;s->complete=0; }
