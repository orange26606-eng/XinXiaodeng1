#include "imu_auto_start.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint32_t token=0x12345678;
static void word(uint8_t *p,uint32_t value)
{ for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(value>>(8*i)); }
static void finish(uint8_t *p) { word(p+28,imu_crc32(p,28)); }
static void ack(struct imu_auto_start *s,unsigned state,unsigned error,uint64_t now)
{
    uint8_t p[32]={0};memcpy(p,"IMA1",4);p[4]=1;p[5]=(uint8_t)state;p[6]=(uint8_t)error;
    word(p+8,token);if(state==1 || state==2) { p[12]=9;p[13]=50; }finish(p);
    (void)imu_control_session_receive(&s->control,p,32);imu_auto_start_tick(s,now);
}
static void angle(uint8_t p[32],uint32_t sequence,unsigned error)
{
    memset(p,0,32);memcpy(p,"IMU1",4);p[4]=1;p[5]=(uint8_t)error;word(p+8,sequence);finish(p);
}
static void waiting(struct imu_auto_start *s)
{
    uint8_t tx[32];imu_auto_start_init(s,token,0,0);
    assert(imu_auto_start_next_tx(s,0,tx) && tx[5]==IMU_CONTROL_START);
    ack(s,IMU_ACK_WAIT_LED,0,1000);
    assert(s->waiting && s->wait_ms==1000 && !imu_auto_start_may_open(s,1000));
}
static void ready(struct imu_auto_start *s)
{
    uint8_t tx[32];waiting(s);
    assert(imu_auto_start_next_tx(s,11000,tx) && tx[5]==IMU_CONTROL_CONFIRM_LED_STOPPED);
    ack(s,IMU_ACK_READY,0,11010);assert(s->ready && !imu_auto_start_may_open(s,11010));
}
static void samples(struct imu_auto_start *s,uint32_t first,unsigned count,uint64_t now,unsigned step)
{
    for(unsigned i=0;i<count;++i) {
        uint8_t p[32];angle(p,first+i,0);assert(imu_auto_start_angles(s,p,32,now+(uint64_t)i*step));
    }
}
int main(void)
{
    struct imu_auto_start s;uint8_t tx[32],p[32];
    waiting(&s);ack(&s,IMU_ACK_READY,0,1100); /* 提前到达的READY不能跳过静置等待。 */
    angle(p,1,0);assert(!imu_auto_start_angles(&s,p,32,10000));
    assert(!imu_auto_start_next_tx(&s,10999,tx) && s.control.phase==IMU_PHASE_WAIT_LED);
    ack(&s,IMU_ACK_WAIT_LED,0,10500);assert(s.wait_ms==1000); /* 重复回执不延长等待时间。 */
    assert(imu_auto_start_next_tx(&s,11000,tx) && tx[5]==IMU_CONTROL_CONFIRM_LED_STOPPED);
    assert(!imu_auto_start_next_tx(&s,11001,tx));
    assert(imu_auto_start_next_tx(&s,12000,tx) && tx[5]==IMU_CONTROL_CONFIRM_LED_STOPPED);
    ack(&s,IMU_ACK_READY,0,12010);samples(&s,1,4,12020,20);
    assert(!imu_auto_start_may_open(&s,12080));samples(&s,5,1,12100,20);
    assert(imu_auto_start_may_open(&s,12100));assert(!imu_auto_start_may_open(&s,12301));

    ready(&s);samples(&s,1,5,11020,0); /* 同一时刻排队的5个报文不足以通过连续新样本检查。 */
    imu_auto_start_tick(&s,11100);assert(!imu_auto_start_may_open(&s,11100));
    samples(&s,6,1,11100,20);assert(imu_auto_start_may_open(&s,11100));
    imu_auto_start_tick(&s,11301);assert(!s.complete);imu_auto_start_tick(&s,16010);
    assert(imu_auto_start_failed(&s)); /* 有效突发之后没有新数据时，不应无限等待。 */

    ready(&s);samples(&s,1,4,11020,20);angle(p,5,0);p[31]^=1;
    assert(!imu_auto_start_angles(&s,p,32,11100) && !s.consecutive_good);
    samples(&s,6,4,11120,20);assert(!imu_auto_start_may_open(&s,11180));
    samples(&s,10,1,11200,20);assert(imu_auto_start_may_open(&s,11200));

    ready(&s);samples(&s,1,4,11020,20);angle(p,4,0);
    assert(!imu_auto_start_angles(&s,p,32,11100) && !s.consecutive_good); /* 重复回放报文。 */
    angle(p,5,2);assert(!imu_auto_start_angles(&s,p,32,11120)); /* 传感器错误。 */
    angle(p,6,0);word(p+12,0x7fc00000);finish(p);
    assert(!imu_auto_start_angles(&s,p,32,11140)); /* CRC正确但数值为NaN。 */
    assert(!imu_auto_start_may_open(&s,11140));
    samples(&s,7,4,11160,20);samples(&s,11,1,11421,20); /* 201ms间隔使连续计数重新开始。 */
    assert(s.consecutive_good==1 && !imu_auto_start_may_open(&s,11421));

    ready(&s);imu_auto_start_tick(&s,16009);assert(!imu_auto_start_failed(&s));
    angle(p,1,0);assert(!imu_auto_start_angles(&s,p,32,16010));
    assert(s.data_timed_out && !imu_auto_start_may_open(&s,16010));
    waiting(&s);imu_auto_start_cancel(&s);assert(!imu_auto_start_next_tx(&s,11000,tx));
    assert(imu_auto_start_failed(&s));
    waiting(&s);ack(&s,IMU_ACK_FAILED,1,2000);assert(imu_auto_start_failed(&s));
    waiting(&s);ack(&s,IMU_ACK_BUSY,0,2000);assert(imu_auto_start_failed(&s));
    imu_auto_start_init(&s,token,0,0);
    for(unsigned i=0;i<10;++i)assert(imu_auto_start_next_tx(&s,(uint64_t)i*1000,tx));
    assert(!imu_auto_start_next_tx(&s,14000,tx) && imu_auto_start_failed(&s));
    waiting(&s);
    for(unsigned i=0;i<10;++i)assert(imu_auto_start_next_tx(&s,11000+(uint64_t)i*1000,tx));
    assert(!imu_auto_start_next_tx(&s,25000,tx) && imu_auto_start_failed(&s));
    imu_auto_start_init(&s,token,1,0);ack(&s,IMU_ACK_READY,0,10);
    assert(!imu_auto_start_may_open(&s,10));samples(&s,1,5,20,20);
    assert(imu_auto_start_may_open(&s,100));
    puts("PASS automatic IMU startup: exact 10s dwell, ACK ordering, fresh angles, CRC/status/NaN/replay, gaps, timeouts, cancellation and resume");
    return 0;
}
