/* 无硬件回归，检查事件循环时刻与传输时刻。 */
#define _GNU_SOURCE
#define clock_gettime fake_clock_gettime
#define recv fake_recv
#define sendto fake_sendto
#include "../../rk/src/chassis/chassis_linux.c"
#undef clock_gettime
#undef recv
#undef sendto
#include <assert.h>

static uint64_t fake_time_us;
static unsigned sent;
static uint8_t sent_op;

int fake_clock_gettime(clockid_t kind, struct timespec *out){
 assert(kind==CLOCK_MONOTONIC);
 out->tv_sec=(time_t)(fake_time_us/1000000);
 out->tv_nsec=(long)(fake_time_us%1000000)*1000;
 return 0;
}
ssize_t fake_recv(int fd,void *buf,size_t len,int flags){
 (void)fd;(void)buf;(void)len;(void)flags;errno=EAGAIN;return -1;
}
ssize_t fake_sendto(int fd,const void *buf,size_t len,int flags,const struct sockaddr *to,socklen_t tolen){
 (void)fd;(void)flags;(void)to;(void)tolen;
 assert(len==106);
 ch_wire_command command;
 assert(ch_command_decode(&command,(const uint8_t*)buf+42,64)==CH_OK);
 sent_op=command.op;++sent;
 return (ssize_t)len;
}
int main(void){
 ch_linux t={0};
 t.fd=77;t.config_id=42;t.have_status=1;t.verified=1;
 t.status_us=1000000;t.last_host=1000000;
 t.status=(ch_wire_status){0};
 t.status.config_id=42;t.status.session=7;t.status.mode=CH_NAVIGATION;
 t.status.lease=9;t.status.state=CH_RX_ARMED;t.status.max_rpm_centi=8000;
 ch_wire_status out={0};
 ch_wire_command stop={0};
 stop.op=CH_STOP;stop.mode=CH_NAVIGATION;stop.session=7;
 stop.sequence=1;stop.lease=9;stop.config_id=42;

 fake_time_us=1000100;
 assert(ch_linux_poll(&t,1000000,&out)==CH_WAIT);
 assert(t.verified==1&&t.last_host==1000100);
 fake_time_us=1000169;
 assert(ch_linux_send(&t,&stop,1000000,1000000)==CH_OK);
 assert(sent==1&&sent_op==CH_STOP);
 /* 调用方时间早于上一次发送时刻，但真实时间并未倒退。 */
 fake_time_us=1000250;
 assert(ch_linux_poll(&t,1000000,&out)==CH_WAIT);
 assert(t.verified==1&&t.last_host==1000250);
 stop.sequence=2;
 fake_time_us=1000270;
 assert(ch_linux_send(&t,&stop,1000000,1000000)==CH_OK);
 assert(sent==2);
 /* 实际过期FPGA状态仍需在本地拒绝STOP验证请求。 */
 stop.sequence=3;fake_time_us=1260000;
 assert(ch_linux_send(&t,&stop,1000000,1000000)==CH_READONLY);
 assert(sent==2);
 /* 真实倒退的传输时钟仍使验证失效。 */
 fake_time_us=1000200;
 assert(ch_linux_poll(&t,1000000,&out)==CH_TIME);
 assert(!t.verified);
 fprintf(stdout,"PASS transport clock ordering, STOP send, stale gate and rollback gate\n");
 return 0;
}