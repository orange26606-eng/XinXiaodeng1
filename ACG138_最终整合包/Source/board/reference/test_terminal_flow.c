/* 离线集成实际输入确认与控制协议，不访问Linux或Wayland设备。 */
#include "imu_terminal_flow.h"
#include "imu_protocol.h"
#include <stdio.h>
#include <string.h>
static unsigned checks, failures;
#define CHECK(test) do { ++checks; if (!(test)) { ++failures; \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #test); } } while (0)
static const uint32_t token = 0x12345678;
static void hex(const char *text, uint8_t output[32])
{ for (unsigned i=0; i<32; ++i) { unsigned byte=0; (void)sscanf(text+2*i,"%2x",&byte); output[i]=(uint8_t)byte; } }
static void fresh(struct imu_terminal_flow *f) { imu_terminal_flow_init(f,token,0,0); }
static uint8_t wait_ack[32], ready_ack[32], failed_ack[32], busy_ack[32];
static void waiting(struct imu_terminal_flow *f)
{
    fresh(f); CHECK(!imu_terminal_flow_receive(f,wait_ack,32));
    CHECK(imu_terminal_flow_prompt_needed(f)); CHECK(!imu_terminal_flow_may_open_window(f));
}
int main(void)
{
    hex("494d41310101000078563412093200000000000000000000000000008dea2f69",wait_ack);
    hex("494d4131010200007856341209320000000000000000000000000000b0d3ca1f",ready_ack);
    hex("494d41310103060078563412000000000000000000000000000000007a7d124f",failed_ack);
    hex("494d413101040000785634120000000000000000000000000000000054bf5a4b",busy_ack);
    struct imu_terminal_flow f; uint8_t command[32], first[32];
    fresh(&f); CHECK(!imu_terminal_flow_may_open_window(&f));
    CHECK(!imu_terminal_flow_input(&f,"y\n",2,0));
    CHECK(!imu_terminal_flow_receive(&f,ready_ack,32));
    CHECK(!imu_terminal_flow_may_open_window(&f));
    for (unsigned i=0;i<10;++i) {
        CHECK(imu_terminal_flow_next_tx(&f,(uint64_t)i*1000,command));
        CHECK(command[5]==IMU_CONTROL_START);
        if (!i) memcpy(first,command,32); else CHECK(!memcmp(first,command,32));
        CHECK(!imu_terminal_flow_may_open_window(&f));
    }
    CHECK(!imu_terminal_flow_next_tx(&f,13999,command)); CHECK(!imu_terminal_flow_failed(&f));
    CHECK(!imu_terminal_flow_next_tx(&f,14000,command)); CHECK(imu_terminal_flow_failed(&f));
    CHECK(!imu_terminal_flow_may_open_window(&f));
    waiting(&f);
    CHECK(!imu_terminal_flow_input(&f,"y",1,0)); /* 提示显示前读取输入。 */
    CHECK(!imu_terminal_flow_receive(&f,ready_ack,32));
    CHECK(!imu_terminal_flow_may_open_window(&f));
    CHECK(imu_terminal_flow_after_input_flush(&f)); /* 模拟tcflush及重置成功。 */
    CHECK(!imu_terminal_flow_prompt_needed(&f)); CHECK(!f.gate.used);
    CHECK(!imu_terminal_flow_input(&f,"\n",1,0)); /* 提前输入的半行应被刷新清除。 */
    const char invalid[] = "yes\n y\ny \nn\n";
    CHECK(!imu_terminal_flow_input(&f,invalid,sizeof(invalid)-1,0));
    CHECK(f.control.phase==IMU_PHASE_WAIT_LED); CHECK(!imu_terminal_flow_may_open_window(&f));
    CHECK(!imu_terminal_flow_input(&f,"y",1,0)); CHECK(!imu_terminal_flow_may_open_window(&f));
    CHECK(imu_terminal_flow_input(&f,"\n",1,1));
    CHECK(f.control.phase==IMU_PHASE_CONFIRMING); CHECK(!imu_terminal_flow_may_open_window(&f));
    CHECK(imu_terminal_flow_next_tx(&f,1,command)); CHECK(command[5]==IMU_CONTROL_CONFIRM_LED_STOPPED);
    hex("494d43310102000078563412000000000000000000000000000000004186dab3",first);
    CHECK(!memcmp(first,command,32));
    CHECK(!imu_terminal_flow_input(&f,"y\n",2,2));
    CHECK(!imu_terminal_flow_receive(&f,wait_ack,32)); CHECK(!imu_terminal_flow_may_open_window(&f));
    CHECK(imu_terminal_flow_receive(&f,ready_ack,32)); CHECK(imu_terminal_flow_may_open_window(&f));
    CHECK(!imu_terminal_flow_receive(&f,ready_ack,32)); CHECK(imu_terminal_flow_may_open_window(&f));
    CHECK(!imu_terminal_flow_next_tx(&f,500000,command));
    /* 任意阶段的TTY EOF、HUP或取消都会阻止新命令和窗口创建。 */
    for (unsigned stage=0;stage<4;++stage) {
        fresh(&f);
        if (stage>0) { (void)imu_terminal_flow_receive(&f,wait_ack,32);
            CHECK(imu_terminal_flow_after_input_flush(&f)); }
        if (stage>1) CHECK(imu_terminal_flow_input(&f,"Y\r\n",3,0));
        if (stage>2) CHECK(imu_terminal_flow_receive(&f,ready_ack,32));
        imu_terminal_flow_disconnect(&f);
        CHECK(imu_terminal_flow_failed(&f)); CHECK(!imu_terminal_flow_may_open_window(&f));
        CHECK(!imu_terminal_flow_next_tx(&f,0,command));
        CHECK(!imu_terminal_flow_input(&f,"y\n",2,0));
        CHECK(!imu_terminal_flow_receive(&f,ready_ack,32));
        CHECK(!imu_terminal_flow_may_open_window(&f));
    }
    waiting(&f); CHECK(!imu_terminal_flow_receive(&f,failed_ack,32));
    CHECK(imu_terminal_flow_failed(&f)); CHECK(!imu_terminal_flow_may_open_window(&f));
    waiting(&f); CHECK(!imu_terminal_flow_receive(&f,busy_ack,32));
    CHECK(imu_terminal_flow_failed(&f)); CHECK(!imu_terminal_flow_may_open_window(&f));
    uint8_t corrupt[32]; memcpy(corrupt,ready_ack,32); corrupt[31]^=1;
    waiting(&f); CHECK(imu_terminal_flow_after_input_flush(&f));
    CHECK(imu_terminal_flow_input(&f,"y\n",2,0));
    CHECK(!imu_terminal_flow_receive(&f,corrupt,32)); CHECK(!imu_terminal_flow_may_open_window(&f));
    memcpy(corrupt,ready_ack,32); corrupt[8]^=1;
    uint32_t crc=imu_crc32(corrupt,28);
    for(unsigned i=0;i<4;++i) corrupt[28+i]=(uint8_t)(crc>>(i*8));
    CHECK(!imu_terminal_flow_receive(&f,corrupt,32)); CHECK(!imu_terminal_flow_may_open_window(&f));
    /* 唯一例外是显式恢复已经确认的会话。 */
    imu_terminal_flow_init(&f,token,1,0); CHECK(imu_terminal_flow_receive(&f,ready_ack,32));
    CHECK(imu_terminal_flow_may_open_window(&f)); CHECK(!f.gate.confirmed);
    imu_terminal_flow_init(&f,token,1,0); CHECK(!imu_terminal_flow_receive(&f,wait_ack,32));
    CHECK(!imu_terminal_flow_may_open_window(&f)); CHECK(imu_terminal_flow_after_input_flush(&f));
    CHECK(imu_terminal_flow_input(&f,"y\n",2,0)); CHECK(imu_terminal_flow_receive(&f,ready_ack,32));
    CHECK(imu_terminal_flow_may_open_window(&f));
    printf("TERMINAL-FIRST FLOW TEST: checks=%u failures=%u\n",checks,failures);
    return failures?1:0;
}
