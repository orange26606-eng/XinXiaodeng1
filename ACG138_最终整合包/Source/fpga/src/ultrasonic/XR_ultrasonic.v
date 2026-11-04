`include "r6_motion_policy.vh"
`timescale 1ns / 1ps
// ============================================================================
// 模块：XR_ultrasonic —— 三路 A02YYUW 自动输出测距接入层
// ============================================================================
// 一、数据路径
//   前/左/右 TX -> 各自 XR_uart_rx_core -> 各自 XR_a02_frame_parser -> 避障层。
//   三路独立同步、收字节、解帧和计时，允许同时发送；不轮流切换一个接收器。
//   当前顶层不再实例化旧 XR_ultrasonic_measure，也不产生 Trig 或计量 Echo。
//
// 二、时钟和接线
//   clk_sys 默认50MHz，rst_n低有效；三个串口共用16倍波特率采样使能 tick。
//   tick 是一个系统时钟周期的使能，不是新时钟；所有 always 仍由 clk_sys 驱动。
//   默认326分频：50MHz / (326 * 16) = 9585.9bps，接收名义9600bps、8N1。
//   每个 UART 核内部有独立两级同步器；异步 RX 不直接进入报文解析器。
//   传感器 RX 引脚接地选择实时值；us_*_rxd 接对应传感器 TX，注意3.3V电平。
//   自动输出探头的发声时刻不受本模块控制，串口并行接收不等于消除声学串扰。
//
// 三、通道映射（所有总线统一此约定）
//   通道0=正前，通道1=左前，通道2=右前。
//   Distance/distance_left/distance_right：24位毫米值，Distance是保留的旧端口名。
//   valid_mask[0/1/2]：对应距离现在是否有效，含校验、量程和新鲜度检查。
//   sample_update[0/1/2]：对应通道新有效帧的单拍通知；缓存保持不重复发脉冲。
//   status_codes[2:0]前、[5:3]左、[8:6]右；3位状态码定义见下方说明。
//   age_front/left/right：距最后有效帧的系统时钟拍数，不是毫秒。
//
// 四、错误与停车关系
//   状态码：0正常/1校验错/2 UART错/3半帧超时/4量程越界/5未就绪/6过期。
//   复位时三路均无效。错误只影响对应通道，其他通道不能为它刷新年龄。
//   无效距离输出 EEEEEE 占位，不是空旷距离；上层必须同时检查 valid_mask。
//   本模块只提供数据，不直接控制电机；XR_function 的统一门控负责无效停车。
// ============================================================================

module XR_ultrasonic #(
    parameter integer P_CLOCK_HZ=50000000,
    parameter integer P_TICK_CLKS              = 326,  // 50MHz除以326×16约为9585.9比特每秒。
    parameter integer P_GAP_CLKS               = 500000,  // 帧内字节间超时10ms
    parameter integer P_STALE_CLKS             = 15000000,  // 有效数据过期300ms
    // 量程参数只负责软件接受范围，不是车辆停车/转向阈值。
    parameter integer P_IGNORE_ZERO=0,
    parameter integer P_LOST_CLKS = 50000000,
    parameter integer P_ZERO_FRESH_CLKS=10000000,P_ZERO_CLKS=15000000,
    parameter integer P_NEAR_CONFIRM=0,P_NEAR_CLKS=15000000,
    parameter integer P_MIN_MM                 = 30,
    parameter integer P_MAX_MM                 = 4500
) (
    input[2:0] near_allow,
    output [2:0] near_pending_mask,near_timeout_mask,near_recover_mask,
    output [5:0] near_counts,output [95:0] nonzero_sequence,output [95:0] raw_ages_us,output[2:0]raw_seen_mask,
    input                 clk_sys,
    input                 rst_n,
    input                 us_front_rxd,
    input                 us_left_rxd,
    input                 us_right_rxd,
    output      [23:0]    Distance,
    output      [23:0]    distance_left,
    output      [23:0]    distance_right,
    output      [2:0]     valid_mask,
    output [2:0] packet_update,
    output      [2:0]     sample_update,
    output      [8:0]     status_codes,
    output      [31:0]    age_front,
    output      [31:0]    age_left,
    output      [2:0]     near_mask,
    output      [5:0]     receive_errors,
    output[2:0]last_raw_zero_mask,output [2:0] zero_mask,zero_expired_mask,
    output [47:0] control_raw,
    output      [287:0]   diagnostic,
    output      [31:0]    age_right
);

// 分频计数0..P_TICK_CLKS-1，终值产生一拍tick，然后清零重计。
localparam integer CLOCKS_PER_US=(P_CLOCK_HZ>=1000000)?P_CLOCK_HZ/1000000:1;
reg [31:0] microsecond_divider;
wire microsecond_tick=microsecond_divider==CLOCKS_PER_US-1;
always @(posedge clk_sys or negedge rst_n)
 if(!rst_n)microsecond_divider<=0;
 else if(microsecond_tick)microsecond_divider<=0;
 else microsecond_divider<=microsecond_divider+1'b1;
reg [31:0] tick_count;
wire tick = (tick_count == P_TICK_CLKS - 1);
always @(posedge clk_sys or negedge rst_n)
    if (!rst_n)
        tick_count <= 0;
    else if (tick)
        tick_count <= 0;
    else
        tick_count <= tick_count + 1'b1;

// 拼接顺序使rx[0]为前、rx[1]为左、rx[2]为右，与上层状态位一致。
wire [2:0] rx = {us_right_rxd, us_left_rxd, us_front_rxd};
wire [23:0] distances[0:2];
wire [31:0] ages[0:2];
// generate在综合时展开为三套硬件，不是运行中按顺序执行的软件循环。
genvar ch;
generate
    for (ch = 0; ch < 3; ch = ch + 1) begin : channels
        wire [7:0] data;
        wire byte_valid, byte_error;
        // 每路拥有独立同步器和收字节状态机；不对三个串口做输入多路选择。
        // valid/error均为单拍；busy只表示收字节过程，本层无需使用该诊断端口。
        XR_uart_rx_core u_rx (
            .clk(clk_sys),
            .rst_n(rst_n),
            .tick(tick),
            .rx(rx[ch]),
            .data(data),
            .valid(byte_valid),
            .error(byte_error),
            .busy()
        );
        // 三个解析器共享参数但不共享缓存/年龄，互相不能续期或覆盖距离。
        XR_a02_frame_parser #(
            .P_IGNORE_ZERO(P_IGNORE_ZERO),
            .P_STOP_MM(ch==0?`R6_FRONT_FORWARD_STOP_MM:80),
            .P_HARD_STOP_MM(ch==0?`R6_FRONT_FORWARD_STOP_MM:0),
            .P_NEAR_CONFIRM(P_NEAR_CONFIRM),.P_NEAR_CLKS(P_NEAR_CLKS),
            .P_EMERGENCY_MM(ch==0?`R6_FRONT_EMERGENCY_MM:30),.P_EMERGENCY_INCLUSIVE(ch==0),.P_TREND_MM(ch==0?350:250),
            .P_ZERO_MIN_MM(ch==0?250:100),.P_ZERO_FRESH_CLKS(P_ZERO_FRESH_CLKS),.P_ZERO_CLKS(P_ZERO_CLKS),
            .P_RELEASE_MM(ch==0?`R6_FRONT_RELEASE_MM:120),
            .P_LOST_CLKS(P_LOST_CLKS),
            .P_GAP_CLKS(P_GAP_CLKS),
            .P_STALE_CLKS(P_STALE_CLKS),
            .P_MIN_MM(P_MIN_MM),
            .P_MAX_MM(P_MAX_MM)
        ) u_parser (
            .clk(clk_sys),
            .rst_n(rst_n),
            .age_tick(microsecond_tick),.raw_age_us(raw_ages_us[ch*32+:32]),.byte_data(data),
            .byte_valid(byte_valid),
            .byte_error(byte_error),
            .packet_update(packet_update[ch]),.distance_mm(distances[ch]),
            .valid(valid_mask[ch]),
            .sample_update(sample_update[ch]),
            .status(status_codes[ch*3+:3]),
            .age(ages[ch]),
            .diagnostic(diagnostic[ch*96+:96]),.near_blocked(near_mask[ch]),.receive_error(receive_errors[ch*2+:2]),
            .near_allow(near_allow[ch]),.near_pending(near_pending_mask[ch]),.near_timeout(near_timeout_mask[ch]),
            .raw_seen(raw_seen_mask[ch]),.nonzero_sequence(nonzero_sequence[ch*32+:32]),.near_count(near_counts[ch*2+:2]),.near_recover(near_recover_mask[ch]),
            .last_raw_zero(last_raw_zero_mask[ch]),.zero_active(zero_mask[ch]),.zero_expired(zero_expired_mask[ch]),.control_raw(control_raw[ch*16+:16])
        );
    end
endgenerate

// 这里只做连线，不增加输出寄存级；距离和状态由对应解析器统一管理。
assign Distance = distances[0];
assign distance_left = distances[1];
assign distance_right = distances[2];
assign age_front = ages[0];
assign age_left = ages[1];
assign age_right = ages[2];

endmodule
