
// 增加认证代理电机命令通路和独立500ms看门狗。
`timescale 1ns/1ps

// 模块：imu_rgmii_command_rx
// 功能：Gowin IDDR RGMII接收前端及严格IMC1解析。

// 板端接收使用RK3588直连1.8V固定链路，不另加PLL相移，
// 接收域外部复位异步进入，
// 在共用FPGA PLL锁定之后同步释放。
module imu_rgmii_command_rx (
    input wire clk_rx,
    input wire rst_n,
    input wire [3:0] rgmii_rxd,
    input wire rgmii_rx_ctl,
    output wire tap_en,tap_er,output wire [7:0] tap_data,
    output wire remote_valid, input wire remote_ready, output wire [79:0] remote_data,
    output wire cmd_valid,
    input wire cmd_ready,
    output wire [1:0] cmd_op,
    output wire [31:0] cmd_token
);
    wire [3:0] nibble_rising;
    wire [3:0] nibble_falling;
    wire control_rising;
    wire control_falling;
    genvar bit_index;
    generate
        for (bit_index = 0; bit_index < 4; bit_index = bit_index + 1) begin : g_rx
            IDDR u_iddr (
                .Q0(nibble_rising[bit_index]),
                .Q1(nibble_falling[bit_index]),
                .D(rgmii_rxd[bit_index]), .CLK(clk_rx)
            );
        end
    endgenerate
    IDDR u_iddr_control (
        .Q0(control_rising), .Q1(control_falling),
        .D(rgmii_rx_ctl), .CLK(clk_rx)
    );
    // 增加共用GMII寄存级以降低IDDR扇出。
    reg [7:0] gmii_data;
    reg gmii_enable,gmii_error;
    always @(posedge clk_rx or negedge rst_n)
        if(!rst_n)begin gmii_data<=0;gmii_enable<=0;gmii_error<=0;end
        else begin
            gmii_data<={nibble_falling,nibble_rising};
            gmii_enable<=control_rising;
            gmii_error<=control_rising ^ control_falling;
        end
    assign tap_en=gmii_enable;assign tap_er=gmii_error;assign tap_data=gmii_data;
    imu_command_rx_gmii u_parser (
        .clk_rx(clk_rx), .rst_n(rst_n),
        .gmii_rxen(gmii_enable),
        .gmii_rxer(gmii_error),
        .gmii_rxd(gmii_data),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_op(cmd_op), .cmd_token(cmd_token)
    );
    phone_command_rx_gmii u_phone_parser(
        .clk_rx(clk_rx),.rst_n(rst_n),.gmii_rxen(gmii_enable),
        .gmii_rxer(gmii_error),.gmii_rxd(gmii_data),
        .cmd_valid(remote_valid),.cmd_ready(remote_ready),.cmd_op(remote_data[79:72]),
        .cmd_motion(remote_data[71:64]),.cmd_token(remote_data[63:32]),.cmd_sequence(remote_data[31:0]));
endmodule
