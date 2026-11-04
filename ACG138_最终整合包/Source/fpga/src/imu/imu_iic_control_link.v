
// 增加认证代理电机命令通路和独立500ms看门狗。
`timescale 1ns/1ps


// 日期：2026-10-12 18:20
// 模块：imu_iic_control_link
// 功能: 三时钟域IIC控制与RGMII命令链路，回执优先，未确认校准时不发送姿态。
// 版本：1.1.3
// 修订记录 ： v1.1.0----三时钟域IIC控制与RGMII命令链路，回执优先，未确认校准时不发送姿态。
//                   v1.1.2----仅发送器空闲时提出整帧请求，使仲裁提交与实际接收一致。
//                   v1.1.3----合入已上板的配置250ms超时与算法写后1秒等待，保留整帧仲裁的sender_ready门控。


module imu_iic_control_link #(
    parameter integer BOOT_WAIT_CLKS=250000000,
    parameter integer POLL_CLKS=1000000,
    parameter integer IIC_DIV=500,
    parameter integer CONFIG_TIMEOUT_CLKS=12500000,
    parameter integer ALGO_SETTLE_CLKS=50000000,
    parameter integer IIC_TIMEOUT_CLKS=500000
)(
    output remote_valid, input remote_ready, output [79:0] remote_data,
    input clk_iic, input clk_tx, input rgmii_rx_clk, input global_rst_n,
    input [3:0] rgmii_rxd, input rgmii_rx_ctl,
    inout imu_sda, inout imu_scl, output [3:0] led_out,
    input tx_grant, output send_pending, output calibration_wait,
    output gmii_txen, output [7:0] gmii_txd,
    output odom_imu_valid,output [130:0] odom_imu_data
);
// 两端须共同进入复位，翻转握手的任一端不能单独复位。
(* ASYNC_REG="TRUE" *) reg [1:0] iic_reset_sync=0;
(* ASYNC_REG="TRUE" *) reg [1:0] tx_reset_sync=0;
(* ASYNC_REG="TRUE" *) reg [1:0] rx_reset_sync=0;
always @(posedge clk_iic or negedge global_rst_n)
    if(!global_rst_n) iic_reset_sync<=0; else iic_reset_sync<={iic_reset_sync[0],1'b1};
always @(posedge clk_tx or negedge global_rst_n)
    if(!global_rst_n) tx_reset_sync<=0; else tx_reset_sync<={tx_reset_sync[0],1'b1};
always @(posedge rgmii_rx_clk or negedge global_rst_n)
    if(!global_rst_n) rx_reset_sync<=0; else rx_reset_sync<={rx_reset_sync[0],1'b1};
wire rst_iic=iic_reset_sync[1];
wire rst_tx=tx_reset_sync[1];
wire rst_rx=rx_reset_sync[1];
wire rx_valid,rx_ready,cmd_valid,cmd_ready;
wire [1:0] rx_op,cmd_op;
wire [31:0] rx_token,cmd_token;
imu_rgmii_command_rx u_rx(
    .remote_valid(remote_valid),.remote_ready(remote_ready),.remote_data(remote_data),
    .clk_rx(rgmii_rx_clk),.rst_n(rst_rx),.rgmii_rxd(rgmii_rxd),
    .rgmii_rx_ctl(rgmii_rx_ctl),.cmd_valid(rx_valid),.cmd_ready(rx_ready),
    .cmd_op(rx_op),.cmd_token(rx_token));
imu_command_cdc u_command(
    .src_clk(rgmii_rx_clk),.src_rst_n(global_rst_n),.src_valid(rx_valid),.src_ready(rx_ready),
    .src_op(rx_op),.src_token(rx_token),.dst_clk(clk_iic),.dst_rst_n(global_rst_n),
    .dst_valid(cmd_valid),.dst_ready(cmd_ready),.dst_op(cmd_op),.dst_token(cmd_token));
wire ack_valid,ack_ready,sample_valid,sample_ready,stream_enabled;
wire [2:0] ack_state,ack_error,sample_status;
wire [31:0] ack_token;
wire [95:0] sample_angles;
assign odom_imu_valid=sample_valid;
assign odom_imu_data={ack_token,sample_status,sample_angles};
imu_iic_control_core #(.BOOT_WAIT_CLKS(BOOT_WAIT_CLKS),.POLL_CLKS(POLL_CLKS),
    .IIC_DIV(IIC_DIV),.IIC_TIMEOUT_CLKS(IIC_TIMEOUT_CLKS),
    .CONFIG_TIMEOUT_CLKS(CONFIG_TIMEOUT_CLKS),.ALGO_SETTLE_CLKS(ALGO_SETTLE_CLKS)) u_core(
    .clk_iic(clk_iic),.rst_n(rst_iic),.imu_sda(imu_sda),.imu_scl(imu_scl),.led_out(led_out),
    .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_op(cmd_op),.cmd_token(cmd_token),
    .ack_valid(ack_valid),.ack_ready(ack_ready),.ack_state(ack_state),.ack_error(ack_error),.ack_token(ack_token),
    .sample_valid(sample_valid),.sample_ready(sample_ready),.sample_status(sample_status),.sample_angles(sample_angles),
    .stream_enabled(stream_enabled),.calibration_wait(calibration_wait),.config_diag(),.config_diag_valid());
wire ack_tx_valid,ack_tx_ready,sample_tx_valid,sample_tx_ready;
wire [37:0] ack_bundle;
wire [98:0] sample_bundle;
imu_bundle_cdc #(.WIDTH(38)) u_ack_mailbox(
    .src_clk(clk_iic),.src_rst_n(global_rst_n),.src_valid(ack_valid),.src_ready(ack_ready),
    .src_data({ack_error,ack_state,ack_token}),.dst_clk(clk_tx),.dst_rst_n(global_rst_n),
    .dst_valid(ack_tx_valid),.dst_ready(ack_tx_ready),.dst_data(ack_bundle));
imu_bundle_cdc #(.WIDTH(99)) u_sample_mailbox(
    .src_clk(clk_iic),.src_rst_n(global_rst_n),.src_valid(sample_valid),.src_ready(sample_ready),
    .src_data({sample_status,sample_angles}),.dst_clk(clk_tx),.dst_rst_n(global_rst_n),
    .dst_valid(sample_tx_valid),.dst_ready(sample_tx_ready),.dst_data(sample_bundle));
(* ASYNC_REG="TRUE" *) reg stream_meta,stream_sync;
always @(posedge clk_tx or negedge rst_tx)
    if(!rst_tx) begin stream_meta<=0; stream_sync<=0; end
    else begin stream_meta<=stream_enabled; stream_sync<=stream_meta; end
wire sender_ready;
wire pending_angle=stream_sync && sample_tx_valid;
// 只有发送器本周期可接收时才提交请求。
// 因此grant与pending同时有效即为仲裁器实际接受握手。
assign send_pending=sender_ready && (ack_tx_valid || pending_angle);
assign ack_tx_ready=tx_grant && sender_ready && ack_tx_valid;
// 新会话校准期间丢弃旧角度，回执和角度发送互斥。
assign sample_tx_ready=!stream_sync || (tx_grant && sender_ready && !ack_tx_valid);
imu_udp_message_tx_gmii u_sender(
    .clk125m(clk_tx),.rst_n(rst_tx),.sample_valid(tx_grant && send_pending),.sample_ready(sender_ready),
    .sample_status(ack_tx_valid ? ack_bundle[34:32] : sample_bundle[98:96]),
    .sample_angles(sample_bundle[95:0]),.message_ack(ack_tx_valid),
    .message_error(ack_bundle[37:35]),.message_token(ack_bundle[31:0]),
    .gmii_txen(gmii_txen),.gmii_txd(gmii_txd),.tx_done());
endmodule
