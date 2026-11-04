`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////

// 模块：imu_command_cdc
// 功能：使用打包数据CDC邮箱传输34位命令。
// 版本1.1.0：初始实现。
// src_rst_n与dst_rst_n同时拉低，共同复位清除待传命令。
//////////////////////////////////////////////////////////////////////////////////
module imu_command_cdc (
    input wire src_clk,
    input wire src_rst_n,
    input wire src_valid,
    output wire src_ready,
    input wire [1:0] src_op,
    input wire [31:0] src_token,
    input wire dst_clk,
    input wire dst_rst_n,
    output wire dst_valid,
    input wire dst_ready,
    output wire [1:0] dst_op,
    output wire [31:0] dst_token
);
    imu_bundle_cdc #(.WIDTH(34)) u_mailbox (
        .src_clk(src_clk), .src_rst_n(src_rst_n),
        .src_valid(src_valid), .src_ready(src_ready),
        .src_data({src_op, src_token}),
        .dst_clk(dst_clk), .dst_rst_n(dst_rst_n),
        .dst_valid(dst_valid), .dst_ready(dst_ready),
        .dst_data({dst_op, dst_token})
    );
endmodule
