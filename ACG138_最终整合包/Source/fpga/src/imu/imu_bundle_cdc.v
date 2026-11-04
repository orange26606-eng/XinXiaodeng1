`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////

// 模块：imu_bundle_cdc
// 功能：单项打包数据请求与确认CDC邮箱。
// 版本1.1.0：初始实现。
// 版本1.1.1：局部复位仅使用同步后的复位信号，避免
// 异步全局复位进入同步数据使能路径。
// 两端须同时进入复位，复位释放时刻允许不同，
// 各端独立将释放同步到本地时钟。
// 共同复位会丢弃正在传递的数据，翻转握手邮箱
// 不支持只复位其中一端。
//////////////////////////////////////////////////////////////////////////////////
module imu_bundle_cdc #(
    parameter integer WIDTH = 34
) (
    input wire src_clk,
    input wire src_rst_n,
    input wire src_valid,
    output wire src_ready,
    input wire [WIDTH-1:0] src_data,
    input wire dst_clk,
    input wire dst_rst_n,
    output reg dst_valid,
    input wire dst_ready,
    output reg [WIDTH-1:0] dst_data
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] src_reset_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] dst_reset_sync;
    wire src_local_rst_n = src_reset_sync[1];
    wire dst_local_rst_n = dst_reset_sync[1];

    always @(posedge src_clk or negedge src_rst_n)
        if (!src_rst_n) src_reset_sync <= 2'b00;
        else src_reset_sync <= {src_reset_sync[0], 1'b1};
    always @(posedge dst_clk or negedge dst_rst_n)
        if (!dst_rst_n) dst_reset_sync <= 2'b00;
        else dst_reset_sync <= {dst_reset_sync[0], 1'b1};

    reg request_toggle;
    reg acknowledge_toggle;
    reg [WIDTH-1:0] source_hold;
    (* ASYNC_REG = "TRUE" *) reg acknowledge_meta;
    (* ASYNC_REG = "TRUE" *) reg acknowledge_sync;
    (* ASYNC_REG = "TRUE" *) reg request_meta;
    (* ASYNC_REG = "TRUE" *) reg request_sync;

    // source_hold在同步、目标端锁存及目标端背压期间
    // 保持不变，只有确认信号
    // 经过源时钟域两级同步返回后才能更新。
    assign src_ready = src_local_rst_n &&
                       (request_toggle == acknowledge_sync);

    always @(posedge src_clk or negedge src_local_rst_n) begin
        if (!src_local_rst_n) begin
            acknowledge_meta <= 1'b0;
            acknowledge_sync <= 1'b0;
            request_toggle <= 1'b0;
            source_hold <= {WIDTH{1'b0}};
        end else begin
            acknowledge_meta <= acknowledge_toggle;
            acknowledge_sync <= acknowledge_meta;
            if (src_valid && src_ready) begin
                source_hold <= src_data;
                request_toggle <= ~request_toggle;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_local_rst_n) begin
        if (!dst_local_rst_n) begin
            request_meta <= 1'b0;
            request_sync <= 1'b0;
            acknowledge_toggle <= 1'b0;
            dst_valid <= 1'b0;
            dst_data <= {WIDTH{1'b0}};
        end else begin
            request_meta <= request_toggle;
            request_sync <= request_meta;
            if (!dst_valid && (request_sync != acknowledge_toggle)) begin
                // 请求经过目标端两级同步期间，源端已保持整组数据，
                // 目标端锁存一次，并保持到消费者就绪。
                dst_data <= source_hold;
                dst_valid <= 1'b1;
            end else if (dst_valid && dst_ready) begin
                dst_valid <= 1'b0;
                acknowledge_toggle <= request_sync;
            end
        end
    end
endmodule
