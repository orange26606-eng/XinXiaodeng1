`timescale 1ns / 1ps
//=============================================================

// 模块：XR_key_debounce
// 功能: 两级同步后确认按键电平连续稳定，输出高电平表示按下。
//=============================================================
module XR_key_debounce #(
    parameter integer P_DEBOUNCE_CLKS = 1000000 // 输入电平连续稳定的最少时钟拍数
)(
    input  wire clk_sys,
    input  wire rst_n,
    input  wire key_n,
    output reg  pressed
);

// 异步按键先经过两级触发器同步，再进入消抖判断。
reg key_meta, key_sync;
reg [31:0] stable_cnt;

always @(posedge clk_sys or negedge rst_n) begin
    if (!rst_n) begin
        key_meta   <= 1'b1;
        key_sync   <= 1'b1;
        pressed    <= 1'b0;
        stable_cnt <= 32'd0;
    end else begin
        key_meta <= key_n;
        key_sync <= key_meta;
        // 当前同步电平与输出一致时清零；变化持续足够久才更新输出。
        if (pressed == ~key_sync) begin
            stable_cnt <= 32'd0;
        end else if (stable_cnt >= P_DEBOUNCE_CLKS - 1) begin
            pressed    <= ~key_sync;
            stable_cnt <= 32'd0;
        end else begin
            stable_cnt <= stable_cnt + 1'b1;
        end
    end
end

endmodule
