`timescale 1ns/1ps


// 日期：2026-10-07 15:13
// 模块：XR_quad_decoder
// 版本：1.2.0

// A/B正交四倍频解码。只支持两相编码器；单路霍尔完整周期会相互抵消。
// 端口单位：pos_cnt为32位自然回绕的净计数；speed_ticks为窗口净计数（饱和16位）。
// initialized：同步链填满且首个状态稳定后置1，初始状态本身不算运动。
// invalid_count：稳定但非法的对角跳变次数，饱和不回绕；仅诊断，不在本版触发故障。
// step_valid：接受合法跳变后的单拍通知，未来可供无脉冲保护使用。
// 默认50MHz。P_STABLE_CLKS=25表示候选捕获后再匹配26拍才接受。
// 测速窗口为P_WINDOW_CLKS+1周期，监测端使用CLK_HZ/10形成精确100ms窗口。
// 计数器扩为32位，以容纳50MHz下的5000000周期。
// 锁存当拍的合法跳变转入下一窗口，计数与位置均只接收一次。
module XR_encoder_quad #(
    parameter [5:0] P_STABLE_CLKS=6'd25,
    parameter integer P_WINDOW_CLKS=4999999
)(
    input clk_sys,input rst_n,input enc_a,input enc_b,
    output reg [31:0] pos_cnt,
    output reg signed [15:0] speed_ticks,
    output reg window_end,
    output reg initialized,
    output reg [31:0] invalid_count,
    output wire [1:0] ab_state,
    output reg step_valid
);
reg [1:0] a_sync,b_sync,sync_filled;
always @(posedge clk_sys or negedge rst_n) begin
    if(!rst_n) begin a_sync<=0;b_sync<=0;sync_filled<=0;end
    else begin
        a_sync<={a_sync[0],enc_a};b_sync<={b_sync[0],enc_b};
        sync_filled<={sync_filled[0],1'b1};
    end
end
wire a_f=a_sync[1],b_f=b_sync[1];
wire [1:0] synced_ab={a_f,b_f};
reg [1:0] state,cand;
assign ab_state=state;
reg [5:0] stab_cnt;
// 提交条件包含当前同步输入。输入变化时撤销候选。
// 状态、位置、窗口计数和诊断共用commit。
wire commit=initialized && (synced_ab==cand) && (cand!=state) &&
            (stab_cnt==P_STABLE_CLKS);
wire fwd=commit && (({state,cand}==4'b0010)||({state,cand}==4'b1011)||
                   ({state,cand}==4'b1101)||({state,cand}==4'b0100));
wire rev=commit && (({state,cand}==4'b0001)||({state,cand}==4'b0111)||
                   ({state,cand}==4'b1110)||({state,cand}==4'b1000));
wire signed [1:0] commit_inc=fwd?2'sd1:(rev?-2'sd1:2'sd0);
always @(posedge clk_sys or negedge rst_n) begin
    // 异步复位加载常量，A/B输入经同步后再建立初始状态。
    if(!rst_n) begin state<=0;cand<=0;stab_cnt<=0;initialized<=0;end
    else if(!sync_filled[1]) begin state<=0;cand<=0;stab_cnt<=0;initialized<=0;end
    else if(synced_ab!=cand) begin cand<=synced_ab;stab_cnt<=0;end
    else if(!initialized) begin
        if(stab_cnt==P_STABLE_CLKS) begin
            state<=cand;initialized<=1;stab_cnt<=0;
        end else stab_cnt<=stab_cnt+1'b1;
    end else if(commit) begin
        // 非法跳变也更新已接受状态以重新同步，但其commit_inc为0。
        state<=cand;stab_cnt<=0;
    end else if(cand!=state) stab_cnt<=stab_cnt+1'b1;
end
always @(posedge clk_sys or negedge rst_n) begin
    if(!rst_n) begin pos_cnt<=0;invalid_count<=0;step_valid<=0;end
    else begin
        step_valid<=fwd || rev;
        if(fwd)pos_cnt<=pos_cnt+1'b1;
        else if(rev)pos_cnt<=pos_cnt-1'b1;
        if(commit && !fwd && !rev && invalid_count!=32'hFFFFFFFF)
            invalid_count<=invalid_count+1'b1;
    end
end
reg [31:0] win_cnt;
reg signed [23:0] win_acc;
function [15:0] sat16;
    input signed [23:0] value;
    begin
        if(value>24'sd32767)sat16=16'sd32767;
        else if(value < -24'sd32768)sat16=-16'sd32768;
        else sat16=value[15:0];
    end
endfunction
always @(posedge clk_sys or negedge rst_n) begin
    if(!rst_n) begin win_cnt<=0;win_acc<=0;speed_ticks<=0;window_end<=0;end
    else if(win_cnt==P_WINDOW_CLKS) begin
        win_cnt<=0;window_end<=1;speed_ticks<=sat16(win_acc);win_acc<=commit_inc;
    end else begin
        win_cnt<=win_cnt+1'b1;window_end<=0;win_acc<=win_acc+commit_inc;
    end
end
endmodule
