`timescale 1ns/1ps
//-------------------------------


// 日期：2026-10-12 18:20
// 模块：gmii_tx_frame_arbiter
// 功能: 两个发送源在启动前仲裁，保持完整GMII帧及至少12字节帧间隙。
// 版本：1.1.1
// 修订记录 ： v1.1.0----创建camera/IMU整帧仲裁，锁定源直到实际TXEN下降，IMU在空闲仲裁时优先。
//                   v1.1.1----待发请求在接受前撤销时释放仲裁，接受后覆盖CRC准备到整帧结束，避免校准切换锁死。
//-------------------------------
module gmii_tx_frame_arbiter #(
    parameter integer IFG_CLKS = 12
)(
    input clk125m,
    input rst_n,
    input cam_pending,
    input cam_txen,
    input [7:0] cam_txd,
    output cam_grant,
    input imu_pending,
    input imu_txen,
    input [7:0] imu_txd,
    output imu_grant,
    output gmii_txen,
    output [7:0] gmii_txd
);
    localparam [1:0] IDLE=0, CAMERA=1, IMU=2, GAP=3;
    reg [1:0] owner;
    reg frame_started;
    reg frame_committed;
    reg [31:0] gap_count;
    wire selected_txen = (owner==CAMERA) ? cam_txen : imu_txen;
    wire selected_pending = (owner==CAMERA) ? cam_pending : imu_pending;
    assign cam_grant = rst_n && (owner==CAMERA) && !frame_committed;
    assign imu_grant = rst_n && (owner==IMU) && !frame_committed;
    assign gmii_txen = rst_n && ((owner==CAMERA && cam_txen) ||
                                (owner==IMU && imu_txen));
    assign gmii_txd = (owner==CAMERA) ? cam_txd :
                      (owner==IMU) ? imu_txd : 8'h00;
    always @(posedge clk125m or negedge rst_n) begin
        if (!rst_n) begin
            owner<=IDLE; frame_started<=0; frame_committed<=0; gap_count<=0;
        end else begin
            case (owner)
                IDLE: begin
                    frame_started<=0; frame_committed<=0; gap_count<=0;
                    if (imu_pending) owner<=IMU;
                    else if (cam_pending) owner<=CAMERA;
                end
                CAMERA, IMU: begin
                    // 生产者可在接受边沿之前撤销请求，
                    // 例如校准状态禁用之前待发的角度。
                    // 请求一经接受，在CRC准备期间持续保留发送权。
                    if (!frame_committed) begin
                        if (selected_pending) frame_committed<=1;
                        else if (!selected_txen) owner<=IDLE;
                    end
                    if (selected_txen) frame_started<=1;
                    else if (frame_started) begin
                        owner<=GAP; gap_count<=0; frame_started<=0; frame_committed<=0;
                    end
                end
                GAP: begin
                    if (gap_count >= IFG_CLKS-1) begin
                        owner<=IDLE; gap_count<=0;
                    end else gap_count<=gap_count+1'b1;
                end
                default: begin
                    owner<=IDLE; frame_started<=0; frame_committed<=0; gap_count<=0;
                end
            endcase
        end
    end
endmodule
