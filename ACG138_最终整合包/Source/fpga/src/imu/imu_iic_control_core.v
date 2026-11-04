`timescale 1ns/1ps
//-------------------------------


// 日期：2026-10-12 19:02
// 模块：imu_iic_control_core
// 功能: 50MHz单一控制状态机设置九轴并启动一次静置校准，等待用户确认后50Hz读取姿态；重发命令幂等。
// 版本：1.1.4
// 修订记录 ： v1.1.0----50MHz单一控制状态机设置九轴并启动一次静置校准，等待用户确认后50Hz读取姿态；重发命令幂等。
//                   v1.1.1----重复READY确认仅重发回执，保留背压样本和轮询节拍。
//                   v1.1.2----诊断副本：失败后D1/D2/D3闪烁分别定位写算法、读算法、写校准；通信行为不变。
//                   v1.1.3----将诊断always块移到寄存器声明后，兼容Verilog语法检查。
//                   v1.1.4----配置事务超时独立放宽至250ms；按厂家例程写算法后等待1秒；输出UART诊断快照。
//-------------------------------

module imu_iic_control_core #(
    parameter integer BOOT_WAIT_CLKS=250000000,
    parameter integer POLL_CLKS=1000000,
    parameter integer IIC_DIV=500,
    parameter integer DIAG_BLINK_CLKS=12500000,
    parameter integer CONFIG_TIMEOUT_CLKS=12500000,
    parameter integer ALGO_SETTLE_CLKS=50000000,
    parameter integer IIC_TIMEOUT_CLKS=500000
)(
    input clk_iic, input rst_n,
    inout imu_sda, inout imu_scl, output [3:0] led_out,
    input cmd_valid, output cmd_ready,
    input [1:0] cmd_op, input [31:0] cmd_token,
    output reg ack_valid, input ack_ready,
    output reg [2:0] ack_state, ack_error, output reg [31:0] ack_token,
    output reg sample_valid, input sample_ready,
    output reg [2:0] sample_status, output reg [95:0] sample_angles,
    output stream_enabled, output calibration_wait, output reg [63:0] config_diag, output reg config_diag_valid
);
localparam [2:0] NONE=0, WAIT_LED=1, READY=2, FAILED=3, BUSY=4;
localparam [3:0] IDLE=0, WRITE_ALGO=1, WAIT_ALGO_WRITE=2,
    READ_ALGO=3, WAIT_ALGO_READ=4, WRITE_CALIB=5, WAIT_CALIB_WRITE=6, WAIT_ALGO_SETTLE=7;
reg [3:0] phase;
localparam integer SETTLE_BITS=$clog2(ALGO_SETTLE_CLKS>1 ? ALGO_SETTLE_CLKS : 2);
reg [SETTLE_BITS-1:0] settle_count;
wire [31:0] config_max_wait;
wire [15:0] config_trace;
// 状态仅供诊断，不反馈至IIC主机或网络控制。
reg [1:0] failed_step;
localparam integer BLINK_BITS=$clog2(DIAG_BLINK_CLKS>1 ? DIAG_BLINK_CLKS : 2);
reg [BLINK_BITS-1:0] diag_blink_count;
reg diag_blink;

reg [2:0] session_state, session_error;
reg [31:0] session_token, boot_count;
reg boot_ready, have_session;
reg [31:0] poll_count;
reg read_start, read_pending;
wire read_done, read_busy, read_valid;
wire [2:0] read_error;
wire [95:0] read_block;
reg [1:0] config_start;
reg [7:0] config_address, config_data;
wire config_done, config_busy, config_valid;
wire [2:0] config_error;
wire [7:0] config_read;
reg success_toggle, last_error, change_toggle;
reg [95:0] last_good_angles;
wire iic_rst_n=rst_n && boot_ready;
assign stream_enabled=have_session && (session_state==READY);
assign calibration_wait=have_session && (session_state==WAIT_LED);
assign cmd_ready=rst_n && (phase==IDLE) && !ack_valid &&
                 !read_pending && !read_busy && !config_busy;

always @(posedge clk_iic or negedge rst_n) begin
    if(!rst_n) begin diag_blink_count<=0; diag_blink<=0; end
    else if(session_state!=FAILED) begin diag_blink_count<=0; diag_blink<=0; end
    else if(diag_blink_count==DIAG_BLINK_CLKS-1) begin
        diag_blink_count<=0; diag_blink<=!diag_blink;
    end else diag_blink_count<=diag_blink_count+1'b1;
end

// 保存最近完成的配置事务，UART报告不阻塞IIC。
// 64位依次包含A2格式、4位阶段、4位错误、16位轨迹及32位最长SCL等待。
always @(posedge clk_iic or negedge rst_n) begin
    if(!rst_n) begin config_diag<=0; config_diag_valid<=0; end
    else if(cmd_valid && cmd_ready && cmd_op==1 &&
            (!have_session || cmd_token!=session_token) && session_state!=WAIT_LED) begin
        config_diag<=0; config_diag_valid<=0;
    end else if(config_done) begin
        config_diag<={8'hA2,phase,1'b0,config_error,config_trace,config_max_wait};
        config_diag_valid<=1;
    end
end

// 两个实例空闲时均释放SDA和SCL，状态机不同时启动两者。
// 配置事务不重试，NACK之前可能已有副作用，CALIB不能重复执行。
XR_iic_com #(.DEV_ADDR7(7'h23),.READ_BYTES(1),.DIV_250K(IIC_DIV),
    .RETRY_MAX(0),.BUS_TIMEOUT_CLKS(CONFIG_TIMEOUT_CLKS)) u_config (
    .clk_sys(clk_iic),.rst_n(iic_rst_n),.Start_Sig(config_start),
    .Addr_Sig(config_address),.WrData(config_data),.RdData(),
    .Done_Sig(config_done),.scl(imu_scl),.sda(imu_sda),
    .Busy_Sig(config_busy),.Error_Sig(config_error),.Data_Valid(config_valid),
    .RdBlock(config_read),.Byte_Valid(),.Byte_Index(),
    .Diag_MaxWait(config_max_wait),.Diag_Trace(config_trace)
);
XR_iic_com #(.DEV_ADDR7(7'h23),.READ_BYTES(12),.DIV_250K(IIC_DIV),
    .RETRY_MAX(2),.BUS_TIMEOUT_CLKS(IIC_TIMEOUT_CLKS)) u_reader (
    .clk_sys(clk_iic),.rst_n(iic_rst_n),.Start_Sig({read_start,1'b0}),
    .Addr_Sig(8'h26),.WrData(8'h00),.RdData(),.Done_Sig(read_done),
    .scl(imu_scl),.sda(imu_sda),.Busy_Sig(read_busy),.Error_Sig(read_error),
    .Data_Valid(read_valid),.RdBlock(read_block),.Byte_Valid(),.Byte_Index()
);

always @(posedge clk_iic or negedge rst_n) begin
    if(!rst_n) begin boot_count<=0; boot_ready<=0; end
    else if(!boot_ready) begin
        if(boot_count>=BOOT_WAIT_CLKS-1) boot_ready<=1;
        else boot_count<=boot_count+1'b1;
    end
end

always @(posedge clk_iic or negedge rst_n) begin
    if(!rst_n) begin
        phase<=IDLE; settle_count<=0; failed_step<=0; session_state<=NONE; session_error<=0;
        session_token<=0; have_session<=0;
        ack_valid<=0; ack_state<=0; ack_error<=0; ack_token<=0;
        sample_valid<=0; sample_status<=0; sample_angles<=0;
        poll_count<=0; read_start<=0; read_pending<=0;
        config_start<=0; config_address<=0; config_data<=0;
        success_toggle<=0; last_error<=0; change_toggle<=0; last_good_angles<=0;
    end else begin
        config_start<=0; read_start<=0;
        if(ack_valid && ack_ready) ack_valid<=0;
        if(sample_valid && sample_ready) sample_valid<=0;
        if(poll_count!=0) poll_count<=poll_count-1'b1;

        if(cmd_valid && cmd_ready) begin
            // 每个接受的请求仅产生一个响应，重试复用同一令牌。
            ack_token<=cmd_token; ack_error<=0;
            if(cmd_op==1) begin
                if(have_session && cmd_token==session_token) begin
                    ack_state<=session_state; ack_error<=session_error; ack_valid<=1;
                end else if(session_state==WAIT_LED) begin
                    ack_state<=BUSY; ack_valid<=1;
                end else begin
                    have_session<=1; session_token<=cmd_token;
                    session_state<=NONE; session_error<=0;
                    sample_valid<=0; sample_angles<=0; last_error<=0;
                    failed_step<=0; phase<=WRITE_ALGO;
                end
            end else if(cmd_op==2) begin
                if(have_session && cmd_token==session_token &&
                   (session_state==WAIT_LED || session_state==READY)) begin
                    ack_state<=READY; ack_valid<=1;
                    // 重复READY仅重发确认，保留待处理数据。
                    if(session_state==WAIT_LED) begin
                        session_state<=READY; session_error<=0;
                        sample_valid<=0; poll_count<=0;
                    end
                end else begin ack_state<=BUSY; ack_valid<=1; end
            end else begin ack_state<=FAILED; ack_error<=5; ack_valid<=1; end
        end else if(phase!=IDLE) begin
            case(phase)
                WRITE_ALGO:if(boot_ready && !config_busy && !read_busy) begin
                    config_address<=8'h61; config_data<=8'd9;
                    config_start<=2'b01; phase<=WAIT_ALGO_WRITE;
                end
                WAIT_ALGO_WRITE:if(config_done) begin
                    if(config_error!=0) begin
                        failed_step<=1;
                        session_state<=FAILED; session_error<=config_error; last_error<=1;
                        ack_state<=FAILED; ack_error<=config_error;
                        ack_token<=session_token; ack_valid<=1; phase<=IDLE;
                    end else begin settle_count<=0; phase<=WAIT_ALGO_SETTLE; end
                end
                // 厂商YbImuI2cLib.set_algo_type在写入后等待1秒。
                WAIT_ALGO_SETTLE:begin
                    if(settle_count>=ALGO_SETTLE_CLKS-1) phase<=READ_ALGO;
                    else settle_count<=settle_count+1'b1;
                end
                READ_ALGO:if(!config_busy) begin
                    config_address<=8'h61; config_start<=2'b10;
                    phase<=WAIT_ALGO_READ;
                end
                WAIT_ALGO_READ:if(config_done) begin
                    if(config_error!=0 || !config_valid || config_read!=8'd9) begin
                        failed_step<=2;
                        session_state<=FAILED; last_error<=1;
                        session_error<=config_error!=0 ? config_error :
                                       (!config_valid ? 3'd5 : 3'd6);
                        ack_state<=FAILED;
                        ack_error<=config_error!=0 ? config_error :
                                   (!config_valid ? 3'd5 : 3'd6);
                        ack_token<=session_token; ack_valid<=1; phase<=IDLE;
                    end else phase<=WRITE_CALIB;
                end
                WRITE_CALIB:if(!config_busy) begin
                    config_address<=8'h70; config_data<=8'd1;
                    config_start<=2'b01; phase<=WAIT_CALIB_WRITE;
                end
                WAIT_CALIB_WRITE:if(config_done) begin
                    failed_step<=config_error==0 ? 0 : 3;
                    session_state<=config_error==0 ? WAIT_LED : FAILED;
                    session_error<=config_error; last_error<=config_error!=0;
                    ack_state<=config_error==0 ? WAIT_LED : FAILED;
                    ack_error<=config_error; ack_token<=session_token;
                    ack_valid<=1; phase<=IDLE;
                end
                default:phase<=IDLE;
            endcase
        end else if(boot_ready && stream_enabled && !read_busy &&
                    !read_pending && !sample_valid && poll_count==0) begin
            read_start<=1; read_pending<=1; poll_count<=POLL_CLKS-1;
        end

        if(read_done) begin
            read_pending<=0;
            if(stream_enabled) begin
                sample_valid<=1;
                if(read_valid && read_error==0) begin
                    sample_status<=0; sample_angles<=read_block;
                    success_toggle<=!success_toggle; last_error<=0;
                    if(read_block!=last_good_angles) change_toggle<=!change_toggle;
                    last_good_angles<=read_block;
                end else begin
                    sample_status<=read_error==0 ? 3'd5 : read_error;
                    sample_angles<=0; last_error<=1;
                end
            end
        end
    end
end
// 配置失败时D1、D2、D3中对应的一盏以2Hz闪烁。
// 闪烁阶段含义独立于LED的实际有效电平。
assign led_out=(session_state==FAILED) ?
    {diag_blink && failed_step==3, diag_blink && failed_step==2,
     diag_blink && failed_step==1, boot_ready} :
    {change_toggle,last_error,success_toggle,boot_ready};
endmodule
