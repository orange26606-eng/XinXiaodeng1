// 单主机I2C控制器，支持动态7位地址和1至14字节连续事务。
// 开漏输出、重复起始、末字节NACK及有界等待，连续读取成功后原子发布。
module imu_i2c_master #(
    parameter integer RETRY_MAX = 2,
    // 保留旧参数名称；此版本默认实际为 100kHz，不再是名称中的 250kHz。
    parameter integer DIV_250K = 500,
    parameter integer READ_BYTES = 14,
    parameter integer BUS_TIMEOUT_CLKS = 500000
)(
    input clk_sys,
    input rst_n,
    input [6:0] device_address,
    input [7:0] read_length,
    input [1:0] Start_Sig,
    input [7:0] Addr_Sig,
    input [7:0] WrData,
    output [7:0] RdData,
    output Done_Sig,
    inout scl,
    inout sda,
    output reg Busy_Sig,
    output reg [2:0] Error_Sig,
    output reg Data_Valid,
    output reg [READ_BYTES*8-1:0] RdBlock,
    output reg Byte_Valid,
    output reg [7:0] Byte_Index,
    output reg [31:0] Diag_MaxWait,
    output reg [15:0] Diag_Trace
);

// 对于 50MHz 时钟，默认低/高各 250 拍，即各 5us。
// DIV_250K 可为奇数，低相位取整，高相位补足，总拍数不丢失。
localparam integer LOW_CLKS = DIV_250K / 2;
localparam integer HIGH_CLKS = DIV_250K - LOW_CLKS;
localparam [2:0] ERR_NONE=0, ERR_NACK=1, ERR_SCL_TIMEOUT=2,
                 ERR_BUS_BUSY=3, ERR_COMMAND=4;
localparam [4:0] IDLE=0, BUS_FREE=1, START_HOLD=2,
    TX_LOW=3, TX_RISE=4, TX_HIGH=5,
    ACK_LOW=6, ACK_RISE=7, ACK_HIGH=8, ACK_FALL=9,
    REP_LOW=10, REP_RISE=11, REP_HIGH=12, REP_HOLD=13,
    RX_LOW=14, RX_RISE=15, RX_HIGH=16,
    MASTER_ACK_LOW=17, MASTER_ACK_RISE=18, MASTER_ACK_HIGH=19,
    MASTER_ACK_FALL=20, STOP_LOW=21, STOP_RISE=22, STOP_HIGH=23,
    STOP_FREE=24, FINISH=25, WAIT_RELEASE=26;

// 保留原文件中主要寄存器的名称。
reg [4:0] i;
reg [1:0] Go; // 0:写地址，1:寄存器，2:读地址，3:写数据
// C1既计低/高相位，也在*_RISE状态计SCL拉伸超时，必须覆盖超时上限。
function integer bits_for_max;
    input integer max_value;
    integer value;
    begin
        value=max_value; bits_for_max=0;
        while(value>0) begin bits_for_max=bits_for_max+1; value=value>>1; end
        if(bits_for_max<1) bits_for_max=1;
    end
endfunction
localparam integer C1_MAX = (DIV_250K > BUS_TIMEOUT_CLKS) ? DIV_250K : BUS_TIMEOUT_CLKS;
localparam integer C1_BITS = bits_for_max(C1_MAX-1);
localparam integer WAIT_BITS = bits_for_max(BUS_TIMEOUT_CLKS-1);
localparam integer RETRY_BITS = bits_for_max(RETRY_MAX);
reg [C1_BITS-1:0] C1;
reg [WAIT_BITS-1:0] wait_count;
reg rSCL, rSDA, isAck, isDone, isOut;
reg [7:0] rData;
reg [RETRY_BITS-1:0] retry_cnt;
reg sda_d0, sda_d1, scl_d0, scl_d1;
reg [2:0] bit_index;
reg [7:0] read_index, saved_addr, saved_write;
reg is_read, retry_pending;
reg [6:0] saved_device; reg [7:0] saved_length;
reg [READ_BYTES*8-1:0] pending_block;

// 连续等待SCL上升的最长时间，以clk_sys周期计。
// 轨迹字段：[15:11]状态、[10:9]字节组、[8:6]位编号，
// [5]主机释放SCL、[4]主机拉低SDA，
// [3]采样SCL、[2]采样SDA、[1:0]保留0。
wire [15:0] current_trace={i,Go,bit_index,rSCL,(isOut && !rSDA),scl_d1,sda_d1,2'b00};
wire waiting_scl=(i==TX_RISE || i==ACK_RISE || i==REP_RISE ||
                  i==RX_RISE || i==MASTER_ACK_RISE || i==STOP_RISE);
always @(posedge clk_sys or negedge rst_n) begin
    if(!rst_n) begin Diag_MaxWait<=0; Diag_Trace<=0; end
    else if(i==IDLE && Start_Sig!=0) begin Diag_MaxWait<=0; Diag_Trace<=0; end
    else begin
        if(waiting_scl && !scl_d1 && C1+32'd1>Diag_MaxWait) begin
            Diag_MaxWait<=C1+32'd1; Diag_Trace<=current_trace;
        end
        if((waiting_scl && !scl_d1 && C1>=BUS_TIMEOUT_CLKS-1) ||
           (i==ACK_FALL && isAck) ||
           ((i==BUS_FREE || i==STOP_FREE) && wait_count>=BUS_TIMEOUT_CLKS-1 &&
            !(scl_d1 && sda_d1 && C1>=HIGH_CLKS-1)))
            Diag_Trace<=current_trace;
    end
end

assign RdData = rData;
assign Done_Sig = isDone;
assign scl = rSCL ? 1'bz : 1'b0;
assign sda = (isOut && !rSDA) ? 1'b0 : 1'bz;

// 同步的是引脚实际电平，因此 SCL 被从机拉低时不会继续计高相位。
always @(posedge clk_sys or negedge rst_n) begin
    if(!rst_n) begin
        sda_d0<=1; sda_d1<=1; scl_d0<=1; scl_d1<=1;
    end else begin
        sda_d0<=sda; sda_d1<=sda_d0;
        scl_d0<=scl; scl_d1<=scl_d0;
    end
end

`ifndef SYNTHESIS
initial begin
    if(DIV_250K<8 || READ_BYTES<1 || READ_BYTES>256 ||
       RETRY_MAX<0 || BUS_TIMEOUT_CLKS<HIGH_CLKS+4) begin
        $error("XR_iic_com: illegal parameters");
        $finish;
    end
end
`endif

always @(posedge clk_sys or negedge rst_n) begin
    if(!rst_n) begin
        i<=IDLE; Go<=0; C1<=0; wait_count<=0;
        rSCL<=1; rSDA<=1; isOut<=0; isAck<=1; isDone<=0;
        rData<=0; retry_cnt<=0; bit_index<=7; read_index<=0;
        saved_addr<=0; saved_write<=0; is_read<=0; retry_pending<=0; saved_device<=0; saved_length<=1;
        pending_block<=0; RdBlock<=0; Byte_Index<=0;
        Byte_Valid<=0; Busy_Sig<=0; Error_Sig<=ERR_NONE; Data_Valid<=0;
    end else begin
        isDone<=0;
        Byte_Valid<=0;
        case(i)
        IDLE: begin
            rSCL<=1; rSDA<=1; isOut<=0; Busy_Sig<=0;
            if(Start_Sig!=2'b00) begin
                saved_addr<=Addr_Sig; saved_write<=WrData; saved_device<=device_address; saved_length<=read_length;
                is_read<=Start_Sig[1]; Busy_Sig<=1;
                Error_Sig<=ERR_NONE; Data_Valid<=0;
                pending_block<=0; retry_cnt<=0; retry_pending<=0;
                read_index<=0; C1<=0; wait_count<=0;
                if(Start_Sig==2'b11 || (Start_Sig[1] && (read_length==0 || read_length>READ_BYTES))) begin Error_Sig<=ERR_COMMAND; i<=FINISH; end
                else i<=BUS_FREE;
            end
        end
        BUS_FREE: begin
            // 起始前要求总线连续空闲；SDA/SCL 卡低均有等待上限。
            rSCL<=1; rSDA<=1; isOut<=0;
            if(scl_d1 && sda_d1 && C1>=HIGH_CLKS-1) begin
                isOut<=1; rSDA<=0; C1<=0; wait_count<=0; i<=START_HOLD;
            end else if(wait_count>=BUS_TIMEOUT_CLKS-1) begin
                Error_Sig<=ERR_BUS_BUSY; i<=FINISH;
            end else begin
                wait_count<=wait_count+1'b1;
                if(scl_d1 && sda_d1) C1<=C1+1'b1;
                else C1<=0;
            end
        end
        START_HOLD: begin
            if(C1>=HIGH_CLKS-1) begin
                rSCL<=0; rData<={saved_device,1'b0}; Go<=0;
                bit_index<=7; C1<=0; i<=TX_LOW;
            end else C1<=C1+1'b1;
        end
        TX_LOW: begin
            isOut<=1; rSDA<=rData[bit_index];
            if(C1>=LOW_CLKS-1) begin rSCL<=1; C1<=0; i<=TX_RISE; end
            else C1<=C1+1'b1;
        end
        TX_RISE, ACK_RISE, REP_RISE, RX_RISE, MASTER_ACK_RISE, STOP_RISE: begin
            // 释放 SCL 后，先确认引脚确实变高，再计高电平时间。
            if(scl_d1) begin
                C1<=0;
                case(i)
                    TX_RISE:i<=TX_HIGH;
                    ACK_RISE:i<=ACK_HIGH;
                    REP_RISE:i<=REP_HIGH;
                    RX_RISE:i<=RX_HIGH;
                    MASTER_ACK_RISE:i<=MASTER_ACK_HIGH;
                    default:i<=STOP_HIGH;
                endcase
            end else if(C1>=BUS_TIMEOUT_CLKS-1) begin
                // SCL 无法变高时不能制造 STOP；释放两根线后报告超时。
                rSCL<=1; isOut<=0; rSDA<=1;
                Error_Sig<=ERR_SCL_TIMEOUT; retry_pending<=0; i<=FINISH;
            end else C1<=C1+1'b1;
        end
        TX_HIGH: begin
            if(C1>=HIGH_CLKS-1) begin
                rSCL<=0; C1<=0;
                if(bit_index==0) i<=ACK_LOW;
                else begin bit_index<=bit_index-1'b1; i<=TX_LOW; end
            end else C1<=C1+1'b1;
        end
        ACK_LOW: begin
            isOut<=0; rSDA<=1;
            if(C1>=LOW_CLKS-1) begin rSCL<=1; C1<=0; i<=ACK_RISE; end
            else C1<=C1+1'b1;
        end
        ACK_HIGH: begin
            if(C1==HIGH_CLKS/2) isAck<=sda_d1;
            if(C1>=HIGH_CLKS-1) begin rSCL<=0; C1<=0; i<=ACK_FALL; end
            else C1<=C1+1'b1;
        end
        ACK_FALL: begin
            if(isAck) begin
                Error_Sig<=ERR_NACK;
                retry_pending<=(retry_cnt<RETRY_MAX);
                C1<=0; i<=STOP_LOW;
            end else begin
                bit_index<=7; C1<=0;
                case(Go)
                0: begin rData<=saved_addr; Go<=1; i<=TX_LOW; end
                1: if(is_read) i<=REP_LOW;
                   else begin rData<=saved_write; Go<=3; i<=TX_LOW; end
                2: begin rData<=0; read_index<=0; i<=RX_LOW; end
                default: i<=STOP_LOW;
                endcase
            end
        end
        REP_LOW: begin
            // SDA 在 SCL 低期间释放，不产生额外 STOP。
            isOut<=0; rSDA<=1;
            if(C1>=LOW_CLKS-1) begin rSCL<=1; C1<=0; i<=REP_RISE; end
            else C1<=C1+1'b1;
        end
        REP_HIGH: begin
            if(C1>=HIGH_CLKS-1) begin
                isOut<=1; rSDA<=0; C1<=0; i<=REP_HOLD;
            end else C1<=C1+1'b1;
        end
        REP_HOLD: begin
            if(C1>=HIGH_CLKS-1) begin
                rSCL<=0; rData<={saved_device,1'b1}; Go<=2;
                bit_index<=7; C1<=0; i<=TX_LOW;
            end else C1<=C1+1'b1;
        end
        RX_LOW: begin
            isOut<=0; rSDA<=1;
            if(C1>=LOW_CLKS-1) begin rSCL<=1; C1<=0; i<=RX_RISE; end
            else C1<=C1+1'b1;
        end
        RX_HIGH: begin
            if(C1==HIGH_CLKS/2) rData[bit_index]<=sda_d1;
            if(C1>=HIGH_CLKS-1) begin
                rSCL<=0; C1<=0;
                if(bit_index==0) begin
                    pending_block[read_index*8+:8]<=rData;
                    Byte_Index<=read_index; Byte_Valid<=1;
                    i<=MASTER_ACK_LOW;
                end else begin bit_index<=bit_index-1'b1; i<=RX_LOW; end
            end else C1<=C1+1'b1;
        end
        MASTER_ACK_LOW: begin
            // 前 READ_BYTES-1 字节 ACK=0；最后一字节 NACK=释放。
            isOut<=1; rSDA<=(read_index==saved_length-1);
            if(C1>=LOW_CLKS-1) begin rSCL<=1; C1<=0; i<=MASTER_ACK_RISE; end
            else C1<=C1+1'b1;
        end
        MASTER_ACK_HIGH: begin
            if(C1>=HIGH_CLKS-1) begin rSCL<=0; C1<=0; i<=MASTER_ACK_FALL; end
            else C1<=C1+1'b1;
        end
        MASTER_ACK_FALL: begin
            isOut<=0; rSDA<=1; C1<=0;
            if(read_index==saved_length-1) i<=STOP_LOW;
            else begin read_index<=read_index+1'b1; bit_index<=7; rData<=0; i<=RX_LOW; end
        end
        STOP_LOW: begin
            rSCL<=0; isOut<=1; rSDA<=0;
            if(C1>=LOW_CLKS-1) begin rSCL<=1; C1<=0; i<=STOP_RISE; end
            else C1<=C1+1'b1;
        end
        STOP_HIGH: begin
            if(C1>=HIGH_CLKS-1) begin
                isOut<=0; rSDA<=1; C1<=0; wait_count<=0; i<=STOP_FREE;
            end else C1<=C1+1'b1;
        end
        STOP_FREE: begin
            if(scl_d1 && sda_d1 && C1>=HIGH_CLKS-1) begin
                C1<=0;
                if(retry_pending) begin
                    retry_cnt<=retry_cnt+1'b1; retry_pending<=0;
                    Error_Sig<=ERR_NONE; pending_block<=0; read_index<=0;
                    wait_count<=0; i<=BUS_FREE;
                end else i<=FINISH;
            end else if(wait_count>=BUS_TIMEOUT_CLKS-1) begin
                Error_Sig<=ERR_BUS_BUSY; retry_pending<=0; i<=FINISH;
            end else begin
                wait_count<=wait_count+1'b1;
                if(scl_d1 && sda_d1) C1<=C1+1'b1;
                else C1<=0;
            end
        end
        FINISH: begin
            rSCL<=1; rSDA<=1; isOut<=0; Busy_Sig<=0; isDone<=1;
            if(is_read && Error_Sig==ERR_NONE) begin
                RdBlock<=pending_block; Data_Valid<=1;
            end
            // 单拍请求早已撤销时直接回空闲；下一拍可接受下一笔请求。
            i<=(Start_Sig==0)?IDLE:WAIT_RELEASE;
        end
        WAIT_RELEASE: begin
            // 兼容旧版持续使能，也接受单拍请求，避免保持使能导致反复重启。
            if(Start_Sig==0) i<=IDLE;
        end
        default: begin
            rSCL<=1; rSDA<=1; isOut<=0;
            Error_Sig<=ERR_COMMAND; Data_Valid<=0; i<=FINISH;
        end
        endcase
    end
end
endmodule
