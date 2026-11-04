`timescale 1ns/1ps
// 四组独立正交解码器、一致快照及ASCII串口报告。
// 本模块只监测，不接入电机使能或命令逻辑。
// 默认测速窗口100ms，输出轴每圈1320计数。
module XR_encoder_monitor #(
    parameter integer CLK_HZ=50000000,
    parameter integer BAUD=115200,
    parameter integer REPORT_CLKS=10000000,
    parameter integer COUNTS_PER_REV=1320,
    parameter integer STABLE_CLKS=25,
    parameter [3:0] ENCODER_INVERT=4'b0000
)(
    input clk,rst_n,
    input [127:0] positions,errors,
    input [63:0] deltas,input [7:0] ab_states,input [3:0] ready,
    input trigger,output active,
    input [3:0] motion_code,
    output serial_out
);
reg [31:0] report_count;
reg [127:0] snap_positions,snap_errors;
reg [63:0] snap_deltas;
reg [7:0] snap_ab;
reg [3:0] snap_ready,snap_cmd;
reg [1:0] row,field;
reg sign_position,sign_delta;
reg [31:0] position_mag,delta_mag,rpm_mag,error_mag;
function [31:0] abs32;
    input [31:0] v;
    begin abs32=v[31]?(~v+32'd1):v;end
endfunction
function [31:0] abs16;
    input [15:0] v;
    begin abs16=v[15]?{16'd0,(~v+16'd1)}:{16'd0,v};end
endfunction
function [7:0] hex_ascii;
    input [3:0] v;
    begin hex_ascii=v<10?(8'd48+v):(8'd55+v);end
endfunction
function [79:0] decimal_ascii;
    input [39:0] v;
    integer d;
    begin
        for(d=0;d<10;d=d+1)decimal_ascii[d*8 +: 8]=8'd48+v[d*4 +: 4];
    end
endfunction

localparam IDLE=0,ROW_LOAD=1,DIVIDE=2,BCD_START=3,BCD_WAIT=4,PACK=5,SEND=6,TX_WAIT=7,SNAPSHOT=8;
reg [3:0] state;
assign active=state!=IDLE;
reg [31:0] div_number,div_quotient;
reg [32:0] div_rem;
reg [5:0] div_count;
wire [32:0] div_shift={div_rem[31:0],div_number[31]};
wire div_ge=(div_shift >= COUNTS_PER_REV);
wire [31:0] quotient_next={div_quotient[30:0],div_ge};
wire [32:0] remainder_next=div_ge?(div_shift-COUNTS_PER_REV):div_shift;
// 转速以0.1RPM计，按abs(100ms计数)×6000/CPR计算并四舍五入。
// 使用迭代除法器，避免50MHz时钟域出现长组合除法路径。
wire [31:0] row_delta_mag=abs16(snap_deltas[row*16 +: 16]);
reg [31:0] convert_value;
always @* case(field)
    0:convert_value=position_mag;
    1:convert_value=delta_mag;
    2:convert_value=rpm_mag;
    default:convert_value=error_mag;
endcase
wire bcd_done,bcd_busy;
wire [39:0] bcd_result;
XR_bin32_bcd u_decimal(.clk(clk),.rst_n(rst_n),.start(state==BCD_START),
    .value(convert_value),.busy(bcd_busy),.done(bcd_done),.bcd(bcd_result));
reg [79:0] position_text,delta_text,rpm_text,error_text;
// 字段：M1 AB=00 INIT=1 CNT=+0000000000 D=+00000 RPM=+00000.0 ERR=0000000000 CMD=0 CR LF
localparam integer LINE_BYTES=76;
reg [LINE_BYTES*8-1:0] packet;
reg [6:0] byte_index;
wire tx_busy;
uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_uart(
    .clk(clk),.rst(!rst_n),.data(packet[LINE_BYTES*8-1 -: 8]),
    .valid(state==SEND),.busy(tx_busy),.serial_out(serial_out));

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state<=IDLE;report_count<=0;row<=0;field<=0;byte_index<=0;
        snap_positions<=0;snap_errors<=0;snap_deltas<=0;snap_ab<=0;snap_ready<=0;snap_cmd<=0;
        sign_position<=0;sign_delta<=0;position_mag<=0;delta_mag<=0;rpm_mag<=0;error_mag<=0;
        div_number<=0;div_quotient<=0;div_rem<=0;div_count<=0;
        position_text<=0;delta_text<=0;rpm_text<=0;error_text<=0;packet<=0;
    end else begin
        if(report_count>=REPORT_CLKS-1)report_count<=0;
        else report_count<=report_count+1'b1;
        case(state)
        IDLE:if(trigger)state<=SNAPSHOT;
        SNAPSHOT:begin
            snap_positions<=positions;snap_errors<=errors;snap_deltas<=deltas;
            snap_ab<=ab_states;snap_ready<=ready;snap_cmd<=motion_code;
            row<=0;state<=ROW_LOAD;
        end
        ROW_LOAD:begin
            position_mag<=abs32(snap_positions[row*32 +: 32]);
            sign_position<=snap_positions[row*32+31];
            delta_mag<=row_delta_mag;sign_delta<=snap_deltas[row*16+15];
            error_mag<=snap_errors[row*32 +: 32];
            div_number<=row_delta_mag*32'd6000+COUNTS_PER_REV/2;
            div_quotient<=0;div_rem<=0;div_count<=0;state<=DIVIDE;
        end
        DIVIDE:begin
            div_rem<=remainder_next;div_quotient<=quotient_next;
            div_number<={div_number[30:0],1'b0};
            if(div_count==31)begin rpm_mag<=quotient_next;field<=0;state<=BCD_START;end
            else div_count<=div_count+1'b1;
        end
        BCD_START:state<=BCD_WAIT;
        BCD_WAIT:if(bcd_done)begin
            case(field)
            0:position_text<=decimal_ascii(bcd_result);
            1:delta_text<=decimal_ascii(bcd_result);
            2:rpm_text<=decimal_ascii(bcd_result);
            3:error_text<=decimal_ascii(bcd_result);
            endcase
            if(field==3)state<=PACK;
            else begin field<=field+1'b1;state<=BCD_START;end
        end
        PACK:begin
            packet<={"M",(8'd49+{6'd0,row})," AB=",
                (snap_ab[row*2+1]?8'h31:8'h30),(snap_ab[row*2]?8'h31:8'h30),
                " INIT=",(snap_ready[row]?8'h31:8'h30),
                " CNT=",(sign_position?8'h2d:8'h2b),position_text,
                " D=",(sign_delta?8'h2d:8'h2b),delta_text[39:0],
                " RPM=",(sign_delta?8'h2d:8'h2b),rpm_text[47:8],".",rpm_text[7:0],
                " ERR=",error_text," CMD=",hex_ascii(snap_cmd),8'h0d,8'h0a};
            byte_index<=0;state<=SEND;
        end
        SEND:if(!tx_busy)begin packet<=packet<<8;state<=TX_WAIT;end
        TX_WAIT:if(!tx_busy)begin
            if(byte_index==LINE_BYTES-1)begin
                if(row==3)state<=IDLE;
                else begin row<=row+1'b1;state<=ROW_LOAD;end
            end else begin byte_index<=byte_index+1'b1;state<=SEND;end
        end
        default:state<=IDLE;
        endcase
    end
end
endmodule
