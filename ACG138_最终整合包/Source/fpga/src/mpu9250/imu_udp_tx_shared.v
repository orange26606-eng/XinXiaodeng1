`timescale 1ns/1ps
// I9E1与F6A1共用100字节载荷发送器，每帧锁存5016或5017端口。
// 沿用v11的odom_udp_tx，IPv4的UDP校验和为0。
module imu_udp_tx_shared #(
    parameter [47:0] DST_MAC = 48'hFFFFFFFFFFFF,
    parameter [47:0] SRC_MAC = 48'h000A3501FEC0,
    parameter [31:0] DST_IP = 32'hC0A80003,
    parameter [31:0] SRC_IP = 32'hC0A80002,
    parameter [15:0] DST_PORT = 16'd5016,
    parameter [15:0] SRC_PORT = 16'd5016,
    parameter [31:0] SEQ_INIT = 32'd0
)(
    input clk125m, input rst_n,
    input sample_valid, output sample_ready,
    input [767:0] sample_data,input attitude_frame,
    output reg gmii_txen, output reg [7:0] gmii_txd,
    output reg tx_done
);
localparam IDLE=0, APP_CRC=1, SEND=2, IFG=3, LOAD=4;
reg [2:0] state;
reg eth_crc_valid_d;
reg attitude_latched;
reg [767:0] data_latched;
reg [7:0] byte_index;
reg [6:0] crc_index;
// 连续输入管线在发送许可边沿锁存完整一致正文。
// 邮箱可在该边沿立即确认，不依赖延后数据保持假设。
reg [767:0] sample_pipe;
always @(posedge clk125m) sample_pipe <= sample_data;
genvar bank;
generate for(bank=0;bank<24;bank=bank+1)begin: capture_bank
 (* syn_keep=1, syn_preserve=1, syn_maxfan=32 *) reg capture_enable;
 (* syn_keep=1, syn_preserve=1, syn_maxfan=32 *) reg shift_enable;
 always @(posedge clk125m or negedge rst_n)
  if(!rst_n)begin capture_enable<=0;shift_enable<=0;end
  else begin
   capture_enable<=(state==IDLE)&&sample_valid;
   shift_enable<=(state==LOAD)||(state==APP_CRC&&crc_index<95)||(state==SEND&&byte_index>=49&&byte_index<145);
  end
 always @(posedge clk125m or negedge rst_n)
  if(!rst_n)data_latched[bank*32+:32]<=0;
  else if(capture_enable)data_latched[bank*32+:32]<=sample_pipe[bank*32+:32];
  else if(shift_enable)data_latched[bank*32+:32]<={data_latched[((bank+1)%24)*32+:8],data_latched[bank*32+8+:24]};
end endgenerate
reg [31:0] sequence_next, sequence_latched;
reg [31:0] app_crc_work, app_crc_latched, eth_crc;
// 正文在每个CRC或正文时钟旋转一字节，
// 96周期转完一圈，以固定抽头替代宽可变载荷选择器。
// 将载荷多路选择单独寄存，与CRC反馈路径分离。
reg [7:0] crc_payload;
reg [3:0] ifg_count;
assign sample_ready = (state == IDLE);

function [31:0] crc_byte;
    input [31:0] previous;
    input [7:0] data;
    reg [7:0] mixed;
    // 使用显式线性CRC方程，避免级联条件多路选择。
    begin
        mixed=previous[7:0]^data;
        crc_byte[0] = previous[8] ^ mixed[2];
        crc_byte[1] = previous[9] ^ mixed[0] ^ mixed[3];
        crc_byte[2] = previous[10] ^ mixed[0] ^ mixed[1] ^ mixed[4];
        crc_byte[3] = previous[11] ^ mixed[1] ^ mixed[2] ^ mixed[5];
        crc_byte[4] = (previous[12] ^ mixed[0]) ^ (mixed[2] ^ mixed[3] ^ mixed[6]);
        crc_byte[5] = (previous[13] ^ mixed[1]) ^ (mixed[3] ^ mixed[4] ^ mixed[7]);
        crc_byte[6] = previous[14] ^ mixed[4] ^ mixed[5];
        crc_byte[7] = previous[15] ^ mixed[0] ^ mixed[5] ^ mixed[6];
        crc_byte[8] = previous[16] ^ mixed[1] ^ mixed[6] ^ mixed[7];
        crc_byte[9] = previous[17] ^ mixed[7];
        crc_byte[10] = previous[18] ^ mixed[2];
        crc_byte[11] = previous[19] ^ mixed[3];
        crc_byte[12] = previous[20] ^ mixed[0] ^ mixed[4];
        crc_byte[13] = previous[21] ^ mixed[0] ^ mixed[1] ^ mixed[5];
        crc_byte[14] = previous[22] ^ mixed[1] ^ mixed[2] ^ mixed[6];
        crc_byte[15] = previous[23] ^ mixed[2] ^ mixed[3] ^ mixed[7];
        crc_byte[16] = (previous[24] ^ mixed[0]) ^ (mixed[2] ^ mixed[3] ^ mixed[4]);
        crc_byte[17] = (previous[25] ^ mixed[0] ^ mixed[1]) ^ (mixed[3] ^ mixed[4] ^ mixed[5]);
        crc_byte[18] = (previous[26] ^ mixed[0] ^ mixed[1]) ^ (mixed[2] ^ mixed[4] ^ mixed[5] ^ mixed[6]);
        crc_byte[19] = (previous[27] ^ mixed[1] ^ mixed[2]) ^ (mixed[3] ^ mixed[5] ^ mixed[6] ^ mixed[7]);
        crc_byte[20] = (previous[28] ^ mixed[3]) ^ (mixed[4] ^ mixed[6] ^ mixed[7]);
        crc_byte[21] = (previous[29] ^ mixed[2]) ^ (mixed[4] ^ mixed[5] ^ mixed[7]);
        crc_byte[22] = (previous[30] ^ mixed[2]) ^ (mixed[3] ^ mixed[5] ^ mixed[6]);
        crc_byte[23] = (previous[31] ^ mixed[3]) ^ (mixed[4] ^ mixed[6] ^ mixed[7]);
        crc_byte[24] = (mixed[0] ^ mixed[2]) ^ (mixed[4] ^ mixed[5] ^ mixed[7]);
        crc_byte[25] = (mixed[0] ^ mixed[1] ^ mixed[2]) ^ (mixed[3] ^ mixed[5] ^ mixed[6]);
        crc_byte[26] = (mixed[0] ^ mixed[1] ^ mixed[2]) ^ (mixed[3] ^ mixed[4] ^ mixed[6] ^ mixed[7]);
        crc_byte[27] = (mixed[1] ^ mixed[3]) ^ (mixed[4] ^ mixed[5] ^ mixed[7]);
        crc_byte[28] = mixed[0] ^ mixed[4] ^ mixed[5] ^ mixed[6];
        crc_byte[29] = (mixed[0] ^ mixed[1]) ^ (mixed[5] ^ mixed[6] ^ mixed[7]);
        crc_byte[30] = mixed[0] ^ mixed[1] ^ mixed[6] ^ mixed[7];
        crc_byte[31] = mixed[1] ^ mixed[7];
    end
endfunction
wire [31:0] eth_crc_next=crc_byte(eth_crc,gmii_txd);


function [7:0] payload_byte;
 input [6:0] index;
 begin
  if(index<96)payload_byte=data_latched[7:0];
  else case(index)
   96:payload_byte=app_crc_latched[7:0];97:payload_byte=app_crc_latched[15:8];
   98:payload_byte=app_crc_latched[23:16];99:payload_byte=app_crc_latched[31:24];
   default:payload_byte=0;
  endcase
 end
endfunction

function [15:0] ip_checksum;
    input [31:0] source_ip;
    input [31:0] dest_ip;
    reg [31:0] sum;
    begin
        // IPv4头长5，总长60，标识0，禁止分片，TTL为64，协议UDP17，无IP选项。
        sum=32'h4500+32'd128+32'h4000+32'h4011+
            {16'd0,source_ip[31:16]}+{16'd0,source_ip[15:0]}+
            {16'd0,dest_ip[31:16]}+{16'd0,dest_ip[15:0]};
        sum={16'd0,sum[15:0]}+{16'd0,sum[31:16]};
        sum={16'd0,sum[15:0]}+{16'd0,sum[31:16]};
        ip_checksum=~sum[15:0];
    end
endfunction
localparam [15:0] IP_SUM=ip_checksum(SRC_IP,DST_IP);
reg [7:0] frame_byte;
always @(*) begin
    case(byte_index)
        0,1,2,3,4,5,6:frame_byte=8'h55;
        7:frame_byte=8'hD5;
        8:frame_byte=DST_MAC[47:40]; 9:frame_byte=DST_MAC[39:32];
        10:frame_byte=DST_MAC[31:24]; 11:frame_byte=DST_MAC[23:16];
        12:frame_byte=DST_MAC[15:8]; 13:frame_byte=DST_MAC[7:0];
        14:frame_byte=SRC_MAC[47:40]; 15:frame_byte=SRC_MAC[39:32];
        16:frame_byte=SRC_MAC[31:24]; 17:frame_byte=SRC_MAC[23:16];
        18:frame_byte=SRC_MAC[15:8]; 19:frame_byte=SRC_MAC[7:0];
        20:frame_byte=8'h08; 21:frame_byte=8'h00;
        22:frame_byte=8'h45; 23:frame_byte=8'h00;
        24:frame_byte=8'h00; 25:frame_byte=8'h80;
        26:frame_byte=0; 27:frame_byte=0;
        28:frame_byte=8'h40; 29:frame_byte=0;
        30:frame_byte=8'h40; 31:frame_byte=8'h11;
        32:frame_byte=IP_SUM[15:8]; 33:frame_byte=IP_SUM[7:0];
        34:frame_byte=SRC_IP[31:24]; 35:frame_byte=SRC_IP[23:16];
        36:frame_byte=SRC_IP[15:8]; 37:frame_byte=SRC_IP[7:0];
        38:frame_byte=DST_IP[31:24]; 39:frame_byte=DST_IP[23:16];
        40:frame_byte=DST_IP[15:8]; 41:frame_byte=DST_IP[7:0];
        42:frame_byte=SRC_PORT[15:8]; 43:frame_byte=SRC_PORT[7:0]^{7'd0,attitude_latched};
        44:frame_byte=DST_PORT[15:8]; 45:frame_byte=DST_PORT[7:0]^{7'd0,attitude_latched};
        46:frame_byte=0; 47:frame_byte=8'h6C;
        48:frame_byte=0; 49:frame_byte=0; // IPv4允许UDP校验和为0。
        150:frame_byte=~eth_crc_next[7:0];
        151:frame_byte=~eth_crc[15:8];
        152:frame_byte=~eth_crc[23:16];
        153:frame_byte=~eth_crc[31:24];
        50:frame_byte=payload_byte(7'd0);
        51:frame_byte=payload_byte(7'd1);
        52:frame_byte=payload_byte(7'd2);
        53:frame_byte=payload_byte(7'd3);
        54:frame_byte=payload_byte(7'd4);
        55:frame_byte=payload_byte(7'd5);
        56:frame_byte=payload_byte(7'd6);
        57:frame_byte=payload_byte(7'd7);
        58:frame_byte=payload_byte(7'd8);
        59:frame_byte=payload_byte(7'd9);
        60:frame_byte=payload_byte(7'd10);
        61:frame_byte=payload_byte(7'd11);
        62:frame_byte=payload_byte(7'd12);
        63:frame_byte=payload_byte(7'd13);
        64:frame_byte=payload_byte(7'd14);
        65:frame_byte=payload_byte(7'd15);
        66:frame_byte=payload_byte(7'd16);
        67:frame_byte=payload_byte(7'd17);
        68:frame_byte=payload_byte(7'd18);
        69:frame_byte=payload_byte(7'd19);
        70:frame_byte=payload_byte(7'd20);
        71:frame_byte=payload_byte(7'd21);
        72:frame_byte=payload_byte(7'd22);
        73:frame_byte=payload_byte(7'd23);
        74:frame_byte=payload_byte(7'd24);
        75:frame_byte=payload_byte(7'd25);
        76:frame_byte=payload_byte(7'd26);
        77:frame_byte=payload_byte(7'd27);
        78:frame_byte=payload_byte(7'd28);
        79:frame_byte=payload_byte(7'd29);
        80:frame_byte=payload_byte(7'd30);
        81:frame_byte=payload_byte(7'd31);
        82:frame_byte=payload_byte(7'd32);
        83:frame_byte=payload_byte(7'd33);
        84:frame_byte=payload_byte(7'd34);
        85:frame_byte=payload_byte(7'd35);
        86:frame_byte=payload_byte(7'd36);
        87:frame_byte=payload_byte(7'd37);
        88:frame_byte=payload_byte(7'd38);
        89:frame_byte=payload_byte(7'd39);
        90:frame_byte=payload_byte(7'd40);
        91:frame_byte=payload_byte(7'd41);
        92:frame_byte=payload_byte(7'd42);
        93:frame_byte=payload_byte(7'd43);
        94:frame_byte=payload_byte(7'd44);
        95:frame_byte=payload_byte(7'd45);
        96:frame_byte=payload_byte(7'd46);
        97:frame_byte=payload_byte(7'd47);
        98:frame_byte=payload_byte(7'd48);
        99:frame_byte=payload_byte(7'd49);
        100:frame_byte=payload_byte(7'd50);
        101:frame_byte=payload_byte(7'd51);
        102:frame_byte=payload_byte(7'd52);
        103:frame_byte=payload_byte(7'd53);
        104:frame_byte=payload_byte(7'd54);
        105:frame_byte=payload_byte(7'd55);
        106:frame_byte=payload_byte(7'd56);
        107:frame_byte=payload_byte(7'd57);
        108:frame_byte=payload_byte(7'd58);
        109:frame_byte=payload_byte(7'd59);
        110:frame_byte=payload_byte(7'd60);
        111:frame_byte=payload_byte(7'd61);
        112:frame_byte=payload_byte(7'd62);
        113:frame_byte=payload_byte(7'd63);
        114:frame_byte=payload_byte(7'd64);
        115:frame_byte=payload_byte(7'd65);
        116:frame_byte=payload_byte(7'd66);
        117:frame_byte=payload_byte(7'd67);
        118:frame_byte=payload_byte(7'd68);
        119:frame_byte=payload_byte(7'd69);
        120:frame_byte=payload_byte(7'd70);
        121:frame_byte=payload_byte(7'd71);
        122:frame_byte=payload_byte(7'd72);
        123:frame_byte=payload_byte(7'd73);
        124:frame_byte=payload_byte(7'd74);
        125:frame_byte=payload_byte(7'd75);
        126:frame_byte=payload_byte(7'd76);
        127:frame_byte=payload_byte(7'd77);
        128:frame_byte=payload_byte(7'd78);
        129:frame_byte=payload_byte(7'd79);
        130:frame_byte=payload_byte(7'd80);
        131:frame_byte=payload_byte(7'd81);
        132:frame_byte=payload_byte(7'd82);
        133:frame_byte=payload_byte(7'd83);
        134:frame_byte=payload_byte(7'd84);
        135:frame_byte=payload_byte(7'd85);
        136:frame_byte=payload_byte(7'd86);
        137:frame_byte=payload_byte(7'd87);
        138:frame_byte=payload_byte(7'd88);
        139:frame_byte=payload_byte(7'd89);
        140:frame_byte=payload_byte(7'd90);
        141:frame_byte=payload_byte(7'd91);
        142:frame_byte=payload_byte(7'd92);
        143:frame_byte=payload_byte(7'd93);
        144:frame_byte=payload_byte(7'd94);
        145:frame_byte=payload_byte(7'd95);
        146:frame_byte=payload_byte(7'd96);
        147:frame_byte=payload_byte(7'd97);
        148:frame_byte=payload_byte(7'd98);
        149:frame_byte=payload_byte(7'd99);
        default:frame_byte=0; // 位于实际发送的字节范围之外。
    endcase
end

always @(posedge clk125m or negedge rst_n) begin
    if(!rst_n) begin
        eth_crc_valid_d<=0;state<=IDLE;attitude_latched<=0; gmii_txen<=0; gmii_txd<=0; tx_done<=0;
        sequence_next<=SEQ_INIT; sequence_latched<=0;
        crc_index<=0;crc_payload<=0;
        app_crc_work<=32'hFFFFFFFF; app_crc_latched<=0;
        eth_crc<=32'hFFFFFFFF; byte_index<=0; ifg_count<=0;
    end else begin
        tx_done<=0;
        eth_crc_valid_d<=(state==SEND)&&(byte_index>=8)&&(byte_index<150);
        case(state)
            IDLE:begin
                gmii_txen<=0; gmii_txd<=0;
                if(sample_valid) begin
                    attitude_latched<=attitude_frame;
                    sequence_latched<=sequence_next;
                    sequence_next<=sequence_next+1'b1;
                    app_crc_work<=32'hFFFFFFFF; crc_index<=0;crc_payload<=sample_data[7:0];
                    state<=LOAD;
                end
            end
            LOAD:begin
                // 正文存储在此完成锁存，首字节已在许可边沿寄存。
                state<=APP_CRC;
            end
            APP_CRC:begin
                app_crc_work<=crc_byte(app_crc_work,crc_payload);
                if(crc_index==95) begin
                    app_crc_latched<=~crc_byte(app_crc_work,crc_payload);
                    byte_index<=0; eth_crc<=32'hFFFFFFFF; state<=SEND;
                end else begin
                    crc_index<=crc_index+1'b1;
                    crc_payload<=data_latched[15:8];
                end
            end
            SEND:begin
                gmii_txen<=1; gmii_txd<=frame_byte;
                if(eth_crc_valid_d)
                    eth_crc<=eth_crc_next;
                if(byte_index==153) begin
                    state<=IFG; ifg_count<=0;
                end else byte_index<=byte_index+1'b1;
            end
            IFG:begin
                gmii_txen<=0; gmii_txd<=0;
                if(ifg_count==11) begin
                    state<=IDLE; tx_done<=1;
                end else ifg_count<=ifg_count+1'b1;
            end
            default:begin state<=IDLE; gmii_txen<=0; end
        endcase
    end
end
endmodule
