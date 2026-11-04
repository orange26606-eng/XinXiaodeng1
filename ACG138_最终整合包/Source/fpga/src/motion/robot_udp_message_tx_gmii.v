
// 增加认证代理电机命令通路和独立500ms看门狗。
`timescale 1ns/1ps
// RBT1遥测使用UDP5008的32字节载荷，包含应用CRC32和以太网FCS。
module robot_udp_message_tx_gmii #(
    parameter [47:0] DST_MAC = 48'hFFFFFFFFFFFF,
    parameter [47:0] SRC_MAC = 48'h000A3501FEC0,
    parameter [31:0] DST_IP = 32'hC0A80003,
    parameter [31:0] SRC_IP = 32'hC0A80002,
    parameter [15:0] DST_PORT = 16'd5008,
    parameter [15:0] SRC_PORT = 16'd5008,
    parameter [31:0] SEQ_INIT = 32'd0
)(
    input clk125m, input rst_n,
    input sample_valid, output sample_ready,
    input [111:0] sample_data,
    output reg gmii_txen, output reg [7:0] gmii_txd,
    output reg tx_done
);
localparam IDLE=0, APP_CRC=1, SEND=2, IFG=3;
reg [1:0] state;
reg eth_crc_valid_d;
reg [111:0] data_latched;
reg [31:0] sequence_next, sequence_latched;
reg [31:0] app_crc_work, app_crc_latched, eth_crc;
reg [6:0] byte_index;
reg [5:0] crc_index;
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
 input [5:0] index;
 begin
  case(index)
   0:payload_byte="R";1:payload_byte="B";2:payload_byte="T";3:payload_byte="1";
   4:payload_byte=1;
   5:payload_byte={5'b0,data_latched[50:48]};
   6:payload_byte={5'b0,data_latched[53:51]};
   7:payload_byte={5'b0,data_latched[56:54]};
   8:payload_byte={5'b0,data_latched[59:57]};
   9:payload_byte={7'b0,data_latched[60]};
   10:payload_byte={4'b0,data_latched[64:61]};
   11:payload_byte={(data_latched[87:80]==8'hA3 ? data_latched[95:92]:4'b0),data_latched[68:65]};
   12:payload_byte={4'b0,data_latched[72:69]};
   13:payload_byte={4'b0,data_latched[76:73]};
   14:payload_byte=data_latched[7:0];15:payload_byte=data_latched[15:8];
   16:payload_byte=data_latched[23:16];17:payload_byte=data_latched[31:24];
   18:payload_byte=data_latched[39:32];19:payload_byte=data_latched[47:40];
   20:payload_byte=sequence_latched[7:0];21:payload_byte=sequence_latched[15:8];
   22:payload_byte=sequence_latched[23:16];23:payload_byte=sequence_latched[31:24];
   24:payload_byte=data_latched[87:80];25:payload_byte={1'b0,data_latched[79:77],data_latched[91:88]};
   26:payload_byte=data_latched[103:96];27:payload_byte=data_latched[111:104];
   28:payload_byte=app_crc_latched[7:0];29:payload_byte=app_crc_latched[15:8];
   30:payload_byte=app_crc_latched[23:16];31:payload_byte=app_crc_latched[31:24];
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
        sum=32'h4500+32'd60+32'h4000+32'h4011+
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
        24:frame_byte=8'h00; 25:frame_byte=8'h3C;
        26:frame_byte=0; 27:frame_byte=0;
        28:frame_byte=8'h40; 29:frame_byte=0;
        30:frame_byte=8'h40; 31:frame_byte=8'h11;
        32:frame_byte=IP_SUM[15:8]; 33:frame_byte=IP_SUM[7:0];
        34:frame_byte=SRC_IP[31:24]; 35:frame_byte=SRC_IP[23:16];
        36:frame_byte=SRC_IP[15:8]; 37:frame_byte=SRC_IP[7:0];
        38:frame_byte=DST_IP[31:24]; 39:frame_byte=DST_IP[23:16];
        40:frame_byte=DST_IP[15:8]; 41:frame_byte=DST_IP[7:0];
        42:frame_byte=SRC_PORT[15:8]; 43:frame_byte=SRC_PORT[7:0];
        44:frame_byte=DST_PORT[15:8]; 45:frame_byte=DST_PORT[7:0];
        46:frame_byte=0; 47:frame_byte=8'h28;
        48:frame_byte=0; 49:frame_byte=0; // IPv4允许UDP校验和为0。
        82:frame_byte=~eth_crc_next[7:0];
        83:frame_byte=~eth_crc[15:8];
        84:frame_byte=~eth_crc[23:16];
        85:frame_byte=~eth_crc[31:24];
        50:frame_byte=payload_byte(6'd0);
        51:frame_byte=payload_byte(6'd1);
        52:frame_byte=payload_byte(6'd2);
        53:frame_byte=payload_byte(6'd3);
        54:frame_byte=payload_byte(6'd4);
        55:frame_byte=payload_byte(6'd5);
        56:frame_byte=payload_byte(6'd6);
        57:frame_byte=payload_byte(6'd7);
        58:frame_byte=payload_byte(6'd8);
        59:frame_byte=payload_byte(6'd9);
        60:frame_byte=payload_byte(6'd10);
        61:frame_byte=payload_byte(6'd11);
        62:frame_byte=payload_byte(6'd12);
        63:frame_byte=payload_byte(6'd13);
        64:frame_byte=payload_byte(6'd14);
        65:frame_byte=payload_byte(6'd15);
        66:frame_byte=payload_byte(6'd16);
        67:frame_byte=payload_byte(6'd17);
        68:frame_byte=payload_byte(6'd18);
        69:frame_byte=payload_byte(6'd19);
        70:frame_byte=payload_byte(6'd20);
        71:frame_byte=payload_byte(6'd21);
        72:frame_byte=payload_byte(6'd22);
        73:frame_byte=payload_byte(6'd23);
        74:frame_byte=payload_byte(6'd24);
        75:frame_byte=payload_byte(6'd25);
        76:frame_byte=payload_byte(6'd26);
        77:frame_byte=payload_byte(6'd27);
        78:frame_byte=payload_byte(6'd28);
        79:frame_byte=payload_byte(6'd29);
        80:frame_byte=payload_byte(6'd30);
        81:frame_byte=payload_byte(6'd31);
        default:frame_byte=0; // 位于实际发送的字节范围之外。
    endcase
end

// CRC计算或发送前，载荷存储已完整覆盖。
always @(posedge clk125m)
    if(state==IDLE&&sample_valid)data_latched<=sample_data;

always @(posedge clk125m or negedge rst_n) begin
    if(!rst_n) begin
        eth_crc_valid_d<=0;state<=IDLE; gmii_txen<=0; gmii_txd<=0; tx_done<=0;
        sequence_next<=SEQ_INIT; sequence_latched<=0;
        crc_index<=0;crc_payload<=0;
        app_crc_work<=32'hFFFFFFFF; app_crc_latched<=0;
        eth_crc<=32'hFFFFFFFF; byte_index<=0; ifg_count<=0;
    end else begin
        tx_done<=0;
        eth_crc_valid_d<=(state==SEND)&&(byte_index>=8)&&(byte_index<82);
        case(state)
            IDLE:begin
                gmii_txen<=0; gmii_txd<=0;
                if(sample_valid) begin
                    sequence_latched<=sequence_next;
                    sequence_next<=sequence_next+1'b1;

                    app_crc_work<=32'hFFFFFFFF; crc_index<=0;crc_payload<=8'h52;
                    state<=APP_CRC;
                end
            end
            APP_CRC:begin
                app_crc_work<=crc_byte(app_crc_work,crc_payload);
                if(crc_index==27) begin
                    app_crc_latched<=~crc_byte(app_crc_work,crc_payload);
                    byte_index<=0; eth_crc<=32'hFFFFFFFF; state<=SEND;
                end else begin crc_index<=crc_index+1'b1;crc_payload<=payload_byte(crc_index+1'b1);end
            end
            SEND:begin
                gmii_txen<=1; gmii_txd<=frame_byte;
                if(eth_crc_valid_d)
                    eth_crc<=eth_crc_next;
                if(byte_index==85) begin
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
