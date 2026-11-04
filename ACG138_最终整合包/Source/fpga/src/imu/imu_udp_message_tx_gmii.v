`timescale 1ns/1ps


// 日期：2026-10-11 16:19
// 模块：imu_udp_message_tx_gmii
// 功能: 冻结姿态快照，发送带应用CRC32的IPv4 UDP以太网帧，保留12字节帧间隔。
// 版本：1.2.0
// 修订记录 ： v1.1.0----冻结姿态快照，发送带应用CRC32的IPv4 UDP以太网帧，保留12字节帧间隔。
//                   v1.2.0----增加IMA1校准回报、token和端口5007，与IMU1姿态共享整帧发送器。

module imu_udp_message_tx_gmii #(
    parameter [47:0] DST_MAC = 48'hFFFFFFFFFFFF,
    parameter [47:0] SRC_MAC = 48'h000A3501FEC0,
    parameter [31:0] DST_IP = 32'hC0A80003,
    parameter [31:0] SRC_IP = 32'hC0A80002,
    parameter [15:0] DST_PORT = 16'd5006,
    parameter [15:0] SRC_PORT = 16'd5006,
    parameter [31:0] SEQ_INIT = 32'd0
)(
    input clk125m, input rst_n,
    input sample_valid, output sample_ready,
    input [2:0] sample_status, input [95:0] sample_angles,
    input message_ack, input [2:0] message_error, input [31:0] message_token,
    output reg gmii_txen, output reg [7:0] gmii_txd,
    output reg tx_done
);
localparam IDLE=0, APP_CRC=1, SEND=2, IFG=3;
reg [1:0] state;
reg [2:0] status_latched, error_latched;
reg ack_latched;
reg [15:0] dst_port_latched, src_port_latched;
reg [95:0] angles_latched;
reg [31:0] sequence_next, sequence_latched;
reg [31:0] app_crc_work, app_crc_latched, eth_crc;
reg [6:0] byte_index;
reg [5:0] crc_index;
reg [3:0] ifg_count;
assign sample_ready = (state == IDLE);

function [31:0] crc_byte;
    input [31:0] previous;
    input [7:0] data;
    // 使用显式线性CRC方程，避免级联条件多路选择。
    begin
        crc_byte[0] = previous[2] ^ previous[8] ^ data[2];
        crc_byte[1] = previous[0] ^ previous[3] ^ previous[9] ^ data[0] ^ data[3];
        crc_byte[2] = previous[0] ^ previous[1] ^ previous[4] ^ previous[10] ^ data[0] ^ data[1] ^ data[4];
        crc_byte[3] = previous[1] ^ previous[2] ^ previous[5] ^ previous[11] ^ data[1] ^ data[2] ^ data[5];
        crc_byte[4] = previous[0] ^ previous[2] ^ previous[3] ^ previous[6] ^ previous[12] ^ data[0] ^ data[2] ^ data[3] ^ data[6];
        crc_byte[5] = previous[1] ^ previous[3] ^ previous[4] ^ previous[7] ^ previous[13] ^ data[1] ^ data[3] ^ data[4] ^ data[7];
        crc_byte[6] = previous[4] ^ previous[5] ^ previous[14] ^ data[4] ^ data[5];
        crc_byte[7] = previous[0] ^ previous[5] ^ previous[6] ^ previous[15] ^ data[0] ^ data[5] ^ data[6];
        crc_byte[8] = previous[1] ^ previous[6] ^ previous[7] ^ previous[16] ^ data[1] ^ data[6] ^ data[7];
        crc_byte[9] = previous[7] ^ previous[17] ^ data[7];
        crc_byte[10] = previous[2] ^ previous[18] ^ data[2];
        crc_byte[11] = previous[3] ^ previous[19] ^ data[3];
        crc_byte[12] = previous[0] ^ previous[4] ^ previous[20] ^ data[0] ^ data[4];
        crc_byte[13] = previous[0] ^ previous[1] ^ previous[5] ^ previous[21] ^ data[0] ^ data[1] ^ data[5];
        crc_byte[14] = previous[1] ^ previous[2] ^ previous[6] ^ previous[22] ^ data[1] ^ data[2] ^ data[6];
        crc_byte[15] = previous[2] ^ previous[3] ^ previous[7] ^ previous[23] ^ data[2] ^ data[3] ^ data[7];
        crc_byte[16] = previous[0] ^ previous[2] ^ previous[3] ^ previous[4] ^ previous[24] ^ data[0] ^ data[2] ^ data[3] ^ data[4];
        crc_byte[17] = previous[0] ^ previous[1] ^ previous[3] ^ previous[4] ^ previous[5] ^ previous[25] ^ data[0] ^ data[1] ^ data[3] ^ data[4] ^ data[5];
        crc_byte[18] = previous[0] ^ previous[1] ^ previous[2] ^ previous[4] ^ previous[5] ^ previous[6] ^ previous[26] ^ data[0] ^ data[1] ^ data[2] ^ data[4] ^ data[5] ^ data[6];
        crc_byte[19] = previous[1] ^ previous[2] ^ previous[3] ^ previous[5] ^ previous[6] ^ previous[7] ^ previous[27] ^ data[1] ^ data[2] ^ data[3] ^ data[5] ^ data[6] ^ data[7];
        crc_byte[20] = previous[3] ^ previous[4] ^ previous[6] ^ previous[7] ^ previous[28] ^ data[3] ^ data[4] ^ data[6] ^ data[7];
        crc_byte[21] = previous[2] ^ previous[4] ^ previous[5] ^ previous[7] ^ previous[29] ^ data[2] ^ data[4] ^ data[5] ^ data[7];
        crc_byte[22] = previous[2] ^ previous[3] ^ previous[5] ^ previous[6] ^ previous[30] ^ data[2] ^ data[3] ^ data[5] ^ data[6];
        crc_byte[23] = previous[3] ^ previous[4] ^ previous[6] ^ previous[7] ^ previous[31] ^ data[3] ^ data[4] ^ data[6] ^ data[7];
        crc_byte[24] = previous[0] ^ previous[2] ^ previous[4] ^ previous[5] ^ previous[7] ^ data[0] ^ data[2] ^ data[4] ^ data[5] ^ data[7];
        crc_byte[25] = previous[0] ^ previous[1] ^ previous[2] ^ previous[3] ^ previous[5] ^ previous[6] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[5] ^ data[6];
        crc_byte[26] = previous[0] ^ previous[1] ^ previous[2] ^ previous[3] ^ previous[4] ^ previous[6] ^ previous[7] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[6] ^ data[7];
        crc_byte[27] = previous[1] ^ previous[3] ^ previous[4] ^ previous[5] ^ previous[7] ^ data[1] ^ data[3] ^ data[4] ^ data[5] ^ data[7];
        crc_byte[28] = previous[0] ^ previous[4] ^ previous[5] ^ previous[6] ^ data[0] ^ data[4] ^ data[5] ^ data[6];
        crc_byte[29] = previous[0] ^ previous[1] ^ previous[5] ^ previous[6] ^ previous[7] ^ data[0] ^ data[1] ^ data[5] ^ data[6] ^ data[7];
        crc_byte[30] = previous[0] ^ previous[1] ^ previous[6] ^ previous[7] ^ data[0] ^ data[1] ^ data[6] ^ data[7];
        crc_byte[31] = previous[1] ^ previous[7] ^ data[1] ^ data[7];
    end
endfunction

function [7:0] payload_byte;
    input [5:0] index;
    begin
        case(index)
            0:payload_byte=8'h49; 1:payload_byte=8'h4D;
            2:payload_byte=ack_latched ? 8'h41 : 8'h55; 3:payload_byte=8'h31;
            4:payload_byte=8'h01; 5:payload_byte={5'd0,status_latched};
            6:payload_byte=ack_latched ? {5'd0,error_latched} : 8'd0;
            8:payload_byte=sequence_latched[7:0];
            9:payload_byte=sequence_latched[15:8];
            10:payload_byte=sequence_latched[23:16];
            11:payload_byte=sequence_latched[31:24];
            28:payload_byte=app_crc_latched[7:0];
            29:payload_byte=app_crc_latched[15:8];
            30:payload_byte=app_crc_latched[23:16];
            31:payload_byte=app_crc_latched[31:24];
            12:payload_byte=ack_latched ? ((status_latched==1 || status_latched==2)?8'd9:8'd0) : angles_latched[7:0];
            13:payload_byte=ack_latched ? ((status_latched==1 || status_latched==2)?8'd50:8'd0) : angles_latched[15:8];
            14:payload_byte=ack_latched ? 8'd0 : angles_latched[23:16];
            15:payload_byte=ack_latched ? 8'd0 : angles_latched[31:24];
            16:payload_byte=ack_latched ? 8'd0 : angles_latched[39:32];
            17:payload_byte=ack_latched ? 8'd0 : angles_latched[47:40];
            18:payload_byte=ack_latched ? 8'd0 : angles_latched[55:48];
            19:payload_byte=ack_latched ? 8'd0 : angles_latched[63:56];
            20:payload_byte=ack_latched ? 8'd0 : angles_latched[71:64];
            21:payload_byte=ack_latched ? 8'd0 : angles_latched[79:72];
            22:payload_byte=ack_latched ? 8'd0 : angles_latched[87:80];
            23:payload_byte=ack_latched ? 8'd0 : angles_latched[95:88];
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
        42:frame_byte=src_port_latched[15:8]; 43:frame_byte=src_port_latched[7:0];
        44:frame_byte=dst_port_latched[15:8]; 45:frame_byte=dst_port_latched[7:0];
        46:frame_byte=0; 47:frame_byte=8'h28;
        48:frame_byte=0; 49:frame_byte=0; // IPv4允许UDP校验和为0。
        82:frame_byte=~eth_crc[7:0];
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

always @(posedge clk125m or negedge rst_n) begin
    if(!rst_n) begin
        state<=IDLE; gmii_txen<=0; gmii_txd<=0; tx_done<=0;
        sequence_next<=SEQ_INIT; sequence_latched<=0;
        status_latched<=0; angles_latched<=0; crc_index<=0;
        ack_latched<=0; error_latched<=0;
        dst_port_latched<=DST_PORT; src_port_latched<=SRC_PORT;
        app_crc_work<=32'hFFFFFFFF; app_crc_latched<=0;
        eth_crc<=32'hFFFFFFFF; byte_index<=0; ifg_count<=0;
    end else begin
        tx_done<=0;
        case(state)
            IDLE:begin
                gmii_txen<=0; gmii_txd<=0;
                if(sample_valid) begin
                    sequence_latched<=message_ack ? message_token : sequence_next;
                    if(!message_ack) sequence_next<=sequence_next+1'b1;
                    ack_latched<=message_ack; error_latched<=message_error;
                    dst_port_latched<=message_ack ? 16'd5007 : DST_PORT;
                    src_port_latched<=message_ack ? 16'd5007 : SRC_PORT;
                    status_latched<=sample_status;
                    angles_latched<=sample_status==0 ? sample_angles : 96'd0;
                    app_crc_work<=32'hFFFFFFFF; crc_index<=0;
                    state<=APP_CRC;
                end
            end
            APP_CRC:begin
                app_crc_work<=crc_byte(app_crc_work,payload_byte(crc_index));
                if(crc_index==27) begin
                    app_crc_latched<=~crc_byte(app_crc_work,payload_byte(crc_index));
                    byte_index<=0; eth_crc<=32'hFFFFFFFF; state<=SEND;
                end else crc_index<=crc_index+1'b1;
            end
            SEND:begin
                gmii_txen<=1; gmii_txd<=frame_byte;
                if(byte_index>=8 && byte_index<82)
                    eth_crc<=crc_byte(eth_crc,frame_byte);
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
