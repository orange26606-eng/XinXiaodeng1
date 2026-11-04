`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////

// 模块：imu_command_rx_gmii
// 功能：接收无VLAN标签、IPv4头长5、UDP封装的固定IMC1命令。
// 版本1.1.0：初始实现。
// 要求完整86字节GMII帧，其中含7个55字节与D5前导码及4字节FCS，
// UDP端口5007到5007，源192.168.0.3，目标192.168.0.2。
// 在以太网层和应用层分别独立检查IEEE CRC32。
// 输出只保留一个待取命令，槽位忙时新到的整帧丢弃。
//////////////////////////////////////////////////////////////////////////////////
module imu_command_rx_gmii (
    input wire clk_rx,
    input wire rst_n,
    input wire gmii_rxen,
    input wire gmii_rxer,
    input wire [7:0] gmii_rxd,
    output reg cmd_valid,
    input wire cmd_ready,
    output reg [1:0] cmd_op,
    output reg [31:0] cmd_token
);
    reg in_frame;
    reg [6:0] byte_count;
    reg frame_bad;
    reg frame_drop;
    reg [47:0] destination_mac;
    reg [1:0] candidate_op;
    reg [31:0] candidate_token;
    reg [31:0] ethernet_crc;
    reg [31:0] application_crc;
    reg [31:0] received_fcs;
    reg [31:0] received_application_crc;
    reg [7:0] ip_high_byte;
    reg [19:0] ip_sum;

    function [31:0] crc32_byte;
        input [31:0] prior;
        input [7:0] octet;
        reg [31:0] value;
        integer bit_number;
        begin
            value = prior ^ {24'b0, octet};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
                if (value[0]) value = (value >> 1) ^ 32'hEDB88320;
                else value = value >> 1;
            crc32_byte = value;
        end
    endfunction

    function ip_checksum_valid;
        input [19:0] total;
        reg [16:0] folded;
        begin
            folded = {1'b0, total[15:0]} + {13'b0, total[19:16]};
            folded = {1'b0, folded[15:0]} + folded[16];
            ip_checksum_valid = (folded == 17'h0FFFF);
        end
    endfunction

    always @(posedge clk_rx or negedge rst_n) begin
        if (!rst_n) begin
            in_frame <= 1'b0;
            byte_count <= 7'd0;
            frame_bad <= 1'b0;
            frame_drop <= 1'b0;
            destination_mac <= 48'b0;
            candidate_op <= 2'b0;
            candidate_token <= 32'b0;
            ethernet_crc <= 32'hFFFFFFFF;
            application_crc <= 32'hFFFFFFFF;
            received_fcs <= 32'b0;
            received_application_crc <= 32'b0;
            ip_high_byte <= 8'b0;
            ip_sum <= 20'b0;
            cmd_valid <= 1'b0;
            cmd_op <= 2'b0;
            cmd_token <= 32'b0;
        end else begin
            if (cmd_valid && cmd_ready) cmd_valid <= 1'b0;
            if (gmii_rxen) begin
                if (!in_frame) begin
                    in_frame <= 1'b1;
                    byte_count <= 7'd1;
                    frame_bad <= gmii_rxer || (gmii_rxd != 8'h55);
                    frame_drop <= cmd_valid && !cmd_ready;
                    destination_mac <= 48'b0;
                    candidate_op <= 2'b0;
                    candidate_token <= 32'b0;
                    ethernet_crc <= 32'hFFFFFFFF;
                    application_crc <= 32'hFFFFFFFF;
                    received_fcs <= 32'b0;
                    received_application_crc <= 32'b0;
                    ip_high_byte <= 8'b0;
                    ip_sum <= 20'b0;
                end else begin
                    if (gmii_rxer) frame_bad <= 1'b1;
                    if (byte_count < 7'd86) byte_count <= byte_count + 1'b1;
                    else frame_bad <= 1'b1;
                    if ((byte_count >= 7'd8) && (byte_count <= 7'd81))
                        ethernet_crc <= crc32_byte(ethernet_crc, gmii_rxd);
                    if ((byte_count >= 7'd50) && (byte_count <= 7'd77))
                        application_crc <= crc32_byte(application_crc, gmii_rxd);
                    if ((byte_count >= 7'd8) && (byte_count <= 7'd13))
                        destination_mac <= {destination_mac[39:0], gmii_rxd};
                    if ((byte_count >= 7'd22) && (byte_count <= 7'd41)) begin
                        if (!byte_count[0]) ip_high_byte <= gmii_rxd;
                        else ip_sum <= ip_sum + {4'b0, ip_high_byte, gmii_rxd};
                    end
                    case (byte_count)
                        7'd1, 7'd2, 7'd3, 7'd4, 7'd5, 7'd6:
                            if (gmii_rxd != 8'h55) frame_bad <= 1'b1;
                        7'd7: if (gmii_rxd != 8'hD5) frame_bad <= 1'b1;
                        7'd20: if (gmii_rxd != 8'h08) frame_bad <= 1'b1;
                        7'd21: if (gmii_rxd != 8'h00) frame_bad <= 1'b1;
                        7'd22: if (gmii_rxd != 8'h45) frame_bad <= 1'b1;
                        7'd24: if (gmii_rxd != 8'h00) frame_bad <= 1'b1;
                        7'd25: if (gmii_rxd != 8'h3C) frame_bad <= 1'b1;
                        7'd28: if ((gmii_rxd != 8'h00) && (gmii_rxd != 8'h40))
                            frame_bad <= 1'b1;
                        7'd29: if (gmii_rxd != 8'h00) frame_bad <= 1'b1;
                        7'd31: if (gmii_rxd != 8'h11) frame_bad <= 1'b1;
                        7'd34, 7'd38: if (gmii_rxd != 8'hC0) frame_bad <= 1'b1;
                        7'd35, 7'd39: if (gmii_rxd != 8'hA8) frame_bad <= 1'b1;
                        7'd36, 7'd40: if (gmii_rxd != 8'h00) frame_bad <= 1'b1;
                        7'd37: if (gmii_rxd != 8'h03) frame_bad <= 1'b1;
                        7'd41: if (gmii_rxd != 8'h02) frame_bad <= 1'b1;
                        7'd42, 7'd44: if (gmii_rxd != 8'h13) frame_bad <= 1'b1;
                        7'd43, 7'd45: if (gmii_rxd != 8'h8F) frame_bad <= 1'b1;
                        7'd46: if (gmii_rxd != 8'h00) frame_bad <= 1'b1;
                        7'd47: if (gmii_rxd != 8'h28) frame_bad <= 1'b1;
                        7'd48, 7'd49: if (gmii_rxd != 8'h00) frame_bad <= 1'b1;
                        7'd50: if (gmii_rxd != 8'h49) frame_bad <= 1'b1;
                        7'd51: if (gmii_rxd != 8'h4D) frame_bad <= 1'b1;
                        7'd52: if (gmii_rxd != 8'h43) frame_bad <= 1'b1;
                        7'd53: if (gmii_rxd != 8'h31) frame_bad <= 1'b1;
                        7'd54: if (gmii_rxd != 8'h01) frame_bad <= 1'b1;
                        7'd55: begin
                            candidate_op <= gmii_rxd[1:0];
                            if ((gmii_rxd != 8'h01) && (gmii_rxd != 8'h02))
                                frame_bad <= 1'b1;
                        end
                        7'd56, 7'd57, 7'd62, 7'd63, 7'd64, 7'd65,
                        7'd66, 7'd67, 7'd68, 7'd69, 7'd70, 7'd71,
                        7'd72, 7'd73, 7'd74, 7'd75, 7'd76, 7'd77:
                            if (gmii_rxd != 8'h00) frame_bad <= 1'b1;
                        7'd58: candidate_token[7:0] <= gmii_rxd;
                        7'd59: candidate_token[15:8] <= gmii_rxd;
                        7'd60: candidate_token[23:16] <= gmii_rxd;
                        7'd61: candidate_token[31:24] <= gmii_rxd;
                        7'd78: received_application_crc[7:0] <= gmii_rxd;
                        7'd79: received_application_crc[15:8] <= gmii_rxd;
                        7'd80: received_application_crc[23:16] <= gmii_rxd;
                        7'd81: received_application_crc[31:24] <= gmii_rxd;
                        7'd82: received_fcs[7:0] <= gmii_rxd;
                        7'd83: received_fcs[15:8] <= gmii_rxd;
                        7'd84: received_fcs[23:16] <= gmii_rxd;
                        7'd85: received_fcs[31:24] <= gmii_rxd;
                        default: begin end
                    endcase
                end
            end else if (in_frame) begin
                in_frame <= 1'b0;
                if ((byte_count == 7'd86) && !frame_bad && !frame_drop &&
                    ((destination_mac == 48'h000A3501FEC0) ||
                     (destination_mac == 48'hFFFFFFFFFFFF)) &&
                    ip_checksum_valid(ip_sum) &&
                    (received_application_crc == ~application_crc) &&
                    (received_fcs == ~ethernet_crc) &&
                    (!cmd_valid || cmd_ready)) begin
                    cmd_valid <= 1'b1;
                    cmd_op <= candidate_op;
                    cmd_token <= candidate_token;
                end
            end
        end
    end
endmodule
