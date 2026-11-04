
`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////



// 模块：phone_command_rx_gmii
// 功能：接收无VLAN标签、IPv4头长5、UDP封装的固定MCV1命令。

// 要求完整86字节GMII帧，其中含7个55字节与D5前导码及4字节FCS，
// UDP端口5009到5009，源192.168.0.3，目标192.168.0.2。
// 在以太网层和应用层分别独立检查IEEE CRC32。
// 输出只保留一个待取命令，槽位忙时新到的整帧丢弃。
//////////////////////////////////////////////////////////////////////////////////
module phone_command_rx_gmii (
    input wire clk_rx,
    input wire rst_n,
    input wire gmii_rxen,
    input wire gmii_rxer,
    input wire [7:0] gmii_rxd,
    output reg cmd_valid,
    input wire cmd_ready,
    output reg [7:0] cmd_op,
    output reg [7:0] cmd_motion,
    output reg [31:0] cmd_sequence,
    output reg [31:0] cmd_token
);
    reg in_frame;
    reg [6:0] byte_count;
    reg frame_bad;
    reg frame_drop;
    reg [47:0] destination_mac;
    reg [7:0] candidate_op, candidate_motion;
    reg [31:0] candidate_sequence;
    reg [31:0] candidate_token;
    reg [31:0] ethernet_crc;
    reg [31:0] application_crc;
    reg [31:0] received_fcs;
    reg [31:0] received_application_crc;
    reg [7:0] ip_high_byte;
    reg [19:0] ip_sum;
    // 利用载荷接收空隙，在整帧结束前完成校验和折叠。
    // 将两个串联加法器移出125MHz命令接受路径。
    reg [16:0] ip_fold_first;
    reg ip_fold_good;
    reg ip_good;
    reg mac_good,app_good,op_good,complete_pending,complete_good;

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



    // 提前寄存下一字节规则，输入只需经过
    // 8位掩码比较即可更新粘滞错误寄存器。
    function [15:0] header_rule;
        input [6:0] index;
        begin
            case(index)
                0,1,2,3,4,5:header_rule=16'hff55;
                6:header_rule=16'hffd5;
                19:header_rule=16'hff08;
                20,23,28,35,39,45,47,48,56,65,66,67,68,69,70,71,72,73,74,75,76:header_rule=16'hff00;
                21:header_rule=16'hff45;
                24:header_rule=16'hff3c;
                27:header_rule=16'hbf00;
                30:header_rule=16'hff11;
                33,37:header_rule=16'hffc0;
                34,38:header_rule=16'hffa8;
                36:header_rule=16'hff03;
                40:header_rule=16'hff02;
                41,43:header_rule=16'hff13;
                42,44:header_rule=16'hff91;
                46:header_rule=16'hff28;
                49:header_rule=16'hff4d;
                50:header_rule=16'hff43;
                51:header_rule=16'hff56;
                52:header_rule=16'hff31;
                53:header_rule=16'hff01;
                default:header_rule=0;
            endcase
        end
    endfunction
    reg [15:0] header_rule_q;
    reg check_operation,check_motion;
    reg [9:0] field_capture;
    always @(posedge clk_rx or negedge rst_n)
        if(!rst_n)field_capture<=0;
        else begin
            field_capture[0]<=in_frame&&byte_count==54;
            field_capture[1]<=in_frame&&byte_count==55;
            field_capture[2]<=in_frame&&byte_count==57;
            field_capture[3]<=in_frame&&byte_count==58;
            field_capture[4]<=in_frame&&byte_count==59;
            field_capture[5]<=in_frame&&byte_count==60;
            field_capture[6]<=in_frame&&byte_count==61;
            field_capture[7]<=in_frame&&byte_count==62;
            field_capture[8]<=in_frame&&byte_count==63;
            field_capture[9]<=in_frame&&byte_count==64;
        end
    always @(posedge clk_rx)begin
        if(gmii_rxen&&field_capture[0])candidate_op<=gmii_rxd;
        if(gmii_rxen&&field_capture[1])candidate_motion<=gmii_rxd;
        if(gmii_rxen&&field_capture[2])candidate_token[7:0]<=gmii_rxd;
        if(gmii_rxen&&field_capture[3])candidate_token[15:8]<=gmii_rxd;
        if(gmii_rxen&&field_capture[4])candidate_token[23:16]<=gmii_rxd;
        if(gmii_rxen&&field_capture[5])candidate_token[31:24]<=gmii_rxd;
        if(gmii_rxen&&field_capture[6])candidate_sequence[7:0]<=gmii_rxd;
        if(gmii_rxen&&field_capture[7])candidate_sequence[15:8]<=gmii_rxd;
        if(gmii_rxen&&field_capture[8])candidate_sequence[23:16]<=gmii_rxd;
        if(gmii_rxen&&field_capture[9])candidate_sequence[31:24]<=gmii_rxd;
    end
    wire [6:0] rule_index=in_frame?byte_count:7'd0;
    always @(posedge clk_rx or negedge rst_n)
        if(!rst_n)begin header_rule_q<=16'hff55;check_operation<=0;check_motion<=0;end
        else begin
            header_rule_q<=header_rule(rule_index);
            check_operation<=rule_index==54;
            check_motion<=rule_index==55;
        end
    wire header_bad=((gmii_rxd&header_rule_q[15:8])!=header_rule_q[7:0]) ||
                    (check_operation&&gmii_rxd>4)||(check_motion&&gmii_rxd>8);

    always @(posedge clk_rx or negedge rst_n) begin
        if (!rst_n) begin
            in_frame <= 1'b0;
            byte_count <= 7'd0;
            frame_bad <= 1'b0;
            frame_drop <= 1'b0;
            destination_mac <= 48'b0;
            
            ethernet_crc <= 32'hFFFFFFFF;
            application_crc <= 32'hFFFFFFFF;
            received_fcs <= 32'b0;
            received_application_crc <= 32'b0;
            ip_high_byte <= 8'b0;
            ip_sum <= 20'b0;
            ip_fold_first<=0;ip_fold_good<=0;ip_good<=0;
            mac_good<=0;app_good<=0;op_good<=0;complete_pending<=0;complete_good<=0;
            cmd_valid <= 1'b0;
            cmd_op <= 0; cmd_motion <= 0; cmd_sequence <= 0;
            cmd_token <= 32'b0;
        end else begin
            if (cmd_valid && cmd_ready) cmd_valid <= 1'b0;
            complete_pending<=0;
            if(complete_pending && complete_good && (!cmd_valid || cmd_ready))begin
                cmd_valid<=1;cmd_op<=candidate_op;cmd_motion<=candidate_motion;
                cmd_sequence<=candidate_sequence;cmd_token<=candidate_token;
            end
            if (gmii_rxen) begin
                if (!in_frame) begin
                    in_frame <= 1'b1;
                    byte_count <= 7'd1;
                    frame_bad <= gmii_rxer || (gmii_rxd != 8'h55);
                    frame_drop <= cmd_valid && !cmd_ready;
                    ethernet_crc <= 32'hFFFFFFFF;
                    application_crc <= 32'hFFFFFFFF;
                    ip_sum <= 20'b0;
                    ip_fold_first<=0;ip_fold_good<=0;ip_good<=0;
                    mac_good<=0;app_good<=0;op_good<=0;
                end else begin
                    if (gmii_rxer || header_bad) frame_bad <= 1'b1;
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
                    if(byte_count==42)ip_fold_first<={1'b0,ip_sum[15:0]}+{13'b0,ip_sum[19:16]};
                    if(byte_count==43)ip_fold_good<=ip_fold_first==17'h0ffff || ip_fold_first==17'h1fffe;
                    if(byte_count==44)ip_good<=ip_fold_good;
                    if(byte_count==14)mac_good<=destination_mac==48'h000A3501FEC0 || destination_mac==48'hFFFFFFFFFFFF;
                    if(byte_count==57)op_good<=(candidate_op==3 && candidate_motion>=1 && candidate_motion<=8) ||
                                               (candidate_op!=3 && candidate_motion==0);
                    if(byte_count==82)app_good<=received_application_crc==~application_crc;
                end
            end else if (in_frame) begin
                in_frame <= 1'b0;
                complete_pending<=1;
                complete_good<=(byte_count==86) && !frame_bad && !frame_drop && mac_good &&
                               ip_good && app_good && op_good && received_fcs==~ethernet_crc;
            end
        end
    end
endmodule
