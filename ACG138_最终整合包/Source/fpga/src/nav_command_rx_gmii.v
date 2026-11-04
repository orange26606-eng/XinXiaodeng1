`timescale 1ns/1ps
// NCV1 命令接收器，监听 UDP5012。
// 沿用motor_network_tuner_20261025的net_command_rx.v。
// 固定64字节载荷，CRC和FCS正确后由命令保护模块检查NCV1语义。
// 固定长度帧校验后，按32位分组提交完整命令。
module nav_command_rx_gmii(input clk,input rst_n,input en,input er,input [7:0] data,
 output reg valid,input ready,output reg [479:0] command);
reg active,bad,drop;
reg [7:0] index;
reg [47:0] mac;
reg [479:0] candidate;
reg [31:0] eth_crc,app_crc,app_received;
reg [3:0] fcs_good;
reg app_good;
reg [7:0] data_pipe;
reg [19:0] ip_sum;reg [7:0] ip_hi;reg [16:0] fold1;reg fold_good;
reg ip_ok,mac_ok;
wire accepted_end=!en&&active&&index==118&&!bad&&!drop&&ip_ok&&mac_ok&&
                  (&fcs_good)&&app_good&&(!valid||ready);
wire [14:0] commit_enable;
// 固定60字节正文经局部使能的短存储组移入。
// 完整载荷接受前已覆盖全部暂存字节。
always @(posedge clk) data_pipe<=data;
genvar bank;
generate for(bank=0;bank<15;bank=bank+1)begin:capture_bank
 (* syn_keep=1,syn_preserve=1,syn_maxfan=32 *) reg capture;
 (* syn_keep=1,syn_preserve=1,syn_maxfan=32 *) reg commit;
 assign commit_enable[bank]=commit;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin capture<=0;commit<=0;end
  else begin capture<=en&&active&&index>=50&&index<=109;commit<=accepted_end;end
 if(bank==14)begin:last
  always @(posedge clk)
   if(capture)candidate[bank*32+:32]<={data_pipe,candidate[bank*32+8+:24]};
 end else begin:body
  always @(posedge clk)
   if(capture)candidate[bank*32+:32]<={candidate[(bank+1)*32+:8],candidate[bank*32+8+:24]};
 end
 always @(posedge clk or negedge rst_n)
  if(!rst_n)command[bank*32+:32]<=0;
  else if(commit)command[bank*32+:32]<=candidate[bank*32+:32];
end endgenerate
function [31:0] crc32;
    input [31:0] previous;
    input [7:0] data;
    // 使用显式线性CRC方程，避免级联条件多路选择。
    begin
        crc32[0] = previous[2] ^ previous[8] ^ data[2];
        crc32[1] = previous[0] ^ previous[3] ^ previous[9] ^ data[0] ^ data[3];
        crc32[2] = previous[0] ^ previous[1] ^ previous[4] ^ previous[10] ^ data[0] ^ data[1] ^ data[4];
        crc32[3] = previous[1] ^ previous[2] ^ previous[5] ^ previous[11] ^ data[1] ^ data[2] ^ data[5];
        crc32[4] = previous[0] ^ previous[2] ^ previous[3] ^ previous[6] ^ previous[12] ^ data[0] ^ data[2] ^ data[3] ^ data[6];
        crc32[5] = previous[1] ^ previous[3] ^ previous[4] ^ previous[7] ^ previous[13] ^ data[1] ^ data[3] ^ data[4] ^ data[7];
        crc32[6] = previous[4] ^ previous[5] ^ previous[14] ^ data[4] ^ data[5];
        crc32[7] = previous[0] ^ previous[5] ^ previous[6] ^ previous[15] ^ data[0] ^ data[5] ^ data[6];
        crc32[8] = previous[1] ^ previous[6] ^ previous[7] ^ previous[16] ^ data[1] ^ data[6] ^ data[7];
        crc32[9] = previous[7] ^ previous[17] ^ data[7];
        crc32[10] = previous[2] ^ previous[18] ^ data[2];
        crc32[11] = previous[3] ^ previous[19] ^ data[3];
        crc32[12] = previous[0] ^ previous[4] ^ previous[20] ^ data[0] ^ data[4];
        crc32[13] = previous[0] ^ previous[1] ^ previous[5] ^ previous[21] ^ data[0] ^ data[1] ^ data[5];
        crc32[14] = previous[1] ^ previous[2] ^ previous[6] ^ previous[22] ^ data[1] ^ data[2] ^ data[6];
        crc32[15] = previous[2] ^ previous[3] ^ previous[7] ^ previous[23] ^ data[2] ^ data[3] ^ data[7];
        crc32[16] = previous[0] ^ previous[2] ^ previous[3] ^ previous[4] ^ previous[24] ^ data[0] ^ data[2] ^ data[3] ^ data[4];
        crc32[17] = previous[0] ^ previous[1] ^ previous[3] ^ previous[4] ^ previous[5] ^ previous[25] ^ data[0] ^ data[1] ^ data[3] ^ data[4] ^ data[5];
        crc32[18] = previous[0] ^ previous[1] ^ previous[2] ^ previous[4] ^ previous[5] ^ previous[6] ^ previous[26] ^ data[0] ^ data[1] ^ data[2] ^ data[4] ^ data[5] ^ data[6];
        crc32[19] = previous[1] ^ previous[2] ^ previous[3] ^ previous[5] ^ previous[6] ^ previous[7] ^ previous[27] ^ data[1] ^ data[2] ^ data[3] ^ data[5] ^ data[6] ^ data[7];
        crc32[20] = previous[3] ^ previous[4] ^ previous[6] ^ previous[7] ^ previous[28] ^ data[3] ^ data[4] ^ data[6] ^ data[7];
        crc32[21] = previous[2] ^ previous[4] ^ previous[5] ^ previous[7] ^ previous[29] ^ data[2] ^ data[4] ^ data[5] ^ data[7];
        crc32[22] = previous[2] ^ previous[3] ^ previous[5] ^ previous[6] ^ previous[30] ^ data[2] ^ data[3] ^ data[5] ^ data[6];
        crc32[23] = previous[3] ^ previous[4] ^ previous[6] ^ previous[7] ^ previous[31] ^ data[3] ^ data[4] ^ data[6] ^ data[7];
        crc32[24] = previous[0] ^ previous[2] ^ previous[4] ^ previous[5] ^ previous[7] ^ data[0] ^ data[2] ^ data[4] ^ data[5] ^ data[7];
        crc32[25] = previous[0] ^ previous[1] ^ previous[2] ^ previous[3] ^ previous[5] ^ previous[6] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[5] ^ data[6];
        crc32[26] = previous[0] ^ previous[1] ^ previous[2] ^ previous[3] ^ previous[4] ^ previous[6] ^ previous[7] ^ data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[6] ^ data[7];
        crc32[27] = previous[1] ^ previous[3] ^ previous[4] ^ previous[5] ^ previous[7] ^ data[1] ^ data[3] ^ data[4] ^ data[5] ^ data[7];
        crc32[28] = previous[0] ^ previous[4] ^ previous[5] ^ previous[6] ^ data[0] ^ data[4] ^ data[5] ^ data[6];
        crc32[29] = previous[0] ^ previous[1] ^ previous[5] ^ previous[6] ^ previous[7] ^ data[0] ^ data[1] ^ data[5] ^ data[6] ^ data[7];
        crc32[30] = previous[0] ^ previous[1] ^ previous[6] ^ previous[7] ^ data[0] ^ data[1] ^ data[6] ^ data[7];
        crc32[31] = previous[1] ^ previous[7] ^ data[1] ^ data[7];
    end
endfunction
    function [15:0] header_rule;
        input [6:0] index;
        begin
            case(index)
                0,1,2,3,4,5:header_rule=16'hff55;
                6:header_rule=16'hffd5;
                19:header_rule=16'hff08;
                20,23,28,35,39,45,47,48:header_rule=16'hff00;
                21:header_rule=16'hff45;
                24:header_rule=16'hff5c;
                27:header_rule=16'hbf00;
                30:header_rule=16'hff11;
                33,37:header_rule=16'hffc0;
                34,38:header_rule=16'hffa8;
                36:header_rule=16'hff03;
                40:header_rule=16'hff02;
                41,43:header_rule=16'hff13;
                42,44:header_rule=16'hff94;
                46:header_rule=16'hff48;
                default:header_rule=0;
            endcase
        end
    endfunction
    reg [15:0] header_rule_q;
    wire [6:0] rule_index=active?index[6:0]:7'd0;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)header_rule_q<=16'hff55;
        else header_rule_q<=header_rule(rule_index);
    wire header_bad=(data&header_rule_q[15:8])!=header_rule_q[7:0];

always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin active<=0;bad<=0;drop<=0;index<=0;mac<=0;
 eth_crc<=32'hffffffff;app_crc<=32'hffffffff;fcs_good<=0;app_good<=0;app_received<=0;
 ip_sum<=0;ip_hi<=0;fold1<=0;fold_good<=0;ip_ok<=0;mac_ok<=0;valid<=0;end
 else begin
  if(valid&&ready)valid<=0;
  if(commit_enable[0])valid<=1;
  if(en)begin
   if(!active)begin active<=1;index<=1;bad<=er||data!=8'h55;drop<=(valid||commit_enable[0])&&!ready;
    eth_crc<=32'hffffffff;app_crc<=32'hffffffff;
    fcs_good<=0;app_good<=0;app_received<=0;ip_sum<=0;ip_ok<=0;mac_ok<=0;
   end else begin
    if(er)bad<=1;
    if(index<118)index<=index+1'b1;else bad<=1;
    if(index>=8&&index<=113)eth_crc<=crc32(eth_crc,data);
    if(index>=50&&index<=109)app_crc<=crc32(app_crc,data);
    if(index>=110&&index<=113)app_received[(index-110)*8+:8]<=data;
    case(index)
     114:begin app_good<=app_received==~app_crc;fcs_good[0]<=data==~eth_crc[7:0];end
     115:fcs_good[1]<=data==~eth_crc[15:8];
     116:fcs_good[2]<=data==~eth_crc[23:16];
     117:fcs_good[3]<=data==~eth_crc[31:24];
     default:begin end
    endcase
    if(index>=8&&index<=13)mac<={mac[39:0],data};
    if(index>=22&&index<=41)begin if(!index[0])ip_hi<=data;else ip_sum<=ip_sum+{4'b0,ip_hi,data};end
    if(index==14)mac_ok<=mac==48'h000a3501fec0||mac==48'hffffffffffff;
    if(index==42)fold1<={1'b0,ip_sum[15:0]}+{13'b0,ip_sum[19:16]};
    if(index==43)fold_good<=fold1==17'h0ffff || fold1==17'h1fffe;
    if(index==44)ip_ok<=fold_good;
    if(header_bad)bad<=1;
   end
  end else if(active)begin
   active<=0;
  end
 end
end
endmodule
