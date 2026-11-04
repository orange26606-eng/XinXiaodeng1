`timescale 1ns/1ps
// NCV1 命令接收器，监听 UDP5012。
// 沿用motor_network_tuner_20261025的net_command_rx.v。
// 固定64字节载荷，CRC和FCS正确后由命令保护模块检查NCV1语义。
// 此模块未单独完成顶层接入和时序验证。
module nav_command_rx_gmii(input clk,input rst_n,input en,input er,input [7:0] data,
 output reg valid,input ready,output reg [479:0] command);
reg active,bad,drop;
reg [7:0] index;
reg [47:0] mac;
reg [479:0] candidate;
reg [31:0] eth_crc,app_crc,fcs,app_received;
reg [19:0] ip_sum;reg [7:0] ip_hi;reg [16:0] fold1,fold2;
reg ip_ok,mac_ok;
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
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin active<=0;bad<=0;drop<=0;index<=0;mac<=0;candidate<=0;
 eth_crc<=32'hffffffff;app_crc<=32'hffffffff;fcs<=0;app_received<=0;
 ip_sum<=0;ip_hi<=0;fold1<=0;fold2<=0;ip_ok<=0;mac_ok<=0;valid<=0;command<=0;end
 else begin
  if(valid&&ready)valid<=0;
  if(en)begin
   if(!active)begin active<=1;index<=1;bad<=er||data!=8'h55;drop<=valid&&!ready;
    mac<=0;candidate<=0;eth_crc<=32'hffffffff;app_crc<=32'hffffffff;
    fcs<=0;app_received<=0;ip_sum<=0;ip_ok<=0;mac_ok<=0;
   end else begin
    if(er)bad<=1;
    if(index<118)index<=index+1'b1;else bad<=1;
    if(index>=8&&index<=113)eth_crc<=crc32(eth_crc,data);
    if(index>=50&&index<=109)begin candidate[(index-50)*8+:8]<=data;app_crc<=crc32(app_crc,data);end
    if(index>=110&&index<=113)app_received[(index-110)*8+:8]<=data;
    if(index>=114&&index<=117)fcs[(index-114)*8+:8]<=data;
    if(index>=8&&index<=13)mac<={mac[39:0],data};
    if(index>=22&&index<=41)begin if(!index[0])ip_hi<=data;else ip_sum<=ip_sum+{4'b0,ip_hi,data};end
    if(index==14)mac_ok<=mac==48'h000a3501fec0||mac==48'hffffffffffff;
    if(index==42)fold1<={1'b0,ip_sum[15:0]}+{13'b0,ip_sum[19:16]};
    if(index==43)fold2<={1'b0,fold1[15:0]}+fold1[16];
    if(index==44)ip_ok<=fold2==17'h0ffff;
    case(index)
     1,2,3,4,5,6:if(data!=8'h55)bad<=1;
     7:if(data!=8'hd5)bad<=1;
     20:if(data!=8'h08)bad<=1;
     21,24,29,36,40,46,48,49:if(data!=0)bad<=1;
     22:if(data!=8'h45)bad<=1;
     25:if(data!=92)bad<=1;
     28:if(data!=0&&data!=8'h40)bad<=1;
     31:if(data!=17)bad<=1;
     34,38:if(data!=192)bad<=1;
     35,39:if(data!=168)bad<=1;
     37:if(data!=3)bad<=1;
     41:if(data!=2)bad<=1;
     42,44:if(data!=8'h13)bad<=1;
     43,45:if(data!=8'h94)bad<=1;
     47:if(data!=72)bad<=1;
    endcase
   end
  end else if(active)begin
   active<=0;
   if(index==118&&!bad&&!drop&&ip_ok&&mac_ok&&fcs==~eth_crc&&app_received==~app_crc&&(!valid||ready))begin command<=candidate;valid<=1;end
  end
 end
end
endmodule
