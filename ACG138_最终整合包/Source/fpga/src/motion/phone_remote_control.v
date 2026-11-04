`timescale 1ns/1ps

// 新的显式MANUAL命令获得控制权，HOLD或PING不能恢复已过期会话。
module phone_remote_control #(
 parameter WATCHDOG_CLKS=25000000
)(
 input clk,rst_n,input cmd_valid,input [79:0] cmd_data,input [3:0] local_pressed,
 output reg claimed,live,output reg [3:0] motion,
 output reg start_auto,force_stop,output [31:0] status,
 output reg [7:0] stop_source
);
 wire [7:0] op=cmd_data[79:72],move=cmd_data[71:64];
 wire [31:0] incoming_token=cmd_data[63:32],incoming_sequence=cmd_data[31:0];
 reg [31:0] token,sequence_last,age;
 reg expired,local_override;
 wire signed [31:0] advance=incoming_sequence-sequence_last;
 wire matching=claimed && incoming_token==token && advance>0;
 wire claim_ok=op==1 && incoming_token!=0 && (!claimed || !live || incoming_token==token) &&
               (!claimed || incoming_token!=token || advance>0);
 assign status={sequence_last[15:0],4'b0,local_override,expired,live,claimed,8'hA1};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin claimed<=0;live<=0;motion<=0;start_auto<=0;force_stop<=0;
   token<=0;sequence_last<=0;age<=0;expired<=0;local_override<=0;stop_source<=1;end
  else begin
   start_auto<=0;force_stop<=0;stop_source<=0;
   if(local_pressed!=0)begin
    if(claimed)begin force_stop<=1;stop_source<=8'h02;end
    claimed<=0;live<=0;motion<=0;expired<=0;local_override<=1;age<=0;
   end else if(cmd_valid && claim_ok)begin
    token<=incoming_token;sequence_last<=incoming_sequence;claimed<=1;live<=1;expired<=0;
    local_override<=0;motion<=0;force_stop<=1;stop_source<=8'h80;age<=0;
   end else if(cmd_valid && matching && op==0)begin
    sequence_last<=incoming_sequence;motion<=0;force_stop<=1;stop_source<=8'h08;age<=0;
   end else if(cmd_valid && matching && live && !expired && op>=2 && op<=4)begin
    sequence_last<=incoming_sequence;age<=0;
    if(op==2)begin start_auto<=1;motion<=0;stop_source<=8'h20;end
    else if(op==3 && move>=1 && move<=8)motion<=move[3:0];
    else if(op==4)motion<=0;
   end else if(claimed && live)begin
    if(age>=WATCHDOG_CLKS-1)begin live<=0;motion<=0;force_stop<=1;stop_source<=8'h40;expired<=1;end
    else age<=age+1'b1;
   end
  end
 end
endmodule
