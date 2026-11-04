`timescale 1ns/1ps
`include "r6_motion_policy.vh"
// 执行端使用带符号的 RPM×100；命令载荷由 GMII CRC 校验。
// 主机只映射一次驱动极性，不再重复应用旧远程电机异或处理。
// 相对命令看门狗上限500ms，不是与RK同步的绝对期限。
// 上层需连接ready、local_stop和fault，并保留原有停止处理。
module nav_velocity_guard #(
 parameter integer CLK_HZ=50000000,
 parameter [31:0] CONFIG_ID=`R6_CONFIG_ID,
 parameter integer MAX_TTL_MS=`R6_COMMAND_TTL_MS,
 parameter integer MAX_RPM_CENTI=12000,
 parameter integer MAX_PIVOT_RPM_CENTI=8000
)(
 input clk,rst_n,input cmd_valid,input [479:0] command,
 input ready,local_stop,fault,
 output reg claimed,armed,output reg [63:0] targets,output reg[7:0] motion_intent,
 output reg [31:0] session,sequence_last,output reg [63:0] lease,
 output reg [7:0] state,reason,output reg [1:0] owner_mode,
 output reg [31:0] accepted_count,rejected_count,watchdog_count,
 output reg accepted_pulse
);
 localparam integer CYCLES_PER_MS=CLK_HZ/1000;
 reg [31:0] divider;
 reg [15:0] remaining;
 reg exhausted;
 wire millisecond=divider==CYCLES_PER_MS-1;
 wire [7:0] op=command[47:40],mode=command[55:48];
 wire [31:0] incoming_session=command[95:64],incoming_sequence=command[127:96];
 wire [63:0] incoming_lease=command[191:128];
 wire [15:0] ttl=command[207:192];
 wire [63:0] incoming_targets=command[287:224];
 wire [31:0] config_id=command[319:288];
 wire [31:0] advance=incoming_sequence-sequence_last;
 wire sequence_new=advance!=0&&!advance[31];
 wire envelope=command[31:0]==32'h3156434e&&command[39:32]==3&&op<=5&&
   command[223:208]==0&&command[479:320]==0;
 wire probe=op==0&&mode==0&&incoming_session==0&&incoming_sequence==0&&incoming_lease==0&&ttl==0&&incoming_targets==0&&config_id==0&&command[63:56]==0;
 wire ordinary=(mode==1||mode==2)&&incoming_session!=0&&incoming_lease!=0&&config_id==CONFIG_ID&&
   ((op==3&&ttl>=1&&ttl<=MAX_TTL_MS)||(op!=3&&ttl==0&&incoming_targets==0&&command[63:56]==0));
 function targets_valid;
  input [63:0] values;input [7:0] intent;
  integer i;reg signed [15:0] v;
  begin targets_valid=1;
   for(i=0;i<4;i=i+1)begin
    v=values[i*16+:16];
    if((v>MAX_RPM_CENTI || v< -MAX_RPM_CENTI) ||
       (intent==3 && (v>MAX_PIVOT_RPM_CENTI || v< -MAX_PIVOT_RPM_CENTI)))targets_valid=0;
   end
  end
 endfunction
 function intent_valid;
 input[63:0] values;input[7:0] intent;
 reg signed[16:0] l,r;reg signed[17:0] sum,diff;
 begin
 l=$signed(values[15:0]);r=-$signed({values[31],values[31:16]});sum=l+r;diff=r-l;if(diff<0)diff=-diff;
 intent_valid=values[15:0]==values[47:32]&&values[31:16]==values[63:48]&&
 ((l==0&&r==0&&intent==0)||(sum>0&&(intent==1||(intent==2&&l!=r&&4*diff>=sum)))||
 (sum==0&&(l!=0||r!=0)&&intent==3)||(sum<0&&intent==4));
 end endfunction
 task halt;
  input [7:0] next_state,next_reason;
  begin
   armed<=0;targets<=0;motion_intent<=0;remaining<=0;state<=next_state;reason<=next_reason;
   if(lease==64'hffffffffffffffff)begin exhausted<=1;state<=5;reason<=10;end
   else lease<=lease+1'b1;
  end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   claimed<=0;armed<=0;targets<=0;motion_intent<=0;session<=0;sequence_last<=0;lease<=1;
   state<=0;reason<=0;owner_mode<=0;accepted_count<=0;rejected_count<=0;watchdog_count<=0;
   divider<=0;remaining<=0;exhausted<=0;accepted_pulse<=0;
  end else begin
   accepted_pulse<=0;
   if(millisecond)divider<=0;else divider<=divider+1'b1;
   if(exhausted||lease==64'hffffffffffffffff)begin exhausted<=1;armed<=0;targets<=0;motion_intent<=0;remaining<=0;state<=5;reason<=10;end
   else if(claimed&&local_stop)begin
    halt(5,6);claimed<=0;session<=0;owner_mode<=0;
    if(cmd_valid)rejected_count<=rejected_count+1'b1;
   end else if(armed&&(!ready||fault))begin
    halt(5,local_stop?6:fault?7:5);
    if(cmd_valid)rejected_count<=rejected_count+1'b1;
   end else if(millisecond&&remaining==1)begin
    halt(6,8);watchdog_count<=watchdog_count+1'b1;
    if(cmd_valid)rejected_count<=rejected_count+1'b1;
   end else begin
    if(millisecond&&remaining!=0)remaining<=remaining-1'b1;
    if(cmd_valid)begin
     if(!envelope||(!probe&&!ordinary))begin
      rejected_count<=rejected_count+1'b1;reason<=config_id!=CONFIG_ID&&op!=0?9:1;
     end else if(probe)begin /* 不修改控制权、期限或已接受命令计数。 */ end
     else if(op==1)begin
      if(armed||local_stop||fault||!ready)begin rejected_count<=rejected_count+1'b1;reason<=5;end
      else if(incoming_lease!=lease)begin rejected_count<=rejected_count+1'b1;reason<=3;end
      else if(claimed&&incoming_session==session&&!sequence_new)begin rejected_count<=rejected_count+1'b1;reason<=4;end
      else begin
       halt(1,0);claimed<=1;session<=incoming_session;sequence_last<=incoming_sequence;owner_mode<=mode[1:0];
       accepted_count<=accepted_count+1'b1;accepted_pulse<=1;
      end
     end else if(!claimed||incoming_session!=session||mode!={6'b0,owner_mode})begin rejected_count<=rejected_count+1'b1;reason<=2;end
     else if(!sequence_new)begin rejected_count<=rejected_count+1'b1;reason<=4;end
     else if(op!=4&&incoming_lease!=lease)begin rejected_count<=rejected_count+1'b1;reason<=3;end
     else if(op==4||op==5)begin
      halt(op==5?0:4,0);sequence_last<=incoming_sequence;accepted_count<=accepted_count+1'b1;accepted_pulse<=1;
      if(op==5)begin claimed<=0;session<=0;owner_mode<=0;end
     end else if(!ready||local_stop||fault)begin rejected_count<=rejected_count+1'b1;reason<=local_stop?6:fault?7:5;end
     else if(op==2&&!armed)begin
      halt(2,0);remaining<=MAX_TTL_MS;armed<=1;sequence_last<=incoming_sequence;accepted_count<=accepted_count+1'b1;accepted_pulse<=1;
     end else if(op==3&&armed&&targets_valid(incoming_targets,command[63:56])&&intent_valid(incoming_targets,command[63:56]))begin
      targets<=incoming_targets;motion_intent<=command[63:56];remaining<=ttl;state<=3;reason<=0;
      sequence_last<=incoming_sequence;accepted_count<=accepted_count+1'b1;accepted_pulse<=1;
     end else begin rejected_count<=rejected_count+1'b1;reason<=op==3&&!armed?5:1;end
    end
   end
  end
 end
endmodule
