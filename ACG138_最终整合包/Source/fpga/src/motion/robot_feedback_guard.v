`timescale 1ns/1ps
// 整车反馈监控保留停机前后的计时状态。
module robot_feedback_guard #(
 parameter START_CLKS=50000000,QUIET_CLKS=30000000,LIMIT_CLKS=15000000,
 parameter WAIT_CLKS=50000000,STABLE_CLKS=100000000
)(input clk,rst_n,clear_request,input [3:0] demand,actual,
 input [43:0] targets,input [63:0] ticks,duties,input [3:0] window_end,
 output stop_drive,freeze_pi,low_speed,reset_pi,
 output reg [2:0] phase,output reg [1:0] retries,output reg [3:0] fault_mask,
 output reg [63:0] ceiling);
 localparam RUN=0,LIMITED=1,WAITING=2,PROBE=3,LOCKED=4;
 reg [31:0] startup[0:3],quiet[0:3];
 reg [1:0] good[0:3];reg [3:0] running;
 reg [31:0] timer,stable;reg [3:0] action_saved;
 reg [3:0] bad,confirmed;integer i;
 wire [3:0] active={targets[33+:11]!=0,targets[22+:11]!=0,targets[11+:11]!=0,targets[0+:11]!=0};
 assign stop_drive=phase==WAITING || phase==LOCKED;
 assign reset_pi=stop_drive;
 assign freeze_pi=phase==LIMITED;
 assign low_speed=phase==PROBE;
 always @*begin
  bad=0;confirmed=0;
  for(integer j=0;j<4;j=j+1)begin
   bad[j]=active[j] && (running[j] || startup[j]>=START_CLKS) && quiet[j]>=QUIET_CLKS;
   confirmed[j]=!active[j] || good[j]>=3;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   phase<=RUN;retries<=0;fault_mask<=0;ceiling<=0;timer<=0;stable<=0;running<=0;action_saved<=0;
   for(i=0;i<4;i=i+1)begin startup[i]<=0;quiet[i]<=0;good[i]<=0;end
  end else if(clear_request)begin
   phase<=RUN;retries<=0;fault_mask<=0;timer<=0;stable<=0;running<=0;ceiling<=0;action_saved<=0;
   for(i=0;i<4;i=i+1)begin startup[i]<=0;quiet[i]<=0;good[i]<=0;end
  end else begin
   for(i=0;i<4;i=i+1)begin
    if(!active[i] || stop_drive)begin startup[i]<=0;quiet[i]<=0;good[i]<=0;running[i]<=0;end
    else begin
     if(startup[i]<START_CLKS)startup[i]<=startup[i]+1'b1;
     if(window_end[i])begin
      if($signed(ticks[i*16+:16])>=2 || $signed(ticks[i*16+:16])<=-2)begin
       if(good[i]<3)good[i]<=good[i]+1'b1;
       if(good[i]>=2)running[i]<=1;
      end else good[i]<=0;
     end
     if(window_end[i] && ticks[i*16+:16]!=0)quiet[i]<=0;
     else if((!running[i] && startup[i]<START_CLKS) || duties[i*16+:16]<156)quiet[i]<=0;
     else if(quiet[i]<QUIET_CLKS)quiet[i]<=quiet[i]+1'b1;
    end
   end
   case(phase)
    RUN,PROBE:begin
     if(|bad)begin
      phase<=LIMITED;timer<=0;stable<=0;fault_mask<=fault_mask|bad;ceiling<=duties;action_saved<=actual;
      for(i=0;i<4;i=i+1)good[i]<=0;
     end else if(phase==PROBE)begin
      if(actual!=0 && (&confirmed))begin
       if(stable>=STABLE_CLKS-1)begin phase<=RUN;retries<=0;fault_mask<=0;stable<=0;end
       else stable<=stable+1'b1;
      end else stable<=0;
     end
    end
    LIMITED:begin
     // 开环受限恢复期间不换方向，传感器和用户停止条件优先。
     if(demand==0 || demand!=action_saved || timer>=LIMIT_CLKS-1)begin phase<=WAITING;timer<=0;stable<=0;end
     else if((|(active)) && (&confirmed))begin phase<=PROBE;timer<=0;stable<=0;end
     else timer<=timer+1'b1;
    end
    WAITING:begin
     if(timer<WAIT_CLKS)timer<=timer+1'b1;
     if(timer>=WAIT_CLKS-1)begin
      if(retries>=2)phase<=LOCKED;
      else if(demand!=0)begin phase<=PROBE;retries<=retries+1'b1;timer<=0;stable<=0;end
     end
    end
    LOCKED:begin end
    default:phase<=LOCKED;
   endcase
  end
 end
endmodule
