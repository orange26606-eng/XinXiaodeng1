`timescale 1ns/1ps
module three_a02 #(parameter CLK_HZ=50000000, TICK_CLKS=326,
 GAP_CLKS=500000, STALE_CLKS=50000000, HOST_BAUD=115200)(
 input clk, input front_rx,left_rx,right_rx,
 output host_tx, output [7:0] led);
 reg [7:0] startup=0;
 wire rst_n=&startup;
 always @(posedge clk) if(!rst_n) startup<=startup+1'b1;
 wire [23:0] front,left,right;
 wire [2:0] valid,update;
 wire [8:0] status;
 XR_ultrasonic #(.P_TICK_CLKS(TICK_CLKS),.P_GAP_CLKS(GAP_CLKS),.P_STALE_CLKS(STALE_CLKS)) sensors(
 .clk_sys(clk),.rst_n(rst_n),.us_front_rxd(front_rx),.us_left_rxd(left_rx),.us_right_rxd(right_rx),
 .Distance(front),.distance_left(left),.distance_right(right),.valid_mask(valid),.sample_update(update),
 .status_codes(status),.age_front(),.age_left(),.age_right());
 wire [3:0] decision,attempts,cmd;
 wire blocked,fault;
 XR_obstacle_avoid avoid(.clk_sys(clk),.rst_n(rst_n),.enable(1'b1),
 .front_mm(front),.left_mm(left),.right_mm(right),.valid_mask(valid),.sample_update(update),
 .motor_cmd(cmd),.state_code(decision),.attempts(attempts),.blocked(blocked),.fault(fault),
 .target_speed_l(),.target_speed_r());
 // 动作决策仅用于遥测，本模块没有电机输出引脚。
 reg [31:0] count;
 reg blink;
 wire report=count==CLK_HZ/5-1;
 reg [31:0] heartbeat_count;
 always @(posedge clk) begin
  if(!rst_n) begin count<=0;heartbeat_count<=0;blink<=0;end
  else begin
   if(report)count<=0;else count<=count+1'b1;
   if(heartbeat_count==CLK_HZ/2-1)begin heartbeat_count<=0;blink<=~blink;end
   else heartbeat_count<=heartbeat_count+1'b1;
  end
 end
 assign led[0]=blink;
 assign led[1]=valid[0] && front<200;
 assign led[2]=valid[0] && front>=200 && front<300;
 assign led[3]=valid[0] && front>=300 && front<450;
 assign led[4]=valid[0] && front>=450 && front<600;
 assign led[5]=valid[1] && ((left<120) || (left<450 && blink));
 assign led[6]=valid[2] && ((right<120) || (right<450 && blink));
 assign led[7]=!(&valid);
 function [39:0] decimal5;
 input [15:0] n;
 reg [19:0] bcd;
 integer i,j;
 begin
  bcd=0;
  for(i=15;i>=0;i=i-1)begin
   for(j=0;j<5;j=j+1)if(bcd[j*4+:4]>=5)bcd[j*4+:4]=bcd[j*4+:4]+4'd3;
   bcd={bcd[18:0],n[i]};
  end
  decimal5={8'd48+{4'b0,bcd[19:16]},8'd48+{4'b0,bcd[15:12]},8'd48+{4'b0,bcd[11:8]},8'd48+{4'b0,bcd[7:4]},8'd48+{4'b0,bcd[3:0]}};
 end
 endfunction
 function [7:0] digit;
 input [3:0] n;
 begin digit=n<10?8'd48+n:8'd65+n-10;end
 endfunction
 localparam LEN=58;
 reg [LEN*8-1:0] packet;
 reg [7:0] remaining;
 wire busy;
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(HOST_BAUD)) tx(.clk(clk),.rst(!rst_n),
 .data(packet[LEN*8-1 -:8]),.valid(remaining!=0),.busy(busy),.serial_out(host_tx));
 always @(posedge clk)begin
  if(!rst_n)begin packet<=0;remaining<=0;end
  else if(report && remaining==0 && !busy)begin
   packet<={"F=",(valid[0]?decimal5(front[15:0]):"-----"),
   " L=",(valid[1]?decimal5(left[15:0]):"-----")," R=",(valid[2]?decimal5(right[15:0]):"-----"),
   " mm V=",(valid[0]?8'h31:8'h30),(valid[1]?8'h31:8'h30),(valid[2]?8'h31:8'h30),
   " S=",digit(decision)," CMD=",digit(cmd)," TRY=",digit(attempts),
   " ERR=",digit({1'b0,status[2:0]}),digit({1'b0,status[5:3]}),digit({1'b0,status[8:6]}),8'd13,8'd10};
   remaining<=LEN;
  end else if(remaining!=0 && !busy)begin packet<=packet<<8;remaining<=remaining-1'b1;end
 end
endmodule


