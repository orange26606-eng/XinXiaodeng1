`timescale 1ns/1ps
// 板上D0至D7高电平点亮，仅IIC的WAIT_LED状态启用流水灯。
module imu_wait_leds #(parameter integer STEP_CLKS=10000000)(
 input clk,rst_n,waiting,input [7:0] normal_led,output [7:0] led);
 localparam integer BITS=$clog2(STEP_CLKS>1?STEP_CLKS:2);
 reg [BITS-1:0] count;
 reg [7:0] chase;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin count<=0;chase<=8'b00000001;end
  else if(!waiting)begin count<=0;chase<=8'b00000001;end
  else if(count>=STEP_CLKS-1)begin count<=0;chase<={chase[6:0],chase[7]};end
  else count<=count+1'b1;
 end
 assign led=rst_n && waiting?chase:normal_led;
endmodule
