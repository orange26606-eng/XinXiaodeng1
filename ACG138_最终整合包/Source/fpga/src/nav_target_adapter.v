`timescale 1ns/1ps
// 将目标转换为旧100ms PI窗口使用的无符号编码器
// 计数，有符号RPM×100保留最终驱动极性。
// 独立许可还需作用于最终PWM，本转换模块不直接驱动电机。
module nav_target_adapter #(
 parameter integer COUNTS_PER_REV=1320,
 parameter integer WINDOW_HZ=10
)(
 input [63:0] rpm_centi,input permit,
 output [43:0] ticks_per_window,output [3:0] reverse
);
 genvar i;
 generate for(i=0;i<4;i=i+1)begin:g
  wire signed [15:0]value=rpm_centi[i*16+:16];
  wire[15:0]magnitude=value[15]?(~value+16'd1):value;
  wire[47:0]product=magnitude*COUNTS_PER_REV;
  // RPM到计数向零取整。
  wire[47:0]ticks=product/(6000*WINDOW_HZ);
  assign ticks_per_window[i*11+:11]=!permit?0:ticks>2047?11'd2047:ticks[10:0];
  assign reverse[i]=permit&&magnitude!=0&&value[15];
 end endgenerate
endmodule
