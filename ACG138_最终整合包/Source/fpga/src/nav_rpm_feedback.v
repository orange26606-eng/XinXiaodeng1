`timescale 1ns/1ps
// 将有符号100ms计数换算为RPM×100，即counts×500/11，
// 用25周期有界恢复除法完成，避免
// 50MHz状态路径出现长组合除法，仅消费实际编码器窗口。
module nav_rpm_feedback(input clk,rst_n,input[63:0]ticks,input[3:0]window_end,
 output[63:0]rpm_centi);
genvar i;generate for(i=0;i<4;i=i+1)begin:g
 wire[15:0]raw=ticks[i*16+:16];wire[15:0]absolute=raw[15]?(~raw+16'd1):raw;
 reg[24:0]work;reg[3:0]remainder;reg[4:0]remaining;reg negative;reg[15:0]rpm;
 wire[4:0]trial={remainder,work[24]};wire bit_value=trial>=11;
 wire[24:0]next_work={work[23:0],bit_value};
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin work<=0;remainder<=0;remaining<=0;negative<=0;rpm<=0;end
  else if(remaining!=0)begin
   work<=next_work;remainder<=bit_value?trial-5'd11:trial;remaining<=remaining-1'b1;
   if(remaining==1)begin
    if(negative)rpm<=next_work>=32768?16'h8000:(~next_work[15:0]+16'd1);
    else rpm<=next_work>32767?16'h7fff:next_work[15:0];
   end
  end else if(window_end[i])begin work<=absolute*25'd500;remainder<=0;remaining<=25;negative<=raw[15];end
 assign rpm_centi[i*16+:16]=rpm;
end endgenerate
endmodule
