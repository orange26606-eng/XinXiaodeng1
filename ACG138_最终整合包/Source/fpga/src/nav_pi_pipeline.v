`timescale 1ns/1ps
// 默认1320计数每圈、100ms窗口，ticks=floor(abs(RPM×100)×11/500)。
// 120RPM每窗口为264计数，商9位、乘积18位、幅值14位。
// 六级一致流水线将原48位组合除法器与
// 许可及PI多路选择分开，permit撤销后立即禁止输出并清空
// 所有级，因此控制超时及STOP不等待流水线延迟。
module nav_pi_pipeline(input clk,rst_n,permit,input[63:0]rpm_centi,
 output[43:0]ticks_per_window,output[3:0]reverse);
reg[23:0]signs;reg[5:0]valid;
wire[3:0]incoming_sign={rpm_centi[63],rpm_centi[47],rpm_centi[31],rpm_centi[15]};
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin signs<=0;valid<=0;end
 else if(!permit)begin signs<=0;valid<=0;end
 else begin signs<={signs[19:0],incoming_sign};valid<={valid[4:0],1'b1};end
function[26:0]high_bits;input[17:0]n;reg[17:0]r;reg[8:0]q;
 begin r=n;q=0;if(r>=128000)begin r=r-128000;q[8]=1;end if(r>=64000)begin r=r-64000;q[7]=1;end if(r>=32000)begin r=r-32000;q[6]=1;end if(r>=16000)begin r=r-16000;q[5]=1;end high_bits={q,r};end
endfunction
function[26:0]middle_bits;input[26:0]n;reg[17:0]r;reg[8:0]q;
 begin r=n[17:0];q=n[26:18];if(r>=8000)begin r=r-8000;q[4]=1;end if(r>=4000)begin r=r-4000;q[3]=1;end middle_bits={q,r};end
endfunction
function[26:0]low_bits;input[26:0]n;reg[17:0]r;reg[8:0]q;
 begin r=n[17:0];q=n[26:18];if(r>=2000)begin r=r-2000;q[2]=1;end if(r>=1000)begin r=r-1000;q[1]=1;end low_bits={q,r};end
endfunction
genvar i;generate for(i=0;i<4;i=i+1)begin:g
 wire[15:0]value=rpm_centi[i*16+:16];wire[15:0]absolute=value[15]?(~value+16'd1):value;
 reg[13:0]magnitude;reg[17:0]product;reg[26:0]p2,p3,p4;reg[8:0]result;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin magnitude<=0;product<=0;p2<=0;p3<=0;p4<=0;result<=0;end
  else if(!permit)begin magnitude<=0;product<=0;p2<=0;p3<=0;p4<=0;result<=0;end
  else begin
   magnitude<=absolute>12000?14'd12000:absolute[13:0];
   product<=magnitude*18'd11;p2<=high_bits(product);p3<=middle_bits(p2);p4<=low_bits(p3);
   result<=p4[26:18]|(p4[17:0]>=500?9'd1:9'd0);
  end
 assign ticks_per_window[i*11+:11]=permit&&valid[5]?{2'd0,result}:11'd0;
 assign reverse[i]=permit&&valid[5]&&signs[20+i];
end endgenerate
endmodule
