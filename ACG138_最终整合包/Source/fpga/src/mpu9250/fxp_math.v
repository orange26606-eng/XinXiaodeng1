`timescale 1ns/1ps
// 请求按事务提交；饱和及除零通过状态位报告。

module fxp_math(input clk,rst_n,start,input[1:0]op,
 input signed[31:0]a,b,input[63:0]rad,
 output reg busy,done,error,output reg signed[31:0]result);
 reg[2:0]state;reg[6:0]count;reg neg;
 reg signed[63:0]product;
 reg[63:0]dividend,quotient,square;
 reg[32:0]remainder;reg[31:0]denominator,root;
 reg[65:0]srem;
 wire[32:0]dr={remainder[31:0],dividend[63]};
 wire[63:0]dq={quotient[62:0],dr>={1'b0,denominator}};
 wire[65:0]sr={srem[63:0],square[63:62]};
 wire[65:0]trial={32'd0,root,2'b01};
 wire[31:0]next_root={root[30:0],sr>=trial};
 wire[31:0]aa=a[31]?-a:a,bb=b[31]?-b:b;
 wire signed[63:0]scaled=product>>>24;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=0;busy<=0;done<=0;error<=0;result<=0;count<=0;neg<=0;product<=0;dividend<=0;quotient<=0;square<=0;remainder<=0;denominator<=0;root<=0;srem<=0;end
  else begin
   done<=0;
   case(state)
    0:if(start)begin
     busy<=1;error<=0;
     case(op)
      0:begin product<=a*b;state<=1;end
      1,3:if(!b)begin result<=0;error<=1;state<=4;end
       else begin dividend<=op==1?{8'd0,aa,24'd0}:{32'd0,aa};denominator<=bb;
        quotient<=0;remainder<=0;count<=0;neg<=a[31]^b[31];state<=2;end
      2:begin square<=rad;root<=0;srem<=0;count<=0;state<=3;end
     endcase
    end
    1:begin
     if(scaled>64'sh7fffffff)begin result<=32'h7fffffff;error<=1;end
     else if(scaled< -64'sh80000000)begin result<=32'h80000000;error<=1;end
     else result<=scaled[31:0];state<=4;
    end
    2:begin
     dividend<=dividend<<1;quotient<=dq;
     remainder<=dr>={1'b0,denominator}?dr-{1'b0,denominator}:dr;
     if(count==63)begin
      if((!neg&&dq>64'h7fffffff)||(neg&&dq>64'h80000000))begin
       error<=1;result<=neg?32'h80000000:32'h7fffffff;
      end else result<=neg?-dq[31:0]:dq[31:0];state<=4;
     end else count<=count+1'b1;
    end
    3:begin
     square<=square<<2;srem<=sr>=trial?sr-trial:sr;root<=next_root;
     if(count==31)begin result<=next_root;state<=4;end else count<=count+1'b1;
    end
    4:begin done<=1;busy<=0;state<=0;end
    default:begin error<=1;state<=4;end
   endcase
  end
 end
endmodule
