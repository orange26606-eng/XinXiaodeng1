`timescale 1ns/1ps
module age_equivalence #(parameter CLK_HZ=50000000)(output reg done=0);
 function[3:0]age200;input[31:0]a;reg[31:0]v;reg[3:0]q;

 begin v=a;q=0;

 if(v>=CLK_HZ*3)q=15;

 else begin

  if(v>=CLK_HZ*8/5)begin v=v-CLK_HZ*8/5;q=q+8;end

  if(v>=CLK_HZ*4/5)begin v=v-CLK_HZ*4/5;q=q+4;end

  if(v>=CLK_HZ*2/5)begin v=v-CLK_HZ*2/5;q=q+2;end

  if(v>=CLK_HZ/5)q=q+1;

 end age200=q;end endfunction
 function [34:0] age200_high;
 input [31:0] age;
 reg [31:0] remainder;reg [3:0] bucket;reg clipped;
 begin
  remainder=age;bucket=0;clipped=age>=CLK_HZ*3;
  if(clipped)remainder=0;
  else begin
   if(remainder>=CLK_HZ*8/5)begin remainder=remainder-CLK_HZ*8/5;bucket=bucket+8;end
   if(remainder>=CLK_HZ*4/5)begin remainder=remainder-CLK_HZ*4/5;bucket=bucket+4;end
  end
  age200_high={clipped,bucket[3:2],remainder};
 end
 endfunction
 function [3:0] age200_low;
 input [34:0] intermediate;
 reg [31:0] remainder;reg [3:0] bucket;
 begin
  remainder=intermediate[31:0];bucket={intermediate[33:32],2'b00};
  if(intermediate[34])bucket=15;
  else begin
   if(remainder>=CLK_HZ*2/5)begin remainder=remainder-CLK_HZ*2/5;bucket=bucket+2;end
   if(remainder>=CLK_HZ/5)bucket=bucket+1;
  end
  age200_low=bucket;
 end
 endfunction
 integer i,seed=33931;reg[31:0]value;reg[3:0]expected,actual;
 initial begin
  for(i=0;i<30000;i=i+1)begin
   case(i%3)
    0:value=($random(seed)&32'h7fffffff)%(CLK_HZ*4);
    1:value=(i%16)*CLK_HZ/5+((i/3)%3)-1;
    default:value=$random(seed);
   endcase
   expected=age200(value);actual=age200_low(age200_high(value));
   if(expected!==actual)begin $display("FAIL age CLK=%0d value=%0d expected=%0d actual=%0d",CLK_HZ,value,expected,actual);$fatal;end
   #1;
  end
  done=1;
 end
endmodule
module tb_route_age;
 wire[2:0]done;
 age_equivalence #(.CLK_HZ(50000000)) a(done[0]);
 age_equivalence #(.CLK_HZ(10000)) b(done[1]);
 age_equivalence #(.CLK_HZ(12347)) c(done[2]);
 initial begin wait(&done);$display("PASS: age bucket arithmetic identical random/range/boundary cases=90000 clock_profiles=3");$finish;end
endmodule
