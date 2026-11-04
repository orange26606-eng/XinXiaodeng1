`timescale 1ns/1ps
// atan2输入有符号Q8.24向量，输出经象限修正的有符号Q8.24弧度。
module atan2_cordic(input clk,rst_n,start,input signed[31:0]x,y,
 output reg busy,done,output reg signed[31:0]angle);
 reg signed[39:0]vx,vy;reg signed[31:0]z;reg[5:0]i;
 function signed[31:0]table_angle;input[5:0]n;
 begin case(n)
  0:table_angle=13176795;1:table_angle=7778716;2:table_angle=4110060;
  3:table_angle=2086331;4:table_angle=1047214;5:table_angle=524117;
  6:table_angle=262123;7:table_angle=131069;8:table_angle=65536;
  9:table_angle=32768;10:table_angle=16384;11:table_angle=8192;
  12:table_angle=4096;13:table_angle=2048;14:table_angle=1024;
  15:table_angle=512;16:table_angle=256;17:table_angle=128;
  18:table_angle=64;19:table_angle=32;20:table_angle=16;
  21:table_angle=8;22:table_angle=4;23:table_angle=2;default:table_angle=1;
 endcase end endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;angle<=0;vx<=0;vy<=0;z<=0;i<=0;end
  else begin done<=0;
   if(start&&!busy)begin
    busy<=1;i<=0;
    if(x<0)begin vx<=-$signed({{8{x[31]}},x});vy<=-$signed({{8{y[31]}},y});z<=y<0?-32'sd52707179:32'sd52707179;end
    else begin vx<=x;vy<=y;z<=0;end
   end else if(busy)begin
    if(i==25)begin angle<=z;busy<=0;done<=1;end
    else begin
     if(vy>0)begin vx<=vx+(vy>>>i);vy<=vy-(vx>>>i);z<=z+table_angle(i);end
     else if(vy<0)begin vx<=vx-(vy>>>i);vy<=vy+(vx>>>i);z<=z-table_angle(i);end
     i<=i+1'b1;
    end
   end
  end
 end
endmodule
