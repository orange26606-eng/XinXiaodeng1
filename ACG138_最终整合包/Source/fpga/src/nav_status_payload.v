`timescale 1ns/1ps
// 锁存92字节状态，每周期计算一个字节应用CRC，并保持至
// 下游取走；下游补充IPv4、UDP5013及以太网FCS。
module nav_status_payload(
 input clk,rst_n,input request,input [735:0] snapshot,
 output request_ready,output reg valid,input ready,output reg [767:0] payload
);
 reg busy;reg [6:0] index;reg [31:0] crc;
 assign request_ready=!busy&&(!valid||ready);
 function [31:0] crc_byte;
  input [31:0] previous;input [7:0] value;
  integer b;reg [31:0] c;
  begin c=previous^{24'b0,value};for(b=0;b<8;b=b+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc_byte=c;end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin valid<=0;payload<=0;busy<=0;index<=0;crc<=32'hffffffff;end
  else begin
   if(valid&&ready)valid<=0;
   if(request&&!busy&&(!valid||ready))begin
    payload<={32'b0,snapshot};busy<=1;index<=0;crc<=32'hffffffff;
   end else if(busy)begin
    if(index<92)begin crc<=crc_byte(crc,payload[index*8+:8]);index<=index+1'b1;end
    else begin payload[767:736]<=~crc;busy<=0;valid<=1;end
   end
  end
 end
endmodule
