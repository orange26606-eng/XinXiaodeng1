`timescale 1ns/1ps
// 共用顺序移位加3转换器，每个值固定处理32周期。
module XR_bin32_bcd (
    input clk, rst_n, start,
    input [31:0] value,
    output reg busy, done,
    output reg [39:0] bcd
);
reg [31:0] binary;
reg [5:0] count;
reg [39:0] adjusted;
integer i;
always @* begin
    adjusted=bcd;
    for(i=0;i<10;i=i+1)
        if(bcd[i*4 +: 4]>=5) adjusted[i*4 +: 4]=bcd[i*4 +: 4]+4'd3;
end
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin binary<=0;count<=0;bcd<=0;busy<=0;done<=0;end
    else begin
        done<=0;
        if(start && !busy) begin binary<=value;count<=0;bcd<=0;busy<=1;end
        else if(busy) begin
            bcd<={adjusted[38:0],binary[31]};binary<={binary[30:0],1'b0};
            if(count==31)begin busy<=0;done<=1;end
            else count<=count+1'b1;
        end
    end
end
endmodule
