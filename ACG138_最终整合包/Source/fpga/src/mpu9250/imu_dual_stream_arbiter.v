`timescale 1ns/1ps
// 原整车IMU发送许可之后增加整帧子仲裁，
// 每次外层许可最多发送一帧，不交错前导码或CRC。
module imu_dual_stream_arbiter(input clk,rst_n,grant,
 input raw_pending,att_pending,output raw_start,att_start,
 input raw_txen,input[7:0]raw_txd,input raw_done,
 input att_txen,input[7:0]att_txd,input att_done,
 output pending,txen,output[7:0]txd);
 reg[1:0]owner;reg last_att;
 assign pending=rst_n&&owner==0&&(raw_pending||att_pending);
 assign raw_start=pending&&grant&&raw_pending&&(!att_pending||last_att);
 assign att_start=pending&&grant&&att_pending&&(!raw_pending||!last_att);
 assign txen=rst_n&&((owner==1&&raw_txen)||(owner==2&&att_txen));
 assign txd=owner==1?raw_txd:owner==2?att_txd:8'd0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin owner<=0;last_att<=1;end
  else if(owner==0)begin
   if(raw_start)begin owner<=1;last_att<=0;end
   else if(att_start)begin owner<=2;last_att<=1;end
  end else if((owner==1&&raw_done)||(owner==2&&att_done))owner<=0;
 end
endmodule
