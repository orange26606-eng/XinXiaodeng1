`timescale 1ns/1ps
// 仅被动遥测，不输出电机控制，也不执行I2C事务。
module odom_link #(parameter CLK_HZ=50000000,REPORT_CLKS=1000000)(
 input clk50,clk_imu,clk125,rst_n,reset_key_n,
 input [127:0] positions,input [3:0] enc_ready,
 input imu_valid,input [130:0] imu_data,input tx_grant,
 output pending,txen,output [7:0] txd);
wire imu_mail_valid;wire [130:0] imu_mail_data;
(* ASYNC_REG="TRUE" *) reg [1:0] tx_reset_sync=0,imu_reset_sync=0;
always @(posedge clk125 or negedge rst_n)
 if(!rst_n)tx_reset_sync<=0;else tx_reset_sync<={tx_reset_sync[0],1'b1};
always @(posedge clk_imu or negedge rst_n)
 if(!rst_n)imu_reset_sync<=0;else imu_reset_sync<={imu_reset_sync[0],1'b1};
reg imu_was_valid;
always @(posedge clk_imu or negedge imu_reset_sync[1])
 if(!imu_reset_sync[1])imu_was_valid<=0;else imu_was_valid<=imu_valid;
imu_bundle_cdc #(.WIDTH(131)) imu_tap(.src_clk(clk_imu),.src_rst_n(rst_n),
 .src_valid(imu_valid && !imu_was_valid),.src_ready(),.src_data(imu_data),.dst_clk(clk50),.dst_rst_n(rst_n),
 .dst_valid(imu_mail_valid),.dst_ready(1'b1),.dst_data(imu_mail_data));
(* ASYNC_REG="TRUE" *) reg [1:0] reset_sync=0,key_sync=3;
always @(posedge clk50 or negedge rst_n)
 if(!rst_n)begin reset_sync<=0;key_sync<=3;end
 else begin reset_sync<={reset_sync[0],1'b1};key_sync<={key_sync[0],reset_key_n};end
reg key_prev;
reg [31:0] epoch,clock_ms,ms_div,report_div,packet_sequence,imu_age,imu_seq,imu_token;
reg [31:0] yaw;
reg [2:0] imu_state;
reg snap_valid;reg [479:0] snapshot;
wire mail_ready,mail_valid,tx_ready;wire [479:0] mail_data;
always @(posedge clk50 or negedge reset_sync[1])begin
 if(!reset_sync[1])begin
  epoch<=0;clock_ms<=0;ms_div<=0;report_div<=0;packet_sequence<=0;imu_age<=32'hffffffff;
  imu_seq<=0;imu_token<=0;yaw<=0;imu_state<=0;key_prev<=1;snap_valid<=0;snapshot<=0;
 end else begin
  key_prev<=key_sync[1];snap_valid<=0;
  if(key_prev && !key_sync[1])epoch<=epoch+1'b1;
  if(ms_div==CLK_HZ/1000-1)begin
   ms_div<=0;clock_ms<=clock_ms+1'b1;
   if(imu_age!=32'hffffffff)imu_age<=imu_age+1'b1;
  end else ms_div<=ms_div+1'b1;
  if(imu_mail_valid)begin
   yaw<=imu_mail_data[95:64];imu_state<=imu_mail_data[98:96];imu_token<=imu_mail_data[130:99];
   imu_age<=0;imu_seq<=imu_seq+1'b1;
  end
  if(report_div==REPORT_CLKS-1)begin
   report_div<=0;
   if(mail_ready)begin
    snapshot<=0;
    snapshot[31:0]<=32'h3130444f; // OD01字节布局。
    snapshot[39:32]<=1;
    snapshot[47:40]<={7'b0,!key_sync[1]};
    snapshot[55:48]<={4'b0,enc_ready};snapshot[63:56]<={5'b0,imu_state};
    snapshot[95:64]<=packet_sequence;snapshot[127:96]<=clock_ms;snapshot[159:128]<=epoch;
    snapshot[287:160]<=positions;snapshot[319:288]<=yaw;
    snapshot[351:320]<=imu_age;snapshot[383:352]<=imu_seq;snapshot[415:384]<=imu_token;
    packet_sequence<=packet_sequence+1'b1;snap_valid<=1;
   end
  end else report_div<=report_div+1'b1;
 end
end
imu_bundle_cdc #(.WIDTH(480)) odom_mail(.src_clk(clk50),.src_rst_n(rst_n),
 .src_valid(snap_valid),.src_ready(mail_ready),.src_data(snapshot),.dst_clk(clk125),.dst_rst_n(rst_n),
 .dst_valid(mail_valid),.dst_ready(tx_grant && tx_ready),.dst_data(mail_data));
assign pending=mail_valid &&tx_ready;
odom_udp_tx sender(.clk125m(clk125),.rst_n(tx_reset_sync[1]),.sample_valid(tx_grant &&pending),
 .sample_ready(tx_ready),.sample_data(mail_data),.gmii_txen(txen),.gmii_txd(txd),.tx_done());
endmodule
