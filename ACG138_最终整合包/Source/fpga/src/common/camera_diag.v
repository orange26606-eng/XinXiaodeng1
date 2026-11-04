//-------------------------------


// 日期：2026-10-12 18:20
// 模块：camera_diag
// 功能: 保持24字节UART诊断格式，使用握手快照传递相机计数，状态位双触发器同步。
// 版本：1.1.0
// 修订记录 ： v1.1.0----保持24字节UART诊断格式，使用握手快照传递相机计数，状态位双触发器同步。
//-------------------------------
// UART诊断保留24字节CA报文，波特率115200。
// 每眼独立产生一致的PCLK计数、行长度和帧计数
// 快照，两眼无需同时锁存。
// af1和af2须与clk同步，外层提供PLL产生的50MHz时钟。
module camera_diag #(parameter PERIOD=50000000, parameter BAUD_DIV=434)(
 input [15:0] af1,af2,
 input clk, reset_n, input [2:0] init_done,
 input pclk1,href1,vsync1,pclk2,href2,vsync2,
 output reg uart_tx=1'b1);
 (* ASYNC_REG="TRUE" *) reg [1:0] reset1_sync=0,reset2_sync=0;
 always @(posedge pclk1 or negedge reset_n)
  if(!reset_n)reset1_sync<=0;else reset1_sync<={reset1_sync[0],1'b1};
 always @(posedge pclk2 or negedge reset_n)
  if(!reset_n)reset2_sync<=0;else reset2_sync<={reset2_sync[0],1'b1};
 wire reset1_n=reset1_sync[1],reset2_n=reset2_sync[1];
 reg [31:0] pc1=0,pc2=0;
 reg [15:0] len1=0,len2=0,last1=0,last2=0,frames1=0,frames2=0;
 reg h1=0,h2=0,v1=0,v2=0;
 always @(posedge pclk1 or negedge reset1_n) begin
  if(!reset1_n)begin pc1<=0;len1<=0;last1<=0;frames1<=0;h1<=0;v1<=0;end
  else begin
   pc1<=pc1+1'b1;h1<=href1;v1<=vsync1;
   if(href1)len1<=len1+1'b1;else len1<=0;
   if(h1&&!href1)last1<=len1;
   if(vsync1&&!v1)frames1<=frames1+1'b1;
  end
 end
 always @(posedge pclk2 or negedge reset2_n) begin
  if(!reset2_n)begin pc2<=0;len2<=0;last2<=0;frames2<=0;h2<=0;v2<=0;end
  else begin
   pc2<=pc2+1'b1;h2<=href2;v2<=vsync2;
   if(href2)len2<=len2+1'b1;else len2<=0;
   if(h2&&!href2)last2<=len2;
   if(vsync2&&!v2)frames2<=frames2+1'b1;
  end
 end
 wire [63:0] camera1_snapshot,camera2_snapshot;
 wire snapshot1_valid,snapshot2_valid;
 imu_bundle_cdc #(.WIDTH(64)) u_cam1_snapshot(
  .src_clk(pclk1),.src_rst_n(reset_n),.src_valid(1'b1),.src_ready(),
  .src_data({pc1,last1,frames1}),.dst_clk(clk),.dst_rst_n(reset_n),
  .dst_valid(snapshot1_valid),.dst_ready(1'b1),.dst_data(camera1_snapshot));
 imu_bundle_cdc #(.WIDTH(64)) u_cam2_snapshot(
  .src_clk(pclk2),.src_rst_n(reset_n),.src_valid(1'b1),.src_ready(),
  .src_data({pc2,last2,frames2}),.dst_clk(clk),.dst_rst_n(reset_n),
  .dst_valid(snapshot2_valid),.dst_ready(1'b1),.dst_data(camera2_snapshot));
 (* ASYNC_REG="TRUE" *) reg [2:0] init_meta=0,init_sync=0;
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin init_meta<=0;init_sync<=0;end
  else begin init_meta<=init_done;init_sync<=init_meta;end
 reg [63:0] sample1=0,sample2=0;
 always @(posedge clk or negedge reset_n)
  if(!reset_n)begin sample1<=0;sample2<=0;end
  else begin
   if(snapshot1_valid)sample1<=camera1_snapshot;
   if(snapshot2_valid)sample2<=camera2_snapshot;
  end
 reg [31:0] timer=0;
 reg [8*24-1:0] packet=0;
 reg [15:0] baud=0;
 reg [4:0] index=0;
 reg [3:0] bitno=0;
 reg busy=0;
 wire [7:0] octet=packet[191:184];
 always @(posedge clk)begin
  if(!reset_n)begin timer<=0;busy<=0;uart_tx<=1;end
  else if(!busy)begin
   uart_tx<=1;
   if(timer==PERIOD-1)begin
    timer<=0;packet<={16'h4341,5'b0,init_sync,sample1,sample2,af1,af2,8'h0a};
    busy<=1;baud<=0;bitno<=0;index<=0;
   end else timer<=timer+1'b1;
  end else if(baud==0)begin
   baud<=BAUD_DIV-1;
   if(bitno==0)uart_tx<=0;
   else if(bitno<9)uart_tx<=octet[bitno-1];
   else uart_tx<=1;
   if(bitno==9)begin
    bitno<=0;packet<=packet<<8;
    if(index==23)busy<=0;else index<=index+1'b1;
   end else bitno<=bitno+1'b1;
  end else baud<=baud-1'b1;
 end
endmodule
