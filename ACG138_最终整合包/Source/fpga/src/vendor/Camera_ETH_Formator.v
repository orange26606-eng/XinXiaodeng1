//-------------------------------


// 日期：2026-10-12 18:20
// 模块：Camera_ETH_Formator
// 功能: 相机PCLK域复位同步释放并两级同步初始化完成信号，像素数据路径不变。
// 版本：1.1.0
// 修订记录：保留下列历史条目。
/* 版本1.0，2019-04-10，初始发布。

 */
// 修订记录 ： v1.1.0----相机PCLK域复位同步释放并两级同步初始化完成信号，像素数据路径不变。
//-------------------------------

module Camera_ETH_Formator(
	PCLK,
	Rst_n,
    Init_Done,
	HREF,
	VSYNC,
	DATA,

	fifo_aclr,
	wrdata,
	wrreq
);

	input PCLK;
	input Rst_n;
	input Init_Done;
	input HREF;
	input VSYNC;
	input [7:0]DATA;
	output reg fifo_aclr;	
	output [7:0]wrdata;
	output reg wrreq;
	
    (* ASYNC_REG="TRUE" *) reg [1:0] pclk_reset_sync = 2'b00;
    (* ASYNC_REG="TRUE" *) reg [1:0] init_done_sync = 2'b00;
    always @(posedge PCLK or negedge Rst_n)
        if (!Rst_n) pclk_reset_sync <= 2'b00;
        else pclk_reset_sync <= {pclk_reset_sync[0],1'b1};
    wire pclk_rst_n = pclk_reset_sync[1];
    always @(posedge PCLK or negedge pclk_rst_n)
        if (!pclk_rst_n) init_done_sync <= 2'b00;
        else init_done_sync <= {init_done_sync[0],Init_Done};

	reg [23:0]data_tmp;
	
	reg [15:0]Vcnt;
		
	reg href_r1;
	reg href_r2;
	
	assign wrdata = data_tmp[23:16];
	
	always@(posedge PCLK)begin
		href_r1 <= HREF;
		href_r2 <= href_r1;	
	end
	
    always @ (posedge PCLK or negedge pclk_rst_n)
	if (!pclk_rst_n)
		fifo_aclr <= #1 1'b1;
	else if (init_done_sync[1] && VSYNC ) //等到初始化摄像完成且头场同步信号出现，释放清零信号，开始写入数据
		fifo_aclr <= #1 1'b0;
	else
		fifo_aclr <= #1 fifo_aclr;
	
	always@(posedge PCLK or negedge pclk_rst_n)
	if(!pclk_rst_n)
		data_tmp <= 0;
	else if({href_r1,HREF} == 2'b01)
		data_tmp <= {Vcnt[7:0],Vcnt[15:8],DATA};
	else 
		data_tmp <= {data_tmp[15:0],DATA};
	
	always@(posedge PCLK)
	if({href_r1,HREF} == 2'b01)
		wrreq <= 1;
	else if(href_r2 | href_r1) 
		wrreq <= 1;
	else
		wrreq <= 0;

	always@(posedge PCLK or negedge pclk_rst_n)
	if(!pclk_rst_n)
		Vcnt <= 0;
	else if(VSYNC)
		Vcnt <= 0;
	else if({href_r1,HREF} == 2'b10)
		Vcnt <= Vcnt + 1'd1;

endmodule
