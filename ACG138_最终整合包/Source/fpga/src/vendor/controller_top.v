//-------------------------------


// 日期：2026-10-12 18:20
// 模块：controller_top
// 功能: 配对控制器复位异步拉低并在125MHz域两级同步释放。
// 版本：1.1.0
// 修订记录 ： v1.1.0----配对控制器复位异步拉低并在125MHz域两级同步释放。
//-------------------------------
module controller_top(
        pair_fault,
        clk,
        rst_n,
        
        wrclk1,
        fifo1_aclr,
        wrdata1,
        wrreq1,
        
        wrclk2,
        fifo2_aclr,
        wrdata2,
        wrreq2,
        
        wrclk,
        fifo_aclr,
        fifo_wrreq,
        fifo_wrdata
    );
    
    output pair_fault;
    input clk;
    input rst_n;
    
    input wrclk1;
    input fifo1_aclr;
    input [7:0]wrdata1;
    input wrreq1;
        
    input wrclk2;
    input fifo2_aclr;
    input [7:0]wrdata2;
    input wrreq2;
        
    output wrclk;
    output fifo_aclr;
    output fifo_wrreq;
    output [7:0]fifo_wrdata;
    
    wire cache_rdclk;
    wire cache1_rdreq;
    wire cache2_rdreq;
    wire [7:0] cache1_rddata;
    wire [7:0] cache2_rddata;
    wire [13:0] rdusedw1;
    wire [13:0] rdusedw2;
    
    
	cache cache1(
		.wrclk				(wrclk1     	),
		.aclr				(fifo1_aclr		),
		.wrreq				(wrreq1     	),
		.wrdata				(wrdata1    	),
		.rdclk				(cache_rdclk	),
		.rdreq				(cache1_rdreq	),
		.rddata				(cache1_rddata	),
		.rdusedw			(rdusedw1		)
	);
	
	cache cache2(
		.wrclk				(wrclk2      	),
		.aclr				(fifo2_aclr		),
		.wrreq				(wrreq2     	),
		.wrdata				(wrdata2     	),
		.rdclk				(cache_rdclk	),
		.rdreq				(cache2_rdreq	),
		.rddata				(cache2_rddata	),
		.rdusedw			(rdusedw2		)
	);
	
    // 共同进入复位，仅在接收侧125MHz时钟同步释放。
    // 两路相机FIFO复位分别属于独立PCLK时钟域。
    wire controller_async_rst_n = rst_n & !fifo1_aclr & !fifo2_aclr;
    (* ASYNC_REG="TRUE" *) reg [1:0] controller_reset_sync = 2'b00;
    always @(posedge clk or negedge controller_async_rst_n)
        if (!controller_async_rst_n) controller_reset_sync <= 2'b00;
        else controller_reset_sync <= {controller_reset_sync[0],1'b1};

	controller controller(
        .pair_fault(pair_fault),
		.clk				(clk  	     	),
		.rst_n				(controller_reset_sync[1]),
		.rdclk				(cache_rdclk	),
		.rdreq1				(cache1_rdreq	),
		.rddata1			(cache1_rddata	),
		.rdusedw1			(rdusedw1		),
		.rdreq2				(cache2_rdreq	),
		.rddata2			(cache2_rddata	),
		.rdusedw2			(rdusedw2		),
		.wrclk				(wrclk			),
		.fifo_aclr			(fifo_aclr		),
		.fifo_wrreq			(fifo_wrreq		),
		.fifo_wrdata		(fifo_wrdata	)
	);

endmodule
