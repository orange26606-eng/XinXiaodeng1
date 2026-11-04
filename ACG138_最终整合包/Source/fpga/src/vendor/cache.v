module cache(
	wrclk,
	aclr,
	wrreq,
	wrdata,
	rdclk,
	rdreq,
	rddata,
	rdusedw
);
	input wrclk;
	input aclr;
	input wrreq;
	input [7:0] wrdata;
	input rdclk;
	input rdreq;
	output [7:0] rddata;
	output [13:0] rdusedw;
	

	dc_fifo dc_fifo(
		.Data(wrdata), // 输入接口：[7:0] Data
		.Reset(aclr), // 输入接口：Reset
		.WrClk(wrclk), // 输入接口：WrClk
		.RdClk(rdclk), // 输入接口：RdClk
		.WrEn(wrreq), // 输入接口：WrEn
		.RdEn(rdreq), // 输入接口：RdEn
		.Wnum(), // 输出接口：[13:0] Wnum
		.Rnum(rdusedw), // 输出接口：[13:0] Rnum
		.Q(rddata), // 输出接口：[7:0] Q
		.Empty(), // 输出接口：Empty
		.Full() // 输出接口：Full
	);

endmodule
