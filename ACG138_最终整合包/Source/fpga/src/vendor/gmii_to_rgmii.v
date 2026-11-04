//////////////////////////////////////////////////////////////////////////////////
// 厂商: 武汉芯路恒科技有限公司
// 厂商资料：www.corecourse.cn
// 

// 设计名称。
// 模块：
// 工程名称。
// 功能: gmii转rgmii
// 
// 依赖：
// 
// 版本：
// 版本0.01：建立文件。
// 补充说明：
// 
//////////////////////////////////////////////////////////////////////////////////
module gmii_to_rgmii(
  reset_n,

  gmii_tx_clk,
  gmii_txd,
  gmii_txen,
  gmii_txer,

  rgmii_tx_clk,
  rgmii_txd,
  rgmii_txen
);



  input        reset_n;

  input        gmii_tx_clk;
  input  [7:0] gmii_txd;
  input        gmii_txen;
  input        gmii_txer;
  
  output       rgmii_tx_clk;
  output [3:0] rgmii_txd;
  output       rgmii_txen;

  generate
    genvar i;
        for(i=0;i<4;i=i+1)
        begin
        ODDR    U_ODDR_dq1
                (.Q0(rgmii_txd[i]), 
                 .Q1(),
                 .D0(gmii_txd[i]), 
                 .D1(gmii_txd[i+4]), 
                 .TX(1),
                 .CLK(gmii_tx_clk) 
                );
        end
  endgenerate

	ODDR    U_ODDR_en1
			(.Q0(rgmii_txen), 
             .Q1(),

             .D0(gmii_txen), 
             .D1(gmii_txen^gmii_txer), 
             .TX(1),

             .CLK(gmii_tx_clk) 
			);

	ODDR    U_ODDR_clk1
			(.Q0(rgmii_tx_clk), 
             .Q1(),

             .D0(1), 
             .D1(0), 
             .TX(1),

             .CLK(gmii_tx_clk) 
			);

endmodule
