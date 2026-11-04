
// 增加认证代理电机命令通路和独立500ms看门狗。


// 日期：2026-10-12 18:26
// 模块：ov5640x2_udp_rgmii
// 功能: 双目RAW行包和IMU姿态及控制回执共用完整帧RGMII仲裁。
// 版本：1.2.3
// 版本0.01：建立文件。
// 版本1.1.0，2026-10-11：在单一RGMII输出前加入IMU I2C、UDP发送及整帧仲裁。
// 修订记录：v1.2.0，IIC控制使用50MHz时钟，命令接收及回执与角度共用发送器按整帧仲裁。
//                   v1.2.1----相机配置域同步释放复位并导出同域诊断时钟，保留原图像和IMU业务接口。
//                   v1.2.2----相机就绪状态寄存器使用TX域同步复位，消除真实复位恢复违例。
//                   v1.2.3----将初始化状态声明移到首次引用之前，兼容严格Verilog解析器，逻辑不变。

module ov5640x2_udp_rgmii(
 nav_rx_en,nav_rx_er,nav_rx_data,nav_status_valid,nav_status_ready,nav_status_data,
 remote_valid,remote_ready,remote_data,remote_reset_n,
 robot_snapshot,imu_calibration_wait,odom_positions,odom_ready,odom_reset_key,odom_ab,odom_invalid,
 diag_clk,diag_reset_n,
 af_status1,af_status2,
	camera_status,
    imu_sda, imu_scl, imu_led,
    rgmii_rx_clk, rgmii_rx_ctl, rgmii_rxd,
	clk,
	rst_n,
	
	eth_gtxc,
	eth_txd,
	eth_txen,

	eth_mdc,
	eth_mdio,
	eth_rst_n,

    led,
	
	camera1_sclk,
	camera1_sdat,
	camera1_vsync,
	camera1_href,
	camera1_pclk,
	camera1_xclk,
	camera1_data,
	camera1_rst_n,
	camera1_pwdn,
	
	
	camera2_sclk,
	camera2_sdat,
	camera2_vsync,
	camera2_href,
	camera2_pclk,
	camera2_xclk,
	camera2_data,
	camera2_rst_n,
	camera2_pwdn
);
 output remote_valid,remote_reset_n; input remote_ready; output [79:0] remote_data;
 input [111:0] robot_snapshot;
 input [7:0] odom_ab;input [31:0] odom_invalid;
 input [127:0] odom_positions;input [3:0] odom_ready;input odom_reset_key;
 output imu_calibration_wait;
 output diag_clk,diag_reset_n;
 output [15:0] af_status1,af_status2;
	output [2:0] camera_status;
    input rgmii_rx_clk,rgmii_rx_ctl;
    input [3:0] rgmii_rxd;
    inout imu_sda, imu_scl;
    output [3:0] imu_led;
 wire pair_fault;
	wire Init_Done1;
	wire Init_Done2;
	wire Init_Done = (Init_Done1 && Init_Done2);
	assign camera_status = {pair_fault,Init_Done2,Init_Done1};
 output nav_rx_en,nav_rx_er;output[7:0]nav_rx_data;
 input nav_status_valid;output nav_status_ready;input[767:0]nav_status_data;

	input clk;
	input rst_n;
	
	output eth_gtxc;
	output [3:0] eth_txd;
	output eth_txen;
	inout eth_mdc;
	inout eth_mdio;
	output eth_rst_n;
    output led;
	assign led = Init_Done;
	// 相机1接口。
	output		camera1_sclk;
	inout		camera1_sdat;
	input		camera1_vsync;
	input		camera1_href;
	input		camera1_pclk;
	output		camera1_xclk;
	input [7:0]	camera1_data;
	output		camera1_rst_n;
	output		camera1_pwdn;
	
	// 相机2接口。
	output		camera2_sclk;
	inout		camera2_sdat;
	input		camera2_vsync;
	input		camera2_href;
	input		camera2_pclk;
	output		camera2_xclk;
	input [7:0]	camera2_data;
	output		camera2_rst_n;
	output		camera2_pwdn;
	
	wire clk_125m;
	wire clk_50m;
	wire clk_24m;
	wire locked;
	wire global_rst_n = locked && rst_n;
    // GMII与IMU时钟域复位异步进入、同步释放。
    (* ASYNC_REG="TRUE" *) reg [1:0] cfg_reset_sync = 2'b00;
    always @(posedge clk_50m or negedge global_rst_n)
        if (!global_rst_n) cfg_reset_sync <= 2'b00;
        else cfg_reset_sync <= {cfg_reset_sync[0],1'b1};
    assign remote_reset_n=global_rst_n;
    wire cfg_rst_n = cfg_reset_sync[1];
    assign diag_clk = clk_50m;
    assign diag_reset_n = cfg_rst_n;
    (* ASYNC_REG="TRUE" *) reg [1:0] tx_reset_sync = 2'b00;
    (* ASYNC_REG="TRUE" *) reg [1:0] camera_ready_sync = 2'b00;
    always @(posedge clk_125m or negedge global_rst_n) begin
        if (!global_rst_n) tx_reset_sync<=0;
        else tx_reset_sync <= {tx_reset_sync[0],1'b1};
    end
    wire tx_rst_n = tx_reset_sync[1];
    // 电平同步器属于普通数据逻辑，需等发送域
    // 复位同步器完成后再释放其复位。
    always @(posedge clk_125m or negedge tx_rst_n) begin
        if (!tx_rst_n) camera_ready_sync<=0;
        else camera_ready_sync <= {camera_ready_sync[0],Init_Done};
    end
    wire camera_tx_rst_n = tx_rst_n && camera_ready_sync[1];
	assign camera1_xclk = clk_24m;
	assign camera2_xclk = clk_24m;

    Gowin_PLL your_instance_name(
        .clkin(clk), // 输入接口：clkin
        .clkout0(clk_125m), // 输出接口：clkout0
        .clkout1(clk_50m), // 输出接口：clkout1
        .clkout2(clk_24m), // 输出接口：clkout2
        .lock(locked), // 输出接口：lock
        .mdclk(clk), // 输入接口：mdclk
        .reset(~rst_n) // 输入接口：reset
    );
	
	wire phy_init;	// PHY初始化完成。
	wire Rd_Data;
	phy_reg_config phy_reg_config(
		.Clk       			(clk_50m       	),
		.Rst_n     			(cfg_rst_n     	),
		.Phy_rst_n 			(eth_rst_n    	),
		.mdc       			(eth_mdc    	), 
		.mdio      			(eth_mdio   	),
		.Rd_Data  			(Rd_Data      	),
		.Phy_init_done		(phy_init      	)
	);

	assign camera1_pwdn = 1'b0;
	assign camera2_pwdn = 1'b0;
	
	localparam RGB = 0;
	localparam JPEG = 1;
	
	parameter IMAGE_WIDTH  = 960; //图片宽度
	parameter IMAGE_HEIGHT = 1080; //图片高度(≤720)
	parameter IMAGE_FLIP   = 0;   //0：不翻转，1：上下翻转
	parameter IMAGE_MIRROR = 0;   //0：不镜像，1：左右镜像
	//摄像头1,摄像头2初始化配置

	camera_init
	#(
		.CAMERA_TYPE		("ov5640"		),
		.IMAGE_TYPE         (0              ),
		.IMAGE_WIDTH		(IMAGE_WIDTH	),
		.IMAGE_HEIGHT		(IMAGE_HEIGHT	),
		.IMAGE_FLIP_EN		(1	            ),
		.IMAGE_MIRROR_EN	(0             	)
	)
	camera_init1(
 .af_status(af_status1),
		.Clk				(clk_50m		),
		.Rst_n				(cfg_rst_n	),
		.Init_Done			(Init_Done1		),
		.camera_rst_n 		(camera1_rst_n  ),
		.i2c_sclk			(camera1_sclk	),
		.i2c_sdat			(camera1_sdat	)
	),
	camera_init2(
 .af_status(af_status2),
		.Clk				(clk_50m		),
		.Rst_n				(cfg_rst_n	),
		.Init_Done			(Init_Done2		),
		.camera_rst_n   	(camera2_rst_n  ),
		.i2c_sclk			(camera2_sclk	),
		.i2c_sdat			(camera2_sdat	)
	);
	
	//摄像头1,摄像头2图像输出
	wire fifo1_aclr;
	wire fifo1_wrreq;
	wire [7:0] fifo1_wrdata;
	wire fifo2_aclr;
	wire fifo2_wrreq;
	wire [7:0] fifo2_wrdata;
	Camera_ETH_Formator Camera_ETH_Formator1(
		.PCLK		    	(camera1_pclk   ),
		.Rst_n				(global_rst_n     	),
		.Init_Done			(Init_Done		),
		.HREF	     		(camera1_href   ),
		.VSYNC		    	(camera1_vsync	),
		.DATA			    (camera1_data	),
		.fifo_aclr			(fifo1_aclr		),
		.wrdata	            (fifo1_wrdata	),
		.wrreq		     	(fifo1_wrreq	)
	),
	Camera_ETH_Formator2(
		.PCLK			    (camera2_pclk   ),
		.Rst_n				(global_rst_n     	),
		.Init_Done			(Init_Done		),
		.HREF       		(camera2_href	),
		.VSYNC      		(camera2_vsync	),
		.DATA       		(camera2_data	),
		.fifo_aclr			(fifo2_aclr		),
		.wrdata	        	(fifo2_wrdata	),
		.wrreq	    		(fifo2_wrreq	)
	);

	wire fifo_aclr;
	wire fifo_wrreq;
	wire [7:0] fifo_wrdata;

	wire wrclk;// 以太网FIFO写时钟。
	wire GMII_GTXC;
	assign GMII_GTXC = clk_125m;

	
	controller_top controller_top(
        .pair_fault(pair_fault),
        .clk(GMII_GTXC),
        .rst_n(global_rst_n),
        
        .wrclk1(camera1_pclk),
        .fifo1_aclr(fifo1_aclr),
        .wrdata1(fifo1_wrdata),
        .wrreq1(fifo1_wrreq),
        
        .wrclk2(camera2_pclk),
        .fifo2_aclr(fifo2_aclr),
        .wrdata2(fifo2_wrdata),
        .wrreq2(fifo2_wrreq),
        
        .wrclk(wrclk),
        .fifo_aclr(fifo_aclr),
        .fifo_wrreq(fifo_wrreq),
        .fifo_wrdata(fifo_wrdata)
    );
	
	
	wire [7:0] GMII_TXD;
	wire GMII_TXEN;
    wire cam_pending, cam_grant, imu_grant;
    wire imu_pending;
    wire imu_gmii_txen, merged_gmii_txen;
    wire [7:0] imu_gmii_txd, merged_gmii_txd;
    mpu9250_full_link u_mpu9250(
        .clk50(clk),.clk125(clk_125m),.rx_clk(rgmii_rx_clk),.rst_n(global_rst_n),
        .reset_key_n(odom_reset_key),.positions(odom_positions),.enc_ready(odom_ready),
        .enc_ab(odom_ab),.enc_invalid(odom_invalid),
        .rgmii_rxd(rgmii_rxd),.rgmii_rx_ctl(rgmii_rx_ctl),
        .tap_en(nav_rx_en),.tap_er(nav_rx_er),.tap_data(nav_rx_data),
        .remote_valid(remote_valid),.remote_ready(remote_ready),.remote_data(remote_data),
        .sda(imu_sda),.scl(imu_scl),.led(imu_led),
        .tx_grant(imu_grant),.pending(imu_pending),.txen(imu_gmii_txen),.txd(imu_gmii_txd));
    // RK执行标定与解算，采集模块独立于电机控制。
    assign imu_calibration_wait=1'b0;
    // S7复位电机数据，CDC邮箱保持统一复位关系。

    (* ASYNC_REG="TRUE" *) reg [1:0] robot_source_reset_sync=0;
    always @(posedge clk or negedge global_rst_n)
        if(!global_rst_n)robot_source_reset_sync<=0;
        else robot_source_reset_sync<={robot_source_reset_sync[0],1'b1};
    reg [23:0] robot_report_count;
    wire robot_sample=robot_report_count==24'd9999999;
    always @(posedge clk or negedge robot_source_reset_sync[1])
        if(!robot_source_reset_sync[1])robot_report_count<=0;
        else if(robot_sample)robot_report_count<=0;
        else robot_report_count<=robot_report_count+1'b1;
    wire robot_mail_valid,robot_ready,robot_grant,robot_pending,robot_txen;
    wire [111:0] robot_mail_data;
    wire [7:0] robot_txd;
    imu_bundle_cdc #(.WIDTH(112)) u_robot_mailbox(
        .src_clk(clk),.src_rst_n(global_rst_n),.src_valid(robot_sample),.src_ready(),
        .src_data(robot_snapshot),.dst_clk(clk_125m),.dst_rst_n(global_rst_n),
        .dst_valid(robot_mail_valid),.dst_ready(robot_grant && robot_ready),.dst_data(robot_mail_data));
    assign robot_pending=robot_mail_valid && robot_ready;
    robot_udp_message_tx_gmii u_robot_tx(
        .clk125m(clk_125m),.rst_n(tx_rst_n),.sample_valid(robot_mail_valid && robot_grant),
        .sample_ready(robot_ready),.sample_data(robot_mail_data),
        .gmii_txen(robot_txen),.gmii_txd(robot_txd),.tx_done());
    wire odom_txen,odom_pending,odom_grant;
    wire[7:0]odom_txd;wire nav_mail_valid,nav_tx_ready;wire[767:0]nav_mail_data;
    imu_bundle_cdc #(.WIDTH(768)) nav_mail(.src_clk(clk),.src_rst_n(global_rst_n),
     .src_valid(nav_status_valid),.src_ready(nav_status_ready),.src_data(nav_status_data),
     .dst_clk(clk_125m),.dst_rst_n(global_rst_n),.dst_valid(nav_mail_valid),
     .dst_ready(odom_grant&&nav_tx_ready),.dst_data(nav_mail_data));
    assign odom_pending=nav_mail_valid&&nav_tx_ready;
    nav_udp_tx nav_sender(.clk125m(clk_125m),.rst_n(tx_rst_n),.sample_valid(odom_grant&&odom_pending),
     .sample_ready(nav_tx_ready),.sample_data(nav_mail_data),.gmii_txen(odom_txen),.gmii_txd(odom_txd),.tx_done());
    gmii_odom_frame_arbiter arbiter(
      .odom_pending(odom_pending),.odom_txen(odom_txen),.odom_txd(odom_txd),.odom_grant(odom_grant),
        .clk125m(clk_125m), .rst_n(tx_rst_n),
        .cam_pending(cam_pending), .cam_grant(cam_grant),
        .cam_txen(GMII_TXEN), .cam_txd(GMII_TXD),
        .robot_pending(robot_pending),.robot_grant(robot_grant),
        .robot_txen(robot_txen),.robot_txd(robot_txd),
        .imu_pending(imu_pending), .imu_grant(imu_grant),
        .imu_txen(imu_gmii_txen), .imu_txd(imu_gmii_txd),
        .gmii_txen(merged_gmii_txen), .gmii_txd(merged_gmii_txd));
	UDP_Send UDP_Send(
		.Clk				(				),
		.GMII_GTXC			(GMII_GTXC		),
		.GMII_TXD			(GMII_TXD		),
		.GMII_TXEN			(GMII_TXEN		),
		.Rst_n				(camera_tx_rst_n),
        .tx_grant(cam_grant), .tx_pending(cam_pending),
		.Tx_Done			(				),
		.data_length		(IMAGE_WIDTH*2+2		),
		.des_ip				(32'hc0_a8_00_03),//hc0_a8_00_03; 192.168.0.3
		.des_mac			(48'hFF_FF_FF_FF_FF_FF),
		.des_port			(16'd6102		),
		.src_ip				(32'hc0_a8_00_02),//hc0_a8_65_02; 192.168.0.2
		.src_mac			(48'h00_0a_35_01_fe_c0),
		.src_port			(16'd5000		),
		// 接口或逻辑示例：.wrclk			(camera_pclk		),
		.wrclk				(wrclk			),
		.wrdata				(fifo_wrdata	),
		.wrreq				(fifo_wrreq		),
		.aclr				(fifo_aclr		),
		.wrusedw			( )
	);
	
	gmii_to_rgmii gmii_to_rgmii(
	    .reset_n            (tx_rst_n       ),
		.gmii_tx_clk		(GMII_GTXC		),
		.gmii_txd 			(merged_gmii_txd),
		.gmii_txen			(merged_gmii_txen),
		.gmii_txer          (     0         ),
		
		.rgmii_tx_clk		(eth_gtxc   	),
		.rgmii_txd 			(eth_txd		),
		.rgmii_txen			(eth_txen		)
	);
	
endmodule
