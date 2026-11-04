/////////////////////////////////////////////////////////////////////////////////
// 厂商       : 武汉芯路恒科技有限公司
// 资料地址：http://xiaomeige.taobao.com
// 网站：http://www.corecourse.cn
// 

// 模块：ov5640_init_table_rgb
// 功能   : OV5640初始化寄存器表(RGB模式专用)
// 
// 依赖：
// 
// 版本：
// 版本0.01：建立文件。
// 补充说明：
// 
/////////////////////////////////////////////////////////////////////////////////

module ov7725_init_table_rgb#(
  parameter DATA_WIDTH   = 16,
  parameter ADDR_WIDTH   = 8,
  parameter IMAGE_WIDTH  = 16'd640,
  parameter IMAGE_HEIGHT = 16'd480,
  parameter IMAGE_FLIP_EN   = 1'b1,
  parameter IMAGE_MIRROR_EN = 1'b1
)
(
  addr,
  clk, 
  q
);
  input clk;
  input [(ADDR_WIDTH-1):0] addr;
  output reg [(DATA_WIDTH-1):0] q;

  // 声明ROM变量。
  reg [DATA_WIDTH-1:0] rom [2**ADDR_WIDTH-1:0];

  initial begin
    /*OV7725 VGA RGB565参数  */
    // 写入数据索引。
    rom[0 ] = 16'h12_80; // 位7复位所有寄存器。
    rom[1 ] = 16'h3d_03; // 模拟处理直流偏移。
    rom[2 ] = 16'h15_02; // COM10控制HREF、VSYNC、PCLK及数据反向，VSYNC高电平有效。
    rom[3 ] = 16'h17_22; // VGA配置为16'h22，QVGA为16'h3f。
    rom[4 ] = 16'h18_a4; // VGA配置为16'ha4，QVGA为16'h50。
    rom[5 ] = 16'h19_07; // VGA配置为16'h07，QVGA为16'h03。
    rom[6 ] = 16'h1a_f0; // VGA配置为16'hf0，QVGA为16'h78。
    rom[7 ] = 16'h32_00; // HREF配置值16'h80。
    rom[8 ] = 16'h29_A0; // VGA配置为16'hA0，QVGA为16'hF0。
    rom[9 ] = 16'h2C_F0; // VGA配置为16'hF0，QVGA为16'h78。
    rom[10] = 16'h0d_41; // 旁路PLL。
    rom[11] = 16'h11_01; // 内部时钟为输入时钟乘PLL倍频，再除以(CLKRC[5:0]+1)×2，即25MHz×4/[(x+1)×2]。
                         // 50Hz频闪滤波时，00对应50帧每秒，01为25帧每秒，03为12.5帧每秒。
    rom[12] = 16'h12_06; // 位6选择尺寸，0为VGA，1为QVGA。
                         // 位3至2取01表示RGB565。
                         // 格式00为YUV，01为处理后Bayer RGB，10为RGB，11为Bayer RAW；位7复位全部寄存器。
    // 原配置示例：rom[13]=16'h0C_10；COM3位7至6分别控制垂直和水平镜像，位0控制彩条，默认16'h10。
    rom[13] = {8'h0c, IMAGE_MIRROR_EN[0], IMAGE_FLIP_EN[0], 2'd1, 4'd0};
    // DSP控制。
    rom[14] = 16'h42_7f; // 黑电平蓝通道目标值，默认16'h80。
    rom[15] = 16'h4d_09; // 黑电平红通道目标值，默认16'h80。
    rom[16] = 16'h63_f0; // 自动白平衡控制。
    rom[17] = 16'h64_ff; // DSP控制寄存器1。
    rom[18] = 16'h65_00; // DSP控制寄存器2。
    rom[19] = 16'h66_00; // COM3[4]与DSP_Ctrl3[7]组合选择字节序：00为YUYV、01为YVYU、10为UYVY、11为VYUY。
    rom[20] = 16'h67_00; // DSP_Ctrl4选择格式：00或01为YUV/RGB，10为RAW8，11为RAW10。
    // 字段：AGC AEC AWB
    rom[21] = 16'h13_ff;
    rom[22] = 16'h0f_c5;
    rom[23] = 16'h14_11;
    rom[24] = 16'h22_98; // 频闪滤波的最小自动曝光值，默认16'h09。
    rom[25] = 16'h23_03; // 频闪滤波最大步长。
    rom[26] = 16'h24_40; // 自动增益及曝光稳定工作区上限。
    rom[27] = 16'h25_30; // 自动增益及曝光稳定工作区下限。
    rom[28] = 16'h26_a1; // 自动增益及曝光快速模式工作区。
    rom[29] = 16'h2b_9e; // 60Hz频闪滤波配置16'h00，50Hz配置16'h9e。
    rom[30] = 16'h6b_aa; // 自动白平衡控制寄存器3。
    rom[31] = 16'h13_ff; // 16'hff启用自动增益、曝光和白平衡，16'hf0关闭三者。
    
    // 色彩矩阵、锐度、亮度、对比度及UV参数。
    rom[32] = 16'h90_0a;  
    rom[33] = 16'h91_01;
    rom[34] = 16'h92_01;
    rom[35] = 16'h93_01;
    rom[36] = 16'h94_5f;
    rom[37] = 16'h95_53;
    rom[38] = 16'h96_11;
    rom[39] = 16'h97_1a;
    rom[40] = 16'h98_3d;
    rom[41] = 16'h99_5a;
    rom[42] = 16'h9a_1e;
    rom[43] = 16'h9b_3f; // 字段：Brightness
    rom[44] = 16'h9c_25;
    rom[45] = 16'h9e_81;  
    rom[46] = 16'ha6_06;
    rom[47] = 16'ha7_65;
    rom[48] = 16'ha8_65;
    rom[49] = 16'ha9_80;
    rom[50] = 16'haa_80;
    // 伽玛校正。
    rom[51] = 16'h7e_0c;
    rom[52] = 16'h7f_16; //
    rom[53] = 16'h80_2a;
    rom[54] = 16'h81_4e;
    rom[55] = 16'h82_61;
    rom[56] = 16'h83_6f;
    rom[57] = 16'h84_7b;
    rom[58] = 16'h85_86;
    rom[59] = 16'h86_8e;
    rom[60] = 16'h87_97;
    rom[61] = 16'h88_a4;
    rom[62] = 16'h89_af;
    rom[63] = 16'h8a_c5;
    rom[64] = 16'h8b_d7;
    rom[65] = 16'h8c_e8;
    rom[66] = 16'h8d_20;
    // 字段：Others
    rom[67] = 16'h0e_65; // 夜间模式自动帧率控制。
  end

  always @ (posedge clk)
  begin
    q <= rom[addr];
  end
  
endmodule
