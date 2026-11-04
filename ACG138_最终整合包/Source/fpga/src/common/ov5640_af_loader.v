// OV5640内部对焦MCU固件加载器，对焦失败时图像仍保持启用。
// 状态字段为stage[3:0]、error[3:0]及last_read[7:0]。
// 错误1为NACK，2为事务或空闲超时，3为器件编号错误，4为轮询或监测超时。
// 总线空闲并经过冷却后自动重试，保留图像初始化状态。
// 阶段F且错误0表示单次对焦完成，连续对焦命令已确认。
module ov5640_af_loader #(
 parameter FW_FILE="src/common/ov5640_af.hex",
 parameter FW_BYTES=4077,
 parameter WAIT_CYCLES=500000,
 parameter TIMEOUT_CYCLES=5000000,
 parameter MAX_POLLS=500,
 parameter RETRY_CYCLES=5000000,
 parameter SLOW_RETRY_CYCLES=50000000,
 parameter QUIET_CYCLES=50000,
 parameter MONITOR_CYCLES=50000000,
 parameter MONITOR_BAD_LIMIT=3
)(input clk,input rst_n,input start,
 output reg wr_req,output reg rd_req,output reg [15:0]addr,
 output reg [7:0]wrdata,input [7:0]rddata,input rw_done,input nack,
 output [15:0]status,input i2c_idle,output reg bus_reset_n);
 reg [7:0]rom[0:4076];
 initial $readmemh(FW_FILE,rom);
 reg [11:0]index;
 reg [7:0]rom_q;
 always @(posedge clk)rom_q<=rom[index];
 reg [3:0]stage,error;
 reg [7:0]last_read;
 reg [1:0]phase;
 reg [31:0]timer;
 reg [9:0]polls;
 reg running;
 reg[1:0]retries;reg[31:0]recover_count,quiet_count,monitor_count;
 reg[3:0]monitor_bad;

 assign status={stage,error,last_read};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   stage<=0;error<=0;last_read<=0;phase<=0;timer<=0;
   index<=0;polls<=0;running<=0;wr_req<=0;rd_req<=0;addr<=0;wrdata<=0;
   bus_reset_n<=1;retries<=0;recover_count<=0;quiet_count<=0;monitor_count<=0;monitor_bad<=0;
  end else begin
   wr_req<=0;rd_req<=0;
   if(start)running<=1;
   if(running&&error!=0)begin
    // I2C事务超时后，先排空或复位该事务再重试。
    // 此复位仅作用于对焦总线，不复位图像初始化或传感器。
    if(error==2&&recover_count<16)bus_reset_n<=0;else bus_reset_n<=1;
    if(recover_count<(retries<3?RETRY_CYCLES:SLOW_RETRY_CYCLES))recover_count<=recover_count+1;
    if(i2c_idle&&!rw_done&&bus_reset_n)begin if(quiet_count<QUIET_CYCLES)quiet_count<=quiet_count+1;end
    else quiet_count<=0;
    if(recover_count>=(retries<3?RETRY_CYCLES:SLOW_RETRY_CYCLES)&&quiet_count>=QUIET_CYCLES)begin
     if(retries<3)retries<=retries+1'b1;
     // 原位重试幂等写入；单次对焦轮询失败时，
     // 沿原释放及连续对焦序列继续，避免
     // 反复全程扫焦。固件启动或编号错误则重新加载。
     if(error==3||(error==4&&stage==6))begin stage<=0;index<=0;end
     else if(error==4&&stage==8)stage<=9;
     else if(error==4&&(stage==11||stage==14||stage==15))stage<=12;
     error<=0;phase<=0;timer<=0;polls<=0;recover_count<=0;quiet_count<=0;monitor_count<=0;monitor_bad<=0;
    end
   end else if(running&&stage==15)begin
    // 连续对焦保持运行，只读已有ACK寄存器，
    // 每秒检查一次，正常时不再发送单次对焦命令。
    case(phase)
     0:if(monitor_count>=MONITOR_CYCLES)begin
       if(i2c_idle&&!rw_done)begin addr<=16'h3023;rd_req<=1;phase<=1;timer<=0;monitor_count<=0;end
       else begin timer<=timer+1;if(timer==TIMEOUT_CYCLES-1)begin error<=2;recover_count<=0;quiet_count<=0;end end
      end else monitor_count<=monitor_count+1;
     1:begin
      timer<=timer+1;
      if(timer==TIMEOUT_CYCLES-1)begin error<=2;recover_count<=0;quiet_count<=0;end
      else if(rw_done)begin
       last_read<=rddata;phase<=0;timer<=0;
       if(nack)begin error<=1;recover_count<=0;quiet_count<=0;end
       else if(rddata!=0)begin
        if(monitor_bad>=MONITOR_BAD_LIMIT-1)begin error<=4;recover_count<=0;quiet_count<=0;end
        else monitor_bad<=monitor_bad+1'b1;
       end else begin monitor_bad<=0;retries<=0;end
      end
     end
     default:begin phase<=0;timer<=0;end
    endcase
   end else if(running && error==0)begin
    bus_reset_n<=1;recover_count<=0;quiet_count<=0;
    case(phase)
     0:begin
      if(!i2c_idle||rw_done)begin
       timer<=timer+1;
       if(timer==TIMEOUT_CYCLES-1)begin error<=2;recover_count<=0;quiet_count<=0;end
      end else begin
      timer<=0;phase<=1;
      case(stage)
       0:begin rd_req<=1;addr<=16'h300a;end
       1:begin rd_req<=1;addr<=16'h300b;end
       2:begin wr_req<=1;addr<=16'h3000;wrdata<=8'h20;end
       3:begin wr_req<=1;addr<=16'h8000+index;wrdata<=rom_q;end
       4:begin wr_req<=1;addr<=16'h3022+index;wrdata<=index==7?8'h7f:0;end
       5:begin wr_req<=1;addr<=16'h3000;wrdata<=0;end
       6:begin rd_req<=1;addr<=16'h3029;end
       7:begin wr_req<=1;addr<=16'h3022;wrdata<=3;end
       8:begin rd_req<=1;addr<=16'h3029;end
       9,12:begin wr_req<=1;addr<=16'h3023;wrdata<=1;end
       10:begin wr_req<=1;addr<=16'h3022;wrdata<=8;end
       11,14:begin rd_req<=1;addr<=16'h3023;end
       13:begin wr_req<=1;addr<=16'h3022;wrdata<=4;end
      endcase
      end
     end
     1:begin
      timer<=timer+1;
      if(timer==TIMEOUT_CYCLES-1)error<=2;
      else if(rw_done)begin
       timer<=0;phase<=2;
       if(nack)error<=1;
       else begin
        if(stage==0 || stage==1 || stage==6 || stage==8 || stage==11 || stage==14)
         last_read<=rddata;
        case(stage)
         0:if(rddata!=8'h56)error<=3;else stage<=1;
         1:if(rddata!=8'h40)error<=3;else stage<=2;
         2:begin index<=0;stage<=3;end
         3:if(index==FW_BYTES-1)begin index<=0;stage<=4;end else index<=index+1;
         4:if(index==7)begin index<=0;stage<=5;end else index<=index+1;
         6,8,11,14:begin
          if((stage==6 && rddata==8'h70) || (stage==8 && rddata==8'h10) ||
             ((stage==11 || stage==14) && rddata==0))begin stage<=stage+1;polls<=0;end
          else if(polls==MAX_POLLS-1)error<=4;
          else polls<=polls+1;
         end
         default:stage<=stage+1;
        endcase
       end
      end
     end
     2:begin
      // 固件字节写入仅需短暂稳定等待，轮询间隔为10ms。
      timer<=timer+1;
      if(timer==((stage==6 || stage==8 || stage==11 || stage==14)?WAIT_CYCLES:3))begin
       timer<=0;phase<=0;
      end
     end
     default:phase<=0;
    endcase
    if(stage==14&&phase==1&&rw_done&&!nack&&rddata==0)begin phase<=0;monitor_count<=0;monitor_bad<=0;retries<=0;end
   end
  end
 end
endmodule

