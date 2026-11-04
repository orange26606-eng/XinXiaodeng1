`timescale 1ns/1ps
// MPU9250原始量程为±4g、±500度每秒，AK8963为16位100Hz；本采集模块不解算姿态。
// 轮询时间表示寄存器开始读取时刻，不是传感器DRDY边沿。
module mpu9250_capture #(
 parameter integer CLK_HZ=125000000,I2C_DIV=1250,
 parameter integer BOOT_CLKS=12500000,RESET_CLKS=12500000,MODE_CLKS=12500,
 parameter integer RETRY_CLKS=125000000,BUS_TIMEOUT=125000
)(input clk,input rst_n,input poll,input[31:0]time_us,inout sda,inout scl,
 output reg[111:0]ag_data,output reg[47:0]mag_data,output reg[23:0]asa,
 output reg[31:0]ag_time,mag_time,ag_sequence,mag_sequence,
 output reg mpu_ready,mag_ready,ag_valid,mag_valid,
 output reg[7:0]whoami,mag_id,last_error,output reg[31:0]errors,overflows,
 output[7:0]phase);
 localparam BOOT=0,IDENT=1,IDENT_DONE=2,RESET_MPU=3,RESET_WAIT=4,
 CFG=5,CFG_DONE=6,MAG_IDENT=7,MAG_IDENT_DONE=8,MAG_PD=9,MAG_FUSE=10,
 MAG_ASA=11,MAG_ASA_DONE=12,MAG_PD2=13,MAG_CONT=14,MAG_READY=15,
 RUN=20,STATUS_DONE=21,AG_DONE=22,MAG_DONE=23,
 FAILED=30,MAG_FAILED=31,WAIT=200,TX=201,TX_DONE=202;
 reg[7:0]state,return_state,fail_state,wait_next,config_index,asa_index;
 reg[31:0]wait_count,read_started;
 reg[6:0]device;
 reg[7:0]address,write_value,length;
 reg[1:0]operation,start;
 wire done,busy,valid;wire[2:0]error;wire[111:0]block;
 assign phase=state;
 imu_i2c_master #(.RETRY_MAX(0),.DIV_250K(I2C_DIV),.READ_BYTES(14),.BUS_TIMEOUT_CLKS(BUS_TIMEOUT)) bus(
  .clk_sys(clk),.rst_n(rst_n),.device_address(device),.read_length(length),
  .Start_Sig(start),.Addr_Sig(address),.WrData(write_value),.RdData(),.Done_Sig(done),
  .scl(scl),.sda(sda),.Busy_Sig(busy),.Error_Sig(error),.Data_Valid(valid),
  .RdBlock(block),.Byte_Valid(),.Byte_Index(),.Diag_MaxWait(),.Diag_Trace());
 task transaction;
  input[6:0]dev;input[7:0]regaddr,value,count,success,failure;input reading;
  begin device<=dev;address<=regaddr;write_value<=value;length<=count;
   operation<=reading?2'b10:2'b01;return_state<=success;fail_state<=failure;state<=TX;end
 endtask
 task delay_to;
  input[31:0]cycles;input[7:0]next_state;
  begin wait_count<=cycles;wait_next<=next_state;state<=WAIT;end
 endtask
 function[7:0]cfg_reg;input[7:0]index;
  begin case(index)
   0:cfg_reg=8'h6B;1:cfg_reg=8'h6C;2:cfg_reg=8'h1A;3:cfg_reg=8'h1B;
   4:cfg_reg=8'h1C;5:cfg_reg=8'h1D;6:cfg_reg=8'h19;7:cfg_reg=8'h6A;
   8:cfg_reg=8'h37;default:cfg_reg=8'h38;endcase end
 endfunction
 function[7:0]cfg_value;input[7:0]index;
  begin case(index)
   0:cfg_value=1;1:cfg_value=0;2:cfg_value=3;3:cfg_value=8'h08;
   4:cfg_value=8'h08;5:cfg_value=3;6:cfg_value=4;7:cfg_value=0;
   8:cfg_value=2;default:cfg_value=1;endcase end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=BOOT;return_state<=0;fail_state<=0;wait_next<=0;wait_count<=0;config_index<=0;asa_index<=0;
   device<=7'h68;address<=0;write_value<=0;length<=1;operation<=0;start<=0;read_started<=0;
   ag_data<=0;mag_data<=0;asa<=24'h808080;ag_time<=0;mag_time<=0;ag_sequence<=0;mag_sequence<=0;
   mpu_ready<=0;mag_ready<=0;ag_valid<=0;mag_valid<=0;whoami<=0;mag_id<=0;last_error<=0;errors<=0;overflows<=0;
  end else begin
   start<=0;
   case(state)
    BOOT:delay_to(BOOT_CLKS,IDENT);
    IDENT:transaction(7'h68,8'h75,0,1,IDENT_DONE,FAILED,1);
    IDENT_DONE:begin whoami<=block[7:0];
     if(block[7:0]!=8'h71)begin last_error<=8'h10;errors<=errors+1'b1;state<=FAILED;end
     else state<=RESET_MPU;
    end
    RESET_MPU:transaction(7'h68,8'h6B,8'h80,1,RESET_WAIT,FAILED,0);
    RESET_WAIT:begin config_index<=0;delay_to(RESET_CLKS,CFG);end
    CFG:transaction(7'h68,cfg_reg(config_index),cfg_value(config_index),1,CFG_DONE,FAILED,0);
    CFG_DONE:if(config_index==9)begin mpu_ready<=1;last_error<=0;state<=MAG_IDENT;end
      else begin config_index<=config_index+1'b1;state<=CFG;end
    MAG_IDENT:transaction(7'h0C,0,0,1,MAG_IDENT_DONE,MAG_FAILED,1);
    MAG_IDENT_DONE:begin mag_id<=block[7:0];
     if(block[7:0]!=8'h48)begin last_error<=8'h20;errors<=errors+1'b1;state<=MAG_FAILED;end
     else state<=MAG_PD;
    end
    MAG_PD:transaction(7'h0C,8'h0A,0,1,16,MAG_FAILED,0);
    16:delay_to(MODE_CLKS,MAG_FUSE);
    MAG_FUSE:transaction(7'h0C,8'h0A,8'h0F,1,17,MAG_FAILED,0);
    17:begin asa_index<=0;delay_to(MODE_CLKS,MAG_ASA);end
    MAG_ASA:transaction(7'h0C,8'h10+asa_index,0,1,MAG_ASA_DONE,MAG_FAILED,1);
    MAG_ASA_DONE:begin asa[asa_index*8+:8]<=block[7:0];
     if(asa_index==2)state<=MAG_PD2;else begin asa_index<=asa_index+1'b1;state<=MAG_ASA;end
    end
    MAG_PD2:transaction(7'h0C,8'h0A,0,1,18,MAG_FAILED,0);
    18:delay_to(MODE_CLKS,MAG_CONT);
    MAG_CONT:transaction(7'h0C,8'h0A,8'h16,1,MAG_READY,MAG_FAILED,0);
    MAG_READY:begin mag_ready<=1;delay_to(MODE_CLKS,RUN);end
    RUN:if(poll)transaction(7'h68,8'h3A,0,1,STATUS_DONE,24,1);
    STATUS_DONE:if(block[0])begin
      read_started<=time_us;transaction(7'h68,8'h3B,0,14,AG_DONE,24,1);
     end else if(mag_ready)begin read_started<=time_us;transaction(7'h0C,8'h02,0,8,MAG_DONE,25,1);end
     else state<=RUN;
    AG_DONE:begin ag_data<=block;ag_time<=read_started;ag_sequence<=ag_sequence+1'b1;ag_valid<=1;
     if(last_error!=8'h20)last_error<=0;
     if(mag_ready)begin read_started<=time_us;transaction(7'h0C,8'h02,0,8,MAG_DONE,25,1);end
     else state<=RUN;
    end
    MAG_DONE:begin
     if(block[0])begin
      if(block[59])begin mag_valid<=0;overflows<=overflows+1'b1;last_error<=8'h30;end
      else begin mag_data<=block[8+:48];mag_time<=read_started;mag_sequence<=mag_sequence+1'b1;mag_valid<=1;end
     end
     state<=RUN;
    end
    24:begin ag_valid<=0;mpu_ready<=0;mag_ready<=0;mag_valid<=0;delay_to(RETRY_CLKS,IDENT);end
    25:begin mag_valid<=0;state<=RUN;end
    FAILED:begin mpu_ready<=0;mag_ready<=0;ag_valid<=0;mag_valid<=0;delay_to(RETRY_CLKS,IDENT);end
    MAG_FAILED:begin mag_ready<=0;mag_valid<=0;state<=RUN;end
    WAIT:if(wait_count==0)state<=wait_next;else wait_count<=wait_count-1'b1;
    TX:if(!busy)begin start<=operation;state<=TX_DONE;end
    TX_DONE:if(done)begin
     if(error!=0||(operation[1]&&!valid))begin
      last_error<=error!=0?{5'd0,error}:8'h05;errors<=errors+1'b1;state<=fail_state;
     end else state<=return_state;
    end
    default:state<=FAILED;
   endcase
  end
 end
endmodule
