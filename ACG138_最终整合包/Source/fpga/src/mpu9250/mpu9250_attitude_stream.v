`timescale 1ns/1ps
// 增加F6A1 v1及UDP5017，原子保存数据源标识与完整姿态。
// 沿用RAW5016的50MHz时钟、原加速度与角速度读起始时间及序号，
// 以及复位epoch；相对欧拉偏航不代表磁北方向。
module mpu9250_attitude_stream #(parameter CAL_SAMPLES=400)(
 input clk50,clk125,rst_n,local_reset_n,key_n,snapshot_enable,
 input[31:0]now_us,epoch,input[111:0]ag,input[31:0]ag_time,ag_seq,
 input ag_valid,input[7:0]whoami,last_error,input[31:0]i2c_errors,
 input tx_grant,output pending,output[767:0]tx_body);
 wire signed[15:0]ax={ag[7:0],ag[15:8]},ay={ag[23:16],ag[31:24]},az={ag[39:32],ag[47:40]};
 wire signed[15:0]gx={ag[71:64],ag[79:72]},gy={ag[87:80],ag[95:88]},gz={ag[103:96],ag[111:104]};
 wire valid,bias_ready,accel_used;wire[15:0]cal_count;wire[3:0]status;
 wire[127:0]quaternion;wire[95:0]bias;wire signed[31:0]roll,pitch,yaw;
 wire[31:0]result_seq,result_us,math_errors,gaps;
 // S7只清除姿态校准，不单独复位数据邮箱任一端。
 // 采集及原始数据时钟继续运行，原epoch只增加一次。
 imu_attitude_core #(.CAL_SAMPLES(CAL_SAMPLES)) core(.clk(clk50),.rst_n(local_reset_n&&key_n),
  .now_us(now_us),.sample_us(ag_time),.sample_seq(ag_seq),.ag_valid(ag_valid),.whoami(whoami),
  .ax(ax),.ay(ay),.az(az),.gx(gx),.gy(gy),.gz(gz),.ready(),.updated(),.valid(valid),.bias_ready(bias_ready),
  .status(status),.cal_count(cal_count),.quaternion(quaternion),.roll(roll),.pitch(pitch),.yaw(yaw),
  .result_seq(result_seq),.result_us(result_us),.math_errors(math_errors),.gaps(gaps),.accel_used(accel_used),.bias_raw(bias));
 reg[767:0]body;reg offer;reg[31:0]drops;wire room,mail_valid;wire[767:0]mail_data;
 always @(posedge clk50 or negedge local_reset_n)begin
  if(!local_reset_n)begin body<=0;offer<=0;drops<=0;end
  else begin
   offer<=0;
   if(snapshot_enable)begin
    if(room)begin
     offer<=1;body<=0;
     body[0+:32]<=32'h31413646;body[32+:8]<=1;
     body[40+:8]<={5'd0,accel_used,bias_ready,valid};body[48+:8]<=whoami;body[56+:8]<={4'd0,status};
     body[64+:32]<=result_seq;body[96+:32]<=result_us;body[128+:32]<=now_us;body[160+:32]<=ag_seq;
     body[192+:128]<=quaternion;body[320+:32]<=roll;body[352+:32]<=pitch;body[384+:32]<=yaw;
     body[416+:96]<=bias;body[512+:32]<=i2c_errors;body[544+:32]<=math_errors;body[576+:32]<=gaps;
     body[608+:32]<=now_us-result_us;body[640+:16]<=cal_count;body[656+:8]<=last_error;
     body[672+:32]<=drops;body[704+:32]<=ag_time;body[736+:32]<=epoch;
    end else drops<=drops+1'b1;
   end
  end
 end
 imu_bundle_cdc #(.WIDTH(768)) mail(.src_clk(clk50),.src_rst_n(rst_n),.src_valid(offer),.src_ready(room),.src_data(body),
  .dst_clk(clk125),.dst_rst_n(rst_n),.dst_valid(mail_valid),.dst_ready(tx_grant&&pending),.dst_data(mail_data));
 // 整车模块中与RAW5016共用串行化发送器。
 assign pending=mail_valid;
 assign tx_body=mail_data;
endmodule
