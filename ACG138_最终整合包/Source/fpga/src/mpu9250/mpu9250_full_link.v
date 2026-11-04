`timescale 1ns/1ps
// 被动形成I9E1 v2快照，编码器仍由原电机解码器
// 统一处理，源快照与累计计数使用同一clk50。
module mpu9250_full_link #(
 parameter integer CLK_HZ=50000000,REPORT_CLKS=250000,
 parameter integer BOOT_CLKS=5000000,RESET_CLKS=5000000,
 parameter integer ATT_CAL_SAMPLES=400,
 parameter integer I2C_DIV=500,MODE_CLKS=5000,RETRY_CLKS=50000000,BUS_TIMEOUT=50000
)(input clk50,clk125,rx_clk,rst_n,reset_key_n,
 input[127:0]positions,input[3:0]enc_ready,input[7:0]enc_ab,input[31:0]enc_invalid,
 input[3:0]rgmii_rxd,input rgmii_rx_ctl,
 output tap_en,tap_er,output[7:0]tap_data,
 output remote_valid,input remote_ready,output[79:0]remote_data,
 inout sda,scl,output[3:0]led,
 input tx_grant,output pending,txen,output[7:0]txd);
 (* ASYNC_REG="TRUE" *)reg[1:0]s50=0,s125=0,srx=0,key_sync=3;
 always @(posedge clk50 or negedge rst_n)
  if(!rst_n)begin s50<=0;key_sync<=3;end
  else begin s50<={s50[0],1'b1};key_sync<={key_sync[0],reset_key_n};end
 always @(posedge clk125 or negedge rst_n)
  if(!rst_n)s125<=0;else s125<={s125[0],1'b1};
 always @(posedge rx_clk or negedge rst_n)
  if(!rst_n)srx<=0;else srx<={srx[0],1'b1};
 // 保留原MCV1解码器及远程命令CDC通路。
 // 旧IMC1校准命令仅被接收消费，不操作总线。
 imu_rgmii_command_rx command_rx(.clk_rx(rx_clk),.rst_n(srx[1]),
  .rgmii_rxd(rgmii_rxd),.rgmii_rx_ctl(rgmii_rx_ctl),
  .tap_en(tap_en),.tap_er(tap_er),.tap_data(tap_data),
  .remote_valid(remote_valid),.remote_ready(remote_ready),.remote_data(remote_data),
  .cmd_valid(),.cmd_ready(1'b1),.cmd_op(),.cmd_token());
 reg[31:0]us_div,time_us,period_count,packet_sequence,drops,epoch;
 reg key_previous,poll;
 always @(posedge clk50 or negedge s50[1])begin
  if(!s50[1])begin us_div<=0;time_us<=0;period_count<=0;poll<=0;epoch<=0;key_previous<=1;end
  else begin
   if(us_div==CLK_HZ/1000000-1)begin us_div<=0;time_us<=time_us+1'b1;end
   else us_div<=us_div+1'b1;
   poll<=period_count==REPORT_CLKS-1;
   if(period_count==REPORT_CLKS-1)period_count<=0;else period_count<=period_count+1'b1;
   key_previous<=key_sync[1];
   if(key_previous&&!key_sync[1])epoch<=epoch+1'b1;
  end
 end
 wire[111:0]ag;wire[47:0]mag;wire[23:0]asa;
 wire[31:0]ag_time,mag_time,ag_seq,mag_seq,errors,overflows;
 wire mpu_ready,mag_ready,ag_valid,mag_valid;
 wire[7:0]whoami,mag_id,last_error,phase;
 mpu9250_capture #(.CLK_HZ(CLK_HZ),.I2C_DIV(I2C_DIV),.BOOT_CLKS(BOOT_CLKS),
  .RESET_CLKS(RESET_CLKS),.MODE_CLKS(MODE_CLKS),.RETRY_CLKS(RETRY_CLKS),.BUS_TIMEOUT(BUS_TIMEOUT)) capture(
  .clk(clk50),.rst_n(s50[1]),.poll(poll),.time_us(time_us),.sda(sda),.scl(scl),
  .ag_data(ag),.mag_data(mag),.asa(asa),.ag_time(ag_time),.mag_time(mag_time),
  .ag_sequence(ag_seq),.mag_sequence(mag_seq),.mpu_ready(mpu_ready),.mag_ready(mag_ready),
  .ag_valid(ag_valid),.mag_valid(mag_valid),.whoami(whoami),.mag_id(mag_id),
  .last_error(last_error),.errors(errors),.overflows(overflows),.phase(phase));
 assign led={mpu_ready,mag_ready,ag_valid,last_error!=0};
 // 在整车中增加六轴姿态通路，原采集模块仍为
 // 唯一I2C主机，下方编码器处理及原始报文生成保持不变。
 wire att_pending,att_grant,raw_start;wire[767:0]att_body;
 mpu9250_attitude_stream #(.CAL_SAMPLES(ATT_CAL_SAMPLES)) attitude(
  .clk50(clk50),.clk125(clk125),.rst_n(rst_n),.local_reset_n(s50[1]),
  .key_n(key_sync[1]),.snapshot_enable(poll&&key_sync[1]&&reset_key_n),
  .now_us(time_us),.epoch(epoch),.ag(ag),.ag_time(ag_time),.ag_seq(ag_seq),
  .ag_valid(ag_valid&&mpu_ready),.whoami(whoami),.last_error(last_error),.i2c_errors(errors),
  .tx_grant(att_grant),.pending(att_pending),.tx_body(att_body));

 reg[767:0]payload;reg offer;wire mail_ready,mail_valid,tx_ready;wire[767:0]mail_data;
 integer k;
 always @(posedge clk50 or negedge s50[1])begin
  if(!s50[1])begin payload<=0;offer<=0;packet_sequence<=0;drops<=0;end
  else begin
   offer<=0;
   // 电机异步复位及释放期间不锁存快照。
   if(poll&&key_sync[1]&&reset_key_n)begin
    packet_sequence<=packet_sequence+1'b1;
    if(mail_ready)begin
     offer<=1;payload<=0;
     payload[0+:32]<=32'h31453949;payload[32+:8]<=2;
     payload[40+:8]<={3'd0,(&enc_ready),mag_ready,mpu_ready,mag_valid,ag_valid};
     payload[48+:8]<=whoami;payload[56+:8]<=mag_id;
     payload[64+:32]<=packet_sequence;payload[96+:32]<=time_us;
     payload[128+:32]<=ag_time;payload[160+:32]<=mag_time;
     payload[192+:128]<=positions;
     for(k=0;k<7;k=k+1)payload[(40+k*2)*8+:16]<={ag[k*16+:8],ag[k*16+8+:8]};
     payload[432+:48]<=mag;payload[480+:24]<=asa;payload[504+:8]<=last_error;
     payload[512+:32]<=errors;payload[544+:32]<=overflows;payload[576+:32]<=drops;
     payload[608+:32]<=enc_invalid;payload[640+:8]<=enc_ab;
     payload[648+:8]<={4'b0,enc_ready};payload[656+:8]<=phase;
     payload[672+:32]<=ag_seq;payload[704+:32]<=mag_seq;payload[736+:32]<=epoch;
    end else drops<=drops+1'b1;
   end
  end
 end
 // 邮箱在仲裁与CRC准备阶段保持完整正文。
 imu_bundle_cdc #(.WIDTH(768)) mail(.src_clk(clk50),.src_rst_n(rst_n),
  .src_valid(offer),.src_ready(mail_ready),.src_data(payload),.dst_clk(clk125),.dst_rst_n(rst_n),
  .dst_valid(mail_valid),.dst_ready(raw_start),.dst_data(mail_data));
 // 两条数据流共用一个正文、CRC和帧发送引擎。
 // 仅在对应帧实际接受边沿确认各自邮箱。
 reg last_att;
 wire raw_pending=mail_valid;
 wire choose_att=att_pending&&(!raw_pending||!last_att);
 assign pending=s125[1]&&tx_ready&&(raw_pending||att_pending);
 wire start_frame=tx_grant&&pending;
 assign raw_start=start_frame&&!choose_att;
 assign att_grant=start_frame&&choose_att;
 always @(posedge clk125 or negedge s125[1])
  if(!s125[1])last_att<=1;
  else if(start_frame)last_att<=choose_att;
 imu_udp_tx_shared sender(.clk125m(clk125),.rst_n(s125[1]),.sample_valid(start_frame),
  .sample_ready(tx_ready),.sample_data(choose_att?att_body:mail_data),.attitude_frame(choose_att),
  .gmii_txen(txen),.gmii_txd(txd),.tx_done());
endmodule
