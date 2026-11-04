`timescale 1ns/1ps
`include "r6_motion_policy.vh"
// 链路由GMII接收、打包CDC、50MHz命令保护和NCS1状态载荷组成。
// 复用imu_bundle_cdc，两时钟域须共同复位。
// 本模块只输出执行目标，不直接连接电机引脚或PI选择器。
module nav_control_link #(
 parameter integer CLK_HZ=50000000,
 parameter [31:0] CONFIG_ID=`R6_CONFIG_ID
)(
 input rx_clk,clk,rst_n,input gmii_rxen,gmii_rxer,input [7:0] gmii_rxd,
 input[47:0]raw_sonar,input[95:0]raw_sequence,raw_age_us,input[2:0]raw_available,raw_zero,
 input[63:0]effective_duty,input[7:0]execution_flags,input[2:0]sonar_fresh,sonar_near,sonar_pending,sonar_observation_block,
 input[1:0]front_nonzero_sequence,input[14:0]sonar_age_diag,input[2:0]observation_pending,input motion_blocked,ready,local_stop,fault,input [63:0] measured_rpm_centi,
 input [127:0] encoder_counts,input [3:0] encoder_valid,fault_mask,
 input [55:0] diagnostic,
 output claimed,armed,output [63:0] targets,output[7:0]motion_intent,output [1:0] owner_mode,
 output status_valid,input status_ready,output [767:0] status_payload
);
 (* ASYNC_REG="TRUE" *)reg[1:0]rx_reset=0,control_reset=0;
 always @(posedge rx_clk or negedge rst_n)if(!rst_n)rx_reset<=0;else rx_reset<={rx_reset[0],1'b1};
 always @(posedge clk or negedge rst_n)if(!rst_n)control_reset<=0;else control_reset<={control_reset[0],1'b1};
 wire rx_valid,rx_ready,command_valid;wire[479:0]rx_command,command;
 nav_command_rx_gmii receiver(.clk(rx_clk),.rst_n(rx_reset[1]),.en(gmii_rxen),.er(gmii_rxer),.data(gmii_rxd),
  .valid(rx_valid),.ready(rx_ready),.command(rx_command));
 imu_bundle_cdc #(.WIDTH(480)) crossing(.src_clk(rx_clk),.src_rst_n(rst_n),.src_valid(rx_valid),.src_ready(rx_ready),.src_data(rx_command),
  .dst_clk(clk),.dst_rst_n(rst_n),.dst_valid(command_valid),.dst_ready(1'b1),.dst_data(command));
 wire[31:0]session,sequence_last,accepted_count,rejected_count,watchdog_count;
 wire[63:0]lease;wire[7:0]state,reason;wire accepted_pulse;
 nav_velocity_guard #(.CLK_HZ(CLK_HZ),.CONFIG_ID(CONFIG_ID)) guard(.clk(clk),.rst_n(control_reset[1]),.cmd_valid(command_valid),.command(command),
  .ready(ready),.local_stop(local_stop),.fault(fault),.claimed(claimed),.armed(armed),.targets(targets),.motion_intent(motion_intent),.session(session),.sequence_last(sequence_last),
  .lease(lease),.state(state),.reason(reason),.owner_mode(owner_mode),.accepted_count(accepted_count),.rejected_count(rejected_count),
  .watchdog_count(watchdog_count),.accepted_pulse(accepted_pulse));
 localparam integer CYCLES_PER_MS=CLK_HZ/1000;
 reg[31:0]divider,uptime_ms;reg[5:0]period_ms;reg periodic;
 reg[63:0]previous_lease;reg[7:0]previous_state;
 always @(posedge clk or negedge control_reset[1])begin
  if(!control_reset[1])begin divider<=0;uptime_ms<=0;period_ms<=0;periodic<=0;previous_lease<=0;previous_state<=0;end
  else begin
   previous_lease<=lease;previous_state<=state;periodic<=0;
   if(divider==CYCLES_PER_MS-1)begin
    divider<=0;uptime_ms<=uptime_ms+1'b1;
    if(period_ms==49)begin period_ms<=0;periodic<=1;end else period_ms<=period_ms+1'b1;
   end else divider<=divider+1'b1;
  end
 end
 wire[735:0]snapshot;
 assign snapshot[31:0]=32'h3153434e;
 assign snapshot[39:32]=1;
 assign snapshot[47:40]=state;
 assign snapshot[55:48]={front_nonzero_sequence,observation_pending,3'b111};
 assign snapshot[63:56]=motion_blocked?8'd11:reason;
 assign snapshot[95:64]=session;
 assign snapshot[127:96]=sequence_last;
 assign snapshot[191:128]=lease;
 assign snapshot[223:192]=uptime_ms;
 assign snapshot[255:224]=CONFIG_ID;
 assign snapshot[319:256]=targets;
 assign snapshot[383:320]=measured_rpm_centi;
 assign snapshot[511:384]=encoder_counts;
 assign snapshot[519:512]={sonar_age_diag[3:0],encoder_valid};
 assign snapshot[527:520]={sonar_age_diag[7:4],fault_mask};
 assign snapshot[543:528]=`R6_COMMAND_TTL_MS;
 assign snapshot[559:544]=12000; // 轮速能力及NCV/NCS报文布局保持不变。
 assign snapshot[575:560]=5; // 字节6的位7至6携带前方非零报文序号对4取模的结果。
 assign snapshot[607:576]=accepted_count;
 assign snapshot[639:608]=rejected_count;
 assign snapshot[671:640]=watchdog_count;
 assign snapshot[679:672]={sonar_age_diag[13:12],sonar_age_diag[11:8],owner_mode};
 assign snapshot[719:680]=diagnostic[39:0];
 assign snapshot[727:720]={sonar_age_diag[14],diagnostic[46:40]};
 assign snapshot[735:728]=diagnostic[55:48];
 // NCD5为同一状态通道和同一发送者的只读伴随数据。
 // 每类最多保留一项待处理任务，编码器接受时NCS或ACK优先。
 // 原子捕获92字节并使用原CRC，保证源值与年龄一致。
 wire[735:0]diagnostic_snapshot;
 assign diagnostic_snapshot[31:0]=32'h3544434e; // 小端字符NCD5。
 assign diagnostic_snapshot[39:32]=1;
 assign diagnostic_snapshot[47:40]=5; // 实际S5策略。
 assign diagnostic_snapshot[55:48]={5'b0,raw_available};
 assign diagnostic_snapshot[63:56]={5'b0,raw_zero};
 assign diagnostic_snapshot[95:64]=CONFIG_ID;
 assign diagnostic_snapshot[127:96]=uptime_ms;
 assign diagnostic_snapshot[175:128]=raw_sonar;
 assign diagnostic_snapshot[191:176]=0;
 assign diagnostic_snapshot[287:192]=raw_sequence;
 // 寄存的源数据微秒年龄，不是状态或文件发布时间。
 // 实际50MHz时钟下，源时间量化误差最多1微秒。
 assign diagnostic_snapshot[319:288]=raw_age_us[31:0];
 assign diagnostic_snapshot[351:320]=raw_age_us[63:32];
 assign diagnostic_snapshot[383:352]=raw_age_us[95:64];
 assign diagnostic_snapshot[447:384]=effective_duty;
 assign diagnostic_snapshot[455:448]=execution_flags;
 assign diagnostic_snapshot[463:456]={5'b0,sonar_fresh};
 assign diagnostic_snapshot[471:464]={5'b0,sonar_near};
 assign diagnostic_snapshot[479:472]={5'b0,sonar_pending};
 assign diagnostic_snapshot[487:480]={5'b0,sonar_observation_block};
 assign diagnostic_snapshot[735:488]=0;
 reg ncs_pending,ncd_pending;
 wire encoder_ready;
 wire ncs_event=periodic||accepted_pulse||lease!=previous_lease||state!=previous_state;
 wire take_ncs=ncs_pending&&encoder_ready;
 wire take_ncd=!ncs_pending&&ncd_pending&&encoder_ready;
 always @(posedge clk or negedge control_reset[1])begin
  if(!control_reset[1])begin ncs_pending<=0;ncd_pending<=0;end
  else begin
   if(ncs_event)ncs_pending<=1;else if(take_ncs)ncs_pending<=0;
   if(periodic)ncd_pending<=1;else if(take_ncd)ncd_pending<=0;
  end
 end
 // 先同步流水寄存遥测数据源，再选择NCS或NCD。
 // 报告的源快照延后一个控制时钟，运动执行不受影响。
 reg [735:0] snapshot_q,diagnostic_snapshot_q;
 always @(posedge clk)begin
  snapshot_q<=snapshot;
  diagnostic_snapshot_q<=diagnostic_snapshot;
 end
 nav_status_payload status_encoder(.clk(clk),.rst_n(control_reset[1]),.request(take_ncs||take_ncd),
  .snapshot(take_ncs?snapshot_q:diagnostic_snapshot_q),.request_ready(encoder_ready),.valid(status_valid),.ready(status_ready),.payload(status_payload));
endmodule
