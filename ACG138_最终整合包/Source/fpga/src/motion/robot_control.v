`timescale 1ns/1ps
`include "r6_motion_policy.vh"
module robot_control #(
 parameter NAV_CONTROL=0,FEEDBACK_RECOVERY=1,
 parameter CLK_HZ=50000000,BAUD=115200,TICK_CLKS=326,STALE_CLKS=30000000,LOST_CLKS=CLK_HZ*3/2,
 parameter KEY_CLKS=1000000,WINDOW_CLKS=5000000,REPORT_CLKS=10000000,
 parameter HOLD_CLKS=2500000,LONG_CLKS=50000000,RAMP_CLKS=500000,
 parameter START_CLKS=15000000,NO_FEEDBACK_CLKS=10000000,UNIFORM_RANGE=0,
 // 67.5mm轮径、1320四倍频计数每圈时，249计数约40.00mm，作为暂定制动余量。
 parameter IGNORE_ZERO=1,
 parameter ZERO_TRAVEL_TICKS=249,ZERO_HOLD_CLKS=CLK_HZ*3/5,
 // 67.5mm轮径时186计数约29.88mm，保守低于30mm。
 parameter NEAR_CONFIRM=1,NEAR_TRAVEL_TICKS=186,NEAR_HOLD_CLKS=CLK_HZ*3/10,
 parameter [3:0] ENCODER_INVERT=0,MOTOR_INVERT=0
)(
 input nav_claimed,nav_armed,input[63:0]nav_rpm,input[7:0]nav_intent,
 output[47:0]nav_raw_sonar,output[95:0]nav_raw_sequence,nav_raw_age_us,output[2:0]nav_raw_available,nav_raw_zero,
 output[11:0]nav_sensor_flags,output[63:0]nav_effective_duty,output[7:0]nav_execution_flags,
 output[1:0]nav_front_sequence,output[2:0]nav_pending,output reg[14:0]nav_sonar_diag,
 output nav_blocked,nav_ready,nav_local_stop,nav_fault,output[3:0]nav_fault_mask,output[63:0]nav_measured,
 output[55:0]nav_diagnostic,
 input remote_valid,input [79:0] remote_data,
 input clk,front_rx,left_rx,right_rx,input [4:0] key_n,input [3:0] enc_a,enc_b,
 output [111:0] telemetry_data,
 output [127:0] odom_positions,output [3:0] odom_ready,
 output [7:0] odom_ab,output [31:0] odom_invalid,
 output host_tx,output [7:0] led,output [3:0] motor_in1,motor_in2,motor_pwm);
 (* ASYNC_REG="TRUE" *) reg [1:0] reset_release=0;
 always @(posedge clk or negedge key_n[4])
  if(!key_n[4])reset_release<=0;else reset_release<={reset_release[0],1'b1};
 wire rst_n=key_n[4] && reset_release[1];
 wire [3:0] pressed;
 wire remote_claimed,remote_live,remote_start_auto,remote_stop;
 wire [3:0] remote_motion;
 wire [31:0] remote_status;
 wire [7:0] remote_stop_source;
 phone_remote_control phone_remote(.clk(clk),.rst_n(rst_n),.cmd_valid(remote_valid),.cmd_data(remote_data),
  .local_pressed(pressed),.claimed(remote_claimed),.live(remote_live),.motion(remote_motion),
  .start_auto(remote_start_auto),.force_stop(remote_stop),.status(remote_status),.stop_source(remote_stop_source));
 // 直接同步按键释放状态，避免仍按住的按键在消抖完成前重新使能。
 (* ASYNC_REG="TRUE" *) reg [3:0] key_meta,key_sync;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin key_meta<=15;key_sync<=15;end
  else begin key_meta<=key_n[3:0];key_sync<=key_meta;end
 wire [23:0] front,left,right;
 wire [2:0] valid,update,sonar_packet;
 wire [47:0]auto_ranges;wire [5:0]auto_zones,auto_near_counts;wire [2:0]auto_valid,auto_fresh,auto_pending,auto_pending_expired;
 wire [5:0] receive_errors;
 wire [8:0] status;
 wire [287:0] diagnostic;
 wire [2:0] healthy,near_mask,fresh,zero_mask,zero_expired_mask;
 wire [47:0] control_raw;
 wire [2:0] near_pending_mask,near_timeout_mask,near_recover_mask;
 wire [5:0] near_counts;
 wire[95:0]nonzero_sequence;wire[2:0]raw_seen;
 assign nav_front_sequence=nonzero_sequence[1:0];
 assign nav_raw_sequence=nonzero_sequence;assign nav_raw_available=raw_seen;assign nav_raw_zero=last_raw_zero;
 assign nav_raw_sonar=raw_mm;
 wire[1:0]guard_diag;
 assign nav_execution_flags={1'b0,!nav_claimed,guard_diag[1],guard_diag[0],hold_off,nav_blocked,actual!=0,stable_front};
 wire[2:0]last_raw_zero;wire[2:0]near_allow;wire[2:0]observation_block;
 assign nav_pending=near_pending_mask;
 assign nav_sensor_flags={observation_block,near_pending_mask,near_mask,fresh};
 wire [2:0] observation_mask=near_pending_mask|near_recover_mask;
 wire [3:0] encoder_steps;
 wire [47:0] raw_mm={diagnostic[287:272],diagnostic[191:176],diagnostic[95:80]};
 // 接收错误表示最近一帧状态，不表示保留缓存的年龄。
 // 单个损坏帧或半帧不立即撤销原有效运动。
 // 只有完整且校验正确的帧，包括按策略忽略的零值帧，
 // 才刷新rx_age，连续坏帧仍会进入陈旧或失联状态。
 wire [2:0] recent_rx={diagnostic[271:240]<STALE_CLKS,
                       diagnostic[175:144]<STALE_CLKS,
                       diagnostic[79:48]<STALE_CLKS};
 // 新鲜度表示接收年龄，不表示净空；正确接收的
 // 非零盲区值仍属新鲜近障，方向处理可
 // 允许经RK和雷达核对的后退，同时禁止继续靠近该侧。
 assign fresh=valid & recent_rx;
 assign healthy=valid;
 wire degraded=!( &fresh );



 wire [23:0] control_front=valid[0]?front:24'd4500;
 wire [23:0] control_left=valid[1]?left:24'd4500;
 wire [23:0] control_right=valid[2]?right:24'd4500;
 XR_ultrasonic #(.P_CLOCK_HZ(CLK_HZ),.P_IGNORE_ZERO(IGNORE_ZERO),.P_NEAR_CONFIRM(NEAR_CONFIRM),.P_NEAR_CLKS(NEAR_HOLD_CLKS),.P_TICK_CLKS(TICK_CLKS),.P_STALE_CLKS(STALE_CLKS),.P_LOST_CLKS(LOST_CLKS),.P_ZERO_FRESH_CLKS(CLK_HZ/5),.P_ZERO_CLKS(ZERO_HOLD_CLKS)) sensors(
  .packet_update(sonar_packet),.raw_ages_us(nav_raw_age_us),.raw_seen_mask(raw_seen),.nonzero_sequence(nonzero_sequence),.last_raw_zero_mask(last_raw_zero),.near_allow(near_allow),.near_pending_mask(near_pending_mask),.near_timeout_mask(near_timeout_mask),
  .near_recover_mask(near_recover_mask),.near_counts(near_counts),
  .clk_sys(clk),.rst_n(rst_n),.us_front_rxd(front_rx),.us_left_rxd(left_rx),.us_right_rxd(right_rx),
  .Distance(front),.distance_left(left),.distance_right(right),.valid_mask(valid),
  .sample_update(update),.status_codes(status),.receive_errors(receive_errors),.zero_mask(zero_mask),.zero_expired_mask(zero_expired_mask),.control_raw(control_raw),.age_front(),.age_left(),.age_right(),.diagnostic(diagnostic),.near_mask(near_mask));
 wire [127:0] positions,errors;
 assign odom_positions=positions;
 assign odom_ready=ready;
 wire [63:0] deltas,duties;
 wire [43:0] targets;
 wire [7:0] ab_states;
 assign odom_ab=ab_states;
 assign odom_invalid=errors[31:0]+errors[63:32]+errors[95:64]+errors[127:96];
 wire [3:0] ready,window_end;
 genvar i;
 generate for(i=0;i<4;i=i+1)begin:g_input
  XR_key_debounce #(.P_DEBOUNCE_CLKS(KEY_CLKS)) key_filter(
   .clk_sys(clk),.rst_n(rst_n),.key_n(key_n[i]),.pressed(pressed[i]));
  XR_encoder_quad #(.P_WINDOW_CLKS(WINDOW_CLKS-1)) quad(
   .clk_sys(clk),.rst_n(rst_n),.enc_a(ENCODER_INVERT[i]?enc_b[i]:enc_a[i]),
   .enc_b(ENCODER_INVERT[i]?enc_a[i]:enc_b[i]),.pos_cnt(positions[i*32+:32]),
   .speed_ticks(deltas[i*16+:16]),.window_end(window_end[i]),.initialized(ready[i]),
   .invalid_count(errors[i*32+:32]),.ab_state(ab_states[i*2+:2]),.step_valid(encoder_steps[i]));
 end endgenerate
 // 诊断年龄分桶采用流水线，运动时效独立使用
 // 原分辨率年龄，不依赖这些200ms分桶。
 function [34:0] age200_high;
 input [31:0] age;
 reg [31:0] remainder;reg [3:0] bucket;reg clipped;
 begin
  remainder=age;bucket=0;clipped=age>=CLK_HZ*3;
  if(clipped)remainder=0;
  else begin
   if(remainder>=CLK_HZ*8/5)begin remainder=remainder-CLK_HZ*8/5;bucket=bucket+8;end
   if(remainder>=CLK_HZ*4/5)begin remainder=remainder-CLK_HZ*4/5;bucket=bucket+4;end
  end
  age200_high={clipped,bucket[3:2],remainder};
 end
 endfunction
 function [3:0] age200_low;
 input [34:0] intermediate;
 reg [31:0] remainder;reg [3:0] bucket;
 begin
  remainder=intermediate[31:0];bucket={intermediate[33:32],2'b00};
  if(intermediate[34])bucket=15;
  else begin
   if(remainder>=CLK_HZ*2/5)begin remainder=remainder-CLK_HZ*2/5;bucket=bucket+2;end
   if(remainder>=CLK_HZ/5)bucket=bucket+1;
  end
  age200_low=bucket;
 end
 endfunction
 reg [34:0] age_stage[0:2];
 reg [2:0] age_zero_snapshot;
 integer age_index;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
   nav_sonar_diag<=15'hfff;age_zero_snapshot<=0;
   for(age_index=0;age_index<3;age_index=age_index+1)age_stage[age_index]<={1'b1,34'd0};
  end else begin
   age_stage[0]<=age200_high(diagnostic[47:16]);
   age_stage[1]<=age200_high(diagnostic[143:112]);
   age_stage[2]<=age200_high(diagnostic[239:208]);
   age_zero_snapshot<=last_raw_zero;
   nav_sonar_diag<={age_zero_snapshot,age200_low(age_stage[2]),age200_low(age_stage[1]),age200_low(age_stage[0])};
  end
 wire auto_mode,hold_off;
 wire[15:0]display_front=auto_mode?auto_ranges[15:0]:front[15:0];
 wire[15:0]display_left=auto_mode?auto_ranges[31:16]:left[15:0];
 wire[15:0]display_right=auto_mode?auto_ranges[47:32]:right[15:0];
 wire[2:0]display_valid=auto_mode?auto_valid:wire_valid;
 wire[2:0]report_valid=auto_mode?auto_valid:valid;
 wire[2:0]display_near=auto_mode?{auto_zones[5:4]==2,auto_zones[3:2]==2,auto_zones[1:0]==2}:near_mask;
 wire[8:0]display_status;
 genvar ac;
 generate for(ac=0;ac<3;ac=ac+1)begin:g_auto_display
  assign display_status[ac*3+:3]=!auto_mode?status[ac*3+:3]:!auto_valid[ac]?3'd6:(!auto_fresh[ac]||auto_pending[ac])?3'd7:3'd0;
 end endgenerate
 assign nav_ready=rst_n&&(&ready);
 // 只有接受的活动手机状态、STOP或CLAIM可覆盖导航，空闲HOLD
 // 心跳保持控制权，无效或未认证报文不能夺取控制权。
 assign nav_local_stop=!key_n[4]||(|pressed)||remote_stop||auto_mode||remote_start_auto||
  (remote_claimed&&remote_live&&(remote_motion!=0));
 // 撤销控制权后保留触发标记，下一次CLAIM再清除。
 reg nav_claimed_previous;
 reg [7:0] local_stop_latched;
 wire [7:0] local_stop_now=(!key_n[4]?8'h01:8'h00) |
  ((|pressed)?8'h02:8'h00) | (auto_mode?8'h04:8'h00) |
  (remote_stop?remote_stop_source:8'h00) |
  ((remote_claimed&&remote_live&&remote_motion!=0)?8'h10:8'h00) |
  ((remote_claimed&&remote_live&&remote_start_auto)?8'h20:8'h00);
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin nav_claimed_previous<=0;local_stop_latched<=1;end
  else begin
   nav_claimed_previous<=nav_claimed;
   if(nav_local_stop&&local_stop_now!=0)local_stop_latched<=local_stop_now;
   else if(nav_claimed&&!nav_claimed_previous)local_stop_latched<=0;
  end
 function[11:0] range12;input[23:0]value;input good;
  begin range12=!good?12'hfff:value>4094?12'd4094:value[11:0];end
 endfunction
 reg front_clear_once,stable_front;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin front_clear_once<=0;stable_front<=0;end
  else if(!valid[0]||front<`R6_FRONT_RELEASE_MM||control_raw[15:0]<`R6_FRONT_RELEASE_MM)begin front_clear_once<=0;stable_front<=0;end
  else if(update[0])begin
   if(front_clear_once)stable_front<=1;
   front_clear_once<=1;
  end
 // NCS1版本3的85至91字节，位39表示已有两笔新的前方样本不小于R6_FRONT_RELEASE_MM，即40mm。
 // 被忽略的零值帧不产生update，也不累计确认。
 assign nav_diagnostic={local_stop_latched,1'b0,degraded,(near_mask|observation_block),fresh,
  stable_front,valid,range12(right,valid[2]),range12(left,valid[1]),range12(front,valid[0])};
 assign nav_fault=FEEDBACK_RECOVERY?(feedback_phase==4&&|feedback_fault):|feedback_fault;assign nav_fault_mask=nav_fault?feedback_fault:4'd0;
 nav_rpm_feedback measured_units(.clk(clk),.rst_n(rst_n),.ticks(deltas),.window_end(window_end),.rpm_centi(nav_measured));
 wire[43:0]nav_ticks;wire[3:0]nav_reverse;
 nav_pi_pipeline nav_units(.clk(clk),.rst_n(rst_n),.rpm_centi(nav_rpm),.permit(nav_armed),.ticks_per_window(nav_ticks),.reverse(nav_reverse));
 wire [2:0] feedback_phase;wire [1:0] feedback_retries;
 // 四个原保留位容纳11种有效阶段与重试组合。
 wire [3:0] feedback_meta=!FEEDBACK_RECOVERY || feedback_phase==0?4'd0:feedback_phase==4?4'd10:({2'b0,feedback_retries}*4'd3)+{1'b0,feedback_phase};
  wire [3:0] state_code,attempts,decision,actual,feedback_fault,stop_reason;
 assign nav_blocked=NAV_CONTROL&&nav_claimed&&nav_armed&&nav_rpm!=0&&actual==0&&!hold_off&&
  (stop_reason==1||stop_reason==2||stop_reason==3||stop_reason==9||stop_reason==10||stop_reason==13||stop_reason==15);
 // 仅判定观察准入：已在直行、可信基准新鲜、无其他不确定通道或故障。
 // 获准后解析器计时独立运行，不受此准入信号影响。
 assign near_allow=NEAR_CONFIRM&&(&ready)&&!(|feedback_fault)&&!degraded ?
 {((actual==4||actual==6)&&!near_mask[2]&&!observation_mask[2]),
  ((actual==3||actual==5)&&!near_mask[1]&&!observation_mask[1]),
  ((actual==1||actual==3||actual==4||actual==5||actual==6)&&!near_mask[0]&&!observation_mask[0])}:3'd0;
 // A3在高保留位加入反馈阶段及重试次数，主机继续兼容A2。
 // 导航遥测仍为32字节RBT1及CRC32。
 // 正常超量程沿用valid=0、error=0及MAX距离约定。
 wire [2:0] wire_valid;
 genvar t;
 generate for(t=0;t<3;t=t+1)begin:g_wire
  assign wire_valid[t]=valid[t] && !(status[t*3+:3]==0 && (IGNORE_ZERO?control_raw[t*16+:16]:diagnostic[t*96+80+:16])>4500);
 end endgenerate
 reg [111:0] telemetry_snapshot;
 assign telemetry_data=telemetry_snapshot;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)telemetry_snapshot<=0;
  else telemetry_snapshot<={remote_status[31:16],feedback_meta,remote_status[11:8],(FEEDBACK_RECOVERY?8'hA3:8'hA2),display_near,(stop_reason==12 || stop_reason==14?4'd5:stop_reason==13 || stop_reason==15?4'd3:stop_reason),state_code,
   feedback_fault,actual,auto_mode,display_status,display_valid,display_right,display_left,display_front};
 robot_motion #(.AUTO_CLK_HZ(CLK_HZ),.NAV_CONTROL(NAV_CONTROL),.FEEDBACK_RECOVERY(FEEDBACK_RECOVERY),
  .RECOVERY_START(CLK_HZ),.RECOVERY_QUIET(CLK_HZ*3/5),.RECOVERY_LIMIT(CLK_HZ*3/10),.RECOVERY_WAIT(CLK_HZ),.RECOVERY_STABLE(CLK_HZ*2),.HOLD_CLKS(HOLD_CLKS),.LONG_CLKS(LONG_CLKS),.RELEASE_CLKS(KEY_CLKS),
  .RAMP_CLKS(RAMP_CLKS),.START_CLKS(START_CLKS),.NO_FEEDBACK_CLKS(NO_FEEDBACK_CLKS),
  .NEAR_CONFIRM(NEAR_CONFIRM),.NEAR_TRAVEL_TICKS(NEAR_TRAVEL_TICKS),
  .UNIFORM_RANGE(UNIFORM_RANGE),.MOTOR_INVERT(MOTOR_INVERT),.ZERO_FILTER(!IGNORE_ZERO),.ZERO_TRAVEL_TICKS(ZERO_TRAVEL_TICKS)) motion(
  .sonar_packet(sonar_packet),.sonar_raw(raw_mm),.auto_ranges(auto_ranges),.auto_zones(auto_zones),.auto_valid(auto_valid),.auto_fresh(auto_fresh),.auto_pending(auto_pending),.auto_pending_expired(auto_pending_expired),.auto_near_counts(auto_near_counts),
  .guard_diag(guard_diag),.effective_duties(nav_effective_duty),.nav_rpm(nav_rpm),.nav_intent(nav_intent),.near_timeout_mask(near_timeout_mask),.observation_block(observation_block),.nav_claimed(nav_claimed),.nav_armed(nav_armed),.nav_ticks(nav_ticks),.nav_reverse(nav_reverse),.nav_stable_front(stable_front),
  .remote_claimed(remote_claimed),.remote_live(remote_live),.remote_motion(remote_motion),
  .remote_start_auto(remote_start_auto),.remote_stop(remote_stop),
  .clk(clk),.rst_n(rst_n),.keys_released(key_sync==15),.pressed(pressed),
  .front(control_front),.left(control_left),.right(control_right),
  .near_pending_mask(near_pending_mask),.near_recover_mask(near_recover_mask),
  .zero_mask(zero_mask),.encoder_steps(encoder_steps),.raw_mm(control_raw),.near_mask(near_mask),.fresh(fresh),.degraded(degraded),.valid(healthy),.update(update),.enc_ready(ready),.window_end(window_end),.ticks(deltas),
  .motor_in1(motor_in1),.motor_in2(motor_in2),.motor_pwm(motor_pwm),
  .feedback_phase(feedback_phase),.feedback_retries(feedback_retries),.hold_off(hold_off),.auto_mode(auto_mode),.state_code(state_code),.attempts(attempts),.decision(decision),
  .actual(actual),.feedback_fault(feedback_fault),.stop_reason(stop_reason),.targets(targets),.duties(duties));
 reg [31:0] beat_count;
 reg blink;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin beat_count<=0;blink<=0;end
  else if(beat_count>=CLK_HZ/2-1)begin beat_count<=0;blink<=~blink;end
  else beat_count<=beat_count+1'b1;
 assign led[0]=blink;
 assign led[1]=display_near[0];
 assign led[2]=report_valid[0] && display_front>=200 && display_front<300;
 assign led[3]=report_valid[0] && display_front>=300 && display_front<450;
 assign led[4]=report_valid[0] && display_front>=450 && display_front<600;
 assign led[5]=display_near[1];
 assign led[6]=display_near[2];
 assign led[7]=auto_mode?(!(&auto_valid)||(|feedback_fault)):!(&healthy) || |feedback_fault || state_code==6;
 // 按整条消息仲裁，不混合两个独立活动UART的数据。
 reg [2:0] report_state;
 reg [31:0] report_count;
 wire status_active,encoder_active,status_tx,encoder_tx;
 wire status_trigger=report_state==1,encoder_trigger=report_state==4;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin report_state<=0;report_count<=0;end
  else begin
   if(report_count<REPORT_CLKS-1)report_count<=report_count+1'b1;
   case(report_state)
    0:if(report_count>=REPORT_CLKS-1)begin report_count<=0;report_state<=1;end
    1:report_state<=2;
    2:if(status_active)report_state<=3;
    3:if(!status_active)report_state<=4;
    4:report_state<=5;
    5:if(encoder_active)report_state<=6;
    6:if(!encoder_active)report_state<=0;
    default:report_state<=0;
   endcase
  end
 end
 assign host_tx=(report_state>=4)?encoder_tx:status_tx;
 robot_status #(.FEEDBACK_DIAG(FEEDBACK_RECOVERY),.CLK_HZ(CLK_HZ),.HOST_BAUD(BAUD),.NEAR_DIAG(NEAR_CONFIRM)) report(
  .feedback_phase(feedback_phase),.feedback_retries(feedback_retries),.clk(clk),.rst_n(rst_n),.trigger(status_trigger),.front({8'd0,display_front}),.left({8'd0,display_left}),.right({8'd0,display_right}),
  .near_pending_mask(auto_mode?auto_pending:near_pending_mask),.near_timeout_mask(auto_mode?auto_pending_expired:near_timeout_mask),.near_recover_mask(auto_mode?3'd0:near_recover_mask),.near_counts(auto_mode?auto_near_counts:near_counts),
  .zero_mask(zero_mask),.zero_expired_mask(zero_expired_mask),.reason(stop_reason),.near_mask(display_near),.receive_errors(receive_errors),.diagnostic(diagnostic),.valid(report_valid),.status(display_status),.state_code(state_code),.decision_cmd(decision),.attempts(attempts),
  .actual(actual),.feedback_fault(feedback_fault),.auto_mode(auto_mode),
  .hold_off(hold_off),.host_tx(status_tx),.active(status_active));
 XR_encoder_monitor #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) encoder_report(
  .clk(clk),.rst_n(rst_n),.positions(positions),.errors(errors),.deltas(deltas),
  .ab_states(ab_states),.ready(ready),.trigger(encoder_trigger),.active(encoder_active),
  .motion_code(actual),.serial_out(encoder_tx));
endmodule
