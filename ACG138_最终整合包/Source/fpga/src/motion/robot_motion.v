`timescale 1ns/1ps
`include "r6_motion_policy.vh"
module robot_motion #(
 parameter NAV_CONTROL=0,FEEDBACK_RECOVERY=0,AUTO_CLK_HZ=50000000,
 parameter RECOVERY_START=50000000,RECOVERY_QUIET=30000000,RECOVERY_LIMIT=15000000,RECOVERY_WAIT=50000000,RECOVERY_STABLE=100000000,
 parameter ZERO_FILTER=0,ZERO_TRAVEL_TICKS=0,
 parameter NEAR_CONFIRM=0,NEAR_TRAVEL_TICKS=186,
 parameter HOLD_CLKS=2500000,LONG_CLKS=50000000,RELEASE_CLKS=1000000,
 parameter RAMP_CLKS=500000,START_CLKS=15000000,NO_FEEDBACK_CLKS=10000000,
 parameter TURN_CLKS=50000000,ARC_CLKS=100000000,
 parameter PWM_PERIOD=2500,MAX_PWM=1750,UNIFORM_RANGE=0,DIRECT_TARGET=1,
 parameter MANUAL_STRAIGHT_TICKS=220,MANUAL_CURVE_INNER_TICKS=53,MANUAL_CURVE_OUTER_TICKS=264,MANUAL_SPIN_TICKS=198,
 parameter TURN_START_CLKS=17500000,TURN_START_PWM=1350,
 parameter INNER_TURN_START_CLKS=10000000,INNER_TURN_START_PWM=1000,
 parameter [3:0] REMOTE_MOTOR_XOR=4'b1010, MOTOR_INVERT=0
)(
 input [2:0] sonar_packet,input [47:0] sonar_raw,
 output [47:0] auto_ranges,output [5:0] auto_zones,auto_near_counts,output [2:0] auto_valid,auto_fresh,auto_pending,auto_pending_expired,
 input [2:0] near_pending_mask,near_recover_mask,near_timeout_mask,
 output[2:0] observation_block,
 input [2:0] zero_mask,input [3:0] encoder_steps,
 input nav_claimed,nav_armed,nav_stable_front,input[43:0]nav_ticks,input[3:0]nav_reverse,input[63:0]nav_rpm,input[7:0]nav_intent,
 input remote_claimed,remote_live,remote_start_auto,remote_stop,input [3:0] remote_motion,
 input clk,rst_n,input degraded,input keys_released,input [3:0] pressed,
 input [23:0] front,left,right,input [47:0] raw_mm,
 input [2:0] valid,update,near_mask,fresh,
 input [3:0] enc_ready,window_end,input [63:0] ticks,
 output [2:0] feedback_phase,output [1:0] feedback_retries,
 output hold_off,output reg auto_mode,output [3:0] state_code,attempts,decision,
 output [3:0] actual,feedback_fault,output reg [3:0] stop_reason,
 output [1:0]guard_diag,output[63:0]effective_duties,output [43:0] targets,output [63:0] duties,output [3:0] motor_in1,motor_in2,motor_pwm);

 wire nav_owned=NAV_CONTROL&&nav_claimed;
 wire manual_owned=!nav_owned&&!auto_mode;
 reg[10:0]ramp_l,ramp_r;wire[10:0]target_l,target_r;
 wire[3:0]legacy_fault,guard_fault;wire[3:0]nav_fault=FEEDBACK_RECOVERY?4'd0:feedback_fault;
 wire guard_stop,guard_freeze,guard_low,guard_reset;wire[63:0]guard_ceiling;
 reg[3:0]safe_demand,guard_demand;
 always @(posedge clk or negedge rst_n)if(!rst_n)guard_demand<=0;else guard_demand<=safe_demand;
 assign guard_diag={guard_low||guard_freeze||feedback_phase!=0,guard_stop};
 wire explicit_stop=(nav_owned&&!nav_armed)||remote_stop||
 (!nav_owned&&remote_claimed&&(!remote_live||(!auto_mode&&remote_motion==0)))||
 (!nav_owned&&!auto_mode&&!remote_claimed&&pressed==0)||(auto_mode&&pressed!=0);
 assign feedback_fault=FEEDBACK_RECOVERY?guard_fault:legacy_fault;
 generate if(FEEDBACK_RECOVERY)begin:g_guard
 robot_feedback_guard #(.START_CLKS(RECOVERY_START),.QUIET_CLKS(RECOVERY_QUIET),.LIMIT_CLKS(RECOVERY_LIMIT),.WAIT_CLKS(RECOVERY_WAIT),.STABLE_CLKS(RECOVERY_STABLE)) guard(
 .clk(clk),.rst_n(rst_n),.clear_request(explicit_stop),.demand(guard_demand),.actual(actual),.targets(targets),.ticks(ticks),.duties(duties),.window_end(window_end),
 .stop_drive(guard_stop),.freeze_pi(guard_freeze),.low_speed(guard_low),.reset_pi(guard_reset),.phase(feedback_phase),.retries(feedback_retries),.fault_mask(guard_fault),.ceiling(guard_ceiling));
 end else begin:g_no_guard
 assign guard_stop=0;assign guard_freeze=0;assign guard_low=0;assign guard_reset=0;assign guard_ceiling=0;assign guard_fault=0;assign feedback_phase=0;assign feedback_retries=0;
 end endgenerate


 localparam[10:0]MANUAL_SPEED=MANUAL_STRAIGHT_TICKS,ARC_INNER_SPEED=MANUAL_CURVE_INNER_TICKS;
 wire auto_enable=auto_mode&&armed&&!nav_owned&&remote_claimed&&remote_live&&pressed==0;
 wire [3:0] auto_action,auto_reason;wire [10:0] auto_l,auto_r;wire [1:0] auto_light;
 robot_auto_sonar #(.HOLD_CLKS(AUTO_CLK_HZ/10),.ARC_CLKS(AUTO_CLK_HZ/2),.TURN_CLKS(AUTO_CLK_HZ*2/5),
 .OBSERVE_CLKS(AUTO_CLK_HZ/5),.BACK_CLKS(AUTO_CLK_HZ*2),.STALE_CLKS(AUTO_CLK_HZ*3/5),
 .LOST_CLKS(AUTO_CLK_HZ*3/2),.HISTORY_CLKS(AUTO_CLK_HZ*20),.PENDING_CLKS(AUTO_CLK_HZ*3/10)) sonar_auto(
 .clk(clk),.rst_n(rst_n),.enable(auto_enable),.packet(sonar_packet),.raw(sonar_raw),.actual(actual),
 .enc_ready(enc_ready),.ticks(ticks),.window_end(window_end),.action(auto_action),.state_code(state_code),
 .reason(auto_reason),.speed_l(auto_l),.speed_r(auto_r),.attempts(attempts),.light(auto_light),
 .ranges(auto_ranges),.zones(auto_zones),.usable(auto_valid),.fresh(auto_fresh),
 .pending(auto_pending),.pending_expired(auto_pending_expired),.confirm_counts(auto_near_counts));
 reg [3:0] auto_action_q,auto_reason_q;reg [10:0] auto_l_q,auto_r_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin auto_action_q<=0;auto_reason_q<=0;auto_l_q<=0;auto_r_q<=0;end
  else begin auto_action_q<=auto_action;auto_reason_q<=auto_reason;auto_l_q<=auto_l;auto_r_q<=auto_r;end
 end
 assign decision=auto_action==1||auto_action==3||auto_action==4?4'hA:auto_action==5?4'h6:auto_action==6?4'h9:0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)auto_mode<=0;
  else if(remote_stop||(remote_claimed&&!remote_live)||pressed!=0)auto_mode<=0;
  else if(remote_start_auto&&remote_claimed&&remote_live)auto_mode<=1;
  else if(nav_owned||(remote_claimed&&remote_motion!=0))auto_mode<=0;
 end
 reg armed;reg[31:0]release_count;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin armed<=0;release_count<=0;end
  else if(remote_stop||(remote_claimed&&!remote_live))begin armed<=0;release_count<=0;end
  else if(remote_start_auto&&remote_claimed&&remote_live)begin armed<=1;release_count<=0;end
  else if(!keys_released||pressed!=0)release_count<=0;
  else if(!armed)begin if(release_count>=RELEASE_CLKS-1)armed<=1;else release_count<=release_count+1'b1;end
 end
 reg[3:0]manual_action;
 always @*case(pressed)
 1:manual_action=1;2:manual_action=2;4:manual_action=5;8:manual_action=6;
 5:manual_action=3;9:manual_action=4;6:manual_action=7;10:manual_action=8;default:manual_action=0;endcase
 // 在PI量化前按最终原始有符号机械RPM分类。
 // RPM转计数后原子锁存整组命令，原始
 // nav_armed及STOP仍在管线之外立即控制执行许可。
 reg[63:0]rpm_q;reg[43:0]ticks_q;reg[7:0]intent_q;reg[3:0]reverse_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin rpm_q<=0;ticks_q<=0;intent_q<=0;reverse_q<=0;end
  else begin rpm_q<=nav_rpm;ticks_q<=nav_ticks;intent_q<=nav_intent;reverse_q<=nav_reverse;end
 end
 // 线上右轮命令使用与车体前进相反的机械符号。
 // 选择目标之前，直接从编码后的左右命令对解析方向。
 wire signed [15:0] raw_l=$signed(rpm_q[15:0]);
 wire signed [15:0] raw_r=$signed(rpm_q[31:16]);
 wire forward_pair=raw_l>raw_r;
 wire signed [16:0] raw_sum=$signed({raw_l[15],raw_l})+$signed({raw_r[15],raw_r});
 wire signed [16:0] raw_difference=$signed({raw_l[15],raw_l})-$signed({raw_r[15],raw_r});
 wire signed [18:0] scaled_sum={raw_sum,2'b00};
 wire signed [18:0] turn_left_bound=scaled_sum-raw_difference;
 wire signed [18:0] turn_right_bound=scaled_sum+raw_difference;
 // 前进命令对满足4×abs(L+R)不小于L-R时，可等价为下列两个符号判断。
 wire turn_shape_ok=turn_left_bound>=0 || turn_right_bound<=0;
 wire pair_ok=rpm_q[15:0]==rpm_q[47:32]&&rpm_q[31:16]==rpm_q[63:48];
 wire [7:0] shape=raw_l==0&&raw_r==0?0:raw_l==raw_r?3:forward_pair?1:4;
 wire curve_left=raw_sum<0,curve=raw_sum!=0,mixed=raw_l<0||raw_r>0;
 wire shape_ok=pair_ok&&((intent_q==2&&shape==1&&curve&&turn_shape_ok)||intent_q==shape);
 reg[3:0]nav_request;
 always @*begin
  nav_request=0;
  if(shape_ok)case(shape)
   1:nav_request=!curve?1:curve_left?3:4;
   3:nav_request=curve_left?5:6;
   4:nav_request=2;
   default:nav_request=0;
  endcase
 end
 wire[2:0]obs=near_pending_mask|near_recover_mask;
 reg[2:0]near_limited;
 localparam TRAVEL_BITS=NEAR_TRAVEL_TICKS>0?$clog2(NEAR_TRAVEL_TICKS+1):1;
 reg [TRAVEL_BITS-1:0] travel[0:11];
 wire [2:0] travel_reached;
 genvar channel;
 generate for(channel=0;channel<3;channel=channel+1)begin:g_near_travel
  assign travel_reached[channel]=travel[channel*4]>=NEAR_TRAVEL_TICKS ||
   travel[channel*4+1]>=NEAR_TRAVEL_TICKS || travel[channel*4+2]>=NEAR_TRAVEL_TICKS ||
   travel[channel*4+3]>=NEAR_TRAVEL_TICKS;
 end endgenerate
 integer j,k;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin near_limited<=0;for(k=0;k<12;k=k+1)travel[k]<=0;end
  else for(j=0;j<3;j=j+1)begin
   if(!obs[j])begin near_limited[j]<=0;for(k=0;k<4;k=k+1)travel[j*4+k]<=0;end
   else begin
    for(k=0;k<4;k=k+1)if(encoder_steps[k]&&travel[j*4+k]<NEAR_TRAVEL_TICKS)travel[j*4+k]<=travel[j*4+k]+1'b1;
    if(near_timeout_mask[j]===1'b1||travel_reached[j])near_limited[j]<=1;
   end
  end
 end
 assign observation_block=near_limited|near_timeout_mask;
 wire[2:0]blocked=observation_block&near_pending_mask;
 wire[23:0]df=front<raw_mm[15:0]?front:raw_mm[15:0];
 wire[23:0]dl=left<raw_mm[31:16]?left:raw_mm[31:16];
 wire[23:0]dr=right<raw_mm[47:32]?right:raw_mm[47:32];
 wire[2:0]ok=valid&fresh;
 wire enc_ok=(&enc_ready)&&!(|nav_fault);
 reg nav_allowed;
 always @*begin
  nav_allowed=0;
  case(intent_q)
   0:nav_allowed=shape==0;
   4:nav_allowed=(ok&6)==6;
   3:nav_allowed=ok==7&&blocked==0&&df>=30&&dl>=30&&dr>=30;
   1,2:begin
    nav_allowed=ok[0]&&!blocked[0]&&(intent_q==2?df>=`R6_FRONT_FORWARD_STOP_MM:!near_mask[0]&&df>=`R6_FRONT_FORWARD_STOP_MM);
    if(curve)begin
     if(curve_left)nav_allowed=nav_allowed&&ok[1]&&!blocked[1]&&dl>=30;
     else nav_allowed=nav_allowed&&ok[2]&&!blocked[2]&&dr>=30;
     if(mixed)nav_allowed=nav_allowed&&ok==7&&blocked==0&&dl>=30&&dr>=30;
    end
   end
   default:nav_allowed=0;
  endcase
  nav_allowed=nav_allowed&&shape_ok&&enc_ok;
 end
 wire [10:0] nav_l=ticks_q[0+:11],nav_r=ticks_q[11+:11];
 wire [2:0] observation_shift=(nav_l>1439||nav_r>1439)?6:
  (nav_l>719||nav_r>719)?5:(nav_l>359||nav_r>359)?4:
  (nav_l>179||nav_r>179)?3:(nav_l>89||nav_r>89)?2:
  (nav_l>44||nav_r>44)?1:0;
 wire observation_slow=forward_pair&&obs[0];
 wire [10:0] observed_l=nav_l==nav_r?(nav_l>44?11'd44:nav_l):(nav_l>>observation_shift);
 wire [10:0] observed_r=nav_l==nav_r?(nav_r>44?11'd44:nav_r):(nav_r>>observation_shift);
 wire [10:0] nav_target_l=observation_slow?observed_l:nav_l;
 wire [10:0] nav_target_r=observation_slow?observed_r:nav_r;
 reg[3:0]requested,request_next,reason_next;reg[10:0]want_l,want_r,want_l_next,want_r_next;
 always @*begin
  request_next=0;want_l_next=0;want_r_next=0;reason_next=0;
  if(nav_owned)begin
   if(nav_armed&&nav_allowed)begin request_next=nav_request;want_l_next=nav_target_l;want_r_next=nav_target_r;end
   else if(nav_armed&&shape!=0)reason_next=!enc_ok?10:9;
  end else if(auto_mode)begin
   request_next=auto_action_q;want_l_next=auto_l_q;want_r_next=auto_r_q;reason_next=auto_reason_q;
  end else if(remote_claimed)begin
   if(remote_live)begin request_next=remote_motion;want_l_next=MANUAL_SPEED;want_r_next=MANUAL_SPEED;end
  end else if(armed)begin request_next=manual_action;want_l_next=MANUAL_SPEED;want_r_next=MANUAL_SPEED;end
  if(!nav_owned&&!auto_mode)begin
   case(request_next)
    3,7:begin want_l_next=ARC_INNER_SPEED;want_r_next=MANUAL_CURVE_OUTER_TICKS;end
    4,8:begin want_l_next=MANUAL_CURVE_OUTER_TICKS;want_r_next=ARC_INNER_SPEED;end
    5,6:begin want_l_next=MANUAL_SPIN_TICKS;want_r_next=MANUAL_SPIN_TICKS;end
    default:begin end
   endcase
   if(!enc_ok)begin request_next=0;reason_next=10;end
  end
  if(remote_stop||(!nav_owned&&remote_claimed&&!remote_live)||(nav_owned&&!nav_armed))begin request_next=0;reason_next=11;end
  if(nav_owned&&observation_slow&&request_next!=0)reason_next=14;
  safe_demand=request_next;
  if(FEEDBACK_RECOVERY)begin
   if(guard_stop)request_next=0;
   if(feedback_phase!=0)reason_next=feedback_phase==4?4:6;
  end
  if(request_next==0)begin want_l_next=0;want_r_next=0;end
 end
 // 方向准入和反馈恢复缩放分级处理，恢复阶段使用
 // 同一个二次幂缩放系数保持请求曲率，独立
 // 按44或旧目标分别裁剪可能将脱困弧线拉直。
 reg[3:0]request_pre,reason_pre;reg[10:0]pre_l,pre_r;
 function [3:0] fit_shift;
 input [10:0] want,limit;
 reg [11:0] fits,first_fit;
 integer bit_shift;
 begin
  for(bit_shift=0;bit_shift<12;bit_shift=bit_shift+1)
   fits[bit_shift]=(want>>bit_shift)<=limit;
  first_fit[0]=fits[0];
  for(bit_shift=1;bit_shift<12;bit_shift=bit_shift+1)
   first_fit[bit_shift]=fits[bit_shift]&&!fits[bit_shift-1];
  fit_shift[0]=first_fit[1]|first_fit[3]|first_fit[5]|first_fit[7]|first_fit[9]|first_fit[11];
  fit_shift[1]=first_fit[2]|first_fit[3]|first_fit[6]|first_fit[7]|first_fit[10]|first_fit[11];
  fit_shift[2]=|first_fit[7:4];
  fit_shift[3]=|first_fit[11:8];
 end
 endfunction
 wire[10:0]freeze_l=guard_freeze?target_l:11'd2047,freeze_r=guard_freeze?target_r:11'd2047;
 wire[10:0]low_limit=auto_mode?11'd66:11'd44;
 wire[10:0]limit_l=guard_low&&freeze_l>low_limit?low_limit:freeze_l,limit_r=guard_low&&freeze_r>low_limit?low_limit:freeze_r;
 wire[3:0]shift_l=fit_shift(pre_l,limit_l),shift_r=fit_shift(pre_r,limit_r);
 wire[3:0]common_shift=shift_l>shift_r?shift_l:shift_r;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin requested<=0;want_l<=0;want_r<=0;stop_reason<=0;request_pre<=0;pre_l<=0;pre_r<=0;reason_pre<=0;end
  else begin
   request_pre<=request_next;pre_l<=want_l_next;pre_r<=want_r_next;reason_pre<=reason_next;
   requested<=guard_stop?0:request_pre;want_l<=pre_l>>common_shift;want_r<=pre_r>>common_shift;stop_reason<=reason_pre;
   if(auto_mode&&guard_low&&!guard_freeze&&(pre_l>66||pre_r>66))begin
    want_l<=pre_l<pre_r?11'd28:11'd66;want_r<=pre_r<pre_l?11'd28:11'd66;
   end
  end
 end
  // 前进弧线沿用前进极性，只有极性变化才重新等待50ms，
  // 同极性的直行或弧线速度变化仍走原PI及目标渐变路径。
  // 手动四弧线使用显式机械方向：FL=-24.09/+120、FR=+120/-24.09，后退整体反向。
  // 目标53/264始终为绝对计数，不能将负值塞入无符号11位目标。
  wire [3:0] auto_polarity=(requested==1||requested==3||requested==4)?4'b0000:
  (requested==2||requested==7||requested==8)?4'b1111:requested==5?4'b0101:4'b1010;
 wire [3:0] polarity=nav_owned?(reverse_q^REMOTE_MOTOR_XOR):auto_mode?auto_polarity:requested==1?4'b0000:
                     requested==2?4'b1111:(requested==3 || requested==8 || requested==5)?4'b0101:4'b1010;
 reg [3:0] held_polarity;
 reg holding,drive_ready;
 reg [31:0] hold_count;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin held_polarity<=0;holding<=0;drive_ready<=0;hold_count<=0;end
  else if(requested==0) begin
   // 普通零目标等待保留已知极性，但电机输出为零。
   // 解除使能或退出导航时仍重置方向切换间隔。
   if(!nav_owned||!nav_armed)begin holding<=0;drive_ready<=0;hold_count<=0;end
  end
  else if(!holding || polarity!=held_polarity) begin
   held_polarity<=polarity;holding<=1;drive_ready<=0;hold_count<=0;
  end else if(!drive_ready) begin
   if(hold_count>=HOLD_CLKS-1) drive_ready<=1;
   else hold_count<=hold_count+1'b1;
  end
 end
 // AUTO准入使用已一致寄存的命令，超声波决策改变后
 // 在下一50MHz时钟到达此处，先于方向切换生效。
 wire auto_current=!auto_mode||(auto_enable&&auto_action_q!=0&&requested==auto_action_q);
 wire permit=rst_n && !explicit_stop && requested!=0 && drive_ready && held_polarity==polarity && (!nav_owned||nav_armed)&&auto_current;
 assign actual=permit?requested:4'd0;
 assign hold_off=requested!=0 && !permit;
 reg [31:0] ramp_count;
 function [10:0] rise;input [10:0] now,want;
  begin rise=now>=want?want:((want-now>4)?now+11'd4:want);end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin ramp_l<=0;ramp_r<=0;ramp_count<=0;end
  else if(!permit) begin ramp_l<=0;ramp_r<=0;ramp_count<=0;end
  else if(DIRECT_TARGET&&!auto_mode)begin ramp_l<=want_l;ramp_r<=want_r;ramp_count<=0;end
  else if(ramp_count>=RAMP_CLKS-1) begin
   ramp_count<=0;ramp_l<=rise(ramp_l,want_l);ramp_r<=rise(ramp_r,want_r);
  end else ramp_count<=ramp_count+1'b1;
 end
 assign target_l=!permit?0:((ramp_l>want_l)?want_l:ramp_l);
 assign target_r=!permit?0:((ramp_r>want_r)?want_r:ramp_r);
 // AUTO 起步助力持续250ms；持续停指令500ms后才重新允许。
 reg auto_assist_ready,auto_assist_live;reg [3:0] auto_assist_seen;
 reg [31:0] auto_assist_age,auto_stop_age;
 localparam AUTO_ASSIST_CLKS=AUTO_CLK_HZ/4,AUTO_ASSIST_REARM=AUTO_CLK_HZ/2;
 integer ai;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin auto_assist_ready<=1;auto_assist_live<=0;auto_assist_seen<=0;auto_assist_age<=0;auto_stop_age<=0;end
  else if(!auto_mode)begin auto_assist_ready<=1;auto_assist_live<=0;auto_assist_seen<=0;auto_assist_age<=0;auto_stop_age<=0;end
  else begin
   if(requested==0)begin if(auto_stop_age<AUTO_ASSIST_REARM)auto_stop_age<=auto_stop_age+1'b1;else auto_assist_ready<=1;end
   else auto_stop_age<=0;
   if(!permit||guard_stop||guard_freeze||guard_low)begin auto_assist_live<=0;auto_assist_age<=0;end
   else if(auto_assist_ready&&!auto_assist_live)begin auto_assist_live<=1;auto_assist_ready<=0;auto_assist_age<=0;auto_assist_seen<=0;end
   else if(auto_assist_live)begin
    if(auto_assist_age>=AUTO_ASSIST_CLKS-1)auto_assist_live<=0;else auto_assist_age<=auto_assist_age+1'b1;
    for(ai=0;ai<4;ai=ai+1)if(window_end[ai]&&enc_ready[ai]&&ticks[ai*16+:16]!=0)auto_assist_seen[ai]<=1;
   end
  end
 end
 reg [15:0] pwm_count;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) pwm_count<=0;
  else if(pwm_count>=PWM_PERIOD-1)pwm_count<=0;else pwm_count<=pwm_count+1'b1;
 genvar i;
 generate for(i=0;i<4;i=i+1)begin:g_wheel
  wire [10:0] target=(i==0 || i==2)?target_l:target_r;
  wire [15:0] duty;wire assist_tracking;wire[15:0]applied_duty_for_pi,assist_applied_snapshot;
  assign targets[i*11+:11]=target;
  assign duties[i*16+:16]=duty;
  robot_wheel #(.RECOVERY(FEEDBACK_RECOVERY),.START_CLKS(START_CLKS),.NO_FEEDBACK_CLKS(NO_FEEDBACK_CLKS),.MAX_PWM(MAX_PWM)) wheel(
   .clk(clk),.rst_n(rst_n),.target(target),.ticks(ticks[i*16+:16]),
   .window_end(window_end[i]),.freeze_output(guard_freeze),.clear_pi(guard_reset),
   .track_output(assist_tracking),.applied_duty(assist_applied_snapshot),.duty(duty),.fault(legacy_fault[i]));
  // AUTO与手机手动模式共用实际前、左、右方向约定。
  // 实体按键沿用原映射，相机和IMU接线保持不变。
  wire rev=polarity[i]^MOTOR_INVERT[i]^((nav_owned || auto_mode || remote_claimed)?REMOTE_MOTOR_XOR[i]:1'b0);
  // S7异步复位和各停止条件绕过PI更新窗口，立即作用。
  // 暂定启动助力不是持续最低PWM，仅在弧线外轮
  // 或任一原地转向轮从零目标起步时提供一个窗口。
  // 行进中转弯或重复刷新命令不能反复重启助力计时。
  localparam[15:0]TURN_FLOOR=(TURN_START_PWM>MAX_PWM)?MAX_PWM:TURN_START_PWM;
  localparam[15:0]INNER_TURN_FLOOR=(INNER_TURN_START_PWM>MAX_PWM)?MAX_PWM:INNER_TURN_START_PWM;
  wire outer_turn=!auto_mode&&((actual==3||actual==7)?(i==1||i==3):
                  (actual==4||actual==8)?(i==0||i==2):actual==5||actual==6);
  wire inner_turn=!auto_mode&&!nav_owned&&((actual==3||actual==7)?(i==0||i==2):
                  (actual==4||actual==8)?(i==1||i==3):1'b0);
  wire wheel_driving=permit&&!guard_stop&&target!=0;
  // 与现有自动前方观察限速保持相同规则。
  // 不额外限制手动模式及无关的侧向或原地观察动作。
  wire observation_limited=nav_owned&&forward_pair&&obs[0];
  wire recovery_restricted=guard_stop||guard_low||guard_freeze||feedback_phase!=0||(|feedback_fault)||observation_limited;
  reg was_driving,turn_start_active;reg[31:0]turn_start_count;
  always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin was_driving<=0;turn_start_active<=0;turn_start_count<=0;end
   else begin
    was_driving<=wheel_driving;
    if(!wheel_driving||recovery_restricted||!outer_turn)begin turn_start_active<=0;turn_start_count<=0;end
    else if(!was_driving&&TURN_START_CLKS>0)begin turn_start_active<=1;turn_start_count<=0;end
    else if(turn_start_active)begin
     if(turn_start_count>=TURN_START_CLKS-1)turn_start_active<=0;
     else turn_start_count<=turn_start_count+1'b1;
    end
   end
  end
  reg inner_start_active;reg[31:0]inner_start_count;
  always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin inner_start_active<=0;inner_start_count<=0;end
   else if(!wheel_driving||recovery_restricted||!inner_turn)begin inner_start_active<=0;inner_start_count<=0;end
   else if(!was_driving&&INNER_TURN_START_CLKS>0)begin inner_start_active<=1;inner_start_count<=0;end
   else if(inner_start_active)begin
    if(inner_start_count>=INNER_TURN_START_CLKS-1)inner_start_active<=0;
    else inner_start_count<=inner_start_count+1'b1;
   end
  end
  wire boost_allowed=turn_start_active&&outer_turn&&wheel_driving&&!recovery_restricted;
  wire inner_boost_allowed=inner_start_active&&inner_turn&&wheel_driving&&!recovery_restricted;
  wire auto_boost_allowed=auto_mode&&auto_assist_live&&!auto_assist_seen[i]&&wheel_driving&&!recovery_restricted&&state_code!=13&&state_code!=14;
  wire[15:0]auto_floor=(auto_action_q==5||auto_action_q==6)?16'd1250:
   (auto_action_q==3||auto_action_q==7)?((i==0||i==2)?16'd850:16'd1100):
   (auto_action_q==4||auto_action_q==8)?((i==0||i==2)?16'd1100:16'd850):16'd1100;
  wire[15:0]bounded_auto_floor=auto_floor>MAX_PWM?MAX_PWM:auto_floor;
  assign assist_tracking=boost_allowed||inner_boost_allowed||auto_boost_allowed;
  wire[15:0]assist_floor=auto_boost_allowed?bounded_auto_floor:boost_allowed?TURN_FLOOR:inner_boost_allowed?INNER_TURN_FLOOR:16'd0;
  wire[15:0]start_duty=duty<assist_floor?assist_floor:duty;
  wire [15:0] effective_duty=guard_freeze && start_duty>guard_ceiling[i*16+:16]?guard_ceiling[i*16+:16]:start_duty;
  assign applied_duty_for_pi=(permit&&!guard_stop&&target!=0)?effective_duty:16'd0;
  // valid为真时与真实施加占空相等；数据只用已寄存的助力/PI/上限，避免把许可长路径串到16位快照。
  wire auto_snapshot_assist=auto_mode&&auto_assist_live&&!auto_assist_seen[i]&&state_code!=13&&state_code!=14;
  wire[15:0]snapshot_floor=auto_snapshot_assist?bounded_auto_floor:turn_start_active?TURN_FLOOR:inner_start_active?INNER_TURN_FLOOR:16'd0;
  wire[15:0]snapshot_start=duty<snapshot_floor?snapshot_floor:duty;
  assign assist_applied_snapshot=guard_freeze&&snapshot_start>guard_ceiling[i*16+:16]?guard_ceiling[i*16+:16]:snapshot_start;
  assign effective_duties[i*16+:16]=applied_duty_for_pi;
  assign motor_pwm[i]=permit && !guard_stop && target!=0 && pwm_count<effective_duty;
  assign motor_in1[i]=permit && target!=0 && rev;
  assign motor_in2[i]=permit && target!=0 && !rev;
 end endgenerate
endmodule

module robot_wheel #(parameter RECOVERY=0,START_CLKS=15000000,NO_FEEDBACK_CLKS=10000000,MAX_PWM=1750)(
 input freeze_output,clear_pi,track_output,input [15:0] applied_duty,
 input clk,rst_n,input [10:0] target,input signed [15:0] ticks,input window_end,
 output reg [15:0] duty,output reg fault);
 // 通过窗口快照隔离较长的许可与目标路径和PI运算。
 // 最终PWM仍受实时许可及目标控制，停车不等待快照更新。
 reg [10:0] target_sample;
 reg signed [15:0] ticks_sample;
 reg pending;
 reg track_valid;reg[12:0]track_ff;reg signed[24:0]track_error;reg[15:0]track_duty;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin target_sample<=0;ticks_sample<=0;pending<=0;end
  else begin
   pending<=target!=0 && !fault && (window_end || (pending && track_valid && !track_output));
   if(window_end)begin target_sample<=target;ticks_sample<=ticks;end
  end
 end
 wire [16:0] mag=ticks_sample[15]?{1'b0,(~ticks_sample+16'd1)}:{1'b0,ticks_sample};
 reg signed [23:0] integral;
 wire signed [24:0] error=$signed({1'b0,target_sample})-$signed({1'b0,mag});
 // 前馈与PI输出单位为PWM计数，当前增益为暂定值，需落地调参。
 wire signed [25:0] candidate=integral+error;
 wire signed [26:0] raw=($signed({1'b0,target_sample})*4)+(error*2)+candidate;
 wire freeze=(raw>MAX_PWM && error>0)||(raw<0 && error<0);
 // 跟踪已施加的占空，寄存器闭环，无组合环；同一窗口的目标/反馈快照用于FF和P。
 // 跟踪态直接设置本周期I，不再额外累加一次e；退出后仅在新pending窗口正常积分。
 // 加一级相干快照隔离实际执行门到积分算术的长路径；退出跟踪时只重放尚未处理的新窗口。
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin track_valid<=0;track_ff<=0;track_error<=0;track_duty<=0;end
  else begin
   // 无效数据不消费，只清有效位；真正PI输出仍由下方STOP/target0路径立即清零。
   track_valid<=track_output && target!=0 && !fault && !(RECOVERY && (clear_pi||freeze_output));
   track_ff<={target_sample,2'b00};track_error<=error;
   track_duty<=applied_duty>MAX_PWM?MAX_PWM:applied_duty;
  end
 end
 wire signed[27:0]tracked_integral=$signed({1'b0,track_duty})-$signed({1'b0,track_ff})-(track_error*2);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin integral<=0;duty<=0;end
  else if(target==0 || fault || (RECOVERY && clear_pi))begin integral<=0;duty<=0;end
  else if(RECOVERY && freeze_output)begin end
  else if(track_valid)begin
   integral<=tracked_integral>4096?24'sd4096:(tracked_integral< -4096?-24'sd4096:tracked_integral);
   duty<=track_duty;
  end else if(pending)begin
   if(!freeze)integral<=candidate>4096?24'sd4096:(candidate< -4096?-24'sd4096:candidate);
   duty<=raw<0?0:(raw>MAX_PWM?MAX_PWM:raw);
  end
 end
 reg [31:0] startup_count,quiet_count;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin fault<=0;startup_count<=0;quiet_count<=0;end
  else if(RECOVERY)begin fault<=0;startup_count<=0;quiet_count<=0;end
  else if(!fault)begin
   if(target==0)begin startup_count<=0;quiet_count<=0;end
   else if(startup_count<START_CLKS)begin startup_count<=startup_count+1'b1;quiet_count<=0;end
   else if(duty<156)quiet_count<=0;
   else if(window_end && ticks!=0)quiet_count<=0;
   else if(quiet_count>=NO_FEEDBACK_CLKS-1)fault<=1;
   else quiet_count<=quiet_count+1'b1;
  end
 end
endmodule
