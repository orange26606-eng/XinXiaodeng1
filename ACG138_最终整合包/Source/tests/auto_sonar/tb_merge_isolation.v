`timescale 1ns/1ps
module tb_merge_isolation;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,nav_owned=0,nav_armed=0,remote_stop=0,remote_live=1;
 reg [3:0] move=0,pressed=0,windows=0;
 reg [2:0] valid=7,fresh=7,near_mask=0,pending=0;
 reg degraded=0;reg [63:0] ticks=0,rpm=0;reg [43:0] nav_ticks=0;reg[7:0]intent=0;
 wire[43:0] targets[0:1];wire[63:0]duties[0:1],effective[0:1];
 wire[3:0]actual[0:1],reason[0:1],pwm[0:1],in1[0:1],in2[0:1],fault[0:1];
 wire[2:0]phase[0:1];wire[1:0]retries[0:1],diag[0:1];wire hold[0:1];
 integer cycles_seen=0,window=0,a;
 `define PARAMETERS .NAV_CONTROL(1),.FEEDBACK_RECOVERY(1),.HOLD_CLKS(4),.RELEASE_CLKS(2),.RAMP_CLKS(2),.RECOVERY_START(100000),.RECOVERY_QUIET(100000),.RECOVERY_LIMIT(10000),.TURN_START_CLKS(30),.INNER_TURN_START_CLKS(20)
 `define INPUTS .clk(clk),.rst_n(rst),.nav_claimed(nav_owned),.nav_armed(nav_armed),.nav_stable_front(1'b1),.nav_rpm(rpm),.nav_ticks(nav_ticks),.nav_reverse(4'b1010),.nav_intent(intent),.remote_claimed(1'b1),.remote_live(remote_live),.remote_start_auto(1'b0),.remote_stop(remote_stop),.remote_motion(move),.degraded(degraded),.keys_released(1'b1),.pressed(pressed),.front(24'd800),.left(24'd500),.right(24'd500),.raw_mm({16'd500,16'd500,16'd800}),.valid(valid),.fresh(fresh),.near_mask(near_mask),.update(3'd0),.near_pending_mask(pending),.near_recover_mask(3'd0),.near_timeout_mask(3'd0),.zero_mask(3'd0),.encoder_steps(4'd0),.enc_ready(4'hf),.window_end(windows),.ticks(ticks)
 robot_motion_baseline #(`PARAMETERS) baseline(`INPUTS,.actual(actual[0]),.stop_reason(reason[0]),.targets(targets[0]),.duties(duties[0]),.motor_pwm(pwm[0]),.motor_in1(in1[0]),.motor_in2(in2[0]),.feedback_fault(fault[0]),.feedback_phase(phase[0]),.feedback_retries(retries[0]),.guard_diag(diag[0]),.effective_duties(effective[0]),.hold_off(hold[0]));
 robot_motion #(`PARAMETERS) merged(`INPUTS,.sonar_packet(3'd0),.sonar_raw(48'd0),.actual(actual[1]),.stop_reason(reason[1]),.targets(targets[1]),.duties(duties[1]),.motor_pwm(pwm[1]),.motor_in1(in1[1]),.motor_in2(in2[1]),.feedback_fault(fault[1]),.feedback_phase(phase[1]),.feedback_retries(retries[1]),.guard_diag(diag[1]),.effective_duties(effective[1]),.hold_off(hold[1]));
 always @(negedge clk)begin window=(window+1)%31;windows=window==0?15:0;ticks={4{16'd30}};end
 always @(posedge clk)begin #1;if(rst)begin
  cycles_seen=cycles_seen+1;
  if({targets[0],duties[0],actual[0],reason[0],pwm[0],in1[0],in2[0],fault[0],phase[0],retries[0],diag[0],effective[0],hold[0]} !==
     {targets[1],duties[1],actual[1],reason[1],pwm[1],in1[1],in2[1],fault[1],phase[1],retries[1],diag[1],effective[1],hold[1]})begin
   $display("FAIL nonAUTO differential cycle=%0d move=%0d nav=%b actual=%0d/%0d targets=%h/%h duty=%h/%h",cycles_seen,move,nav_owned,actual[0],actual[1],targets[0],targets[1],duties[0],duties[1]);$fatal;
  end
 end end
 task waitcycles;input integer count;begin repeat(count)@(negedge clk);#2;end endtask
 task navdrive;input signed[15:0]l,r;input[10:0]lt,rt;input[7:0]shape;
 begin rpm={r,l,r,l};nav_ticks={rt,lt,rt,lt};intent=shape;waitcycles(180);end endtask
 initial begin
  waitcycles(4);rst=1;waitcycles(5);
  for(a=1;a<=8;a=a+1)begin move=a;waitcycles(180);valid=0;fresh=0;near_mask=7;degraded=1;waitcycles(80);valid=7;fresh=7;near_mask=0;degraded=0;remote_stop=1;waitcycles(2);remote_stop=0;move=0;waitcycles(5);end
  move=0;nav_owned=1;nav_armed=1;
  navdrive(4000,-4000,88,88,1);navdrive(1000,-4000,22,88,2);navdrive(4000,-1000,88,22,2);
  navdrive(-3000,-3000,66,66,3);navdrive(3000,3000,66,66,3);navdrive(-3000,3000,66,66,4);
  pending=1;navdrive(4000,-4000,88,88,1);pending=0;
  nav_armed=0;waitcycles(20);nav_owned=0;move=1;waitcycles(60);remote_live=0;waitcycles(10);
  $display("PASS: unchanged manual8directions/NAVstraight-arcs-pivots-reverse, old-sonar isolation, PI/PWM/assist/feedback/STOP differential cycles=%0d",cycles_seen);$finish;
 end
 initial begin #1000000;$fatal(1,"isolation timeout");end
endmodule
