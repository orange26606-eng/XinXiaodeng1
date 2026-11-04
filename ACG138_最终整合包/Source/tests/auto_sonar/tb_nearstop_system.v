`timescale 1ns/1ps
// 只使用公开运动、测距和编码器端口，刻意保留非零滑行反馈，
// 验证新近障清零命令而不等待实测完全静止。
module tb_nearstop_system;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,stop=0,start_auto=0,live=1,publish_windows=1,watching=0;
 reg[3:0]motion=0,windows=0;reg[2:0]packet=0;reg[47:0]raw={16'd500,16'd500,16'd800};reg[63:0]ticks={4{16'd40}};
 wire[3:0]actual,state,reason,pwm;wire[43:0]targets;wire[63:0]duties;wire[5:0]zones;wire automode,hold_off;
 integer divider=0,checks=0,c,j,zero_cycles=0,forbidden_seen=0;reg[3:0]forbidden=15;
 robot_motion #(.AUTO_CLK_HZ(1000),
 .FEEDBACK_RECOVERY(1),.HOLD_CLKS(20),.RELEASE_CLKS(3),.RAMP_CLKS(2),
 .RECOVERY_START(10000),.RECOVERY_QUIET(10000),.RECOVERY_LIMIT(1000)) dut(
 .clk(clk),.rst_n(rst),.remote_claimed(1'b1),.remote_live(live),.remote_stop(stop),.remote_start_auto(start_auto),.remote_motion(motion),
 .pressed(4'd0),.keys_released(1'b1),.degraded(1'b0),.front(24'd800),.left(24'd500),.right(24'd500),
 .raw_mm({16'd500,16'd500,16'd800}),.valid(3'd7),.fresh(3'd7),.update(3'd7),.near_mask(3'd0),
 .sonar_packet(packet),.sonar_raw(raw),.enc_ready(4'hf),.window_end(windows),.ticks(ticks),
 .zero_mask(3'd0),.near_pending_mask(3'd0),.near_recover_mask(3'd0),.encoder_steps(4'd0),
 .targets(targets),.duties(duties),.actual(actual),.motor_pwm(pwm),.state_code(state),.stop_reason(reason),.auto_mode(automode),.auto_zones(zones),.hold_off(hold_off));
 always @(negedge clk)begin divider=(divider+1)%32;windows=publish_windows&&divider==0?15:0;end
 always @(posedge clk)begin #1;if(watching)begin
  if(actual==0&&targets==0&&pwm==0)zero_cycles=zero_cycles+1;
  if(actual==forbidden)forbidden_seen=1;
 end end
 task cycles;input integer n;begin repeat(n)@(negedge clk);#2;end endtask
 task check;input ok;input[1023:0]msg;begin checks=checks+1;if(ok!==1)begin
  $display("FAIL: %s actual=%0d state=%0d zones=%b reason=%0d zero=%0d target=%h",msg,actual,state,zones,reason,zero_cycles,targets);$fatal;end
 end endtask
 task sample;input[15:0]f,l,r;begin @(negedge clk);raw={r,l,f};packet=7;cycles(1);packet=0;cycles(2);end endtask
 task reset;begin rst=0;stop=0;start_auto=0;live=1;motion=0;packet=0;publish_windows=1;watching=0;zero_cycles=0;forbidden=15;forbidden_seen=0;raw={16'd500,16'd500,16'd800};cycles(4);rst=1;cycles(8);end endtask
 task start_clear;begin sample(800,500,500);sample(800,500,500);start_auto=1;cycles(1);start_auto=0;
  c=0;while(actual!=1&&c<200)begin cycles(1);c=c+1;end check(actual==1&&automode,"clear fresh sonar startsAUTO");cycles(180);
 end endtask
 task watch;input[3:0]bad;begin zero_cycles=0;forbidden_seen=0;forbidden=bad;watching=1;end endtask
 task wait_pivot;input[3:0]want;begin
  c=0;while(actual!=want&&c<48)begin cycles(1);c=c+1;end
  check(actual==want,"eligible pivot begins without100ms BRAKE or measured-stop wait");
  check(zero_cycles>0,"actual command targets andPWM were zero before newpivot");
  check(ticks!=0&&!publish_windows,"pivot selection does not require zero wheel counts or two stopped windows");
  check(!forbidden_seen,"confirmed blocked side never leaks a stale opposite pivot through pipeline");watching=0;
 end endtask
 initial begin
  // 仅前方近障时先清零再左转，不等待编码器停转，持续近障应被确认。
  reset;start_clear;publish_windows=0;
  sample(120,500,500);sample(120,500,500);check(actual==1,"two sudden near candidates alone do notconfirm");
  watch(6);sample(120,500,500);wait_pivot(5);
  for(j=0;j<12;j=j+1)begin sample(120,500,500);check(actual==5,"same acknowledged frontnear does not repeatedly stopturn");end
  // 左原地转过程中出现新的左侧近障，先清零再切可行右转。
  sample(120,80,500);sample(120,80,500);watch(15);sample(120,80,500);wait_pivot(6);
  for(j=0;j<8;j=j+1)begin sample(120,80,500);check(actual==6,"same front+leftnear permits continued safe rightpivot");end
  // 近障解除后再次确认形成新事件，即使仍选择同一转向。
  sample(120,200,500);sample(120,200,500);sample(120,80,500);sample(120,80,500);
  watch(5);sample(120,80,500);wait_pivot(6);
  // 前进中仅右侧近障时，先清零再左转，不转向被阻挡的右侧。
  reset;start_clear;publish_windows=0;sample(800,500,80);sample(800,500,80);watch(6);sample(800,500,80);wait_pivot(5);
  // 前进中仅左侧近障时，即使前方畅通也先清零再右转。
  reset;start_clear;publish_windows=0;sample(800,80,500);sample(800,80,500);watch(5);sample(800,80,500);wait_pivot(6);
  // 首次原地转仍处于20周期换向等待时，再到达第二个新近障。
  reset;sample(120,500,500);start_auto=1;cycles(1);start_auto=0;
  c=0;while(!(state==3&&hold_off)&&c<160)begin cycles(1);c=c+1;end
  check(state==3&&hold_off&&actual==0,"firstpivot genuinely queued inside original direction interlock");
  publish_windows=0;sample(120,80,500);sample(120,80,500);watch(5);sample(120,80,500);wait_pivot(6);
  // 后退使用真实近期前进历史，后退中解除再确认近障也需先清零。
  reset;start_clear;cycles(500);
  sample(120,80,80);sample(120,80,80);watch(15);sample(120,80,80);
  c=0;while(actual!=2&&c<100)begin cycles(1);c=c+1;end
  check(actual==2&&zero_cycles>0,"both unsafe sides skipforcedturn and use genuine history after zero command");watching=0;
  sample(120,200,80);sample(120,200,80);sample(120,80,80);sample(120,80,80);
  publish_windows=0;watch(15);sample(120,80,80);
  c=0;while(zero_cycles==0&&c<20)begin cycles(1);c=c+1;end
  check(zero_cycles>0,"newnearevent whilebacking clears actual targets andPWM too");
  check(automode,"ordinarynear duringreverse never cancelsAUTO");watching=0;
  stop=1;cycles(2);check(!automode&&actual==0&&pwm==0,"explicitSTOP overrides queuedAUTOnear recovery");
  // 无历史且三侧均近时不强制不安全转向，但保留任务。
  reset;sample(120,80,80);start_auto=1;cycles(1);start_auto=0;cycles(120);
  check(automode&&actual==0&&targets==0,"allunsafe withno recordedrear stays observedAUTO instead ofblindmotion");
  // AUTO的新近障事件处理不改变已有手动许可输入。
  reset;motion=3;cycles(160);check(actual==3&&targets[10:0]==53&&targets[21:11]==264,"retained manual53/264 curve established");
  sample(120,80,80);sample(120,80,80);sample(120,80,80);cycles(20);
  check(!automode&&actual==3&&targets[10:0]==53&&targets[21:11]==264,"AUTOnear event input doesnot change manualcurve");
  live=0;cycles(2);check(actual==0&&pwm==0,"commandlink loss stilloverrides manualoutput");
  $display("PASS: nearstop independent publicsystem any-channelzero-first, persistentack, new/reconfirmedchannel, reverse, interlock, nomotionstopwait, STOP andmanualisolation checks=%0d",checks);$finish;
 end
 initial begin #2000000;$fatal(1,"nearstop systemtimeout");end
endmodule
