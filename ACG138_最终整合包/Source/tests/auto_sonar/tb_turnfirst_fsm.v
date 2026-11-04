`timescale 1ns/1ps
module tb_turnfirst_fsm;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0,moving=1,wrong_curve=0;reg [2:0] packet=0;reg [47:0] raw=0;
 reg [3:0] actual=0,window_end=0;reg [63:0] ticks=0;
 wire [3:0] action,state,reason,attempts;wire [10:0] sl,sr;wire [1:0] light;
 wire [47:0] ranges;wire [5:0] zones;wire [2:0] usable,fresh,pending;
 integer checks=0,divider=0,i,first_credit;reg seen_left=0,seen_right=0;
 robot_auto_sonar #(.HOLD_CLKS(4),.ARC_CLKS(20),.TURN_CLKS(8),.OBSERVE_CLKS(8),.BACK_CLKS(80),
  .STALE_CLKS(20000),.LOST_CLKS(40000),.PENDING_CLKS(200),.HISTORY_CLKS(10000),
  .MAX_CREDIT(320),.BACK_STEP(16),.NEAR_TRAVEL(186),.RESTORE_TOL(0),
  .TURN_PROGRESS_TICKS(8),.TURN_LIMIT_TICKS(25),.RESTORE_FINE_ZONE(8),.RESTORE_CROSS_TOL(4)) dut(
  .clk(clk),.rst_n(rst),.enable(enable),.packet(packet),.raw(raw),.actual(actual),.enc_ready(4'hf),
  .ticks(ticks),.window_end(window_end),.action(action),.state_code(state),.reason(reason),.speed_l(sl),.speed_r(sr),
  .attempts(attempts),.light(light),.ranges(ranges),.zones(zones),.usable(usable),.fresh(fresh),.pending(pending));
 always @(negedge clk)begin
  divider=(divider+1)%4;window_end=divider==0?15:0;actual=action;
  ticks=(moving && actual!=0)?{4{16'd3}}:0;
  if(moving && !wrong_curve && actual==3)ticks={16'd7,16'd3,16'd7,16'd3};
  if(moving && !wrong_curve && actual==4)ticks={16'd3,16'd7,16'd3,16'd7};
  if(actual==5)seen_left=1;if(actual==6)seen_right=1;
 end
 task cycles;input integer n;begin repeat(n)@(posedge clk);#1;end endtask
 task check;input ok;input [1023:0] msg;begin checks=checks+1;if(ok!==1)begin
  $display("FAIL: %0s state=%d action=%d tried=%b%b pivot=%d credit=%d attempts=%d",msg,state,action,dut.tried_left,dut.tried_right,dut.pivot_ticks,dut.credit,attempts);$fatal;
 end end endtask
 task sample;input [15:0] f,l,r;begin @(negedge clk);raw={r,l,f};packet=7;cycles(1);@(negedge clk);packet=0;cycles(2);end endtask
 task reset;begin rst=0;enable=0;packet=0;moving=1;wrong_curve=0;seen_left=0;seen_right=0;cycles(4);rst=1;cycles(3);end endtask
 task wait_action;input [3:0] wanted;input integer maximum;begin
  i=0;while(action!=wanted && i<maximum)begin cycles(1);i=i+1;end
  check(action==wanted,"expected action reached within bounded test window");
 end endtask
 initial begin
  // 实际实例化自动阈值，不继承旧手动近障锁存。
  reset;sample(400,180,180);enable=1;wait_action(1,40);check(sl==110&&sr==110,"AUTO normal50RPM target110 counts");
  sample(399,179,179);cycles(20);check(action==3&&sl==33&&sr==77,"front warning proactively chooses15/35RPM arc");
  // ARC_CLKS是重新评估周期，不是每500ms强制STOP。
  repeat(120)begin cycles(1);check(action==3,"legal warning arc remains continuous across evaluation periods");end
  check(dut.credit>0,"measured3:7ratio arc establishes bounded matching geometry history");
  wrong_curve=1;cycles(16);check(dut.credit==0&&dut.history_action==0,"equal wheel feedback during arc invalidates history instead of claiming curved clearance");
  reset;sample(80,300,300);enable=1;wait_action(5,40);check(sl==88&&sr==88,"AUTO pivot40RPM target88 counts");
  wait_action(6,120);check(seen_left && dut.tried_left,"first physical left sweep precedes opposite side");
  check(dut.pivot_ticks>=25,"first side reached real encoder angle before marking failed");
  wait_action(0,180);check(seen_right,"opposite physical side actually attempted");
  // 只有许可原因码超时，不能把未实际运动的轮子计为已探索空间。
  reset;moving=0;sample(80,300,300);enable=1;wait_action(5,40);cycles(400);
  check(action==5&&attempts==0&&!dut.tried_left&&!dut.tried_right,"zero encoder feedback does not fake turn completion");
  check(dut.credit==0,"no movement cannot manufacture reverse history");
  // 实际未完成20度进展前，仅两笔前方样本不能恢复前进。
  reset;moving=0;sample(80,300,300);enable=1;wait_action(5,40);sample(500,300,300);sample(500,300,300);cycles(10);
  check(action==5,"front clear samples alone do not skip physical turn progress");
  moving=1;wait_action(1,80);check(sl==55&&sr==55,"two fresh250mm samples plus actual angle permit25RPM recovery");
  sample(500,300,300);sample(500,300,300);cycles(20);check(action==1&&sl==110,"normal clearance restores50RPM");
  sample(80,300,300);sample(80,300,300);sample(80,300,300);wait_action(5,40);
  check(!dut.tried_left&&!dut.tried_right,"new obstruction does not inherit prior failed-side flags");
  // 侧向候选超时保留原普通区域分类，但不能选择被阻挡直行。
  reset;sample(500,300,300);enable=1;wait_action(1,40);sample(500,20,300);
  repeat(70)sample(500,0,300);
  wait_action(4,40);check(dut.pending_expired[1]&&pending[1]&&zones[3:2]==0,"expired left candidate preserves old zone but selects away-right arc");
  repeat(100)begin cycles(1);check(action==4,"awayright arc stays continuous while oppositeleft candidate remains pending");end
  reset;sample(500,300,300);enable=1;wait_action(1,40);sample(500,300,20);
  repeat(70)sample(500,300,0);
  wait_action(3,40);check(dut.pending_expired[2]&&pending[2]&&zones[5:4]==0,"expired right candidate preserves old zone but selects away-left arc");
  repeat(100)begin cycles(1);check(action==3,"awayleft arc stays continuous while oppositeright candidate remains pending");end
  // 真正不安全侧直接跳过，不要求实际碰撞后才记已尝试。
  reset;sample(80,30,300);enable=1;wait_action(6,40);check(!seen_left,"left near does not prevent safe right pivot");
  reset;sample(80,300,30);enable=1;wait_action(5,40);check(!seen_right,"right near does not prevent safe left pivot");
  reset;sample(80,30,30);enable=1;cycles(80);check(action==0&&dut.credit==0,"both unsafe with unknown rear does not reverse blindly");
  check(state==11||state==12,"no-path remains in observation/retry instead of cancelling AUTO");
  // 实际前进行程建立后退额度，后退之前先扫过两侧可行方向。
  reset;sample(500,300,300);enable=1;wait_action(1,40);cycles(220);first_credit=dut.credit;
  check(first_credit>80,"actual encoder windows establish rear credit");
  seen_left=0;seen_right=0;sample(80,300,300);sample(80,300,300);sample(80,300,300);
  wait_action(5,40);first_credit=dut.credit;
  i=0;while(action!=2&&i<700)begin
   if(action==2)check(seen_left&&seen_right,"both turns before reverse");cycles(1);i=i+1;
  end
  check(action==2&&seen_left&&seen_right,"two failed physical sweeps restore heading then reverse");
  check(dut.pivot_ticks==0,"bounded fine-heading crossing establishes restored baseline");
  check(sl==66&&sr==66,"reverse30RPM counts66");
  check(dut.credit<=first_credit,"pivot does not add translation credit");
  cycles(60);check(dut.credit<first_credit,"only executed reverse consumes history");
  enable=0;cycles(4);check(action==0&&state==0&&dut.credit==0,"AUTO cancellation immediately clears autonomous action and history");
  $display("PASS: turnfirst public-input FSM order, physical-turn confirmation, no fake progress, continuous arc, new episode, two-sample recovery, side specificity, bounded history restore and speeds checks=%0d",checks);$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
