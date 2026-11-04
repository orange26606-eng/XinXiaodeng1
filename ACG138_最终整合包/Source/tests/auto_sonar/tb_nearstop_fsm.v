`timescale 1ns/1ps
// 直接通过公开端口测试近障协议，命令清零确认不是车轮停止反馈。
module tb_nearstop_fsm;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0;reg [2:0] packet=0;reg [47:0] raw=0;
 reg [3:0] pipe0=0,pipe1=0,actual=0,window_end=0;reg [63:0] ticks={4{16'd88}};
 wire [3:0] action,state,reason,attempts;wire [10:0] sl,sr;wire [1:0] light;
 wire [47:0] ranges;wire [5:0] zones;wire [2:0] usable,fresh,pending;
 integer checks=0,divider=0,n;
 robot_auto_sonar #(.HOLD_CLKS(50),.ARC_CLKS(1000),.TURN_CLKS(400),.OBSERVE_CLKS(20),.BACK_CLKS(1000),
 .STALE_CLKS(20000),.LOST_CLKS(40000),.PENDING_CLKS(10000),.HISTORY_CLKS(20000),
 .TURN_PROGRESS_TICKS(315),.TURN_LIMIT_TICKS(5000)) dut(
 .clk(clk),.rst_n(rst),.enable(enable),.packet(packet),.raw(raw),.actual(actual),.enc_ready(4'hf),
 .ticks(ticks),.window_end(window_end),.action(action),.state_code(state),.reason(reason),.speed_l(sl),.speed_r(sr),
 .attempts(attempts),.light(light),.ranges(ranges),.zones(zones),.usable(usable),.fresh(fresh),.pending(pending));
 always @(posedge clk)begin
  if(!rst)begin pipe0<=0;pipe1<=0;actual<=0;end
  else begin pipe0<=action;pipe1<=pipe0;actual<=pipe1;end
 end
 always @(negedge clk)begin divider=(divider+1)%4;window_end=divider==0?15:0;end
 task cycles;input integer k;begin repeat(k)@(posedge clk);#1;end endtask
 task check;input ok;input [1023:0] msg;begin checks=checks+1;if(ok!==1)begin
  $display("FAIL: %0s state=%d action=%d actual=%d ack=%b new=%b zone=%b timer=%d",msg,state,action,actual,dut.near_ack,dut.near_new,zones,dut.timer);$fatal;
 end end endtask
 task reset;begin rst=0;enable=0;packet=0;cycles(4);rst=1;cycles(3);end endtask
 task sample;input [15:0] f,l,r;begin @(negedge clk);raw={r,l,f};packet=7;cycles(1);@(negedge clk);packet=0;cycles(2);end endtask
 task wait_action;input [3:0] wanted;input integer maximum;begin
  n=0;while(action!=wanted&&n<maximum)begin cycles(1);n=n+1;end
  check(action==wanted,"expected action within command-clear bound");
 end endtask
 initial begin
  reset;sample(350,300,300);enable=1;wait_action(3,80);check(sl==33&&sr==77,"warning arc15/35RPM");cycles(8);
  sample(350,300,80);sample(350,300,80);sample(350,300,80);
  check(action==0&&sl==0&&sr==0,"new rightnear clears even an already-away left arc before choosing pivot");
  wait_action(5,12);check(n<12&&dut.timer<50,"newnear pivot starts after commandclear without fixedHOLD wait");
  check(ticks=={4{16'd88}},"no zero wheel-speed measurement was supplied or required");
  reset;sample(500,300,300);enable=1;wait_action(1,80);cycles(8);
  sample(120,300,300);sample(120,300,300);sample(120,300,300);
  check(action==0,"confirmed frontnear first produces zero-demand step");wait_action(5,12);
  repeat(5)begin sample(120,300,300);check(action==5,"same acknowledged frontnear does not brake every packet");end
  check(dut.near_ack[0]&&!dut.near_new[0],"frontnear episode acknowledged per channel");
  sample(120,80,300);sample(120,80,300);sample(120,80,300);
  check(action==0,"new left channel brakes while old frontnear remains acknowledged");wait_action(6,12);
  check(dut.near_ack[1:0]==3&&action==6,"right pivot allowed with acknowledged front-left obstacles");
  sample(120,130,300);sample(120,130,300);cycles(2);check(!dut.near_ack[1],"two new130mm side samples release and rearm that channel");
  sample(120,80,300);sample(120,80,300);sample(120,80,300);
  check(action==0,"released then reconfirmed same side triggers another zero-demand step");wait_action(6,12);
  // 已有BRAKE且到达首侧转角边界时，另一通道首次确认近障。
  // 新可行原地转存在时，旧已尝试标记不能强迫后退或观察。
  reset;sample(120,300,300);enable=1;wait_action(5,80);
  sample(120,300,80);sample(120,300,80);
  n=0;while(state!=12&&n<1000)begin cycles(1);n=n+1;end
  check(state==12&&dut.tried_left,"first left trial completed before an overlapping new-channel confirmation");
  sample(120,300,80);check(action==0,"new rightnear confirming insideBRAKE keeps zero command");
  wait_action(5,12);check(!dut.tried_left&&!dut.tried_right,"fresh near episode resets old trial flags even when first detected inBRAKE");
  // 恢复阈值250mm，与近障锁存解除的200mm分开。
  reset;sample(120,300,300);enable=1;wait_action(5,80);cycles(24);
  sample(249,300,300);sample(249,300,300);cycles(8);
  check(zones[1:0]==1&&action==5,"front249 releases near latch but cannot resume forward");
  sample(250,300,300);check(action==5,"one new250 sample is insufficient");
  sample(250,300,300);wait_action(1,16);check(sl==55&&sr==55,"two fresh250 plus actual20deg permit25RPM forward recovery");
  enable=0;cycles(1);check(action==0,"disable always overrides near event and recovery");
  $display("PASS: nearstop per-channel event clear, away-arc brake before pivot, persistent acknowledgment, new/reconfirmed side, no timed or encoderstop wait,250mm two-sample recovery checks=%0d",checks);$finish;
 end
 initial begin #4000000;$fatal(1,"timeout");end
endmodule

module tb_nearstop_context;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0,drain_delay=0;reg [2:0] packet=0;reg [47:0] raw=0;
 reg [3:0] p0=0,p1=0,actual=0,window_end=0;wire [3:0] action,state,reason,attempts;wire [10:0] sl,sr;
 wire [47:0] ranges;wire [5:0] zones;wire [2:0] usable,fresh,pending;wire [1:0] light;
 integer checks=0,n,divider=0;
 robot_auto_sonar #(.HOLD_CLKS(4),.ARC_CLKS(1000),.TURN_CLKS(400),.OBSERVE_CLKS(8),.BACK_CLKS(1000),
 .STALE_CLKS(60),.LOST_CLKS(80),.PENDING_CLKS(20),.HISTORY_CLKS(20000),.TURN_LIMIT_TICKS(5000)) dut(
 .clk(clk),.rst_n(rst),.enable(enable),.packet(packet),.raw(raw),.actual(actual),.enc_ready(4'hf),
 .ticks({4{16'd88}}),.window_end(window_end),.action(action),.state_code(state),.reason(reason),.speed_l(sl),.speed_r(sr),
 .attempts(attempts),.light(light),.ranges(ranges),.zones(zones),.usable(usable),.fresh(fresh),.pending(pending));
 always @(posedge clk)begin if(!rst)begin p0<=0;p1<=0;actual<=0;end else begin p0<=action;p1<=p0;actual<=drain_delay?4'd4:p1;end end
 always @(negedge clk)begin divider=(divider+1)%4;window_end=divider==0?15:0;end
 task cycles;input integer k;begin repeat(k)@(posedge clk);#1;end endtask
 task check;input ok;input [1023:0] msg;begin checks=checks+1;if(ok!==1)begin
  $display("FAIL: %0s state=%d action=%d actual=%d age=%d usable=%b pendingflag=%b nearack=%b",msg,state,action,actual,dut.cf.age,usable,dut.near_brake_pending,dut.near_ack);$fatal;
 end end endtask
 task sample;input [15:0] f,l,r;input [2:0] mask;begin @(negedge clk);raw={r,l,f};packet=mask;cycles(1);@(negedge clk);packet=0;cycles(2);end endtask
 initial begin
  rst=0;cycles(4);rst=1;cycles(3);sample(500,300,300,7);enable=1;cycles(12);
  check(action==1,"context test begins with public normal movement");
  sample(0,80,0,2);sample(0,80,0,2);
  // 前方达到实际失联年龄期间，其他通道保持更新，不强制修改内部状态。
  @(negedge clk);raw={16'd300,16'd0,16'd0};packet=4;
  while(dut.cf.age<76)cycles(1);
  // 下游公开端口模型在年龄边界仍保留上一非零命令。
  // 独立于当前流水线阶段检查上下文清理。
  drain_delay=1;
  @(negedge clk);packet=0;
  sample(0,80,0,2);
  check(dut.near_brake_pending&&state==12,"new side event overlaps front expiry during nonzero command drain");
  cycles(4);check(!usable[0]&&state==0&&!dut.near_brake_pending,"readiness loss discards stale unacknowledged force-pivot flag");
  drain_delay=0;
  // 恢复前方前先解除侧向近障，保证当前确无新的近障事件。
  sample(0,300,300,6);sample(0,300,300,6);
  sample(500,0,0,1);cycles(10);check(action==1&&!dut.near_brake_pending,"allclear context recovery returns normal without a ghost near task");
  sample(500,50,300,7);
  repeat(10)sample(500,0,300,7);
  check(pending[1]&&dut.pending_expired[1]&&action==4,"later unconfirmed side timeout selects away arc rather than stale forced pivot");
  enable=0;cycles(1);check(action==0,"disable clears context recovery motion");
  $display("PASS: public age-edge readiness loss clears near-task pending, allclear recovers, later ordinary pending remains awayarc checks=%0d",checks);$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
