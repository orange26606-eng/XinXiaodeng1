`timescale 1ns/1ps
// 传感器不确定只拒绝受影响方向的几何，不拒绝全部方向。
// 固定规模模型在一周期后回报已接受动作，不使用force或release。
module tb_auto_directional;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0;reg [2:0] packet=0;reg [47:0] raw=0;reg [3:0] actual=0;
 wire [3:0] action,state_code,reason,attempts;wire [10:0] speed_l,speed_r;
 wire [47:0] ranges;wire [5:0] zones;wire [2:0] usable,fresh,pending;wire [1:0] light;
 integer checks=0,i;
 robot_auto_sonar #(.HOLD_CLKS(4),.ARC_CLKS(200),.TURN_CLKS(100),.OBSERVE_CLKS(8),
 .BACK_CLKS(100),.STALE_CLKS(100),.LOST_CLKS(200),.PENDING_CLKS(20),.HISTORY_CLKS(1000)) dut(
 .clk(clk),.rst_n(rst),.enable(enable),.packet(packet),.raw(raw),.actual(actual),
 .enc_ready(4'hf),.ticks(64'd0),.window_end(4'd0),.action(action),.state_code(state_code),
 .reason(reason),.speed_l(speed_l),.speed_r(speed_r),.attempts(attempts),.light(light),
 .ranges(ranges),.zones(zones),.usable(usable),.fresh(fresh),.pending(pending));
 always @(posedge clk)if(!rst)actual<=0;else actual<=action;
 task cycles;input integer n;begin repeat(n)@(negedge clk);#1;end endtask
 task check;input ok;input [767:0] text;begin checks=checks+1;if(ok!==1)begin
  $display("FAIL: %0s act=%0d state=%0d zone=%b pending=%b expired=%b ranges=%h",text,action,state_code,zones,pending,dut.pending_expired,ranges);$fatal;
 end end endtask
 task sample;input [15:0] f,l,r;input [2:0] mask;begin raw={r,l,f};packet=mask;cycles(1);packet=0;cycles(2);end endtask
 task start_clear;begin
  rst=0;enable=0;packet=0;raw=0;cycles(3);rst=1;cycles(2);
  sample(500,250,250,7);sample(500,250,250,7);enable=1;cycles(8);
  check(action==1,"initial clear task is moving straight");
 end endtask
 initial begin
  start_clear;
  sample(500,20,250,7);check(pending[1]&&zones[3:2]==0,"single left jump stays an unconfirmed candidate");
  repeat(10)sample(500,0,250,7);cycles(8);
  check(dut.pending_expired[1]&&pending[1]&&ranges[31:16]==250,"left zero heartbeat does not clear/restart pending deadline or overwrite distance");
  check(action==4&&speed_l>speed_r,"left pending timeout permits away RIGHT forward arc");
  check(!dut.left_go&&dut.right_go&&dut.front_go,"left pending affects left geometry, not front/right permission");
  enable=0;cycles(1);check(action==0,"STOP cuts an away arc immediately");

  start_clear;
  sample(500,250,20,7);repeat(10)sample(500,250,0,7);cycles(8);
  check(dut.pending_expired[2]&&pending[2]&&ranges[47:32]==250,"right candidate timeout survives zero heartbeats");
  check(action==3&&speed_l<speed_r,"right pending timeout permits away LEFT forward arc");
  check(dut.left_go&&!dut.right_go&&dut.front_go,"right pending affects right geometry, not front/left permission");

  start_clear;
  sample(20,250,250,7);repeat(10)sample(0,250,250,7);cycles(8);
  check(dut.pending_expired[0]&&!dut.front_go,"front pending timeout rejects additional forward translation");
  check(action==5||action==6,"front pending timeout still permits a side pivot");
  check(speed_l==88&&speed_r==88,"pending-front recovery pivot uses40RPM targets");

  start_clear;
  sample(500,25,250,7);sample(500,25,250,7);sample(500,25,250,7);cycles(8);
  check(zones[3:2]==2&&action==6,"confirmed LEFT near first clears then selects away RIGHT pivot");
  check(light==0&&action!=0,"near/red severity is independent of an active recovery action");

  start_clear;
  sample(500,250,25,7);sample(500,250,25,7);sample(500,250,25,7);cycles(8);
  check(zones[5:4]==2&&action==5,"confirmed RIGHT near first clears then selects away LEFT pivot");
  check(light==0&&action!=0,"red near severity does not mean AUTO cancelled/stopped");

  start_clear;
  sample(45,25,250,7);sample(45,25,250,7);sample(45,25,250,7);cycles(8);
  check(zones[1:0]==2&&zones[3:2]==2&&action==6,"front and LEFT near still permit away RIGHT pivot");

  // 侧面真正失联时只保留之前已确认净空的直行走廊，
  // 即使前方和右方持续更新，也不新增朝失联侧的转向。
  start_clear;
  repeat(72)sample(500,250,250,5);cycles(8);
  check(!usable[1]&&usable[0]&&usable[2],"only left link is lost");
  check(action==1&&speed_l==55&&speed_r==55,"single previously-clear lost side does not impose a global stop");
  check(!dut.left_go,"cannot initiate a turn towards a genuinely lost side");
  enable=0;cycles(1);check(action==0&&speed_l==0&&speed_r==0,"STOP overrides directional recovery and lost-side hold");
  $display("PASS: directional pending expiry, zero heartbeat, front-only forward block, away arcs/pivots for true near, red moving recovery, single-lost-side hold and STOP checks=%0d",checks);$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
