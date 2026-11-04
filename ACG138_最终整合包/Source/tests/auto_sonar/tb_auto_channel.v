`timescale 1ns/1ps
module tb_auto_channel;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,packet=0;reg [15:0] raw=0;
 wire [15:0] distance;wire usable,fresh,pending,accepted,expired;
 wire [1:0] zone;
 integer checks=0,i;
 robot_auto_sonar_channel #(.STALE_CLKS(20),.LOST_CLKS(40),.PENDING_CLKS(64)) dut(
  .clk(clk),.rst_n(rst),.packet(packet),.raw(raw),.distance(distance),
  .usable(usable),.fresh(fresh),.pending(pending),.zone(zone),.accepted(accepted),.pending_expired(expired));
 task cycles;input integer n;begin repeat(n)@(negedge clk);#1;end endtask
 task check;input ok;input [767:0] text;begin checks=checks+1;if(ok!==1)begin
  $display("FAIL: %0s raw=%0d distance=%0d zone=%0d pending=%0b expired=%0b usable=%0b",text,raw,distance,zone,pending,expired,usable);$fatal;
 end end endtask
 task reset;begin rst=0;packet=0;raw=0;cycles(3);rst=1;cycles(2);end endtask
 task sample;input [15:0] n;begin raw=n;packet=1;cycles(1);packet=0;cycles(2);end endtask
 initial begin
  reset;check(!usable,"no first nonzero sample is not clearance");
  sample(0);sample(0);check(!usable,"zero cannot initialize a trusted range");
  reset;sample(400);check(usable&&zone==0&&distance==400,"front400 inclusive normal boundary");
  reset;sample(399);check(zone==1&&distance==399,"front399 warning");
  reset;sample(150);check(zone==1&&distance==150,"front150 warning inclusive");
  reset;sample(149);check(zone==2&&distance==149,"front149 initial near");
  reset;sample(25);check(zone==2&&distance==25,"sub30 stays numeric ordinary near, not blind zero");
  reset;sample(500);sample(500);sample(500);sample(50);
  check(zone==0&&pending&&distance==500,"isolated sudden-near held for confirmation");
  sample(0);sample(0);check(zone==0&&pending&&distance==500,"zero neither confirms nor clears candidate");
  sample(50);check(zone==0&&pending,"only two new near samples do not confirm");
  sample(50);check(zone==2&&!pending&&distance==50,"third new near confirms");
  sample(0);check(zone==2&&distance==50,"zero does not release confirmed near");
  sample(199);sample(199);check(zone==2,"front199 below release margin");
  sample(200);check(zone==2,"one release sample cannot clear near");
  sample(200);check(zone==1&&distance==200,"two front200 samples release to warning");
  sample(400);check(zone==1,"one normal sample does not release warning");
  sample(400);check(zone==0,"two normal samples return normal");
  reset;sample(500);sample(40);sample(40);sample(500);
  check(zone==0&&!pending&&distance>=400,"rejected near candidates never poison far median");
  reset;sample(500);sample(350);sample(220);sample(170);sample(50);
  check(zone==2&&distance==50,"genuine decreasing approach bypasses sudden-jump delay");
  reset;sample(500);sample(50);
  repeat(24)sample(0);
  check(usable&&fresh&&distance==500,"zero heartbeat preserves accepted distance and link");
  check(expired,"zero cannot restart pending confirmation deadline");
  reset;sample(500);cycles(21);check(!fresh&&usable,"link warning before true lost");
  cycles(21);check(!usable,"no packets causes real lost");
  sample(500);sample(500);check(usable&&fresh,"new packets recover automatically from lost");
  $display("PASS: AUTO front cache boundaries, zero semantics, sudden-near qualification, trend fast path, hysteresis, no median pollution and real link recovery checks=%0d",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule

module tb_auto_side_channel;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,packet=0;reg [15:0] raw=0;wire [15:0] distance;
 wire usable,fresh,pending,accepted;wire [1:0] zone;
 integer checks=0;
 robot_auto_sonar_channel #(.NORMAL_MM(180),.NEAR_MM(100),.RELEASE_MM(130),.STALE_CLKS(100),.LOST_CLKS(200)) dut(
  .clk(clk),.rst_n(rst),.packet(packet),.raw(raw),.distance(distance),.usable(usable),.fresh(fresh),.pending(pending),.zone(zone),.accepted(accepted));
 task cycles;input integer n;begin repeat(n)@(negedge clk);#1;end endtask
 task check;input ok;input [511:0] text;begin checks=checks+1;if(ok!==1)begin $display("FAIL: %0s distance=%0d zone=%0d",text,distance,zone);$fatal;end end endtask
 task reset;begin rst=0;packet=0;raw=0;cycles(3);rst=1;cycles(2);end endtask
 task sample;input [15:0] n;begin raw=n;packet=1;cycles(1);packet=0;cycles(2);end endtask
 initial begin
  reset;sample(180);check(zone==0,"side180 normal inclusive");
  reset;sample(179);check(zone==1,"side179 warning");
  reset;sample(100);check(zone==1,"side100 warning inclusive");
  reset;sample(99);check(zone==2&&distance==99,"side99 near");
  sample(129);sample(129);check(zone==2,"side129 not released");
  sample(130);check(zone==2,"one side130 cannot release");sample(130);check(zone==1,"two side130 release near");
  sample(180);check(zone==1,"one side180 cannot release warning");sample(180);check(zone==0,"two side180 normal");
  $display("PASS: AUTO side180/100 boundaries and130mm release hysteresis checks=%0d",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
