`timescale 1ns/1ps
// 长时回归仅驱动公开端口，累加器状态只读观察。
module tb_turnfirst_wrap;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0,active=0;reg[2:0]packet=0;reg[47:0]raw={16'd300,16'd30,16'd80};
 reg[3:0]actual=0,window_end=0;reg[63:0]ticks=0;
 wire[3:0]action,state,reason,attempts;wire[10:0]sl,sr;
 integer divider=0,c,checks=0,observe_entries=0,last_state=0;reg signed_wrap=0,whole_wrap=0;
 reg signed[17:0]previous_pivot=0;
 robot_auto_sonar #(.HOLD_CLKS(4),.ARC_CLKS(20),.OBSERVE_CLKS(8),.BACK_CLKS(80),
  .STALE_CLKS(20000),.LOST_CLKS(40000),.PENDING_CLKS(200),.HISTORY_CLKS(10000)) dut(
  .clk(clk),.rst_n(rst),.enable(enable),.packet(packet),.raw(raw),.actual(actual),.enc_ready(4'hf),
  .ticks(ticks),.window_end(window_end),.action(action),.state_code(state),.reason(reason),.attempts(attempts),.speed_l(sl),.speed_r(sr));
 always @(negedge clk)begin
  divider=(divider+1)%4;window_end=divider==0?15:0;actual=action;
  ticks=action!=0?{4{16'd512}}:0;packet=active&&divider==0?7:0;
 end
 always @(posedge clk)begin
  #1;
  if(rst&&enable)begin
   if(previous_pivot< -18'sd120000 && dut.pivot_ticks>18'sd120000)signed_wrap=1;
   if(signed_wrap && previous_pivot>=0 && dut.pivot_ticks<0)whole_wrap=1;
   previous_pivot=dut.pivot_ticks;
   if(state==11 && last_state!=11)observe_entries=observe_entries+1;
   last_state=state;
  end
 end
 task cycles;input integer n;begin repeat(n)@(negedge clk);#2;end endtask
 task check;input ok;input[1023:0]msg;begin checks=checks+1;if(ok!==1)begin
  $display("FAIL: %s action=%0d state=%0d pivot=%0d origin=%0d delta=%0d observe=%0d",msg,action,state,dut.pivot_ticks,dut.turn_origin,dut.turn_travel,observe_entries);$fatal;end
 end endtask
 initial begin
  cycles(4);rst=1;active=1;cycles(8);enable=1;
  c=0;while(!whole_wrap&&c<20000)begin cycles(1);c=c+1;end
  check(signed_wrap,"repeated physical single-side turns cross signed18-bit boundary without internal injection");
  check(whole_wrap,"continued public encoder windows cross a complete18-bit cycle");
  check(observe_entries>=100,"physical turns continue completing and retrying after many obstacle episodes");
  check(dut.credit==0,"rotations never manufacture reverse translation credit");
  c=0;while((action!=6 || dut.turn_travel>=512)&&c<200)begin cycles(1);c=c+1;end
  check(action==6,"a new effective turn remains available after accumulator wrap");
  raw={16'd300,16'd30,16'd300};c=0;
  while(action!=4&&c<150)begin cycles(1);c=c+1;end
  check(action==4 && sl==77 && sr==33,"two fresh front-clear samples plus real turn progress recover to right forward arc after wrap");
  cycles(16);check(dut.pivot_ticks==0,"new actual forward segment establishes a fresh heading baseline");
  enable=0;cycles(3);check(action==0&&state==0,"user cancellation still stops long-running exploration");
  $display("PASS: turnfirst public-input signed/full18-bitwrap, continuous attempts, fresh-front recovery and new baseline checks=%0d observed_episodes=%0d",checks,observe_entries);$finish;
 end
 initial begin #1000000;$fatal(1,"longrun wrap timeout");end
endmodule
