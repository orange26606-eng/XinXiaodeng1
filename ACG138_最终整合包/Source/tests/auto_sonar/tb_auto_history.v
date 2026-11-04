`timescale 1ns/1ps
module tb_auto_history;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,enable=0,plant=1;reg [2:0] packet=0;reg [47:0] raw=0;
 reg [3:0] actual=0,window_end=0;reg [63:0] ticks=0;
 wire [3:0] action,state,reason,attempts;wire [10:0] sl,sr;wire [1:0] light;
 integer divider=0,checks=0,c,lev,first_credit,back_runs=0;
 reg [3:0] previous_action=0;
 robot_auto_sonar #(.HOLD_CLKS(4),.ARC_CLKS(40),.TURN_CLKS(40),.OBSERVE_CLKS(6),
 .BACK_CLKS(400),.STALE_CLKS(10000),.LOST_CLKS(20000),.PENDING_CLKS(10000),
 .HISTORY_CLKS(20000),.MAX_CREDIT(320),.BACK_STEP(16),.RESTORE_TOL(1),.NEAR_TRAVEL(200),.TURN_PROGRESS_TICKS(16),.TURN_LIMIT_TICKS(32),.RESTORE_FINE_ZONE(16),.RESTORE_CROSS_TOL(8)) dut(
 .clk(clk),.rst_n(rst),.enable(enable),.packet(packet),.raw(raw),.actual(actual),.enc_ready(4'hf),
 .ticks(ticks),.window_end(window_end),.action(action),.state_code(state),.reason(reason),
 .speed_l(sl),.speed_r(sr),.attempts(attempts),.light(light));
 always @(negedge clk)begin
  divider=(divider+1)%4;window_end=divider==0?15:0;
  actual=plant?action:0;
  ticks=actual!=0?{4{16'd8}}:64'd0;
 end
 task cycles;input integer n;begin repeat(n)@(posedge clk);#1;end endtask
 task check;input ok;input [767:0] msg;begin
  checks=checks+1;if(ok!==1)begin
   $display("FAIL: %s state=%d actual=%d credit=%d pivot=%d level=%d timer=%d",msg,state,actual,dut.credit,dut.pivot_ticks,dut.back_level,dut.timer);$fatal;
  end
 end endtask
 task sample;input [15:0] f,l,r;begin
  @(negedge clk);raw={r,l,f};packet=7;cycles(1);@(negedge clk);packet=0;cycles(2);
 end endtask
 task reset;begin rst=0;enable=0;plant=1;packet=0;cycles(4);rst=1;cycles(4);end endtask
 task run_forward;begin
  sample(500,250,250);sample(500,250,250);enable=1;cycles(180);
  check(actual==1 && dut.credit>=200,"actual forward encoder windows create history");
 end endtask
 initial begin
  reset;sample(45,35,35);enable=1;cycles(100);
  check(dut.credit==0 && actual==0,"never reverse into unknown rear without encoder history");
  check(state==11||state==12,"no path observes and keeps AUTO enabled");
  reset;run_forward;first_credit=dut.credit;
  sample(45,35,35);sample(45,35,35);sample(45,35,35);
  for(lev=0;lev<4;lev=lev+1)begin
   c=0;while(!(action==2 && dut.back_level==lev)&&c<600)begin cycles(1);c=c+1;end
   check(c<600,"progressively larger reverse stage starts");
   check(dut.stage_limit==16*(lev+1),"reverse stage is100/200/300/400mm scaled");
   check(light==0,"near red severity coexists with active reverse");
   c=0;while(action==2&&c<400)begin cycles(1);c=c+1;end
   check(c<400,"each reverse has a finite executed distance/time budget");
  end
  cycles(1000);check(dut.credit<first_credit,"reverse consumes rather than renews forward history");
  check(dut.credit==0 && action==0 && enable,"credit exhausted remains AUTO, never blind unlimited reverse");
  sample(500,250,250);sample(500,250,250);cycles(30);
  check(action==1,"new clear samples restart AUTO without a new AUTO command");

  reset;run_forward;plant=0;
  sample(120,250,250);sample(120,250,250);sample(120,250,250);cycles(100);
  check(state==3 && dut.timer==0,"unexecuted pivot does not consume turn budget during interlock/feedback wait");
  plant=1;c=0;
  while(state!=13 && state!=14 && c<400)begin cycles(1);c=c+1;end
  check((state==13 || state==14) && dut.credit!=0 && dut.pivot_ticks!=0,"both failed physical sides restore heading before using rear history");
  c=0;while(action!=2 && c<400)begin cycles(1);c=c+1;end
  check(action==2 && dut.heading_aligned,"reverse begins only after actual encoder heading restoration");
  enable=0;cycles(2);check(action==0 && dut.credit==0,"disable cancels reverse and clears credit");
  $display("PASS: actual-history/progressive reverse/heading restoration/execution budgets checks=%0d",checks);$finish;
 end
 initial begin #2000000;$fatal(1,"history timeout");end
endmodule
