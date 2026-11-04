`timescale 1ns/1ps
module tb_merge_remote;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,valid=0;reg[79:0]data=0;reg[3:0]keys=0;
 wire claimed,live,start_auto,force_stop;wire[3:0]motion;wire[31:0]status;wire[7:0]source;
 integer checks=0;
 phone_remote_control #(.WATCHDOG_CLKS(40)) dut(.clk(clk),.rst_n(rst),.cmd_valid(valid),.cmd_data(data),.local_pressed(keys),
  .claimed(claimed),.live(live),.motion(motion),.start_auto(start_auto),.force_stop(force_stop),.status(status),.stop_source(source));
 task clocks;input integer n;begin repeat(n)@(negedge clk);#1;end endtask
 task command;input[7:0]op,move;input[31:0]token,seq;
 begin @(negedge clk);data={op,move,token,seq};valid=1;clocks(1);valid=0;end endtask
 task check;input ok;input[1023:0]msg;
 begin checks=checks+1;if(ok!==1)begin $display("FAIL %s",msg);$fatal;end end endtask
 initial begin
  clocks(3);rst=1;clocks(3);
  command(1,0,123,1);check(claimed&&live&&force_stop&&motion==0,"claim starts stopped");clocks(1);
  command(2,0,123,2);check(start_auto&&!force_stop&&motion==0,"authenticated AUTO starts without emitting STOP");clocks(1);check(!start_auto,"start is one cycle");
  command(2,0,123,2);check(!start_auto,"replay cannot restart AUTO");
  command(2,0,124,3);check(!start_auto,"wrong token cannot start AUTO");
  command(4,0,123,3);check(live&&!start_auto&&!force_stop,"PING refreshes session without stopping AUTO");
  clocks(41);check(!live&&force_stop==0&&motion==0&&status[10],"watchdog expires after missing valid commands");
  command(2,0,123,4);check(!start_auto&&!live,"AUTO cannot revive an expired owner");
  command(1,0,124,1);check(claimed&&live&&force_stop,"new explicit claim recovers owner");
  command(3,7,124,2);check(motion==7&&!force_stop,"manual reverse-left keeps its original code");
  command(0,0,124,3);check(force_stop&&motion==0&&!start_auto,"STOP takes immediate priority");
  command(2,0,124,4);check(start_auto,"AUTO may be explicitly selected after STOP");
  keys=1;clocks(1);check(!claimed&&!live&&force_stop&&motion==0,"physical key revokes phone owner");
  $display("PASS: AUTO claim/token/replay/PING/watchdog/STOP/manual/local-key checks=%0d",checks);$finish;
 end
 initial begin #100000;$fatal(1,"remote test timeout");end
endmodule
