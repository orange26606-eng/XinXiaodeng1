`timescale 1ns/1ps
module tb_merge_uart;
 reg clk=0;always #10 clk=~clk;
 reg[4:0]keys=15;reg[2:0]rx=7;reg cmd=0;reg[79:0]data=0;
 wire[111:0]telemetry;wire[3:0]pwm,in1,in2;wire[7:0]led;
 integer checks=0,cycles_wait=0;
 robot_control #(.CLK_HZ(10000),.FEEDBACK_RECOVERY(0),.TICK_CLKS(1),.KEY_CLKS(2),.WINDOW_CLKS(1000),.REPORT_CLKS(500),.HOLD_CLKS(4),.RAMP_CLKS(2),.NO_FEEDBACK_CLKS(1000000)) dut(
 .clk(clk),.front_rx(rx[0]),.left_rx(rx[1]),.right_rx(rx[2]),.key_n(keys),.enc_a(4'd0),.enc_b(4'd0),
 .remote_valid(cmd),.remote_data(data),.nav_claimed(1'b0),.nav_armed(1'b0),.nav_rpm(64'd0),.nav_intent(8'd0),
 .telemetry_data(telemetry),.motor_in1(in1),.motor_in2(in2),.motor_pwm(pwm),.led(led));
 task clocks;input integer n;begin repeat(n)@(negedge clk);#1;end endtask
 task check;input ok;input[1023:0]label;begin checks=checks+1;if(ok!==1)begin $display("FAIL %s telemetry=%h",label,telemetry);$fatal;end end endtask
 task command;input[7:0]op;input[31:0]command_seq;begin @(negedge clk);data={op,8'd0,32'd123,command_seq};cmd=1;clocks(1);cmd=0;clocks(4);end endtask
 task bytes;input[7:0]f,l,r;integer bitnum;begin
  rx=0;clocks(16);
  for(bitnum=0;bitnum<8;bitnum=bitnum+1)begin rx={r[bitnum],l[bitnum],f[bitnum]};clocks(16);end
  rx=7;clocks(16);
 end endtask
 task frame;input[15:0]f,l,r;input corrupt;reg[7:0]fc,lc,rc;begin
  fc=8'hff+f[15:8]+f[7:0];lc=8'hff+l[15:8]+l[7:0];rc=8'hff+r[15:8]+r[7:0];
  bytes(255,255,255);bytes(f[15:8],l[15:8],r[15:8]);bytes(f[7:0],l[7:0],r[7:0]);bytes(corrupt?fc^8'h40:fc,lc,rc);clocks(15);
 end endtask
 initial begin
  keys=15;clocks(4);keys=31;clocks(200);frame(800,500,500,0);frame(800,500,500,0);
  command(1,1);command(2,2);clocks(1100);
  check(telemetry[60]&&telemetry[64:61]==1,"AUTO enters through remote command and real UART packets");
  frame(120,500,500,0);frame(120,500,500,0);frame(120,500,500,0);clocks(25);
  check(telemetry[60]&&telemetry[64:61]==5,"confirmed raw front-near selects left pivot through unchanged motor pipeline");
  check(telemetry[15:0]==120,"AUTO telemetry uses its trusted raw-packet cache");
  frame(0,500,500,0);clocks(5);
  check(telemetry[15:0]==120&&telemetry[60]&&telemetry[64:61]==5,"zero packet refreshes link but neither range nor near episode");
  frame(900,500,500,1);clocks(5);
  check(telemetry[15:0]==120&&telemetry[64:61]==5,"bad checksum never reaches AUTO raw-packet tap");
  clocks(16000);check(telemetry[60]&&telemetry[64:61]==0,"true sensor silence stops output while retaining AUTO task");
  command(0,3);clocks(5);check(!telemetry[60]&&pwm==0,"explicit STOP cancels AUTO at full-control boundary");
  $display("PASS: full robot_control UART/checksum/tap/cache/near/zero/loss/remoteSTOP checks=%0d",checks);$finish;
 end
 initial begin #2000000;$fatal(1,"UART merge timeout");end
endmodule
