`timescale 1ns/1ps
module route_status_test #(parameter FB=0,NEAR=0);
 reg clk=0;always #5 clk=~clk;
 reg rst=0,trigger=0;
 reg [23:0]front=0,left=0,right=0;reg[2:0]valid=0;reg[8:0]status=0;
 reg[3:0]state_code=0,decision_cmd=0,attempts=0,actual=0,feedback_fault=0,reason=0;
 reg[2:0]near_mask=0,zero_mask=0,zero_expired_mask=0,near_pending_mask=0,near_timeout_mask=0,near_recover_mask=0,feedback_phase=0;
 reg[1:0]feedback_retries=0;reg[5:0]near_counts=0,receive_errors=0;
 reg[287:0]diagnostic=0;reg auto_mode=0,hold_off=0;
 wire serial_old,serial_new,active_old,active_new;
 integer n_old=0,n_new=0,report_id,i,seed=87363;
 reg[7:0]bytes_old[0:8191],bytes_new[0:8191];
 localparam LEN=(NEAR?194:188)+(FB?12:0);
 `define INPUTS .clk(clk),.rst_n(rst),.trigger(trigger),.front(front),.left(left),.right(right),.valid(valid),.status(status),.state_code(state_code),.decision_cmd(decision_cmd),.attempts(attempts),.actual(actual),.feedback_fault(feedback_fault),.reason(reason),.near_mask(near_mask),.zero_mask(zero_mask),.zero_expired_mask(zero_expired_mask),.near_pending_mask(near_pending_mask),.near_timeout_mask(near_timeout_mask),.near_recover_mask(near_recover_mask),.near_counts(near_counts),.diagnostic(diagnostic),.receive_errors(receive_errors),.auto_mode(auto_mode),.hold_off(hold_off),.feedback_phase(feedback_phase),.feedback_retries(feedback_retries)
 robot_status_before #(.CLK_HZ(1600),.HOST_BAUD(100),.FEEDBACK_DIAG(FB),.NEAR_DIAG(NEAR)) golden(`INPUTS,.host_tx(serial_old),.active(active_old));
 robot_status #(.CLK_HZ(1600),.HOST_BAUD(100),.FEEDBACK_DIAG(FB),.NEAR_DIAG(NEAR)) upgraded(`INPUTS,.host_tx(serial_new),.active(active_new));
 `undef INPUTS
 initial forever begin:decode_old
  reg[7:0]octet;integer bit_id;
  @(negedge serial_old);if(rst)begin
   repeat(8)@(posedge clk);#1;if(serial_old!==0)$fatal(1,"old UART start");
   for(bit_id=0;bit_id<8;bit_id=bit_id+1)begin repeat(16)@(posedge clk);#1;octet[bit_id]=serial_old;end
   repeat(16)@(posedge clk);#1;if(serial_old!==1)$fatal(1,"old UART stop");
   bytes_old[n_old]=octet;n_old=n_old+1;
  end
 end
 initial forever begin:decode_new
  reg[7:0]octet;integer bit_id;
  @(negedge serial_new);if(rst)begin
   repeat(8)@(posedge clk);#1;if(serial_new!==0)$fatal(1,"new UART start");
   for(bit_id=0;bit_id<8;bit_id=bit_id+1)begin repeat(16)@(posedge clk);#1;octet[bit_id]=serial_new;end
   repeat(16)@(posedge clk);#1;if(serial_new!==1)$fatal(1,"new UART stop");
   bytes_new[n_new]=octet;n_new=n_new+1;
  end
 end
 initial begin
  repeat(5)@(negedge clk);rst=1;repeat(5)@(negedge clk);
  for(report_id=0;report_id<18;report_id=report_id+1)begin
   front=(report_id*3471)%65536;left=(report_id*17777)%65536;right=(report_id*439)%65536;
   valid=report_id;status=report_id*37;actual=report_id%9;reason=report_id%16;feedback_fault=report_id;
   feedback_phase=report_id%8;feedback_retries=report_id%4;near_mask=report_id;
   zero_mask=report_id>>1;zero_expired_mask=report_id>>2;near_pending_mask=report_id>>1;
   near_timeout_mask=report_id>>2;near_recover_mask=report_id>>3;near_counts=report_id*7;receive_errors=report_id;
   auto_mode=report_id%2;diagnostic={$random(seed),$random(seed),$random(seed),$random(seed),$random(seed),$random(seed),$random(seed),$random(seed),$random(seed)};
   diagnostic[95:80]=report_id%5==0?0:report_id%5==1?1:report_id%5==2?29:report_id%5==3?30:65535;
   diagnostic[191:176]=report_id%3==0?0:29;diagnostic[287:272]=report_id%3==0?5000:30;
   trigger=1;@(negedge clk);trigger=0;repeat(3)@(negedge clk);
   // 新格式化器仍在处理时扰动全部实时输入。
   front=65535;left=65535;right=65535;valid=0;status=511;diagnostic=0;actual=0;reason=15;
   wait(!active_old&&!active_new&&n_old>=(report_id+1)*LEN&&n_new>=(report_id+1)*LEN);
   for(i=report_id*LEN;i<(report_id+1)*LEN;i=i+1)
    if(bytes_old[i]!==bytes_new[i])begin $display("FAIL status FB=%d NEAR=%d report=%d byte=%d old=%h new=%h",FB,NEAR,report_id,i%LEN,bytes_old[i],bytes_new[i]);$fatal;end
   repeat(4)@(negedge clk);
  end
  $display("PASS: exact UART bytes, decimal/style/flags/hex snapshot coherence FB=%0d NEAR=%0d reports=%0d bytes=%0d",FB,NEAR,report_id,n_new);$finish;
 end
 initial begin #20000000;$fatal(1,"status timeout FB=%0d NEAR=%0d bytes=%0d/%0d",FB,NEAR,n_old,n_new);end
endmodule
module tb_route_status_00;route_status_test #(.FB(0),.NEAR(0)) test();endmodule
module tb_route_status_01;route_status_test #(.FB(0),.NEAR(1)) test();endmodule
module tb_route_status_10;route_status_test #(.FB(1),.NEAR(0)) test();endmodule
module tb_route_status_11;route_status_test #(.FB(1),.NEAR(1)) test();endmodule
