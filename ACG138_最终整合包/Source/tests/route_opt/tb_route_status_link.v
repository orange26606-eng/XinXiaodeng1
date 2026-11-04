`timescale 1ns/1ps
module tb_route_status_link;
 reg clk=0;always #10 clk=~clk;
 reg rx_clk=0;always #4 rx_clk=~rx_clk;
 reg rst=0,ready=1;
 reg[47:0]sonar=0;reg[95:0]seq=0,age=0;
 reg[127:0]counts=0;reg[63:0]rpm=0,duty=0;
 reg[55:0]diagnostic=0;reg[14:0]age_diag=0;
 wire valid;wire[767:0]payload;
 reg[735:0]normal_prior,diagnostic_prior,expected;
 reg[31:0]crc;
 integer cycle=0,reports=0,k;
 nav_control_link #(.CLK_HZ(10000)) dut(
  .rx_clk(rx_clk),.clk(clk),.rst_n(rst),.gmii_rxen(1'b0),.gmii_rxer(1'b0),.gmii_rxd(8'b0),
  .raw_sonar(sonar),.raw_sequence(seq),.raw_age_us(age),.raw_available(3'b111),.raw_zero(3'b000),
  .effective_duty(duty),.execution_flags(8'h35),.sonar_fresh(3'b111),.sonar_near(3'b010),.sonar_pending(3'b100),.sonar_observation_block(3'b000),
  .front_nonzero_sequence(2'b10),.sonar_age_diag(age_diag),.observation_pending(3'b010),.motion_blocked(1'b0),.ready(1'b1),.local_stop(1'b0),.fault(1'b0),.measured_rpm_centi(rpm),
  .encoder_counts(counts),.encoder_valid(4'hf),.fault_mask(4'b0000),.diagnostic(diagnostic),.status_valid(valid),.status_ready(ready),.status_payload(payload));
 function[31:0]crc_byte;input[31:0]c0;input[7:0]v;reg[31:0]c;integer b;
 begin c=c0^v;for(b=0;b<8;b=b+1)c=c[0]?(c>>1)^32'hedb88320:c>>1;crc_byte=c;end endfunction
 always @(posedge clk)begin
  if(dut.control_reset[1])begin
   if(valid&&ready)begin
    if(payload[735:0]!==expected)$fatal(1,"Status snapshot mismatch cycle=%0d",cycle);
    crc=32'hffffffff;
    for(k=0;k<92;k=k+1)crc=crc_byte(crc,expected[k*8+:8]);
    if(payload[767:736]!==~crc)$fatal(1,"Status CRC mismatch");
    reports=reports+1;
   end
   if(dut.take_ncs||dut.take_ncd)expected=dut.take_ncs?normal_prior:diagnostic_prior;
  end
  normal_prior=dut.snapshot;diagnostic_prior=dut.diagnostic_snapshot;
 end
 initial begin
  repeat(6)@(negedge clk);rst=1;
  for(cycle=0;cycle<18000;cycle=cycle+1)begin
   @(negedge clk);ready=(cycle%601)>110;
   sonar={$random,$random};seq={$random,$random,$random};age={$random,$random,$random};
   duty={$random,$random};rpm={$random,$random};counts={$random,$random,$random,$random};diagnostic={$random,$random};age_diag=$random;
  end
  ready=1;repeat(200)@(negedge clk);
  if(reports<50)$fatal(1,"Insufficient status reports");
  $display("PASS: NCS/NCD telemetry snapshots and CRC reports=%0d, source pipeline=one control clock",reports);$finish;
 end
endmodule
