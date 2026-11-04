`timescale 1ns/1ps
module tb_route_udp_tx;
 reg clk=0;always #4 clk=~clk;
 reg rst=0,sample_valid=0;reg[111:0]sample_data=0;
 wire[1:0]available,en,done;wire[7:0]data[0:1];
 integer cycle=0,bytes=0,frames=0;
 robot_udp_tx_before old(.clk125m(clk),.rst_n(rst),.sample_valid(sample_valid),.sample_ready(available[0]),.sample_data(sample_data),.gmii_txen(en[0]),.gmii_txd(data[0]),.tx_done(done[0]));
 robot_udp_message_tx_gmii dut(.clk125m(clk),.rst_n(rst),.sample_valid(sample_valid),.sample_ready(available[1]),.sample_data(sample_data),.gmii_txen(en[1]),.gmii_txd(data[1]),.tx_done(done[1]));
 always @(posedge clk)begin
  #1;
  if({available[0],en[0],done[0],data[0]}!=={available[1],en[1],done[1],data[1]})
   $fatal(1,"Telemetry TX cycle mismatch at %0d",cycle);
  if(rst&&en[0])bytes=bytes+1;
  if(rst&&done[0])frames=frames+1;
 end
 initial begin
  repeat(5)@(negedge clk);rst=1;
  for(cycle=0;cycle<16000;cycle=cycle+1)begin
   @(negedge clk);
   sample_valid=($random&7)!=0;sample_data={$random,$random,$random,$random};
   if(cycle==1000||cycle==5000)rst=0;
   if(cycle==1004||cycle==5004)rst=1;
  end
  sample_valid=0;repeat(160)@(negedge clk);
  if(frames<100||bytes<8600)$fatal(1,"Insufficient TX coverage");
  $display("PASS: telemetry TX cycle-equivalent frames=%0d bytes=%0d with partial-frame resets",frames,bytes);$finish;
 end
endmodule
