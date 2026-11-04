`timescale 1ns/1ps
module tb_route_controller;
 reg clk=0;always #5 clk=~clk;
 reg rst=0;reg[13:0] available1=0,available2=0;
 reg[7:0] eye1[0:511],eye2[0:511];
 integer p1[0:1],p2[0:1];wire[1:0]r1,r2,out_valid,fault;wire[7:0]out_data[0:1];
 integer i,cycle,checks=0,emitted=0,seed=49349;
 controller_before #(.IMG_WIDTH(4),.IMG_HEIGHT(3)) golden(.clk(clk),.rst_n(rst),.rdreq1(r1[0]),.rdreq2(r2[0]),.rddata1(eye1[p1[0]]),.rddata2(eye2[p2[0]]),.rdusedw1(available1-p1[0]),.rdusedw2(available2-p2[0]),.fifo_wrreq(out_valid[0]),.fifo_wrdata(out_data[0]),.pair_fault(fault[0]));
 controller #(.IMG_WIDTH(4),.IMG_HEIGHT(3)) upgraded(.clk(clk),.rst_n(rst),.rdreq1(r1[1]),.rdreq2(r2[1]),.rddata1(eye1[p1[1]]),.rddata2(eye2[p2[1]]),.rdusedw1(available1-p1[1]),.rdusedw2(available2-p2[1]),.fifo_wrreq(out_valid[1]),.fifo_wrdata(out_data[1]),.pair_fault(fault[1]));
 always @(posedge clk)begin
  if(!rst)begin p1[0]<=0;p1[1]<=0;p2[0]<=0;p2[1]<=0;end
  else for(integer j=0;j<2;j=j+1)begin
   if(r1[j])p1[j]<=p1[j]+1;
   if(r2[j])p2[j]<=p2[j]+1;
  end
  #1;
  if(rst)begin
   checks=checks+1;
   if({r1[0],r2[0],out_valid[0],fault[0]}!=={r1[1],r2[1],out_valid[1],fault[1]} ||
      (out_valid[0]&&out_data[0]!==out_data[1]) ||p1[0]!=p1[1]||p2[0]!=p2[1])begin
    $display("FAIL FIFO timing/order cycle=%0d state=%0d/%0d read=%b/%b",cycle,golden.state,upgraded.state,{r1[0],r2[0]},{r1[1],r2[1]});$fatal;
   end
   if(out_valid[0])emitted=emitted+1;
  end
 end
 initial begin
  for(i=0;i<500;i=i+10)begin
   eye1[i]=i%30==0?0:(i/10)%3;eye2[i]=i%50==0?0:(i/10)%3;
   eye1[i+1]=0;eye2[i+1]=0;
   for(integer j=2;j<10;j=j+1)begin eye1[i+j]=8'h10+j+i;eye2[i+j]=8'h90+j+i;end
  end
  repeat(4)@(negedge clk);rst=1;
  for(cycle=0;cycle<4000;cycle=cycle+1)begin
   if(available1<500&&($random(seed)&3)!=0)available1=available1+1;
   if(available2<500&&($random(seed)&3)!=0)available2=available2+1;
   @(negedge clk);
  end
  if(emitted<100)$fatal(1,"Too few paired image bytes");
  $display("PASS: FIFO reads, eye order, headers, mismatch resync and output bytes identical cycles=%0d bytes=%0d",checks,emitted);$finish;
 end
endmodule
