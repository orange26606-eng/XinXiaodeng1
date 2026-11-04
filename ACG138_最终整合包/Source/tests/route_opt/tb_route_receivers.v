`timescale 1ns/1ps
module tb_route_receivers;
 reg clk=0;always #4 clk=~clk;
 reg rst=0,en=0,er=0,ready=1;reg[7:0]data=0;
 wire[1:0]nv,mv;wire[479:0]nc[0:1];wire[79:0]mc[0:1];
 reg[479:0]nav_old[0:255],nav_new[0:255];reg[79:0]phone_old[0:255],phone_new[0:255];
 reg[7:0]image[0:65535];
 integer cn0=0,cn1=0,cm0=0,cm1=0,vector_id,total,len,offset,kind,accept,error_index,j;
 integer old_nav_count,old_phone_count,first_nav=0,first_phone=0;
 nav_command_rx_before ng(.clk(clk),.rst_n(rst),.en(en),.er(er),.data(data),.valid(nv[0]),.ready(ready),.command(nc[0]));
 nav_command_rx_gmii nn(.clk(clk),.rst_n(rst),.en(en),.er(er),.data(data),.valid(nv[1]),.ready(ready),.command(nc[1]));
 phone_command_rx_before mg(.clk_rx(clk),.rst_n(rst),.gmii_rxen(en),.gmii_rxer(er),.gmii_rxd(data),.cmd_valid(mv[0]),.cmd_ready(ready),.cmd_op(mc[0][79:72]),.cmd_motion(mc[0][71:64]),.cmd_token(mc[0][63:32]),.cmd_sequence(mc[0][31:0]));
 phone_command_rx_gmii mn(.clk_rx(clk),.rst_n(rst),.gmii_rxen(en),.gmii_rxer(er),.gmii_rxd(data),.cmd_valid(mv[1]),.cmd_ready(ready),.cmd_op(mc[1][79:72]),.cmd_motion(mc[1][71:64]),.cmd_token(mc[1][63:32]),.cmd_sequence(mc[1][31:0]));
 always @(posedge clk)if(rst)begin
  if(nv[0]&&ready)begin nav_old[cn0]=nc[0];cn0=cn0+1;end
  if(nv[1]&&ready)begin nav_new[cn1]=nc[1];cn1=cn1+1;end
  if(mv[0]&&ready)begin phone_old[cm0]=mc[0];cm0=cm0+1;end
  if(mv[1]&&ready)begin phone_new[cm1]=mc[1];cm1=cm1+1;end
  #1;
  if({mv[0],mc[0]}!=={mv[1],mc[1]})begin $display("FAIL MCV cycle mismatch vector=%0d count=%0d",vector_id,mg.byte_count);$fatal;end
 end
 task send_frame;input integer id;integer k,base,count;
 begin
  base=id*256;count=image[base];
  for(k=0;k<count;k=k+1)begin
   @(negedge clk);en=1;er=k==image[base+1];data=image[base+128+k];
  end
  @(negedge clk);en=0;er=0;data=0;repeat(20)@(negedge clk);
 end
 endtask
 task compare_queues;
 begin
  if(cn0!=cn1||cm0!=cm1)begin $display("FAIL receiver counts vector=%0d NAV=%0d/%0d MCV=%0d/%0d",vector_id,cn0,cn1,cm0,cm1);$fatal;end
  for(j=0;j<cn0;j=j+1)if(nav_old[j]!==nav_new[j])begin $display("FAIL NAV payload vector=%0d record=%0d",vector_id,j);$fatal;end
  for(j=0;j<cm0;j=j+1)if(phone_old[j]!==phone_new[j])$fatal(1,"MCV payload mismatch");
 end
 endtask
 initial begin
  $readmemh("receiver_frames.hex",image);total={image[1],image[0]};
  repeat(5)@(negedge clk);rst=1;repeat(5)@(negedge clk);
  for(vector_id=1;vector_id<=total;vector_id=vector_id+1)begin
   offset=vector_id*256;kind=image[offset+3];accept=image[offset+2];
   old_nav_count=cn0;old_phone_count=cm0;send_frame(vector_id);compare_queues;
   if(cn0!=old_nav_count+(kind==0&&accept)||cm0!=old_phone_count+(kind==1&&accept))begin $display("FAIL expected reception vector=%0d kind=%0d expected=%0d oldNAV=%0d now=%0d oldMCV=%0d now=%0d",vector_id,kind,accept,old_nav_count,cn0,old_phone_count,cm0);$fatal;end
   if(accept&&kind==0&&first_nav==0)first_nav=vector_id;
   if(accept&&kind==1&&first_phone==0)first_phone=vector_id;
  end
  ready=0;send_frame(first_nav);if(nv!==2'b11||nc[0]!==nc[1])$fatal(1,"NAV held slot mismatch");
  send_frame(first_nav);if(nv!==2'b11||nc[0]!==nc[1])$fatal(1,"NAV busy-slot overwrite");
  ready=1;repeat(20)@(negedge clk);compare_queues;
  ready=0;send_frame(first_phone);if(mv!==2'b11||mc[0]!==mc[1])$fatal(1,"MCV held slot mismatch");
  send_frame(first_phone);if(mv!==2'b11||mc[0]!==mc[1])$fatal(1,"MCV busy-slot overwrite");
  ready=1;repeat(20)@(negedge clk);compare_queues;
  for(integer reset_kind=0;reset_kind<2;reset_kind=reset_kind+1)begin
   offset=(reset_kind==0?first_nav:first_phone)*256;
   for(j=0;j<60;j=j+1)begin @(negedge clk);en=1;data=image[offset+128+j];end
   @(negedge clk);rst=0;en=0;er=0;repeat(3)@(negedge clk);
   rst=1;repeat(5)@(negedge clk);send_frame(reset_kind==0?first_nav:first_phone);compare_queues;
  end
  $display("PASS: NAV/MCV payloads, CRC/header/range rejection, malformed lengths, RX errors and held-slot backpressure vectors=%0d NAV=%0d MCV=%0d",total,cn0,cm0);$finish;
 end
 initial begin #500000;$fatal(1,"receiver comparison timeout");end
endmodule
