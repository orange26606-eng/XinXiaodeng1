`timescale 1ns/1ps
module robot_status #(parameter FEEDBACK_DIAG=0,CLK_HZ=50000000, HOST_BAUD=115200,NEAR_DIAG=0)(
 input [2:0] feedback_phase,input [1:0] feedback_retries,
 input clk,rst_n,trigger,input [23:0] front,left,right,input [2:0] valid,
 input [8:0] status,input [3:0] state_code,decision_cmd,attempts,actual,feedback_fault,
 input [3:0] reason,input [2:0] near_mask,
 input [2:0] zero_mask,zero_expired_mask,
 input [2:0] near_pending_mask,near_timeout_mask,near_recover_mask,input [5:0] near_counts,
 input [287:0] diagnostic,input [5:0] receive_errors,
 input auto_mode,hold_off,output host_tx,output active);
 function [7:0] digit;
 input [3:0] n;
 begin digit=n<10?8'd48+n:8'd65+n-10;end
 endfunction
 function [47:0] action_name;input [3:0] a;
 begin case(a)
 1:action_name="FWD   ";2:action_name="BACK  ";3:action_name="ARC_L ";4:action_name="ARC_R ";
 5:action_name="TURN_L";6:action_name="TURN_R";7:action_name="BK_L  ";8:action_name="BK_R  ";
 default:action_name="STOP  ";endcase end endfunction
 function [79:0] reason_name;input [3:0] a;
 begin case(a)
 1:reason_name="FRONT_NEAR";2:reason_name="SIDE_NEAR ";3:reason_name="DATA_LOST ";
 4:reason_name="MOTOR_FB  ";5:reason_name="DATA_HOLD ";6:reason_name="FRONT_SLOW";
 7:reason_name="WAIT_CLEAR";8:reason_name="TURN_TIME ";9:reason_name="TURN_SPACE";
 10:reason_name="ENC_WAIT  ";11:reason_name="MANUAL_OFF";12:reason_name="ZERO_HOLD ";13:reason_name="ZERO_STOP ";14:reason_name="NEAR_CHECK";15:reason_name="NEAR_WAIT ";default:reason_name="CLEAR     ";
 endcase end endfunction
 function [7:0] quality;input v;input [2:0] e;input [1:0] rxerr;input [31:0] age;
 begin if(e==5)quality="W";else if(!v)quality="L";
 else if(rxerr!=0 || e==7 || age>=CLK_HZ*3/5)quality="H";else quality="O";end endfunction
 function [7:0] range_name;input [2:0] e;input [15:0] raw;
 begin if(e==5)range_name="W";else if(raw==0)range_name="Z";else if(raw<30)range_name="N";
 else if(raw>4500)range_name="F";else range_name="O";end endfunction
 // 每次报告使用一致快照、共用BCD转换器、
 // 已初始化的字节缓冲和紧凑可变字段位置表。
 localparam LEN=(NEAR_DIAG?194:188)+(FEEDBACK_DIAG?12:0);
 localparam VARIABLE_BYTES=FEEDBACK_DIAG?(NEAR_DIAG?126:122):(NEAR_DIAG?124:121);
 localparam IDLE=0,FIELD=1,BCD_START=2,BCD_WAIT=3,WRITE_FIELDS=4,DRAIN=5;
 reg [2:0] format_state;
 reg [1:0] channel;
 reg [15:0] distance_snapshot[0:2];
 reg [1:0] distance_style[0:2];
 reg [7:0] format_address,remaining;
 reg [39:0] text_f,text_l,text_r;
 reg [23:0] text_q,text_range,text_error,text_zero,text_confirm;
 reg [31:0] text_mode;
 reg [47:0] text_action;
 reg [7:0] text_fb,text_near,text_phase,text_retry;
 reg [79:0] text_reason;
 reg [287:0] diag_snapshot;
 (* syn_romstyle="block_rom" *) reg [19:0] field_table[0:255];
 (* syn_ramstyle="block_ram" *) reg [7:0] packet_bytes[0:255];
 initial begin
  if(FEEDBACK_DIAG && NEAR_DIAG)begin
   $readmemh("src/motion/status_fields_fb_near.hex",field_table);
   $readmemh("src/motion/status_template_fb_near.hex",packet_bytes);
  end else if(FEEDBACK_DIAG)begin
   $readmemh("src/motion/status_fields_fb_legacy.hex",field_table);
   $readmemh("src/motion/status_template_fb_legacy.hex",packet_bytes);
  end else if(NEAR_DIAG)begin
   $readmemh("src/motion/status_fields_near.hex",field_table);
   $readmemh("src/motion/status_template_near.hex",packet_bytes);
  end else begin
   $readmemh("src/motion/status_fields_legacy.hex",field_table);
   $readmemh("src/motion/status_template_legacy.hex",packet_bytes);
  end
 end
 function [1:0] style;
 input good;input [2:0] error;input [15:0] raw;input held;
 begin
  if(good&&held)style=0;
  else if(!good||error==5)style=1;
  else if(error==4||(raw!=0&&raw<30))style=2;
  else style=0;
 end
 endfunction
 function [39:0] decimal_ascii;
 input [19:0] bcd;
 begin
  decimal_ascii={4'h3,bcd[19:16],4'h3,bcd[15:12],4'h3,bcd[11:8],4'h3,bcd[7:4],4'h3,bcd[3:0]};
 end
 endfunction
 wire bcd_done;
 wire [39:0] bcd_value;
 XR_bin32_bcd decimal_converter(.clk(clk),.rst_n(rst_n),.start(format_state==BCD_START),
  .value({16'd0,distance_snapshot[channel]}),.busy(),.done(bcd_done),.bcd(bcd_value));
 task store_distance;
 input [39:0] value;
 begin
  case(channel)
   0:text_f<=value;
   1:text_l<=value;
   default:text_r<=value;
  endcase
  if(channel==2)begin format_address<=0;format_state<=WRITE_FIELDS;end
  else begin channel<=channel+1'b1;format_state<=FIELD;end
 end
 endtask

 reg [19:0] descriptor;
 reg descriptor_valid;
 always @(posedge clk)descriptor<=field_table[format_address];
 always @(posedge clk)begin
  if(!rst_n)descriptor_valid<=0;
  else descriptor_valid<=format_state==WRITE_FIELDS;
 end
 function [7:0] character;
 input [79:0] word;input [3:0] last,position;
 reg [6:0] bit_index;
 begin
  bit_index=(last-position)<<3;
  character=word[bit_index+:8];
 end
 endfunction
 reg [7:0] variable_byte;
 wire [4:0] nibble_index=5'd23-descriptor[4:0];
 always @*begin
  variable_byte=0;
  case(descriptor[11:7])
   1:variable_byte=character({40'd0,text_f},4,descriptor[3:0]);
   2:variable_byte=character({40'd0,text_l},4,descriptor[3:0]);
   3:variable_byte=character({40'd0,text_r},4,descriptor[3:0]);
   4:variable_byte=character({56'd0,text_q},2,descriptor[3:0]);
   5:variable_byte=character({56'd0,text_range},2,descriptor[3:0]);
   6:variable_byte=character({56'd0,text_error},2,descriptor[3:0]);
   7:variable_byte=character({56'd0,text_zero},2,descriptor[3:0]);
   8:variable_byte=character({56'd0,text_confirm},2,descriptor[3:0]);
   9:variable_byte=character({48'd0,text_mode},3,descriptor[3:0]);
   10:variable_byte=character({32'd0,text_action},5,descriptor[3:0]);
   11:variable_byte=text_fb;
   12:variable_byte=character(text_reason,9,descriptor[3:0]);
   13:variable_byte=text_near;
   14:variable_byte=text_phase;
   15:variable_byte=text_retry;
   16:variable_byte=digit(diag_snapshot[nibble_index*4+:4]);
   17:variable_byte=digit(diag_snapshot[96+nibble_index*4+:4]);
   18:variable_byte=digit(diag_snapshot[192+nibble_index*4+:4]);
   default:begin end
  endcase
 end
 reg [7:0] transmitted_byte,read_tag;
 wire [7:0] read_address=LEN-remaining;
 always @(posedge clk)begin
  if(rst_n&&descriptor_valid)packet_bytes[descriptor[19:12]]<=variable_byte;
  transmitted_byte<=packet_bytes[read_address];
  read_tag<=read_address;
 end
 wire busy;
 wire byte_available=remaining!=0&&read_tag==read_address;
 assign active=format_state!=IDLE||remaining!=0||busy;
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(HOST_BAUD)) tx(.clk(clk),.rst(!rst_n),
  .data(transmitted_byte),.valid(byte_available),.busy(busy),.serial_out(host_tx));
 integer i;
 always @(posedge clk)begin
  if(!rst_n)begin
   format_state<=IDLE;channel<=0;format_address<=0;remaining<=0;
   text_f<=0;text_l<=0;text_r<=0;text_q<=0;text_range<=0;text_error<=0;text_zero<=0;text_confirm<=0;
   text_mode<=0;text_action<=0;text_fb<=0;text_near<=0;text_reason<=0;diag_snapshot<=0;
   text_phase<=48;text_retry<=48;
   for(i=0;i<3;i=i+1)begin distance_snapshot[i]<=0;distance_style[i]<=0;end
  end else if(trigger&&format_state==IDLE&&remaining==0&&!busy)begin
   distance_snapshot[0]<=front[15:0];distance_snapshot[1]<=left[15:0];distance_snapshot[2]<=right[15:0];
   distance_style[0]<=style(valid[0],status[2:0],diagnostic[95:80],zero_mask[0]||(NEAR_DIAG&&near_pending_mask[0]));
   distance_style[1]<=style(valid[1],status[5:3],diagnostic[191:176],zero_mask[1]||(NEAR_DIAG&&near_pending_mask[1]));
   distance_style[2]<=style(valid[2],status[8:6],diagnostic[287:272],zero_mask[2]||(NEAR_DIAG&&near_pending_mask[2]));
   channel<=0;format_state<=FIELD;
   text_q<={quality(valid[0],status[2:0],receive_errors[1:0],diagnostic[79:48]),quality(valid[1],status[5:3],receive_errors[3:2],diagnostic[175:144]),quality(valid[2],status[8:6],receive_errors[5:4],diagnostic[271:240])};
   text_range<={range_name(status[2:0],diagnostic[95:80]),range_name(status[5:3],diagnostic[191:176]),range_name(status[8:6],diagnostic[287:272])};
   text_error<={digit({2'b0,receive_errors[1:0]}),digit({2'b0,receive_errors[3:2]}),digit({2'b0,receive_errors[5:4]})};
   text_zero<={digit(zero_expired_mask[0]?4'd2:zero_mask[0]?4'd1:4'd0),digit(zero_expired_mask[1]?4'd2:zero_mask[1]?4'd1:4'd0),digit(zero_expired_mask[2]?4'd2:zero_mask[2]?4'd1:4'd0)};
   text_confirm<={near_timeout_mask[0]?8'h54:near_pending_mask[0]?digit({2'b0,near_counts[1:0]}):near_recover_mask[0]?8'h52:8'h30,
                  near_timeout_mask[1]?8'h54:near_pending_mask[1]?digit({2'b0,near_counts[3:2]}):near_recover_mask[1]?8'h52:8'h30,
                  near_timeout_mask[2]?8'h54:near_pending_mask[2]?digit({2'b0,near_counts[5:4]}):near_recover_mask[2]?8'h52:8'h30};
   text_phase<=digit({1'b0,feedback_phase});text_retry<=digit({2'b0,feedback_retries});
   text_mode<=auto_mode?"AUTO":"MANU";text_action<=action_name(actual);
   text_fb<=digit(feedback_fault);text_near<=digit({1'b0,near_mask});text_reason<=reason_name(reason);
   diag_snapshot<=diagnostic;
  end else begin
   case(format_state)
    FIELD:case(distance_style[channel])
     1:store_distance("-----");
     2:store_distance("NEAR ");
     default:format_state<=BCD_START;
    endcase
    BCD_START:format_state<=BCD_WAIT;
    BCD_WAIT:if(bcd_done)store_distance(decimal_ascii(bcd_value[19:0]));
    WRITE_FIELDS:begin
     if(format_address==VARIABLE_BYTES-1)format_state<=DRAIN;
     else format_address<=format_address+1'b1;
    end
    DRAIN:if(!descriptor_valid)begin format_state<=IDLE;remaining<=LEN;end
    default:begin end
   endcase
   if(byte_available&&!busy)remaining<=remaining-1'b1;
  end
 end
endmodule
