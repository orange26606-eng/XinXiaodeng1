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
 function [39:0] decimal5;
 input [15:0] n;
 reg [19:0] bcd;
 integer i,j;
 begin
  bcd=0;
  for(i=15;i>=0;i=i-1)begin
   for(j=0;j<5;j=j+1)if(bcd[j*4+:4]>=5)bcd[j*4+:4]=bcd[j*4+:4]+4'd3;
   bcd={bcd[18:0],n[i]};
  end
  decimal5={8'd48+{4'b0,bcd[19:16]},8'd48+{4'b0,bcd[15:12]},8'd48+{4'b0,bcd[11:8]},8'd48+{4'b0,bcd[7:4]},8'd48+{4'b0,bcd[3:0]}};
 end
 endfunction
 function [7:0] digit;
 input [3:0] n;
 begin digit=n<10?8'd48+n:8'd65+n-10;end
 endfunction
 function [191:0] hex24;
 input [95:0] n;integer k;
 begin for(k=0;k<24;k=k+1)hex24[k*8+:8]=digit({n[k*4+:4]});end
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
 function [39:0] distance_text;input v;input [2:0] e;input [15:0] raw,n;
 begin if(!v || e==5)distance_text="-----";else if(e==4 || (raw!=0 && raw<30))distance_text="NEAR ";
 else distance_text=decimal5(n);end endfunction
 localparam LEN=(NEAR_DIAG?194:188)+(FEEDBACK_DIAG?12:0);
 reg [7:0] text_phase,text_retry;
 // 只锁存可变字段，固定文字为常量连接，不建立大移位寄存器。
 reg [39:0] text_f,text_l,text_r;
 reg [23:0] text_q,text_range,text_error,text_zero,text_confirm;
 reg [31:0] text_mode;
 reg [47:0] text_action;
 reg [7:0] text_fb,text_near;
 reg [79:0] text_reason;
 reg [287:0] diag_snapshot;
 wire [LEN*8-1:0] packet;
 generate if(FEEDBACK_DIAG)begin:g_feedback_packet
  assign packet={"F=",text_f," L=",text_l," R=",text_r,
 " Q=",text_q," RANGE=",text_range," ERR=",text_error," Z=",text_zero," C=",text_confirm," MODE=",text_mode," ACT=",text_action," FB=",text_fb,
 " WHY=",text_reason," N=",text_near," FBS=",text_phase," FBR=",text_retry,8'd13,8'd10,
 "USHEX F=",hex24(diag_snapshot[95:0])," L=",hex24(diag_snapshot[191:96])," R=",hex24(diag_snapshot[287:192]),8'd13,8'd10};
 end else if(NEAR_DIAG)begin:g_near_packet
  assign packet={"F=",text_f," L=",text_l," R=",text_r,
 " Q=",text_q," RANGE=",text_range," ERR=",text_error," Z=",text_zero," C=",text_confirm," MODE=",text_mode," ACT=",text_action," FB=",text_fb,
 " WHY=",text_reason," N=",text_near,8'd13,8'd10,
 "USHEX F=",hex24(diag_snapshot[95:0])," L=",hex24(diag_snapshot[191:96]),
 " R=",hex24(diag_snapshot[287:192]),8'd13,8'd10};
 end else begin:g_legacy_packet
  assign packet={"F=",text_f," L=",text_l," R=",text_r,
 " Q=",text_q," RANGE=",text_range," ERR=",text_error," Z=",text_zero," MODE=",text_mode," ACT=",text_action," FB=",text_fb,
 " WHY=",text_reason," N=",text_near,8'd13,8'd10,
 "USHEX F=",hex24(diag_snapshot[95:0])," L=",hex24(diag_snapshot[191:96]),
 " R=",hex24(diag_snapshot[287:192]),8'd13,8'd10};
 end endgenerate
 reg [7:0] remaining;
 wire busy;
 wire [10:0] byte_offset={remaining,3'b000}-11'd1;
 assign active=remaining!=0 || busy;
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(HOST_BAUD)) tx(.clk(clk),.rst(!rst_n),
 .data(packet[byte_offset -:8]),.valid(remaining!=0),.busy(busy),.serial_out(host_tx));
 always @(posedge clk)begin
  if(!rst_n)begin
   remaining<=0;text_phase<=48;text_retry<=48;text_f<=0;text_l<=0;text_r<=0;text_q<=0;text_range<=0;text_error<=0;text_zero<=0;text_confirm<=0;text_mode<=0;
   text_action<=0;text_fb<=0;text_near<=0;text_reason<=0;diag_snapshot<=0;
  end else if(trigger && remaining==0 && !busy)begin
   text_f<=(zero_mask[0] || (NEAR_DIAG && near_pending_mask[0])) && valid[0]?decimal5(front[15:0]):distance_text(valid[0],status[2:0],diagnostic[95:80],front[15:0]);
   text_l<=(zero_mask[1] || (NEAR_DIAG && near_pending_mask[1])) && valid[1]?decimal5(left[15:0]):distance_text(valid[1],status[5:3],diagnostic[191:176],left[15:0]);
   text_r<=(zero_mask[2] || (NEAR_DIAG && near_pending_mask[2])) && valid[2]?decimal5(right[15:0]):distance_text(valid[2],status[8:6],diagnostic[287:272],right[15:0]);
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
   diag_snapshot<=diagnostic;remaining<=LEN;
  end else if(remaining!=0 && !busy)remaining<=remaining-1'b1;
 end
endmodule
