`timescale 1ns/1ps
// 每通道独立可信缓存，只有完整且校验正确的帧刷新年龄。
// 可选突近确认，紧急近障及连续接近趋势优先处理。
module XR_a02_frame_parser #(
 parameter integer P_GAP_CLKS=500000,P_STALE_CLKS=30000000,
 parameter integer P_LOST_CLKS=50000000,P_MIN_MM=30,P_MAX_MM=4500,
 parameter integer P_IGNORE_ZERO=0,
 parameter integer P_STOP_MM=150,P_RELEASE_MM=220,P_HARD_STOP_MM=0,
 parameter integer P_NEAR_CONFIRM=0,P_NEAR_CLKS=15000000,P_EMERGENCY_MM=50,P_EMERGENCY_INCLUSIVE=0,P_TREND_MM=350,
 parameter integer P_ZERO_MIN_MM=250,P_ZERO_FRESH_CLKS=10000000,P_ZERO_CLKS=15000000
)(input clk,rst_n,input age_tick,input [7:0] byte_data,input byte_valid,byte_error,
 output [23:0] distance_mm,output valid,output reg sample_update,
 output [2:0] status,output reg [31:0] age,output [95:0] diagnostic,
 output reg near_blocked,output [1:0] receive_error,
 output reg packet_update,output reg [31:0] raw_age_us,output reg [31:0] nonzero_sequence,output reg raw_seen,output reg last_raw_zero,output reg zero_active,output zero_expired,output [15:0] control_raw,
 input near_allow,output reg near_pending,output near_timeout,output reg [1:0] near_count,output reg near_recover);
 reg [1:0] state;
 reg [7:0] hi,lo;
 reg [31:0] gap,rx_age;
 reg [15:0] raw_value,bad_count,h1,h2,filtered;
 reg [1:0] hist_count;
 reg seen,clear_once,blind;
 reg [2:0] last_error;
 reg [15:0] accepted_raw;
 reg [31:0] zero_timer;
 reg zero_clear_once;
 reg [31:0] near_timer;
 reg [15:0] previous_raw,near_baseline;
 reg have_previous;
 wire [15:0] candidate={hi,lo};
 wire [7:0] checksum=8'hff+hi+lo;
 wire emergency=candidate!=0 && (candidate<P_EMERGENCY_MM || (P_EMERGENCY_INCLUSIVE && candidate==P_EMERGENCY_MM));
 wire [15:0] bounded=candidate>P_MAX_MM?P_MAX_MM:candidate;
 wire expired=(P_IGNORE_ZERO?rx_age:age)>=P_LOST_CLKS;
 wire approaching=have_previous && previous_raw>accepted_raw && previous_raw-accepted_raw>=20 && accepted_raw<=P_TREND_MM;
 wire [16:0] baseline_delta=candidate>=near_baseline?candidate-near_baseline:near_baseline-candidate;
 // 阈值为保留基准的15%，下限100mm，不使用除法硬件。
 wire [31:0] baseline_percent=near_baseline*15;
 wire baseline_return=(baseline_delta<=100 || baseline_delta*100<=baseline_percent);
 wire near_entry=P_NEAR_CONFIRM && near_allow && seen && !expired && !near_blocked &&
                  !zero_active && !near_recover && age<P_ZERO_FRESH_CLKS && accepted_raw>=P_RELEASE_MM && !approaching;
 assign near_timeout=P_NEAR_CONFIRM && near_pending && near_timer>=P_NEAR_CLKS;
 assign zero_expired=zero_active && zero_timer>=P_ZERO_CLKS;
 assign control_raw=accepted_raw;
 wire history_ok=seen && !expired && hist_count!=0;
 wire [15:0] median=(bounded>h1)?((h1>h2)?h1:((bounded>h2)?h2:bounded)):
                                       ((bounded>h2)?bounded:((h1>h2)?h2:h1));
 assign receive_error=last_error[1:0];
 assign valid=seen && !expired && !zero_expired ;
 assign distance_mm=!valid?24'hEEEEEE:blind?{8'd0,accepted_raw}:{8'd0,filtered};
 assign status=!seen?3'd5:(expired || zero_expired)?3'd6:(zero_active || near_pending || near_recover || near_timeout)?3'd7:blind?3'd4:(P_IGNORE_ZERO?rx_age:age)>=P_STALE_CLKS?3'd7:last_error;
 assign diagnostic={raw_value,rx_age,age,bad_count};
 // 所有已接受距离统一更新滤波历史与年龄。
 task take_sample;
 input restart_history;
 begin
  zero_active<=0;zero_timer<=0;zero_clear_once<=0;
  near_pending<=0;near_timer<=0;near_count<=0;near_recover<=0;
  previous_raw<=accepted_raw;have_previous<=seen;
  age<=0;seen<=1;sample_update<=1;accepted_raw<=candidate;
  blind<=candidate<P_MIN_MM;
  if(candidate<P_MIN_MM)begin
   near_blocked<=1;clear_once<=0;hist_count<=0;filtered<=bounded;
  end else if(restart_history)begin
   filtered<=bounded;h1<=bounded;h2<=bounded;hist_count<=2;
   near_blocked<=bounded<P_STOP_MM;clear_once<=bounded>=P_RELEASE_MM;
  end else begin
   h2<=h1;h1<=bounded;
   if(!history_ok)begin hist_count<=1;filtered<=bounded;end
   else if(hist_count==1)begin hist_count<=2;filtered<=bounded<h1?bounded:h1;end
   else begin hist_count<=2;filtered<=median;end
   if(bounded<P_STOP_MM)begin near_blocked<=1;clear_once<=0;end
   else if(bounded>=P_RELEASE_MM)begin
    if(clear_once && history_ok)near_blocked<=0;
    clear_once<=1;
   end else clear_once<=0;
  end
 end
 endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=0;hi<=0;lo<=0;gap<=0;rx_age<=0;age<=0;last_raw_zero<=0;nonzero_sequence<=0;raw_seen<=0;raw_age_us<=0;
   packet_update<=0;raw_value<=0;bad_count<=0;h1<=0;h2<=0;filtered<=0;hist_count<=0;
   zero_active<=0;zero_timer<=0;zero_clear_once<=0;accepted_raw<=0;
   near_pending<=0;near_timer<=0;near_count<=0;near_recover<=0;
   previous_raw<=0;near_baseline<=0;have_previous<=0;
   seen<=0;near_blocked<=1;clear_once<=0;blind<=0;last_error<=0;sample_update<=0;
  end else begin
   sample_update<=0;packet_update<=0;
   // 数据源直接测得微秒年龄，避免宽组合除50路径。
   if(raw_seen && age_tick && raw_age_us!=32'hffffffff)raw_age_us<=raw_age_us+1'b1;
   if(near_pending && near_timer<P_NEAR_CLKS)near_timer<=near_timer+1'b1;
   if(near_timeout)near_blocked<=1;
   if(zero_active && zero_timer<P_ZERO_CLKS)zero_timer<=zero_timer+1'b1;
   if(age!=32'hffffffff)age<=age+1'b1;
   if(rx_age!=32'hffffffff)rx_age<=rx_age+1'b1;
   if(state!=0 && gap<P_GAP_CLKS)gap<=gap+1'b1;
   if(expired)begin hist_count<=0;near_blocked<=1;clear_once<=0;end
   if(byte_error)begin
    state<=0;gap<=0;last_error<=2;clear_once<=0;zero_clear_once<=0;near_count<=0;
    if(bad_count!=16'hffff)bad_count<=bad_count+1'b1;
   end else if(state!=0 && gap>=P_GAP_CLKS-1)begin
    state<=(byte_valid && byte_data==8'hff)?1:0;gap<=0;last_error<=3;clear_once<=0;zero_clear_once<=0;near_count<=0;
    if(bad_count!=16'hffff)bad_count<=bad_count+1'b1;
   end else if(byte_valid)begin
    gap<=0;
    case(state)
     0:if(byte_data==8'hff)state<=1;
     1:begin hi<=byte_data;state<=2;end
     2:begin lo<=byte_data;state<=3;end
     3:begin
      state<=0;
      if(byte_data!=checksum)begin
       last_error<=1;clear_once<=0;zero_clear_once<=0;near_count<=0;
       if(bad_count!=16'hffff)bad_count<=bad_count+1'b1;
       if(byte_data==8'hff)state<=1;
      end else begin
       packet_update<=1;raw_seen<=1;raw_age_us<=0;raw_value<=candidate;rx_age<=0;last_error<=0;last_raw_zero<=candidate==0;
       if(candidate!=0)nonzero_sequence<=nonzero_sequence+1'b1;
       if(candidate==0)begin
        if(!P_IGNORE_ZERO)begin
        // 0值保留可信距离及已有近障观察预算。
        zero_active<=1;zero_clear_once<=0;clear_once<=0;near_count<=0;near_recover<=0;
        if(!zero_active)zero_timer<=0;
        end
        // 忽略0值时仅更新原始诊断与接收年龄。

       end else if(emergency)begin
        // 新鲜有效的非零紧急近障绕过普通确认和中值延迟。
        // 输出实际小于最小量程的非零值，原始零值仍按忽略策略处理。
        take_sample(1);
       end else if(P_HARD_STOP_MM>0 && candidate<P_HARD_STOP_MM)begin
        // 仅前方使用硬接近边界，有效非零盲区值
        // 需在可选突近确认之前送入control_raw。
        // 零值已在上方处理，侧通道沿用原确认规则。
        take_sample(0);
       end else if(candidate<P_STOP_MM)begin
        zero_clear_once<=0;clear_once<=0;
        if(P_NEAR_CONFIRM && near_pending && candidate>=P_EMERGENCY_MM && !near_timeout)begin
         if(near_count>=2)take_sample(0);
         else near_count<=near_count+1'b1;
        end else if(near_entry && !near_pending && candidate>=P_EMERGENCY_MM)begin
         near_pending<=1;near_timer<=0;near_count<=1;near_baseline<=accepted_raw;
        end else take_sample(0);
       end else if(P_NEAR_CONFIRM && near_pending)begin
        near_count<=0;
        if(candidate>=P_RELEASE_MM)begin
         if(baseline_return && !near_timeout && !zero_active)begin
          take_sample(0);near_recover<=1;
         end else if(zero_clear_once)take_sample(1);
         else zero_clear_once<=1;
        end else zero_clear_once<=0;
       end else if(zero_active)begin
        // 未建立可信状态时，孤立非零帧按该事件原预算处理。
        if(candidate>=((zero_expired || near_blocked || !seen)?P_RELEASE_MM:P_STOP_MM))begin
         if(zero_clear_once)take_sample(1);
         else zero_clear_once<=1;
        end else zero_clear_once<=0;
       end else begin
        take_sample(0);
        if(near_recover && candidate<P_RELEASE_MM)near_recover<=1;
       end
      end
     end
     default:state<=0;
    endcase
   end
  end
 end
endmodule
