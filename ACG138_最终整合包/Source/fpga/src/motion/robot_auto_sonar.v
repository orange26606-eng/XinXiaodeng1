`timescale 1ns/1ps
// AUTO独立缓存，完整校验正确的报文先进入此处，
// 再交给原MANUAL解析器的150/100mm锁存处理，两模式不共享这些锁存。
module robot_auto_sonar_channel #(
 parameter NORMAL_MM=400,NEAR_MM=150,RELEASE_MM=200,
 parameter STALE_CLKS=30000000,LOST_CLKS=75000000,PENDING_CLKS=15000000
)(input clk,rst_n,packet,input [15:0] raw,
 output reg [15:0] distance,output usable,fresh,
 output reg pending,output pending_expired,output [1:0] confirm_count,output reg [1:0] zone,output reg accepted);
 localparam AGE_BITS=$clog2(LOST_CLKS+1)>0?$clog2(LOST_CLKS+1):1;
 localparam PENDING_BITS=$clog2(PENDING_CLKS+1)>0?$clog2(PENDING_CLKS+1):1;
 reg seen;reg [AGE_BITS-1:0] age;reg [PENDING_BITS-1:0] pending_timer;reg [15:0] h1,h2;reg [1:0] history,near_count,release_count;
 wire [15:0] bounded=raw>4500?16'd4500:raw;
 wire [15:0] median=bounded>h1?(h1>h2?h1:(bounded>h2?h2:bounded)):
                                      (bounded>h2?bounded:(h1>h2?h2:h1));
 assign confirm_count=near_count;
 assign usable=seen && age<LOST_CLKS;
 assign fresh=usable && age<STALE_CLKS;
 assign pending_expired=pending && pending_timer>=PENDING_CLKS;
 wire approaching=zone==1 && h1<h2 && h2<=NORMAL_MM && bounded<h1;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin seen<=0;age<=LOST_CLKS;distance<=0;h1<=0;h2<=0;history<=0;
   near_count<=0;release_count<=0;pending<=0;pending_timer<=0;zone<=2;accepted<=0;end
  else begin
   accepted<=0;if(age<LOST_CLKS)age<=age+1'b1;
   if(!pending)pending_timer<=0;else if(pending_timer<PENDING_CLKS)pending_timer<=pending_timer+1'b1;
   if(packet)begin
    age<=0; // 零值仅表示链路有活动，不作为新距离或净空确认。
    if(raw!=0)begin
     // 被拒绝的近障候选不进入可信滤波历史。
     if(!seen)begin seen<=1;distance<=bounded;h1<=bounded;h2<=bounded;history<=1;zone<=bounded<NEAR_MM?2:bounded<NORMAL_MM?1:0;accepted<=1;end
     else if(bounded<NEAR_MM)begin
      release_count<=0;
      if(zone==2 || near_count>=2 || approaching)begin
       distance<=bounded;h1<=bounded;h2<=bounded;history<=1;zone<=2;near_count<=0;pending<=0;accepted<=1;
      end else begin pending<=1;near_count<=near_count+1'b1;end
     end else begin
      near_count<=0;pending<=0;h2<=h1;h1<=bounded;if(history<2)history<=history+1'b1;
      if(pending)begin h1<=bounded;h2<=bounded;history<=1;end
      if(zone==2)begin
       if(bounded>=RELEASE_MM)begin
        if(release_count>=1)begin zone<=bounded<NORMAL_MM?1:0;distance<=bounded;accepted<=1;release_count<=0;end
        else release_count<=release_count+1'b1;
       end else release_count<=0;
      end else if(bounded<NORMAL_MM)begin
       distance<=bounded;zone<=1;release_count<=0;accepted<=1;
      end else begin
       // 退出预警需要两笔新的净空报文，普通远距波动使用中值处理。
       distance<=pending || history<2?bounded:median;accepted<=1;
       if(zone==1)begin
        if(release_count>=1)begin zone<=0;release_count<=0;end
        else release_count<=release_count+1'b1;
       end else begin zone<=0;release_count<=0;end
      end
     end
    end
   end
  end
 end
endmodule

// 纯超声波优先转向AUTO，使用定点计数，不调用除法器或几何求解器。
// 动作1/2为直行，3/4前进弧线，5/6原地转，7/8后退弧线；遥测编码0至14不变。
module robot_auto_sonar #(
 parameter HOLD_CLKS=5000000,ARC_CLKS=25000000,TURN_CLKS=20000000,
 parameter OBSERVE_CLKS=10000000,BACK_CLKS=100000000,
 parameter STALE_CLKS=30000000,LOST_CLKS=75000000,PENDING_CLKS=15000000,
 parameter HISTORY_CLKS=500000000,MAX_CREDIT=3735,BACK_STEP=622,
 parameter NEAR_TRAVEL=186,RESTORE_TOL=28,MAX_WINDOW_TICKS=512,
 // 按67.5mm轮径及290mm轮距，计算原地转20度和90度的平均轮计数。
 parameter TURN_PROGRESS_TICKS=315,TURN_LIMIT_TICKS=1418,
 parameter RESTORE_FINE_ZONE=132,RESTORE_CROSS_TOL=48,ARC_PROGRESS_TICKS=1244
)(input clk,rst_n,enable,input [2:0] packet,input [47:0] raw,
 input [3:0] actual,enc_ready,input [63:0] ticks,input [3:0] window_end,
 output reg [3:0] action,state_code,reason,output reg [10:0] speed_l,speed_r,
 output reg [3:0] attempts,output [1:0] light,
 output [47:0] ranges,output [5:0] zones,confirm_counts,output [2:0] usable,fresh,pending,pending_expired);
 localparam WAIT=0,RUN=1,ARC_L=2,TURN_L=3,TURN_R=4,RECOVER=5,
            ARC_R=6,BACK=7,SLOW=8,BACK_L=9,BACK_R=10,OBSERVE=11,BRAKE=12,
            RESTORE_L=13,RESTORE_R=14;
 wire [15:0] f=ranges[15:0],l=ranges[31:16],r=ranges[47:32];
 wire [1:0] zf=zones[1:0],zl=zones[3:2],zr=zones[5:4];
 wire [2:0] accepted;
 robot_auto_sonar_channel #(.STALE_CLKS(STALE_CLKS),.LOST_CLKS(LOST_CLKS),.PENDING_CLKS(PENDING_CLKS)) cf(
  .clk(clk),.rst_n(rst_n),.packet(packet[0]),.raw(raw[15:0]),.distance(ranges[15:0]),
  .zone(zones[1:0]),.usable(usable[0]),.fresh(fresh[0]),.pending(pending[0]),.pending_expired(pending_expired[0]),.confirm_count(confirm_counts[0+:2]),.accepted(accepted[0]));
 robot_auto_sonar_channel #(.NORMAL_MM(180),.NEAR_MM(100),.RELEASE_MM(130),.STALE_CLKS(STALE_CLKS),.LOST_CLKS(LOST_CLKS),.PENDING_CLKS(PENDING_CLKS)) cl(
  .clk(clk),.rst_n(rst_n),.packet(packet[1]),.raw(raw[31:16]),.distance(ranges[31:16]),
  .zone(zones[3:2]),.usable(usable[1]),.fresh(fresh[1]),.pending(pending[1]),.pending_expired(pending_expired[1]),.confirm_count(confirm_counts[2+:2]),.accepted(accepted[1]));
 robot_auto_sonar_channel #(.NORMAL_MM(180),.NEAR_MM(100),.RELEASE_MM(130),.STALE_CLKS(STALE_CLKS),.LOST_CLKS(LOST_CLKS),.PENDING_CLKS(PENDING_CLKS)) cr(
  .clk(clk),.rst_n(rst_n),.packet(packet[2]),.raw(raw[47:32]),.distance(ranges[47:32]),
  .zone(zones[5:4]),.usable(usable[2]),.fresh(fresh[2]),.pending(pending[2]),.pending_expired(pending_expired[2]),.confirm_count(confirm_counts[4+:2]),.accepted(accepted[2]));
 reg [31:0] timer,history_age,restore_timer;
 reg [15:0] credit,back_origin,progress_forward,near_travelled,arc_travel,arc_front,arc_side;
 reg [3:0] history_action,window_action,sample_action;
 reg window_mixed,sample_valid,sample_geometry;
 reg [15:0] sample_travel;
 reg signed [17:0] pivot_ticks,episode_origin,turn_origin;
 reg [1:0] back_level,resume_samples;
 reg chosen_right,tried_left,tried_right,blocked_episode,restore_positive;
 reg [2:0] near_ack;
 reg near_brake_pending;
 // 对应通道的清零命令通过驱动管线后，才确认该近障事件。
 // 同一持续近障随后可允许远离方向的原地转，不重复触发清零。
 wire [2:0] near_now={usable[2]&&zr==2,usable[1]&&zl==2,usable[0]&&zf==2};
 wire [2:0] near_new=near_now & ~near_ack;
 function [15:0] mag;input [15:0] n;begin mag=n[15]?(~n+16'd1):n;end endfunction
 // 检查实际曲率前先锁存每个编码器窗口，
 // 有效标记防止截断后的无效计数形成历史。
 localparam SIDE_BITS=$clog2(MAX_WINDOW_TICKS*2+1);
 reg [SIDE_BITS-1:0] captured_left,captured_right;
 reg [3:0] captured_action;
 reg captured_valid;
 wire enc_ok=&enc_ready;
 wire [SIDE_BITS:0] captured_total={1'b0,captured_left}+{1'b0,captured_right};
 wire [SIDE_BITS:0] side_diff=captured_left>captured_right?
   captured_left-captured_right:captured_right-captured_left;
 wire [SIDE_BITS-1:0] largest_side=captured_left>captured_right?captured_left:captured_right;
 wire [SIDE_BITS-1:0] inner=captured_action==3?captured_left:captured_right;
 wire [SIDE_BITS-1:0] outer=captured_action==3?captured_right:captured_left;
 wire [SIDE_BITS+2:0] inner5={inner,2'b00}+inner;
 wire [SIDE_BITS+2:0] outer2={outer,1'b0};
 wire [SIDE_BITS+2:0] ratio_diff=inner5>outer2?inner5-outer2:outer2-inner5;
 wire forward_geometry=captured_left!=0 && captured_right!=0 &&
   (captured_action==1?side_diff<=((largest_side>>2)+4):ratio_diff<=(outer>>1)+2);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   window_action<=0;window_mixed<=1;captured_valid<=0;captured_action<=0;
   captured_left<=0;captured_right<=0;
   sample_valid<=0;sample_action<=0;sample_travel<=0;sample_geometry<=0;
  end else begin
   captured_valid<=0;sample_valid<=captured_valid && enable;
   if(captured_valid)begin
    sample_action<=captured_action;sample_travel<=captured_total>>2;sample_geometry<=forward_geometry;
   end
   if(actual!=window_action)window_mixed<=1;
   if(window_end[0])begin
    window_action<=actual;window_mixed<=0;captured_action<=actual;
    captured_left<={1'b0,mag(ticks[15:0])}+{1'b0,mag(ticks[47:32])};
    captured_right<={1'b0,mag(ticks[31:16])}+{1'b0,mag(ticks[63:48])};
    captured_valid<=enable && enc_ok && !window_mixed && actual==window_action &&
      mag(ticks[15:0])<=MAX_WINDOW_TICKS && mag(ticks[31:16])<=MAX_WINDOW_TICKS &&
      mag(ticks[47:32])<=MAX_WINDOW_TICKS && mag(ticks[63:48])<=MAX_WINDOW_TICKS;
   end
  end
 end
 wire raw_aligned=pivot_ticks<=$signed(RESTORE_TOL) && pivot_ticks>=-$signed(RESTORE_TOL);
 wire restoring=state_code==RESTORE_L || state_code==RESTORE_R;
 wire [17:0] pivot_abs=pivot_ticks[17]?(~pivot_ticks+18'd1):pivot_ticks;
 // 跨越低速区的变化须受限，允许跨越不代表允许大幅跳变。
 wire restore_crossed=restoring && (restore_positive?pivot_ticks<0:pivot_ticks>0) && pivot_abs<=RESTORE_CROSS_TOL;
 wire restore_done=restoring && (raw_aligned || restore_crossed);
 wire heading_aligned=raw_aligned || restore_done;
 wire credit_ok=credit!=0 && history_age<HISTORY_CLKS;
 wire [16:0] credit_sum={1'b0,credit}+{1'b0,sample_travel};
 wire [16:0] progress_sum={1'b0,progress_forward}+{1'b0,sample_travel};
 wire [16:0] arc_sum={1'b0,arc_travel}+{1'b0,sample_travel};
 wire [16:0] near_sum={1'b0,near_travelled}+{1'b0,sample_travel};
 wire [15:0] stage_limit=back_level==0?BACK_STEP:back_level==1?BACK_STEP*2:back_level==2?BACK_STEP*3:BACK_STEP*4;
 wire [31:0] stage_time=back_level==0?BACK_CLKS:back_level==1?BACK_CLKS*2:back_level==2?BACK_CLKS*3:BACK_CLKS*4;
 wire back_distance_done=back_origin>=credit && back_origin-credit>=stage_limit;
 wire back_finished=back_distance_done || !credit_ok || timer>=stage_time;
 wire pending_stop=|pending_expired || ((|pending) && near_travelled>=NEAR_TRAVEL);
 wire front_pending_stop=pending_expired[0] || (pending[0] && near_travelled>=NEAR_TRAVEL);
 wire front_go=usable[0] && zf!=2 && !front_pending_stop;
 wire left_go=usable[1] && fresh[1] && zl!=2 && !pending[1];
 wire right_go=usable[2] && fresh[2] && zr!=2 && !pending[2];
 wire left_turn_ok=left_go && usable[2];
 wire right_turn_ok=right_go && usable[1];
 wire left_straight_ok=usable[1]?(zl!=2 && !(pending[1] && (pending_expired[1] || near_travelled>=NEAR_TRAVEL))):(l!=0 && zl==0);
 wire right_straight_ok=usable[2]?(zr!=2 && !(pending[2] && (pending_expired[2] || near_travelled>=NEAR_TRAVEL))):(r!=0 && zr==0);
 wire slow_zone=(zf!=0 || zl!=0 || zr!=0 || |pending || !(&fresh));
 wire prefer_right=!left_go && right_go?1'b1:!right_go && left_go?1'b0:
                   (zl>zr)?1'b1:(zr>zl)?1'b0:
                   ({1'b0,r}>{1'b0,l}+17'd10)?1'b1:({1'b0,l}>{1'b0,r}+17'd10)?1'b0:chosen_right;
 wire reverse_geometry_ok=history_action==1?(&usable):history_action==3?right_go:history_action==4?left_go:1'b0;
 // 18位模差计算保证计数回绕时短程相对转角仍有效。
 // 20秒有效后退历史最多102400个合理计数，低于有符号半量程131072。
 wire signed [17:0] turn_delta=$signed(pivot_ticks)-$signed(turn_origin);
 wire signed [17:0] episode_delta=$signed(pivot_ticks)-$signed(episode_origin);
 wire [17:0] turn_travel=turn_delta[17]?(~turn_delta+18'd1):turn_delta;
 wire turn_real_progress=turn_travel>=TURN_PROGRESS_TICKS;
 wire turn_limit=(state_code==TURN_L)?episode_delta>=$signed(TURN_LIMIT_TICKS):episode_delta<=-$signed(TURN_LIMIT_TICKS);
 wire front_resume=front_go && f>=250 && resume_samples>=2;
 // 灯光表示障碍严重程度，与当前脱困动作分开。
 assign light=!(&usable)?2'd3:(zf==2 || zl==2 || zr==2)?2'd0:slow_zone?2'd1:2'd2;
 // 只保留最近一段前进行程，曲率变化时替换历史额度，不累加延长。
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin credit<=0;history_age<=0;history_action<=0;pivot_ticks<=0;progress_forward<=0;near_travelled<=0;arc_travel<=0;end
  else if(!enable)begin credit<=0;history_age<=0;history_action<=0;pivot_ticks<=0;progress_forward<=0;near_travelled<=0;arc_travel<=0;end
  else begin
   if(history_age!=32'hffffffff)history_age<=history_age+1'b1;
   if(state_code!=ARC_L && state_code!=ARC_R)arc_travel<=0;
   if(!(|pending))near_travelled<=0;
   if(sample_valid && sample_travel!=0)begin
    if((state_code==ARC_L || state_code==ARC_R) && (sample_action==3 || sample_action==4))
      arc_travel<=arc_sum>65535?16'hffff:arc_sum[15:0];
    if(sample_action==1 || sample_action==3 || sample_action==4)begin
     history_age<=0;pivot_ticks<=0;history_action<=sample_geometry?sample_action:4'd0;
     if(!sample_geometry)credit<=0;
     else if(history_action!=sample_action || !raw_aligned || history_age>=HISTORY_CLKS)
       credit<=sample_travel>MAX_CREDIT?MAX_CREDIT:sample_travel;
     else credit<=credit_sum>MAX_CREDIT?MAX_CREDIT:credit_sum[15:0];
     progress_forward<=progress_sum>MAX_CREDIT?MAX_CREDIT:progress_sum[15:0];
     if(|pending)near_travelled<=near_sum>65535?16'hffff:near_sum[15:0];
    end else if(sample_action==5)begin
     pivot_ticks<=pivot_ticks+$signed({2'b0,sample_travel});
    end else if(sample_action==6)begin
     pivot_ticks<=pivot_ticks-$signed({2'b0,sample_travel});
    end else if(sample_action==2 || sample_action==7 || sample_action==8)begin
     progress_forward<=0;
     if(!heading_aligned || (sample_action==2 && history_action!=1) ||
       (sample_action==7 && history_action!=3) || (sample_action==8 && history_action!=4))credit<=0;
     else credit<=sample_travel>=credit?16'd0:credit-sample_travel;
    end
   end
   // 最终微调残差须在上限内，恢复完成后只建立一次新基准。
   if(restore_done)pivot_ticks<=0;
  end
 end
 reg [3:0] expected_action;
 always @*begin
  case(state_code)
   RUN,SLOW,RECOVER:expected_action=1;
   ARC_L:expected_action=3;ARC_R:expected_action=4;
   TURN_L,RESTORE_L:expected_action=5;TURN_R,RESTORE_R:expected_action=6;
   BACK:expected_action=2;BACK_L:expected_action=7;BACK_R:expected_action=8;
   default:expected_action=0;
  endcase
 end
 task next_state;input [3:0] n;begin state_code<=n;timer<=0;end endtask
 task start_turn;input go_right;
 begin
  chosen_right<=go_right;turn_origin<=pivot_ticks;resume_samples<=0;
  next_state(go_right?TURN_R:TURN_L);
 end endtask
 task begin_back;
 begin
  if(!credit_ok)next_state(OBSERVE);
  else if(!heading_aligned)begin
   restore_timer<=0;restore_positive<=!pivot_ticks[17];
   if(pivot_ticks>0 && right_turn_ok)next_state(RESTORE_R);
   else if(pivot_ticks<0 && left_turn_ok)next_state(RESTORE_L);
   else next_state(OBSERVE);
  end else if(reverse_geometry_ok)begin
   back_origin<=credit;next_state(history_action==3?BACK_L:history_action==4?BACK_R:BACK);
  end else next_state(OBSERVE);
 end endtask
 task choose_escape;
 begin
  // 已变为不安全的侧面直接跳过，不为累计尝试次数而向该侧运动。
  if(!tried_left && left_turn_ok && (!prefer_right || tried_right || !right_turn_ok))start_turn(0);
  else if(!tried_right && right_turn_ok)start_turn(1);
  else if(!tried_left && left_turn_ok)start_turn(0);
  else begin_back;
 end endtask
 task choose_motion;
 begin
  chosen_right<=prefer_right;
  if(!front_go || ((!left_straight_ok || !right_straight_ok) && !left_go && !right_go))begin
   if(!blocked_episode)begin
    blocked_episode<=1;episode_origin<=pivot_ticks;tried_left<=0;tried_right<=0;
    // 开始新一轮阻挡处理时不使用上一轮标记。
    if(prefer_right?right_turn_ok:left_turn_ok)start_turn(prefer_right);
    else if(prefer_right?left_turn_ok:right_turn_ok)start_turn(!prefer_right);
    else begin_back;
   end else choose_escape;
  end else if(zf==0 && zl==0 && zr==0 && !(|pending))begin
   next_state(RUN);blocked_episode<=0;tried_left<=0;tried_right<=0;
  end else if((pending[0] || (zf==0 && zl==zr)) && left_straight_ok && right_straight_ok)next_state(SLOW);
  else if(prefer_right?right_go:left_go)begin
   arc_front<=f;arc_side<=prefer_right?r:l;next_state(prefer_right?ARC_R:ARC_L);
  end
  else next_state(SLOW);
 end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state_code<=WAIT;timer<=0;chosen_right<=0;attempts<=0;back_origin<=0;back_level<=0;
   tried_left<=0;tried_right<=0;blocked_episode<=0;episode_origin<=0;turn_origin<=0;
   resume_samples<=0;restore_timer<=0;restore_positive<=0;arc_front<=0;arc_side<=0;near_ack<=0;near_brake_pending<=0;
  end else if(!enable)begin
   state_code<=WAIT;timer<=0;chosen_right<=0;attempts<=0;back_level<=0;
   tried_left<=0;tried_right<=0;blocked_episode<=0;episode_origin<=0;turn_origin<=0;
   resume_samples<=0;restore_timer<=0;restore_positive<=0;arc_front<=0;arc_side<=0;near_ack<=0;near_brake_pending<=0;
  end else begin
   // 某通道近障解除时，仅重新使能该通道的近障触发。
   near_ack<=near_ack & near_now;
   // 计时器只限定活动与期限，不能证明车轮实际移动。
   if(expected_action==0 || actual==expected_action)if(timer!=32'hffffffff)timer<=timer+1'b1;
   if(restoring && restore_timer!=32'hffffffff)restore_timer<=restore_timer+1'b1;
   if(packet[0] && raw[15:0]!=0)begin
    if(raw[15:0]>=250)begin if(resume_samples<2)resume_samples<=resume_samples+1'b1;end
    else resume_samples<=0;
   end
   if(progress_forward>=1244 && zf==0 && actual==1)back_level<=0;
   if(!enc_ok || !usable[0])begin
    // 就绪条件丢失时清除待执行原地转请求。
    // 就绪恢复后重新判断当前近障事件。
    near_brake_pending<=0;next_state(WAIT);
   end
   else if((|near_new) && state_code!=BRAKE)begin
    // 任一方向出现新的确认近障都先清零，即使当前在远离弧线或后退。
    // 重置方向尝试状态，保留仍有效的近期平移历史。
    near_brake_pending<=1;blocked_episode<=1;episode_origin<=pivot_ticks;
    tried_left<=0;tried_right<=0;next_state(BRAKE);
   end else case(state_code)
    WAIT:if((&usable) && timer>=HOLD_CLKS)choose_motion;
    RUN,SLOW,RECOVER:begin
     if(!front_go || !left_straight_ok || !right_straight_ok)next_state(BRAKE);
     else if(state_code==RECOVER && !front_resume)next_state(BRAKE);
     else if(timer>=HOLD_CLKS)choose_motion;
    end
    ARC_L,ARC_R:begin
     if(!front_go || (state_code==ARC_L?!left_go:!right_go))next_state(BRAKE);
     else if(zf==0 && zl==0 && zr==0 && !(|pending))begin
      next_state(RUN);blocked_episode<=0;tried_left<=0;tried_right<=0;
     end else if(arc_travel>=ARC_PROGRESS_TICKS && f<=arc_front+16'd20 &&
             (state_code==ARC_L?l:r)<=arc_side+16'd20)begin
      // 已实际完成弧线但前方或侧面仍未打开时，升级一次原地转向。
      blocked_episode<=1;episode_origin<=pivot_ticks;tried_left<=0;tried_right<=0;
      start_turn(state_code==ARC_R);
     end else if(timer>=ARC_CLKS)begin
      // 周期性重新判断，不强制定时停车；航向或侧向变化也算进展。
      timer<=0;
      if(zf==0 && zl==zr && left_straight_ok && right_straight_ok)next_state(SLOW);
     end
    end
    TURN_L,TURN_R:begin
     if(state_code==TURN_L?!left_turn_ok:!right_turn_ok)begin
      // 当前侧不安全时允许尝试另一侧，不将该切换当作编码器运动成功。
      if(state_code==TURN_L)tried_left<=1;else tried_right<=1;
      next_state(BRAKE);
     end else if(front_resume && turn_real_progress)begin
      next_state(RECOVER);blocked_episode<=0;tried_left<=0;tried_right<=0;
     end else if(turn_limit)begin
      if(state_code==TURN_L)tried_left<=1;else tried_right<=1;
      if(attempts!=15)attempts<=attempts+1'b1;
      next_state(BRAKE);
     end
     // 计数没有变化时保留本次尝试，由反馈保护负责电机恢复。
    end
    RESTORE_L,RESTORE_R:begin
     if(!credit_ok || restore_timer>=BACK_CLKS*4)next_state(OBSERVE);
     else if(restore_done)begin_back;
     else if((state_code==RESTORE_L && !left_turn_ok)||(state_code==RESTORE_R && !right_turn_ok))next_state(OBSERVE);
     else if((pivot_ticks>0 && state_code==RESTORE_L) || (pivot_ticks<0 && state_code==RESTORE_R))begin
      // 换向时保留restore_timer，避免无限来回切换。
      next_state(pivot_ticks>0?RESTORE_R:RESTORE_L);
     end
    end
    BACK,BACK_L,BACK_R:begin
     if(back_finished || !reverse_geometry_ok || !heading_aligned)begin
      if(back_origin>credit && back_level<3)back_level<=back_level+1'b1;
      next_state(OBSERVE);
     end
    end
    BRAKE:if(actual==0)begin
     // 此处确认命令已经清零，不代表车轮已完全静止。
     // 方向切换间隔仍位于清零确认之后。
     near_ack<=near_now;
     if(near_brake_pending || (|near_new))begin
      near_brake_pending<=0;
      if(!blocked_episode || (|near_new))begin
       blocked_episode<=1;episode_origin<=pivot_ticks;tried_left<=0;tried_right<=0;
       if(prefer_right?right_turn_ok:left_turn_ok)start_turn(prefer_right);
       else if(prefer_right?left_turn_ok:right_turn_ok)start_turn(!prefer_right);
       else begin_back;
      end else choose_escape;
     end else choose_motion;
    end
    OBSERVE:if(timer>=OBSERVE_CLKS)begin
     // 新观测可打开下一轮探索，不因累计尝试次数取消任务。
     tried_left<=0;tried_right<=0;blocked_episode<=0;next_state(BRAKE);
    end
    default:next_state(WAIT);
   endcase
  end
 end
 always @*begin
  action=0;speed_l=0;speed_r=0;reason=7;
  if(enable && enc_ok && usable[0])begin
   case(state_code)
    RUN,RECOVER,SLOW:if(front_go && left_straight_ok && right_straight_ok && (state_code!=RECOVER || front_resume))begin
     action=1;speed_l=(state_code!=RUN || slow_zone)?55:110;speed_r=speed_l;reason=(|pending)?5:speed_l==110?0:6;end
    ARC_L:if(front_go && left_go)begin action=3;speed_l=33;speed_r=77;reason=6;end
    ARC_R:if(front_go && right_go)begin action=4;speed_l=77;speed_r=33;reason=6;end
    TURN_L:if(left_turn_ok)begin action=5;speed_l=88;speed_r=88;reason=tried_right?7:6;end
    TURN_R:if(right_turn_ok)begin action=6;speed_l=88;speed_r=88;reason=tried_left?7:6;end
    RESTORE_L:if(credit_ok && left_turn_ok && !heading_aligned)begin action=5;speed_l=pivot_abs<=RESTORE_FINE_ZONE?33:66;speed_r=speed_l;reason=6;end
    RESTORE_R:if(credit_ok && right_turn_ok && !heading_aligned)begin action=6;speed_l=pivot_abs<=RESTORE_FINE_ZONE?33:66;speed_r=speed_l;reason=6;end
    BACK,BACK_L,BACK_R:if(credit_ok && !back_finished && heading_aligned && reverse_geometry_ok)begin
     action=state_code==BACK_L?7:state_code==BACK_R?8:2;
     // 28/66近似前进33/77的3比7曲率，每测速窗口误差不超过一个编码器计数。
     speed_l=state_code==BACK_L?28:66;speed_r=state_code==BACK_R?28:66;reason=6;end
    BRAKE:reason=pending_stop?5:zf==2?1:2;
    OBSERVE:reason=credit_ok?9:8;
    default:reason=7;
   endcase
  end
  // 新的近障在进入寄存BRAKE状态前，已使本拍目标立即归零。
  if(enable && (|near_new))begin action=0;speed_l=0;speed_r=0;reason=near_new[0]?1:2;end
  if(!enc_ok)reason=10;
  else if(!usable[0])reason=3;
 end
endmodule
