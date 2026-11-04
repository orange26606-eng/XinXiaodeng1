`timescale 1ns/1ps
// 六轴重力比例校正及一阶四元数积分。
// 使用有符号Q8.24并复用算术单元，欧拉角转换也在FPGA内完成。
// sample_us为寄存器读事务起始时刻，不代表DRDY边沿或曝光时刻。
module imu_attitude_core #(parameter CAL_SAMPLES=400)(
 input clk,rst_n,input[31:0]now_us,sample_us,sample_seq,
 input ag_valid,input[7:0]whoami,
 input signed[15:0]ax,ay,az,gx,gy,gz,
 output ready,output reg updated,valid,bias_ready,
 output reg[3:0]status,output reg[15:0]cal_count,
 output reg[127:0]quaternion,output reg signed[31:0]roll,pitch,yaw,
 output reg[31:0]result_seq,result_us,math_errors,gaps,
 output reg accel_used,output[95:0]bias_raw);
 localparam signed[31:0]ONE=16777216,PI=52707179,TWO_PI=105414357;
 reg[7:0]state,ret;reg[4:0]idx;reg have_sample,need_ref,integrate;
 reg[31:0]seen_seq,last_us,dt_us;
 reg signed[31:0]a[0:2],rawg[0:2],bias[0:2],sumg[0:2],ming[0:2],maxg[0:2];
 reg signed[31:0]q[0:3],tq[0:3],p[0:9],c[0:5],w[0:2],h[0:2];
 reg signed[31:0]sum,norm,dtq,vx,vy,vz,sine,yaw_ref,roll_pending,pitch_pending;
 reg use_accel;
 reg ms;reg[1:0]mo;reg signed[31:0]ma,mb;reg[63:0]mr;
 wire md,me;wire signed[31:0]mv;
 reg cs;reg signed[31:0]cx,cy;wire cd;wire signed[31:0]cv;
 fxp_math math_unit(.clk(clk),.rst_n(rst_n),.start(ms),.op(mo),.a(ma),.b(mb),.rad(mr),.busy(),.done(md),.error(me),.result(mv));
 atan2_cordic angle_unit(.clk(clk),.rst_n(rst_n),.start(cs),.x(cx),.y(cy),.busy(),.done(cd),.angle(cv));
 assign ready=state==0;

 assign bias_raw={bias[2],bias[1],bias[0]};
 task calc;input[1:0]op;input signed[31:0]aa,bb;input[63:0]rad;input[7:0]next;
 begin mo<=op;ma<=aa;mb<=bb;mr<=rad;ms<=1;ret<=next;state<=250;end endtask
 task atan;input signed[31:0]xx,yy;input[7:0]next;
 begin cx<=xx;cy<=yy;cs<=1;ret<=next;state<=251;end endtask
 function signed[31:0]abs32;input signed[31:0]x;begin abs32=x<0?-x:x;end endfunction
 function signed[31:0]wrap;input signed[31:0]x;
 begin wrap=x>PI?x-TWO_PI:(x< -PI?x+TWO_PI:x);end endfunction
 function signed[31:0]pa;input[4:0]i;begin case(i)
 0:pa=q[0];1:pa=q[1];2:pa=q[2];3:pa=q[3];4:pa=q[1];5:pa=q[0];6:pa=q[0];7:pa=q[2];8:pa=q[1];default:pa=q[0];endcase end endfunction
 function signed[31:0]pb;input[4:0]i;begin case(i)
 0:pb=q[0];1:pb=q[1];2:pb=q[2];3:pb=q[3];4:pb=q[3];5:pb=q[2];6:pb=q[1];7:pb=q[3];8:pb=q[2];default:pb=q[3];endcase end endfunction
 function signed[31:0]ca;input[4:0]i;begin case(i)
 0:ca=a[1];1:ca=a[2];2:ca=a[2];3:ca=a[0];4:ca=a[0];default:ca=a[1];endcase end endfunction
 function signed[31:0]cb;input[4:0]i;begin case(i)
 0:cb=vz;1:cb=vy;2:cb=vx;3:cb=vz;4:cb=vy;default:cb=vx;endcase end endfunction
 function signed[31:0]ia;input[4:0]i;begin case(i)
 0:ia=q[1];1:ia=q[2];2:ia=q[3];3:ia=q[0];4:ia=q[2];5:ia=q[3];6:ia=q[0];7:ia=q[1];8:ia=q[3];9:ia=q[0];10:ia=q[1];default:ia=q[2];endcase end endfunction
 function signed[31:0]ib;input[4:0]i;begin case(i)
 0:ib=h[0];1:ib=h[1];2:ib=h[2];3:ib=h[0];4:ib=h[2];5:ib=h[1];6:ib=h[1];7:ib=h[2];8:ib=h[0];9:ib=h[2];10:ib=h[1];default:ib=h[0];endcase end endfunction
 integer k;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=0;ret<=0;idx<=0;have_sample<=0;need_ref<=1;integrate<=0;seen_seq<=0;last_us<=0;dt_us<=0;
   updated<=0;valid<=0;bias_ready<=0;status<=0;cal_count<=0;roll<=0;pitch<=0;yaw<=0;result_seq<=0;result_us<=0;math_errors<=0;gaps<=0;accel_used<=0;
   sum<=0;norm<=0;dtq<=0;vx<=0;vy<=0;vz<=0;sine<=0;yaw_ref<=0;roll_pending<=0;pitch_pending<=0;quaternion<=128'h00000000000000000000000001000000;use_accel<=0;
   ms<=0;mo<=0;ma<=0;mb<=0;mr<=0;cs<=0;cx<=0;cy<=0;
   for(k=0;k<3;k=k+1)begin a[k]<=0;rawg[k]<=0;bias[k]<=0;sumg[k]<=0;ming[k]<=32767;maxg[k]<=-32768;w[k]<=0;h[k]<=0;end
   for(k=0;k<4;k=k+1)begin q[k]<=k==0?ONE:0;tq[k]<=0;end
   for(k=0;k<10;k=k+1)p[k]<=0;for(k=0;k<6;k=k+1)c[k]<=0;
  end else begin
   ms<=0;cs<=0;updated<=0;
   if(have_sample&&now_us-last_us>50000)begin valid<=0;if(state==0)status<=3;end
   case(state)
    0:begin
     if(!ag_valid||whoami!=8'h71)begin valid<=0;status<=0;end
     else if((!have_sample||sample_seq!=seen_seq)&&now_us-sample_us<=50000)begin
      seen_seq<=sample_seq;last_us<=sample_us;have_sample<=1;
      if(have_sample&&(sample_us==last_us||sample_us-last_us>100000))begin
       valid<=0;bias_ready<=0;cal_count<=0;need_ref<=1;gaps<=gaps+1'b1;status<=4;
       for(k=0;k<3;k=k+1)begin sumg[k]<=0;ming[k]<=32767;maxg[k]<=-32768;end
      end else begin
       dt_us<=have_sample?sample_us-last_us:0;
       a[0]<=$signed(ax)*32'sd2048;a[1]<=$signed(ay)*32'sd2048;a[2]<=$signed(az)*32'sd2048;
       rawg[0]<=gx;rawg[1]<=gy;rawg[2]<=gz;idx<=0;sum<=0;state<=10;
      end
     end
    end
    10:calc(0,a[idx],a[idx],0,11);
    11:begin sum<=sum+mv;if(idx==2)state<=12;else begin idx<=idx+1'b1;state<=10;end end
    12:begin
     use_accel<=sum>=12121539&&sum<=22187868; // 0.85g至1.15g区间的平方。
     if(sum<=0)begin use_accel<=0;norm<=ONE;idx<=0;state<=15;end
     else calc(2,0,0,{8'd0,sum[31:0],24'd0},13);
    end
    13:begin norm<=mv;idx<=0;state<=14;end
    14:calc(1,a[idx],norm,0,16);
    16:begin a[idx]<=mv;if(idx==2)state<=15;else begin idx<=idx+1'b1;state<=14;end end
    15:begin
     if(!bias_ready)begin
      status<=1;
      if(sum<10737418||sum>24159191||abs32(rawg[0])>983||abs32(rawg[1])>983||abs32(rawg[2])>983)begin
       cal_count<=0;status<=6;
       for(k=0;k<3;k=k+1)begin sumg[k]<=0;ming[k]<=32767;maxg[k]<=-32768;end
       state<=0;
      end else if(cal_count==CAL_SAMPLES-1)begin
       if((maxg[0]-ming[0]>98)||(maxg[1]-ming[1]>98)||(maxg[2]-ming[2]>98)||abs32(rawg[0]-ming[0])>98||abs32(rawg[0]-maxg[0])>98||abs32(rawg[1]-ming[1])>98||abs32(rawg[1]-maxg[1])>98||abs32(rawg[2]-ming[2])>98||abs32(rawg[2]-maxg[2])>98)begin
        cal_count<=0;status<=6;
        for(k=0;k<3;k=k+1)begin sumg[k]<=0;ming[k]<=32767;maxg[k]<=-32768;end
        state<=0;
       end else begin cal_count<=CAL_SAMPLES;idx<=0;state<=18;end
      end else begin
       cal_count<=cal_count+1'b1;
       for(k=0;k<3;k=k+1)begin sumg[k]<=sumg[k]+rawg[k];if(rawg[k]<ming[k])ming[k]<=rawg[k];if(rawg[k]>maxg[k])maxg[k]<=rawg[k];end
       state<=0;
      end
     end else begin idx<=0;state<=30;end
    end
    18:calc(3,sumg[idx]+rawg[idx],CAL_SAMPLES,0,19);
    19:begin bias[idx]<=mv;if(idx==2)begin bias_ready<=1;need_ref<=1;state<=20;end else begin idx<=idx+1'b1;state<=18;end end
    20:begin
     if(a[2]< -15938355)begin tq[0]<=0;tq[1]<=ONE;tq[2]<=0;tq[3]<=0;end
     else begin tq[0]<=ONE+a[2];tq[1]<=a[1];tq[2]<=-a[0];tq[3]<=0;end
     integrate<=0;idx<=0;sum<=0;state<=60;
    end
    30:calc(0,(rawg[idx]-bias[idx])<<<9,32'sd146489298,0,31);
    31:begin w[idx]<=mv;if(idx==2)state<=32;else begin idx<=idx+1'b1;state<=30;end end
    32:calc(0,$signed(dt_us),32'sd281474977,0,33);
    33:begin dtq<=mv;integrate<=1;idx<=0;state<=40;end
    40:calc(0,pa(idx),pb(idx),0,41);
    41:begin p[idx]<=mv;if(idx==9)state<=42;else begin idx<=idx+1'b1;state<=40;end end
    42:begin
     if(integrate)begin vx<=(p[4]-p[5])<<<1;vy<=(p[6]+p[7])<<<1;vz<=p[0]-p[1]-p[2]+p[3];idx<=0;state<=43;end
     else state<=80;
    end
    43:if(use_accel)calc(0,ca(idx),cb(idx),0,44);else begin idx<=0;state<=46;end
    44:begin c[idx]<=mv;if(idx==5)state<=45;else begin idx<=idx+1'b1;state<=43;end end
    45:begin
     w[0]<=w[0]+((c[0]-c[1])<<<1);w[1]<=w[1]+((c[2]-c[3])<<<1);w[2]<=w[2]+((c[4]-c[5])<<<1);
     idx<=0;state<=46;
    end
    46:calc(0,w[idx],dtq,0,47);
    47:begin h[idx]<=mv>>>1;if(idx==2)begin idx<=0;state<=50;end else begin idx<=idx+1'b1;state<=46;end end
    50:calc(0,ia(idx),ib(idx),0,51);
    51:begin
     case(idx)
      0:tq[0]<=q[0]-mv;1,2:tq[0]<=tq[0]-mv;
      3:tq[1]<=q[1]+mv;4:tq[1]<=tq[1]+mv;5:tq[1]<=tq[1]-mv;
      6:tq[2]<=q[2]+mv;7:tq[2]<=tq[2]-mv;8:tq[2]<=tq[2]+mv;
      9:tq[3]<=q[3]+mv;10:tq[3]<=tq[3]+mv;11:tq[3]<=tq[3]-mv;
     endcase
     if(idx==11)begin integrate<=0;idx<=0;sum<=0;state<=60;end else begin idx<=idx+1'b1;state<=50;end
    end
    60:calc(0,tq[idx],tq[idx],0,61);
    61:begin sum<=sum+mv;if(idx==3)state<=62;else begin idx<=idx+1'b1;state<=60;end end
    62:if(sum<ONE/4||sum>ONE*8)begin math_errors<=math_errors+1'b1;bias_ready<=0;valid<=0;cal_count<=0;status<=5;state<=0;need_ref<=1;
      for(k=0;k<3;k=k+1)begin sumg[k]<=0;ming[k]<=32767;maxg[k]<=-32768;end
     end
     else calc(2,0,0,{8'd0,sum[31:0],24'd0},63);
    63:begin norm<=mv;idx<=0;state<=64;end
    64:calc(1,tq[idx],norm,0,65);
    65:begin q[idx]<=mv;if(idx==3)begin idx<=0;state<=40;end else begin idx<=idx+1'b1;state<=64;end end
    80:atan(ONE-((p[1]+p[2])<<<1),(p[6]+p[7])<<<1,81);
    81:begin roll_pending<=cv;sine<=(p[5]-p[4])<<<1;state<=82;end
    82:begin if(sine>ONE)sine<=ONE;else if(sine< -ONE)sine<=-ONE;state<=83;end
    83:calc(0,sine,sine,0,84);
    84:calc(2,0,0,{8'd0,(32'd16777216-$unsigned(mv)),24'd0},85);
    85:atan(mv,sine,86);
    86:begin pitch_pending<=cv;state<=87;end
    87:atan(ONE-((p[2]+p[3])<<<1),(p[9]+p[8])<<<1,88);
    88:begin
     if(need_ref)begin yaw_ref<=cv;yaw<=0;need_ref<=0;end else yaw<=wrap(cv-yaw_ref);
     quaternion<={q[3],q[2],q[1],q[0]};roll<=roll_pending;pitch<=pitch_pending;accel_used<=use_accel;
     result_seq<=seen_seq;result_us<=last_us;valid<=1;status<=2;updated<=1;state<=0;
    end
    250:if(md)begin
     if(me)begin math_errors<=math_errors+1'b1;valid<=0;bias_ready<=0;cal_count<=0;need_ref<=1;status<=5;state<=0;
      for(k=0;k<3;k=k+1)begin sumg[k]<=0;ming[k]<=32767;maxg[k]<=-32768;end
     end else state<=ret;
    end
    251:if(cd)state<=ret;
    default:state<=0;
   endcase
  end
 end
endmodule
