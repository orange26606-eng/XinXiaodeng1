/* 时间戳使用原观测或首次收到的新FPGA样本；重复NCS状态保留同一样本身份。 */
static double s5_clearance(const nav_engine*n,unsigned side){
 uint64_t now=n->last_tick>n->s5_policy_now?n->last_tick:n->s5_policy_now;
 if((n->s5_raw_available&(1u<<side))&&n->s5_diag_time<=now&&now-n->s5_diag_time<=250000&&n->s5_raw_age_us[side]<=250000-(now-n->s5_diag_time)&&n->s5_raw_mm[side]>0&&n->s5_raw_mm[side]<(side?30:51))return n->s5_raw_mm[side]/1000.+(side?n->c.passage.sonar_side_offset_m:n->c.passage.sonar_front_offset_m);
 if(!n->flow_have_hint||!(n->flow_valid&(1u<<side))||!(n->flow_fresh&(1u<<side))||
    (n->s5_raw_zero_mask&(1u<<side))||!n->flow_sonar[side])return -1;
 return n->flow_sonar[side]/1000.+(side?n->c.passage.sonar_side_offset_m:n->c.passage.sonar_front_offset_m);
}
static double s5_body_pad(const passage_config*c){
 return c->margin_m+c->wheel_outer_margin_m+(c->wheel_outer_margin_verified?0:.01)+(c->sonar_offsets_verified?0:.02);
}
static void s5_reject(nav_engine*n,double v,double w,const char*why){
 n->s5_rejected_v=v;n->s5_rejected_w=w;snprintf(n->s5_rejected_reason,sizeof(n->s5_rejected_reason),"%s",why);
}
void nav_set_chassis_samples_s5(nav_engine*n,uint64_t stamp,uint8_t sequence,uint8_t valid,uint8_t zeros){
 if(!n)return;n->s5_raw_zero_mask=zeros&7;n->s5_sequence_valid=valid!=0;
 if(!valid||!stamp)return;
 sequence&=3;if(n->s5_sample_seen&&sequence==n->s5_front_sequence)return;
 if(n->s5_sample_seen&&stamp<=n->s5_sample_time)return;
 n->s5_previous_sample_time=n->s5_sample_time;n->s5_sample_time=stamp;
 n->s5_front_sequence=sequence;n->s5_sample_seen=1;n->s5_sample_new=1;
}
void nav_set_chassis_raw_s5(nav_engine*n,uint64_t stamp,uint32_t uptime,const uint16_t raw[3],const uint32_t seq[3],const uint32_t age[3],uint8_t available){
 if(!n||!stamp||!raw||!seq||!age)return;
 if(n->s5_diag_time&&stamp<n->s5_diag_time)return;
 if(n->s5_diag_time&&uptime<n->s5_source_uptime_ms&&n->s5_source_uptime_ms-uptime<0x80000000u){
  memset(n->s5_raw_seq,0,sizeof(n->s5_raw_seq));n->s5_emergency_hits=n->s5_glitch_clears=0;n->s5_emergency_sample_time=0;
 }
 n->s5_diag_time=stamp;n->s5_source_uptime_ms=uptime;n->s5_raw_available=available&7;
 for(unsigned j=0;j<3;j++){
  n->s5_raw_mm[j]=raw[j];n->s5_raw_age_us[j]=age[j];
  uint64_t source=(uint64_t)uptime*1000;source=source>=age[j]?source-age[j]:0;
  if(available&(1u<<j)){
   if(!raw[j])n->s5_raw_zero_mask|=1u<<j;else n->s5_raw_zero_mask&=~(1u<<j);
   if(raw[j]&&seq[j]!=n->s5_raw_seq[j]&&(!n->s5_raw_seq[j]||(int32_t)(seq[j]-n->s5_raw_seq[j])>0)){
    n->s5_raw_new_mask|=1u<<j;n->s5_raw_seq[j]=seq[j];n->s5_raw_source_us[j]=source;
   }
  }
 }
}
/* 检查全部车角，尤其车头远离墙时先向墙扫过的后角。侧超声波只保守支持当前车侧平面，不构造其后方观测。

 */
static int s5_sonar_geometry(nav_engine*n,double v,double w,double horizon){
 if(!n->flow_have_hint)return 1;
 double pad=s5_body_pad(&n->c.passage),hx=n->c.passage.length_m*.5+pad,hy=n->c.passage.width_m*.5+pad;
 double clear[3]={s5_clearance(n,0),s5_clearance(n,1),s5_clearance(n,2)};
 double beam=n->c.passage.sonar_beam_margin_m,x=0,y=0,a=0;
 double minimum[2]={1e6,1e6},last[2]={1e6,1e6};
 unsigned steps=(unsigned)ceil(horizon/.02);if(steps<1)steps=1;if(steps>256)return 0;
 for(unsigned k=0;k<=steps;k++){
  double cs=cos(a),sn=sin(a);
  if(clear[0]>=0){
   double ox=n->c.passage.length_m*.5+clear[0],dx=ox-x,dy=-y;
   double bx=cs*dx+sn*dy,by=-sn*dx+cs*dy;
   if(fabs(bx)<hx-1e-6&&fabs(by)<hy+beam-1e-6){s5_reject(n,v,w,"FRONT_BODY_SWEEP");return 0;}
  }
  for(unsigned side=0;side<2;side++)if(clear[side+1]>=0){
   double sign=side?-1:1,remaining=1e6,initial=clear[side+1],baseline=initial-pad;
   /* 超声波回波表示有限支撑线段，不是无限墙。点到旋转矩形精确距离包含四个车角，特别是远离转向的另一后角。

 */
   for(int ray=-1;ray<=1;ray++){
    double ox=ray*beam,oy=sign*(n->c.passage.width_m*.5+initial);
    double dx=ox-x,dy=oy-y,bx=cs*dx+sn*dy,by=-sn*dx+cs*dy;
    double ex=fabs(bx)-hx,ey=fabs(by)-hy;
    double distance=hypot(fmax(0,ex),fmax(0,ey))+fmin(fmax(ex,ey),0);
    remaining=fmin(remaining,distance);
   }
   minimum[side]=fmin(minimum[side],remaining);last[side]=remaining;
   if(remaining<-.0001){s5_reject(n,v,w,side?"SIDE_ACTION_REJECT_RIGHT_SWEEP":"SIDE_ACTION_REJECT_LEFT_SWEEP");return 0;}
   if(k&&initial>=.030&&remaining+pad<.030&&remaining<baseline-.001){s5_reject(n,v,w,side?"SIDE_ACTION_REJECT_RIGHT_30MM":"SIDE_ACTION_REJECT_LEFT_30MM");return 0;}
  }
  if(k<steps)advance(&x,&y,&a,v,w,horizon/steps);
 }
 for(unsigned side=0;side<2;side++)if(clear[side+1]>=.030&&(clear[side+1]<.080||n->s5_side_state[side]>=2)){
  double baseline=clear[side+1]-pad;int away=side?w>0:w<0;
  if(minimum[side]<baseline-.001&&!away){s5_reject(n,v,w,side?"SIDE_ACTION_REJECT_RIGHT_CLOSER":"SIDE_ACTION_REJECT_LEFT_CLOSER");return 0;}
 }
 for(unsigned side=0;side<2;side++)if(clear[side+1]>=0&&clear[side+1]<.030){
  double baseline=clear[side+1]-pad;
  if(last[side]<=baseline+.001||minimum[side]<baseline-.001){s5_reject(n,v,w,side?"SIDE_EMERGENCY_RIGHT_NOT_AWAY":"SIDE_EMERGENCY_LEFT_NOT_AWAY");return 0;}
 }
 return 1;
}
/* 诊断局部预测使用同一选定命令，不新增第二路线规划器，也不重写全局参考路线。
 */
static void s5_command_path(nav_engine*n,uint64_t now,double v,double w){
 n->s5_command_path_n=0;if(!n->have_motion_pose)return;
 double x=n->motion_pose.x_m,y=n->motion_pose.y_m,a=n->motion_yaw;
 for(unsigned k=0;k<=32;k++){n->s5_command_path[n->s5_command_path_n++]=(nav_xy){x,y};advance(&x,&y,&a,v,w,.025);}
 if(n->path_n>1&&fabs(v)>.001){
  double route=atan2(n->path[1].y_m-n->path[0].y_m,n->path[1].x_m-n->path[0].x_m);
  double action=n->motion_yaw+(v<0?PI:0);
  if(fabs(wrap(route-action))>PI/6){
   if(!n->s5_path_mismatch_since)n->s5_path_mismatch_since=now;
   if(now>=n->s5_path_mismatch_since&&now-n->s5_path_mismatch_since>=300000&&
      (!n->s5_policy_log_time||now-n->s5_policy_log_time>=1000000)){
    fprintf(stderr,"PATH_ACTION_MISMATCH policy_revision=5 now_us=%llu route_rad=%.4f action_rad=%.4f actual_action_v=%.4f w=%.4f local_path_source=COMMAND global_route_preserved=1\n",(unsigned long long)now,route,action,v,w);n->s5_policy_log_time=now;
   }
  }else n->s5_path_mismatch_since=0;
 }
}
static void s5_corridor_observe(nav_engine*n,const lab_pose*p,const nav_xy*points,unsigned count,uint64_t now,const s3_nearfield_view*nf){
 double length=n->c.passage.front_corridor_m,hx=n->c.passage.length_m*.5;
 double half=n->c.passage.width_m*.5+n->c.passage.wheel_outer_margin_m+.05;
 n->s5_corridor_clearance=-1;n->s5_front_approach=0;
 if(n->scan_time>now||now-n->scan_time>=n->c.max_scan_age_us||!nf->valid)return;
 double cs=cos(p->yaw_rad),sn=sin(p->yaw_rad),nearest=length;
 unsigned hits=0;double sidefree[2]={length,length};
 for(unsigned k=0;k<count;k++){
  double dx=points[k].x_m-p->x_m,dy=points[k].y_m-p->y_m;
  double x=cs*dx+sn*dy,y=-sn*dx+cs*dy;
  if(x<hx||x>hx+length)continue;
  if(fabs(y)<=half){nearest=fmin(nearest,x-hx);hits++;}
  if(fabs(y)<=half+.6){unsigned side=y>=0?0:1;sidefree[side]=fmin(sidefree[side],fmax(.01,fabs(y)-half));}
 }
 /* 即使旧扫描简化漏掉细杆，独立当前RAW近场仍保留完整点。
 */
 lab_pose raw_current;int raw_pose_ok=nf_current(&n->nearfield,now,&raw_current);
 for(unsigned k=0;raw_pose_ok&&k<n->nearfield.latest_count;k++){
  double x,y;nf_body_point(&raw_current,n->nearfield.latest[k].x,n->nearfield.latest[k].y,&x,&y);
  if(x>=hx&&x<=hx+length&&fabs(y)<=half){nearest=fmin(nearest,x-hx);hits++;}
 }
 /* 同时读取近期实测栅格，查询限于旋转1.2m走廊的轴对齐包围盒，不遍历全图；未知或过期格既不确认障碍，也不证明净空。

 */
 if(raw_pose_ok&&n->nearfield.have_window){
  double rc=cos(raw_current.yaw_rad),rs=sin(raw_current.yaw_rad),xmin=DBL_MAX,xmax=-DBL_MAX,ymin=DBL_MAX,ymax=-DBL_MAX;
  for(unsigned k=0;k<4;k++){double bx=k&1?hx+length:hx,by=k&2?half:-half,wx=raw_current.x_m+rc*bx-rs*by,wy=raw_current.y_m+rs*bx+rc*by;xmin=fmin(xmin,wx);xmax=fmax(xmax,wx);ymin=fmin(ymin,wy);ymax=fmax(ymax,wy);}
  int32_t x0=(int32_t)floor(xmin/NF_CELL_M),x1=(int32_t)floor(xmax/NF_CELL_M),y0=(int32_t)floor(ymin/NF_CELL_M),y1=(int32_t)floor(ymax/NF_CELL_M);
  for(int32_t yy=y0;yy<=y1;yy++)for(int32_t xx=x0;xx<=x1;xx++){
   const nf_cell*q=nf_const_cell(&n->nearfield,xx,yy);if(!q||q->state!=NF_CELL_HIT||q->time>now||now-q->time>=NF_FRESH_US)continue;
   double x,y;nf_body_point(&raw_current,q->x,q->y,&x,&y);if(x>=hx&&x<=hx+length&&fabs(y)<=half){nearest=fmin(nearest,x-hx);hits++;}
  }
 }
 if(hits)n->s5_corridor_clearance=nearest;
 n->s5_front_approach=hits&&(nearest<n->c.passage.front_approach_m||nearest<=.6);
 for(unsigned side=0;side<2;side++)n->s5_corridor_scores[side]=sidefree[side];
}
static void s5_update(nav_engine*n,const lab_pose*p,const nav_xy*points,unsigned count,uint64_t now,int new_scan,const s3_nearfield_view*nf){
 unsigned previous_approach=n->s5_front_approach;n->s5_policy_now=now;s5_corridor_observe(n,p,points,count,now,nf);
 if(previous_approach!=n->s5_front_approach)fprintf(stderr,"FRONT_APPROACH policy_revision=5 now_us=%llu active=%u corridor_clearance_m=%.4f corridor_m=%.2f source_scan_us=%llu\n",(unsigned long long)now,n->s5_front_approach,n->s5_corridor_clearance,n->c.passage.front_corridor_m,(unsigned long long)n->scan_time);
 double front=s5_clearance(n,0);
 if(nf->valid&&nf->front_m>=0&&(front<0||nf->front_m<front))front=nf->front_m;
 if(n->s5_corridor_clearance>=0&&(front<0||n->s5_corridor_clearance<front))front=n->s5_corridor_clearance;
 n->s5_front_clearance=front;
 for(unsigned side=0;side<2;side++){
  double d=s5_clearance(n,side+1);n->s5_side_clearance[side]=d;if(d<0)continue;
  if(d<.030)n->s5_side_state[side]=3;
  else if(d<.080)n->s5_side_state[side]=2;
  else if(d>=.120)n->s5_side_state[side]=0;
  else if(n->s5_side_state[side]<2)n->s5_side_state[side]=1;
 }
 int hintfresh=n->flow_have_hint&&n->flow_hint_time<=now&&now-n->flow_hint_time<=250000;
 int rawfresh=(n->s5_raw_available&1)&&n->s5_diag_time<=now&&now-n->s5_diag_time<=250000&&n->s5_raw_age_us[0]<=250000-(now-n->s5_diag_time);
 int urgent=rawfresh?(n->s5_raw_mm[0]>0&&n->s5_raw_mm[0]<=50):
  (hintfresh&&(n->flow_valid&n->flow_fresh&1)&&!(n->s5_raw_zero_mask&1)&&n->flow_sonar[0]>0&&n->flow_sonar[0]<=50);
 int raw_new=(n->s5_raw_new_mask&1)&&rawfresh;n->s5_raw_new_mask=0;n->s5_sample_new=0;
 if(urgent&&!n->s5_emergency){
  n->s5_emergency=1;n->s5_emergency_confirmed=0;n->s5_emergency_hits=0;n->s5_glitch_clears=0;n->s5_emergency_since=now;n->s5_emergency_sample_time=0;
  fprintf(stderr,"NF_EMERGENCY policy_revision=5 now_us=%llu front_mm=%u raw_available=%u immediate_brake=1\n",(unsigned long long)now,rawfresh?n->s5_raw_mm[0]:n->flow_sonar[0],rawfresh);
 }
 if(n->s5_emergency&&raw_new&&rawfresh){
  if(urgent){
   /* 使用同一FPGA时钟源时间并加1ms量化余量，诊断接收时间只判断时效，不累计样本确认。
 */
   if(n->s5_emergency_sample_time&&n->s5_raw_source_us[0]>n->s5_emergency_sample_time&&n->s5_raw_source_us[0]-n->s5_emergency_sample_time+1000<=100000)n->s5_emergency_hits++;
   else n->s5_emergency_hits=1;
   n->s5_emergency_sample_time=n->s5_raw_source_us[0];n->s5_glitch_clears=0;
   if(n->s5_emergency_hits>=2)n->s5_emergency_confirmed=1;
  }else if(n->s5_raw_mm[0]>150&&n->s5_raw_mm[0]<=4500){
   if(n->s5_glitch_clears<3)n->s5_glitch_clears++;
   if(n->s5_glitch_clears>=3){n->s5_emergency=0;n->s5_emergency_confirmed=0;n->s5_resume_limited=1;fprintf(stderr,"SONAR_GLITCH_RELEASE policy_revision=5 now_us=%llu distinct_clear_samples=3\n",(unsigned long long)now);}
  }else n->s5_glitch_clears=0;
 }
 if(n->s5_emergency&&new_scan&&n->nearfield.have_scan&&n->nearfield.scan_time<=now&&now-n->nearfield.scan_time<NF_FRESH_US){
  lab_pose current;
  if(nf_current(&n->nearfield,now,&current))for(unsigned k=0;k<n->nearfield.latest_count;k++){
   double x,y;nf_body_point(&current,n->nearfield.latest[k].x,n->nearfield.latest[k].y,&x,&y);
   if(x>=n->c.passage.length_m*.5&&x-n->c.passage.length_m*.5<=.050000001&&fabs(y)<=n->c.passage.width_m*.5+.03){n->s5_emergency_confirmed=1;break;}
  }
 }
 /* 保持与低速恢复使用不同扫描令牌，静止保持消耗过的扫描不再计入独立的250mm恢复阶段。
 */
 if(n->s5_resume_limited&&!n->s5_turn_hold&&new_scan&&n->scan_time!=n->s5_recover_scan){
  n->s5_recover_scan=n->scan_time;
  if(front>=.25){if(n->s5_recover_clear<2)n->s5_recover_clear++;}else n->s5_recover_clear=0;
  if(n->s5_recover_clear>=2)n->s5_resume_limited=0;
 }
}
static double s5_speed_cap(const nav_engine*n){
 double d=n->s5_front_clearance,cap=.25;
 if(d<0||(n->s5_raw_zero_mask&1))cap=.12;
 else if(d<.10)cap=0;
 else if(d<.15)cap=.06;
 else if(d<=.30)cap=.12;
 else if(d<=.60)cap=.20;
 if(n->s5_resume_limited&&cap>.12)cap=.12;
 return cap;
}
static int s5_turn_hold_wait(nav_engine*n,int new_scan){
 if(!n->s5_turn_hold)return 0;
 if(new_scan&&n->scan_time!=n->s5_hold_scan){
  n->s5_hold_scan=n->scan_time;
  if(n->s5_front_clearance>=.10){if(n->s5_hold_clear<2)n->s5_hold_clear++;}else n->s5_hold_clear=0;
 }
 if(n->s5_hold_clear<2||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return 1;
 n->s5_turn_hold=0;n->s5_resume_limited=1;n->s5_recover_clear=0;n->s5_recover_scan=n->scan_time;
 n->s5_after_backup=n->s5_force_corner=0;
 return 0;
}
static int s5_candidate_policy(nav_engine*n,double v,double w){
 if(v<=0)return 1;
 double d=n->s5_front_clearance,cap=s5_speed_cap(n);
 if(d>=0&&d<=.150000001){s5_reject(n,v,w,d<.10?"NEAR_AVOID_FORWARD_FORBIDDEN":"FRONT_100_150_PIVOT_OR_BACKUP_ONLY");return 0;}
 if(v>cap+1e-6){s5_reject(n,v,w,"FRONT_SPEED_ENVELOPE");return 0;}
 if(v>0&&fabs(w)>1.1&&v<.11&&d>=0&&d<=.150000001){s5_reject(n,v,w,"ESCAPE_REQUIRES_150MM");return 0;}
 return 1;
}
