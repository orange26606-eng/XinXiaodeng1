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
static int s5_current_sonar(nav_engine*n,double v,double w){
 if(!n->flow_have_hint)return 1;
 double f=s5_clearance(n,0),l=s5_clearance(n,1),r=s5_clearance(n,2);int turning=fabs(w)>.1,pivot=fabs(v)<1e-9;
 if((v>0||turning)&&f>=0&&f<.030){s5_reject(n,v,w,"CURRENT_FRONT_UNDER_30MM");return 0;}
 if(!turning&&fabs(v)>1e-9&&((l>=0&&l<.015)||(r>=0&&r<.015))){s5_reject(n,v,w,"CURRENT_SIDE_UNDER_15MM");return 0;}
 if(turning&&(((pivot||w>0)&&l>=0&&l<.030)||((pivot||w<0)&&r>=0&&r<.030))){s5_reject(n,v,w,"CURRENT_TURN_SIDE_UNDER_30MM");return 0;}
 return 1;
}
static int s5_sonar_geometry(nav_engine*n,double v,double w,double horizon){(void)horizon;return s5_current_sonar(n,v,w);}
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
 /* 历史栅格证据仅供建议，执行所用走廊测量只来自上方当前RAW点。
 */
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
