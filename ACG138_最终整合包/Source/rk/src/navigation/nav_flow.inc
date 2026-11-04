/* 矩形车体实时局部恢复，不访问设备，使用单一导航管理者。 */
static int flow_enabled(const nav_engine*n){return n->c.rectangular_footprint&&n->c.passage.continuous;}
void nav_set_chassis_hint(nav_engine*n,uint64_t stamp,const uint16_t mm[3],uint8_t valid,uint8_t fresh,uint8_t near,uint8_t degraded,uint8_t stable){
 if(!n||!mm)return;n->flow_hint_time=stamp;memcpy(n->flow_sonar,mm,sizeof(n->flow_sonar));n->flow_valid=valid&7;n->flow_fresh=fresh&7;n->flow_near=near&7;n->flow_degraded=degraded!=0;n->flow_front_stable=stable!=0;n->flow_have_hint=1;
}
static int flow_sonar_allows(const nav_engine*n,uint64_t now,double v,double w){
 /* 无底盘测试只使用实际扫描和地图，接入底盘时不从被忽略零值或缺失通道构造新距离。
 */
 if(!n->flow_have_hint)return 1;
 if(now<n->flow_hint_time||now-n->flow_hint_time>250000)return 0;
 unsigned f=n->flow_sonar[0],l=n->flow_sonar[1],r=n->flow_sonar[2];
 if(v<-.001)return (n->flow_valid&6)==6&&(n->flow_fresh&6)==6&&!(n->flow_near&6)&&l>=100&&r>=100;
 if(n->flow_valid!=7)return 0;
 if(fabs(v)<.001&&fabs(w)>.001)return n->flow_fresh==7&&!n->flow_degraded&&f>=30&&l>=100&&r>=100;
 if(v>.001){
  if((n->flow_near&1)||f<150||(n->flow_near&6)||l<100||r<100)return 0;
  if(fabs(w)>.05){
   unsigned side=w>0?l:r,other=w>0?r:l;
   if(n->flow_fresh!=7||n->flow_degraded||f<220||side<180||other<150)return 0;
   int same=n->turn_dir&&(w*n->turn_dir>0);
   if(!same&&(side<250||other<180))return 0;
  }
 }
 return 1;
}
static void flow_limit(double*v,double*w){
 const double wheel=.0675*PI*80/60,track=.29;
 double peak=fmax(fabs(*v-*w*track*.5),fabs(*v+*w*track*.5));if(peak>wheel){double s=wheel/peak;*v*=s;*w*=s;}
}
static void flow_episode(nav_engine*n,const lab_pose*p,uint64_t now,nav_reason why){
 if(!n->episode){n->episode=1;n->episode_started=now;n->episode_origin=(nav_xy){p->x_m,p->y_m};n->reverse_attempts=0;n->backup_total=0;n->turn_total=0;}
 n->blocking_reason=why;
}
static int flow_reverse_clear(nav_engine*n,const lab_pose*p,const nav_xy*points,unsigned count,uint64_t now,double distance){
 if(!n->c.require_scan||n->scan_time>now||(double)(now-n->scan_time)+n->time_uncertainty_s*1000000>250000||!rear_observed(n,p,points,count))return 0;
 if(!flow_sonar_allows(n,now,-.2,0))return 0;
 double age=(now-n->scan_time)/1000000.0+n->time_uncertainty_s;
 double reserve=.2*.2/(2*n->c.brake_m_s2)+.2*(age+.05)+.01;
 if(n->rear_clearance<distance+reserve)return 0;
 passage_pose start={p->x_m,p->y_m,p->yaw_rad};passage_world world=rectangle_world(n,start);world.confirmed_only=1;passage_result hit;
 passage_xy end={start.x-(distance+reserve)*cos(start.yaw),start.y-(distance+reserve)*sin(start.yaw)};
 return passage_translate_clear(&world,start,end,world.config.width_m,&hit);
}
static int flow_candidate(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,double v,double w,double*score){
 flow_limit(&v,&w);if(!flow_sonar_allows(n,now,v,w))return 0;
 if(sweep_reason(n,p,v,w,points,count)!=NAV_REASON_NONE)return 0;
 double x=p->x_m,y=p->y_m,a=p->yaw_rad;advance(&x,&y,&a,v,w,.8);
 double d=hypot(target.x_m-x,target.y_m-y),heading=fabs(wrap(atan2(target.y_m-y,target.x_m-x)-a));
 *score=4*d+1.2*path_distance(n,x,y)+.30*heading+.06*fabs(w-n->cmd.angular_rad_s)-.35*v+.35*n->flow_soft;
 return 1;
}
static lab_status flow_wait(nav_engine*n,uint64_t now,nav_reason why,nav_command*out){
 n->state=NAV_WAITING;n->reason=why;command(n,now,0,0,out);return LAB_OK;
}
static lab_status flow_step(nav_engine*n,uint64_t now,const lab_pose*p,nav_xy target,double preferred_v,double preferred_w,const nav_xy*points,unsigned count,int new_scan,nav_command*out){
 const double pivot=90*PI*.0675/(60*.29);
 if(!n->flow_have_progress){n->flow_have_progress=1;n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_yaw=p->yaw_rad;}
 double progress=hypot(p->x_m-n->flow_progress_pose.x_m,p->y_m-n->flow_progress_pose.y_m),rotation=fabs(wrap(p->yaw_rad-n->flow_progress_yaw));
 if(progress>.06||rotation>.25){n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_yaw=p->yaw_rad;}
 if(n->phase==NAV_PHASE_BACKUP){
  double moved=hypot(p->x_m-n->backup_start.x_m,p->y_m-n->backup_start.y_m);n->backup_progress=moved;
  double speed=fmax(.02,fabs(n->measured_v)),age=(now>=n->scan_time?(now-n->scan_time)/1000000.0:1)+n->time_uncertainty_s;
  double stopping=speed*speed/(2*n->c.brake_m_s2)+speed*(age+.05)+.01;
  if(moved>=.20||now-n->phase_started>=2500000||moved+stopping>=.12){phase_to(n,NAV_PHASE_REJOIN,now);return flow_wait(n,now,NAV_LOW_RECOVERY,out);}
  if(!flow_reverse_clear(n,p,points,count,now,fmin(.12-moved,.025))){phase_to(n,NAV_PHASE_WAIT,now);return flow_wait(n,now,n->rear_clearance<0?NAV_REAR_UNOBSERVED:NAV_REAR_BLOCKED,out);}
  return recovering_command(n,now,NAV_BACKING,-.20,0,out);
 }
 if(n->phase==NAV_PHASE_REJOIN){
  if(now-n->phase_started<150000||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,NAV_LOW_RECOVERY,out);
  n->dirty=1;n->phase=NAV_PHASE_TRACK;n->flow_pivot=1;n->flow_progress_time=now;
 }
 double alpha=wrap(atan2(target.y_m-p->y_m,target.x_m-p->x_m)-p->yaw_rad);
 if(n->flow_pivot&&fabs(alpha)<20*PI/180)n->flow_pivot=0;
 if(fabs(alpha)>50*PI/180)n->flow_pivot=1;
 if(n->flow_pivot){preferred_v=0;preferred_w=copysign(fmin(pivot,n->c.max_yaw_rate_rad_s),alpha);}
 else if(fabs(alpha)>15*PI/180){preferred_v=fmin(.1767145868,n->c.max_speed_m_s);preferred_w=copysign(fmin(.7312327728,n->c.max_yaw_rate_rad_s),alpha);}
 if(n->narrow&&preferred_v>n->c.passage.narrow_speed_m_s)preferred_v=n->c.passage.narrow_speed_m_s;
 if(n->flow_have_hint&&!n->flow_front_stable&&preferred_v>.12)preferred_v=.12;
 flow_limit(&preferred_v,&preferred_w);
 double best=DBL_MAX,bv=0,bw=0,score=0;int selected=0,preferred=0;
 if(flow_candidate(n,p,target,points,count,now,preferred_v,preferred_w,&score)){
  best=score-.05;bv=preferred_v;bw=preferred_w;selected=1;preferred=1;
 }
 /* 优选动作通过且无远期预测冲突时保持连续，否则比较有界曲率备选。
 */
 if(!selected||n->flow_soft||n->episode){
  const double speeds[]={.25,.18,.12,0},rates[]={0,.35,-.35,.7,-.7,1.1,-1.1};
  for(unsigned pass=0;pass<2;pass++){
   for(unsigned i=0;i<4;i++)for(unsigned j=0;j<7;j++){
    double v=fmin(speeds[i],n->narrow?n->c.passage.narrow_speed_m_s:n->c.max_speed_m_s),w=clamp(rates[j],-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);
    if(v==0&&w==0)continue;if(n->flow_pivot&&v>0)continue;
    if(pass==0&&now<n->flow_direction_until&&n->turn_dir&&w*n->turn_dir<-.05)continue;
    if(n->flow_have_hint&&!n->flow_front_stable&&v>.12)v=.12;flow_limit(&v,&w);if(flow_candidate(n,p,target,points,count,now,v,w,&score)&&score<best){best=score;bv=v;bw=w;selected=1;preferred=0;}
   }
   if(selected)break; /* 同侧动作全部失败后才解除方向锁定。 */
  }
 }
 int stalled=now>=n->flow_progress_time&&now-n->flow_progress_time>=3000000;
 if(selected&&!stalled){
  n->flow_blocked_since=0;
  if(!preferred){flow_episode(n,p,now,NAV_OBSTACLE);if(fabs(bw)>.1){int sign=bw>0?1:-1;if(sign!=n->turn_dir){n->turn_dir=sign;n->flow_direction_until=now+600000;}}
   n->phase=bv==0?NAV_PHASE_ALIGN:bw>0?NAV_PHASE_DETOUR_LEFT:NAV_PHASE_DETOUR_RIGHT;
  }else if(n->flow_pivot)n->phase=NAV_PHASE_ALIGN;
  else {
   int clear=!n->flow_have_hint||(n->flow_front_stable&&(n->flow_fresh&1));
   if(n->flow_have_hint)n->clear_scans=clear?2:0;else if(new_scan&&n->clear_scans<2)++n->clear_scans;
   if(n->clear_scans>=2){n->phase=NAV_PHASE_TRACK;n->turn_dir=0;n->blocking_reason=NAV_REASON_NONE;}
  }
  return recovering_command(n,now,n->phase==NAV_PHASE_TRACK?NAV_REASON_NONE:NAV_LOCAL_AVOID,bv,bw,out);
 }
 flow_episode(n,p,now,NAV_OBSTACLE);
 if(!n->flow_blocked_since)n->flow_blocked_since=now;
 n->phase=NAV_PHASE_BACK_CHECK;
 if(now-n->flow_blocked_since<150000||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,NAV_BACK_CHECK,out);
 if(n->reverse_attempts>=2){n->phase=NAV_PHASE_BLOCKED;return flow_wait(n,now,NAV_RECOVERY_EXHAUSTED,out);}
 if(!flow_reverse_clear(n,p,points,count,now,.12)){n->phase=NAV_PHASE_WAIT;return flow_wait(n,now,n->rear_clearance<0?NAV_REAR_UNOBSERVED:NAV_REAR_BLOCKED,out);}
 ++n->reverse_attempts;n->backup_start=(nav_xy){p->x_m,p->y_m};n->backup_progress=0;n->flow_blocked_since=0;
 phase_to(n,NAV_PHASE_BACKUP,now);return flow_wait(n,now,NAV_BACK_CHECK,out);
}
