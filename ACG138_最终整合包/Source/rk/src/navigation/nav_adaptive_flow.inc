/* 自适应实时控制，路线失败仍保留目标，实际近场检查继续拒绝不合适动作，不访问设备，也不新增控制持有者。
 */
static int flow_enabled(const nav_engine*n){return n->c.rectangular_footprint&&n->c.passage.continuous;}
void nav_set_chassis_hint(nav_engine*n,uint64_t stamp,const uint16_t mm[3],uint8_t valid,uint8_t fresh,uint8_t near,uint8_t degraded,uint8_t stable){
 if(!n||!mm)return;n->flow_hint_time=stamp;memcpy(n->flow_sonar,mm,sizeof(n->flow_sonar));n->flow_valid=valid&7;n->flow_fresh=fresh&7;n->flow_near=near&7;n->flow_degraded=degraded!=0;n->flow_front_stable=stable!=0;n->flow_have_hint=1;
}
static int flow_sonar_allows(const nav_engine*n,uint64_t now,double v,double w){
 if(!n->flow_have_hint)return 1;
 if(now<n->flow_hint_time||now-n->flow_hint_time>nav_feedback_age_limit(n))return 0;
 unsigned f=n->flow_sonar[0],l=n->flow_sonar[1],r=n->flow_sonar[2];
 if(v<-.001)return (n->flow_valid&6)==6&&(n->flow_fresh&6)==6;
 if(fabs(v)<.001&&fabs(w)>.001)return n->flow_valid==7&&n->flow_fresh==7&&f>=30&&l>=30&&r>=30;
 if(v>.001){
  if(fabs(w)<=.05)return (n->flow_valid&1)&&(n->flow_fresh&1)&&!(n->flow_near&1)&&f>=150;
  unsigned mask=w>0?3u:5u,side=w>0?l:r;
  return (n->flow_valid&mask)==mask&&(n->flow_fresh&mask)==mask&&f>=50&&side>=30;
 }
 return 1;
}
static double flow_sonar_cost(const nav_engine*n,double v,double w){
 if(!n->flow_have_hint)return 0;double left=(n->flow_valid&2)?clamp((150.-n->flow_sonar[1])/150.,0,1):0;
 double right=(n->flow_valid&4)?clamp((150.-n->flow_sonar[2])/150.,0,1):0;
 /* 侧向预警用于引导远离，不否决整车；侧距例如93mm时仍可允许平行运动或已核对后方的退行。
 */
 double cost=.50*(left*fmax(w,0)+right*fmax(-w,0))-.10*(left*fmax(-w,0)+right*fmax(w,0));
 if(v>0&&(n->flow_valid&1)&&n->flow_sonar[0]<220)cost+=.15*(220.-n->flow_sonar[0])/220.;return cost;
}
static void flow_limit(double*v,double*w){
 const double wheel=.0675*PI*80/60,track=.29;double peak=fmax(fabs(*v-*w*track*.5),fabs(*v+*w*track*.5));
 if(peak>wheel){double s=wheel/peak;*v*=s;*w*=s;}
}
static void flow_episode(nav_engine*n,const lab_pose*p,uint64_t now,nav_reason why){
 if(!n->episode){n->episode=1;n->episode_started=now;n->episode_origin=(nav_xy){p->x_m,p->y_m};n->reverse_attempts=0;n->backup_total=0;n->turn_total=0;n->flow_forward_progress=0;n->flow_forward_origin=n->episode_origin;}
 n->blocking_reason=why;
}
static passage_world flow_local_world(const nav_engine*n,passage_pose p){
 passage_world world=rectangle_world(n,p);world.confirmed_only=1;
 /* 越过旧全局占用进行探索时，需当前明确观测的自由几何；局部扫描缺失不能开放通行。
 */
 if(n->c.require_scan&&n->local&&n->local->initialized){world.local_only=1;world.observed_probe=1;world.probe_unknown_fraction=.20;world.probe_unknown_margin_m=.05;}
 return world;
}
static int flow_reverse_clear(nav_engine*n,const lab_pose*p,const nav_xy*points,unsigned count,uint64_t now,double distance){
 if(!n->c.require_scan||n->scan_time>now||(double)(now-n->scan_time)+n->time_uncertainty_s*1000000>n->c.max_scan_age_us){n->flow_rear_reason=NAV_REAR_SCAN_STALE;return 0;}
 if(!rear_observed(n,p,points,count)){n->flow_rear_reason=NAV_REAR_COVERAGE;return 0;}
 if(!flow_sonar_allows(n,now,-.2,0)){n->flow_rear_reason=NAV_REAR_SONAR_STALE;return 0;}
 double age=(now-n->scan_time)/1000000.0+n->time_uncertainty_s;
 double reserve=.2*.2/(2*n->c.brake_m_s2)+.2*(age+.05)+.01;
 if(n->rear_clearance<distance+reserve){n->flow_rear_reason=NAV_REAR_CLEARANCE;return 0;}
 passage_pose start={p->x_m,p->y_m,p->yaw_rad};passage_world world=flow_local_world(n,start);passage_result hit;
 passage_xy end={start.x-(distance+reserve)*cos(start.yaw),start.y-(distance+reserve)*sin(start.yaw)};
 int clear=passage_translate_clear(&world,start,end,world.config.width_m,&hit);
 n->flow_rear_reason=clear?NAV_REASON_NONE:hit.reason==PASSAGE_OCCUPIED?NAV_REAR_BODY_OCCUPIED:NAV_REAR_GEOMETRY_UNKNOWN;return clear;
}
static double flow_failure_cost(const nav_engine*n,const lab_pose*p,double yaw,uint64_t now){
 double cost=0;for(unsigned i=0;i<8;i++)if(n->flow_failed[i].expires>now&&hypot(p->x_m-n->flow_failed[i].p.x_m,p->y_m-n->flow_failed[i].p.y_m)<.5&&fabs(wrap(yaw-n->flow_failed[i].yaw))<.45)cost+=1.;return cost;
}
static void flow_remember(nav_engine*n,const lab_pose*p,uint64_t now){
 unsigned i=n->flow_failed_next++%8;n->flow_failed[i].p=(nav_xy){p->x_m,p->y_m};n->flow_failed[i].yaw=p->yaw_rad;n->flow_failed[i].expires=now+8000000;
}
/* 采用VFH式极坐标空隙偏好，不绕过占用地图。有效回波提供角域覆盖，缺失回波不形成空闲桶；各候选仍由完整矩形扫掠核对。

 */
static int flow_gap_target(nav_engine*n,const lab_pose*p,nav_xy wanted,const nav_xy*points,unsigned count,uint64_t now,nav_xy*out){
 double ranges[72]={0},limits[72];for(unsigned i=0;i<72;i++)limits[i]=30;
 for(unsigned j=0;j<count;j++){
  double dx=points[j].x_m-p->x_m,dy=points[j].y_m-p->y_m,d=hypot(dx,dy);if(d<.03)continue;
  double a=wrap(atan2(dy,dx)-p->yaw_rad);int b=(int)floor((a+PI)*72/(2*PI));if(b<0)b=0;if(b>71)b=71;
  if(!ranges[b]||d<ranges[b])ranges[b]=d;
  double spread=asin(fmin(1,(n->c.passage.width_m*.5+.025)/d));int pad=(int)ceil(spread*72/(2*PI));
  for(int k=-pad;k<=pad;k++){unsigned q=(unsigned)(b+k+144)%72;if(d<limits[q])limits[q]=d;}
 }
 double bearing=wrap(atan2(wanted.y_m-p->y_m,wanted.x_m-p->x_m)-p->yaw_rad),best=DBL_MAX;int found=0;
 for(unsigned b=0;b<72;b++){
  double a=-PI+(b+.5)*2*PI/72,clear=fmin(ranges[b],limits[b]);if(fabs(wrap(a-bearing))<=PI/72)a=bearing;if(!ranges[b]||clear<.40||fabs(a)>150*PI/180)continue;
  unsigned wide=0;for(int k=-3;k<=3;k++){unsigned q=(unsigned)((int)b+k+144)%72;if(ranges[q]&&fmin(ranges[q],limits[q])>=.40)wide++;}
  double score=(n->flow_cruise?.65:1.4)*fabs(wrap(a-bearing))+.35*fabs(a)-.10*fmin(clear,3)-.035*wide+flow_failure_cost(n,p,p->yaw_rad+a,now);
  if(n->turn_dir&&now<n->flow_direction_until&&a*n->turn_dir<-.1)score+=.8;
  if(score<best){best=score;double length=fmin(.9,fmax(.25,clear-.25));*out=(nav_xy){p->x_m+length*cos(p->yaw_rad+a),p->y_m+length*sin(p->yaw_rad+a)};found=1;}
 }
 return found;
}
static int flow_candidate(nav_engine*n,const lab_pose*p,nav_xy target,const nav_xy*points,unsigned count,uint64_t now,double v,double w,double*score){
 flow_limit(&v,&w);if(!flow_sonar_allows(n,now,v,w)){++n->flow_reject_sonar;return 0;}
 nav_reason unsafe=sweep_reason(n,p,v,w,points,count);if(unsafe!=NAV_REASON_NONE){if(unsafe==NAV_OBSTACLE)++n->flow_reject_occupied;else ++n->flow_reject_unknown;return 0;}++n->flow_accept;
 double x=p->x_m,y=p->y_m,a=p->yaw_rad;advance(&x,&y,&a,v,w,.8);
 double d=hypot(target.x_m-x,target.y_m-y),heading=fabs(wrap(atan2(target.y_m-y,target.x_m-x)-a));
 double goal_term=n->flow_cruise?0:.5*hypot(n->goal.x_m-x,n->goal.y_m-y);
 *score=4*d+goal_term+(n->flow_local_probe?0:1.2*path_distance(n,x,y))+.30*heading+.06*fabs(w-n->cmd.angular_rad_s)-.35*v+.35*n->flow_soft+.4*flow_failure_cost(n,p,a,now)+flow_sonar_cost(n,v,w);return 1;
}
static lab_status flow_wait(nav_engine*n,uint64_t now,nav_reason why,nav_command*out){n->flow_can_start=0;n->state=NAV_WAITING;n->reason=why;command(n,now,0,0,out);return LAB_OK;}
static lab_status flow_step(nav_engine*n,uint64_t now,const lab_pose*p,nav_xy target,double preferred_v,double preferred_w,const nav_xy*points,unsigned count,int new_scan,nav_command*out){
 const double pivot=90*PI*.0675/(60*.29);n->flow_can_start=0;n->flow_local_probe=n->path_n<2||n->flow_cruise;n->flow_reject_sonar=n->flow_reject_occupied=n->flow_reject_unknown=n->flow_accept=0;
 double current_goal=hypot(n->goal.x_m-p->x_m,n->goal.y_m-p->y_m);
 if(!n->flow_have_progress){n->flow_have_progress=1;n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_yaw=p->yaw_rad;n->flow_progress_goal=current_goal;}
 double progress=hypot(p->x_m-n->flow_progress_pose.x_m,p->y_m-n->flow_progress_pose.y_m);
 /* 仅旋转不算平移脱困，横向绕行需有明显位移，沿目标方向的小幅前进可更早计入进展。
 */
 if(progress>.12||(progress>.06&&n->flow_progress_goal-current_goal>.025)){n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_goal=current_goal;}
 if(n->phase==NAV_PHASE_BACKUP){
  double moved=-(p->x_m-n->backup_start.x_m)*cos(n->flow_backup_yaw)-(p->y_m-n->backup_start.y_m)*sin(n->flow_backup_yaw);n->backup_progress=fmax(0,moved);
  int executing=!n->flow_exec_known||n->flow_exec_ready;
  if(executing&&!n->flow_backup_execution_us)n->flow_backup_execution_us=now;
  if(!executing)n->flow_backup_execution_us=0; /* 预览不消费时间或尝试预算。 */
  if(executing&&!n->flow_backup_counted&&((moved>=.003&&n->measured_v<-.005)||(n->flow_wheel_reverse&&n->cmd.linear_m_s<-.001))){++n->reverse_attempts;n->flow_backup_counted=1;}
  double speed=fmax(.02,fabs(n->measured_v)),age=(now>=n->scan_time?(now-n->scan_time)/1000000.0:1)+n->time_uncertainty_s;
  double stopping=speed*speed/(2*n->c.brake_m_s2)+speed*(age+.05)+.01;
  if(moved>=.40||(n->flow_backup_execution_us&&now-n->flow_backup_execution_us>=3000000)||moved+stopping>=n->flow_backup_target){phase_to(n,NAV_PHASE_REJOIN,now);return flow_wait(n,now,NAV_LOW_RECOVERY,out);}
  /* 实际净空恢复且出现新的安全前进动作时，可提前结束后退。
 */
  double forward_score=0;if(new_scan&&moved>.06&&flow_candidate(n,p,target,points,count,now,.12,0,&forward_score)){phase_to(n,NAV_PHASE_REJOIN,now);return flow_wait(n,now,NAV_LOW_RECOVERY,out);}
  if(!flow_reverse_clear(n,p,points,count,now,fmin(n->flow_backup_target-fmax(0,moved),.025))){phase_to(n,NAV_PHASE_WAIT,now);return flow_wait(n,now,n->flow_rear_reason,out);}
  n->flow_can_start=1;return recovering_command(n,now,NAV_BACKING,-.20,0,out);
 }
 if(n->phase==NAV_PHASE_REJOIN){
  if(now-n->phase_started<50000||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,NAV_LOW_RECOVERY,out);
  n->dirty=1;n->phase=NAV_PHASE_TRACK;n->flow_pivot=1;n->flow_progress_time=now;
 }
 if(!n->flow_final_yaw&&(n->flow_cruise||n->flow_local_probe||n->episode)){nav_xy gap;if(flow_gap_target(n,p,target,points,count,now,&gap))target=gap;}
 double alpha=wrap(atan2(target.y_m-p->y_m,target.x_m-p->x_m)-p->yaw_rad);
 if(n->flow_pivot&&fabs(alpha)<20*PI/180){n->flow_pivot=0;
  /* 180度初始对准可能占用3秒进展窗口的大部分时间，每个显式目标只给一次切换宽限，不因持续旋转无限重置。

 */
  if(!n->flow_alignment_grace_used&&now>=n->goal_started&&now-n->goal_started<8000000){n->flow_alignment_grace_used=1;n->flow_progress_time=now;n->flow_progress_pose=(nav_xy){p->x_m,p->y_m};n->flow_progress_goal=current_goal;}
 }if(fabs(alpha)>50*PI/180)n->flow_pivot=1;
 if(n->flow_pivot){preferred_v=0;preferred_w=copysign(fmin(pivot,n->c.max_yaw_rate_rad_s),alpha);}
 else if(fabs(alpha)>15*PI/180){preferred_v=fmin(.1767145868,n->c.max_speed_m_s);preferred_w=copysign(fmin(.7312327728,n->c.max_yaw_rate_rad_s),alpha);}
 else if(n->flow_local_probe){preferred_v=n->c.max_speed_m_s;preferred_w=clamp(2*preferred_v*sin(alpha)/fmax(.35,hypot(target.x_m-p->x_m,target.y_m-p->y_m)),-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);}
 if(n->flow_final_yaw){preferred_v=0;preferred_w=clamp(2*alpha,-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);n->flow_pivot=1;}
 if(n->narrow&&preferred_v>n->c.passage.narrow_speed_m_s)preferred_v=n->c.passage.narrow_speed_m_s;
 if(n->flow_have_hint&&!n->flow_front_stable&&preferred_v>.12)preferred_v=.12;flow_limit(&preferred_v,&preferred_w);
 double best=DBL_MAX,bv=0,bw=0,score=0;int selected=0,preferred=0;
 if((fabs(preferred_v)>.001||fabs(preferred_w)>.001)&&flow_candidate(n,p,target,points,count,now,preferred_v,preferred_w,&score)){best=score-.05;bv=preferred_v;bw=preferred_w;selected=1;preferred=1;}
 if(!selected||n->flow_soft||n->episode){
  const double speeds[]={.25,.18,.12,0},rates[]={0,.35,-.35,.7,-.7,1.1,-1.1};
  for(unsigned pass=0;pass<2;pass++){
   for(unsigned i=0;i<4;i++)for(unsigned j=0;j<7;j++){
    double v=fmin(speeds[i],n->narrow?n->c.passage.narrow_speed_m_s:n->c.max_speed_m_s),w=clamp(rates[j],-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);
    if(v==0&&w==0)continue;if(n->flow_pivot&&v>0)continue;
    if(pass==0&&now<n->flow_direction_until&&n->turn_dir&&w*n->turn_dir<-.05)continue;
    if(n->flow_have_hint&&!n->flow_front_stable&&v>.12)v=.12;flow_limit(&v,&w);if(flow_candidate(n,p,target,points,count,now,v,w,&score)&&score<best){best=score;bv=v;bw=w;selected=1;preferred=0;}
   }
   if(selected)break;
  }
 }
 int stalled=now>=n->flow_progress_time&&now-n->flow_progress_time>=3000000;
 /* 后退预算耗尽仍保留目标，新打开的前方通路可在不同扫描确认后恢复，单纯旋转不重置预算。
 */
 if(stalled&&n->reverse_attempts>=3&&new_scan){
  if(selected&&bv>.03&&fabs(bw)<.7){if(n->clear_scans<2)++n->clear_scans;}else n->clear_scans=0;
  if(n->clear_scans>=2){stalled=0;n->flow_progress_time=now;}
 }
 if(selected&&(!stalled||(n->flow_exec_known&&!n->flow_exec_ready))){
  n->flow_blocked_since=0;n->flow_can_start=fabs(bv)>.001||fabs(bw)>.001;
  if(!preferred){flow_episode(n,p,now,NAV_OBSTACLE);if(fabs(bw)>.1){int sign=bw>0?1:-1;if(sign!=n->turn_dir){n->turn_dir=sign;n->flow_direction_until=now+600000;}}n->phase=bv==0?NAV_PHASE_ALIGN:bw>0?NAV_PHASE_DETOUR_LEFT:NAV_PHASE_DETOUR_RIGHT;}
  else if(n->flow_pivot)n->phase=NAV_PHASE_ALIGN;
  else {int clear=!n->flow_have_hint||(n->flow_front_stable&&(n->flow_fresh&1));if(n->flow_have_hint)n->clear_scans=clear?2:0;else if(new_scan&&n->clear_scans<2)++n->clear_scans;if(n->clear_scans>=2){n->phase=n->flow_local_probe?NAV_PHASE_PROBE:NAV_PHASE_TRACK;n->turn_dir=0;n->blocking_reason=NAV_REASON_NONE;}}
  nav_reason why=n->flow_cruise?NAV_CRUISE:n->flow_local_probe?NAV_LOCAL_PROBE:n->phase==NAV_PHASE_TRACK?NAV_REASON_NONE:NAV_LOCAL_AVOID;
  if(!n->flow_diag_time||now-n->flow_diag_time>=1000000){n->flow_diag_time=now;fprintf(stderr,"DIRECTIONAL_CANDIDATES now_us=%llu goal=%u selected_v=%.4f selected_w=%.4f accepted=%u sonar_rejected=%u occupied_rejected=%u coverage_rejected=%u observed_fraction=%.3f rear_reason=%s attempts=%u requests=%u\n",(unsigned long long)now,n->goal.id,bv,bw,n->flow_accept,n->flow_reject_sonar,n->flow_reject_occupied,n->flow_reject_unknown,n->flow_observed_fraction,nav_reason_text(n->flow_rear_reason),n->reverse_attempts,n->flow_backup_requests);}
  return recovering_command(n,now,why,bv,bw,out);
 }
 flow_episode(n,p,now,NAV_OBSTACLE);
 if(!n->flow_blocked_since){n->flow_blocked_since=now;flow_remember(n,p,now);}
 n->phase=NAV_PHASE_BACK_CHECK;
 if(now-n->flow_blocked_since<50000||fabs(n->measured_v)>.025||fabs(n->measured_w)>.1)return flow_wait(n,now,NAV_BACK_CHECK,out);
 if(n->reverse_attempts>=3){n->phase=NAV_PHASE_BLOCKED;return flow_wait(n,now,NAV_RECOVERY_EXHAUSTED,out);}
 /* 后退目标按观测净空缩短，0.25m不是强制最小距离。 */
 double distance=.25;while(distance>=.06&&!flow_reverse_clear(n,p,points,count,now,distance))distance-=.025;
 if(distance<.06){n->phase=NAV_PHASE_WAIT;return flow_wait(n,now,n->flow_rear_reason,out);}
 ++n->flow_backup_requests;n->flow_backup_counted=0;n->flow_backup_execution_us=0;n->flow_backup_target=fmin(.40,distance);n->flow_backup_yaw=p->yaw_rad;
 n->backup_start=(nav_xy){p->x_m,p->y_m};n->backup_progress=0;n->flow_blocked_since=0;
 phase_to(n,NAV_PHASE_BACKUP,now);return flow_wait(n,now,NAV_BACK_CHECK,out);
}
