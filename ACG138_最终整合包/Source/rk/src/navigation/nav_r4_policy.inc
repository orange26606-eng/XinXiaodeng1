/* S5原实现仍独立保留。仅已确认R4场景使用新动作门限。 */
static double r4_sonar_clear(const nav_engine*,unsigned,uint64_t);
static int r4_policy_active(const nav_engine*n){return n->c.passage.r4_enabled&&(n->r4_active||n->r4_checking);}
static double s5_speed_cap(const nav_engine*n){return r4_policy_active(n)?n->r4_output.speed_cap:s5_speed_cap_legacy(n);}
static int s5_candidate_policy(nav_engine*n,double v,double w){
 if(!r4_policy_active(n))return s5_candidate_policy_legacy(n,v,w);
 if(v<=0)return 1;
 if(!r4_direction_usable(&n->r4_input.direction[0],n->r4_input.now_us,CONT_SCAN_US)){
  s5_reject(n,v,w,"FRONT_DIRECTION_UNAVAILABLE");return 0;
 }
 double f=n->r4_input.direction[0].clearance_m;
 if(f<R6_FRONT_FORWARD_STOP_MM/1000.){s5_reject(n,v,w,"FRONT_UNDER_30_REVERSE_OR_TURN_ONLY");return 0;}
 if(!n->r4_checking){double cap=r4_action_speed_cap(&n->r4_state,&n->r4_input,n->r4_output.speed_cap,v,w);
  if(v>cap+.000001){s5_reject(n,v,w,cap<n->r4_output.speed_cap?"SIDE_ACTION_SPEED_ENVELOPE":"SCENE_SPEED_BAND");return 0;}
 }
 return 1;
}
static int s5_sonar_geometry(nav_engine*n,double v,double w,double horizon){
 if(!r4_policy_active(n))return s5_sonar_geometry_legacy(n,v,w,horizon);
 if(!n->flow_have_hint)return 1;
 if(v>0&&fabs(w)<=.1)horizon=fmin(horizon,nav_progressive_horizon(n,n->r4_input.now_us,v,w));
 const double pad=.01+n->c.passage.wheel_outer_margin_m,hx=n->c.passage.length_m*.5+pad,hy=n->c.passage.width_m*.5+pad;
 /* 声束几何只从真正超声来源造点。雷达扇区min是分类摘要，
  * 不能把斜侧雷达点重造到车头/车侧中心；雷达另走原始点完整扫掠。 */
 uint64_t now=n->last_tick>n->r4_input.now_us?n->last_tick:n->r4_input.now_us;
 double clear[3];for(unsigned j=0;j<3;j++)clear[j]=r4_sonar_clear(n,j,now);
 /* 仅允许从既存缓冲重叠单调移出，实际车体、新危险及独立雷达地图检查保留。
 */
 double physical_x=n->c.passage.length_m*.5+n->c.passage.wheel_outer_margin_m;
 double physical_y=n->c.passage.width_m*.5+n->c.passage.wheel_outer_margin_m;
 double previous[3]={0},initial[3]={0};unsigned buffer_escape[3]={0};
 for(unsigned side=0;side<3;side++)if(clear[side]>0){
  double ox=side?0:n->c.passage.length_m*.5+clear[0],oy=side?(side==1?1:-1)*(n->c.passage.width_m*.5+clear[side]):0;
  initial[side]=previous[side]=hypot(fmax(0,fabs(ox)-physical_x),fmax(0,fabs(oy)-physical_y));
  buffer_escape[side]=fabs(ox)<hx-1e-6&&fabs(oy)<hy-1e-6&&initial[side]>1e-6;
 }
 /* 单个侧方<15mm毛刺仅降速；第二笔确认由场景硬刹处理。
  * 雷达独立的真实车身碰撞终检仍然生效。 */
 if(fabs(w)<=.1)for(unsigned j=0;j<2;j++)if(n->r4_state.side_hits[j]==1&&clear[j+1]>0&&clear[j+1]<.015)clear[j+1]=.015;
 double x=0,y=0,a=0;unsigned steps=(unsigned)ceil(horizon/.02);if(!steps)steps=1;if(steps>256)return 0;
 for(unsigned k=0;k<=steps;k++){
  double cs=cos(a),sn=sin(a);
  for(unsigned side=0;side<3;side++)if(clear[side]>0){
   double ox=side?0:n->c.passage.length_m*.5+clear[0],oy=side?(side==1?1:-1)*(n->c.passage.width_m*.5+clear[side]):0;
   double dx=ox-x,dy=oy-y,bx=cs*dx+sn*dy,by=-sn*dx+cs*dy;
   double physical=hypot(fmax(0,fabs(bx)-physical_x),fmax(0,fabs(by)-physical_y));
   if(buffer_escape[side]){
    if(physical<=1e-6||physical+1e-6<previous[side]||(k==steps&&physical<=initial[side]+1e-6)){s5_reject(n,v,w,"INITIAL_BUFFER_NOT_CLEARING");return 0;}
    previous[side]=physical;
   }else if(fabs(bx)<hx-1e-6&&fabs(by)<hy-1e-6){s5_reject(n,v,w,side?"SIDE_BODY_SWEEP":"FRONT_BODY_SWEEP");return 0;}
  }
  if(k<steps)advance(&x,&y,&a,v,w,horizon/steps);
 }
 return 1;
}
