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
static int s5_sonar_geometry(nav_engine*n,double v,double w,double horizon){(void)horizon;return s5_current_sonar(n,v,w);}
