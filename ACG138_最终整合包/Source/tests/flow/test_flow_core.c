/* 不使用套接字或电机，直接测试实际生成的实时控制器。 */
#include "../../rk/src/app/lab_real_nav_generated.c"
#include "chassis_private.h"
#include <assert.h>
int w1_real_carrier_context(const lab_context*c){return c&&c->mode==W1_REAL_CARRIER&&c->timeline_id&&c->origin_epoch;}
int w1_real_carrier_event(const lab_event*e){return e&&e->meta.mode==W1_REAL_CARRIER;}
static void wire(void){
 ch_wire_status a={0},b={0};uint8_t bytes[96];a.lease=9;a.config_id=823144753;a.max_ttl_ms=200;a.max_rpm_centi=8000;a.revision=4;a.sonar_age_bucket[0]=1;a.sonar_age_bucket[1]=2;a.sonar_age_bucket[2]=15;a.sonar_raw_zero_mask=5;a.sonar_pending=3;a.sonar_valid=7;a.sonar_fresh=5;a.sonar_near=2;a.sonar_degraded=1;a.sonar_front_stable=1;a.local_stop_sources=128;a.sonar_mm[0]=220;a.sonar_mm[1]=100;a.sonar_mm[2]=4094;
 assert(ch_status_encode(bytes,&a)==CH_OK);assert(ch_status_decode(&b,bytes,96)==CH_OK);assert(b.revision==4&&b.sonar_age_bucket[0]==1&&b.sonar_age_bucket[1]==2&&b.sonar_age_bucket[2]==15&&b.sonar_raw_zero_mask==5&&b.sonar_pending==3&&b.max_rpm_centi==8000&&b.sonar_front_stable&&b.sonar_mm[0]==220&&b.sonar_mm[2]==4094&&b.local_stop_sources==128&&b.sonar_near==2);
 bytes[6]|=64;uint32_t crc=ch_crc32(bytes,92);for(unsigned i=0;i<4;i++)bytes[92+i]=(uint8_t)(crc>>(8*i));assert(ch_status_decode(&b,bytes,96)==CH_FORMAT);a.sonar_valid=0;a.sonar_fresh=0;assert(ch_status_encode(bytes,&a)==CH_OK&&ch_status_decode(&b,bytes,96)==CH_OK&&b.sonar_mm[0]==4095);a.revision=2;assert(ch_status_encode(bytes,&a)==CH_OK);bytes[85]=1;assert(ch_status_decode(&b,bytes,96)==CH_FORMAT);
 puts("PASS NCS4 96-byte sonar bit packing, stable flag, source latch, invalid sentinel, legacy reserved rejection");
}
static void motor(void){
 ch_config c;ch_config_default(&c);c.max_rpm=80;c.max_linear_m_s=.25;c.max_angular_rad_s=1.1;c.direct_targets=1;ch_output o;assert(ch_kinematics(&c,.25,1.1,&o)==CH_OK);for(unsigned i=0;i<4;i++)assert(fabs(o.logical_rpm[i])<=80.000001);assert(fabs(o.logical_rpm[1]-80)<1e-6);assert(ch_kinematics(&c,.25,0,&o)==CH_OK&&fabs(o.linear_m_s-.25)<1e-9);assert(ch_kinematics(&c,0,90*PI*.0675/(60*.29),&o)==CH_OK&&fabs(fabs(o.logical_rpm[0])-45)<1e-6);
 ch_guard*g=0;ch_context ctx={1,1,CH_LIVE};assert(ch_guard_open(&g,&c,&ctx)==CH_OK);assert(ch_guard_select(g,CH_NAVIGATION,7,1000000)==CH_OK);ch_health h={.context=ctx,.time_us=1000100,.link_time_us=1000100,.pose_time_us=1000100,.scan_time_us=1000100,.link_ok=1,.pose_valid=1,.scan_valid=1};assert(ch_guard_health(g,&h,1000100)==CH_OK);ch_lease lease;assert(ch_guard_arm(g,1000200,&lease)==CH_OK);ch_request q={.lease=lease,.time_us=1000300,.expires_us=1140000,.sequence=1,.linear_m_s=.25};assert(ch_guard_submit(g,&q,1000300)==CH_OK);assert(ch_guard_tick(g,1000400,&o)==CH_OK&&o.linear_m_s==.25);
 q.time_us=1010000;q.sequence=2;q.linear_m_s=-.20;assert(ch_guard_submit(g,&q,q.time_us)==CH_OK);assert(ch_guard_tick(g,1010100,&o)==CH_OK&&o.linear_m_s==0&&o.armed);assert(ch_guard_tick(g,1059000,&o)==CH_OK&&o.linear_m_s==0);assert(ch_guard_tick(g,1060100,&o)==CH_OK&&o.linear_m_s==-.20);
 q.time_us=1070000;q.sequence=3;q.linear_m_s=0;assert(ch_guard_submit(g,&q,q.time_us)==CH_OK);assert(ch_guard_tick(g,1070100,&o)==CH_OK&&o.armed&&o.expires_us==1140000&&o.linear_m_s==0);assert(ch_guard_tick(g,1140000,&o)==CH_WAIT&&o.linear_m_s==0);ch_guard_close(g);
 puts("PASS .25m/s direct targets, simultaneous 80RPM limit, +/-45 pivot, 50ms reversal, zero remains armed until TTL");
}
static int8_t cells[160*160];
static lab_event *ev(int type,uint64_t t,uint32_t seq){lab_event*e=calloc(1,sizeof(*e));assert(e);e->type=type;e->meta=(lab_meta){.mode=W1_REAL_CARRIER,.clock_domain=LAB_CLOCK_HOST_MONOTONIC,.timeline_id=1,.origin_epoch=1,.time_us=t,.received_us=t,.sequence=seq,.stream_session=1,.stream_id=type==LAB_EVENT_POSE?2:1,.provider=type==LAB_EVENT_POSE?LAB_PROVIDER_ODOMETRY:LAB_PROVIDER_USB,.flags=3};if(type==LAB_EVENT_POSE)e->data.pose.quality=LAB_POSE_VALID;return e;}
static nav_engine *engine(void){nav_config c;nav_config_default(&c);c.rectangular_footprint=1;c.enable_recovery=0;c.vehicle_width_m=.32;c.vehicle_length_m=.333;c.lidar_to_rear_m=.1665;c.passage.continuous=1;c.passage.margin_m=0;c.passage.normal_speed_m_s=.25;c.passage.narrow_speed_m_s=.12;c.safety_margin_m=0;c.max_speed_m_s=.25;c.max_yaw_rate_rad_s=1.1;c.require_scan=1;lab_context ctx={W1_REAL_CARRIER,LAB_CLOCK_HOST_MONOTONIC,1,1};nav_engine*n=0;assert(nav_open(&n,&c,&ctx)==LAB_OK);memset(cells,0,sizeof(cells));lab_map_view m={.context=ctx,.width=160,.height=160,.resolution_m=.025,.origin_x_m=-2,.origin_y_m=-2,.cells=cells};assert(nav_set_map(n,&m,1)==LAB_OK);nav_goal g={1,0,1,0,0};assert(nav_set_goal(n,&g,1000000)==LAB_OK);return n;}
/* 固定世界房间及突然加入的墙，位姿改变射线原点，不使用车体相对移动墙或合成自由环清除。
 */
static void control_world_scan(lab_event*s,int rear,double x,int obstacle){
 s->data.scan.count=0;s->data.scan.session=1;s->data.scan.sequence=s->meta.sequence;
 for(int deg=0;deg<360;deg++){
  if(!rear&&deg>115&&deg<245)continue;
  double a=deg*PI/180,dx=cos(a),dy=sin(a),d=100;
  if(dx>1e-9)d=fmin(d,(5-x)/dx);
  if(dx< -1e-9)d=fmin(d,(-5-x)/dx);
  if(dy>1e-9)d=fmin(d,5/dy);
  if(dy< -1e-9)d=fmin(d,-5/dy);
  if(obstacle&&dx>1e-9){double q=(.20-x)/dx;if(q>0&&fabs(q*dy)<.4)d=fmin(d,q);}
  struct scan_point*q=&s->data.scan.points[s->data.scan.count++];
  q->angle_q14=(uint16_t)(deg*65536/360);q->distance_q2=(uint32_t)(d*4000);q->quality=50;
 }
}
static void control(int rear){
 nav_engine*n=engine();uint16_t mm[3]={1000,1000,1000};double x=0;int backed=0,waited=0;nav_command out;uint64_t t=1020000;
 int8_t*world=calloc(320*320,1);assert(world);
 lab_map_view m={.context=n->context,.width=320,.height=320,.resolution_m=.025,.origin_x_m=-4,.origin_y_m=-4,.cells=world};assert(nav_set_map(n,&m,2)==LAB_OK);
 for(unsigned k=0;k<1500;k++,t+=20000){lab_event*p=ev(LAB_EVENT_POSE,t,k+1),*s=ev(LAB_EVENT_SCAN,t,k+1);p->data.pose.x_m=x;control_world_scan(s,rear,x,k>4);
  if(k>4)mm[0]=20;
  if(k==5){for(unsigned yy=0;yy<320;yy++)for(unsigned xx=0;xx<320;xx++){double wx=-4+(xx+.5)*.025,wy=-4+(yy+.5)*.025;if(wx>=.20&&wx<.225&&fabs(wy)<.4)world[yy*320+xx]=100;}assert(nav_set_map(n,&m,3)==LAB_OK);}
  nav_set_chassis_hint(n,t,mm,7,7,k>4?1:0,0,k<=4);nav_set_execution_hint(n,1);lab_status r=nav_step(n,t,p,s,p,&out);assert(r==LAB_OK||r==LAB_AGAIN);if(k==0)assert(out.linear_m_s==.25);
  if(out.linear_m_s<0){backed++;assert(rear);assert(out.linear_m_s>=-.200000001);x+=out.linear_m_s*.02;assert(x>=-2.601);}
  if(out.linear_m_s==0&&out.angular_rad_s==0)waited++;free(p);free(s);
 }
 assert(waited);if(rear)assert(backed&&n->reverse_attempts==5&&n->flow_extended_attempts==1&&n->backup_progress<=.60);else assert(!backed&&!n->reverse_attempts);
 nav_cancel(n,t);assert(!n->have_goal&&n->cmd.linear_m_s==0&&n->cmd.angular_rad_s==0);nav_close(n);free(world);
 printf("PASS actual live navigation fixed-world front blocked: rear=%d backing_ticks=%d zero_waits=%d x=%.3f, 5+1 bounded recovery and cancel\n",rear,backed,waited,x);
}
static void local_votes(void){passage_config c;passage_config_default(&c);passage_local*l=calloc(1,sizeof(*l));assert(l);passage_pose p={0};assert(passage_local_begin(l,&c,p,1,1000000));passage_local_ray(l,&c,p,0,1,50,1,1000000);unsigned at=0;for(unsigned i=0;i<PASSAGE_LOCAL_CELLS;i++)if(l->score[i]>0)at=i;assert(l->hit_count[at]==1);assert(passage_local_begin(l,&c,p,3,1200000));passage_local_ray(l,&c,p,0,1,50,3,1200000);assert(l->hit_count[at]==2);free(l);puts("PASS ordinary local point confirmed by 2 of latest 3 scans, isolated first return remains unconfirmed");}

static void collision_layers(void){nav_engine*n=engine();n->c.require_scan=0; /* 显式纯地图几何测试，下方实时测试另提供实际扫描。 */uint16_t mm[3]={20,500,500};nav_set_chassis_hint(n,1000000,mm,6,6,1,1,0);assert(flow_sonar_allows(n,1000000,-.2,0));nav_set_chassis_hint(n,1000000,mm,4,4,1,1,0);assert(!flow_sonar_allows(n,1000000,-.2,0));lab_pose p={0};n->last_tick=n->scan_time=n->motion_time=1000000;for(unsigned yy=0;yy<160;yy++)for(unsigned xx=0;xx<160;xx++){double x=-2+(xx+.5)*.025,y=-2+(yy+.5)*.025;if(x>=.30&&x<.325&&fabs(y)<.4)n->cells[yy*160+xx]=100;}assert(sweep_reason(n,&p,.25,0,NULL,0)==NAV_REASON_NONE&&n->flow_soft);for(unsigned yy=0;yy<160;yy++)for(unsigned xx=0;xx<160;xx++){double x=-2+(xx+.5)*.025,y=-2+(yy+.5)*.025;if(x>=.20&&x<.225&&fabs(y)<.4)n->cells[yy*160+xx]=100;}assert(sweep_reason(n,&p,.25,0,NULL,0)==NAV_OBSTACLE);nav_close(n);puts("PASS far confirmed collision is soft cost; imminent swept rectangle collision rejects action");}

static void rtl_vectors(const char*path){FILE*f=fopen(path,"r");assert(f);char line[4096];unsigned count=0;while(fgets(line,sizeof(line),f)){size_t n=strcspn(line,"\r\n");if(!n)continue;assert(n==300);uint8_t bytes[150];for(unsigned i=0;i<150;i++){char h[3]={line[2*i],line[2*i+1],0};char*end=0;unsigned long v=strtoul(h,&end,16);assert(end==h+2&&v<=255);bytes[i]=(uint8_t)v;}ch_wire_status s;assert(ch_status_decode(&s,bytes+50,96)==CH_OK);assert(s.revision==4&&s.config_id==823144753&&s.max_rpm_centi==8000&&s.sonar_mm[0]==150&&s.sonar_mm[1]==500&&s.sonar_mm[2]==4094&&s.sonar_valid==7&&s.sonar_fresh==5&&s.sonar_near==2&&s.sonar_degraded&&s.sonar_front_stable&&s.local_stop_sources==128);count++;}assert(!fclose(f)&&count);printf("PASS RK decoded %u actual RTL GMII NCS4 frames incl bit39 and CRC\n",count);}
int main(int argc,char**argv){assert(argc==2);rtl_vectors(argv[1]);wire();motor();local_votes();collision_layers();control(0);control(1);puts("PASS FLOW CORE no hardware IO");return 0;}
