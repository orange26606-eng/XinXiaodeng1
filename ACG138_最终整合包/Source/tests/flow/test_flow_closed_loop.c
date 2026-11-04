/* 估计传感器经真实适配器、连续矩形导航器及轮速保护，驱动同一模拟车体，不使用真实硬件传输。
 */
#include "s2_navigation_limits.h"
#define CONTINUOUS_INCLUDED 1
#include "../../rk/tests/real_integration/test_mapping_shadow.c"
static void run(unsigned turn){
 closed_fixture f;closed_open(&f,9501+turn,0);sim_cl_close(f.plant);sim_cl_config plant_config;sim_cl_config_default(&plant_config);plant_config.sensors.mode=LAB_SIM_NOISY;plant_config.sensors.timeline_id=9501+turn;plant_config.sensors.duration_us=20000000;plant_config.integration_step_us=20000;plant_config.max_linear_m_s=.30;plant_config.max_angular_rad_s=2.0;CHECK(sim_cl_open(&f.plant,&plant_config)==LAB_OK);lab_real_options o;lab_real_options_default(&o);
 o.mapping_shadow=1;o.mode=LAB_REAL_SHADOW;o.has_goal=3;o.goal.x_m=turn?0:.6;o.goal.y_m=turn?.6:0;
 o.passage.continuous=1;o.passage.margin_m=0;o.passage.normal_speed_m_s=.25;o.passage.narrow_speed_m_s=.12;
 lab_live_source_binding source={&f,&closed_ops};lab_real_session s;CHECK(lab_real_session_open(&s,&o,&f.base.ctx,&source)==LAB_OK);
 uint32_t sequence=0;unsigned moved=0;int reached=0;double peak_v=0,peak_w=0,peak_rpm=0;
 for(uint64_t t=0;t<18000000;t+=20000){
  if(t)CHECK(sim_cl_step(f.plant,t)==LAB_OK);lab_status r=lab_real_session_tick(&s,BASE+t);if(r!=LAB_OK&&r!=LAB_AGAIN)fprintf(stderr,"S5_CLOSED_FAILURE turn=%u t=%llu status=%d reason=%s\n",turn,(unsigned long long)t,(int)r,s.view.reason);CHECK(r==LAB_OK||r==LAB_AGAIN);
  CHECK(!s.view.hardware_calls&&!s.view.enabled);double v=0,w=0;
  if(r==LAB_OK&&s.view.protected_output.command.expires_us>BASE+t){const double*rpm=s.view.protected_output.wheel_rpm;
   double l=(rpm[0]+rpm[2])*.5*3.141592653589793*.0675/60,rw=(rpm[1]+rpm[3])*.5*3.141592653589793*.0675/60;v=(l+rw)*.5;w=(rw-l)/.290;
  }
  peak_v=fmax(peak_v,fabs(v));peak_w=fmax(peak_w,fabs(w));for(unsigned i=0;i<4;i++){peak_rpm=fmax(peak_rpm,fabs(s.view.protected_output.wheel_rpm[i]));CHECK(fabs(s.view.protected_output.wheel_rpm[i])<=80.00001);}if(fabs(v)>.001)moved++;
  sim_cl_command command={.context={LAB_SIM_NOISY,LAB_CLOCK_SIMULATION,9501+turn,1},.source_id=1,.sequence=++sequence,.generated_us=t,.valid_until_us=t+150000,.linear_m_s=v,.angular_rad_s=w};
  CHECK(sim_cl_submit(f.plant,&command)==LAB_OK);
  if(!strcmp(s.view.reason,"GOAL_REACHED")){reached=1;break;}
 }
 sim_cl_stats stats;sim_cl_get_stats(f.plant,&stats);CHECK(reached&&moved>10&&stats.collisions==0);/* 此估计传感样例无超声波源，S5未知限速为0.12；已确认净空0.25的测试见tests/s5_navigation。
 */CHECK(peak_v>=.119&&peak_v<=.120001&&peak_w<=S2_PIVOT_YAW+.000001);if(turn)CHECK(peak_w>S2_PIVOT_YAW-.001&&peak_w<=S2_PIVOT_YAW+.000001);
 printf("PASS FLOW COMMON PLANT turn=%u peak_v=%.3f peak_w=%.3f peak_rpm=%.3f unknown_sonar_cap=.12 configured_open_cap=.25 margin=0 moved=%u reached=1 collisions=0 hardware_calls=0\n",turn,peak_v,peak_w,peak_rpm,moved);
 lab_real_session_close(&s);closed_free(&f);
}

int main(void){run(0);run(1);return 0;}
