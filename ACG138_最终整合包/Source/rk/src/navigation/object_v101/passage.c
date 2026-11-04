#include "passage.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <ctype.h>
#include <errno.h>
#define PI 3.14159265358979323846
typedef struct poly {passage_xy p[16];unsigned n;} poly;
static double cross(passage_xy a,passage_xy b,passage_xy c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
static double wrap(double a){return remainder(a,2*PI);}
void passage_config_default(passage_config*c){if(!c)return;memset(c,0,sizeof(*c));c->length_m=.333;c->width_m=.320;c->margin_m=0;c->normal_speed_m_s=.40;c->narrow_speed_m_s=.16;c->narrow_width_m=.38;c->local_resolution_m=.025;c->local_ttl_us=2000000;c->lidar_min_m=.05;c->lidar_max_m=12;c->sonar_beam_margin_m=.02;c->front_approach_m=1.;c->front_corridor_m=1.2;c->planner_cycle_ms=100;c->planner_horizon_s=1.5;c->planner_dt_ms=50;c->costmap_lethal_m=.18;c->costmap_inflation_m=.25;c->planner_w_heading=.5;c->planner_w_path=.3;c->planner_w_clearance=.4;c->planner_w_smoothness=.3;c->planner_w_velocity=.15;c->a_lin_m_s2=.4;c->a_yaw_rad_s2=2.5;c->r4_enabled=1;c->side_straight_emergency_mm=15;c->side_straight_crawl_mm=15;c->side_straight_observe_mm=20;c->w_thresh_straight_rad_s=0.1;c->band_up_hyst=1.1;c->band_up_hold_ms=300;c->scene_confirm_ms=500;c->scene_exit_ms=800;c->wall_band_min_mm=15;c->wall_band_max_mm=120;c->corridor_max_mm=300;c->doorway_len_max_m=0.5;c->doorway_time_max_ms=1500;c->explore_turn_step_deg=30;c->explore_side_budget_deg=90;c->explore_fwd_min_mm=100;c->explore_probe_fast_m_s=0.2;c->explore_probe_crawl_m_s=0.18;c->explore_progress_mm=50;c->explore_progress_s=10;c->explore_rounds=6;c->reverse_attempts=20;c->extended_reverse_attempts=1;c->calibrated=0;c->front_lethal_from_edge_m=0;}
int passage_config_valid(const passage_config*c){
 if(!c)return 0;const double a[]={c->length_m,c->width_m,c->margin_m,c->normal_speed_m_s,c->narrow_speed_m_s,c->narrow_width_m,c->local_resolution_m,c->mount_x_m,c->mount_y_m,c->mount_yaw_rad,c->lidar_min_m,c->lidar_max_m,c->sonar_front_offset_m,c->sonar_side_offset_m,c->sonar_beam_margin_m,c->wheel_outer_margin_m,c->front_approach_m,c->front_corridor_m,c->planner_cycle_ms,c->planner_horizon_s,c->planner_dt_ms,c->costmap_lethal_m,c->costmap_inflation_m,c->planner_w_heading,c->planner_w_path,c->planner_w_clearance,c->planner_w_smoothness,c->planner_w_velocity,c->a_lin_m_s2,c->a_yaw_rad_s2,c->r4_enabled,c->side_straight_emergency_mm,c->side_straight_crawl_mm,c->side_straight_observe_mm,c->w_thresh_straight_rad_s,c->band_up_hyst,c->band_up_hold_ms,c->scene_confirm_ms,c->scene_exit_ms,c->wall_band_min_mm,c->wall_band_max_mm,c->corridor_max_mm,c->doorway_len_max_m,c->doorway_time_max_ms,c->explore_turn_step_deg,c->explore_side_budget_deg,c->explore_fwd_min_mm,c->explore_probe_fast_m_s,c->explore_probe_crawl_m_s,c->explore_progress_mm,c->explore_progress_s,c->explore_rounds,c->reverse_attempts,c->extended_reverse_attempts,c->calibrated,c->front_lethal_from_edge_m};
 for(unsigned i=0;i<sizeof(a)/sizeof(a[0]);i++)if(!isfinite(a[i]))return 0;
 if(c->r4_enabled!=0&&c->r4_enabled!=1)return 0;
 if(c->calibrated!=0&&c->calibrated!=1)return 0;
 if(c->side_straight_emergency_mm<15||c->side_straight_crawl_mm<c->side_straight_emergency_mm||c->side_straight_observe_mm<c->side_straight_crawl_mm||c->side_straight_observe_mm>120)return 0;
 if(c->w_thresh_straight_rad_s<=0||c->w_thresh_straight_rad_s>.1||c->band_up_hyst<1||c->band_up_hyst>2||c->band_up_hold_ms<0||c->band_up_hold_ms>5000||c->scene_confirm_ms<0||c->scene_confirm_ms>5000||c->scene_exit_ms<0||c->scene_exit_ms>5000)return 0;
 if(c->wall_band_min_mm<15||c->wall_band_max_mm<c->wall_band_min_mm||c->corridor_max_mm<c->wall_band_max_mm||c->corridor_max_mm>1000||c->doorway_len_max_m<=0||c->doorway_len_max_m>2||c->doorway_time_max_ms<=0||c->doorway_time_max_ms>5000)return 0;
 if(c->explore_turn_step_deg<=0||c->explore_turn_step_deg>45||c->explore_side_budget_deg<c->explore_turn_step_deg||c->explore_side_budget_deg>90||c->explore_fwd_min_mm<100||c->explore_fwd_min_mm>150||c->explore_probe_fast_m_s<=0||c->explore_probe_fast_m_s>.20||c->explore_probe_crawl_m_s<=0||c->explore_probe_crawl_m_s>.18||c->explore_progress_mm<=0||c->explore_progress_s<=0)return 0;
 if(c->explore_rounds<1||c->explore_rounds>20||floor(c->explore_rounds)!=c->explore_rounds||c->reverse_attempts<1||c->reverse_attempts>50||floor(c->reverse_attempts)!=c->reverse_attempts||c->extended_reverse_attempts<0||c->extended_reverse_attempts>5||floor(c->extended_reverse_attempts)!=c->extended_reverse_attempts||c->front_lethal_from_edge_m<0||c->front_lethal_from_edge_m>.12)return 0;
 return c->length_m>=.1&&c->length_m<=2&&c->width_m>=.1&&c->width_m<=2&&c->margin_m>=0&&c->margin_m<=.10&&c->normal_speed_m_s>0&&c->normal_speed_m_s<=.40&&c->narrow_speed_m_s>0&&c->narrow_speed_m_s<=c->normal_speed_m_s&&c->narrow_width_m>=c->width_m+2*c->margin_m&&c->narrow_width_m<=2&&c->local_resolution_m>=.01&&c->local_resolution_m<=.05&&c->local_ttl_us>=100000&&c->local_ttl_us<=2000000&&fabs(c->mount_x_m)<=1&&fabs(c->mount_yaw_rad)<=PI&&fabs(c->mount_y_m)<=1&&c->mount_verified<=1&&c->lidar_min_m>=.03&&c->lidar_min_m<=.5&&c->lidar_max_m>c->lidar_min_m&&c->lidar_max_m<=30&&fabs(c->sonar_front_offset_m)<=.5&&fabs(c->sonar_side_offset_m)<=.5&&c->sonar_beam_margin_m>=0&&c->sonar_beam_margin_m<=.10&&c->wheel_outer_margin_m>=0&&c->wheel_outer_margin_m<=.20&&c->front_approach_m>=.6&&c->front_approach_m<=2&&c->front_corridor_m>=c->front_approach_m&&c->front_corridor_m<=3&&c->sonar_offsets_verified<=1&&c->wheel_outer_margin_verified<=1&&c->planner_cycle_ms>=50&&c->planner_cycle_ms<=200&&c->planner_dt_ms>=10&&c->planner_dt_ms<=50&&c->planner_horizon_s>=.1&&c->planner_horizon_s<=1.5&&c->planner_cycle_ms<=c->planner_horizon_s*1000&&ceil(c->planner_horizon_s*1000/c->planner_dt_ms-1e-9)<=30&&c->costmap_lethal_m>0&&c->costmap_inflation_m>c->costmap_lethal_m&&c->costmap_inflation_m<=.5&&c->a_lin_m_s2>0&&c->a_lin_m_s2<=5&&c->a_yaw_rad_s2>0&&c->a_yaw_rad_s2<=10&&c->planner_w_heading>=0&&c->planner_w_heading<=10&&c->planner_w_path>=0&&c->planner_w_path<=10&&c->planner_w_clearance>=0&&c->planner_w_clearance<=10&&c->planner_w_smoothness>=0&&c->planner_w_smoothness<=10&&c->planner_w_velocity>=0&&c->planner_w_velocity<=10&&(c->planner_w_heading+c->planner_w_path+c->planner_w_clearance+c->planner_w_smoothness+c->planner_w_velocity)>0;
}
int passage_config_load(passage_config*c,const char*path,char*error,size_t cap){
 if(!c)return 0;passage_config q;passage_config_default(&q);if(!path||!*path){*c=q;return 1;}FILE*f=fopen(path,"r");if(!f){if(error&&cap)snprintf(error,cap,"PASSAGE_CONFIG_OPEN_FAILED");return 0;}
 struct field{const char*name;size_t offset;}fields[]={
  {"vehicle_length_m",offsetof(passage_config,length_m)},{"vehicle_width_m",offsetof(passage_config,width_m)},{"side_margin_m",offsetof(passage_config,margin_m)},
  {"normal_speed_m_s",offsetof(passage_config,normal_speed_m_s)},{"narrow_speed_m_s",offsetof(passage_config,narrow_speed_m_s)},{"narrow_corridor_width_m",offsetof(passage_config,narrow_width_m)},
  {"local_resolution_m",offsetof(passage_config,local_resolution_m)},{"lidar_x_m",offsetof(passage_config,mount_x_m)},{"lidar_y_m",offsetof(passage_config,mount_y_m)},
  {"lidar_yaw_rad",offsetof(passage_config,mount_yaw_rad)},{"lidar_min_m",offsetof(passage_config,lidar_min_m)},{"lidar_max_m",offsetof(passage_config,lidar_max_m)},
  {"sonar_front_offset_m",offsetof(passage_config,sonar_front_offset_m)},{"sonar_side_offset_m",offsetof(passage_config,sonar_side_offset_m)},{"sonar_beam_margin_m",offsetof(passage_config,sonar_beam_margin_m)},{"wheel_outer_margin_m",offsetof(passage_config,wheel_outer_margin_m)},{"front_approach_m",offsetof(passage_config,front_approach_m)},{"front_corridor_m",offsetof(passage_config,front_corridor_m)},{"planner_cycle_ms",offsetof(passage_config,planner_cycle_ms)},{"planner_horizon_s",offsetof(passage_config,planner_horizon_s)},{"planner_dt_ms",offsetof(passage_config,planner_dt_ms)},{"costmap_lethal_m",offsetof(passage_config,costmap_lethal_m)},{"costmap_inflation_m",offsetof(passage_config,costmap_inflation_m)},{"planner_w_heading",offsetof(passage_config,planner_w_heading)},{"planner_w_path",offsetof(passage_config,planner_w_path)},{"planner_w_clearance",offsetof(passage_config,planner_w_clearance)},{"planner_w_smoothness",offsetof(passage_config,planner_w_smoothness)},{"planner_w_velocity",offsetof(passage_config,planner_w_velocity)},{"a_lin_m_s2",offsetof(passage_config,a_lin_m_s2)},{"a_yaw_rad_s2",offsetof(passage_config,a_yaw_rad_s2)},{"r4_enabled",offsetof(passage_config,r4_enabled)},{"side_straight_emergency_mm",offsetof(passage_config,side_straight_emergency_mm)},{"side_straight_crawl_mm",offsetof(passage_config,side_straight_crawl_mm)},{"side_straight_observe_mm",offsetof(passage_config,side_straight_observe_mm)},{"w_thresh_straight_rad_s",offsetof(passage_config,w_thresh_straight_rad_s)},{"band_up_hyst",offsetof(passage_config,band_up_hyst)},{"band_up_hold_ms",offsetof(passage_config,band_up_hold_ms)},{"scene_confirm_ms",offsetof(passage_config,scene_confirm_ms)},{"scene_exit_ms",offsetof(passage_config,scene_exit_ms)},{"wall_band_min_mm",offsetof(passage_config,wall_band_min_mm)},{"wall_band_max_mm",offsetof(passage_config,wall_band_max_mm)},{"corridor_max_mm",offsetof(passage_config,corridor_max_mm)},{"doorway_len_max_m",offsetof(passage_config,doorway_len_max_m)},{"doorway_time_max_ms",offsetof(passage_config,doorway_time_max_ms)},{"explore_turn_step_deg",offsetof(passage_config,explore_turn_step_deg)},{"explore_side_budget_deg",offsetof(passage_config,explore_side_budget_deg)},{"explore_fwd_min_mm",offsetof(passage_config,explore_fwd_min_mm)},{"explore_probe_fast_m_s",offsetof(passage_config,explore_probe_fast_m_s)},{"explore_probe_crawl_m_s",offsetof(passage_config,explore_probe_crawl_m_s)},{"explore_progress_mm",offsetof(passage_config,explore_progress_mm)},{"explore_progress_s",offsetof(passage_config,explore_progress_s)},{"explore_rounds",offsetof(passage_config,explore_rounds)},{"reverse_attempts",offsetof(passage_config,reverse_attempts)},{"extended_reverse_attempts",offsetof(passage_config,extended_reverse_attempts)},{"calibrated",offsetof(passage_config,calibrated)},{"front_lethal_from_edge_m",offsetof(passage_config,front_lethal_from_edge_m)}};
 unsigned char seen[128]={0};char line[256];int ok=1;unsigned number=0;
 while(fgets(line,sizeof(line),f)){number++;if(!strchr(line,'\n')&&!feof(f)){ok=0;break;}char*hash=strchr(line,'#');if(hash)*hash=0;char*p=line;while(isspace((unsigned char)*p))p++;if(!*p)continue;char*eq=strchr(p,'=');if(!eq){ok=0;break;}*eq++=0;char*tail=p+strlen(p);while(tail>p&&isspace((unsigned char)tail[-1]))*--tail=0;while(isspace((unsigned char)*eq))eq++;
  errno=0;char*end;double value=strtod(eq,&end);if(errno||end==eq||!isfinite(value)){ok=0;break;}while(isspace((unsigned char)*end))end++;if(*end){ok=0;break;}
  unsigned i;for(i=0;i<sizeof(fields)/sizeof(fields[0]);i++)if(!strcmp(p,fields[i].name))break;
  unsigned bit=i;if(i<sizeof(fields)/sizeof(fields[0]))*(double*)((unsigned char*)&q+fields[i].offset)=value;
  else if(!strcmp(p,"lidar_offset_verified")){bit=120;if(value!=0&&value!=1){ok=0;break;}q.mount_verified=(uint32_t)value;}
  else if(!strcmp(p,"sonar_offsets_verified")){bit=122;if(value!=0&&value!=1){ok=0;break;}q.sonar_offsets_verified=(uint32_t)value;}
  else if(!strcmp(p,"wheel_outer_margin_verified")){bit=123;if(value!=0&&value!=1){ok=0;break;}q.wheel_outer_margin_verified=(uint32_t)value;}
  else if(!strcmp(p,"local_ttl_ms")){bit=121;if(value<100||value>2000||value!=floor(value)){ok=0;break;}q.local_ttl_us=(uint64_t)value*1000;}
  else{ok=0;break;}
  if(bit>=sizeof(seen)||seen[bit]){ok=0;break;}seen[bit]=1;
 }
 if(ferror(f))ok=0;fclose(f);if(!ok||!passage_config_valid(&q)){if(error&&cap)snprintf(error,cap,"PASSAGE_CONFIG_INVALID_LINE_%u",number);return 0;}*c=q;return 1;
}
void passage_local_reset(passage_local*l){if(l)memset(l,0,sizeof(*l));}
/* 按遍历顺序移动行以准确处理重叠复制，不新建第二个1.7MB网格，也不插值证据，新格没有观测。
 */
static void local_roll(passage_local*l,int dx,int dy){
 int y0=dy<0?(int)PASSAGE_LOCAL_SIDE-1:0,yn=dy<0?-1:(int)PASSAGE_LOCAL_SIDE,ys=dy<0?-1:1;
 int x0=dx<0?(int)PASSAGE_LOCAL_SIDE-1:0,xn=dx<0?-1:(int)PASSAGE_LOCAL_SIDE,xs=dx<0?-1:1;
 for(int y=y0;y!=yn;y+=ys)for(int x=x0;x!=xn;x+=xs){unsigned k=(unsigned)y*PASSAGE_LOCAL_SIDE+(unsigned)x;int sx=x+dx,sy=y+dy;
  if(sx>=0&&sy>=0&&sx<(int)PASSAGE_LOCAL_SIDE&&sy<(int)PASSAGE_LOCAL_SIDE){unsigned j=(unsigned)sy*PASSAGE_LOCAL_SIDE+(unsigned)sx;
   l->score[k]=l->score[j];l->stamp[k]=l->stamp[j];l->sequence[k]=l->sequence[j];l->hit_count[k]=l->hit_count[j];l->hit_sequence[k]=l->hit_sequence[j];l->hit_stamp[k]=l->hit_stamp[j];l->hit_history[k]=l->hit_history[j];l->clear_stamp[k]=l->clear_stamp[j];l->hit_point_x[k]=l->hit_point_x[j];l->hit_point_y[k]=l->hit_point_y[j];
  }else{l->score[k]=0;l->stamp[k]=0;l->sequence[k]=0;l->hit_count[k]=0;l->hit_sequence[k]=0;l->hit_stamp[k]=0;l->hit_history[k]=0;l->clear_stamp[k]=0;l->hit_point_x[k]=l->hit_point_y[k]=0;++l->roll_cleared;}
 }
 ++l->rolls;l->origin_x+=dx*l->resolution;l->origin_y+=dy*l->resolution;
}
int passage_local_reframe(passage_local*l,const passage_config*c,double dx,double dy,double yaw){
 if(!l||!passage_config_valid(c)||!isfinite(dx)||!isfinite(dy)||!isfinite(yaw))return 0;
 if(fabs(dx)<1e-12&&fabs(dy)<1e-12&&fabs(yaw)<1e-12)return 1;
 double cs=cos(yaw),sn=sin(yaw),x=l->map_from_local_x,y=l->map_from_local_y;
 l->map_from_local_x=dx+cs*x-sn*y;l->map_from_local_y=dy+sn*x+cs*y;l->map_from_local_yaw=wrap(yaw+l->map_from_local_yaw);++l->reframes;return 1;
}
static passage_xy local_xy(const passage_local*l,passage_xy p){double cs=cos(l->map_from_local_yaw),sn=sin(l->map_from_local_yaw),x=p.x-l->map_from_local_x,y=p.y-l->map_from_local_y;return (passage_xy){cs*x+sn*y,-sn*x+cs*y};}
static passage_xy map_xy(const passage_local*l,passage_xy p){double cs=cos(l->map_from_local_yaw),sn=sin(l->map_from_local_yaw);return (passage_xy){l->map_from_local_x+cs*p.x-sn*p.y,l->map_from_local_y+sn*p.x+cs*p.y};}
static passage_pose local_pose(const passage_local*l,passage_pose p){passage_xy q=local_xy(l,(passage_xy){p.x,p.y});return (passage_pose){q.x,q.y,wrap(p.yaw-l->map_from_local_yaw)};}
int passage_local_begin(passage_local*l,const passage_config*c,passage_pose p,uint32_t seq,uint64_t t){
 if(!l||!passage_config_valid(c)||!t||!isfinite(p.x)||!isfinite(p.y))return 0;
 if(l->initialized&&(t<=l->last_scan||(int32_t)(seq-l->scan_sequence)<=0))return 0;
 p=local_pose(l,p);double span=PASSAGE_LOCAL_SIDE*c->local_resolution_m;
 if(!l->initialized||l->resolution!=c->local_resolution_m){
  uint64_t rolls=l->rolls,cleared=l->roll_cleared,reframes=l->reframes,resets=l->reframe_resets;
  double tx=l->map_from_local_x,ty=l->map_from_local_y,ta=l->map_from_local_yaw;
  passage_local_reset(l);l->map_from_local_x=tx;l->map_from_local_y=ty;l->map_from_local_yaw=ta;l->rolls=rolls;l->roll_cleared=cleared;l->reframes=reframes;l->reframe_resets=resets;l->resolution=c->local_resolution_m;l->origin_x=floor((p.x-span*.5)/l->resolution)*l->resolution;l->origin_y=floor((p.y-span*.5)/l->resolution)*l->resolution;l->initialized=1;
 }else{
  double ox=floor((p.x-span*.5)/l->resolution)*l->resolution,oy=floor((p.y-span*.5)/l->resolution)*l->resolution;
  int dx=(int)llround((ox-l->origin_x)/l->resolution),dy=(int)llround((oy-l->origin_y)/l->resolution);
  if(dx||dy)local_roll(l,dx,dy);
 }
 for(unsigned k=0;k<PASSAGE_LOCAL_CELLS;k++)if(l->stamp[k]&&t>=l->stamp[k]&&t-l->stamp[k]>c->local_ttl_us){
  l->score[k]=0;l->stamp[k]=0;l->sequence[k]=0;l->hit_count[k]=0;l->hit_sequence[k]=0;l->hit_stamp[k]=0;l->hit_history[k]=0;l->clear_stamp[k]=0;l->hit_point_x[k]=l->hit_point_y[k]=0;
 }
 l->last_scan=t;l->scan_sequence=seq;l->beam_entries=l->beam_prev_valid=l->beam_first_valid=l->scan_finished=l->scan_wedge_rays=0;return 1;
}
static void local_update(passage_local*l,const passage_config*c,int x,int y,int hit,uint32_t seq,uint64_t t,double px,double py){
 if(x<0||y<0||x>=(int)PASSAGE_LOCAL_SIDE||y>=(int)PASSAGE_LOCAL_SIDE)return;unsigned k=(unsigned)y*PASSAGE_LOCAL_SIDE+(unsigned)x;
 if(hit){
 if(l->hit_sequence[k]!=seq){
  unsigned confirmed=0,cluster=0;uint32_t gap=seq-l->hit_sequence[k];
  int recent=l->hit_stamp[k]&&t>=l->hit_stamp[k]&&t-l->hit_stamp[k]<=300000;
  l->hit_history[k]=(uint8_t)((recent&&l->hit_sequence[k]&&gap<3?((l->hit_history[k]&7u)<<gap):0)|1u|128u);
  unsigned history=l->hit_history[k]&7u;if((history&(history-1u))!=0)confirmed=1;
  for(int yy=y-1;yy<=y+1;yy++)for(int xx=x-1;xx<=x+1;xx++)if(xx>=0&&yy>=0&&xx<(int)PASSAGE_LOCAL_SIDE&&yy<(int)PASSAGE_LOCAL_SIDE){unsigned j=(unsigned)yy*PASSAGE_LOCAL_SIDE+(unsigned)xx;
   if(j!=k&&l->hit_sequence[j]==seq)++cluster;
   uint32_t age=seq-l->hit_sequence[j];if(l->hit_sequence[j]&&age>0&&age<=2&&t>=l->hit_stamp[j]&&t-l->hit_stamp[j]<=300000&&l->hit_count[j])confirmed=1;
  }
  if(cluster>=2){confirmed=1;for(int yy=y-1;yy<=y+1;yy++)for(int xx=x-1;xx<=x+1;xx++)if(xx>=0&&yy>=0&&xx<(int)PASSAGE_LOCAL_SIDE&&yy<(int)PASSAGE_LOCAL_SIDE){unsigned j=(unsigned)yy*PASSAGE_LOCAL_SIDE+(unsigned)xx;if(l->hit_sequence[j]==seq)l->hit_count[j]=2;}}
  l->hit_count[k]=confirmed?2:1;l->hit_sequence[k]=seq;l->hit_stamp[k]=t;
 }
 l->score[k]=1;l->stamp[k]=t;l->sequence[k]=seq;l->hit_point_x[k]=(float)px;l->hit_point_y[k]=(float)py;return;}
 if(l->stamp[k]&&l->sequence[k]==seq)return;
 if(!l->stamp[k]||t<l->stamp[k]||t-l->stamp[k]>c->local_ttl_us)l->score[k]=0;
 if(l->score[k]>-2)--l->score[k];if(l->score[k]<0){l->hit_count[k]=0;l->hit_sequence[k]=0;l->hit_history[k]=0;l->clear_stamp[k]=t;}l->stamp[k]=t;l->sequence[k]=seq;
}
static int local_line(passage_local*l,const passage_config*c,double sx,double sy,double ex,double ey,int endpoint_hit,uint32_t seq,uint64_t t){
 double res=l->resolution;
 int x=(int)floor((sx-l->origin_x)/res),y=(int)floor((sy-l->origin_y)/res);if(x<0||y<0||x>=(int)PASSAGE_LOCAL_SIDE||y>=(int)PASSAGE_LOCAL_SIDE){++l->invalid;return 0;}
 int tx=(int)floor((ex-l->origin_x)/res),ty=(int)floor((ey-l->origin_y)/res),inside=tx>=0&&ty>=0&&tx<(int)PASSAGE_LOCAL_SIDE&&ty<(int)PASSAGE_LOCAL_SIDE,hit=endpoint_hit&&inside;
 if(!inside){double u=1,dx=ex-sx,dy=ey-sy,hi_x=l->origin_x+PASSAGE_LOCAL_SIDE*res-1e-9,hi_y=l->origin_y+PASSAGE_LOCAL_SIDE*res-1e-9;
  if(dx>0)u=fmin(u,(hi_x-sx)/dx);else if(dx<0)u=fmin(u,(l->origin_x+1e-9-sx)/dx);if(dy>0)u=fmin(u,(hi_y-sy)/dy);else if(dy<0)u=fmin(u,(l->origin_y+1e-9-sy)/dy);
  tx=(int)floor((sx+u*dx-l->origin_x)/res);ty=(int)floor((sy+u*dy-l->origin_y)/res);
 }
 int dx=abs(tx-x),dy=-abs(ty-y),stepx=x<tx?1:-1,stepy=y<ty?1:-1,error=dx+dy;
 for(unsigned budget=0;budget<PASSAGE_LOCAL_SIDE*3;budget++){
  if(x==tx&&y==ty){local_update(l,c,x,y,hit,seq,t,ex,ey);break;}local_update(l,c,x,y,0,seq,t,0,0);int e=2*error;if(e>=dy){error+=dy;x+=stepx;}if(e<=dx){error+=dx;y+=stepy;}
 }
 return hit;
}
/* 仅相邻有效端点支持角域内部自由空间，量程突变或无效项打断；三角形距离取较短量程减余量，避免在遮挡物后画自由区。

 */
static void local_wedge(passage_local*l,const passage_config*c,double sx,double sy,double a,double d,double b,double e,uint32_t seq,uint64_t t){
 double gap=wrap(b-a),range=fmin(d,e);if(fabs(gap)<1e-9||fabs(gap)>.05235987756||fabs(d-e)>fmax(.15,.20*range))return;
 double length=range-fmax(.02,1.5*l->resolution);if(length<=l->resolution)return;
 unsigned steps=(unsigned)ceil(length*fabs(gap)/(l->resolution*.5));if(steps<2)steps=2;if(steps>128)steps=128;
 for(unsigned i=1;i<steps&&l->scan_wedge_rays<PASSAGE_WEDGE_RAYS_MAX;i++){double angle=a+gap*(double)i/steps;local_line(l,c,sx,sy,sx+length*cos(angle),sy+length*sin(angle),0,seq,t);++l->wedge_rays;++l->scan_wedge_rays;}
 ++l->wedges;
}
void passage_local_ray(passage_local*l,const passage_config*c,passage_pose p,double angle,double distance,unsigned quality,uint32_t seq,uint64_t t){
 if(!l||!l->initialized||t!=l->last_scan||seq!=l->scan_sequence||l->scan_finished)return;
 if(!quality||!isfinite(distance)||!isfinite(angle)||distance<c->lidar_min_m||distance>c->lidar_max_m){++l->invalid;++l->beam_entries;l->beam_prev_valid=0;return;}
 p=local_pose(l,p);double cs=cos(p.yaw),sn=sin(p.yaw),sx=p.x+cs*c->mount_x_m-sn*c->mount_y_m,sy=p.y+sn*c->mount_x_m+cs*c->mount_y_m;
 double a=p.yaw+c->mount_yaw_rad+angle,ex=sx+distance*cos(a),ey=sy+distance*sin(a);
 if(l->beam_prev_valid&&hypot(sx-l->beam_prev_sx,sy-l->beam_prev_sy)<1e-5)local_wedge(l,c,sx,sy,l->beam_prev_angle,l->beam_prev_range,a,distance,seq,t);
 if(!l->beam_entries){l->beam_first_valid=1;l->beam_first_sx=sx;l->beam_first_sy=sy;l->beam_first_angle=a;l->beam_first_range=distance;}
 l->beam_prev_valid=1;l->beam_prev_sx=sx;l->beam_prev_sy=sy;l->beam_prev_angle=a;l->beam_prev_range=distance;++l->beam_entries;
 int hit=local_line(l,c,sx,sy,ex,ey,1,seq,t);++l->rays;if(hit)++l->hits;
}
void passage_local_scan_finish(passage_local*l,const passage_config*c,uint32_t seq,uint64_t t){
 if(!l||!c||!l->initialized||l->last_scan!=t||l->scan_sequence!=seq||l->scan_finished)return;
 if(l->beam_first_valid&&l->beam_prev_valid&&hypot(l->beam_first_sx-l->beam_prev_sx,l->beam_first_sy-l->beam_prev_sy)<1e-5)local_wedge(l,c,l->beam_prev_sx,l->beam_prev_sy,l->beam_prev_angle,l->beam_prev_range,l->beam_first_angle,l->beam_first_range,seq,t);
 l->scan_finished=1;
}
static double body_margin(const passage_config*c){return c->r4_enabled?c->margin_m+c->wheel_outer_margin_m+.01:c->margin_m+c->wheel_outer_margin_m+(c->wheel_outer_margin_verified?0:.01)+(c->sonar_offsets_verified?0:.02);}
static poly rectangle(passage_pose p,double length,double width){
 poly q={.n=4};double c=cos(p.yaw),s=sin(p.yaw),hx=length*.5,hy=width*.5;
 double x[4]={hx,-hx,-hx,hx},y[4]={hy,hy,-hy,-hy};for(unsigned i=0;i<4;i++)q.p[i]=(passage_xy){p.x+c*x[i]-s*y[i],p.y+s*x[i]+c*y[i]};return q;
}
static int compare_xy(const void*a,const void*b){const passage_xy*x=a,*y=b;return x->x<y->x?-1:x->x>y->x?1:x->y<y->y?-1:x->y>y->y?1:0;}
static poly hull(passage_xy*p,unsigned n){
 qsort(p,n,sizeof(*p),compare_xy);poly q={0};for(unsigned i=0;i<n;i++){while(q.n>=2&&cross(q.p[q.n-2],q.p[q.n-1],p[i])<=0)--q.n;q.p[q.n++]=p[i];}
 unsigned lower=q.n;for(int i=(int)n-2;i>=0;i--){while(q.n>lower&&cross(q.p[q.n-2],q.p[q.n-1],p[i])<=0)--q.n;q.p[q.n++]=p[i];}if(q.n>1)--q.n;return q;
}
static poly clip_cell(const poly*shape,double x,double y,double res){
 poly v={.n=4};v.p[0]=(passage_xy){x,y};v.p[1]=(passage_xy){x+res,y};v.p[2]=(passage_xy){x+res,y+res};v.p[3]=(passage_xy){x,y+res};
 for(unsigned e=0;e<shape->n&&v.n;e++){
  poly next={0};passage_xy a=shape->p[e],b=shape->p[(e+1)%shape->n];
  for(unsigned i=0;i<v.n;i++){passage_xy p=v.p[i],q=v.p[(i+1)%v.n];double cp=cross(a,b,p),cq=cross(a,b,q);int in=cp>=-1e-11,inq=cq>=-1e-11;
   if(in&&next.n<16)next.p[next.n++]=p;if(in!=inq&&next.n<16){double t=cp/(cp-cq);next.p[next.n++]=(passage_xy){p.x+t*(q.x-p.x),p.y+t*(q.y-p.y)};}
  }v=next;
 }return v;
}
static int global_value(const passage_grid*g,double x,double y,double res){
 if(!g||!g->cells||!g->width||!g->height||g->resolution<=0)return -2;
 double epsilon=res*1e-8;int x0=(int)floor((x+epsilon-g->origin_x)/g->resolution),x1=(int)floor((x+res-epsilon-g->origin_x)/g->resolution),y0=(int)floor((y+epsilon-g->origin_y)/g->resolution),y1=(int)floor((y+res-epsilon-g->origin_y)/g->resolution);int result=0;
 if(x0<0||y0<0||x1>=(int)g->width||y1>=(int)g->height)return -2;
 for(int yy=y0;yy<=y1;yy++)for(int xx=x0;xx<=x1;xx++){int v=g->cells[(unsigned)yy*g->width+(unsigned)xx];if(v>=65)return 100;if(v<0)result=-1;}return result;
}
static int local_value(const passage_world*w,int x,int y,int*stale){
 const passage_local*l=w->local;if(stale)*stale=0;
 if(!l||!l->initialized||x<0||y<0||x>=(int)PASSAGE_LOCAL_SIDE||y>=(int)PASSAGE_LOCAL_SIDE)return -1;
 unsigned k=(unsigned)y*PASSAGE_LOCAL_SIDE+(unsigned)x;
 if(!l->stamp[k]||w->now_us<l->stamp[k])return -1;
 if(w->now_us-l->stamp[k]>w->config.local_ttl_us){if(stale)*stale=1;return -1;}
 if(l->score[k]>0){if(!w->confirmed_only||l->hit_count[k]>=2)return 100;if(l->clear_stamp[k]&&w->now_us>=l->clear_stamp[k]&&w->now_us-l->clear_stamp[k]<=w->config.local_ttl_us)return 0;return -1;}
 return l->score[k]<0?0:-1;
}
/* 点采样与精确几何使用同一近期局部及全局优先级；采样只作搜索提示，不授予车体或运动许可。
 */
int passage_planning_sample(const passage_world*w,double x,double y){
 if(!w||!isfinite(x)||!isfinite(y))return -2;
 const passage_local*l=w->local;int value=-1;
 if(l&&l->initialized){double c=cos(l->map_from_local_yaw),s=sin(l->map_from_local_yaw),dx=x-l->map_from_local_x,dy=y-l->map_from_local_y;
  int ix=(int)floor((c*dx+s*dy-l->origin_x)/l->resolution),iy=(int)floor((-s*dx+c*dy-l->origin_y)/l->resolution);value=local_value(w,ix,iy,NULL);}
 if(value<0&&!w->local_only)value=global_value(&w->global,x-1e-8,y-1e-8,2e-8);return value;
}
static int nearby_free(const passage_world*w,int x,int y){
 double margin=w->probe_unknown_margin_m>0?fmin(.10,w->probe_unknown_margin_m):.05,res=w->config.local_resolution_m;
 int radius=(int)ceil(margin/res);for(int dy=-radius;dy<=radius;dy++)for(int dx=-radius;dx<=radius;dx++)if((dx||dy)&&hypot(dx*res,dy*res)<=margin+1e-9&&local_value(w,x+dx,y+dy,NULL)==0)return 1;return 0;
}
static double polygon_area(const poly*p){double sum=0;for(unsigned i=0;i<p->n;i++)sum+=p->p[i].x*p->p[(i+1)%p->n].y-p->p[(i+1)%p->n].x*p->p[i].y;return fabs(sum)*.5;}
/* 每次查询的系数不可变，不为每个裁剪格重复调用数学库。 */
typedef struct passage_query {const passage_world*w;double cs,sn,ic,is;int framed;} passage_query;
static passage_xy query_map(const passage_query*q,passage_xy p){const passage_local*l=q->w->local;return (passage_xy){l->map_from_local_x+q->cs*p.x-q->sn*p.y,l->map_from_local_y+q->sn*p.x+q->cs*p.y};}
static poly to_map(const passage_query*q,poly p){if(q->framed)for(unsigned i=0;i<p.n;i++)p.p[i]=query_map(q,p.p[i]);return p;}
static int query_initial(const passage_query*q,const poly*p){const passage_world*w=q->w;for(unsigned i=0;i<p->n;i++){double dx=p->p[i].x-w->initial.x,dy=p->p[i].y-w->initial.y,h=w->config.local_resolution_m*.5;if(fabs(q->ic*dx+q->is*dy)>w->config.length_m*.5+h+1e-9||fabs(-q->is*dx+q->ic*dy)>w->config.width_m*.5+h+1e-9)return 0;}return 1;}
/* 保守分类，不确定边界格仍用原精确裁剪，包括小面积及浮点微量处理。
 */
#ifndef PASSAGE_EXACT_REFERENCE
static int cell_relation(const poly*p,double x,double y,double r){
 int full=1;for(unsigned i=0;i<p->n;i++){passage_xy a=p->p[i],b=p->p[(i+1)%p->n];double nx=-(b.y-a.y),ny=b.x-a.x,cp=nx*(x+r*.5-a.x)+ny*(y+r*.5-a.y),rad=(fabs(nx)+fabs(ny))*r*.5;
 if(cp+rad < -1e-10)return -1;if(cp-rad < 1e-10)full=0;}return full;
}
#endif
static int global_local_cell(const passage_query*q,double x,double y,double res){
 const passage_world*w=q->w;
 const passage_local*l=w->local;if(!l||!l->initialized)return global_value(&w->global,x,y,res);
 if(fabs(l->map_from_local_yaw)<1e-12)return global_value(&w->global,x+l->map_from_local_x,y+l->map_from_local_y,res);
 poly cell={.n=4,.p={{x,y},{x+res,y},{x+res,y+res},{x,y+res}}};cell=to_map(q,cell);double minx=cell.p[0].x,maxx=minx,miny=cell.p[0].y,maxy=miny;
 for(unsigned i=1;i<4;i++){minx=fmin(minx,cell.p[i].x);maxx=fmax(maxx,cell.p[i].x);miny=fmin(miny,cell.p[i].y);maxy=fmax(maxy,cell.p[i].y);}
 const passage_grid*g=&w->global;int x0=(int)floor((minx-g->origin_x)/g->resolution),x1=(int)floor((maxx-g->origin_x)/g->resolution),y0=(int)floor((miny-g->origin_y)/g->resolution),y1=(int)floor((maxy-g->origin_y)/g->resolution),result=0;
 for(int yy=y0;yy<=y1;yy++)for(int xx=x0;xx<=x1;xx++){poly intersection=clip_cell(&cell,g->origin_x+xx*g->resolution,g->origin_y+yy*g->resolution,g->resolution);if(intersection.n<3||polygon_area(&intersection)<1e-14)continue;if(xx<0||yy<0||xx>=(int)g->width||yy>=(int)g->height)return -2;int value=g->cells[(unsigned)yy*g->width+(unsigned)xx];if(value>=65)return 100;if(value<0)result=-1;}return result;
}
static int polygon_clear(const passage_world*w,const poly*p,passage_result*out){
 const passage_local*l=w->local;int framed=l&&l->initialized;passage_query query={.w=w,.framed=framed,.ic=cos(w->initial.yaw),.is=sin(w->initial.yaw)};if(framed){query.cs=cos(l->map_from_local_yaw);query.sn=sin(l->map_from_local_yaw);}poly shape=*p;if(framed)for(unsigned i=0;i<shape.n;i++){double x=shape.p[i].x-l->map_from_local_x,y=shape.p[i].y-l->map_from_local_y;shape.p[i]=(passage_xy){query.cs*x+query.sn*y,-query.sn*x+query.cs*y};}
 double minx=shape.p[0].x,maxx=minx,miny=shape.p[0].y,maxy=miny,res=w->config.local_resolution_m,ox=framed?l->origin_x:0,oy=framed?l->origin_y:0;
 for(unsigned i=1;i<shape.n;i++){minx=fmin(minx,shape.p[i].x);maxx=fmax(maxx,shape.p[i].x);miny=fmin(miny,shape.p[i].y);maxy=fmax(maxy,shape.p[i].y);}
 int x0=(int)floor((minx-ox)/res),x1=(int)floor((maxx-ox)/res),y0=(int)floor((miny-oy)/res),y1=(int)floor((maxy-oy)/res);
 if((uint64_t)(x1-x0+1)*(uint64_t)(y1-y0+1)>2000000){out->reason=PASSAGE_SEARCH_LIMIT;return 0;}
 double free_area=0,unknown_area=0;unsigned unsupported=0;int first_unknown=0;double unknown_x=0,unknown_y=0;
 for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++){
  if(x==x0&&w->cancelled&&w->cancelled(w->cancel_user)){out->reason=PASSAGE_SEARCH_LIMIT;return 0;}
  double wx=ox+x*res,wy=oy+y*res;poly intersection;double area;
#ifndef PASSAGE_EXACT_REFERENCE
  int relation=cell_relation(&shape,wx,wy,res);if(relation<0)continue;
  if(relation>0){intersection=(poly){.n=4,.p={{wx,wy},{wx+res,wy},{wx+res,wy+res},{wx,wy+res}}};area=polygon_area(&intersection);}
  else
#endif
  {intersection=clip_cell(&shape,wx,wy,res);if(intersection.n<3)continue;area=polygon_area(&intersection);if(area<1e-14)continue;}
  int aged=0,value=framed?local_value(w,x,y,&aged):-1;++out->cells_checked;if(aged)++out->stale_cells;
  if(value<0&&!w->local_only&&!w->observed_probe)value=global_local_cell(&query,wx,wy,res);
  if(value>=65){++out->occupied_cells;passage_xy q=framed?map_xy(l,(passage_xy){wx+res*.5,wy+res*.5}):(passage_xy){wx+res*.5,wy+res*.5};out->reason=PASSAGE_OCCUPIED;out->x=q.x;out->y=q.y;return 0;}
  if(value==0){++out->observed_cells;free_area+=area;continue;}
  poly in_map=framed?to_map(&query,intersection):intersection;
  if(w->allow_initial_unknown&&query_initial(&query,&in_map)){out->used_initial_unknown=1;continue;}
  ++out->unknown_cells;unknown_area+=area;if(w->observed_probe&&(!framed||!nearby_free(w,x,y)))++unsupported;
  if(!first_unknown){first_unknown=value==-2?PASSAGE_OUTSIDE:PASSAGE_UNKNOWN;passage_xy q=framed?map_xy(l,(passage_xy){wx+res*.5,wy+res*.5}):(passage_xy){wx+res*.5,wy+res*.5};unknown_x=q.x;unknown_y=q.y;}
  if(!w->observed_probe&&!w->planning_unknown_soft){out->reason=first_unknown;out->x=unknown_x;out->y=unknown_y;out->observed_fraction=(double)out->observed_cells/fmax(1,out->observed_cells+out->unknown_cells);return 0;}
 }
 out->observed_fraction=(double)out->observed_cells/fmax(1,out->observed_cells+out->unknown_cells);
 if(unknown_area>0){
  if(w->planning_unknown_soft){out->reason=PASSAGE_TENTATIVE;return 1;}
  double fraction=w->probe_unknown_fraction>0?fmin(.5,w->probe_unknown_fraction):.20;
  if(w->observed_probe&&framed&&free_area>0&&!unsupported&&unknown_area/(free_area+unknown_area)<=fraction){out->reason=PASSAGE_OBSERVED_PROBE;out->used_probe=1;return 1;}
  out->reason=first_unknown;out->x=unknown_x;out->y=unknown_y;return 0;
 }
 return 1;
}
static int valid_world(const passage_world*w,passage_pose p){return w&&!(w->observed_probe&&w->planning_unknown_soft)&&passage_config_valid(&w->config)&&w->global.cells&&w->global.width&&w->global.height&&w->global.width<=4096&&w->global.height<=4096&&isfinite(w->global.resolution)&&w->global.resolution>=.01&&w->global.resolution<=1&&isfinite(w->global.origin_x)&&isfinite(w->global.origin_y)&&isfinite(p.x)&&isfinite(p.y)&&isfinite(p.yaw)&&fabs(p.x)<1e6&&fabs(p.y)<1e6;}
int passage_pose_clear(const passage_world*w,passage_pose p,double width,passage_result*out){
 passage_result tmp={0};if(!out)out=&tmp;memset(out,0,sizeof(*out));if(!valid_world(w,p)||!isfinite(width)||width<w->config.width_m||width>2){out->reason=PASSAGE_INVALID;return 0;}
 passage_world bounded=*w;double radius=hypot(w->config.length_m*.5,w->config.width_m*.5);if(w->observed_probe&&hypot(p.x-w->initial.x,p.y-w->initial.y)+radius*fabs(wrap(p.yaw-w->initial.yaw))>.25){bounded.observed_probe=0;w=&bounded;}
 poly shape=rectangle(p,w->config.length_m+2*body_margin(&w->config),width+2*body_margin(&w->config));return polygon_clear(w,&shape,out);
}
int passage_translate_clear(const passage_world*w,passage_pose a,passage_xy b,double width,passage_result*out){
 passage_result tmp={0};if(!out)out=&tmp;memset(out,0,sizeof(*out));if(!valid_world(w,a)||!isfinite(b.x)||!isfinite(b.y)||fabs(b.x)>=1e6||fabs(b.y)>=1e6||!isfinite(width)||width<w->config.width_m||width>2){out->reason=PASSAGE_INVALID;return 0;}
 passage_world bounded=*w;if(w->observed_probe&&hypot(b.x-a.x,b.y-a.y)>.25){bounded.observed_probe=0;w=&bounded;}
 poly first=rectangle(a,w->config.length_m+2*body_margin(&w->config),width+2*body_margin(&w->config)),last=rectangle((passage_pose){b.x,b.y,a.yaw},w->config.length_m+2*body_margin(&w->config),width+2*body_margin(&w->config));passage_xy pts[8];memcpy(pts,first.p,4*sizeof(*pts));memcpy(pts+4,last.p,4*sizeof(*pts));poly shape=hull(pts,8);return polygon_clear(w,&shape,out);
}
int passage_rotate_clear(const passage_world*w,passage_pose a,double target,passage_result*out){
 passage_result tmp={0};if(!out)out=&tmp;memset(out,0,sizeof(*out));if(!valid_world(w,a)||!isfinite(target)){out->reason=PASSAGE_INVALID;return 0;}
 double angle=wrap(target-a.yaw);unsigned steps=(unsigned)ceil(fabs(angle)/.025);if(!steps)steps=1;
 double radius=hypot(w->config.length_m*.5+body_margin(&w->config),w->config.width_m*.5+body_margin(&w->config)),pad=radius*fabs(angle)/steps*.5;
 passage_world bounded=*w;if(w->observed_probe&&radius*fabs(angle)>.25){bounded.observed_probe=0;w=&bounded;}
 for(unsigned i=0;i<=steps;i++){if(w->cancelled&&w->cancelled(w->cancel_user)){out->reason=PASSAGE_SEARCH_LIMIT;return 0;}passage_pose p=a;p.yaw+=angle*i/steps;poly shape=rectangle(p,w->config.length_m+2*(body_margin(&w->config)+pad),w->config.width_m+2*(body_margin(&w->config)+pad));if(!polygon_clear(w,&shape,out))return 0;}return 1;
}
int passage_arc_clear(const passage_world*w,passage_pose a,double v,double omega,double horizon,passage_result*out){
 passage_result tmp={0};if(!out)out=&tmp;memset(out,0,sizeof(*out));if(!valid_world(w,a)||!isfinite(v)||!isfinite(omega)||!isfinite(horizon)||fabs(v)>1||fabs(omega)>3||horizon<0||horizon>3){out->reason=PASSAGE_INVALID;return 0;}
 if(fabs(omega)<1e-10)return passage_translate_clear(w,a,(passage_xy){a.x+v*horizon*cos(a.yaw),a.y+v*horizon*sin(a.yaw)},w->config.width_m,out);
 double radius=hypot(w->config.length_m*.5+body_margin(&w->config),w->config.width_m*.5+body_margin(&w->config)),speed=fabs(v)+radius*fabs(omega);
 passage_world bounded=*w;if(w->observed_probe&&speed*horizon>.25){bounded.observed_probe=0;w=&bounded;}
 unsigned steps=(unsigned)ceil(fmax(horizon/.025,horizon*speed/.005));if(!steps)steps=1;if(steps>2000){out->reason=PASSAGE_SEARCH_LIMIT;return 0;}double dt=horizon/steps,pad=speed*dt*.5;
 for(unsigned i=0;i<=steps;i++){if(w->cancelled&&w->cancelled(w->cancel_user)){out->reason=PASSAGE_SEARCH_LIMIT;return 0;}double t=dt*i,angle=a.yaw+omega*t;passage_pose p={a.x+v/omega*(sin(angle)-sin(a.yaw)),a.y+v/omega*(cos(a.yaw)-cos(angle)),angle};
  poly shape=rectangle(p,w->config.length_m+2*(body_margin(&w->config)+pad),w->config.width_m+2*(body_margin(&w->config)+pad));if(!polygon_clear(w,&shape,out))return 0;
 }return 1;
}
const char*passage_reason_text(int r){const char*n[]={"CLEAR","OCCUPIED_FOOTPRINT","UNKNOWN_NEW_AREA","OUTSIDE_MAP","INVALID_GEOMETRY","GEOMETRY_BUDGET_EXCEEDED","NO_RECTANGULAR_ROUTE","OBSERVED_SHORT_PROBE","TENTATIVE_UNKNOWN_ROUTE","OBSERVED_PARTIAL_ROUTE"};return r>=0&&r<10?n[r]:"UNKNOWN";}
