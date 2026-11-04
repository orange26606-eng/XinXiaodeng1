#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "live_nav_ui.h"
#include "s5_font_data.h"
#include "s5_status_text.h"
#include "r4_status_text.h"
#include "passage.h"
#include "navigation_status.h"
#include "motion_preview.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#if defined(__linux__)
#include <time.h>
#endif
typedef struct canvas{uint32_t*p;int w,h;}canvas;
static void box(canvas c,int x,int y,int w,int h,uint32_t color){for(int j=y<0?0:y;j<y+h&&j<c.h;++j)for(int i=x<0?0:x;i<x+w&&i<c.w;++i)c.p[(size_t)j*c.w+i]=color;}
static uint32_t s5_next_utf8(const unsigned char**input){
 const unsigned char*p=*input;uint32_t c=*p++;unsigned n=0;
 if(c<128){*input=p;return c;}if((c&0xe0)==0xc0){c&=31;n=1;}else if((c&0xf0)==0xe0){c&=15;n=2;}else if((c&0xf8)==0xf0){c&=7;n=3;}else{*input=p;return '?';}
 for(unsigned k=0;k<n;k++){if(!*p||(*p&0xc0)!=0x80){*input=p;return '?';}c=(c<<6)|(*p++&63);}*input=p;return c;
}
static void text(canvas c,int x,int y,const char*s,uint32_t color){
 const unsigned char*p=(const unsigned char*)s;
 while(*p){uint32_t cp=s5_next_utf8(&p);const struct s5_glyph*g=NULL;size_t lo=0,hi=S5_FONT_COUNT;
  while(lo<hi){size_t m=lo+(hi-lo)/2;if(s5_glyphs[m].code<cp)lo=m+1;else hi=m;}
  if(lo<S5_FONT_COUNT&&s5_glyphs[lo].code==cp)g=&s5_glyphs[lo];if(!g)continue;
  for(int a=0;a<g->height;++a)for(int b=0;b<g->width;++b){int px=x+g->left+b,py=y+S5_FONT_ASCENT+g->top+a;if(px<0||py<0||px>=c.w||py>=c.h)continue;unsigned k=s5_font_pixels[g->offset+(size_t)a*g->width+b];uint32_t old=c.p[(size_t)py*c.w+px];unsigned rr=(((color>>16)&255)*k+((old>>16)&255)*(255-k))/255,gg=(((color>>8)&255)*k+((old>>8)&255)*(255-k))/255,bb=((color&255)*k+(old&255)*(255-k))/255;c.p[(size_t)py*c.w+px]=(rr<<16)|(gg<<8)|bb;}
  x+=g->advance;if(x>=c.w-10)break;
 }
}
static int side(const live_nav_ui*u){int s=u->width-330;return s<u->height-180?s:u->height-180;}
static int point(const live_nav_ui*u,double x,double y,int*px,int*py){int s=side(u);double xx=(x-u->center_x)/u->span_m+.5,yy=.5-(y-u->center_y)/u->span_m;if(!isfinite(xx)||!isfinite(yy)||xx<0||xx>=1||yy<0||yy>=1)return 0;*px=16+(int)(s*xx);*py=150+(int)(s*yy);return 1;}
static void segment(canvas c,int x,int y,int xx,int yy,uint32_t color){int dx=abs(xx-x),dy=-abs(yy-y),sx=x<xx?1:-1,sy=y<yy?1:-1,e=dx+dy;for(int n=0;n<c.w+c.h+2;++n){box(c,x,y,2,2,color);if(x==xx&&y==yy)break;int k=e*2;if(k>=dy){e+=dy;x+=sx;}if(k<=dx){e+=dx;y+=sy;}}}
void live_nav_ui_init(live_nav_ui*u){memset(u,0,sizeof(*u));u->span_m=4;snprintf(u->message,sizeof(u->message),"Phone map: select / preview / explicit START");}
int live_nav_ui_button(int x,int y){if(y<91||y>=132)return 0;if(x>=16&&x<176)return 1;if(x>=192&&x<352)return 2;if(x>=368&&x<448)return 3;if(x>=464&&x<544)return 4;return 0;}
int live_nav_ui_select_world(live_nav_ui*u,const lab_real_runtime_view*v,uint64_t map_id,uint64_t map_version,double wx,double wy){
 const lab_real_grid_view*m=&v->localization.map;if(!lab_real_grid_valid(m)||!v->has_localization){snprintf(u->message,sizeof(u->message),"WAIT_MAP_AND_LOCALIZATION");return -1;}
 /* 地图内容更新保持世界坐标，使用当前栅格重新核验目标。 */

 if(!map_version||map_id!=m->map_id||map_version>m->map_version){snprintf(u->message,sizeof(u->message),"STALE_MAP");return -1;}
 int content_revalidated=map_version<m->map_version;
 if(!isfinite(wx)||!isfinite(wy)){snprintf(u->message,sizeof(u->message),"INVALID_COORDINATES");return -1;}
 double col=(wx-m->origin_x_m)/m->resolution_m,row=(wy-m->origin_y_m)/m->resolution_m;
 if(col<0||row<0||col>=m->width||row>=m->height){snprintf(u->message,sizeof(u->message),"GOAL_OUTSIDE_MAP");return -1;}
 u->requested_x=wx;u->requested_y=wy;u->goal_adjusted=0;u->target_observation_pending=0;
 if(v->context.source==LAB_REAL_HARDWARE&&v->navigation.body_length_m>0){
  passage_world world={0};passage_config_default(&world.config);world.config.length_m=v->navigation.body_length_m;world.config.width_m=v->navigation.body_width_m;world.config.margin_m=v->navigation.margin_m;
  world.global=(passage_grid){m->width,m->height,m->resolution_m,m->origin_x_m,m->origin_y_m,m->cells};
  world.initial=(passage_pose){v->localization.pose.x_m,v->localization.pose.y_m,v->localization.pose.yaw_rad};world.allow_initial_unknown=1;
  world.planning_unknown_soft=1; /* 路径预览状态。 */
  passage_result hit;double angle=atan2(wy-v->localization.pose.y_m,wx-v->localization.pose.x_m);
  if(!passage_pose_clear(&world,(passage_pose){wx,wy,angle},world.config.width_m,&hit)){
   int found=0;double best=.25*.25+1e-9,bx=wx,by=wy;
   /* 仅作局部调整，JSON仍保留准确请求坐标。 */
   for(int iy=-10;iy<=10;iy++)for(int ix=-10;ix<=10;ix++){double dx=ix*.025,dy=iy*.025,d=dx*dx+dy*dy;if(d>=best||d==0)continue;
    double x=wx+dx,y=wy+dy,a=atan2(y-v->localization.pose.y_m,x-v->localization.pose.x_m);
    if(x<m->origin_x_m||y<m->origin_y_m||x>=m->origin_x_m+m->width*m->resolution_m||y>=m->origin_y_m+m->height*m->resolution_m)continue;
    if(hypot(x-v->localization.pose.x_m,y-v->localization.pose.y_m)<.12&&hypot(wx-v->localization.pose.x_m,wy-v->localization.pose.y_m)>.2)continue;
    passage_world strict=world;strict.planning_unknown_soft=0;
    if(passage_pose_clear(&strict,(passage_pose){x,y,a},world.config.width_m,&hit)){found=1;best=d;bx=x;by=y;}
   }
   if(!found){snprintf(u->message,sizeof(u->message),"GOAL_NO_NEARBY_FOOTPRINT_SPACE");return -1;}
   wx=bx;wy=by;u->goal_adjusted=1;col=(wx-m->origin_x_m)/m->resolution_m;row=(wy-m->origin_y_m)/m->resolution_m;
  }else if(hit.reason==PASSAGE_TENTATIVE||hit.unknown_cells)u->target_observation_pending=1;
 }
 int value=m->cells[(size_t)row*m->width+(size_t)col];if(value>=65){snprintf(u->message,sizeof(u->message),"GOAL_OCCUPIED_REJECTED");return -1;}if(value<0)u->target_observation_pending=1;
 u->goal_x=wx;u->goal_y=wy;u->selected=1;snprintf(u->message,sizeof(u->message),"Selected %.2f, %.2f m; %s",wx,wy,u->target_observation_pending?"TARGET_OBSERVATION_PENDING":content_revalidated?"CURRENT_MAP_REVALIDATED":"START");return 1;
}
int live_nav_ui_pick(live_nav_ui*u,const lab_real_runtime_view*v,int x,int y){int s=side(u);if(s<=0||x<16||x>=16+s||y<150||y>=150+s)return 0;
 double wx=u->center_x+((double)(x-16)/s-.5)*u->span_m,wy=u->center_y+(.5-(double)(y-150)/s)*u->span_m;
 return live_nav_ui_select_world(u,v,v->localization.map.map_id,v->localization.map.map_version,wx,wy);
}
void live_nav_ui_draw(live_nav_ui*u,uint32_t*p,int w,int h,const lab_real_runtime_view*v,int drive){u->width=w;u->height=h;if(w<640||h<480)return;
 int pose_valid=navigation_pose_valid(v);
 if(u->last_pose_map_id!=v->localization.map.map_id){u->have_last_pose=0;u->last_pose_map_id=v->localization.map.map_id;if(!pose_valid){u->center_x=v->localization.map.origin_x_m+v->localization.map.width*v->localization.map.resolution_m*.5;u->center_y=v->localization.map.origin_y_m+v->localization.map.height*v->localization.map.resolution_m*.5;}}
 if(pose_valid){u->center_x=u->last_pose_x=v->localization.pose.x_m;u->center_y=u->last_pose_y=v->localization.pose.y_m;u->last_pose_yaw=v->localization.pose.yaw_rad;u->last_pose_time=v->localization.pose_time_us;u->have_last_pose=1;}
 canvas c={p,w,h};box(c,0,0,w,h,0x111c2c);text(c,16,10,"ACG138 S6-R8 运动补偿 · 主动观察 · 持续建图",0xffffff);
 if(v->context.source!=LAB_REAL_HARDWARE)text(c,w-250,10,"合成测试，非实车",0xffc05e);
 char b[180];snprintf(b,sizeof(b),"%s | %s | %s | 轮速上限120RPM",v->context.source==LAB_REAL_HARDWARE?"真实传感器":"合成测试数据",drive?"真实驾驶入口":"仅预览",v->enabled?"已使能":v->pending_enable?"正在使能":v->has_goal?"任务等待可行动作":"当前无导航任务");text(c,16,43,b,0xffc05e);
 box(c,16,91,160,41,0x186747);text(c,28,97,"开始 / Enter",0xffffff);box(c,192,91,160,41,0x8c3030);text(c,208,97,"停车 / Space",0xffffff);
 box(c,368,91,80,41,0x304b70);text(c,381,97,"放大 +",0xffffff);box(c,464,91,80,41,0x304b70);text(c,475,97,"缩小 -",0xffffff);
 int s=side(u);const lab_real_grid_view*m=&v->localization.map;int map_valid=lab_real_grid_valid(m);
 for(int y=0;y<s;++y)for(int x=0;x<s;++x){double wx=u->center_x+((double)x/s-.5)*u->span_m,wy=u->center_y+(.5-(double)y/s)*u->span_m;uint32_t color=0x45536a;
  if(map_valid){double xx=(wx-m->origin_x_m)/m->resolution_m,yy=(wy-m->origin_y_m)/m->resolution_m;if(xx>=0&&yy>=0&&xx<m->width&&yy<m->height){int q=m->cells[(size_t)yy*m->width+(size_t)xx];if(q>=0){unsigned k=245-q*2;color=(k<<16)|(k<<8)|k;}}}p[(size_t)(150+y)*w+16+x]=color;}
 int px=0,py=0,lx=0,ly=0,have=0;
 for(uint32_t i=0;m->trace&&i<m->trace_count&&i<LAB_TRACE_MAX;++i){int yes=point(u,m->trace[i].x_m,m->trace[i].y_m,&px,&py);if(yes&&have)segment(c,lx,ly,px,py,0x3388ff);have=yes;lx=px;ly=py;}
 have=0;for(uint32_t i=0;v->has_goal&&v->navigation.path&&i<v->navigation.path_count&&i<LAB_REAL_PATH_MAX;++i){int yes=point(u,v->navigation.path[i].x_m,v->navigation.path[i].y_m,&px,&py);if(yes&&have)segment(c,lx,ly,px,py,0xffa100);have=yes;lx=px;ly=py;}
 uint64_t drawing_now=v->time_us;
#if defined(__linux__)
 if(v->context.source==LAB_REAL_HARDWARE){struct timespec wall;if(!clock_gettime(CLOCK_MONOTONIC,&wall)){uint64_t actual=(uint64_t)wall.tv_sec*1000000u+(uint64_t)wall.tv_nsec/1000u;if(actual>drawing_now)drawing_now=actual;}}
#endif
 s2_motion_preview projection;int predicted=s2_motion_preview_make(v,drawing_now,&projection);
 const s3_nearfield_view*nf=&v->navigation.nearfield;
 if(pose_valid&&v->localization.pose_time_us<=drawing_now&&drawing_now-v->localization.pose_time_us<=500000&&nf->scan_time_us&&nf->scan_time_us<=drawing_now&&drawing_now-nf->scan_time_us<=600000){
  double a=v->localization.pose.yaw_rad,ca=cos(a),sa=sin(a);
  for(unsigned i=0;i<nf->hazard_count&&i<S3_HAZARD_VIEW_MAX;i++){const s3_hazard_point*q=&nf->hazards[i];if(!q->time_us||q->time_us>drawing_now||!isfinite(q->x_m)||!isfinite(q->y_m))continue;
   if(point(u,v->localization.pose.x_m+ca*q->x_m-sa*q->y_m,v->localization.pose.y_m+sa*q->x_m+ca*q->y_m,&px,&py))box(c,px-3,py-3,7,7,nf->valid&&!nf->stale&&drawing_now-nf->scan_time_us<=300000&&drawing_now-q->time_us<=300000?0xff6558:0x929ca4);
  }
 }
 if(predicted){have=0;for(unsigned i=0;i<projection.count;i++){int yes=point(u,projection.points[i].x,projection.points[i].y,&px,&py);if(yes&&have&&(projection.commanded||i%2))segment(c,lx,ly,px,py,0x20d6de);have=yes;lx=px;ly=py;}
  /* 纯旋转的中心位移为零，另外显示角度进展。 */
  if(fabs(projection.v)<.01&&fabs(projection.w)>.05){have=0;for(unsigned i=0;i<projection.count;i++){double a=projection.points[i].yaw;int yes=point(u,projection.points[0].x+.14*cos(a),projection.points[0].y+.14*sin(a),&px,&py);if(yes&&have)segment(c,lx,ly,px,py,0x20d6de);have=yes;lx=px;ly=py;}}
  if(v->navigation.body_length_m>0&&v->navigation.body_width_m>0)for(unsigned k=0;k<projection.count;k+=8){const s2_preview_point*q=&projection.points[k];double ca=cos(q->yaw),sa=sin(q->yaw);int fx=0,fy=0,prevx=0,prevy=0,inside=1;const int sx[4]={1,1,-1,-1},sy[4]={1,-1,-1,1};for(unsigned j=0;j<4;j++){double x=sx[j]*v->navigation.body_length_m*.5,y=sy[j]*v->navigation.body_width_m*.5;int xx,yy;if(!point(u,q->x+ca*x-sa*y,q->y+sa*x+ca*y,&xx,&yy)){inside=0;break;}if(j)segment(c,prevx,prevy,xx,yy,0x208e99);else{fx=xx;fy=yy;}prevx=xx;prevy=yy;}if(inside)segment(c,prevx,prevy,fx,fy,0x208e99);}
 }
 if(pose_valid&&point(u,v->localization.pose.x_m,v->localization.pose.y_m,&px,&py)){box(c,px-4,py-4,9,9,0x00a878);int xx=px+(int)(16*cos(v->localization.pose.yaw_rad)),yy=py-(int)(16*sin(v->localization.pose.yaw_rad));segment(c,px,py,xx,yy,0x00a878);}
 if(!pose_valid&&u->have_last_pose&&point(u,u->last_pose_x,u->last_pose_y,&px,&py)){box(c,px-4,py-4,9,9,0x9da5b0);segment(c,px,py,px+(int)(16*cos(u->last_pose_yaw)),py-(int)(16*sin(u->last_pose_yaw)),0x9da5b0);}
 if(pose_valid&&v->navigation.body_length_m>0){
  for(int outline=0;outline<2;outline++){double margin=outline?v->navigation.margin_m:0,ca=cos(v->localization.pose.yaw_rad),sa=sin(v->localization.pose.yaw_rad);int firstx=0,firsty=0,prevx=0,prevy=0,ok=1;const int sx[4]={1,1,-1,-1},sy[4]={1,-1,-1,1};
   for(int i=0;i<4;i++){double x=sx[i]*(v->navigation.body_length_m*.5+margin),y=sy[i]*(v->navigation.body_width_m*.5+margin);int xx,yy;if(!point(u,v->localization.pose.x_m+ca*x-sa*y,v->localization.pose.y_m+sa*x+ca*y,&xx,&yy)){ok=0;break;}if(i)segment(c,prevx,prevy,xx,yy,outline?0xedcb72:0x00a878);else{firstx=xx;firsty=yy;}prevx=xx;prevy=yy;}if(ok)segment(c,prevx,prevy,firstx,firsty,outline?0xedcb72:0x00a878);
  }
 }
 if(u->selected&&point(u,u->goal_x,u->goal_y,&px,&py)){box(c,px-7,py-1,15,3,0xe32655);box(c,px-1,py-7,3,15,0xe32655);}
 int x=32+s,y=150;
 #define ROW(str,col) do{if(y<h-40)text(c,x,y,str,col);y+=24;}while(0)
 int feedback_fresh=navigation_feedback_fresh(v);
 const s3_avoidance_view*a=&v->navigation.avoidance;
 text(c,16,69,"橙色参考路线 | 青色条件预测 | 蓝色实际轨迹",0x65d7bb);
 if(nf->scan_time_us&&nf->scan_time_us<=drawing_now&&drawing_now-nf->scan_time_us<=300000){
  snprintf(b,sizeof(b),"前方雷达：%u回波 · %u/6角格",nf->forward_returns,nf->forward_bins);ROW(b,nf->forward_returns?0x65d7bb:0xffc05e);
  if(nf->forward_corridor_m>=0)snprintf(b,sizeof(b),"当前前走廊净空 %.2fm",nf->forward_corridor_m);else snprintf(b,sizeof(b),"前走廊未见有效端点（不等于空闲）");ROW(b,0xb7c9de);
 }else ROW("前方雷达：等待新的有效扫描",0xffc05e);
 snprintf(b,sizeof(b),"场景：%s",a->scene[0]?r4_scene_zh(a->scene_id):"等待本轮观测");ROW(b,0xffc05e);
 if(!feedback_fresh)ROW("执行反馈：未收到新鲜轮速，不能确认转动",0xffc05e);
 else if(!navigation_wheels_moving(v))ROW("执行反馈：四轮近零，所选动作尚未转动",0xffc05e);
 else ROW("执行反馈：编码器确认轮子转动，非车身位移确认",0x65d7bb);
 snprintf(b,sizeof(b),"路线期望%s · 本轮选定%s",r6_route_side_zh(a->route_expected_side,a->route_expected_v_m_s,a->route_expected_w_rad_s),r6_route_side_zh(a->route_actual_side,a->route_actual_v_m_s,a->route_actual_w_rad_s));ROW(b,0x65d7bb);
 if(a->route_side_changed){ROW("已改向：",0xffc05e);ROW(a->route_change_reason[0]?r4_reason_zh(a->route_change_reason):"缺少具体换侧原因，请查看诊断",0xffc05e);}
 if((v->has_goal||v->enabled||v->pending_enable)&&a->motion_check_time_us&&a->motion_check_time_us<=v->time_us&&v->time_us-a->motion_check_time_us<=300000){ROW(r6_check_zh(a->motion_check_result),a->motion_check_result==R6_CHECK_OCCUPIED?0xff6558:0xb7c9de);}
 snprintf(b,sizeof(b),"任务阶段：%s",r4_stage_zh(a->scene_stage));ROW(b,0xffffff);
 if(a->scene_pending)ROW("场景确认中，动作继续独立检查",0x65d7bb);
 char ds[4][40];for(unsigned d=0;d<4;d++){
  if(a->usable_direction_mask&(1u<<d))snprintf(ds[d],sizeof(ds[d]),"可用");
  else if(a->direction_age_us[d]==UINT64_MAX)snprintf(ds[d],sizeof(ds[d]),"无来源");
  else snprintf(ds[d],sizeof(ds[d]),"待更新%llums",(unsigned long long)(a->direction_age_us[d]/1000));
 }
 snprintf(b,sizeof(b),"方向 前%s 左%s 右%s 后%s",ds[0],ds[1],ds[2],ds[3]);ROW(b,0xb7c9de);
 const char*description=r4_reason_zh(a->scene_reason);
 /* 按UTF-8字符分行，避免具体中文原因在右侧面板被单行裁掉。 */
 unsigned glyph_limit=(unsigned)fmax(8,(w-x-20)/18),offset=0;
 for(unsigned line=0;description[offset]&&line<3;line++){
  unsigned end=offset,glyphs=0;while(description[end]&&glyphs<glyph_limit){unsigned char q=(unsigned char)description[end];unsigned bytes=q<128?1:(q&0xe0)==0xc0?2:(q&0xf0)==0xe0?3:4;end+=bytes;glyphs++;}
  snprintf(b,sizeof(b),"%.*s",(int)(end-offset),description+offset);ROW(b,0xffc05e);offset=end;
 }
 snprintf(b,sizeof(b),"控制：%s · 前进上限%.2fm/s",r4_owner_zh(a->scene_owner),a->scene_speed_cap);ROW(b,0x65d7bb);
 if(a->rejected_reason[0]&&strcmp(a->rejected_reason,"NONE")){ROW("最近候选未采用：",0xffc05e);ROW(r4_reason_zh(a->rejected_reason),0xffc05e);}
 snprintf(b,sizeof(b),"请求 %.3fm/s · 转向 %.3frad/s",v->request.linear_m_s,v->request.angular_rad_s);ROW(b,0xffffff);
 snprintf(b,sizeof(b),"目标轮速 %.1f/%.1f/%.1f/%.1f",v->protected_output.wheel_rpm[0],v->protected_output.wheel_rpm[1],v->protected_output.wheel_rpm[2],v->protected_output.wheel_rpm[3]);ROW(b,0xb7c9de);
 if(feedback_fresh){snprintf(b,sizeof(b),"实际轮速 %.1f/%.1f/%.1f/%.1f",v->feedback.actual_rpm[0],v->feedback.actual_rpm[1],v->feedback.actual_rpm[2],v->feedback.actual_rpm[3]);ROW(b,0xffffff);}else ROW("实际轮速：反馈过期或尚未收到",0xffc05e);
 snprintf(b,sizeof(b),"净空 前%.3f 左%.3f 右%.3fm",a->front_clearance_m,a->side_clearance_m[0],a->side_clearance_m[1]);ROW(b,0xb7c9de);
 unsigned motor_phase=0,motor_retries=0,motor_stop=0;
 if(navigation_shared_motor_state(v->time_us,&motor_phase,&motor_retries,&motor_stop)&&motor_phase){
  static const char*names[]={"反馈正常","受限恢复","停机等待重试","试运行恢复","最终反馈故障"};
  snprintf(b,sizeof(b),"电机：%s · 已重试%u次",names[motor_phase],motor_retries);ROW(b,0xffc05e);
 }
 if(v->feedback.faults){snprintf(b,sizeof(b),"底盘反馈故障位：0x%X",v->feedback.faults);ROW(b,0xff6558);}

 if(a->nf_emergency)ROW("紧急近障：已刹车，具体方向见原因",0xff6558);
 if(a->turn_hold)ROW("转向后停稳，等两笔新样本≥40毫米",0xffc05e);
 snprintf(b,sizeof(b),"前方 观察%u 近障%u 禁前进%u 急刹%u 恢复%umm",R6_FRONT_OBSERVE_MM,R6_FRONT_NEAR_MM,R6_FRONT_FORWARD_STOP_MM,R6_FRONT_EMERGENCY_MM,R6_FRONT_RELEASE_MM);ROW(b,0xb7c9de);
 if(!a->calibrated)ROW("窄缝参数未经实车复核（仅提示）",0xffc05e);
 snprintf(b,sizeof(b),"已探索%u轮 · 后退%u次 · 扩展%u次",a->explore_round,a->reverse_attempts,a->reverse_extended_attempts);ROW(b,0x65d7bb);
 snprintf(b,sizeof(b),"局部规划：%s %.2fm/s %.2frad/s",a->planner_active?"执行轨迹":"场景规则",a->planner_v_m,a->planner_w_rad_s);ROW(b,a->planner_active?0x65d7bb:0xffc05e);
 if(a->planner_veto_reason[0]){ROW(r4_reason_zh(a->planner_veto_reason),0xb7c9de);}
 if(a->scene_reason[0]){ROW(a->scene_reason,0xb7c9de);}
 snprintf(b,sizeof(b),"路线：%s · %u点",v->navigation.route_complete?"已观测到目标":v->navigation.route_partial?"已观测局部通道":v->has_goal?"规划或探索中":"未设置目标",v->navigation.path_count);ROW(b,0xffa100);
 snprintf(b,sizeof(b),"预测：%s",predicted?(projection.commanded?"当前轮速条件预测0.8秒":"预览，尚未执行"):"等待有效动作");ROW(b,0x20d6de);
 if(!pose_valid)ROW("定位恢复中，保留最后地图和目标",0xffc05e);
 if(v->has_goal&&u->target_observation_pending)ROW("目标区域待观测，正在尝试路线",0xffc05e);
 if(v->navigation.recovery_target_m>0){snprintf(b,sizeof(b),"本次后退 %.2f/%.2fm",v->navigation.recovery_distance_m,v->navigation.recovery_target_m);ROW(b,0x65d7bb);}
 snprintf(b,sizeof(b),"上次停车：%s",s5_reason_zh(v->last_stop_detail));ROW(b,0xffc05e);
 if(v->last_stop_detail[0]&&strcmp(v->last_stop_detail,"NONE"))ROW(v->last_stop_detail,0xb7c9de);
 if(h>=640)for(unsigned i=0;i<3;i++){
  const char*name=i==0?"前":i==1?"左":"右";
  if(feedback_fresh&&(v->feedback.sonar_valid&(1u<<i))&&v->feedback.sonar_mm[i]>0)snprintf(b,sizeof(b),"超声%s %umm",name,v->feedback.sonar_mm[i]);
  else snprintf(b,sizeof(b),"超声%s —",name);ROW(b,0xb7c9de);
 }
 snprintf(b,sizeof(b),"位置 %.2f,%.2fm · 航向%.1f°",v->localization.pose.x_m,v->localization.pose.y_m,v->localization.pose.yaw_rad*57.2957795);ROW(b,0xffffff);
 snprintf(b,sizeof(b),"策略9 / 底盘5 · PWM上限70%%");ROW(b,0xb7c9de);
 text(c,16,h-26,u->message,0x65d7bb);
 #undef ROW
}

/* 显示坐标来自只读场景模块，绘制结果不写回导航目标或运动命令。 */
void live_nav_ui_draw_scene(live_nav_ui*u,uint32_t*p,int w,int h,const lab_real_runtime_view*v,const obj_scene*s){
 if(!u||!p||!v||!s||!s->published_us)return;canvas c={p,w,h};int px,py,xx,yy;const lab_localization_view*l=&v->localization;
 int matching=v->has_localization&&l->has_pose&&s->map_id&&s->map_id==l->map.map_id;uint64_t age_us=v->time_us>=s->published_us?v->time_us-s->published_us:0;int scene_fresh=age_us<=1500000;
 if(matching&&s->cloud_pose_valid){double cs=cos(s->cloud_pose[2]),sn=sin(s->cloud_pose[2]);for(unsigned i=0;i<s->cloud_count&&i<OBJ_SCENE_CLOUD_MAX;i++){double bx=s->cloud[i][0],by=s->cloud[i][1];if(s->cloud[i][2]+age_us/1000.>300)continue;if(hypot(bx,by)>(u->span_m>=24?12:8))continue;double x=s->cloud_pose[0]+cs*bx-sn*by,y=s->cloud_pose[1]+sn*bx+cs*by;if(point(u,x,y,&px,&py))box(c,px-1,py-1,3,3,s->cloud[i][2]>150?0x637a8b:0x8acaff);}}
 if(matching&&s->target_valid&&age_us<=10000000){
  int have=0,lx=0,ly=0;for(unsigned i=0;scene_fresh&&i<s->path_count&&i<OBJ_SCENE_PATH_MAX;i++){int yes=point(u,s->path[i][0],s->path[i][1],&px,&py);if(yes&&have)segment(c,lx,ly,px,py,0xf1a23d);have=yes;lx=px;ly=py;}
  if(point(u,s->target_x,s->target_y,&px,&py)){uint32_t color=(s->historical||!scene_fresh)?0x9aa5b2:s->confirmed?0x38df9b:0xffd057;box(c,px-5,py-5,11,11,color);text(c,px+9,py-12,(s->historical||!scene_fresh)?"历史目标":"实物表面",color);
   int have_ring=0,rx=0,ry=0;for(unsigned k=0;k<=32;k++){double a=k*6.283185307179586/32;int yes=point(u,s->target_x+cos(a)*s->error_m,s->target_y+sin(a)*s->error_m,&xx,&yy);if(yes&&have_ring)segment(c,rx,ry,xx,yy,color);have_ring=yes;rx=xx;ry=yy;}
  }
  if(s->dock_valid&&point(u,s->dock_x,s->dock_y,&px,&py)){segment(c,px-7,py,px+7,py,0xd68cff);segment(c,px,py-7,px,py+7,0xd68cff);text(c,px+8,py+5,"停靠点",0xd68cff);}
 }else if(matching&&s->bearing_valid){double a=l->pose.yaw_rad+s->bearing_rad;for(int k=-1;k<=1;k+=2)if(point(u,l->pose.x_m,l->pose.y_m,&px,&py)&&point(u,l->pose.x_m+3*cos(a+k*.12),l->pose.y_m+3*sin(a+k*.12),&xx,&yy))segment(c,px,py,xx,yy,0xffd057);}
 int x=32+side(u),y=h-206;if(x+140>w){x=22;y=h-132;}char b[384];
 box(c,x-5,y-6,w-x,182,0x142238);snprintf(b,sizeof(b),"实物 #%d：%s",s->track_id,s->target_valid?((s->historical||!scene_fresh)?"历史位置":"真实观测粗定位"):"仅方向 / 等待距离");text(c,x,y,b,0xffd057);y+=25;
 snprintf(b,sizeof(b),"来源 %s 误差约 %.2fm",s->source,s->error_m);text(c,x,y,b,0x92dfbd);y+=25;
 if(s->target_valid)snprintf(b,sizeof(b),"目标 %.2f, %.2f / 停靠 %.2f, %.2f",s->target_x,s->target_y,s->dock_x,s->dock_y);else snprintf(b,sizeof(b),"%s",s->message_zh);text(c,x,y,b,0xffffff);y+=25;
 snprintf(b,sizeof(b),"路线：%s / %s",s->path_count>=2&&scene_fresh?(!strcmp(s->preview_status,"HELD")?"路线保留，停车复核":!strcmp(s->preview_kind,"EXECUTING_REFERENCE_ROUTE")?"实际执行路线":!strcmp(s->preview_kind,"VALIDATED_ACTION")?"已检查短动作":"只读预览"):!scene_fresh?"数据过期，保留历史位置":"尚未生成",s->preview_reason);text(c,x,y,b,0xffc05e);y+=25;
 text(c,x,y,"安装位置近似；车头停靠间距0.20m",0xb7c9de);y+=25;text(c,x,y,u->span_m>=24?"全向点云12m / 360度；历史点淡出":"全向点云8m / 360度；历史点淡出",0x8acaff);
}

void live_nav_ui_draw_dual(live_nav_ui*u,uint32_t*p,int w,int h,const lab_real_runtime_view*v,const obj_scene*s,const object_dual_task*t,const obj_frame*f,double measured_w,int measured_valid){
 live_nav_ui_draw_scene(u,p,w,h,v,s);canvas c={p,w,h};char b[512];int y=130;int x=32+side(u);if(x+180>w){x=20;y=h-180;}
 box(c,x-5,y-4,w-x,346,0x142238);snprintf(b,sizeof(b),"实物模式：%s · 目标#%d",t->radar_only?"雷达跟随":t->mode==2?"跟随":"静止前往",t->track_id);text(c,x,y,b,0xffd057);text(c,x,y+24,t->message_zh,0xffffff);
 int usable=v->enabled&&lab_real_command_valid(&v->protected_output.command,v->time_us);double applied=usable?v->protected_output.command.angular_rad_s:0;
 snprintf(b,sizeof(b),"角速 请求%.2f / 保护后%.2f rad/s",t->requested_w,applied);text(c,x,y+48,b,0x92dfbd);
 if(measured_valid)snprintf(b,sizeof(b),"航向反馈角速估计 %.2f rad/s",measured_w);else snprintf(b,sizeof(b),"航向反馈角速：未知（无新航向样本）");text(c,x,y+72,b,0x92dfbd);
 const obj_detection*d=NULL;if(f)for(int i=0;i<f->count;i++)if(f->objects[i].track_id==t->track_id)d=&f->objects[i];
 if(t->radar_only)snprintf(b,sizeof(b),"%s",t->active?"雷达独立跟随：无需相机识别或选框":"雷达任务尚未启动或已暂停");else if(d&&d->reacquire_candidate_valid)snprintf(b,sizeof(b),"疑似原目标 候选#%d，正在对准核对",d->reacquire_candidate_track_id);else if(!t->active)snprintf(b,sizeof(b),"%s",t->prepared?"尚未开始 / 已到达保持":"用户已取消或未准备，需明确重新开始");else snprintf(b,sizeof(b),"目标关联：%s",d&&d->observed?"正在追踪原目标":"新检测尚未关联，使用记忆或搜索");text(c,x,y+96,b,0xffc05e);
 snprintf(b,sizeof(b),"底盘：%s · R8：%s",v->enabled?"已使能":v->pending_enable?"等待使能":"尚未使能",v->navigation.reason);text(c,x,y+120,b,0xffffff);
 if(t->radar_only)snprintf(b,sizeof(b),"移动点簇仅代表雷达目标，不判断物体类别");else if(d&&d->depth_time_us&&v->time_us>=d->depth_time_us){uint64_t age=(v->time_us-d->depth_time_us)/1000;snprintf(b,sizeof(b),"双目%llums：%s",(unsigned long long)age,s->stereo_prior_only?"仅历史参考，不否决新雷达":age<=1200?"按质量使用":age<=2000?"粗距关联参考":"已过期，使用雷达或位置记忆");}else snprintf(b,sizeof(b),"双目暂无有效距离，使用雷达或位置记忆");text(c,x,y+144,b,0xb7c9de);
 if(d&&d->reacquire_candidate_reason[0])text(c,x,y+168,d->reacquire_candidate_reason,0xb7c9de);
 const object_target_observation_v2*m=&t->observation_v2;if(m->valid&&m->source_us<=v->time_us)snprintf(b,sizeof(b),"实测车头净距%.2fm · %llums · %s",m->surface_gap_m,(unsigned long long)((v->time_us-m->source_us)/1000),m->mount_verified?"安装已核对":"安装近似");else snprintf(b,sizeof(b),"当前无新实测净距；估计/记忆不是新测量");text(c,x,y+168,b,0xb7c9de);
 if(s->stereo_conflict)snprintf(b,sizeof(b),"当前雷达与双目冲突，采用新表面并复核");else if(s->lidar_valid&&s->lidar_source_us<=v->time_us)snprintf(b,sizeof(b),"原目标雷达轨迹：%llums / 已更新%u次",(unsigned long long)((v->time_us-s->lidar_source_us)/1000),s->lidar_updates);else snprintf(b,sizeof(b),"原目标雷达轨迹尚未建立，依据视觉方向");text(c,x,y+192,b,0xb7c9de);
 if(t->mode==OBJECT_MODE_FOLLOW){const char*why=t->radar_only?s->message_zh:!strcmp(s->follow_reason,"FOLLOW_WALL_EXCLUDED")?"已排除墙面，寻找目标表面":!strcmp(s->follow_reason,"FOLLOW_ASSOCIATION_CORRECTED")?"已纠正目标位置，保留原任务":!strcmp(s->follow_reason,"FOLLOW_SPARSE_TRACK")?"远处单点维持，未确认到达":!strcmp(s->follow_reason,"FOLLOW_MOVING_CANDIDATE")?"优先关联独立运动目标":!strcmp(s->follow_reason,"FOLLOW_STATIONARY_CANDIDATE")?"目标静止仍可跟随":!strcmp(s->follow_reason,"FOLLOW_RADAR_TRACK")?"同目标雷达持续更新":"等待新的目标表面，保留方向";
  snprintf(b,sizeof(b),"跟随：%s · 关联修订%u",why,s->association_revision);text(c,x,y+216,b,0xffc05e);
  snprintf(b,sizeof(b),"候选%u / 排除墙面%u · 运动证据%.0f%%",s->follow_candidates,s->follow_wall_rejected,100*s->follow_motion_score);text(c,x,y+240,b,0xb7c9de);
  if(t->directed_active)snprintf(b,sizeof(b),"持续定向转身：已转%.0f / 35度（按真实航向）",t->directed_progress_rad*180/3.14159265358979323846);else snprintf(b,sizeof(b),"%s",!strcmp(t->directed_status,"TIMEOUT")?"定向执行超时，等待新目标证据":t->active?"任务保留；新视觉/雷达优先，静止也跟随":"双击目标自动启动；停车/取消优先");text(c,x,y+264,b,0xffc05e);
  snprintf(b,sizeof(b),"实点 前%u/左%u/右%u/后%u · 重点10/8/8/4m",s->obstacle_front_points,s->obstacle_left_points,s->obstacle_right_points,s->obstacle_rear_points);text(c,x,y+288,b,0x8acaff);
  if(s->follow_sparse)snprintf(b,sizeof(b),"本扫描仅%u点：弱支持，旧点未冒充新测距",s->follow_support_points);else snprintf(b,sizeof(b),"全向原始障碍回波%u点；墙筛选不删除障碍",s->obstacle_raw_scan_points);text(c,x,y+312,b,0xb7c9de);
 }else text(c,x,y+216,t->active?"任务保留；关闭手机窗口不会取消":"双击目标自动启动；停车/取消优先",0xffc05e);
}
