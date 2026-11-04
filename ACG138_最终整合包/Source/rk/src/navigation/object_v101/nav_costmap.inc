/* 代价图读取现有近场观测，位置单位为世界坐标米。写入值0可通行、254占用、255未知；固定状态加小于7KiB临时EDT栈。 */
#ifndef ACG_S6_NAV_COSTMAP_INC
#define ACG_S6_NAV_COSTMAP_INC
#include "s6_planning_types.h"
#include <math.h>
#include <string.h>
#include <float.h>
#include <limits.h>

#define S6_CM_RESOLUTION_M 0.02
#define S6_CM_UNKNOWN 255u
#define S6_CM_LETHAL 254u
#define S6_CM_STATIC_BIT 1u
#define S6_CM_OBSTACLE_BIT 2u
#define S6_CM_INFLATION_BIT 4u
#define S6_CM_EDT_INF 1000000000u

static int cm_index(const cm_state *cm,double x,double y,size_t *index) {
 if(!cm||!cm->initialized||!isfinite(x)||!isfinite(y))return 0;
 double gx=(x-cm->origin_x)/S6_CM_RESOLUTION_M;
 double gy=(y-cm->origin_y)/S6_CM_RESOLUTION_M;
 if(gx<0||gy<0||gx>=S6_CM_SIDE||gy>=S6_CM_SIDE)return 0;
 *index=(size_t)floor(gy)*S6_CM_SIDE+(size_t)floor(gx);return 1;
}

static uint8_t cm_merged_occupancy(const cm_state *cm,size_t i) {
 if(cm->static_layer[i]==S6_CM_LETHAL||cm->obstacle_layer[i]==S6_CM_LETHAL)return S6_CM_LETHAL;
 if(cm->static_layer[i]==0||cm->obstacle_layer[i]==0)return 0;
 return S6_CM_UNKNOWN;
}

static void cm_begin(cm_state *cm,s6_pose pose,uint64_t source_us,double lethal,double inflation) {
 if(!cm)return;
 memset(cm,0,sizeof(*cm));
 memset(cm->static_layer,S6_CM_UNKNOWN,sizeof(cm->static_layer));
 memset(cm->obstacle_layer,S6_CM_UNKNOWN,sizeof(cm->obstacle_layer));
 memset(cm->cost,S6_CM_UNKNOWN,sizeof(cm->cost));
 for(size_t i=0;i<S6_CM_CELLS;i++){cm->cells[i].occupancy=S6_CM_UNKNOWN;cm->distance_m[i]=INFINITY;}
 cm->origin_x=pose.x-2.0;cm->origin_y=pose.y-2.0;cm->source_us=source_us;
 cm->lethal_m=(isfinite(lethal)&&lethal>=0)?lethal:0.18;
 cm->inflation_m=(isfinite(inflation)&&inflation>cm->lethal_m)?inflation:0.25;
 if(cm->inflation_m<=cm->lethal_m){cm->lethal_m=0.18;cm->inflation_m=0.25;}
 cm->initialized=isfinite(pose.x)&&isfinite(pose.y)&&isfinite(pose.yaw);
}

static int cm_write_layer(cm_state *cm,double x,double y,uint8_t occupancy,uint64_t obs_us,unsigned layer) {
 size_t i;
 if((occupancy!=0&&occupancy!=S6_CM_LETHAL&&occupancy!=S6_CM_UNKNOWN)||!cm_index(cm,x,y,&i))return 0;
 uint8_t *view=layer==S6_CM_STATIC_BIT?cm->static_layer:cm->obstacle_layer;
 /* 未知写入保留已有证据；明确可通行观测可清除动态障碍。 */

 if(occupancy!=S6_CM_UNKNOWN||view[i]==S6_CM_UNKNOWN)view[i]=occupancy;
 cm->cells[i].last_obs_us=obs_us; /* S6写入路径，不在此执行旧占用衰减。 */
 cm->cells[i].layer_bits|=(uint8_t)layer;
 cm->cells[i].occupancy=cm_merged_occupancy(cm,i);return 1;
}

static int cm_write_static(cm_state *cm,double x,double y,uint8_t occupancy,uint64_t obs_us) {
 return cm_write_layer(cm,x,y,occupancy,obs_us,S6_CM_STATIC_BIT);
}
static int cm_write_obstacle(cm_state *cm,double x,double y,uint8_t occupancy,uint64_t obs_us) {
 return cm_write_layer(cm,x,y,occupancy,obs_us,S6_CM_OBSTACLE_BIT);
}

/* 传感器0前、1左、2右，虚拟原点位于333×320mm车体边缘。带符号偏置以raw+offset形成车体净距，调用方提供已验证样本真实时间；不接收零值提示、不刻画自由射线、不刷新source_us。横向占用带不证明中间栅格状态。



 */
static int cm_project_sonar(cm_state *cm,s6_pose pose,unsigned sensor,double raw_m,
                            double offset_m,double margin_m,uint64_t source_us) {
 if(!cm||!cm->initialized||sensor>2||!source_us||!isfinite(raw_m)||raw_m<=0||
    !isfinite(offset_m)||!isfinite(margin_m)||margin_m<0||
    !isfinite(pose.x)||!isfinite(pose.y)||!isfinite(pose.yaw))return 0;
 double clearance=fmax(0.0,raw_m+offset_m);
 double bx=sensor==0?0.1665+clearance:0.0;
 double by=sensor==1?0.160+clearance:(sensor==2?-0.160-clearance:0.0);
 double c=cos(pose.yaw),s=sin(pose.yaw);
 /* 提示带裁剪到局部窗口，避免错误超大边缘配置造成无界循环。
 */
 double margin=fmin(margin_m,6.0);
 unsigned steps=(unsigned)ceil(2*margin/(S6_CM_RESOLUTION_M*0.5));
 int written=0;
 for(unsigned k=0;k<=steps;k++){
  double transverse=steps?(-margin+2*margin*k/steps):0;
  double x=bx+(sensor==0?0:transverse),y=by+(sensor==0?transverse:0);
  written+=cm_write_obstacle(cm,pose.x+c*x-s*y,pose.y+s*x+c*y,S6_CM_LETHAL,source_us);
 }
 /* 无论带宽采样为奇数或偶数，都包含中心点。 */
 written+=cm_write_obstacle(cm,pose.x+c*bx-s*by,pose.y+s*bx+c*by,S6_CM_LETHAL,source_us);
 return written;
}

static uint8_t cm_distance_cost(const cm_state *cm,double d) {
 /* 浮点微量只补偿准确格心位置的表示误差。 */
 if(d<=cm->lethal_m+1e-8)return S6_CM_LETHAL;
 if(d>cm->inflation_m+1e-8)return 0;
 double scaled=253.0*(cm->inflation_m-d)/(cm->inflation_m-cm->lethal_m);
 int value=(int)ceil(scaled-1e-7);return (uint8_t)(value<1?1:(value>253?253:value));
}

static void cm_finish(cm_state *cm) {
 if(!cm||!cm->initialized)return;
 /* 精确可分离平方欧氏距离变换，不使用近似倒角距离。逐行求水平距离，再逐列用抛物线包络求最小dx²+dy²，工作量有界为O(200×200)。

 */
 for(unsigned y=0;y<S6_CM_SIDE;y++){
  int last=-100000;
  for(unsigned x=0;x<S6_CM_SIDE;x++){
   size_t i=(size_t)y*S6_CM_SIDE+x;
   cm->cells[i].occupancy=cm_merged_occupancy(cm,i);
   cm->cells[i].layer_bits&=(uint8_t)~S6_CM_INFLATION_BIT;
   if(cm->cells[i].occupancy==S6_CM_LETHAL)last=(int)x;
   int d=(int)x-last;cm->work[i]=last<0?S6_CM_EDT_INF:(uint32_t)(d*d);
  }
  last=100000;
  for(int x=(int)S6_CM_SIDE-1;x>=0;x--){
   size_t i=(size_t)y*S6_CM_SIDE+(unsigned)x;
   if(cm->cells[i].occupancy==S6_CM_LETHAL)last=x;
   int d=last-x;uint32_t d2=last==100000?S6_CM_EDT_INF:(uint32_t)(d*d);
   if(d2<cm->work[i])cm->work[i]=d2;
  }
 }
 for(unsigned x=0;x<S6_CM_SIDE;x++){
  uint32_t f[S6_CM_SIDE];int v[S6_CM_SIDE];double z[S6_CM_SIDE+1];int k=-1;
  for(unsigned q=0;q<S6_CM_SIDE;q++)f[q]=cm->work[(size_t)q*S6_CM_SIDE+x];
  for(unsigned q=0;q<S6_CM_SIDE;q++){
   if(f[q]==S6_CM_EDT_INF)continue;
   double intersection=-INFINITY;
   while(k>=0){int p=v[k];intersection=((double)f[q]+q*q-((double)f[p]+p*p))/(2.0*((double)q-p));if(intersection>z[k])break;k--;}
   v[++k]=(int)q;z[k]=k?intersection:-INFINITY;z[k+1]=INFINITY;
  }
  int winner=0;
  for(unsigned q=0;q<S6_CM_SIDE;q++){
   size_t i=(size_t)q*S6_CM_SIDE+x;
   if(k<0){cm->work[i]=S6_CM_EDT_INF;cm->distance_m[i]=INFINITY;}
   else{while(winner<k&&z[winner+1]<(double)q)winner++;int dy=(int)q-v[winner];
    cm->work[i]=f[v[winner]]+(uint32_t)(dy*dy);
    cm->distance_m[i]=(float)(sqrt((double)cm->work[i])*S6_CM_RESOLUTION_M);}
   uint8_t occupancy=cm->cells[i].occupancy;
   cm->cost[i]=occupancy==S6_CM_UNKNOWN?S6_CM_UNKNOWN:cm_distance_cost(cm,cm->distance_m[i]);
   if(cm->cost[i]>0&&cm->cost[i]!=S6_CM_UNKNOWN)cm->cells[i].layer_bits|=S6_CM_INFLATION_BIT;
  }
 }
}

static double cm_clearance_at(const cm_state *cm,double x,double y) {
 size_t i;if(!cm_index(cm,x,y,&i))return 0;
 double cx=cm->origin_x+((i%S6_CM_SIDE)+0.5)*S6_CM_RESOLUTION_M;
 double cy=cm->origin_y+((i/S6_CM_SIDE)+0.5)*S6_CM_RESOLUTION_M;
 /* 格心距离场精确，连续位置按三角不等式使用保守下界，准确格心不额外扣半格。
 */
 double offset=hypot(x-cx,y-cy);if(offset<1e-12)offset=0;
 return fmax(0.0,(double)cm->distance_m[i]-offset);
}

static uint8_t cm_cost_at(const cm_state *cm,double x,double y) {
 size_t i;if(!cm_index(cm,x,y,&i))return S6_CM_UNKNOWN;
 if(cm->cells[i].occupancy==S6_CM_UNKNOWN)return S6_CM_UNKNOWN;
 return cm_distance_cost(cm,cm_clearance_at(cm,x,y));
}

/* 场景矩形必须查原始占用层，不能把旧的圆形膨胀 254 当成实物点。
 * 未观测仍为 255，不能因场景放宽而改写为自由区域。 */
static uint8_t cm_occupancy_at(const cm_state *cm,double x,double y) {
 size_t i;if(!cm_index(cm,x,y,&i))return S6_CM_UNKNOWN;
 return cm->cells[i].occupancy;
}

static double cm_min_clearance_along(const cm_state *cm,const s6_point *points,uint32_t count) {
 if(!points||!count)return 0;double minimum=INFINITY;
 for(uint32_t i=0;i<count;i++){
  if(cm_cost_at(cm,points[i].x,points[i].y)==S6_CM_UNKNOWN)return 0;
  double d=cm_clearance_at(cm,points[i].x,points[i].y);if(d<minimum)minimum=d;
 }
 return minimum;
}

static uint64_t cm_age_ms(const cm_state *cm,uint64_t now_us) {
 if(!cm||!cm->initialized||!cm->source_us||now_us<cm->source_us)return UINT64_MAX;
 return (now_us-cm->source_us)/1000u;
}
#endif
