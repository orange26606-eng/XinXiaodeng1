/* 场景几何计算，距离单位为米，前方 0.12m 从车身前缘量起。占用格按完整的 20mm 方格检查，15mm 实车间隙尚未验证。此层不访问传感器、电机或修改地图；全轨迹声束和转向扫掠由最终检查处理。 */
#ifndef ACG_R4_NAV_SCENE_GEOMETRY_INC
#define ACG_R4_NAV_SCENE_GEOMETRY_INC

/* 旋转矩形与轴对齐格子相交：四个分离轴，包含格子边界而非仅中心点。 */
static int s6_scene_cell_intersects(double px,double py,double cs,double sn,
                                    double cx,double cy,double hx,double hy,double cell_half){
 double dx=px-cx,dy=py-cy,bx=cs*dx+sn*dy,by=-sn*dx+cs*dy;
 double projection=cell_half*(fabs(cs)+fabs(sn));
 if(fabs(bx)>hx+projection+1e-9||fabs(by)>hy+projection+1e-9)return 0;
 if(fabs(dx)>hx*fabs(cs)+hy*fabs(sn)+cell_half+1e-9)return 0;
 if(fabs(dy)>hx*fabs(sn)+hy*fabs(cs)+cell_half+1e-9)return 0;
 return 1;
}

/* 返回值同 s6_probe_result：0 可行，1 占用，2 未知。minimum 为 lethal_m 加剩余矩形净空，与评分使用同一尺度。extra_margin 仅计入终端停车余量，车身尺寸只扣除一次。 */
static int s6_scene_geometry_probe(const cm_state *cm,const s6_config *c,const s6_point *p,
                                  double *minimum,double extra_margin){
 if(!cm||!c||!p||!minimum||!isfinite(extra_margin)||extra_margin<0)return 2;
 if(cm_occupancy_at(cm,p->x,p->y)==255)return 2;
 int straight=c->scene_straight_mode&&fabs(p->w)<=c->straight_w_threshold+1e-9;
 double side=c->body_width_m*.5+c->body_buffer_m+extra_margin;
 double rear=c->body_length_m*.5+c->body_buffer_m+extra_margin;
 double front=c->body_length_m*.5+(straight?fmax(c->body_buffer_m,c->front_lethal_from_edge_m):c->body_buffer_m)+extra_margin;
 double hx=.5*(front+rear),shift=.5*(front-rear),cs=cos(p->yaw),sn=sin(p->yaw);
 double cx=p->x+cs*shift,cy=p->y+sn*shift;
 double ax=hx*fabs(cs)+side*fabs(sn),ay=hx*fabs(sn)+side*fabs(cs);
 double minx=cx-ax,miny=cy-ay,maxx=cx+ax,maxy=cy+ay;
 double map_right=cm->origin_x+S6_CM_SIDE*.02,map_top=cm->origin_y+S6_CM_SIDE*.02;
 if(minx<cm->origin_x||miny<cm->origin_y||maxx>=map_right||maxy>=map_top)return 2;
 /* 转向样本回到旧圆形禁入条件，并额外保留完整矩形车角。 */
 double center_clear=cm_clearance_at(cm,p->x,p->y);
 if(!straight&&(cm_cost_at(cm,p->x,p->y)==254||center_clear<=c->lethal_m+extra_margin+1e-9))return 1;
 /* EDT 下界已经证明整个外接圆及格子半对角线都无占用时直接通过。
  * 这里只作等价加速，不把未知区域变成自由，不跳过窗口边界判断。 */
 double bound=hypot(fmax(front,rear),side)+.01414213562373095;
 if(center_clear>bound+.05){
  if(isfinite(center_clear))*minimum=fmin(*minimum,c->lethal_m+fmax(0,center_clear-bound));
  return 0;
 }
 double remaining=INFINITY;
 /* 再看外扩 50mm 的环，供评分计算净空；循环的空间上界由固定车身约束。
  * 不把环内 unknown 当自由，也不允许它清掉真实占用。未知授权仍走原终检。 */
 double band=.05;
 int x0=(int)floor((minx-band-cm->origin_x)/.02),x1=(int)floor((maxx+band-cm->origin_x)/.02);
 int y0=(int)floor((miny-band-cm->origin_y)/.02),y1=(int)floor((maxy+band-cm->origin_y)/.02);
 if(x0<0)x0=0;if(y0<0)y0=0;if(x1>=(int)S6_CM_SIDE)x1=S6_CM_SIDE-1;if(y1>=(int)S6_CM_SIDE)y1=S6_CM_SIDE-1;
 for(int y=y0;y<=y1;y++)for(int x=x0;x<=x1;x++){
  size_t i=(size_t)y*S6_CM_SIDE+(unsigned)x;if(cm->cells[i].occupancy!=254)continue;
  double wx=cm->origin_x+(x+.5)*.02,wy=cm->origin_y+(y+.5)*.02;
  if(s6_scene_cell_intersects(wx,wy,cs,sn,cx,cy,hx,side,.01))return 1;
  double dx=wx-cx,dy=wy-cy,bx=cs*dx+sn*dy,by=-sn*dx+cs*dy;
  double d=hypot(fmax(0,fabs(bx)-hx),fmax(0,fabs(by)-side))-.01414213562373095;
  remaining=fmin(remaining,fmax(0,d));
 }
 if(!straight)remaining=fmin(remaining,fmax(0,center_clear-c->lethal_m-extra_margin));
 if(isfinite(remaining))*minimum=fmin(*minimum,c->lethal_m+remaining);
 return 0;
}
#endif
