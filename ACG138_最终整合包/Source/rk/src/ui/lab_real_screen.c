#include "lab_real_screen.h"
#include "map_font_data.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
typedef struct screen {uint32_t *p;int w,h;} screen;
static void box(screen c,int x,int y,int w,int h,uint32_t color){int x1=x<0?0:x,y1=y<0?0:y,x2=x+w>c.w?c.w:x+w,y2=y+h>c.h?c.h:y+h;for(int j=y1;j<y2;++j)for(int i=x1;i<x2;++i)c.p[(size_t)j*c.w+i]=color;}
static const struct map_glyph *get_glyph(unsigned ch){const struct map_font *f=&map_fonts[0];size_t lo=0,hi=f->count;while(lo<hi){size_t m=(lo+hi)/2;if(f->glyphs[m].code<ch)lo=m+1;else hi=m;}return lo<f->count&&f->glyphs[lo].code==ch?f->glyphs+lo:NULL;}
static void text(screen c,int x,int y,const char *s,uint32_t color){int max=c.w-18;for(const unsigned char *p=(const unsigned char*)s;*p;++p){const struct map_glyph *g=get_glyph(*p);if(!g)continue;if(x+g->advance>max)break;
 for(int a=0;a<g->height;++a)for(int b=0;b<g->width;++b){int px=x+g->left+b,py=y+map_fonts[0].ascent+g->top+a;if(px<0||px>=c.w||py<0||py>=c.h)continue;unsigned k=map_font_pixels[g->offset+(size_t)a*g->width+b];uint32_t old=c.p[(size_t)py*c.w+px];unsigned r=(((color>>16)&255)*k+((old>>16)&255)*(255-k)+127)/255,gg=(((color>>8)&255)*k+((old>>8)&255)*(255-k)+127)/255,bb=((color&255)*k+(old&255)*(255-k)+127)/255;c.p[(size_t)py*c.w+px]=(r<<16)|(gg<<8)|bb;}x+=g->advance;}}

static void line(screen c,int ax,int ay,int bx,int by,uint32_t color){
 int dx=abs(bx-ax),dy=-abs(by-ay),sx=ax<bx?1:-1,sy=ay<by?1:-1,err=dx+dy;
 /* 调用方已将端点限制在视口内，循环工作量有界。 */
 for(int i=0;i<c.w+c.h+2;++i){box(c,ax,ay,2,2,color);if(ax==bx&&ay==by)break;int e=2*err;if(e>=dy){err+=dy;ax+=sx;}if(e<=dx){err+=dx;ay+=sy;}}
}
static int map_point(const lab_real_grid_view *m,double scale,int ox,int oy,int mw,int mh,double x,double y,int *px,int *py){
 if(!isfinite(x)||!isfinite(y)||x<m->origin_x_m||y<m->origin_y_m||x>=m->origin_x_m+m->width*m->resolution_m||y>=m->origin_y_m+m->height*m->resolution_m)return 0;
 *px=ox+(int)((x-m->origin_x_m)/m->resolution_m*scale);*py=oy+mh-1-(int)((y-m->origin_y_m)/m->resolution_m*scale);return *px>=ox&&*px<ox+mw&&*py>=oy&&*py<oy+mh;
}
void lab_real_screen_draw(uint32_t *p,int w,int h,const lab_real_runtime_view *v){
 if(!p||!v||w<320||h<240)return;screen c={p,w,h};box(c,0,0,w,h,0x101a29);box(c,0,0,w,80,0x1b2d45);
 text(c,18,14,"ACG138 / REAL INTEGRATION",0xeff5ff);char buf[200];snprintf(buf,sizeof(buf),"%s | %s | OUTPUT %s",lab_real_source_text(v->context.source),(v->mode==LAB_REAL_SHADOW&&v->localization.map.cells&&!v->localization.map.readonly)?"MAPPING+SHADOW":lab_real_mode_text(v->mode),v->experimental_timing?"DISABLED / EXPERIMENTAL TIMING":v->enabled?"ENABLED":"DISABLED");text(c,18,45,buf,v->enabled?0xffc05e:0x65d7bb);
 int y=92;snprintf(buf,sizeof(buf),"Status: %s",v->reason);text(c,18,y,buf,0xffc05e);y+=30;
 snprintf(buf,sizeof(buf),"Source: %s",v->sensor_health.reason[0]?v->sensor_health.reason:"WAITING_FOR_DATA");text(c,18,y,buf,0xb7c9de);y+=30;
 snprintf(buf,sizeof(buf),"Localizer %u: %s",v->localization.state,v->localization.reason[0]?v->localization.reason:"NOT_ACTIVE");text(c,18,y,buf,0xb7c9de);y+=30;
 snprintf(buf,sizeof(buf),"Accepted %llu | rejected %llu | aliases %llu | motor IO %llu",(unsigned long long)v->accepted,(unsigned long long)v->rejected,(unsigned long long)v->coalesced_pose_aliases,(unsigned long long)v->hardware_calls);text(c,18,y,buf,0xb7c9de);y+=30;
 snprintf(buf,sizeof(buf),"Scan age %.0f ms | odometry age %.0f ms | timing %s",v->has_scan&&v->scan_time_us<=v->time_us?(double)(v->time_us-v->scan_time_us)/1000:-1,v->has_odometry&&v->odometry_time_us<=v->time_us?(double)(v->time_us-v->odometry_time_us)/1000:-1,(v->scan_uncertainty_us==UINT64_MAX||v->odometry_uncertainty_us==UINT64_MAX)?"UNKNOWN ERROR":v->experimental_timing?"ASSUMED / UNVERIFIED":"BOUNDED");text(c,18,y,buf,0xb7c9de);y+=30;
 if(v->has_localization)snprintf(buf,sizeof(buf),"Pose %.3f, %.3f m | yaw %.1f deg | distance %.3f m | %s",v->localization.pose.x_m,v->localization.pose.y_m,v->localization.pose.yaw_rad*57.29577951308232,v->localization.pose.distance_m,v->localization.state==LAB_REAL_LOC_TRACKING?"TRACKING":"STALE / WAITING");else if(v->has_odometry)snprintf(buf,sizeof(buf),"Raw odometry %.3f, %.3f m | NOT a localized map pose",v->raw_odometry.x_m,v->raw_odometry.y_m);else snprintf(buf,sizeof(buf),"Pose unavailable");text(c,18,y,buf,0xeff5ff);y+=30;
 snprintf(buf,sizeof(buf),"Nav %s | %s | SHADOW %.3f m/s %.3f rad/s",v->navigation.phase_text,v->navigation.reason,v->request.linear_m_s,v->request.angular_rad_s);text(c,18,y,buf,0x65d7bb);y+=36;
 int side=h-y-42;if(side>w-36)side=w-36;if(side<60)return;int x=(w-side)/2;box(c,x,y,side,side,0x33445b);
 const lab_real_grid_view *m=&v->localization.map;
 if(lab_real_grid_valid(m)){
  double scale=fmin((double)side/m->width,(double)side/m->height);int mw=(int)(scale*m->width),mh=(int)(scale*m->height),ox=x+(side-mw)/2,oy=y+(side-mh)/2;
  for(int j=0;j<mh;++j)for(int i=0;i<mw;++i){unsigned col=(unsigned)((double)i/scale),row=m->height-1-(unsigned)((double)j/scale);int value=m->cells[(size_t)row*m->width+col];unsigned k=value<0?100u:(unsigned)(245-value*2);p[(size_t)(oy+j)*w+ox+i]=value<0?0x45536a:(k<<16)|(k<<8)|k;}
  int px=0,py=0,lx=0,ly=0,have=0;
  for(uint32_t i=0;m->trace&&i<m->trace_count&&i<LAB_TRACE_MAX;++i){int yes=map_point(m,scale,ox,oy,mw,mh,m->trace[i].x_m,m->trace[i].y_m,&px,&py);if(yes&&have)line(c,lx,ly,px,py,0x56a5ff);have=yes;lx=px;ly=py;}
  have=0;for(uint32_t i=0;v->navigation.path&&v->navigation.map_version==m->map_version&&i<v->navigation.path_count&&i<LAB_REAL_PATH_MAX;++i){int yes=map_point(m,scale,ox,oy,mw,mh,v->navigation.path[i].x_m,v->navigation.path[i].y_m,&px,&py);if(yes&&have)line(c,lx,ly,px,py,0xffc05e);have=yes;lx=px;ly=py;}
  if(v->has_localization&&map_point(m,scale,ox,oy,mw,mh,v->localization.pose.x_m,v->localization.pose.y_m,&px,&py)){box(c,px-3,py-3,7,7,0x55ddaa);int hx=px+(int)(12*cos(v->localization.pose.yaw_rad)),hy=py-(int)(12*sin(v->localization.pose.yaw_rad));line(c,px,py,hx,hy,0x55ddaa);}
 }else if(v->latest_scan){
  /* 原始局部扫描不等于累计地图或已定位地图。 */
  text(c,x+8,y+8,"RAW SCAN / local view / no map pose",0xffc05e);double k=side/12.0;int cx=x+side/2,cy=y+side/2;box(c,cx-2,cy-2,5,5,0x55ddaa);
  for(unsigned i=0;i<v->latest_scan->count&&i<SCAN_MAX_POINTS;++i){const struct scan_point *q=&v->latest_scan->points[i];if(!q->distance_q2||!q->quality)continue;double r=q->distance_q2/4000.0,a=-q->angle_q14*(3.14159265358979323846/32768.0),xx=r*cos(a)*k,yy=r*sin(a)*k;if(fabs(xx)>=side/2-3||fabs(yy)>=side/2-3)continue;box(c,cx+(int)xx,cy-(int)yy,2,2,0x65d7bb);}
 }else text(c,x+12,y+16,"NO MAP / waiting for input",0xffc05e);
 text(c,18,h-30,"Software only. Actual wheel RPM not supplied by dry-run backend.",0xb7c9de);
}
