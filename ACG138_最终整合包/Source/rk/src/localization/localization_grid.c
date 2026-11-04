#include "localization_grid_private.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

lab_status wl_grid_open(wl_grid **out,uint32_t w,uint32_t h,double res,double ox,double oy,uint64_t id) {
    if(!out)return LAB_ERR_ARGUMENT;*out=NULL;
    if(w<4||h<4||w>W3_LOC_GRID_SIDE_MAX||h>W3_LOC_GRID_SIDE_MAX)return LAB_ERR_LIMIT;
    if(!id||!isfinite(res)||res<.01||res>1||!isfinite(ox)||!isfinite(oy)||fabs(ox)>1e6||fabs(oy)>1e6)return LAB_ERR_ARGUMENT;
    wl_grid *g=calloc(1,sizeof(*g));if(!g)return LAB_ERR_NOMEM;size_t n=(size_t)w*h;
    g->cells=malloc(n);g->scores=calloc(n,sizeof(int16_t));g->marks=malloc(n);g->hit_epoch=calloc(n,sizeof(uint32_t));g->hit_count=calloc(n,1);
    if(!g->cells||!g->scores||!g->marks||!g->hit_epoch||!g->hit_count){wl_grid_close(g);return LAB_ERR_NOMEM;}
    memset(g->cells,-1,n);g->view=(w3_loc_map){w,h,res,ox,oy,id,1,g->cells};
    for(int i=-1024;i<=1024;++i)g->probability[i+1024]=(int8_t)lround(100/(1+exp(-(double)i/256)));
    g->geometry_version=1;g->allocated_bytes=sizeof(*g)+n*(3+sizeof(int16_t)+sizeof(uint32_t));*out=g;return LAB_OK;
}
static unsigned grow_cells(double value){if(value<=0)return 0;double n=ceil(value);if(n>1000)n=1000;return ((unsigned)n+63u)/64u*64u;}
lab_status wl_grid_expand(wl_grid*g,double minx,double miny,double maxx,double maxy){
 if(!g||!isfinite(minx)||!isfinite(miny)||!isfinite(maxx)||!isfinite(maxy)||minx>maxx||miny>maxy||fabs(minx)>1e6||fabs(miny)>1e6||fabs(maxx)>1e6||fabs(maxy)>1e6)return LAB_ERR_ARGUMENT;
 const w3_loc_map*m=&g->view;double r=m->resolution_m;
 unsigned left=grow_cells((m->origin_x_m-minx)/r),right=grow_cells((maxx-(m->origin_x_m+m->width*r))/r),down=grow_cells((m->origin_y_m-miny)/r),up=grow_cells((maxy-(m->origin_y_m+m->height*r))/r);
 if(!left&&!right&&!down&&!up)return LAB_OK;
 /* 地图扩展不丢弃旧栅格，各维上限1000，以兼容规划器、文件格式和手机绘制器。
 */
 unsigned ax=m->width<1000?1000-m->width:0,ay=m->height<1000?1000-m->height:0;
 if(left+right>ax||down+up>ay)++g->growth_limited;
 if(left>ax)left=ax;ax-=left;if(right>ax)right=ax;
 if(down>ay)down=ay;ay-=down;if(up>ay)up=ay;
 if(!left&&!right&&!down&&!up)return LAB_OK;
 unsigned w=m->width+left+right,h=m->height+down+up;size_t n=(size_t)w*h;
 int8_t*cells=malloc(n);int16_t*scores=calloc(n,sizeof(*scores));uint8_t*marks=calloc(n,1),*count=calloc(n,1);uint32_t*epochs=calloc(n,sizeof(*epochs));
 if(!cells||!scores||!marks||!count||!epochs){free(cells);free(scores);free(marks);free(count);free(epochs);return LAB_ERR_NOMEM;}
 memset(cells,-1,n);
 for(unsigned y=0;y<m->height;y++){size_t a=(size_t)y*m->width,b=(size_t)(y+down)*w+left;
  memcpy(cells+b,g->cells+a,m->width);memcpy(scores+b,g->scores+a,m->width*sizeof(*scores));memcpy(count+b,g->hit_count+a,m->width);memcpy(epochs+b,g->hit_epoch+a,m->width*sizeof(*epochs));
 }
 free(g->cells);free(g->scores);free(g->marks);free(g->hit_count);free(g->hit_epoch);g->cells=cells;g->scores=scores;g->marks=marks;g->hit_count=count;g->hit_epoch=epochs;
 g->view.width=w;g->view.height=h;g->view.origin_x_m-=left*r;g->view.origin_y_m-=down*r;g->view.cells=cells;++g->view.map_version;++g->geometry_version;++g->expansions;g->allocated_bytes=sizeof(*g)+n*(3+sizeof(int16_t)+sizeof(uint32_t));return LAB_OK;
}
static int clip_axis(double p,double q,double *lo,double *hi) {
    if(fabs(p)<1e-15)return q>=0;
    double t=q/p;
    if(p<0){if(t>*hi)return 0;if(t>*lo)*lo=t;}
    else{if(t<*lo)return 0;if(t<*hi)*hi=t;}
    return 1;
}
static int cell(double x,double origin,double res,uint32_t count) {
    int v=(int)floor((x-origin)/res);if(v<0)v=0;if(v>=(int)count)v=(int)count-1;return v;
}
static void ray(wl_grid *g,double x0,double y0,double x1,double y1) {
    const w3_loc_map *m=&g->view;double xmax=m->origin_x_m+m->width*m->resolution_m,ymax=m->origin_y_m+m->height*m->resolution_m;
    int hit=x1>=m->origin_x_m&&x1<xmax&&y1>=m->origin_y_m&&y1<ymax;
    if(!hit)++g->outside_endpoints;
    double dx=x1-x0,dy=y1-y0,lo=0,hi=1;
    double safe_xmax=nextafter(xmax,m->origin_x_m),safe_ymax=nextafter(ymax,m->origin_y_m);
    if(!clip_axis(-dx,x0-m->origin_x_m,&lo,&hi)||!clip_axis(dx,safe_xmax-x0,&lo,&hi)||
       !clip_axis(-dy,y0-m->origin_y_m,&lo,&hi)||!clip_axis(dy,safe_ymax-y0,&lo,&hi))return;
    int ax=cell(x0+lo*dx,m->origin_x_m,m->resolution_m,m->width),ay=cell(y0+lo*dy,m->origin_y_m,m->resolution_m,m->height);
    int bx=cell(x0+hi*dx,m->origin_x_m,m->resolution_m,m->width),by=cell(y0+hi*dy,m->origin_y_m,m->resolution_m,m->height);
    int sx=ax<bx?1:-1,sy=ay<by?1:-1,ddx=abs(bx-ax),ddy=-abs(by-ay),error=ddx+ddy;
    for(;;) {
        size_t at=(size_t)ay*m->width+(unsigned)ax;
        if(ax==bx&&ay==by){g->marks[at]|=(uint8_t)(hit?2:1);break;}
        g->marks[at]|=1;int twice=2*error;
        if(twice>=ddy){error+=ddy;ax+=sx;}
        if(twice<=ddx){error+=ddx;ay+=sy;}
    }
}
lab_status wl_grid_insert(wl_grid *g,const w3_loc_config *c,w3_loc_pose pose,const struct scan_frame *scan) {
    if(!g||!c||!scan||!scan->count||scan->count>SCAN_MAX_POINTS||!scan_geometry_valid(scan)||!isfinite(pose.x_m)||!isfinite(pose.y_m)||!isfinite(pose.yaw_rad))return LAB_ERR_ARGUMENT;
    if(g->have_scan){if(scan->session!=g->last_scan_session)return LAB_ERR_CONTEXT;uint32_t diff=scan->sequence-g->last_scan_sequence;if(!diff||diff>=UINT32_C(0x80000000))return LAB_ERR_TIME;}
    if(!c->points_per_scan||c->points_per_scan>W3_LOC_POINTS_MAX||!c->min_points||c->min_points>c->points_per_scan||
       !isfinite(c->mount_x_m)||!isfinite(c->mount_y_m)||!isfinite(c->mount_yaw_rad)||!isfinite(c->min_range_m)||
       !isfinite(c->max_range_m)||c->min_range_m<=0||c->max_range_m<=c->min_range_m)return LAB_ERR_ARGUMENT;
    uint32_t valid=0,cap=c->points_per_scan;
    for(uint32_t i=0;i<cap&&i<scan->count;++i) {
        uint32_t j=scan->count>cap?(uint32_t)((uint64_t)i*scan->count/cap):i;
        valid+=scan_return_in_range(scan,j,c->min_range_m,c->max_range_m);
    }
    if(valid<c->min_points)return LAB_ERR_UNAVAILABLE;
    double ca=cos(pose.yaw_rad),sa=sin(pose.yaw_rad),x=pose.x_m+ca*c->mount_x_m-sa*c->mount_y_m,y=pose.y_m+sa*c->mount_x_m+ca*c->mount_y_m;
    double minx=x-.5,maxx=x+.5,miny=y-.5,maxy=y+.5;
    for(uint32_t i=0;i<cap&&i<scan->count;++i){uint32_t j=scan->count>cap?(uint32_t)((uint64_t)i*scan->count/cap):i;const struct scan_point*p=&scan->points[j];double d=p->distance_q2/4000.0;if(!scan_return_in_range(scan,j,c->min_range_m,c->max_range_m))continue;double a=pose.yaw_rad+c->mount_yaw_rad-p->angle_q14*(6.2831853071795864769/65536.0),px=x+d*cos(a),py=y+d*sin(a);minx=fmin(minx,px-.1);maxx=fmax(maxx,px+.1);miny=fmin(miny,py-.1);maxy=fmax(maxy,py+.1);}
    lab_status grown=wl_grid_expand(g,minx,miny,maxx,maxy);if(grown!=LAB_OK)return grown;
    size_t n=(size_t)g->view.width*g->view.height;memset(g->marks,0,n);
    for(uint32_t i=0;i<cap&&i<scan->count;++i) {
        uint32_t j=scan->count>cap?(uint32_t)((uint64_t)i*scan->count/cap):i;const struct scan_point *p=&scan->points[j];double d=p->distance_q2/4000.0;
        if(!scan_return_in_range(scan,j,c->min_range_m,c->max_range_m)){++g->invalid_points;continue;}
        double a=pose.yaw_rad+c->mount_yaw_rad-p->angle_q14*(6.2831853071795864769/65536.0);
        double qx,qy;scan_ray_origin(scan,j,&qx,&qy);
        double ma=pose.yaw_rad+c->mount_yaw_rad,mc=cos(ma),ms=sin(ma);
        ray(g,x+mc*qx-ms*qy,y+ms*qx+mc*qy,x+d*cos(a),y+d*sin(a));
    }
    for(size_t i=0;i<n;++i)if(g->marks[i]) {
        int score=g->scores[i];
        if(g->marks[i]&2){uint32_t epoch=(uint32_t)(g->integrated+1),gap=epoch-g->hit_epoch[i];
          if(g->hit_epoch[i]&&gap<=3){if(g->hit_count[i]<2)++g->hit_count[i];}else g->hit_count[i]=1;g->hit_epoch[i]=epoch;
          score+=128;
        }else{g->hit_count[i]=0;g->hit_epoch[i]=0;score-=score>0?512:64;}
        if(score>1024)score=1024;if(score< -1024)score=-1024;g->scores[i]=(int16_t)score;
        g->cells[i]=g->probability[score+1024];
        if((g->marks[i]&2)&&g->hit_count[i]<2&&g->cells[i]>=65)g->cells[i]=64;
    }
    g->last_scan_session=scan->session;g->last_scan_sequence=scan->sequence;g->have_scan=1;++g->integrated;++g->view.map_version;return LAB_OK;
}
void wl_grid_close(wl_grid *g) {if(g){free(g->cells);free(g->scores);free(g->marks);free(g->hit_count);free(g->hit_epoch);free(g);}}
