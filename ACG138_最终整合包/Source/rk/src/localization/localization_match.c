#include "localization_internal.h"
#include "localization_quality.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159265358979323846

double wl_wrap(double a) { return remainder(a,2*PI); }
w3_loc_pose wl_compose(w3_loc_pose a,w3_loc_pose b) {
    double c=cos(a.yaw_rad),s=sin(a.yaw_rad);
    return (w3_loc_pose){a.x_m+c*b.x_m-s*b.y_m,a.y_m+s*b.x_m+c*b.y_m,wl_wrap(a.yaw_rad+b.yaw_rad)};
}
w3_loc_pose wl_relative(w3_loc_pose a,w3_loc_pose b) {
    double c=cos(a.yaw_rad),s=sin(a.yaw_rad),x=b.x_m-a.x_m,y=b.y_m-a.y_m;
    return (w3_loc_pose){c*x+s*y,-s*x+c*y,wl_wrap(b.yaw_rad-a.yaw_rad)};
}
/* 计算占用格中心的精确可分离平方欧氏距离场，未知格保持未知，不清除或修改原地图。
 */
static void edt_buffers(double*f,double*d,double*z,int*v,uint32_t n) {
    int k=0;v[0]=0;z[0]=-1e30;z[1]=1e30;
    for(uint32_t q=1;q<n;++q) {
        double cut;
        for(;;) {
            int at=v[k];cut=((f[q]+(double)q*q)-(f[at]+(double)at*at))/(2*((double)q-at));
            if(cut>z[k]||k==0)break;
            --k;
        }
        ++k;v[k]=(int)q;z[k]=cut;z[k+1]=1e30;
    }
    k=0;
    for(uint32_t q=0;q<n;++q) {
        while(z[k+1]<q)++k;
        double delta=(double)q-v[k];d[q]=delta*delta+f[v[k]];
    }
}
void wl_build_field(w3_localizer *s) {
    uint32_t w=s->map.width,h=s->map.height;
    for(uint32_t y=0;y<h;++y) {
        for(uint32_t x=0;x<w;++x)s->line_in[x]=s->cells[(size_t)y*w+x]>=(int)s->c.occupancy_threshold?0:1e12;
        edt_buffers(s->line_in,s->line_out,s->breaks,s->sites,w);for(uint32_t x=0;x<w;++x)s->distance[(size_t)y*w+x]=(float)s->line_out[x];
    }
    for(uint32_t x=0;x<w;++x) {
        for(uint32_t y=0;y<h;++y)s->line_in[y]=s->distance[(size_t)y*w+x];
        edt_buffers(s->line_in,s->line_out,s->breaks,s->sites,h);for(uint32_t y=0;y<h;++y)s->distance[(size_t)y*w+x]=(float)(sqrt(s->line_out[y])*s->map.resolution_m);
    }
}
static void free_reference(wl_reference_job*j){if(!j)return;free(j->cells);free(j->distance);free(j->line_in);free(j->line_out);free(j->breaks);free(j->sites);free(j);}
void wl_reference_discard(w3_localizer*s){if(s){free_reference(s->reference_job);s->reference_job=NULL;}}
lab_status wl_queue_reference(w3_localizer*s,const w3_loc_map*m){
 if(!s||!m||!m->cells||m->map_id!=s->map.map_id||m->resolution_m!=s->map.resolution_m||m->width<4||m->height<4||m->width>W3_LOC_GRID_SIDE_MAX||m->height>W3_LOC_GRID_SIDE_MAX||!isfinite(m->origin_x_m)||!isfinite(m->origin_y_m))return LAB_ERR_ARGUMENT;
 if(s->reference_job)return LAB_AGAIN;size_t n=(size_t)m->width*m->height;uint32_t line=m->width>m->height?m->width:m->height;
 wl_reference_job*j=calloc(1,sizeof(*j));if(!j)return LAB_ERR_NOMEM;j->cells=malloc(n);j->distance=malloc(n*sizeof(float));j->line_in=malloc(line*sizeof(double));j->line_out=malloc(line*sizeof(double));j->breaks=malloc((line+1)*sizeof(double));j->sites=malloc(line*sizeof(int));
 if(!j->cells||!j->distance||!j->line_in||!j->line_out||!j->breaks||!j->sites){free_reference(j);return LAB_ERR_NOMEM;}
 memcpy(j->cells,m->cells,n);j->map=*m;j->map.cells=j->cells;j->allocated_bytes=sizeof(*j)+n*(1+sizeof(float))+(3*(size_t)line+1)*sizeof(double)+(size_t)line*sizeof(int);s->reference_job=j;return LAB_OK;
}
int wl_reference_work(w3_localizer*s,unsigned budget){
 if(!s||!s->reference_job||!budget)return 0;wl_reference_job*j=s->reference_job;uint32_t w=j->map.width,h=j->map.height;
 while(budget--&&j->phase<2){
  if(!j->phase){uint32_t y=j->line;for(uint32_t x=0;x<w;x++)j->line_in[x]=j->cells[(size_t)y*w+x]>=(int)s->c.occupancy_threshold?0:1e12;edt_buffers(j->line_in,j->line_out,j->breaks,j->sites,w);for(uint32_t x=0;x<w;x++)j->distance[(size_t)y*w+x]=(float)j->line_out[x];if(++j->line==h){j->line=0;j->phase=1;}}
  else{uint32_t x=j->line;for(uint32_t y=0;y<h;y++)j->line_in[y]=j->distance[(size_t)y*w+x];edt_buffers(j->line_in,j->line_out,j->breaks,j->sites,h);for(uint32_t y=0;y<h;y++)j->distance[(size_t)y*w+x]=(float)(sqrt(j->line_out[y])*j->map.resolution_m);if(++j->line==w){j->line=0;j->phase=2;}}
 }
 if(j->phase<2)return 0;
 free(s->cells);free(s->distance);free(s->line_in);free(s->line_out);free(s->breaks);free(s->sites);
 s->cells=j->cells;s->distance=j->distance;s->line_in=j->line_in;s->line_out=j->line_out;s->breaks=j->breaks;s->sites=j->sites;s->map=j->map;s->map.cells=s->cells;s->result.map_version=j->map.map_version;
 size_t n=(size_t)w*h,line=w>h?w:h;s->result.allocated_bytes=sizeof(*s)+n*(1+sizeof(float))+sizeof(*s->pending)+(3*line+1)*sizeof(double)+line*sizeof(int);
 free(j);s->reference_job=NULL;return 1;
}
static int field(const w3_localizer *s,double wx,double wy,double *d,double *gx,double *gy) {
    double qx=(wx-s->map.origin_x_m)/s->map.resolution_m,qy=(wy-s->map.origin_y_m)/s->map.resolution_m;
    if(qx<0||qy<0||qx>=s->map.width||qy>=s->map.height)return 0;
    uint32_t cx=(uint32_t)qx,cy=(uint32_t)qy;
    (void)cx;(void)cy;
    /* 实际端点可能落在墙旁未知格，只有占用参考约束匹配，未知区域不清除也不视为可通行，远处未知无近邻参考。

 */
    double x=qx-.5,y=qy-.5;int ix=(int)floor(x),iy=(int)floor(y);
    if(ix<0||iy<0||ix+1>=(int)s->map.width||iy+1>=(int)s->map.height)return 0;
    double u=x-ix,v=y-iy;size_t at=(size_t)iy*s->map.width+(unsigned)ix;
    double a=s->distance[at],b=s->distance[at+1],c=s->distance[at+s->map.width],e=s->distance[at+s->map.width+1];
    *d=(1-v)*((1-u)*a+u*b)+v*((1-u)*c+u*e);
    *gx=((1-v)*(b-a)+v*(e-c))/s->map.resolution_m;
    *gy=((1-u)*(c-a)+u*(e-b))/s->map.resolution_m;
    return 1;
}
static double robust_loss(double d,double k){return d<=k?d*d:2*k*d-k*k;}
static double cost(const w3_localizer *s,const wl_point *p,uint32_t n,w3_loc_pose pose) {
    double c=cos(pose.yaw_rad),v=sin(pose.yaw_rad),sum=0,cap=robust_loss(s->c.correspondence_m,s->c.huber_m);
    for(uint32_t i=0;i<n;++i) {
        double d,gx,gy;
        if(!field(s,pose.x_m+c*p[i].x-v*p[i].y,pose.y_m+v*p[i].x+c*p[i].y,&d,&gx,&gy))sum+=cap;
        else sum+=robust_loss(fmin(s->c.correspondence_m,d),s->c.huber_m);
    }
    return sum/n;
}
double wl_recovery_score(const w3_localizer*s,const wl_point*p,uint32_t n,w3_loc_pose pose){return s&&p&&n?cost(s,p,n,pose):1e9;}
static uint32_t equations(const w3_localizer *s,const wl_point *p,uint32_t n,w3_loc_pose pose,
                         double h[3][3],double g[3],double *sum) {
    double c=cos(pose.yaw_rad),v=sin(pose.yaw_rad);uint32_t used=0;*sum=0;
    for(uint32_t i=0;i<n;++i) {
        double x=c*p[i].x-v*p[i].y,y=v*p[i].x+c*p[i].y,d,gx,gy;
        if(!field(s,pose.x_m+x,pose.y_m+y,&d,&gx,&gy)||d>s->c.correspondence_m)continue;
        double j[3]={gx,gy,-gx*y+gy*x},weight=d>s->c.huber_m?s->c.huber_m/d:1;
        for(int a=0;a<3;++a) {
            g[a]+=weight*j[a]*d;
            for(int b=0;b<3;++b)h[a][b]+=weight*j[a]*j[b];
        }
        *sum+=d*d;++used;
    }
    return used;
}
static int solve(double h[3][3],double g[3],double step[3]) {
    double a[3][4],scale=0;
    for(int i=0;i<3;++i) {
        for(int j=0;j<3;++j)a[i][j]=h[i][j];
        a[i][3]=-g[i];scale=fmax(scale,h[i][i]);
    }
    if(scale<=0)return 0;
    for(int i=0;i<3;++i) {
        int pivot=i;for(int j=i+1;j<3;++j)if(fabs(a[j][i])>fabs(a[pivot][i]))pivot=j;
        if(fabs(a[pivot][i])<scale*1e-7)return 0;
        for(int j=i;j<4;++j) {double tmp=a[i][j];a[i][j]=a[pivot][j];a[pivot][j]=tmp;}
        double den=a[i][i];for(int j=i;j<4;++j)a[i][j]/=den;
        for(int k=0;k<3;++k)if(k!=i) {
            double factor=a[k][i];for(int j=i;j<4;++j)a[k][j]-=factor*a[i][j];
        }
    }
    for(int i=0;i<3;++i)step[i]=a[i][3];return 1;
}
static void coverage(w3_localizer*s,const wl_point*p,uint32_t n,w3_loc_pose pose){
 w3_loc_result*r=&s->result;r->inside_points=r->outside_points=r->reference_points=r->close_points=r->sector_count=0;r->robust_rmse_m=1e9;
 double c=cos(pose.yaw_rad),v=sin(pose.yaw_rad),sum=0,weights=0;unsigned sectors=0;
 for(uint32_t i=0;i<n;i++){double x=pose.x_m+c*p[i].x-v*p[i].y,y=pose.y_m+v*p[i].x+c*p[i].y,d,gx,gy;
  double qx=(x-s->map.origin_x_m)/s->map.resolution_m,qy=(y-s->map.origin_y_m)/s->map.resolution_m;
  if(qx<0||qy<0||qx>=s->map.width||qy>=s->map.height){++r->outside_points;continue;}++r->inside_points;
  if(!field(s,x,y,&d,&gx,&gy)||d>s->c.correspondence_m)continue;++r->reference_points;if(d<=R8_LOC_CLOSE_DISTANCE_M)++r->close_points;
  double weight=d>s->c.huber_m?s->c.huber_m/d:1;sum+=weight*d*d;weights+=weight;
  unsigned sector=(unsigned)floor((atan2(p[i].y,p[i].x)+PI)*12/(2*PI));sectors|=1u<<(sector%12);
 }
 for(unsigned b=0;b<12;b++)r->sector_count+=(sectors>>b)&1u;r->coverage=n?(double)r->inside_points/n:0;r->robust_rmse_m=weights?sqrt(sum/weights):1e9;
}
int wl_match(w3_localizer *s,const wl_point *p,uint32_t n,w3_loc_pose initial,int first,
             w3_loc_pose *out,double *rmse,double *ratio,uint32_t *matches,int *degenerate) {
    double radius=first?s->c.initial_search_m:R8_LOC_TRACKING_SEARCH_M,angle=first?s->c.initial_search_rad:R8_LOC_TRACKING_SEARCH_RAD;
    double xy_step=first?.05:.04,yaw_step=first?.05:.03;
    int nx=(int)ceil(radius/xy_step),na=(int)ceil(angle/yaw_step);
    w3_loc_pose best=initial;double best_cost=cost(s,p,n,best);
    *degenerate=0;*rmse=1e9;*ratio=0;*matches=0;
    coverage(s,p,n,best);
    for(int a=-na;a<=na;++a)for(int x=-nx;x<=nx;++x)for(int y=-nx;y<=nx;++y) {
        w3_loc_pose q={initial.x_m+x*radius/fmax(1,nx),initial.y_m+y*radius/fmax(1,nx),wl_wrap(initial.yaw_rad+a*angle/fmax(1,na))};
        double score=cost(s,p,n,q);
        if(score<best_cost-1e-12) {best=q;best_cost=score;}
    }
    for(uint32_t iteration=0;iteration<s->c.iterations;++iteration) {
        double h[3][3]={{0}},g[3]={0},sum,step[3];uint32_t used=equations(s,p,n,best,h,g,&sum);
        if(used<s->c.min_points){coverage(s,p,n,best);fprintf(stderr,"MATCH_STEP_FAIL stage=iteration_points n=%u used=%u minimum=%u cost=%.6f inside=%u outside=%u references=%u\n",n,used,s->c.min_points,best_cost,s->result.inside_points,s->result.outside_points,s->result.reference_points);return 0;}
        double trace=h[0][0]+h[1][1];
        if(trace<=0||h[0][0]*h[1][1]-h[0][1]*h[0][1]<1e-4*trace*trace||!solve(h,g,step)) {
            coverage(s,p,n,best);fprintf(stderr,"MATCH_STEP_FAIL stage=degenerate n=%u used=%u hxx=%.6f hyy=%.6f hxy=%.6f\n",n,used,h[0][0],h[1][1],h[0][1]);*degenerate=1;return 0;
        }
        double distance=hypot(step[0],step[1]),scale=distance>.04?.04/distance:1;
        if(fabs(step[2])*scale>.025)scale=.025/fabs(step[2]);
        int accepted=0;
        for(int line=0;line<8;++line) {
            w3_loc_pose q={best.x_m+scale*step[0],best.y_m+scale*step[1],wl_wrap(best.yaw_rad+scale*step[2])};
            double score=cost(s,p,n,q);
            if(score<=best_cost+1e-12) {best=q;best_cost=score;accepted=1;break;}
            scale*=.5;
        }
        if(!accepted||(distance*scale<.0001&&fabs(step[2])*scale<.00005))break;
    }
    double h[3][3]={{0}},g[3]={0},sum;
    *matches=equations(s,p,n,best,h,g,&sum);*ratio=(double)*matches/n;
    *rmse=*matches?sqrt(sum/ *matches):1e9;*out=best;
    double weighted=0,weight_sum=0;unsigned close=0;double c=cos(best.yaw_rad),v=sin(best.yaw_rad);
    for(uint32_t i=0;i<n;++i){double d,gx,gy,x=c*p[i].x-v*p[i].y,y=v*p[i].x+c*p[i].y;if(!field(s,best.x_m+x,best.y_m+y,&d,&gx,&gy)||d>s->c.correspondence_m)continue;double w=d>s->c.huber_m?s->c.huber_m/d:1;weighted+=w*d*d;weight_sum+=w;if(d<=R8_LOC_CLOSE_DISTANCE_M)++close;}
    double robust=weight_sum?sqrt(weighted/weight_sum):1e9;
    coverage(s,p,n,best);
    double weight=wl_quality_weight(n,*matches,close,robust,*rmse,s->c.min_match_ratio,s->c.max_rmse_m,s->c.min_points);
    int ok=isfinite(best.x_m)&&isfinite(best.y_m)&&isfinite(best.yaw_rad)&&isfinite(robust)&&isfinite(*rmse)&&*matches>=s->c.min_points;
    s->correction_weight=ok?weight:0;
    if(ok){out->x_m=initial.x_m+weight*(best.x_m-initial.x_m);out->y_m=initial.y_m+weight*(best.y_m-initial.y_m);out->yaw_rad=wl_wrap(initial.yaw_rad+weight*wl_wrap(best.yaw_rad-initial.yaw_rad));}
    if(!ok||weight<1||s->result.accepted%50==0)fprintf(stderr,"MATCH_QUALITY_WEIGHT confidence=%.4f action=%s n=%u used=%u close=%u ratio=%.4f ratio_reference=%.3f close_matched_reference=%.3f close_all_reference=%.3f robust_rmse_m=%.4f robust_reference_m=%.3f raw_rmse_m=%.4f raw_reference_m=%.3f raw_correction_m=%.4f applied_correction_m=%.4f message_zh=匹配质量仅影响修正权重_真实里程计继续推算\n",weight,ok?(weight<1?"WEIGHTED_CORRECTION":"FULL_CORRECTION"):"NO_MATH_SOLUTION",n,*matches,close,*ratio,s->c.min_match_ratio,(double)R8_LOC_MIN_CLOSE_MATCHED_RATIO,(double)R8_LOC_MIN_CLOSE_ALL_RATIO,robust,s->c.max_rmse_m,*rmse,(double)R8_LOC_MAX_RAW_RMSE_M,hypot(best.x_m-initial.x_m,best.y_m-initial.y_m),ok?hypot(out->x_m-initial.x_m,out->y_m-initial.y_m):0);
    if(ok&&s->result.accepted%50==0)fprintf(stderr,"MATCH_QUALITY n=%u matched=%u close=%u ratio=%.4f raw_rmse_m=%.4f robust_rmse_m=%.4f\n",n,*matches,close,*ratio,*rmse,robust);
    if(!ok||s->result.accepted%50==0)fprintf(stderr,"MATCH_COVERAGE input=%u inside=%u outside=%u reference=%u close=%u sectors=%u coverage=%.4f map=%ux%u origin=%.3f,%.3f\n",n,s->result.inside_points,s->result.outside_points,s->result.reference_points,s->result.close_points,s->result.sector_count,s->result.coverage,s->map.width,s->map.height,s->map.origin_x_m,s->map.origin_y_m);
    if(!ok){unsigned bins[5]={0},ranges[3]={0};double sums[3]={0},weighted=0,weight_sum=0;double c=cos(best.yaw_rad),v=sin(best.yaw_rad);
        for(uint32_t i=0;i<n;++i){double d,gx,gy,x=c*p[i].x-v*p[i].y,y=v*p[i].x+c*p[i].y;if(!field(s,best.x_m+x,best.y_m+y,&d,&gx,&gy)||d>s->c.correspondence_m){++bins[4];continue;}unsigned b=d<.04?0:d<.07?1:d<.15?2:3;++bins[b];double range=hypot(p[i].x,p[i].y);unsigned k=range<2?0:range<5?1:2;++ranges[k];sums[k]+=d*d;double w=d>s->c.huber_m?s->c.huber_m/d:1;weighted+=w*d*d;weight_sum+=w;}
        fprintf(stderr,"MATCH_STEP_FAIL stage=threshold n=%u used=%u ratio=%.4f min_ratio=%.4f rmse_m=%.4f robust_rmse_m=%.4f max_rmse_m=%.4f dx_m=%.4f dyaw=%.4f cost=%.6f bins=%u,%u,%u,%u,%u ranges=%u,%u,%u ranges_rmse=%.4f,%.4f,%.4f\n",n,*matches,*ratio,s->c.min_match_ratio,*rmse,weight_sum?sqrt(weighted/weight_sum):0,s->c.max_rmse_m,hypot(best.x_m-initial.x_m,best.y_m-initial.y_m),fabs(wl_wrap(best.yaw_rad-initial.yaw_rad)),best_cost,bins[0],bins[1],bins[2],bins[3],bins[4],ranges[0],ranges[1],ranges[2],ranges[0]?sqrt(sums[0]/ranges[0]):0,ranges[1]?sqrt(sums[1]/ranges[1]):0,ranges[2]?sqrt(sums[2]/ranges[2]):0);}
    return ok;
}
