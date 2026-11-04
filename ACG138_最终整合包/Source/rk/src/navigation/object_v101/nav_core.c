#include "nav_private.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <limits.h>
#define PI 3.14159265358979323846
struct nav_engine {
    nav_config c; lab_context context; nav_goal goal; nav_command cmd;
    nav_state state; nav_reason reason,blocking_reason;
    uint32_t w,h,n,path_n,segment,heap_n,have_goal,dirty,have_tick,waiting;
    double res,ox,oy,radius,last_distance;
    int8_t *cells; uint8_t *blocked,*closed; uint16_t *dist;
    double *cost; int32_t *parent,*heap_pos,*heap;
    nav_xy path[NAV_PATH_MAX],raw[NAV_PATH_MAX];
    uint64_t revision,plans,expanded,stops,goal_started,last_tick,last_plan,wait_started;
    size_t allocated;
    nav_phase phase;
    uint32_t reverse_attempts,clear_scans,scan_sequence,have_scan_sequence,still_samples;
    int turn_dir,episode,have_motion_pose;
    uint64_t phase_started,episode_started,motion_time,scan_time;
    double rear_clearance,backup_progress,backup_total,turn_total,turn_start_yaw;
    double measured_v,measured_w;
    nav_xy motion_pose,backup_start,episode_origin;
    double motion_yaw;
    struct {nav_xy p;uint64_t expires;} obstacles[32];
    struct {nav_xy p;uint64_t expires;} failures[8];
    unsigned obstacle_n,failure_next;
};
static double clamp(double x,double a,double b){return x<a?a:x>b?b:x;}
static double wrap(double a){return remainder(a,2*PI);}
static double approach(double x,double target,double delta){return x+clamp(target-x,-delta,delta);}
static int context_equal(lab_context a,lab_context b){
    return a.mode==b.mode&&a.clock_domain==b.clock_domain&&a.timeline_id==b.timeline_id&&a.origin_epoch==b.origin_epoch;
}
static int context_event(const nav_engine *n,const lab_event *e){return e&&lab_context_matches(&n->context,&e->meta);}
static int pose_valid(const lab_event *e){
    if(!e||e->type!=LAB_EVENT_POSE||e->meta.provider==LAB_PROVIDER_TRUTH||!lab_event_valid(e))return 0;
    return (e->meta.flags&(LAB_TIME_VALID|LAB_DATA_VALID))==(LAB_TIME_VALID|LAB_DATA_VALID)&&
      (e->data.pose.quality&LAB_POSE_VALID)&&!(e->data.pose.quality&(LAB_POSE_STALE|LAB_POSE_GAP|LAB_POSE_SLIP))&&
      isfinite(e->data.pose.x_m)&&isfinite(e->data.pose.y_m)&&isfinite(e->data.pose.yaw_rad);
}
void nav_config_default(nav_config *c){
    if(!c)return;memset(c,0,sizeof(*c));
    c->vehicle_width_m=.36;c->vehicle_length_m=.42;c->safety_margin_m=.04;
    c->max_speed_m_s=.18;c->max_yaw_rate_rad_s=1.0;c->acceleration_m_s2=.30;
    c->angular_acceleration_rad_s2=2.5;c->brake_m_s2=.45;
    c->lookahead_m=.35;c->goal_tolerance_m=.10;c->yaw_tolerance_rad=PI/18;
    c->max_pose_age_us=200000;c->max_scan_age_us=250000;c->command_valid_us=150000;
    c->replan_interval_us=250000;c->wait_timeout_us=3000000;c->goal_timeout_us=180000000;
    c->occupancy_threshold=65;c->max_expansions=150000;c->require_scan=1;
    c->lidar_to_rear_m=.21;c->enable_recovery=1;
}
static int config_valid(const nav_config *c){
    if(!c)return 0;
    const double a[]={c->vehicle_width_m,c->vehicle_length_m,c->safety_margin_m,c->max_speed_m_s,
      c->max_yaw_rate_rad_s,c->acceleration_m_s2,c->angular_acceleration_rad_s2,c->brake_m_s2,
      c->lookahead_m,c->goal_tolerance_m,c->yaw_tolerance_rad,c->mount_x_m,c->mount_y_m,c->mount_yaw_rad,c->lidar_to_rear_m};
    for(size_t i=0;i<sizeof(a)/sizeof(a[0]);++i)if(!isfinite(a[i]))return 0;
    return c->vehicle_width_m>0&&c->vehicle_width_m<=2&&c->vehicle_length_m>0&&c->vehicle_length_m<=2&&
      c->safety_margin_m>=0&&c->safety_margin_m<=.5&&c->max_speed_m_s>0&&c->max_speed_m_s<=1&&
      c->max_yaw_rate_rad_s>0&&c->max_yaw_rate_rad_s<=3&&c->acceleration_m_s2>0&&c->acceleration_m_s2<=5&&
      c->angular_acceleration_rad_s2>0&&c->angular_acceleration_rad_s2<=10&&c->brake_m_s2>0&&c->brake_m_s2<=5&&
      c->lookahead_m>=.05&&c->lookahead_m<=2&&c->goal_tolerance_m>=.02&&c->goal_tolerance_m<=.5&&
      c->yaw_tolerance_rad>=.01&&c->yaw_tolerance_rad<=PI&&c->max_pose_age_us>0&&c->max_pose_age_us<=1000000&&
      c->max_scan_age_us>0&&c->max_scan_age_us<=1000000&&c->command_valid_us>0&&c->command_valid_us<=500000&&
      c->replan_interval_us>0&&c->wait_timeout_us>=c->max_pose_age_us&&c->wait_timeout_us<=30000000&&
      c->goal_timeout_us>0&&c->goal_timeout_us<=600000000&&c->occupancy_threshold>0&&c->occupancy_threshold<=100&&
      c->max_expansions>0&&c->max_expansions<=NAV_GRID_MAX&&c->require_scan<=1&&
      fabs(c->mount_x_m)<=2&&fabs(c->mount_y_m)<=2&&c->enable_recovery<=1&&
      c->lidar_to_rear_m-c->mount_x_m>0&&c->lidar_to_rear_m-c->mount_x_m<c->vehicle_length_m;
}
lab_status nav_open(nav_engine **out,const nav_config *c,const lab_context *ctx){
    if(!out)return LAB_ERR_ARGUMENT;*out=NULL;
    if(!config_valid(c)||!ctx||!ctx->timeline_id||!ctx->origin_epoch||ctx->clock_domain!=LAB_CLOCK_SIMULATION||
       (ctx->mode!=LAB_SIM_IDEAL&&ctx->mode!=LAB_SIM_NOISY))return LAB_ERR_ARGUMENT;
    nav_engine *n=calloc(1,sizeof(*n));if(!n)return LAB_ERR_NOMEM;
    n->c=*c;n->context=*ctx;
    double rear=c->lidar_to_rear_m-c->mount_x_m;
    n->radius=hypot(c->vehicle_width_m/2,fmax(rear,c->vehicle_length_m-rear))+c->safety_margin_m;
    n->rear_clearance=-1;
    n->state=NAV_IDLE;n->reason=NAV_NO_GOAL;n->allocated=sizeof(*n);*out=n;return LAB_OK;
}
static void free_grid(nav_engine *n){
    free(n->cells);free(n->blocked);free(n->closed);free(n->dist);free(n->cost);free(n->parent);free(n->heap_pos);free(n->heap);
    n->cells=NULL;n->blocked=NULL;n->closed=NULL;n->dist=NULL;n->cost=NULL;n->parent=NULL;n->heap_pos=NULL;n->heap=NULL;
}
static int grid_allocate(nav_engine *n,uint32_t count){
    n->cells=malloc(count);n->blocked=malloc(count);n->closed=malloc(count);
    n->dist=malloc(count*sizeof(uint16_t));n->cost=malloc(count*sizeof(double));
    n->parent=malloc(count*sizeof(int32_t));n->heap_pos=malloc(count*sizeof(int32_t));n->heap=malloc(count*sizeof(int32_t));
    return n->cells&&n->blocked&&n->closed&&n->dist&&n->cost&&n->parent&&n->heap_pos&&n->heap;
}
/* 保守正方形切比雪夫膨胀，工作量O(格数)，内存有界；加入栅格对角余量，避免量化缩小圆形车体包络。
 */
static void inflate(nav_engine *n){
    const unsigned inf=32760;unsigned w=n->w,h=n->h;
    for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){
        unsigned k=y*w+x;int v=n->cells[k];
        unsigned edge=x+1;if(w-x<edge)edge=w-x;if(y+1<edge)edge=y+1;if(h-y<edge)edge=h-y;
        n->dist[k]=(uint16_t)((v<0||v>=(int)n->c.occupancy_threshold)?0:(edge<inf?edge:inf));
    }
    for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){
        unsigned k=y*w+x,d=n->dist[k];
        if(x&&n->dist[k-1]+1u<d)d=n->dist[k-1]+1u;
        if(y){for(int dx=-1;dx<=1;++dx){int xx=(int)x+dx;if(xx>=0&&xx<(int)w&&n->dist[k-w+dx]+1u<d)d=n->dist[k-w+dx]+1u;}}
        n->dist[k]=(uint16_t)d;
    }
    for(int y=(int)h-1;y>=0;--y)for(int x=(int)w-1;x>=0;--x){
        unsigned k=(unsigned)y*w+(unsigned)x,d=n->dist[k];
        if(x+1<(int)w&&n->dist[k+1]+1u<d)d=n->dist[k+1]+1u;
        if(y+1<(int)h){for(int dx=-1;dx<=1;++dx){int xx=x+dx;if(xx>=0&&xx<(int)w&&n->dist[k+w+dx]+1u<d)d=n->dist[k+w+dx]+1u;}}
        n->dist[k]=(uint16_t)d;
    }
    unsigned cells=(unsigned)ceil(n->radius/n->res+.7071067811865476);
    for(unsigned k=0;k<n->n;++k)n->blocked[k]=(uint8_t)(n->dist[k]<=cells);
}
lab_status nav_set_map(nav_engine *n,const lab_map_view *m,uint64_t rev){
    if(!n||!m||!m->cells||!rev)return LAB_ERR_ARGUMENT;
    if(!context_equal(n->context,m->context))return LAB_ERR_CONTEXT;
    if(rev<=n->revision)return LAB_ERR_TIME;
    if(!m->width||!m->height||m->width>NAV_SIDE_MAX||m->height>NAV_SIDE_MAX||(uint64_t)m->width*m->height>NAV_GRID_MAX)return LAB_ERR_LIMIT;
    if(!isfinite(m->resolution_m)||m->resolution_m<.01||m->resolution_m>1||!isfinite(m->origin_x_m)||
       !isfinite(m->origin_y_m)||fabs(m->origin_x_m)>1000000||fabs(m->origin_y_m)>1000000)return LAB_ERR_ARGUMENT;
    uint32_t count=m->width*m->height;for(uint32_t k=0;k<count;++k)if(m->cells[k]<-1||m->cells[k]>100)return LAB_ERR_FORMAT;
    if(n->n!=count){
        nav_engine tmp={0};if(!grid_allocate(&tmp,count)){free_grid(&tmp);return LAB_ERR_NOMEM;}
        free_grid(n);n->cells=tmp.cells;n->blocked=tmp.blocked;n->closed=tmp.closed;n->dist=tmp.dist;
        n->cost=tmp.cost;n->parent=tmp.parent;n->heap_pos=tmp.heap_pos;n->heap=tmp.heap;
    }
    n->n=count;n->w=m->width;n->h=m->height;n->res=m->resolution_m;n->ox=m->origin_x_m;n->oy=m->origin_y_m;
    memcpy(n->cells,m->cells,count);inflate(n);n->revision=rev;n->dirty=1;n->path_n=0;n->segment=0;
    if(n->have_goal&&n->state==NAV_UNREACHABLE){n->state=NAV_WAITING;n->reason=NAV_MAP_CHANGED;n->waiting=0;}
    n->allocated=sizeof(*n)+(size_t)count*(3+sizeof(uint16_t)+sizeof(double)+3*sizeof(int32_t));
    return LAB_OK;
}
static int index_at(const nav_engine *n,double x,double y){
    if(!isfinite(x)||!isfinite(y)||x<n->ox||y<n->oy||x>=n->ox+n->w*n->res||y>=n->oy+n->h*n->res)return -1;
    return (int)((unsigned)floor((y-n->oy)/n->res)*n->w+(unsigned)floor((x-n->ox)/n->res));
}
static int free_at(const nav_engine *n,double x,double y){int k=index_at(n,x,y);return k>=0&&!n->blocked[k];}
static nav_xy cell_xy(const nav_engine *n,int k){nav_xy p={n->ox+(k%(int)n->w+.5)*n->res,n->oy+(k/(int)n->w+.5)*n->res};return p;}
static int line_clear(const nav_engine *n,nav_xy a,nav_xy b){
    unsigned steps=(unsigned)ceil(hypot(b.x_m-a.x_m,b.y_m-a.y_m)/(n->res*.3))+1;
    for(unsigned i=0;i<=steps;++i){double t=(double)i/steps;if(!free_at(n,a.x_m+t*(b.x_m-a.x_m),a.y_m+t*(b.y_m-a.y_m)))return 0;}
    return 1;
}
lab_status nav_set_goal(nav_engine *n,const nav_goal *g,uint64_t now){
    if(!n||!g||!g->id||g->has_yaw>1||!isfinite(g->x_m)||!isfinite(g->y_m)||!isfinite(g->yaw_rad))return LAB_ERR_ARGUMENT;
    if(n->have_tick&&now<n->last_tick)return LAB_ERR_TIME;
    n->goal=*g;n->goal.yaw_rad=wrap(g->yaw_rad);n->goal_started=now;n->have_goal=1;n->dirty=1;n->path_n=0;
    n->phase=NAV_PHASE_TRACK;n->episode=0;n->reverse_attempts=0;n->backup_total=0;n->backup_progress=0;
    n->turn_total=0;n->clear_scans=0;n->turn_dir=0;memset(n->failures,0,sizeof(n->failures));
    n->state=NAV_WAITING;n->reason=NAV_MAP_CHANGED;n->blocking_reason=NAV_REASON_NONE;n->waiting=0;n->cmd.linear_m_s=0;n->cmd.angular_rad_s=0;return LAB_OK;
}
void nav_cancel(nav_engine *n,uint64_t now){
    if(!n)return;n->have_goal=0;n->path_n=0;n->waiting=0;n->state=NAV_CANCELLED;n->reason=NAV_USER_CANCEL;
    n->phase=NAV_PHASE_TRACK;n->episode=0;n->clear_scans=0;
    n->cmd.linear_m_s=0;n->cmd.angular_rad_s=0;n->cmd.time_us=now;n->cmd.expires_us=now;n->cmd.context=n->context;
}
static double heuristic(const nav_engine *n,int a,int b){
    double dx=abs(a%(int)n->w-b%(int)n->w),dy=abs(a/(int)n->w-b/(int)n->w);
    return fmax(dx,dy)+.4142135623730951*fmin(dx,dy);
}
static int heap_less(const nav_engine *n,int a,int b,int goal){
    double fa=n->cost[a]+heuristic(n,a,goal),fb=n->cost[b]+heuristic(n,b,goal);
    return fa<fb-1e-12||(fabs(fa-fb)<=1e-12&&a<b);
}
static void heap_swap(nav_engine *n,unsigned a,unsigned b){
    int t=n->heap[a];n->heap[a]=n->heap[b];n->heap[b]=t;n->heap_pos[n->heap[a]]=(int)a;n->heap_pos[n->heap[b]]=(int)b;
}
static void heap_up(nav_engine *n,unsigned p,int goal){while(p){unsigned q=(p-1)/2;if(!heap_less(n,n->heap[p],n->heap[q],goal))break;heap_swap(n,p,q);p=q;}}
static void heap_add(nav_engine *n,int k,int goal){
    if(n->heap_pos[k]>=0){heap_up(n,(unsigned)n->heap_pos[k],goal);return;}
    unsigned p=n->heap_n++;n->heap[p]=k;n->heap_pos[k]=(int)p;heap_up(n,p,goal);
}
static int heap_pop(nav_engine *n,int goal){
    int k=n->heap[0];n->heap_pos[k]=-1;--n->heap_n;if(!n->heap_n)return k;
    n->heap[0]=n->heap[n->heap_n];n->heap_pos[n->heap[0]]=0;unsigned p=0;
    for(;;){unsigned q=p*2+1;if(q>=n->heap_n)break;if(q+1<n->heap_n&&heap_less(n,n->heap[q+1],n->heap[q],goal))++q;
        if(!heap_less(n,n->heap[q],n->heap[p],goal))break;heap_swap(n,p,q);p=q;}
    return k;
}
static nav_reason plan(nav_engine *n,nav_xy start){
    n->path_n=0;n->segment=0;++n->plans;int s=index_at(n,start.x_m,start.y_m),g=index_at(n,n->goal.x_m,n->goal.y_m);
    if(g<0)return NAV_GOAL_OUTSIDE;if(n->blocked[g])return NAV_GOAL_BLOCKED;if(s<0||(n->blocked[s]&1))return NAV_START_BLOCKED;
    for(unsigned i=0;i<n->n;++i){n->cost[i]=DBL_MAX;n->parent[i]=-1;n->heap_pos[i]=-1;n->closed[i]=0;}
    n->heap_n=0;n->cost[s]=0;heap_add(n,s,g);unsigned expanded=0;int found=0;
    while(n->heap_n){
        int k=heap_pop(n,g);if(k==g){found=1;break;}n->closed[k]=1;
        if(++expanded>n->c.max_expansions){n->expanded+=expanded;return NAV_SEARCH_LIMIT;}
        int x=k%(int)n->w,y=k/(int)n->w;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
            int xx=x+dx,yy=y+dy;if((!dx&&!dy)||xx<0||yy<0||xx>=(int)n->w||yy>=(int)n->h)continue;
            int j=yy*(int)n->w+xx;if(n->blocked[j]||n->closed[j])continue;
            if(dx&&dy&&(n->blocked[y*(int)n->w+xx]||n->blocked[yy*(int)n->w+x]))continue;
            double step=dx&&dy?1.4142135623730951:1,extra=0;
            if(n->c.enable_recovery){
                double pad=ceil(n->radius/n->res+.7071067811865476);
                extra=1.5/fmax(1,n->dist[j]-pad);
                nav_xy at=cell_xy(n,j);
                for(unsigned z=0;z<8;++z)if(n->failures[z].expires>n->last_tick){
                    double d=hypot(at.x_m-n->failures[z].p.x_m,at.y_m-n->failures[z].p.y_m);
                    if(d<.45)extra+=3*(1-d/.45);
                }
            }
            double cost=n->cost[k]+step*(1+extra);
            if(cost<n->cost[j]){n->cost[j]=cost;n->parent[j]=k;heap_add(n,j,g);}
        }
    }
    n->expanded+=expanded;if(!found)return NAV_NO_PATH;
    unsigned count=0;for(int k=g;k>=0;k=n->parent[k]){if(count>=NAV_PATH_MAX-2)return NAV_PATH_LIMIT;n->raw[count++]=cell_xy(n,k);if(k==s)break;}
    for(unsigned i=0;i<count/2;++i){nav_xy t=n->raw[i];n->raw[i]=n->raw[count-1-i];n->raw[count-1-i]=t;}
    n->raw[0]=start;n->raw[count-1]=(nav_xy){n->goal.x_m,n->goal.y_m};
    if(count==1){n->path[0]=start;n->path[1]=n->raw[0];n->path_n=2;return NAV_REASON_NONE;}
    n->path[n->path_n++]=start;unsigned anchor=0;
    while(anchor<count-1){
        unsigned last=anchor+1;for(unsigned j=anchor+2;j<count;++j){if(!line_clear(n,n->raw[anchor],n->raw[j]))break;last=j;}
        n->path[n->path_n++]=n->raw[last];anchor=last;
    }
    return NAV_REASON_NONE;
}
static void command(nav_engine *n,uint64_t now,double v,double w,nav_command *out){
    n->cmd.linear_m_s=v;n->cmd.angular_rad_s=w;n->cmd.time_us=now;n->cmd.expires_us=now+n->c.command_valid_us;
    n->cmd.context=n->context;++n->cmd.sequence;*out=n->cmd;
}
static lab_status stop(nav_engine *n,uint64_t now,nav_state state,nav_reason reason,lab_status result,nav_command *out){
    if(n->cmd.linear_m_s!=0||n->cmd.angular_rad_s!=0)++n->stops;
    if(state==NAV_FAILED)n->phase=NAV_PHASE_BLOCKED;
    if(state==NAV_REACHED){n->phase=NAV_PHASE_TRACK;n->blocking_reason=NAV_REASON_NONE;}
    n->state=state;n->reason=reason;command(n,now,0,0,out);return result;
}
static lab_status wait_for(nav_engine *n,uint64_t now,nav_reason why,lab_status result,nav_command *out){
    n->blocking_reason=why;
    n->clear_scans=0;n->still_samples=0;
    if(!n->waiting){n->waiting=1;n->wait_started=now;}
    if(now>=n->wait_started&&now-n->wait_started>=n->c.wait_timeout_us)
        return stop(n,now,NAV_FAILED,NAV_WAIT_TIMEOUT,result,out);
    return stop(n,now,NAV_WAITING,why,result,out);
}
static nav_xy lookahead(nav_engine *n,nav_xy p){
    unsigned s=n->segment;
    while(s+2<n->path_n){nav_xy a=n->path[s],b=n->path[s+1];double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,den=dx*dx+dy*dy;
        double t=den?((p.x_m-a.x_m)*dx+(p.y_m-a.y_m)*dy)/den:1;
        if(hypot(p.x_m-b.x_m,p.y_m-b.y_m)<.035||(t>=1&&hypot(p.x_m-b.x_m,p.y_m-b.y_m)<.05))++s;else break;}
    n->segment=s;nav_xy a=n->path[s],b=n->path[s+1];double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,den=dx*dx+dy*dy;
    double t=den?clamp(((p.x_m-a.x_m)*dx+(p.y_m-a.y_m)*dy)/den,0,1):1;
    nav_xy q={a.x_m+t*dx,a.y_m+t*dy};double d=hypot(b.x_m-q.x_m,b.y_m-q.y_m);
    /* 先到达尖锐路径顶点，再切换线段，避免前视点跨过拐角。 */

    if(d>n->c.lookahead_m)return (nav_xy){q.x_m+(b.x_m-q.x_m)*n->c.lookahead_m/d,q.y_m+(b.y_m-q.y_m)*n->c.lookahead_m/d};
    return b;
}
static void advance(double *x,double *y,double *yaw,double v,double w,double dt){
    if(fabs(w)<1e-9){*x+=v*dt*cos(*yaw);*y+=v*dt*sin(*yaw);}
    else{*x+=v/w*(sin(*yaw+w*dt)-sin(*yaw));*y+=v/w*(cos(*yaw)-cos(*yaw+w*dt));}
    *yaw=wrap(*yaw+w*dt);
}
static nav_reason sweep_reason(const nav_engine *n,const lab_pose *pose,double v,double w,const nav_xy *points,unsigned count){
    double x=pose->x_m,y=pose->y_m,yaw=pose->yaw_rad;
    double horizon=fabs(v)/n->c.brake_m_s2+.20,dt=.025;unsigned steps=(unsigned)ceil(horizon/dt);
    for(unsigned k=0;k<=steps;++k){
        int k=index_at(n,x,y);if(k<0||(n->blocked[k]&1))return NAV_SWEEP_BLOCKED;
        for(unsigned j=0;j<count;++j)if(hypot(x-points[j].x_m,y-points[j].y_m)<=n->radius+.015)return NAV_OBSTACLE;
        advance(&x,&y,&yaw,v,w,dt);
    }
    return NAV_REASON_NONE;
}
#include "nav_recovery.inc"
lab_status nav_step(nav_engine *n,uint64_t now,const lab_event *p,const lab_event *scan,const lab_event *sp,nav_command *out){
    if(!n||!out)return LAB_ERR_ARGUMENT;
    if(now>UINT64_MAX-n->c.command_valid_us)return stop(n,0,NAV_FAILED,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    double dt=n->have_tick?(double)(now>=n->last_tick?now-n->last_tick:0)/1e6:.05;
    if(n->have_tick&&now<=n->last_tick)return stop(n,now,NAV_FAILED,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    n->last_tick=now;n->have_tick=1;
    if(!n->have_goal)return stop(n,now,n->state==NAV_CANCELLED?NAV_CANCELLED:NAV_IDLE,n->state==NAV_CANCELLED?NAV_USER_CANCEL:NAV_NO_GOAL,LAB_OK,out);
    if(n->state==NAV_FAILED||n->state==NAV_UNREACHABLE||n->state==NAV_REACHED)return stop(n,now,n->state,n->reason,LAB_OK,out);
    if(now<n->goal_started)return stop(n,now,NAV_FAILED,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    if(now-n->goal_started>=n->c.goal_timeout_us)return stop(n,now,NAV_FAILED,NAV_GOAL_TIMEOUT,LAB_OK,out);
    if(dt>.25)return wait_for(n,now,NAV_CLOCK_ERROR,LAB_ERR_TIME,out);
    if(!n->n)return wait_for(n,now,NAV_NO_MAP,LAB_AGAIN,out);
    if(!context_event(n,p))return wait_for(n,now,NAV_BAD_CONTEXT,LAB_ERR_CONTEXT,out);
    if(!pose_valid(p))return wait_for(n,now,NAV_INVALID_POSE,LAB_ERR_ARGUMENT,out);
    if(p->meta.time_us>now||p->meta.received_us>now||now-p->meta.time_us>n->c.max_pose_age_us)
        return wait_for(n,now,NAV_POSE_STALE,LAB_AGAIN,out);
    /* 栈使用限制128KiB，仅在匹配位姿下转换有效回波。 */
    nav_xy points[SCAN_MAX_POINTS];unsigned point_n=0;
    if(n->c.require_scan){
        if(!scan||!sp)return wait_for(n,now,NAV_SCAN_STALE,LAB_AGAIN,out);
        if(!context_event(n,scan)||!context_event(n,sp))return wait_for(n,now,NAV_BAD_CONTEXT,LAB_ERR_CONTEXT,out);
        if(scan->type!=LAB_EVENT_SCAN||scan->meta.provider==LAB_PROVIDER_TRUTH||!lab_event_valid(scan)||!pose_valid(sp)||
          (scan->meta.flags&(LAB_TIME_VALID|LAB_DATA_VALID))!=(LAB_TIME_VALID|LAB_DATA_VALID))
            return wait_for(n,now,NAV_SCAN_INVALID,LAB_ERR_ARGUMENT,out);
        if(scan->meta.time_us>now||scan->meta.received_us>now||sp->meta.received_us>now||scan->meta.time_us!=sp->meta.time_us||
           now-scan->meta.time_us>n->c.max_scan_age_us)return wait_for(n,now,NAV_SCAN_STALE,LAB_AGAIN,out);
        double ca=cos(sp->data.pose.yaw_rad),sa=sin(sp->data.pose.yaw_rad);
        for(unsigned j=0;j<scan->data.scan.count;++j){
            const struct scan_point *q=&scan->data.scan.points[j];if(!q->distance_q2||!q->quality)continue;
            /* LS01节点标志仅作诊断。 */
            double r=q->distance_q2/4000.0;if(r>30)continue;
            double a=-(double)q->angle_q14*(PI/32768.0)+n->c.mount_yaw_rad;
            double bx=n->c.mount_x_m+r*cos(a),by=n->c.mount_y_m+r*sin(a);
            points[point_n++]=(nav_xy){sp->data.pose.x_m+ca*bx-sa*by,sp->data.pose.y_m+sa*bx+ca*by};
        }
        if(!point_n)return wait_for(n,now,NAV_SCAN_INVALID,LAB_AGAIN,out);
    }
    int new_scan=scan&&(!n->have_scan_sequence||(scan->meta.time_us>n->scan_time&&
        (int32_t)(scan->meta.sequence-n->scan_sequence)>0));
    if(new_scan){n->have_scan_sequence=1;n->scan_sequence=scan->meta.sequence;n->scan_time=scan->meta.time_us;}
    observe_motion(n,p,now);rear_observed(n,&p->data.pose,points,point_n);
    dynamic_layer(n,&p->data.pose,points,point_n,now,new_scan);
    nav_xy pos={p->data.pose.x_m,p->data.pose.y_m};n->last_distance=hypot(n->goal.x_m-pos.x_m,n->goal.y_m-pos.y_m);
    if(n->dirty&&(n->phase==NAV_PHASE_TRACK||n->phase==NAV_PHASE_AVOID||n->phase==NAV_PHASE_RECOVER)){
        if(n->plans&&now-n->last_plan<n->c.replan_interval_us)return wait_for(n,now,NAV_MAP_CHANGED,LAB_AGAIN,out);
        n->last_plan=now;nav_reason why=plan(n,pos);n->dirty=0;
        if(why!=NAV_REASON_NONE){
            if(n->c.enable_recovery&&why==NAV_NO_PATH&&(n->obstacle_n||n->episode)){
                if(!n->episode)begin_episode(n,&p->data.pose,now,why);
                phase_to(n,NAV_PHASE_BACK_CHECK,now);
            }else return stop(n,now,NAV_UNREACHABLE,why,LAB_OK,out);
        }
    }
    if(n->c.enable_recovery&&n->phase!=NAV_PHASE_TRACK){
        nav_xy target=n->path_n>=2?lookahead(n,pos):(nav_xy){n->goal.x_m,n->goal.y_m};
        return recovery_step(n,now,&p->data.pose,target,dt,points,point_n,new_scan,out);
    }
    double v=0,w=0;
    if(n->last_distance<=n->c.goal_tolerance_m){
        double error=wrap(n->goal.yaw_rad-p->data.pose.yaw_rad);
        if(!n->goal.has_yaw||fabs(error)<=n->c.yaw_tolerance_rad){n->waiting=0;return stop(n,now,NAV_REACHED,NAV_GOAL_REACHED,LAB_OK,out);}
        w=clamp(2*error,-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);n->reason=NAV_FINAL_YAW;
    }else{
        if(n->path_n<2)return stop(n,now,NAV_UNREACHABLE,NAV_NO_PATH,LAB_OK,out);
        nav_xy target=lookahead(n,pos);double d=hypot(target.x_m-pos.x_m,target.y_m-pos.y_m);
        double alpha=wrap(atan2(target.y_m-pos.y_m,target.x_m-pos.x_m)-p->data.pose.yaw_rad);
        if(fabs(alpha)>.65){v=0;w=clamp(2*alpha,-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);}
        else{v=n->c.max_speed_m_s*clamp(1-fabs(alpha)/.9,.20,1);
            v=fmin(v,sqrt(2*n->c.brake_m_s2*fmax(n->last_distance-n->c.goal_tolerance_m,0)));
            if(n->segment+2<n->path_n)v=fmin(v,sqrt(2*n->c.brake_m_s2*fmax(d-.02,0)));
            w=clamp(2*v*sin(alpha)/fmax(d,.05),-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s);}
        n->reason=NAV_REASON_NONE;
    }
    v=approach(n->cmd.linear_m_s,v,n->c.acceleration_m_s2*dt);
    w=approach(n->cmd.angular_rad_s,w,n->c.angular_acceleration_rad_s2*dt);
    nav_reason unsafe=sweep_reason(n,&p->data.pose,v,w,points,point_n);
    if(unsafe!=NAV_REASON_NONE){
        if(!n->c.enable_recovery)return wait_for(n,now,unsafe,LAB_AGAIN,out);
        begin_episode(n,&p->data.pose,now,unsafe);
        nav_xy target=n->path_n>=2?lookahead(n,pos):(nav_xy){n->goal.x_m,n->goal.y_m};
        return recovery_step(n,now,&p->data.pose,target,dt,points,point_n,new_scan,out);
    }
    n->waiting=0;n->state=NAV_TRACKING;command(n,now,v,w,out);return LAB_OK;
}
lab_status nav_get_view(const nav_engine *n,nav_view *out){
    if(!n||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));out->state=n->state;out->reason=n->reason;out->blocking_reason=n->blocking_reason;out->goal=n->goal;
    out->command=n->cmd;out->map_revision=n->revision;out->plans=n->plans;out->expanded_nodes=n->expanded;out->emergency_stops=n->stops;
    out->goal_distance_m=n->last_distance;out->footprint_radius_m=n->radius;out->path_count=n->path_n;out->path=n->path;
    out->blocked=n->blocked;out->width=n->w;out->height=n->h;out->resolution_m=n->res;out->origin_x_m=n->ox;out->origin_y_m=n->oy;
    out->allocated_bytes=n->allocated;out->phase=n->phase;out->reverse_attempts=n->reverse_attempts;
    out->clear_scans=n->clear_scans;out->rear_clearance_m=n->rear_clearance;
    out->backup_progress_m=n->backup_progress;out->backup_total_m=n->backup_total;
    out->dynamic_count=n->obstacle_n;for(unsigned i=0;i<n->obstacle_n;++i)out->dynamic_obstacles[i]=n->obstacles[i].p;
    return LAB_OK;
}
const char *nav_state_text(nav_state s){const char *a[]={"IDLE","TRACKING","WAITING","REACHED","CANCELLED","UNREACHABLE","FAILED"};return s>=0&&s<=NAV_FAILED?a[s]:"UNKNOWN";}
const char *nav_reason_text(nav_reason s){
    const char *a[]={"NONE","NO_GOAL","GOAL_REACHED","USER_CANCEL","NO_MAP","BAD_CONTEXT","INVALID_POSE","POSE_STALE","SCAN_STALE",
      "SCAN_INVALID","GOAL_OUTSIDE","GOAL_BLOCKED","START_BLOCKED","NO_PATH","SEARCH_LIMIT","PATH_LIMIT","MAP_CHANGED","SWEEP_BLOCKED",
      "OBSTACLE","WAIT_TIMEOUT","GOAL_TIMEOUT","CLOCK_ERROR","INVALID_INPUT","FINAL_YAW",
      "LOCAL_AVOID","REPLANNING","TURN_RECOVERY","BACK_CHECK","BACKING","LOW_RECOVERY",
      "REAR_BLOCKED","REAR_UNOBSERVED","RECOVERY_EXHAUSTED"};
    return s>=0&&s<=NAV_RECOVERY_EXHAUSTED?a[s]:"UNKNOWN";
}
void nav_close(nav_engine *n){if(n){free_grid(n);free(n);}}
