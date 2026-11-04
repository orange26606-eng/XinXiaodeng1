#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "passage.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "continuous_policy.h"
#ifdef LAB_REAL_POSIX_SOURCE
#include <time.h>
static uint64_t search_clock(void){struct timespec t;if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;return (uint64_t)t.tv_sec*1000000u+(uint64_t)t.tv_nsec/1000u;}
#endif
#define PI 3.14159265358979323846
#define NODE_LIMIT 1500000u
#define EXPANSION_LIMIT CONT_EXPANSIONS
typedef struct search {
 const passage_world*w;passage_pose start;passage_xy goal;double r,ox,oy,c,s;unsigned width,height,nodes,gx,gy;
 float*cost;int32_t*parent,*heap,*position;unsigned heap_n;unsigned char*closed,*pose_cache;
} search;
static const int dx[8]={1,1,0,-1,-1,-1,0,1},dy[8]={0,1,1,1,0,-1,-1,-1};
static passage_pose at(const search*s,int key){unsigned cell=(unsigned)key/8,h=(unsigned)key%8,x=cell%s->width,y=cell/s->width;double a=s->ox+x*s->r,b=s->oy+y*s->r;return (passage_pose){s->start.x+s->c*a-s->s*b,s->start.y+s->s*a+s->c*b,s->start.yaw+h*PI/4};}
static double heuristic(const search*s,int k){unsigned cell=(unsigned)k/8;return hypot((double)(cell%s->width)-(double)s->gx,(double)(cell/s->width)-(double)s->gy);}
static int less(const search*s,int a,int b){double x=s->cost[a]+heuristic(s,a),y=s->cost[b]+heuristic(s,b);return x<y-1e-8||(fabs(x-y)<=1e-8&&a<b);}
static void swap(search*s,unsigned a,unsigned b){int t=s->heap[a];s->heap[a]=s->heap[b];s->heap[b]=t;s->position[s->heap[a]]=(int)a;s->position[s->heap[b]]=(int)b;}
static void up(search*s,unsigned p){while(p){unsigned q=(p-1)/2;if(!less(s,s->heap[p],s->heap[q]))break;swap(s,p,q);p=q;}}
static void push(search*s,int k){if(s->position[k]>=0){up(s,(unsigned)s->position[k]);return;}unsigned p=s->heap_n++;s->heap[p]=k;s->position[k]=(int)p;up(s,p);}
static int pop(search*s){int result=s->heap[0];s->position[result]=-1;if(!--s->heap_n)return result;s->heap[0]=s->heap[s->heap_n];s->position[s->heap[0]]=0;unsigned p=0;for(;;){unsigned q=p*2+1;if(q>=s->heap_n)break;if(q+1<s->heap_n&&less(s,s->heap[q+1],s->heap[q]))q++;if(!less(s,s->heap[q],s->heap[p]))break;swap(s,p,q);p=q;}return result;}
static int pose_free(search*s,int k){if(!s->pose_cache[k]){passage_result result;s->pose_cache[k]=passage_pose_clear(s->w,at(s,k),s->w->config.width_m,&result)?1:2;}return s->pose_cache[k]==1;}
static int connect(const passage_world*w,passage_pose a,passage_xy b,int has_yaw,double yaw){
 passage_result result;double angle=hypot(b.x-a.x,b.y-a.y)>.000001?atan2(b.y-a.y,b.x-a.x):a.yaw;
 if(!passage_rotate_clear(w,a,angle,&result))return 0;a.yaw=angle;
 if(!passage_translate_clear(w,a,b,w->config.width_m,&result))return 0;
 return !has_yaw||passage_rotate_clear(w,(passage_pose){b.x,b.y,angle},yaw,&result);
}
static void release(search*s){free(s->cost);free(s->parent);free(s->heap);free(s->position);free(s->closed);free(s->pose_cache);}
static int orientation_plan(const passage_world*w,passage_pose start,passage_xy goal,int has_yaw,double goal_yaw,passage_xy*out,unsigned cap,unsigned*count,unsigned*expanded){
 if(count)*count=0;if(expanded)*expanded=0;if(!w||!out||cap<2||cap>PASSAGE_PATH_MAX||!count||!expanded||!passage_config_valid(&w->config)||!isfinite(goal.x)||!isfinite(goal.y)||!isfinite(goal_yaw)||has_yaw<0||has_yaw>1)return PASSAGE_INVALID;
 passage_result reason;if(!passage_pose_clear(w,start,w->config.width_m,&reason))return reason.reason;
 if(connect(w,start,goal,has_yaw,goal_yaw)){out[0]=(passage_xy){start.x,start.y};out[1]=goal;*count=2;return PASSAGE_CLEAR;}
 /* 点击的自由像素仍可能使车体覆盖障碍或未知格，在有界搜索前先拒绝此目标，避免遍历大量不可能到达的位姿。

 */
 int goal_clear=0,blocked_reason=PASSAGE_UNKNOWN;
 for(unsigned h=0;h<(has_yaw?1u:8u);++h){
  passage_pose at_goal={goal.x,goal.y,has_yaw?goal_yaw:start.yaw+(double)h*PI/4};
  if(passage_pose_clear(w,at_goal,w->config.width_m,&reason)){goal_clear=1;break;}
  if(h==0||reason.reason==PASSAGE_OCCUPIED)blocked_reason=reason.reason;
 }
 if(!goal_clear)return blocked_reason;
 search s={.w=w,.start=start,.goal=goal,.r=fmax(.05,w->config.local_resolution_m),.c=cos(start.yaw),.s=sin(start.yaw)};
 double gx=s.c*(goal.x-start.x)+s.s*(goal.y-start.y),gy=-s.s*(goal.x-start.x)+s.c*(goal.y-start.y),pad=1;
 s.ox=floor((fmin(0,gx)-pad)/s.r)*s.r;s.oy=floor((fmin(0,gy)-pad)/s.r)*s.r;
 double ww=ceil((fmax(0,gx)+pad-s.ox)/s.r)+1,hh=ceil((fmax(0,gy)+pad-s.oy)/s.r)+1;
 if(!isfinite(ww)||!isfinite(hh)||ww<1||hh<1||ww*hh*8>NODE_LIMIT)return PASSAGE_SEARCH_LIMIT;
 s.width=(unsigned)ww;s.height=(unsigned)hh;s.nodes=s.width*s.height*8;s.gx=(unsigned)llround((gx-s.ox)/s.r);s.gy=(unsigned)llround((gy-s.oy)/s.r);
 s.cost=malloc(s.nodes*sizeof(*s.cost));s.parent=malloc(s.nodes*sizeof(*s.parent));s.heap=malloc(s.nodes*sizeof(*s.heap));s.position=malloc(s.nodes*sizeof(*s.position));s.closed=calloc(s.nodes,1);s.pose_cache=calloc(s.nodes,1);
 if(!s.cost||!s.parent||!s.heap||!s.position||!s.closed||!s.pose_cache){release(&s);return PASSAGE_SEARCH_LIMIT;}
 for(unsigned i=0;i<s.nodes;i++){s.cost[i]=FLT_MAX;s.parent[i]=s.position[i]=-1;}
 unsigned sx=(unsigned)llround(-s.ox/s.r),sy=(unsigned)llround(-s.oy/s.r);int root=(int)((sy*s.width+sx)*8),found=-1;s.cost[root]=0;push(&s,root);
 int timed_out=0;
#ifdef LAB_REAL_POSIX_SOURCE
 uint64_t started=search_clock();
#endif
 while(s.heap_n){
#ifdef LAB_REAL_POSIX_SOURCE
  if(((*expanded)&31u)==0&&(search_clock()-started>=CONT_SEARCH_US||(w->cancelled&&w->cancelled(w->cancel_user)))){timed_out=1;break;}
#endif
  int k=pop(&s);if(s.closed[k])continue;s.closed[k]=1;passage_pose p=at(&s,k);unsigned cell=(unsigned)k/8,h=(unsigned)k%8,x=cell%s.width,y=cell/s.width;
  if(x==s.gx&&y==s.gy&&connect(w,p,goal,has_yaw,goal_yaw)){found=k;break;}
  if(++*expanded>EXPANSION_LIMIT)break;
  for(unsigned op=0;op<3;op++){
   int xx=(int)x,yy=(int)y,heading=(int)h;double step;
   if(op==0){xx+=dx[h];yy+=dy[h];step=dx[h]&&dy[h]?1.4142135623730951:1;}
   else{heading=(heading+(op==1?1:7))%8;step=2;}
   if(xx<0||yy<0||xx>=(int)s.width||yy>=(int)s.height)continue;int j=(int)(((unsigned)yy*s.width+(unsigned)xx)*8+(unsigned)heading);if(s.closed[j]||!pose_free(&s,j))continue;
   passage_pose q=at(&s,j);int clear=op==0?passage_translate_clear(w,p,(passage_xy){q.x,q.y},w->config.width_m,&reason):passage_rotate_clear(w,p,q.yaw,&reason);
   if(!clear)continue;float cost=s.cost[k]+(float)step;if(cost<s.cost[j]){s.cost[j]=cost;s.parent[j]=k;push(&s,j);}
  }
 }
 if(found<0){int r=timed_out||*expanded>EXPANSION_LIMIT?PASSAGE_SEARCH_LIMIT:PASSAGE_NO_PATH;release(&s);return r;}
 int32_t reverse[PASSAGE_PATH_MAX*4];unsigned n=0;for(int k=found;k>=0;k=s.parent[k]){if(n>=PASSAGE_PATH_MAX*4){release(&s);return PASSAGE_SEARCH_LIMIT;}reverse[n++]=k;}
 unsigned written=0;for(unsigned i=n;i>0;i--){passage_pose p=at(&s,reverse[i-1]);passage_xy point={p.x,p.y};if(written&&hypot(point.x-out[written-1].x,point.y-out[written-1].y)<1e-9)continue;
  if(written>=2){passage_xy a=out[written-2],b=out[written-1];double ax=b.x-a.x,ay=b.y-a.y,bx=point.x-b.x,by=point.y-b.y;if(fabs(ax*by-ay*bx)<1e-9&&ax*bx+ay*by>0){out[written-1]=point;continue;}}
  if(written>=cap){release(&s);return PASSAGE_SEARCH_LIMIT;}out[written++]=point;
 }
 if(written&&hypot(out[written-1].x-goal.x,out[written-1].y-goal.y)<1e-9)out[written-1]=goal;
 else{if(written>=cap){release(&s);return PASSAGE_SEARCH_LIMIT;}out[written++]=goal;}
 if(written<2){out[1]=goal;written=2;}*count=written;release(&s);return PASSAGE_CLEAR;
}

#include "passage_corridor.inc"
