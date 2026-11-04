/* 快速二维走廊提案之后核对矩形车体转弯和平移，未知格带有代价，不直接形成完整可行路线。
 */
#define CORRIDOR_NODES 250000u
typedef struct corridor {
 const passage_world*w;unsigned width,height,n,heap_n,gx,gy;
 double r,ox,oy,safe_radius;float*cost;int32_t*parent,*heap,*pos;
 uint16_t*clearance;uint8_t*closed;int8_t*state;
} corridor;
static double c_h(const corridor*s,int k){return hypot((double)(k%(int)s->width)-s->gx,(double)(k/(int)s->width)-s->gy);}
static int c_less(const corridor*s,int a,int b){double x=s->cost[a]+c_h(s,a),y=s->cost[b]+c_h(s,b);return x<y-1e-8||(fabs(x-y)<=1e-8&&a<b);}
static void c_swap(corridor*s,unsigned a,unsigned b){int k=s->heap[a];s->heap[a]=s->heap[b];s->heap[b]=k;s->pos[s->heap[a]]=(int)a;s->pos[s->heap[b]]=(int)b;}
static void c_push(corridor*s,int k){unsigned p=s->pos[k]>=0?(unsigned)s->pos[k]:s->heap_n++;s->heap[p]=k;s->pos[k]=(int)p;while(p){unsigned q=(p-1)/2;if(!c_less(s,s->heap[p],s->heap[q]))break;c_swap(s,p,q);p=q;}}
static int c_pop(corridor*s){int k=s->heap[0];s->pos[k]=-1;if(!--s->heap_n)return k;s->heap[0]=s->heap[s->heap_n];s->pos[s->heap[0]]=0;unsigned p=0;for(;;){unsigned q=2*p+1;if(q>=s->heap_n)break;if(q+1<s->heap_n&&c_less(s,s->heap[q+1],s->heap[q]))q++;if(!c_less(s,s->heap[q],s->heap[p]))break;c_swap(s,p,q);p=q;}return k;}
static passage_xy c_point(const corridor*s,int k){return (passage_xy){s->ox+(k%(int)s->width+.5)*s->r,s->oy+(k/(int)s->width+.5)*s->r};}
static int c_key(const corridor*s,passage_xy p){int x=(int)floor((p.x-s->ox)/s->r),y=(int)floor((p.y-s->oy)/s->r);return x<0||y<0||x>=(int)s->width||y>=(int)s->height?-1:y*(int)s->width+x;}
static int c_blocked(const corridor*s,int k){double margin=s->safe_radius;return s->state[k]>=65||s->clearance[k]*s->r<margin+s->r*.35;}
static void c_release(corridor*s){free(s->cost);free(s->parent);free(s->heap);free(s->pos);free(s->clearance);free(s->closed);free(s->state);}
static int c_visible(const corridor*s,int a,int b){int x=a%(int)s->width,y=a/(int)s->width,tx=b%(int)s->width,ty=b/(int)s->width,ax=abs(tx-x),ay=abs(ty-y),sx=x<tx?1:-1,sy=y<ty?1:-1,e=ax-ay;
 for(;;){if(c_blocked(s,y*(int)s->width+x))return 0;if(x==tx&&y==ty)return 1;int z=2*e,nx=x,ny=y;if(z>-ay){e-=ay;nx+=sx;}if(z<ax){e+=ax;ny+=sy;}if(nx!=x&&ny!=y&&(c_blocked(s,y*(int)s->width+nx)||c_blocked(s,ny*(int)s->width+x)))return 0;x=nx;y=ny;}}
/* 准确返回取消、预算等失败类型，与碰撞分开；此处不软化未知区，以正确显示路线结果。
 */
static int c_connect(const passage_world*w,passage_pose a,passage_xy b,int final,double yaw,int*reason){passage_result hit;passage_world strict=*w;strict.planning_unknown_soft=0;strict.observed_probe=0;double angle=hypot(b.x-a.x,b.y-a.y)>1e-8?atan2(b.y-a.y,b.x-a.x):a.yaw;
 if(!passage_rotate_clear(&strict,a,angle,&hit)){*reason=hit.reason;return 0;}a.yaw=angle;
 if(!passage_translate_clear(&strict,a,b,w->config.width_m,&hit)){*reason=hit.reason;return 0;}
 if(final&&!passage_rotate_clear(&strict,(passage_pose){b.x,b.y,angle},yaw,&hit)){*reason=hit.reason;return 0;}*reason=PASSAGE_CLEAR;return 1;}
static int c_append(passage_xy*out,unsigned cap,unsigned*n,passage_xy p){if(*n&&hypot(p.x-out[*n-1].x,p.y-out[*n-1].y)<1e-8)return 1;if(*n>=cap)return 0;out[(*n)++]=p;return 1;}
static int corridor_plan(const passage_world*w,passage_pose start,passage_xy goal,int has_yaw,double yaw,passage_xy*out,unsigned cap,unsigned*count,unsigned*expanded,passage_plan_report*report,int relaxed){
 corridor s={.w=w};s.safe_radius=(relaxed?w->config.width_m*.5:hypot(w->config.width_m*.5,w->config.length_m*.5))+w->config.margin_m;s.r=fmax(.05,w->global.resolution);s.r=fmax(s.r,fmax(w->global.width*w->global.resolution,w->global.height*w->global.resolution)/500.0);s.ox=w->global.origin_x;s.oy=w->global.origin_y;
 s.width=(unsigned)ceil(w->global.width*w->global.resolution/s.r);s.height=(unsigned)ceil(w->global.height*w->global.resolution/s.r);if(!s.width||!s.height||(uint64_t)s.width*s.height>CORRIDOR_NODES)return PASSAGE_SEARCH_LIMIT;s.n=s.width*s.height;
 int root=c_key(&s,(passage_xy){start.x,start.y}),target=c_key(&s,goal);if(root<0||target<0)return PASSAGE_OUTSIDE;s.gx=(unsigned)target%s.width;s.gy=(unsigned)target/s.width;
 s.cost=malloc(s.n*sizeof(*s.cost));s.parent=malloc(s.n*sizeof(*s.parent));s.heap=malloc(s.n*sizeof(*s.heap));s.pos=malloc(s.n*sizeof(*s.pos));s.clearance=malloc(s.n*sizeof(*s.clearance));s.closed=calloc(s.n,1);s.state=malloc(s.n);
 if(!s.cost||!s.parent||!s.heap||!s.pos||!s.clearance||!s.closed||!s.state){c_release(&s);return PASSAGE_SEARCH_LIMIT;}
 for(unsigned y=0;y<s.height;y++){if(w->cancelled&&w->cancelled(w->cancel_user)){c_release(&s);return PASSAGE_SEARCH_LIMIT;}for(unsigned x=0;x<s.width;x++){unsigned k=y*s.width+x;int value=0;unsigned samples=(unsigned)ceil(s.r/fmin(w->global.resolution,w->config.local_resolution_m));if(samples>16)samples=16;
  for(unsigned iy=0;iy<samples&&value<65;iy++)for(unsigned ix=0;ix<samples;ix++){double wx=s.ox+(x+(ix+.5)/samples)*s.r,wy=s.oy+(y+(iy+.5)/samples)*s.r;int v=passage_planning_sample(w,wx,wy);if(v>=65){value=100;break;}if(v<0)value=-1;}
  s.state[k]=(int8_t)value;s.clearance[k]=value>=65?0:32760;s.cost[k]=FLT_MAX;s.parent[k]=s.pos[k]=-1;
 }}
 /* 切比雪夫净空仅作保守候选余量，之后精确车体几何决定转弯，无需每节点执行八朝向多边形扫掠。
 */
 for(unsigned y=0;y<s.height;y++)for(unsigned x=0;x<s.width;x++){unsigned k=y*s.width+x,d=s.clearance[k];if(x&&s.clearance[k-1]+1u<d)d=s.clearance[k-1]+1u;if(y)for(int j=-1;j<=1;j++){int xx=(int)x+j;if(xx>=0&&xx<(int)s.width&&s.clearance[k-s.width+j]+1u<d)d=s.clearance[k-s.width+j]+1u;}s.clearance[k]=(uint16_t)d;}
 for(int y=(int)s.height-1;y>=0;y--)for(int x=(int)s.width-1;x>=0;x--){unsigned k=(unsigned)y*s.width+(unsigned)x,d=s.clearance[k];if(x+1<(int)s.width&&s.clearance[k+1]+1u<d)d=s.clearance[k+1]+1u;if(y+1<(int)s.height)for(int j=-1;j<=1;j++){int xx=x+j;if(xx>=0&&xx<(int)s.width&&s.clearance[k+s.width+j]+1u<d)d=s.clearance[k+s.width+j]+1u;}s.clearance[k]=(uint16_t)d;}
 s.cost[root]=0;c_push(&s,root);int found=-1;unsigned visited=0;
 while(s.heap_n){if((visited&127u)==0&&w->cancelled&&w->cancelled(w->cancel_user)){report->budget=1;break;}int k=c_pop(&s);if(s.closed[k])continue;s.closed[k]=1;if(k==target){found=k;break;}if(++visited>=EXPANSION_LIMIT){report->budget=1;break;}
  int x=k%(int)s.width,y=k/(int)s.width;for(unsigned d=0;d<8;d++){int xx=x+dx[d],yy=y+dy[d];if(xx<0||yy<0||xx>=(int)s.width||yy>=(int)s.height)continue;int j=yy*(int)s.width+xx;if(s.closed[j]||c_blocked(&s,j))continue;if(s.state[j]<0&&!w->planning_unknown_soft)continue;if(dx[d]&&dy[d]&&(c_blocked(&s,y*(int)s.width+xx)||c_blocked(&s,yy*(int)s.width+x)))continue;
   double unknown=s.state[j]<0?8:0,clearance=1.5/fmax(1,s.clearance[j]);float cost=s.cost[k]+(float)((dx[d]&&dy[d]?1.4142135623730951:1)*(1+unknown+clearance));if(cost<s.cost[j]){s.cost[j]=cost;s.parent[j]=k;c_push(&s,j);}}
 }
 *expanded+=visited;report->corridor_expanded+=visited;if(found<0){c_release(&s);return report->budget?PASSAGE_SEARCH_LIMIT:PASSAGE_NO_PATH;}
 int32_t*reverse=malloc(s.n*sizeof(*reverse));if(!reverse){c_release(&s);return PASSAGE_SEARCH_LIMIT;}unsigned raw_n=0;for(int k=found;k>=0;k=s.parent[k]){if(raw_n>=s.n){free(reverse);c_release(&s);return PASSAGE_SEARCH_LIMIT;}reverse[raw_n++]=k;}
 for(unsigned i=0;i<raw_n/2;i++){int k=reverse[i];reverse[i]=reverse[raw_n-1-i];reverse[raw_n-1-i]=k;}
 unsigned anchor=0,written=0;passage_pose pose=start;c_append(out,cap,&written,(passage_xy){start.x,start.y});int failure=PASSAGE_CLEAR;int used_pose=0;
 while(anchor+1<raw_n){unsigned next=anchor+1;for(unsigned j=next+1;j<raw_n;j++){if(!c_visible(&s,reverse[anchor],reverse[j]))break;next=j;}
  passage_xy end=next+1==raw_n?goal:c_point(&s,reverse[next]);int final=next+1==raw_n&&has_yaw;
  if(!c_connect(w,pose,end,final,yaw,&failure)){
   /* 仅问题弯道执行朝向搜索，不在整张远距离地图上搜索八朝向。
 */
   int repaired=0;if(failure!=PASSAGE_UNKNOWN&&failure!=PASSAGE_OUTSIDE&&failure!=PASSAGE_SEARCH_LIMIT&&hypot(end.x-pose.x,end.y-pose.y)<=1.5){passage_world local=*w;local.planning_unknown_soft=0;passage_xy patch[256];unsigned pn=0,pe=0;int pr=orientation_plan(&local,pose,end,final,yaw,patch,256,&pn,&pe);*expanded+=pe;report->pose_expanded+=pe;
    if(pr==PASSAGE_CLEAR){for(unsigned i=1;i<pn;i++)if(!c_append(out,cap,&written,patch[i])){failure=PASSAGE_SEARCH_LIMIT;break;}if(failure!=PASSAGE_SEARCH_LIMIT){repaired=1;used_pose=1;}}else failure=pr;}
   if(!repaired){
    /* 截取候选线段中已有观测的前缀作为局部目标；未观测部分不生成直行路线。 */
    double lo=0,hi=1;passage_xy prefix={pose.x,pose.y};if(failure!=PASSAGE_SEARCH_LIMIT)for(unsigned i=0;i<9;i++){double t=(lo+hi)*.5;passage_xy q={pose.x+t*(end.x-pose.x),pose.y+t*(end.y-pose.y)};int why=0;if(c_connect(w,pose,q,0,0,&why)){lo=t;prefix=q;}else{hi=t;if(why==PASSAGE_SEARCH_LIMIT){failure=why;break;}}}
    if(lo>0&&hypot(prefix.x-pose.x,prefix.y-pose.y)>.04)c_append(out,cap,&written,prefix);
    report->budget=failure==PASSAGE_SEARCH_LIMIT;report->reason=failure;report->partial=written>=2;report->kind=written>=2?PASSAGE_ROUTE_PARTIAL:PASSAGE_ROUTE_NONE;*count=written>=2?written:0;free(reverse);c_release(&s);return *count?PASSAGE_PARTIAL:failure;
   }
  }else if(!c_append(out,cap,&written,end)){free(reverse);c_release(&s);return PASSAGE_SEARCH_LIMIT;}
  if(written>=2)pose.yaw=atan2(out[written-1].y-out[written-2].y,out[written-1].x-out[written-2].x);pose.x=end.x;pose.y=end.y;anchor=next;
 }
 free(reverse);c_release(&s);*count=written;report->kind=used_pose?PASSAGE_ROUTE_LOCAL_POSE:PASSAGE_ROUTE_CORRIDOR;report->complete=written>=2;report->reason=PASSAGE_CLEAR;return written>=2?PASSAGE_CLEAR:PASSAGE_NO_PATH;
}
typedef struct corridor_budget {const passage_world*original;
#ifdef LAB_REAL_POSIX_SOURCE
 uint64_t started;
#endif
} corridor_budget;
static int c_budget_cancel(void*u){corridor_budget*b=u;if(b->original->cancelled&&b->original->cancelled(b->original->cancel_user))return 1;
#ifdef LAB_REAL_POSIX_SOURCE
 if(search_clock()-b->started>=CONT_SEARCH_US)return 1;
#endif
 return 0;}
int passage_plan_ex(const passage_world*w,passage_pose start,passage_xy goal,int has_yaw,double yaw,passage_xy*out,unsigned cap,unsigned*count,unsigned*expanded,passage_plan_report*report){
 passage_plan_report local_report={0};if(!report)report=&local_report;memset(report,0,sizeof(*report));if(count)*count=0;if(expanded)*expanded=0;
 if(!w||!out||!count||!expanded||cap<2||cap>PASSAGE_PATH_MAX||!passage_config_valid(&w->config)||!w->global.cells||!w->global.width||!w->global.height||!isfinite(goal.x)||!isfinite(goal.y)||!isfinite(yaw)||has_yaw<0||has_yaw>1){report->reason=PASSAGE_INVALID;return PASSAGE_INVALID;}
 corridor_budget budget={.original=w};
#ifdef LAB_REAL_POSIX_SOURCE
 budget.started=search_clock();
#endif
 passage_world bounded=*w;bounded.cancelled=c_budget_cancel;bounded.cancel_user=&budget;w=&bounded;
 passage_world strict_start=*w;strict_start.planning_unknown_soft=0;strict_start.observed_probe=0;passage_result start_hit;
 if(!passage_pose_clear(&strict_start,start,w->config.width_m,&start_hit)){report->reason=start_hit.reason;report->budget=start_hit.reason==PASSAGE_SEARCH_LIMIT;return start_hit.reason;}
 int goal_free=0,goal_reason=PASSAGE_UNKNOWN;for(unsigned h=0;h<(has_yaw?1u:8u);h++){passage_result hit;if(passage_pose_clear(&strict_start,(passage_pose){goal.x,goal.y,has_yaw?yaw:start.yaw+h*PI/4},w->config.width_m,&hit)){goal_free=1;break;}if(hit.reason==PASSAGE_SEARCH_LIMIT){report->reason=PASSAGE_SEARCH_LIMIT;report->budget=1;return PASSAGE_SEARCH_LIMIT;}if(hit.reason==PASSAGE_OCCUPIED||goal_reason!=PASSAGE_OCCUPIED)goal_reason=hit.reason;}
 if(!goal_free&&(!w->planning_unknown_soft||goal_reason==PASSAGE_OCCUPIED)){report->reason=goal_reason;return goal_reason;}
 int why=0;if(c_connect(w,start,goal,has_yaw,yaw,&why)){out[0]=(passage_xy){start.x,start.y};out[1]=goal;*count=2;report->complete=1;report->kind=PASSAGE_ROUTE_DIRECT;return PASSAGE_CLEAR;}if(why==PASSAGE_SEARCH_LIMIT){report->budget=1;report->reason=why;return why;}
 int result=corridor_plan(w,start,goal,has_yaw,yaw,out,cap,count,expanded,report,0);
 if(result==PASSAGE_NO_PATH){result=corridor_plan(w,start,goal,has_yaw,yaw,out,cap,count,expanded,report,1);}
 /* 已观测窄通道可能不满足粗半宽提案，但短程精确位姿动作仍可行，保留局部回退。
 */
 if(result!=PASSAGE_CLEAR&&result!=PASSAGE_PARTIAL&&result!=PASSAGE_SEARCH_LIMIT&&hypot(goal.x-start.x,goal.y-start.y)<=1.5){passage_world strict=*w;strict.planning_unknown_soft=0;unsigned pe=0;result=orientation_plan(&strict,start,goal,has_yaw,yaw,out,cap,count,&pe);*expanded+=pe;report->pose_expanded+=pe;if(result==PASSAGE_CLEAR){report->kind=PASSAGE_ROUTE_LOCAL_POSE;report->complete=1;}}
 report->reason=result==PASSAGE_PARTIAL?report->reason:result;report->budget=result==PASSAGE_SEARCH_LIMIT||report->budget;return result;
}
int passage_plan(const passage_world*w,passage_pose start,passage_xy goal,int has_yaw,double yaw,passage_xy*out,unsigned cap,unsigned*count,unsigned*expanded){return passage_plan_ex(w,start,goal,has_yaw,yaw,out,cap,count,expanded,NULL);}
