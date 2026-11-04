#ifndef ACG_FOLLOW_ASSOCIATION_V5_H
#define ACG_FOLLOW_ASSOCIATION_V5_H
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define FV5_BEAMS 8192u
#define FV5_TRACKS 32u
#define FV5_LINES 16u
typedef struct fv5_point {double x,y,angle,range;unsigned index;} fv5_point;
typedef struct fv5_line {double x,y,ux,uy,low,high,score;} fv5_line;
typedef struct fv5_track {uint64_t first,last,anchor_time;double x,y,vx,vy,anchor_x,anchor_y,motion,isolation,span;unsigned used,updated,hits,points,history_count;uint64_t history_time[3];double history_x[3],history_y[3];} fv5_track;
typedef struct fv5_state {
 uint64_t source_us;uint32_t sequence;unsigned prepared,line_count,groups;int chosen;
 unsigned char wall[FV5_BEAMS],track[FV5_BEAMS];fv5_line lines[FV5_LINES];fv5_track tracks[FV5_TRACKS];
 uint64_t visual_us,visual_frame,pending_us;double visual_x,visual_y,visual_yaw,visual_half,pending_x,pending_y;unsigned pending_hits;int pending_track;
 /* ODOM目标位置作为候选排序先验。 */
 uint64_t priority_us;double priority_x,priority_y,priority_gate;
} fv5_state;
static double fv5_clamp(double x,double a,double b){return fmax(a,fmin(b,x));}
static int fv5_order(const void*a,const void*b){double x=((const fv5_point*)a)->angle,y=((const fv5_point*)b)->angle;return x<y?-1:x>y;}
/* 跟随点连接阈值随光束间距增长，使用两端较小半径，并同时检查两个阈值。独立径向条件将小前景回波与后方同方向墙面分开。此处按角度顺序建立二维连接，不是完整欧氏聚类；设计参考Autoware.Auto的径向阈值方法。



 */
static int fv5_connected(double ax,double ay,double aa,double ar,double bx,double by,double ba,double br){
 double r=fmin(ar,br),gap=fv5_clamp(.12+.045*r,.18,.55);
 return fabs(ba-aa)<=.06&&fabs(br-ar)<=fmin(.24,.16+.008*r)&&hypot(bx-ax,by-ay)<=gap;
}
/* 孤立点支持需来自两个不同扫描时刻，间隔有上限；重复读取或间隔过长不能形成独立支持。
 */
static int fv5_sparse_supported(const fv5_track*t){
 if(!t||t->history_count<2)return 0;unsigned h=t->history_count-1;
 uint64_t dt=t->history_time[h]-t->history_time[h-1];return dt>=50000&&dt<=350000;
}
/* 在相机裁剪前拟合整扫稳健直线，只有实际直线内点才归为墙面，前景突起仍保留。
 */
static int fv5_line_fit(fv5_state*s,const fv5_point*p,unsigned begin,unsigned end){
 unsigned n=end-begin;if(n<5)return 0;double cx=0,cy=0,ux=1,uy=0,low=0,high=0,rms=0;unsigned accepted=n;
 for(unsigned pass=0;pass<3;pass++){
  double x=0,y=0,xx=0,xy=0,yy=0;unsigned count=0;
  for(unsigned i=begin;i<end;i++){if(pass&&fabs(-uy*(p[i].x-cx)+ux*(p[i].y-cy))>.065)continue;x+=p[i].x;y+=p[i].y;xx+=p[i].x*p[i].x;xy+=p[i].x*p[i].y;yy+=p[i].y*p[i].y;count++;}
  if(count<5)return 0;cx=x/count;cy=y/count;double a=.5*atan2(2*(xy/count-cx*cy),xx/count-cx*cx-(yy/count-cy*cy));ux=cos(a);uy=sin(a);accepted=count;
 }
 low=1e9;high=-1e9;rms=0;accepted=0;
 for(unsigned i=begin;i<end;i++){double q=-uy*(p[i].x-cx)+ux*(p[i].y-cy);if(fabs(q)>.065)continue;double t=ux*(p[i].x-cx)+uy*(p[i].y-cy);low=fmin(low,t);high=fmax(high,t);rms+=q*q;accepted++;}
 if(accepted<5||accepted<n*.60||high-low<.65||sqrt(rms/accepted)>.045)return 0;
 double score=fv5_clamp(.79+.18*fmin(1,(high-low-.65)/1.5),.79,.97);
 for(unsigned i=begin;i<end;i++)if(p[i].index<FV5_BEAMS&&fabs(-uy*(p[i].x-cx)+ux*(p[i].y-cy))<=.065)s->wall[p[i].index]=(unsigned char)fmax(s->wall[p[i].index],score*255);
 if(s->line_count<FV5_LINES)s->lines[s->line_count++]=(fv5_line){cx,cy,ux,uy,low,high,score};return 1;
}
static void fv5_lines(fv5_state*s,const fv5_point*p,unsigned begin,unsigned end,unsigned depth){
 if(end-begin<5)return;if(fv5_line_fit(s,p,begin,end)||depth>=4)return;
 unsigned mid=begin+(end-begin)/2;fv5_lines(s,p,begin,mid,depth+1);fv5_lines(s,p,mid,end,depth+1);
}
static int fv5_track_group(fv5_state*s,const fv5_point*p,unsigned begin,unsigned end,uint64_t stamp,double isolation){
 if(end<=begin)return -1;double x=0,y=0;for(unsigned i=begin;i<end;i++){x+=p[i].x;y+=p[i].y;}x/=(end-begin);y/=(end-begin);
 double span=hypot(p[end-1].x-p[begin].x,p[end-1].y-p[begin].y);int best=-1;double cost=1e9;
 for(unsigned i=0;i<FV5_TRACKS;i++){fv5_track*t=&s->tracks[i];if(!t->used||t->updated||stamp<=t->last||stamp-t->last>1500000)continue;double dt=(stamp-t->last)/1e6,px=t->x+t->vx*fmin(dt,1),py=t->y+t->vy*fmin(dt,1),d=hypot(x-px,y-py);double gate=fmin(1.5,.30+.8*dt+.025*p[begin].range);if(d<gate&&d<cost){cost=d;best=(int)i;}}
 if(best<0){for(unsigned i=0;i<FV5_TRACKS;i++)if(!s->tracks[i].used){best=(int)i;break;}}
 if(best<0){uint64_t oldest=UINT64_MAX;for(unsigned i=0;i<FV5_TRACKS;i++)if((int)i!=s->chosen&&!s->tracks[i].updated&&s->tracks[i].last<oldest){oldest=s->tracks[i].last;best=(int)i;}}
 if(best<0)return -1;fv5_track*t=s->tracks+best;
 if(!t->used||stamp<=t->last||stamp-t->last>1500000||cost==1e9){memset(t,0,sizeof(*t));t->used=1;t->first=t->anchor_time=stamp;t->anchor_x=x;t->anchor_y=y;}
 else{double dt=(stamp-t->last)/1e6;if(dt>.01&&dt<=1.5){double vx=fv5_clamp((x-t->x)/dt,-2,2),vy=fv5_clamp((y-t->y)/dt,-2,2);t->vx=.65*t->vx+.35*vx;t->vy=.65*t->vy+.35*vy;}}
 double elapsed=(stamp-t->anchor_time)/1e6;if(elapsed>=.25){double displacement=hypot(x-t->anchor_x,y-t->anchor_y);t->motion=.65*t->motion+.35*fv5_clamp((displacement-.05)/.18,0,1);if(elapsed>1.0){t->anchor_x=x;t->anchor_y=y;t->anchor_time=stamp;}}
 t->x=x;t->y=y;t->last=stamp;t->updated=1;t->hits++;t->isolation=isolation;t->span=span;t->points=end-begin;
 unsigned h=t->history_count<3?t->history_count++:2;if(h==2&&t->history_time[2]){for(unsigned j=0;j<2;j++){t->history_time[j]=t->history_time[j+1];t->history_x[j]=t->history_x[j+1];t->history_y[j]=t->history_y[j+1];}}t->history_time[h]=stamp;t->history_x[h]=x;t->history_y[h]=y;
 for(unsigned i=begin;i<end;i++)if(p[i].index<FV5_BEAMS)s->track[p[i].index]=(unsigned char)best;return best;
}
/* 调用前按实际扫描位姿将点转换到连续ODOM，地图跳变或车体相对距离不能用于估计目标运动。
 */
static int fv5_prepare_mode(fv5_state*s,fv5_point*p,unsigned n,uint64_t stamp,uint32_t sequence,int walls){
 if(n>FV5_BEAMS||!stamp||(!p&&n))return 0;for(unsigned i=0;i<n;i++)if(p[i].index>=FV5_BEAMS||!isfinite(p[i].x)||!isfinite(p[i].y)||!isfinite(p[i].angle)||!isfinite(p[i].range)||p[i].range<=0)return 0;if(s->prepared&&stamp<=s->source_us)return 0;
 if(!s->prepared)s->chosen=-1;s->prepared=1;s->source_us=stamp;s->sequence=sequence;s->groups=s->line_count=0;
 memset(s->wall,0,sizeof(s->wall));memset(s->track,255,sizeof(s->track));for(unsigned i=0;i<FV5_TRACKS;i++)s->tracks[i].updated=0;
 qsort(p,n,sizeof(*p),fv5_order);unsigned begin=0;
 for(unsigned i=1;i<=n;i++)if(i==n||!fv5_connected(p[i-1].x,p[i-1].y,p[i-1].angle,p[i-1].range,p[i].x,p[i].y,p[i].angle,p[i].range)){if(walls)fv5_lines(s,p,begin,i,0);begin=i;s->groups++;}
 /* 前景遮挡可能将长墙切成短角度段，只合并拟合兼容的墙段，再判断真实内点；前景深度突变保持在墙面外。

 */
 for(unsigned a=0;a<s->line_count;a++)for(unsigned b=a+1;b<s->line_count;b++){
  fv5_line*l=s->lines+a,*m=s->lines+b;double dot=l->ux*m->ux+l->uy*m->uy,dx=m->x-l->x,dy=m->y-l->y;
  if(fabs(dot)<.995||fabs(-l->uy*dx+l->ux*dy)>.065)continue;double offset=l->ux*dx+l->uy*dy,lo=offset+fmin(dot*m->low,dot*m->high),hi=offset+fmax(dot*m->low,dot*m->high);
  if(lo>l->high+1.||hi<l->low-1.)continue;l->low=fmin(l->low,lo);l->high=fmax(l->high,hi);
 }
 for(unsigned j=0;j<n;j++)for(unsigned a=0;a<s->line_count;a++){const fv5_line*l=s->lines+a;double dx=p[j].x-l->x,dy=p[j].y-l->y,t=l->ux*dx+l->uy*dy;if(t>=l->low&&t<=l->high&&fabs(-l->uy*dx+l->ux*dy)<=.065)s->wall[p[j].index]=(unsigned char)fmax(s->wall[p[j].index],l->score*255);}
 /* 墙内点只从目标分组中排除，压紧私有临时数组，避免墙点分开相邻前景光束。
 */
 if(walls){unsigned valid=0;for(unsigned j=0;j<n;j++)if(s->wall[p[j].index]<190)p[valid++]=p[j];n=valid;}
 /* 保留关联度最高的32组，不按角度先后填满数组；整扫墙面和原始障碍输入不变。局部描述符512字节，选择工作量上限为O(n×32)。

 */
 struct {unsigned begin,end;double rank;} keep[FV5_TRACKS];unsigned kept=0;
 begin=0;
 for(unsigned i=0;i<=n;i++){
  int boundary=i==n||(i>begin&&!fv5_connected(p[i-1].x,p[i-1].y,p[i-1].angle,p[i-1].range,p[i].x,p[i].y,p[i].angle,p[i].range))||(i<n&&s->wall[p[i].index]>=190);
  if(boundary){if(i>begin){double x=0,y=0;for(unsigned j=begin;j<i;j++){x+=p[j].x;y+=p[j].y;}x/=i-begin;y/=i-begin;double rank=10+fabs(p[begin].angle);
    if(s->visual_us&&(stamp>=s->visual_us?stamp-s->visual_us:s->visual_us-stamp)<=5000000){double age=stamp>=s->visual_us?(stamp-s->visual_us)/1e6:0;double a=fabs(remainder(atan2(y-s->visual_y,x-s->visual_x)-s->visual_yaw,6.283185307179586)),half=fmin(1.,s->visual_half+.10*age);if(a<=half)rank=-100+a/fmax(.035,half);}
    if(s->priority_us&&stamp>=s->priority_us&&stamp-s->priority_us<=10000000&&s->priority_gate>0){double d=hypot(x-s->priority_x,y-s->priority_y);if(d<=s->priority_gate)rank=-500+d/s->priority_gate;}
    if(s->chosen>=0&&s->chosen<(int)FV5_TRACKS){const fv5_track*t=s->tracks+s->chosen;if(t->used&&stamp>t->last&&stamp-t->last<=1500000){double dt=(stamp-t->last)/1e6,d=hypot(x-t->x-t->vx*fmin(dt,1),y-t->y-t->vy*fmin(dt,1)),gate=fmin(1.5,.30+.8*dt+.025*p[begin].range);if(d<gate)rank=-1000+d/gate;}}
    unsigned at=0;while(at<kept&&keep[at].rank<=rank)at++;if(at<FV5_TRACKS){if(kept<FV5_TRACKS)kept++;for(unsigned j=kept-1;j>at;j--)keep[j]=keep[j-1];keep[at].begin=begin;keep[at].end=i;keep[at].rank=rank;}
   }begin=i;
   if(i<n&&s->wall[p[i].index]>=190)begin=i+1;
  }
 }
 for(unsigned k=0;k<kept;k++){unsigned a=keep[k].begin,b=keep[k].end;double before=a?hypot(p[a].x-p[a-1].x,p[a].y-p[a-1].y):.5,after=b<n?hypot(p[b].x-p[b-1].x,p[b].y-p[b-1].y):.5;fv5_track_group(s,p,a,b,stamp,fv5_clamp(fmin(before,after)/.20,.1,1));}
 return 1;
}
static int fv5_prepare(fv5_state*s,fv5_point*p,unsigned n,uint64_t stamp,uint32_t sequence){return fv5_prepare_mode(s,p,n,stamp,sequence,1);}
static double fv5_wall_at(const fv5_state*s,double x,double y){double score=0;for(unsigned i=0;i<s->line_count;i++){const fv5_line*l=s->lines+i;double dx=x-l->x,dy=y-l->y,along=l->ux*dx+l->uy*dy,perp=fabs(-l->uy*dx+l->ux*dy);if(along>=l->low-.10&&along<=l->high+.10&&perp<.12)score=fmax(score,l->score*(1-.25*perp/.12));}return score;}
#endif
