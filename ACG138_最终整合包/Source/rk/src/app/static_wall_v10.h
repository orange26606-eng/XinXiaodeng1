#ifndef ACG_STATIC_WALL_V10_H
#define ACG_STATIC_WALL_V10_H
/* 仅静止目标使用背景分类，跟随仍由原辅助模块处理，全部计算基于私有ODOM扫描点。
 */
static double st10_wall_at(const fv5_state*s,double x,double y){
 double score=0;for(unsigned i=0;i<s->line_count;i++){const fv5_line*l=s->lines+i;double dx=x-l->x,dy=y-l->y,along=l->ux*dx+l->uy*dy;if(along>=l->low-.02&&along<=l->high+.02&&fabs(-l->uy*dx+l->ux*dy)<=.020)score=fmax(score,l->score);}return score;
}
static int st10_background_in_target(const fv5_state*s){
 if(!s->visual_us)return 0;double vx=cos(s->visual_yaw),vy=sin(s->visual_yaw);
 for(unsigned i=0;i<s->line_count;i++){const fv5_line*l=s->lines+i;double den=-l->uy*vx+l->ux*vy;if(fabs(den)<1e-8)continue;double dx=l->x-s->visual_x,dy=l->y-s->visual_y,t=(-l->uy*dx+l->ux*dy)/den;if(t<=0)continue;double along=l->ux*(s->visual_x+t*vx-l->x)+l->uy*(s->visual_y+t*vy-l->y);if(along>=l->low&&along<=l->high)return 1;}return 0;
}
static int st10_background_line(const fv5_state*s,const fv5_line*l,const fv5_point*p,unsigned n){
 if(!s->visual_us||!isfinite(s->visual_yaw)||!isfinite(s->visual_half)||l->high-l->low<1.0)return 0;
 unsigned count=0,outside=0;double left=1e9,right=-1e9,inlo=1e9,inhi=-1e9,outlo=1e9,outhi=-1e9;
 for(unsigned i=0;i<n;i++){
  double dx=p[i].x-l->x,dy=p[i].y-l->y,perp=fabs(-l->uy*dx+l->ux*dy),t=l->ux*dx+l->uy*dy;if(perp>.020||t<l->low-.02||t>l->high+.02)continue;
  count++;left=fmin(left,t);right=fmax(right,t);double angle=fabs(remainder(atan2(p[i].y-s->visual_y,p[i].x-s->visual_x)-s->visual_yaw,6.283185307179586));
  if(angle>s->visual_half+.035){outside++;outlo=fmin(outlo,t);outhi=fmax(outhi,t);}else{inlo=fmin(inlo,t);inhi=fmax(inhi,t);}
 }
 /* 小型静止物体不直接当墙，需在所选角域外有实际直线支持，且延伸至少65cm；完全在角域外的直线不影响该目标裁剪。

 */
 if(count<8||outside<6||right-left<1.0)return 0;
 if(inlo>inhi)return outhi-outlo>=.65;
 return inlo-left>=.65||right-inhi>=.65||(inlo-left>=.35&&right-inhi>=.35);
}
static int st10_prepare(fv5_state*s,fv5_point*p,unsigned n,uint64_t stamp,uint32_t sequence){
 if(n>FV5_BEAMS||!stamp||(!p&&n)||(s->prepared&&stamp<=s->source_us))return 0;
 for(unsigned i=0;i<n;i++)if(p[i].index>=FV5_BEAMS||!isfinite(p[i].x)||!isfinite(p[i].y)||!isfinite(p[i].angle)||!isfinite(p[i].range)||p[i].range<=0)return 0;
 fv5_state*w=calloc(1,sizeof(*w));if(!w)return 0;qsort(p,n,sizeof(*p),fv5_order);
 unsigned begin=0;for(unsigned i=1;i<=n;i++)if(i==n||!fv5_connected(p[i-1].x,p[i-1].y,p[i-1].angle,p[i-1].range,p[i].x,p[i].y,p[i].angle,p[i].range)){fv5_lines(w,p,begin,i,0);begin=i;}
 /* 只合并遮挡前景两侧共线且有原始回波支持的线段，前景点需保持在狭窄墙面残差之外。
 */
 for(unsigned a=0;a<w->line_count;a++)for(unsigned b=a+1;b<w->line_count;b++){
  fv5_line*l=w->lines+a,*m=w->lines+b;double dot=l->ux*m->ux+l->uy*m->uy,dx=m->x-l->x,dy=m->y-l->y;if(fabs(dot)<.999||fabs(-l->uy*dx+l->ux*dy)>.020)continue;
  double offset=l->ux*dx+l->uy*dy,lo=offset+fmin(dot*m->low,dot*m->high),hi=offset+fmax(dot*m->low,dot*m->high);if(lo>l->high+1.0||hi<l->low-1.0)continue;l->low=fmin(l->low,lo);l->high=fmax(l->high,hi);
 }
 unsigned lines=0;for(unsigned i=0;i<w->line_count;i++)if(st10_background_line(s,w->lines+i,p,n))w->lines[lines++]=w->lines[i];w->line_count=lines;
 memset(w->wall,0,sizeof(w->wall));unsigned valid=0;for(unsigned i=0;i<n;i++){double score=st10_wall_at(w,p[i].x,p[i].y);if(score>=.75)w->wall[p[i].index]=(unsigned char)(255*score);else p[valid++]=p[i];}
 int accepted=fv5_prepare_mode(s,p,valid,stamp,sequence,0);
 if(accepted){memcpy(s->wall,w->wall,sizeof(s->wall));s->line_count=w->line_count;memcpy(s->lines,w->lines,w->line_count*sizeof(*s->lines));}
 free(w);return accepted;
}
#endif
