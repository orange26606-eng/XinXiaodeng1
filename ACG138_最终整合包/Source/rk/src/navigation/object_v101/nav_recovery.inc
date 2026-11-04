/* nav_core.c内的仿真恢复逻辑；静态未知格与动态回波分开处理。 */

static void phase_to(nav_engine *n,nav_phase phase,uint64_t now){
    n->phase=phase;n->phase_started=now;n->clear_scans=0;
}
static void observe_motion(nav_engine *n,const lab_event *e,uint64_t now){
    const lab_pose *p=&e->data.pose;
    if(n->have_motion_pose&&e->meta.time_us>n->motion_time){
        double dt=(e->meta.time_us-n->motion_time)/1e6;
        double dx=p->x_m-n->motion_pose.x_m,dy=p->y_m-n->motion_pose.y_m;
        n->measured_v=(dx*cos(n->motion_yaw)+dy*sin(n->motion_yaw))/dt;
        n->measured_w=wrap(p->yaw_rad-n->motion_yaw)/dt;
        if(hypot(dx,dy)/dt<.008&&fabs(n->measured_w)<.04){if(n->still_samples<20)++n->still_samples;}
        else n->still_samples=0;
        if(n->episode&&n->measured_v<-.001){
            n->backup_total+=hypot(dx,dy); /* 后退后计入制动滑行。 */
            if(n->reverse_attempts)n->backup_progress=hypot(p->x_m-n->backup_start.x_m,p->y_m-n->backup_start.y_m);
            n->flow_forward_progress=0;n->flow_forward_origin=(nav_xy){p->x_m,p->y_m};
        }
        int plausible=hypot(dx,dy)<=2*n->c.max_speed_m_s*dt+.02&&fabs(wrap(p->yaw_rad-n->motion_yaw))<=2*n->c.max_yaw_rate_rad_s*dt+.03;
        if(n->episode&&!plausible){n->flow_forward_progress=0;n->flow_forward_origin=(nav_xy){p->x_m,p->y_m};}
        if(n->episode&&plausible&&n->measured_v>.025&&n->cmd.linear_m_s>.025)n->flow_forward_progress+=hypot(dx,dy);
    }
    n->motion_pose=(nav_xy){p->x_m,p->y_m};n->motion_yaw=p->yaw_rad;
    n->motion_time=e->meta.time_us;n->have_motion_pose=1;
    int escaped=n->c.passage.continuous?(n->flow_forward_progress>=.4&&hypot(p->x_m-n->flow_forward_origin.x_m,p->y_m-n->flow_forward_origin.y_m)>.25):hypot(p->x_m-n->episode_origin.x_m,p->y_m-n->episode_origin.y_m)>.4;
    if(n->episode&&escaped&&
       n->measured_v>.025&&(n->phase==NAV_PHASE_TRACK||n->phase==NAV_PHASE_PROBE)){
       n->episode=0;n->flow_extended_attempts=n->flow_backup_extended=0;n->reverse_attempts=0;n->backup_total=0;n->backup_progress=0;n->turn_total=0;n->flow_backup_counted=0;
    }
    (void)now;
}
static int static_free(const nav_engine *n,double x,double y){
    int k=index_at(n,x,y);return k>=0&&!(n->blocked[k]&1);
}
static int position_safe(const nav_engine *n,double x,double y,const nav_xy *points,unsigned count){
    if(!static_free(n,x,y))return 0;
    double r=n->radius+.015;
    for(unsigned j=0;j<count;++j)if(hypot(x-points[j].x_m,y-points[j].y_m)<=r)return 0;
    return 1;
}
static int trajectory_safe(const nav_engine *n,const lab_pose *p,double v,double w,double horizon,
                           const nav_xy *points,unsigned count){
    double x=p->x_m,y=p->y_m,a=p->yaw_rad;
    unsigned steps=(unsigned)ceil(horizon/.025);if(steps>200)steps=200;
    for(unsigned i=0;i<=steps;++i){
        if(!position_safe(n,x,y,points,count))return 0;
        advance(&x,&y,&a,v,w,.025);
    }
    return 1;
}
/* 核对实际残余运动，不只看最后请求；异常跳变或定位突变不授予反向许可。
 */
static int braking_safe(const nav_engine *n,const lab_pose *p,const nav_xy *points,unsigned count){
    double v=n->measured_v,w=n->measured_w,x=p->x_m,y=p->y_m,a=p->yaw_rad;
    if(fabs(v)>1.1||fabs(w)>4)return 0;
    for(unsigned i=0;i<200;++i){
        if(!position_safe(n,x,y,points,count))return 0;
        if(fabs(v)<1e-5&&fabs(w)<1e-5)return 1;
        advance(&x,&y,&a,v,w,.025);
        v=approach(v,0,n->c.brake_m_s2*.025);
        w=approach(w,0,n->c.angular_acceleration_rad_s2*.025);
    }
    return 0;
}
static void dynamic_layer(nav_engine *n,const lab_pose *p,const nav_xy *points,unsigned count,uint64_t now,int new_scan){
    if(!n->c.enable_recovery)return;
    int changed=n->dirty!=0;
    for(unsigned j=0;j<n->obstacle_n;){
        if(n->obstacles[j].expires<=now){n->obstacles[j]=n->obstacles[--n->obstacle_n];changed=1;}
        else ++j;
    }
    if(new_scan)for(unsigned j=0;j<count;++j){
        double d=hypot(points[j].x_m-p->x_m,points[j].y_m-p->y_m);
        int k=index_at(n,points[j].x_m,points[j].y_m);
        if(k<0||d>2.5||n->cells[k]<0||n->cells[k]>=(int)n->c.occupancy_threshold)continue;
        /* 静态墙边界回波已由栅格表示。 */
        if(n->dist[k]<=2)continue;
        unsigned z=0;for(;z<n->obstacle_n;++z)if(hypot(points[j].x_m-n->obstacles[z].p.x_m,
            points[j].y_m-n->obstacles[z].p.y_m)<.12)break;
        if(z==n->obstacle_n){if(z==32)continue;n->obstacles[z].p=points[j];++n->obstacle_n;changed=1;}
        n->obstacles[z].expires=now+2000000;
    }
    if(!changed)return;
    for(unsigned k=0;k<n->n;++k)n->blocked[k]&=1;
    int cells=(int)ceil((n->radius+n->res*.71)/n->res);
    for(unsigned z=0;z<n->obstacle_n;++z){
        int k=index_at(n,n->obstacles[z].p.x_m,n->obstacles[z].p.y_m);if(k<0)continue;
        int cx=k%(int)n->w,cy=k/(int)n->w;
        for(int dy=-cells;dy<=cells;++dy)for(int dx=-cells;dx<=cells;++dx){
            int x=cx+dx,y=cy+dy;if(x<0||y<0||x>=(int)n->w||y>=(int)n->h)continue;
            nav_xy q=cell_xy(n,y*(int)n->w+x);
            if(hypot(q.x_m-n->obstacles[z].p.x_m,q.y_m-n->obstacles[z].p.y_m)<=n->radius+n->res*.71)
                n->blocked[y*n->w+x]|=2;
        }
    }
    n->dirty=1;
}
static int rear_observed(nav_engine *n,const lab_pose *p,const nav_xy *points,unsigned count){
    uint32_t bins=0;double rear_x=n->c.mount_x_m-n->c.lidar_to_rear_m,min=30;
    double ca=cos(p->yaw_rad),sa=sin(p->yaw_rad),half=n->c.vehicle_width_m/2+n->c.safety_margin_m;
    for(unsigned j=0;j<count;++j){
        double dx=points[j].x_m-p->x_m,dy=points[j].y_m-p->y_m;
        double x=ca*dx+sa*dy,y=-sa*dx+ca*dy;
        double a=atan2(y-n->c.mount_y_m,x-n->c.mount_x_m);if(a<0)a+=2*PI;
        double deg=a*180/PI;
        if(deg>=120&&deg<240){unsigned b=(unsigned)((deg-120)/10);if(b<12)bins|=1u<<b;}
        if(x<0&&fabs(y)<=half){double d=rear_x-x;if(d<min)min=fmax(0,d);}
    }
    int covered=bins==4095;
    if(n->c.rectangular_footprint&&n->c.passage.continuous){
      /* C1稀疏回波可能缺少孤立10度角桶，此处仅预筛；反向仍检查近期完整车体矩形扫掠及制动余量。

 */
      uint32_t missing=(~bins)&4095u;unsigned holes=0;for(uint32_t q=missing;q;q>>=1)holes+=q&1u;
      covered=holes<=2&&!(missing&(missing<<1));
    }
    n->rear_clearance=covered?min:-1;
    return covered;
}
static int reverse_clear(nav_engine *n,const lab_pose *p,const nav_xy *points,unsigned count,double distance){
    if(!n->c.require_scan||!rear_observed(n,p,points,count))return 0;
    double reserve=.05*.05/(2*n->c.brake_m_s2)+n->c.safety_margin_m+.015;
    if(n->rear_clearance<distance+reserve)return 0;
    return trajectory_safe(n,p,-.05,0,(distance+reserve)/.05,points,count);
}
static double path_distance(const nav_engine *n,double x,double y){
    if(n->path_n<2)return 0;
    double best=DBL_MAX;unsigned end=n->segment+4;if(end>=n->path_n)end=n->path_n-1;
    for(unsigned i=n->segment;i<end;++i){
        nav_xy a=n->path[i],b=n->path[i+1];double dx=b.x_m-a.x_m,dy=b.y_m-a.y_m,d=dx*dx+dy*dy;
        double t=d?clamp(((x-a.x_m)*dx+(y-a.y_m)*dy)/d,0,1):0;
        best=fmin(best,hypot(x-a.x_m-t*dx,y-a.y_m-t*dy));
    }
    return best==DBL_MAX?0:best;
}
/* 使用27次有界格点尝试，不直接执行一条未验证追踪命令。 */
static int local_motion(nav_engine *n,const lab_pose *p,nav_xy target,double dt,
                        const nav_xy *points,unsigned count,double *out_v,double *out_w){
    double best=DBL_MAX,initial=hypot(target.x_m-p->x_m,target.y_m-p->y_m);
    double initial_heading=fabs(wrap(atan2(target.y_m-p->y_m,target.x_m-p->x_m)-p->yaw_rad));
    static const double speeds[]={.08,.05,.025},rates[]={0,.2,-.2,.4,-.4,.6,-.6,.8,-.8};
    if(n->measured_v<-.008)return 0; /* 后退切换前进前先制动。 */
    if(!braking_safe(n,p,points,count))return 0;
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<9;++j){
        if(n->turn_dir&&rates[j]*n->turn_dir<-.01)continue;
        double v=approach(n->cmd.linear_m_s,fmin(speeds[i],n->c.max_speed_m_s),n->c.acceleration_m_s2*dt);
        double w=approach(n->cmd.angular_rad_s,clamp(rates[j],-n->c.max_yaw_rate_rad_s,n->c.max_yaw_rate_rad_s),n->c.angular_acceleration_rad_s2*dt);
        if(v<=.001||!trajectory_safe(n,p,v,w,1.0+v/n->c.brake_m_s2,points,count))continue;
        double x=p->x_m,y=p->y_m,a=p->yaw_rad;advance(&x,&y,&a,v,w,1);
        double d=hypot(target.x_m-x,target.y_m-y),heading=fabs(wrap(atan2(target.y_m-y,target.x_m-x)-a));
        if(initial-d<.006&&initial_heading-heading<.035)continue;
        double score=3*d+2*path_distance(n,x,y)+.22*heading+.04*fabs(w-n->cmd.angular_rad_s)-.3*v;
        if(score<best){best=score;*out_v=v;*out_w=w;}
    }
    if(best==DBL_MAX)return 0;
    if(!n->turn_dir&&fabs(*out_w)>.1)n->turn_dir=*out_w>0?1:-1;
    return 1;
}
static lab_status recovering_command(nav_engine *n,uint64_t now,nav_reason why,double v,double w,nav_command *out){
    n->waiting=0;n->state=NAV_TRACKING;n->reason=why;command(n,now,v,w,out);return LAB_OK;
}
static void begin_episode(nav_engine *n,const lab_pose *p,uint64_t now,nav_reason why){
    if(!n->episode){n->episode=1;n->episode_started=now;n->episode_origin=(nav_xy){p->x_m,p->y_m};
        n->reverse_attempts=0;n->backup_total=0;n->turn_total=0;n->turn_dir=0;}
    n->blocking_reason=why;n->waiting=0;phase_to(n,NAV_PHASE_AVOID,now);
}
static lab_status exhausted(nav_engine *n,uint64_t now,nav_command *out){
    phase_to(n,NAV_PHASE_BLOCKED,now);return stop(n,now,NAV_FAILED,NAV_RECOVERY_EXHAUSTED,LAB_OK,out);
}
static lab_status recovery_step(nav_engine *n,uint64_t now,const lab_pose *p,nav_xy target,double dt,
                               const nav_xy *points,unsigned count,int new_scan,nav_command *out){
    if(now-n->episode_started>=20000000)return exhausted(n,now,out);
    double v=0,w=0,alpha=wrap(atan2(target.y_m-p->y_m,target.x_m-p->x_m)-p->yaw_rad);
    switch(n->phase){
    case NAV_PHASE_AVOID:
        if(local_motion(n,p,target,dt,points,count,&v,&w)){
            phase_to(n,NAV_PHASE_RECOVER,now);return recovering_command(n,now,NAV_LOCAL_AVOID,v,w,out);
        }
        if(now-n->phase_started>=300000)phase_to(n,NAV_PHASE_REPLAN,now);
        return stop(n,now,NAV_WAITING,NAV_LOCAL_AVOID,LAB_OK,out);
    case NAV_PHASE_REPLAN:{
        if(now-n->last_plan<500000)return stop(n,now,NAV_WAITING,NAV_REPLANNING,LAB_OK,out);
        n->failures[n->failure_next%8].p=(nav_xy){p->x_m+.25*cos(p->yaw_rad),p->y_m+.25*sin(p->yaw_rad)};
        n->failures[n->failure_next++%8].expires=now+5000000;
        n->last_plan=now;nav_reason why=plan(n,(nav_xy){p->x_m,p->y_m});n->dirty=0;
        if(why==NAV_NO_PATH){
            n->blocking_reason=why;
            phase_to(n,n->reverse_attempts<2?NAV_PHASE_BACK_CHECK:NAV_PHASE_WAIT,now);
            return stop(n,now,NAV_WAITING,why,LAB_OK,out);
        }
        if(why==NAV_GOAL_BLOCKED||why==NAV_GOAL_OUTSIDE||why==NAV_START_BLOCKED||why==NAV_SEARCH_LIMIT||why==NAV_PATH_LIMIT){
            n->blocking_reason=why;phase_to(n,NAV_PHASE_WAIT,now);return stop(n,now,NAV_WAITING,why,LAB_OK,out);
        }
        n->turn_start_yaw=p->yaw_rad;
        nav_xy aimed=lookahead(n,(nav_xy){p->x_m,p->y_m});
        double desired=wrap(atan2(aimed.y_m-p->y_m,aimed.x_m-p->x_m)-p->yaw_rad);
        if(!n->turn_dir)n->turn_dir=desired>=0?1:-1;
        phase_to(n,NAV_PHASE_TURN,now);return stop(n,now,NAV_WAITING,NAV_REPLANNING,LAB_OK,out);
    }
    case NAV_PHASE_TURN:{
        double turned=fabs(wrap(p->yaw_rad-n->turn_start_yaw));
        if(fabs(alpha)<.12||turned>=PI/6||now-n->phase_started>=2000000||n->turn_total+turned>=PI/2){
            n->turn_total+=turned;phase_to(n,NAV_PHASE_BACK_CHECK,now);
            if(fabs(alpha)<.12&&local_motion(n,p,target,dt,points,count,&v,&w)){
                phase_to(n,NAV_PHASE_RECOVER,now);return recovering_command(n,now,NAV_LOW_RECOVERY,v,w,out);
            }
            return stop(n,now,NAV_WAITING,NAV_BACK_CHECK,LAB_OK,out);
        }
        if(fabs(n->measured_v)>.008||(fabs(n->cmd.angular_rad_s)<.001&&n->still_samples<3))
            return stop(n,now,NAV_WAITING,NAV_TURN_RECOVERY,LAB_OK,out);
        w=approach(n->cmd.angular_rad_s,n->turn_dir*fmin(.4,n->c.max_yaw_rate_rad_s),n->c.angular_acceleration_rad_s2*dt);
        if(!braking_safe(n,p,points,count)||!trajectory_safe(n,p,0,w,.5,points,count)){
            n->turn_total+=turned;phase_to(n,NAV_PHASE_BACK_CHECK,now);return stop(n,now,NAV_WAITING,NAV_BACK_CHECK,LAB_OK,out);
        }
        return recovering_command(n,now,NAV_TURN_RECOVERY,0,w,out);
    }
    case NAV_PHASE_BACK_CHECK:
        if(n->still_samples<3||now-n->phase_started<200000)return stop(n,now,NAV_WAITING,NAV_BACK_CHECK,LAB_OK,out);
        if(n->reverse_attempts>=2||n->backup_total>=.29){
            phase_to(n,NAV_PHASE_WAIT,now);return stop(n,now,NAV_WAITING,NAV_RECOVERY_EXHAUSTED,LAB_OK,out);
        }
        if(!reverse_clear(n,p,points,count,.10)){
            n->blocking_reason=n->rear_clearance<0?NAV_REAR_UNOBSERVED:NAV_REAR_BLOCKED;
            phase_to(n,NAV_PHASE_WAIT,now);return stop(n,now,NAV_WAITING,n->blocking_reason,LAB_OK,out);
        }
        ++n->reverse_attempts;n->backup_start=(nav_xy){p->x_m,p->y_m};n->backup_progress=0;
        phase_to(n,NAV_PHASE_BACKUP,now);
        return stop(n,now,NAV_WAITING,NAV_BACK_CHECK,LAB_OK,out);
    case NAV_PHASE_BACKUP:{
        n->backup_progress=hypot(p->x_m-n->backup_start.x_m,p->y_m-n->backup_start.y_m);
        double reserve=n->measured_v*n->measured_v/(2*n->c.brake_m_s2)+.005;
        if(n->backup_progress>=.10-reserve||n->backup_progress>=.15||n->backup_total>=.29||now-n->phase_started>=4000000){
            phase_to(n,NAV_PHASE_REPLAN,now);return stop(n,now,NAV_WAITING,NAV_REPLANNING,LAB_OK,out);
        }
        if(!reverse_clear(n,p,points,count,fmin(.10-n->backup_progress,.025))||!braking_safe(n,p,points,count)){
            n->blocking_reason=n->rear_clearance<0?NAV_REAR_UNOBSERVED:NAV_REAR_BLOCKED;
            phase_to(n,NAV_PHASE_WAIT,now);return stop(n,now,NAV_WAITING,n->blocking_reason,LAB_OK,out);
        }
        v=approach(n->cmd.linear_m_s,-fmin(.05,n->c.max_speed_m_s),n->c.acceleration_m_s2*dt);
        return recovering_command(n,now,NAV_BACKING,v,0,out);
    }
    case NAV_PHASE_WAIT:
        if(now-n->phase_started>=500000){phase_to(n,NAV_PHASE_AVOID,now);n->dirty=1;}
        return stop(n,now,NAV_WAITING,n->blocking_reason,LAB_OK,out);
    case NAV_PHASE_RECOVER:
        if(n->path_n<2){phase_to(n,NAV_PHASE_REPLAN,now);return stop(n,now,NAV_WAITING,NAV_REPLANNING,LAB_OK,out);}
        if(local_motion(n,p,target,dt,points,count,&v,&w)){
            if(new_scan&&n->clear_scans<3)++n->clear_scans;
            if(n->clear_scans>=3&&now-n->phase_started>=300000){
                n->phase=NAV_PHASE_TRACK;n->turn_dir=0;n->blocking_reason=NAV_REASON_NONE;
            }
            return recovering_command(n,now,NAV_LOW_RECOVERY,v,w,out);
        }
        phase_to(n,NAV_PHASE_REPLAN,now);return stop(n,now,NAV_WAITING,NAV_REPLANNING,LAB_OK,out);
    default:return exhausted(n,now,out);
    }
}
const char *nav_phase_text(uint32_t phase){
    static const char *names[]={"TRACK","AVOID","REPLAN","TURN","BACK_CHECK","BACKING","RECOVER","WAIT","BLOCKED","DETOUR_LEFT","DETOUR_RIGHT","ALIGN","REJOIN","PROBING"};
    return phase<sizeof(names)/sizeof(names[0])?names[phase]:"UNKNOWN";
}
