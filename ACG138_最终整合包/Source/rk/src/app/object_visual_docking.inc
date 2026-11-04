/* 静止目标视觉停靠支持低矮物体，不要求地图或雷达回波。几何回退明确为粗估，不刷新旧数据源。
 */
static int ovd_text(char*out,size_t cap,const char*s){size_t n=0;if(!s){if(cap)*out=0;return 1;}while(s[n]){unsigned char c=(unsigned char)s[n];if(n+1>=cap||c<32||c>126||c=='"'||c=='\\'||c=='|')return 0;out[n]=(char)c;++n;}out[n]=0;return 1;}
static int ovd_scalar(const char*s,double*out){char*end;double d=strtod(s,&end);while(*end==' '||*end=='\t'||*end=='\r'||*end=='\n')++end;if(end==s||*end||!isfinite(d))return 0;*out=d;return 1;}
static void ovd_path(char*out,size_t cap,const char*config,const char*relative){
 if(relative[0]=='/'||(relative[0]&&relative[1]==':')){snprintf(out,cap,"%s",relative);return;}
 const char*slash=strrchr(config,'/'),*other=strrchr(config,'\\');if(other&&(!slash||other>slash))slash=other;
 if(slash)snprintf(out,cap,"%.*s/%s",(int)(slash-config),config,relative);else snprintf(out,cap,"%s",relative);
}
static int ovd_json_array(const char*json,const char*key,double*out,unsigned count){
 const char*p=strstr(json,key);if(!p||strstr(p+strlen(key),key))return 0;p+=strlen(key);while(*p==' '||*p=='\n'||*p=='\r'||*p=='\t')++p;if(*p++!=':')return 0;while(*p==' '||*p=='\n'||*p=='\r'||*p=='\t')++p;if(*p++!='[')return 0;
 for(unsigned k=0;k<count;k++){while(*p==' '||*p=='\n'||*p=='\r'||*p=='\t')++p;char*end;out[k]=strtod(p,&end);if(end==p||!isfinite(out[k]))return 0;p=end;while(*p==' '||*p=='\n'||*p=='\r'||*p=='\t')++p;if(*p++!=(k+1<count?',':']'))return 0;}return 1;
}
static int ovd_extrinsics(object_visual_geometry*g,const char*path){
 FILE*f=fopen(path,"rb");if(!f)return 0;char data[16385];size_t n=fread(data,1,sizeof data-1,f);int overflow=!feof(f);fclose(f);if(overflow)return 0;data[n]=0;
 if(!strstr(data,"\"left_eye\": 0")&&!strstr(data,"\"left_eye\":0"))return 0;
 if(!strstr(data,"\"right_eye\": 1")&&!strstr(data,"\"right_eye\":1"))return 0;
 if(!ovd_json_array(data,"\"R\"",g->stereo_R,9)||!ovd_json_array(data,"\"T\"",g->stereo_T,3))return 0;
 double norm=0;for(unsigned k=0;k<3;k++)norm+=g->stereo_T[k]*g->stereo_T[k];if(norm<1e-6||norm>.25)return 0;
 for(unsigned row=0;row<3;row++)for(unsigned col=0;col<3;col++){double dot=0;for(unsigned k=0;k<3;k++)dot+=g->stereo_R[row*3+k]*g->stereo_R[col*3+k];if(fabs(dot-(row==col?1.:0.))>.02)return 0;}
 double*a=g->stereo_R,det=a[0]*(a[4]*a[8]-a[5]*a[7])-a[1]*(a[3]*a[8]-a[5]*a[6])+a[2]*(a[3]*a[7]-a[4]*a[6]);return fabs(det-1)<.02;
}
static void ovd_right_to_left(const object_visual_geometry*g,const double right[3],int direction,double left[3]){
 for(unsigned j=0;j<3;j++){left[j]=0;for(unsigned k=0;k<3;k++)left[j]+=g->stereo_R[k*3+j]*(right[k]-(direction?0:g->stereo_T[k]));}
}
int object_dual_task_geometry_load(object_dual_task*t,const char*file){
 if(!t)return 0;object_visual_geometry*g=&t->visual_geometry;memset(g,0,sizeof(*g));g->loaded=1;snprintf(g->reason,sizeof g->reason,"GEOMETRY_CONFIG_UNAVAILABLE");
 if(!file||!*file)return 0;FILE*f=fopen(file,"rb");if(!f)return 0;
 char line[1024],mono[2][512]={{0}},extrinsics[512]={0};unsigned seen=0;
 while(fgets(line,sizeof line,f)){char*k=line;while(*k==' '||*k=='\t')++k;if(*k=='#'||!*k)continue;char*eq=strchr(k,'=');if(!eq)continue;*eq++=0;char*end=k+strlen(k);while(end>k&&(end[-1]==' '||end[-1]=='\t'))*--end=0;while(*eq==' '||*eq=='\t')++eq;end=eq+strlen(eq);while(end>eq&&(end[-1]=='\n'||end[-1]=='\r'||end[-1]==' '||end[-1]=='\t'))*--end=0;double d;
  if(!strcmp(k,"stereo_extrinsics_file")){ovd_path(extrinsics,sizeof extrinsics,file,eq);continue;}
  if(!strcmp(k,"mono_eye0_file")){ovd_path(mono[0],sizeof mono[0],file,eq);continue;}if(!strcmp(k,"mono_eye1_file")){ovd_path(mono[1],sizeof mono[1],file,eq);continue;}
  if(!ovd_scalar(eq,&d))continue;
  if(!strcmp(k,"camera_to_body_verified"))g->verified=d==1;
  else if(!strcmp(k,"camera_x_m")){g->x=d;seen|=1;}else if(!strcmp(k,"camera_y_m")){g->y=d;seen|=2;}else if(!strcmp(k,"camera_z_m")){g->z=d;seen|=4;}
  else if(!strcmp(k,"camera_roll_deg")){g->roll=d*.017453292519943295;seen|=8;}else if(!strcmp(k,"camera_pitch_deg")){g->pitch=d*.017453292519943295;seen|=16;}else if(!strcmp(k,"camera_yaw_deg")){g->yaw=d*.017453292519943295;seen|=32;}
 }fclose(f);g->configured=(seen&63)==63&&fabs(g->x)<2&&fabs(g->y)<2&&g->z>=0&&g->z<2&&fabs(g->roll)<1.5708&&fabs(g->pitch)<1.5708&&fabs(g->yaw)<=3.1416;
 for(unsigned i=0;i<2;++i){if(!mono[i][0])continue;f=fopen(mono[i],"rb");if(!f)continue;unsigned mask=0;int enabled=0,identity=-1,w=0,h=0;
  while(fgets(line,sizeof line,f)){char key[64],value[128];if(sscanf(line," %63[^=]=%127s",key,value)!=2)continue;char*e=key+strlen(key);while(e>key&&(e[-1]==' '||e[-1]=='\t'))*--e=0;double d;if(!ovd_scalar(value,&d))continue;
   if(!strcmp(key,"calibration_enabled"))enabled=d==1;else if(!strcmp(key,"eye"))identity=(int)d;else if(!strcmp(key,"image_width"))w=(int)d;else if(!strcmp(key,"image_height"))h=(int)d;
   else if(!strcmp(key,"fx")){g->fx[i]=d;mask|=1;}else if(!strcmp(key,"fy")){g->fy[i]=d;mask|=2;}else if(!strcmp(key,"cx")){g->cx[i]=d;mask|=4;}else if(!strcmp(key,"cy")){g->cy[i]=d;mask|=8;}
   else if(!strcmp(key,"k1"))g->distortion[i][0]=d;else if(!strcmp(key,"k2"))g->distortion[i][1]=d;else if(!strcmp(key,"p1"))g->distortion[i][2]=d;else if(!strcmp(key,"p2"))g->distortion[i][3]=d;else if(!strcmp(key,"k3"))g->distortion[i][4]=d;
  }fclose(f);g->mono_valid[i]=enabled&&identity==(int)i&&w==960&&h==1080&&mask==15&&g->fx[i]>100&&g->fy[i]>100&&g->fx[i]<20000&&g->fy[i]<20000;
 }
 if(extrinsics[0])g->extrinsics_valid=ovd_extrinsics(g,extrinsics);
 snprintf(g->reason,sizeof g->reason,"%s",!g->configured?"CAMERA_MOUNT_MISSING":!g->verified?"GROUND_MOUNT_UNVERIFIED":g->z<=.01?"GROUND_HEIGHT_UNAVAILABLE":!g->mono_valid[0]?"GROUND_INTRINSICS_UNAVAILABLE":"GROUND_PLANE_APPROX_AVAILABLE");return g->configured;
}
static void ovd_rotate(const object_visual_geometry*g,const double opt[3],double body[3]){
 double x=opt[2],y=-opt[0],z=-opt[1],roll=g->configured?g->roll:0,pitch=g->configured?g->pitch:0,yaw=g->configured?g->yaw:0;
 double yy=cos(roll)*y-sin(roll)*z,zz=sin(roll)*y+cos(roll)*z,xx=cos(pitch)*x+sin(pitch)*zz;body[0]=cos(yaw)*xx-sin(yaw)*yy;body[1]=sin(yaw)*xx+cos(yaw)*yy;body[2]=-sin(pitch)*x+cos(pitch)*zz;
}
static int ovd_ground(const object_visual_geometry*g,const obj_detection*d,double*bx,double*by){
 int eye=d->eye;if(!g->configured||!g->verified||g->z<=.01||eye<0||eye>1||!g->mono_valid[eye]||d->bbox[3]>=1076)return 0;
 /* 框底接地为地面几何近似。 */
 double xx=((d->bbox[0]+d->bbox[2])*.5-g->cx[eye])/g->fx[eye],yy=(d->bbox[3]-g->cy[eye])/g->fy[eye],x=xx,y=yy;const double*k=g->distortion[eye];
 for(unsigned j=0;j<6;++j){double r=x*x+y*y,rad=1+k[0]*r+k[1]*r*r+k[4]*r*r*r;if(!isfinite(rad)||fabs(rad)<.1)return 0;double dx=2*k[2]*x*y+k[3]*(r+2*x*x),dy=k[2]*(r+2*y*y)+2*k[3]*x*y;x=(xx-dx)/rad;y=(yy-dy)/rad;}
 double optical[3]={x,y,1},ray[3],origin[3]={0};if(eye==1){if(!g->extrinsics_valid)return 0;double left[3],zero[3]={0};ovd_right_to_left(g,optical,1,left);ovd_right_to_left(g,zero,0,origin);memcpy(optical,left,sizeof left);double b[3];ovd_rotate(g,origin,b);memcpy(origin,b,sizeof b);}
 ovd_rotate(g,optical,ray);if(ray[2]>=-.015)return 0;double scale=-(g->z+origin[2])/ray[2];*bx=g->x+origin[0]+scale*ray[0];*by=g->y+origin[1]+scale*ray[1];return isfinite(*bx)&&isfinite(*by)&&*bx>.03&&*bx<12;
}
static void ovd_reset_history(object_visual_dock*q,uint64_t epoch,int eye,uint32_t binding,int track_id){
 int hold=q->hold,metric=q->metric,confirmed=q->confirmed;uint64_t hold_source=q->hold_source_us;char reason[96];memcpy(reason,q->reason,sizeof reason);memset(q,0,sizeof(*q));q->epoch=epoch;q->eye=eye;q->binding=binding;q->track_id=track_id;q->initialized=1;q->hold=hold;q->metric=metric;q->confirmed=confirmed;q->hold_source_us=hold_source;if(hold)memcpy(q->reason,reason,sizeof reason);
}
static void ovd_hold(object_visual_dock*q,uint64_t stamp,int metric,const char*reason){
 if(!q->hold)q->hold_source_us=stamp;q->hold=1;if(!q->source[0])snprintf(q->source,sizeof q->source,"%s",metric?"TARGET_SURFACE":"VISUAL_CLOSEUP");if(metric)q->metric=1;snprintf(q->reason,sizeof q->reason,"%s",reason);
 if(stamp&&stamp>q->confirmation_source_us){q->confirmation_source_us=stamp;if(q->confirmations<UINT32_MAX)q->confirmations++;if(q->confirmations>=2)q->confirmed=1;}
}
static double ovd_front_gap(double bx,double by,double front){return fmax(0,hypot(bx,by)-front);}
static void ovd_metric(object_visual_dock*q,const lab_real_runtime_view*v,double bx,double by,double gap,double error,uint64_t stamp,int metric,const char*source){
 q->gap_valid=1;q->gap_m=gap;q->gap_error_m=error;q->metric_source_us=stamp;double c=cos(v->raw_odometry.yaw_rad),s=sin(v->raw_odometry.yaw_rad);q->metric_world_x=v->raw_odometry.x_m+c*bx-s*by;q->metric_world_y=v->raw_odometry.y_m+s*bx+c*by;q->metric_world_valid=1;(void)metric;snprintf(q->source,sizeof q->source,"%s",source);
}
static void ovd_update(object_dual_task*t,const obj_detection*d,const obj_frame*f,const object_target_observation_v2*ob,int current_ob,const lab_real_runtime_view*v,uint64_t now,int visual){
 object_visual_dock*q=&t->visual_dock;const object_visual_geometry*g=&t->visual_geometry;
 if(t->mode!=OBJECT_MODE_STATIC)return;
 if(d&&q->initialized&&q->track_id!=d->track_id)memset(q,0,sizeof(*q));
 if(d&&f&&d->eye>=0&&d->eye<=1&&(!q->initialized||q->epoch!=f->epoch||q->eye!=d->eye||q->binding!=d->binding_generation))ovd_reset_history(q,f->epoch,d->eye,d->binding_generation,d->track_id);
 /* 真实配对的前景表面可立即使用，包含稀疏回波。 */
 if(current_ob&&ob->surface_valid&&fabs(ob->bearing_rad)<.60&&ob->source_us>q->surface_source_us){
  q->surface_source_us=ob->source_us;
  if(ob->surface_gap_m<=t->standoff_m+.03){ovd_metric(q,v,ob->body_x_m,ob->body_y_m,ob->surface_gap_m,ob->error_m,ob->source_us,1,ob->source);ovd_hold(q,ob->source_us,1,"TARGET_SURFACE_NEAR_HOLD");}
  else if(ob->source_us>q->metric_source_us)ovd_metric(q,v,ob->body_x_m,ob->body_y_m,ob->surface_gap_m,ob->error_m,ob->source_us,1,ob->source);
 }
 uint64_t stamp=d&&d->observation_time_us?d->observation_time_us:f?f->frame_us:0,frame=d&&d->observation_frame_id?d->observation_frame_id:f?f->frame_id:0;
 if(!visual||!d||!f||!d->bbox_valid||!frame||!od_fresh(stamp,now,1500000)||d->eye<0||d->eye>1)goto finish;
 for(unsigned j=0;j<4;++j)if(!isfinite(d->bbox[j]))goto finish;
 if(d->bbox[0]<0||d->bbox[1]<0||d->bbox[2]>960||d->bbox[3]>1080||d->bbox[2]<=d->bbox[0]||d->bbox[3]<=d->bbox[1])goto finish;
 if(!q->initialized||q->epoch!=f->epoch||q->eye!=d->eye||q->binding!=d->binding_generation)ovd_reset_history(q,f->epoch,d->eye,d->binding_generation,d->track_id);
 if(frame<=q->frame_id||stamp<=q->source_us)goto finish;
 q->frame_id=frame;q->source_us=stamp;q->clipped=d->bbox[0]<=3||d->bbox[1]<=3||d->bbox[2]>=957||d->bbox[3]>=1077;
 double width=d->bbox[2]-d->bbox[0],height=d->bbox[3]-d->bbox[1],scale=sqrt(width*height),bx=0,by=0;int sample_gap=0,sample_metric=0,sample_kind=0;uint64_t metric_stamp=0;
 uint64_t pose_delta=v->odometry_time_us>stamp?v->odometry_time_us-stamp:stamp-v->odometry_time_us;int pose_aligned=pose_delta<=200000;
 /* 没有雷达回波时也可使用新鲜双目，其点表示可见表面，不是箱体隐藏中心；保留几何及安装误差。
 */
 uint64_t depth_stamp=d->depth_time_us?d->depth_time_us:stamp;
 if(d->depth_valid&&(d->geometry_eye==0||d->geometry_eye==1)&&od_fresh(depth_stamp,now,1500000)&&isfinite(d->x)&&isfinite(d->y)&&isfinite(d->z)&&d->z>.05&&d->z<12&&isfinite(d->error)&&d->error>=0&&d->error<=fmax(.30,.30*d->z)){
  double optical[3]={d->x,d->y,d->z},body[3];int right_unknown=d->geometry_eye==1&&!g->extrinsics_valid;
  if(d->geometry_eye==1&&g->extrinsics_valid){double left[3];ovd_right_to_left(g,optical,0,left);memcpy(optical,left,sizeof left);}
  ovd_rotate(g,optical,body);bx=(g->configured?g->x:0)+body[0];by=(g->configured?g->y:0)+body[1];
  if(right_unknown){double range=hypot(d->x,d->z);bx=(g->configured?g->x:0)+range*cos(d->bearing);by=(g->configured?g->y:0)+range*sin(d->bearing);}
  int depth_pose=depth_stamp==stamp&&pose_aligned;
  if(!depth_pose)for(unsigned j=0;j<(unsigned)q->count;++j){const object_visual_dock_sample*a=q->samples+j;
   if(a->pose_valid&&a->source_us==depth_stamp&&(!d->depth_frame_id||a->frame_id==d->depth_frame_id)){
    double wx=a->x+cos(a->yaw)*bx-sin(a->yaw)*by,wy=a->y+sin(a->yaw)*bx+cos(a->yaw)*by,dx=wx-v->raw_odometry.x_m,dy=wy-v->raw_odometry.y_m;
    bx=cos(v->raw_odometry.yaw_rad)*dx+sin(v->raw_odometry.yaw_rad)*dy;by=-sin(v->raw_odometry.yaw_rad)*dx+cos(v->raw_odometry.yaw_rad)*dy;depth_pose=1;break;
   }
  }
  if(depth_pose&&(!right_unknown||hypot(d->x,d->z)<=t->standoff_m+.10)&&bx>.03&&fabs(atan2(by,bx))<.60){double gap=ovd_front_gap(bx,by,t->front_m);sample_gap=1;sample_kind=1;sample_metric=g->configured&&g->verified&&!right_unknown;metric_stamp=depth_stamp;
   if(depth_stamp>q->metric_source_us)ovd_metric(q,v,bx,by,gap,fmax(.05,d->error)+(right_unknown?.25:sample_metric?0:.10),depth_stamp,sample_metric,right_unknown?"STEREO_RIGHT_ORIGIN_UNKNOWN_COARSE":sample_metric?"STEREO_SURFACE":"STEREO_SURFACE_APPROX_MOUNT");
  }
 }
 if(!sample_gap&&pose_aligned&&ovd_ground(g,d,&bx,&by)&&fabs(atan2(by,bx))<.60){sample_gap=1;sample_kind=2;metric_stamp=stamp;double gap=ovd_front_gap(bx,by,t->front_m);if(stamp>q->metric_source_us)ovd_metric(q,v,bx,by,gap,.10+.08*hypot(bx,by),stamp,0,"GROUND_BOX_CONTACT_APPROX");}
 q->growth_ratio=1;q->advance_m=0;double scale_range=0;int scale_valid=0;uint64_t best_span=0;
 for(unsigned j=0;j<(unsigned)q->count;++j){const object_visual_dock_sample*a=q->samples+j;if(!a->pose_valid||a->clipped||stamp<=a->source_us||stamp-a->source_us<200000||stamp-a->source_us>1500000||q->clipped||fabs(od_angle(v->raw_odometry.yaw_rad-a->yaw))>.10)continue;
  double ratio=scale/a->scale,aspect=(width/height)/(a->width/a->height);if(!isfinite(ratio)||aspect<.75||aspect>1.333)continue;
  double heading=a->yaw+a->bearing,dx=v->raw_odometry.x_m-a->x,dy=v->raw_odometry.y_m-a->y,advance=dx*cos(heading)+dy*sin(heading),lateral=fabs(-dx*sin(heading)+dy*cos(heading));
  if(advance<.025||lateral>.06||ratio<1.06||ratio>3.5)continue;double range=advance/(ratio-1);if(!isfinite(range)||range<.05||range>8)continue;
  if(!scale_valid||stamp-a->source_us>best_span){q->growth_ratio=ratio;q->advance_m=advance;scale_range=range;scale_valid=1;best_span=stamp-a->source_us;}
 }
 if(!sample_gap&&scale_valid&&pose_aligned){double bearing=d->bearing_valid?d->bearing:t->bearing_rad;bx=(g->configured?g->x:0)+scale_range*cos(bearing);by=(g->configured?g->y:0)+scale_range*sin(bearing);metric_stamp=stamp;sample_gap=1;sample_kind=3;q->camera_range_m=scale_range;if(stamp>q->metric_source_us)ovd_metric(q,v,bx,by,ovd_front_gap(bx,by,t->front_m),.15+.20*scale_range,stamp,0,"MONOCULAR_SCALE_ODOM_COARSE");}
 /* 单个错误近距帧不能永久锁住未完成任务；仅两笔新的未裁边双目距离可解除暂定保持。任务完成后保持终止，直至显式新请求。

 */
 if(q->hold&&!q->confirmed&&sample_kind==1&&d->geometry_eye==0&&!q->clipped&&q->gap_valid&&q->metric_source_us==metric_stamp&&metric_stamp>q->far_source_us){
  q->far_source_us=metric_stamp;
  if(q->gap_m>t->standoff_m+fmax(.15,2*q->gap_error_m)){if(++q->far_count>=2){q->hold=q->metric=q->confirmations=0;q->confirmation_source_us=q->hold_source_us=0;q->far_count=0;snprintf(q->reason,sizeof q->reason,"TENTATIVE_NEAR_REJECTED_TWO_NEW_FAR_STEREO");}}
  else q->far_count=0;
 }
 /* 粗距可用于停车并报告粗略停靠，不能确认精确的 20cm 停靠距离。 */
 if(sample_gap&&q->gap_valid&&q->metric_source_us==metric_stamp&&q->gap_m<=t->standoff_m+.03)ovd_hold(q,metric_stamp,sample_metric,sample_metric?"VISUAL_METRIC_SURFACE_NEAR":"VISUAL_COARSE_NEAR_HOLD");
 if(q->hold&&!q->confirmed&&!sample_gap&&q->gap_valid&&q->gap_m<=t->standoff_m+.03){q->metric=0;ovd_hold(q,stamp,0,"STATIC_NEAR_VISUAL_REOBSERVED_COARSE");}
 int growing=q->growth_ratio>=1.20&&q->advance_m>=.03;
 int big=height/1080.>=.80||width/960.>=.65||(width*height)/(960.*1080.)>=.40;
 int far=q->gap_valid&&od_fresh(q->metric_source_us,now,1500000)&&q->gap_m>t->standoff_m+.15;
 if(growing&&big&&q->gap_valid&&q->gap_m<=t->standoff_m+.20&&!far)ovd_hold(q,stamp,0,"TARGET_EXPANSION_VISUAL_NEAR_HOLD");
 /* 水瓶极近特写仅形成粗估，箱子单纯占满画面不走此分支，因实际尺寸可能本来就大。
 */
 if(!strcmp(d->label,"bottle")&&!far&&q->clipped&&height/1080.>=.94&&(width*height)/(960.*1080.)>=.45)ovd_hold(q,stamp,0,"BOTTLE_EXTREME_CLOSEUP_COARSE_HOLD");
 /* 超声波仅为辅助证据，忽略的0值不形成近障；重复超声波记录不增加视觉确认次数。
 */
 if(big&&!far&&fabs(d->bearing)<.35&&v->feedback.time_us&&od_fresh(v->feedback.time_us,now,600000)&&(v->feedback.sonar_valid&1)&&(v->feedback.sonar_fresh&1)&&!(v->feedback.sonar_raw_zero_mask&1)&&v->feedback.sonar_mm[0]>0&&v->feedback.sonar_mm[0]<=1000*(t->standoff_m+.03))ovd_hold(q,stamp,0,"VISUAL_TARGET_WITH_FRONT_SONAR_NEAR");
 if(q->count<(int)OBJECT_VISUAL_DOCK_SAMPLES)q->count++;
 q->samples[q->next]=(object_visual_dock_sample){frame,stamp,width,height,scale,v->raw_odometry.x_m,v->raw_odometry.y_m,v->raw_odometry.yaw_rad,d->bearing,q->clipped,pose_aligned};q->next=(q->next+1)%OBJECT_VISUAL_DOCK_SAMPLES;
 if(!q->hold&&!sample_gap)snprintf(q->reason,sizeof q->reason,"%s",d->depth_valid&&d->geometry_eye==1&&!g->extrinsics_valid?"RIGHT_DEPTH_ORIGIN_UNVERIFIED":big?"LARGE_IMAGE_ALONE_NOT_ARRIVAL":!g->configured||!g->verified||g->z<=.01?"GROUND_UNVERIFIED_WAIT_STEREO_OR_SCALE":"WAIT_TARGET_DISTANCE_EVIDENCE");
 finish:
 /* 静止表面记忆按实际编码器位移推进，不重新发布为相机或深度新观测，完成质量标为粗估。
 */
 if(q->metric_world_valid&&q->metric_source_us&&v->odometry_time_us>q->metric_source_us){
  double dx=q->metric_world_x-v->raw_odometry.x_m,dy=q->metric_world_y-v->raw_odometry.y_m,bearing=od_angle(atan2(dy,dx)-v->raw_odometry.yaw_rad);
  q->gap_m=fmax(0,hypot(dx,dy)-t->front_m);
  if(q->gap_m<=t->standoff_m+.03&&fabs(bearing)<.60&&!q->hold)ovd_hold(q,v->odometry_time_us,0,"STATIC_GEOMETRY_ODOMETRY_COARSE_NEAR");
 }
 if(!q->last_log_us||now-q->last_log_us>=250000){fprintf(stderr,"STATIC_DOCK epoch=%llu target=%d frame=%llu image_us=%llu eye=%d gap_valid=%d gap_m=%.5f error_m=%.5f measure_us=%llu source=%s growth=%.5f measured_advance_m=%.5f clipped=%d hold=%d metric=%d confirmations=%u geometry=%s reason=%s\n",(unsigned long long)t->epoch,t->track_id,(unsigned long long)q->frame_id,(unsigned long long)q->source_us,q->eye,q->gap_valid,q->gap_m,q->gap_error_m,(unsigned long long)q->metric_source_us,q->source,q->growth_ratio,q->advance_m,q->clipped,q->hold,q->metric,q->confirmations,g->reason,q->reason);q->last_log_us=now;}
 t->near_latch=q->hold;t->near_hold=q->hold;t->near_metric=q->metric&&q->hold;snprintf(t->near_reason,sizeof t->near_reason,"%s",q->reason);
 if(q->metric_world_valid&&(q->metric_source_us>t->last_source_us||q->hold)){t->target_x=q->metric_world_x;t->target_y=q->metric_world_y;t->previous_x=t->target_x;t->previous_y=t->target_y;t->have_target=t->range_valid=1;t->last_source_us=q->metric_source_us;snprintf(t->source,sizeof t->source,"%s",q->source);}
}
