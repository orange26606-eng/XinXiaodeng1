// 位姿投影区域只作为搜索先验，不代表已测像素或距离。
static void project_reacquisition_region(Tracker&tr,const Vision&vision){
 tr.reacquire_image_prediction_valid.fill(false);if(!tr.reacquire_region_valid||!tr.reacquire_anchor_ray_valid)return;
 double direction[3]={-std::sin(tr.reacquire_region_bearing),std::tan(tr.reacquire_anchor_pitch)*std::cos(tr.reacquire_region_bearing),std::cos(tr.reacquire_region_bearing)};
 for(int eye=0;eye<2;eye++){const Mono&m=vision.mono[eye];if(!m.valid)continue;double p[3]={direction[0],direction[1],direction[2]};if(vision.extrinsics.valid&&eye!=vision.extrinsics.left_eye){for(int i=0;i<3;i++){p[i]=0;for(int j=0;j<3;j++)p[i]+=vision.extrinsics.R[i*3+j]*direction[j];}}else if(!vision.extrinsics.valid&&eye!=0)continue;
  if(p[2]<=1e-4)continue;double x=p[0]/p[2],y=p[1]/p[2],r=x*x+y*y,radial=1+m.D[0]*r+m.D[1]*r*r+m.D[4]*r*r*r;
  double u=m.fx*(x*radial+2*m.D[2]*x*y+m.D[3]*(r+2*x*x))+m.cx,v=m.fy*(y*radial+m.D[2]*(r+2*y*y)+2*m.D[3]*x*y)+m.cy;
  if(!std::isfinite(u)||!std::isfinite(v)||u< -360||u>1320||v< -360||v>1440)continue;tr.reacquire_image_prediction_valid[eye]=true;tr.reacquire_image_prediction_x[eye]=u;tr.reacquire_image_prediction_y[eye]=v;
 }
}
void prepare_reacquisition_geometry(Tracker&tr,const Frame&f,const Vision&vision){
 tr.reacquire_cross_eye_geometry=vision.extrinsics.valid;tr.reacquire_geometry_frame=f.id;tr.reacquire_geometry_valid.fill(false);tr.reacquire_anchor_ray_valid=false;
 auto ray=[&](const Box&b,double&bearing,double&pitch){if(b.eye<0||b.eye>1||!vision.mono[b.eye].valid)return false;try{auto r=canonical_ray(vision.mono[b.eye],vision.extrinsics,(b.x1+b.x2)*.5,(b.y1+b.y2)*.5);bearing=std::atan2(-r[0],r[2]);pitch=std::atan2(r[1],r[2]);return std::isfinite(bearing)&&std::isfinite(pitch);}catch(...){return false;}};
 for(size_t i=0;i<f.boxes.size()&&i<64;i++)tr.reacquire_geometry_valid[i]=ray(f.boxes[i],tr.reacquire_bearings[i],tr.reacquire_pitches[i]);
 for(const auto&t:tr.tracks)if(t.id==tr.selected)tr.reacquire_anchor_ray_valid=ray(t.last_observed_box,tr.reacquire_anchor_bearing,tr.reacquire_anchor_pitch);
 project_reacquisition_region(tr,vision);
}
// 历史世界目标以新真实位姿投影后，仅提供软方向先验，
// 不能成为新图像或深度观测。
static bool same_detector_source(const DetectorSource&h,const SpatialSource&s){return h.frame==s.frame&&h.time==s.image_time&&h.box.eye==s.eye&&h.box.cls==s.cls&&std::fabs(h.box.x1-s.box.x1)<=.5&&std::fabs(h.box.y1-s.box.y1)<=.5&&std::fabs(h.box.x2-s.box.x2)<=.5&&std::fabs(h.box.y2-s.box.y2)<=.5;}
void prepare_reacquisition(Tracker&tr,const Frame&f,const Vision&vision,const std::string&path,uint64_t now){
 prepare_reacquisition_geometry(tr,f,vision);tr.reacquire_region_valid=false;tr.reacquire_lidar_valid=false;tr.spatial_target_valid=false;tr.spatial_source_count=0;tr.follow_policy=false;tr.policy_task_class_id=-1;tr.reacquire_region_reason="REGION_NOT_PROVIDED";
 try{J j=JP(read(path,32768)).parse();if(j.at("version").num()!=1||decimal(j.at("epoch").str())!=f.epoch||j.at("track_id").num()!=tr.selected)return;std::string mode=j.at("mode").str();uint64_t published=decimal(j.at("published_us").str());if((mode!="static"&&mode!="follow")||published>now||now-published>1500000)return;
  tr.follow_policy=mode=="follow";tr.policy_time=published;try{double cls=j.at("task_class_id").num();if(cls==std::floor(cls)&&cls>=0&&cls<=80)tr.policy_task_class_id=(int)cls;}catch(...){}
  uint32_t generation=0;for(const auto&t:tr.tracks)if(t.id==tr.selected)generation=t.binding_generation;
  try{double g=j.at("binding_generation").num();if(g<0||g>UINT32_MAX||g!=std::floor(g))return;tr.policy_binding_generation=(uint32_t)g;}catch(...){tr.policy_binding_generation=0;}
  if(tr.policy_binding_generation!=generation){tr.reacquire_region_reason="REGION_BINDING_GENERATION_CHANGED";return;}
  tr.reacquire_region_reason="POLICY_READY_NO_SPATIAL_POSITION";if(j.at("valid").type!='b'||!j.at("valid").boolean)return;
  uint64_t time=decimal(j.at("source_us").str()),target=decimal(j.at("target_source_us").str());if(time>now||now-time>1200000||target>time)return;
  double center=j.at("center_bearing_rad").num(),width=j.at("half_width_rad").num();if(std::fabs(center)>3.141593||width<.03||width>1.5)return;
  std::string context;const J&cx=j.at("context");for(const char*k:{"source","clock","timeline_id","origin_epoch"}){const J&v=cx.at(k);if(v.type=='s')context+=quote(v.str());else if(v.type=='d')context+=std::to_string(v.num());else return;context+='|';}
  if(!tr.reacquire_region_context.empty()&&tr.reacquire_region_context!=context){tr.reacquire_region_context=context;tr.reacquire_region_after_context_us=time;tr.reacquire_region_reason="REGION_CONTEXT_CHANGED_NEW_PROJECTION_REQUIRED";return;}tr.reacquire_region_context=context;if(time<=tr.reacquire_region_after_context_us)return;
  tr.reacquire_region_valid=true;tr.reacquire_region_time=time;tr.reacquire_target_source_time=target;tr.reacquire_region_bearing=center;tr.reacquire_region_half_width=width;tr.reacquire_region_reason="OLD_REAL_TARGET_NEW_POSE_PROJECTION_SOFT_SUPPORT";
  try{if(j.at("lidar_support_valid").type=='b'&&j.at("lidar_support_valid").boolean){uint64_t scan=decimal(j.at("lidar_source_us").str());double angle=j.at("lidar_bearing_rad").num(),half=j.at("lidar_half_width_rad").num();if(scan&&scan<=now&&now-scan<=800000&&std::fabs(angle)<=3.141593&&half>=.03&&half<=1.5){tr.reacquire_lidar_valid=true;tr.reacquire_lidar_time=scan;tr.reacquire_lidar_bearing=angle;tr.reacquire_lidar_half_width=half;}}}catch(...){}
  try{if(j.at("target_frame").str()=="ODOM"){tr.spatial_target_x=j.at("target_odom_x_m").num();tr.spatial_target_y=j.at("target_odom_y_m").num();tr.spatial_target_error=j.at("target_error_m").num();tr.spatial_target_valid=std::isfinite(tr.spatial_target_x)&&std::isfinite(tr.spatial_target_y)&&tr.spatial_target_error>=0&&tr.spatial_target_error<=5;}}catch(...){}
  try{const J&list=j.at("spatial_candidates");if(list.type=='a'&&list.values.size()<=16)for(const J&item:list.values){SpatialSource candidate;candidate.track_id=(int)item.at("track_id").num();candidate.frame=decimal(item.at("frame_id").str());candidate.time=decimal(item.at("source_us").str());try{candidate.image_time=decimal(item.at("image_time_us").str());}catch(...){candidate.image_time=candidate.time;}candidate.eye=(int)item.at("eye").num();candidate.cls=(int)item.at("class_id").num();const J&bb=item.at("bbox");if(bb.type!='a'||bb.values.size()!=4)continue;candidate.box={candidate.cls,candidate.eye,(float)bb.values[0].num(),(float)bb.values[1].num(),(float)bb.values[2].num(),(float)bb.values[3].num(),1};candidate.x=item.at("odom_x_m").num();candidate.y=item.at("odom_y_m").num();candidate.error=item.at("error_m").num();if(candidate.time>now||now-candidate.time>2000000||candidate.image_time>now||now-candidate.image_time>2000000||candidate.eye<0||candidate.eye>1||candidate.error<0||candidate.error>5)continue;
   bool exact=false;for(const auto&track:tr.tracks)if(track.id==candidate.track_id)for(const auto&source:track.detector_history)if(same_detector_source(source,candidate))exact=true;if(!exact)continue;candidate.valid=true;tr.spatial_sources[tr.spatial_source_count++]=candidate;
  }}catch(...){}
  project_reacquisition_region(tr,vision);
 }catch(...){tr.reacquire_region_reason="REGION_UNAVAILABLE_OR_INVALID";}
}
