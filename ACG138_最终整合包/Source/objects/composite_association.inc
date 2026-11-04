// 按当前可用线索归一化分配，各项支持分数
// 均在0至1之间，IoU只占5%权重，不作为准入或最小重叠限制。
static AssociationEvidence association_evidence(const Tracker&owner,const Track&t,const Frame&f,const Box&b,const GrayAppearance&descriptor,int index){
 AssociationEvidence e;double lost=f.time>=t.observed_at?(f.time-t.observed_at)/1e6:0;
 bool same_eye=b.eye==t.box.eye,selected=t.id==owner.selected;
 bool geometry=index>=0&&index<64&&owner.reacquire_geometry_frame==f.id&&owner.reacquire_geometry_valid[index];
 bool region=selected&&geometry&&owner.reacquire_region_valid&&owner.reacquire_region_time<=f.time+100000;
 if(!same_eye&&!owner.reacquire_cross_eye_geometry){e.reason="CROSS_EYE_CANONICAL_GEOMETRY_UNAVAILABLE";return e;}
 double width=std::max(16.f,t.last_observed_box.x2-t.last_observed_box.x1),height=std::max(16.f,t.last_observed_box.y2-t.last_observed_box.y1);
 double sigma=std::clamp(std::max(width,height)*.80+lost*35.,90.,420.);
 double dx=((b.x1+b.x2)-(t.box.x1+t.box.x2))*.5,dy=((b.y1+b.y2)-(t.box.y1+t.box.y2))*.5;
 double distance=std::hypot(dx,dy),spatial=1./(1+distance*distance/(sigma*sigma));
 bool finite_spatial=same_eye&&distance<=3.5*sigma;
 if(geometry&&owner.reacquire_anchor_ray_valid){
  double reference=region?owner.reacquire_region_bearing:owner.reacquire_anchor_bearing;
  double delta=std::remainder(owner.reacquire_bearings[index]-reference,6.28318530718);
  double angular_width=region?std::clamp(owner.reacquire_region_half_width+.035*std::min(10.,lost),.18,1.20):std::clamp(.30+.04*lost,.30,1.20);
  double vertical=std::fabs(owner.reacquire_pitches[index]-owner.reacquire_anchor_pitch);
  // 车体偏航时使用导航或里程计投影，替代过期的水平像素位置。
  double angular=1./(1+delta*delta/(angular_width*angular_width));
  if(!region)angular/=1+vertical*vertical/(.35*.35);
  spatial=region?angular:same_eye?std::max(spatial,angular):angular;
  finite_spatial=std::fabs(delta)<=std::min(1.5,3*angular_width)&&vertical<1.2;
 }else if(!same_eye){e.reason="CROSS_EYE_CANONICAL_GEOMETRY_UNAVAILABLE";return e;}
 e.motion=std::clamp(spatial,0.,1.);e.available|=1;
 if(selected&&geometry&&owner.reacquire_lidar_valid&&owner.reacquire_lidar_time<=f.time+100000&&f.time<=owner.reacquire_lidar_time+800000){
  double delta=std::remainder(owner.reacquire_bearings[index]-owner.reacquire_lidar_bearing,6.28318530718),w=std::clamp(owner.reacquire_lidar_half_width,.12,1.2);
  e.region=1./(1+delta*delta/(w*w));e.available|=2;
 }
 // 真实历史距离归属于准确的原始目标和源帧，可以
 // 支持后续几何连续的检测，但不能修改该距离的时间。
 if(selected&&!owner.follow_policy&&owner.spatial_target_valid){double best_pixel=1e9;const SpatialSource*chosen=nullptr;
  for(unsigned k=0;k<owner.spatial_source_count;k++){const auto&source=owner.spatial_sources[k];if(!source.valid||source.eye!=b.eye||source.time>f.time+100000||f.time>source.time+2000000)continue;
   for(const auto&raw:owner.tracks)if(raw.id==source.track_id){double xx=((raw.box.x1+raw.box.x2)-(b.x1+b.x2))*.5,yy=((raw.box.y1+raw.box.y2)-(b.y1+b.y2))*.5,dist=std::hypot(xx,yy),gate=std::max(100.,std::sqrt(box_area(raw.box))*.8);if(dist<=gate&&dist<best_pixel){best_pixel=dist;chosen=&source;}}
  }
  if(chosen){double gap=std::hypot(chosen->x-owner.spatial_target_x,chosen->y-owner.spatial_target_y),gate=std::min(1.,std::max(.25,chosen->error+owner.spatial_target_error));if(gap>gate){e.reason="STATIC_REAL_SPATIAL_CANDIDATE_CONTRADICTS_ORIGINAL_TARGET";return e;}e.region=std::exp(-.5*gap*gap/(gate*gate));e.available|=2;}
 }
 const GrayAppearance&reference=t.identity_reference.valid?t.identity_reference:t.appearance;
 if(reference.valid&&descriptor.valid){double a=t.identity_reference.valid?profile_distance(t,descriptor):appearance_distance(reference,descriptor);e.appearance=std::exp(-std::max(0.,a)/.32);e.available|=4;}
 double area=box_area(b)/std::max(1.,box_area(t.last_observed_box)),aspect=box_aspect(b)/std::max(.01,box_aspect(t.last_observed_box));
 if(!std::isfinite(area)||!std::isfinite(aspect)||area<=0||aspect<=0){e.reason="NONFINITE_BOX_GEOMETRY";return e;}
 double shape=std::exp(-.35*std::fabs(std::log(area))-.45*std::fabs(std::log(aspect)));
 e.class_shape=t.manual?shape:.65*(b.cls==t.cls?1.:0.)+.35*shape;e.available|=8;
 e.overlap=same_eye?iou(t.box,b):0.;e.available|=16;
 double sum=.35*e.motion+.10*e.class_shape+.05*e.overlap,weight=.50;
 if(e.available&2){sum+=.25*e.region;weight+=.25;}if(e.available&4){sum+=.25*e.appearance;weight+=.25;}
 e.quality=std::clamp(sum/weight,0.,1.);e.cost=1-e.quality;
 e.plausible=finite_spatial&&e.motion>=.08&&e.quality>=.42;
 e.reason=!finite_spatial?"OUTSIDE_EXPANDING_SPATIAL_REGION":e.motion<.08?"MOTION_SUPPORT_INSUFFICIENT":e.quality<.42?"COMBINED_EVIDENCE_WEAK":"COMPOSITE_SPATIAL_EVIDENCE_SUPPORTED";
 // 颜色仅为软身份线索，先按原运动和几何计算
 // 准入，再调整权重，颜色不符不新增否决条件。
 if(t.attributes.color_valid&&b.attributes.color_valid){
  bool same=!std::strcmp(t.attributes.color,b.attributes.color)||!std::strcmp(t.attributes.color,b.attributes.secondary_color)||!std::strcmp(t.attributes.secondary_color,b.attributes.color);
  double weight=.15*std::sqrt(std::clamp(t.attributes.color_support*b.attributes.color_support,0.,1.));
  e.quality=(1-weight)*e.quality+weight*(same?1.:.35);e.cost=1-e.quality;e.available|=32;
  if(e.plausible)e.reason=same?"COMPOSITE_COLOR_SUPPORTED":"COLOR_DIFFERENCE_SOFT_ONLY";
 }
 return e;
}
static bool semantic_support(Tracker&owner,Track&t,const Frame&f,const Config&cfg){
 uint64_t semantic=t.last_detector_us?t.last_detector_us:t.manual_seed_us;
 bool bridge=semantic&&semantic<=f.time&&f.time-semantic<=(uint64_t)(cfg.pixel_bridge_ms*1000);
 bool lidar=owner.reacquire_lidar_valid&&owner.reacquire_lidar_time<=f.time+100000&&f.time<=owner.reacquire_lidar_time+800000;
 if(lidar){t.last_lidar_us=std::max(t.last_lidar_us,owner.reacquire_lidar_time);t.identity_evidence="LIDAR_SUPPORTED_PIXEL";}
 else t.identity_evidence=bridge?"PIXEL_SHORT_GAP_BRIDGE":"PIXEL_UNVERIFIED_AWAIT_DETECTOR_OR_LIDAR";
 return bridge||lidar;
}
