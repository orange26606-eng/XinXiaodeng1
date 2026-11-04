// 有界真实图像记忆，不作为新的距离或last_seen观测。
static FollowKeyframe follow_view(const Frame&f,const Track&t,const Vision&vision){
 FollowKeyframe k;k.frame=f.id;k.time=f.time;k.box=t.last_observed_box;
 if(k.box.eye>=0&&k.box.eye<2&&vision.mono[k.box.eye].valid&&(k.box.eye==0||vision.extrinsics.valid))try{auto ray=canonical_ray(vision.mono[k.box.eye],vision.extrinsics,(k.box.x1+k.box.x2)*.5,(k.box.y1+k.box.y2)*.5);double n=std::hypot(std::hypot(ray[0],ray[1]),ray[2]);if(n>0&&std::isfinite(n)){for(int i=0;i<3;i++)k.ray[i]=ray[i]/n;k.bearing=std::atan2(-ray[0],ray[2]);k.bearing_valid=std::isfinite(k.bearing);}}catch(...){}
 if(f.gray.size()==2073600&&k.box.eye>=0&&k.box.eye<2){k.roi.resize(96*96);for(int y=0;y<96;y++)for(int x=0;x<96;x++){int u=std::clamp((int)(k.box.x1+(k.box.x2-k.box.x1)*(x+.5)/96),0,959),v=std::clamp((int)(k.box.y1+(k.box.y2-k.box.y1)*(y+.5)/96),0,1079);k.roi[y*96+x]=f.gray[v*1920+k.box.eye*960+u];}}
 return k;
}
void update_follow_keyframes(Tracker&tr,const Frame&f,const Vision&vision){
 auto&m=tr.follow_exit;const Track*t=nullptr;for(const auto&track:tr.tracks)if(track.id==tr.selected)t=&track;
 if(!t||tr.selected<1){m=FollowExitMemory{};return;}
 if(m.owner!=tr.selected||m.epoch!=f.epoch||m.generation!=t->binding_generation){m=FollowExitMemory{};m.owner=tr.selected;m.epoch=f.epoch;m.generation=t->binding_generation;}
 if(f.time<m.processed_time||(f.time==m.processed_time&&f.id<m.processed_frame))return;
 m.processed_time=f.time;m.processed_frame=f.id;
 bool actual=t->observed&&t->last_frame==f.id&&t->observed_at==f.time&&(t->detector_observed||t->image_tracking_valid);
 if(actual){
  m.valid=false;m.direction=m.kind=0;m.confidence=0;m.reason="CURRENT_REAL_TARGET_REAPPEARED";
  if(!m.positions.empty()&&f.time<=m.positions.back().time)return;
  auto k=follow_view(f,*t,vision);auto position=k;std::vector<uint8_t>().swap(position.roi);m.positions.push_back(std::move(position));while(m.positions.size()>16)m.positions.pop_front();
  bool edge=k.box.x1<=48||k.box.x2>=912||k.box.y1<=48||k.box.y2>=1032;
  bool moving=false,changed_eye=false;if(!m.keyframes.empty()){const auto&old=m.keyframes.back();changed_eye=old.box.eye!=k.box.eye;moving=std::hypot((k.box.x1+k.box.x2-old.box.x1-old.box.x2)*.5,(k.box.y1+k.box.y2-old.box.y1-old.box.y2)*.5)>=24;}
  k.priority=m.keyframes.empty()?4:edge?3:changed_eye?3:moving?2:1;
  // 普通视图每100ms取样，每个不同的真实边缘观测
  // 都保留，避免最后一帧在原50ms边缘采样间隔内消失，
  // 采集位姿补偿也需使用对应的准确源帧。
  // 最多保留8个视图，首个视图固定保留。
  if(m.keyframes.empty()||f.time>=m.keyframes.back().time+100000||changed_eye||edge){
   if(m.keyframes.size()>=8){size_t victim=1;for(size_t i=2;i+1<m.keyframes.size();i++)if(m.keyframes[i].priority<m.keyframes[victim].priority)victim=i;m.keyframes.erase(m.keyframes.begin()+(ptrdiff_t)victim);}m.keyframes.push_back(std::move(k));
  }
  const auto&last=m.positions.back();m.source=last.time;m.frame=last.frame;m.eye=last.box.eye;m.bearing=last.bearing;m.bearing_valid=last.bearing_valid;m.velocity=0;return;
 }
 m.valid=false;m.direction=0;m.kind=3;m.confidence=0;m.reason="CENTER_MISS_NOT_DIRECTIONAL_EXIT";
 if(!tr.follow_policy){m.kind=0;m.reason="STATIC_MODE_EXIT_CONTROL_DISABLED";return;}
 if(m.positions.empty()){m.kind=0;m.reason="NO_REAL_TARGET_HISTORY";return;}
 const auto&last=m.positions.back();m.source=last.time;m.frame=last.frame;m.eye=last.box.eye;m.bearing=last.bearing;m.bearing_valid=last.bearing_valid;m.velocity=0;
 if(last.time>f.time||f.time-last.time>2000000){m.kind=0;m.reason="EXIT_HINT_SOURCE_EXPIRED";return;}
 double w=last.box.x2-last.box.x1,h=last.box.y2-last.box.y1;
 int borders=(last.box.x1<=32)+(last.box.x2>=928)+(last.box.y1<=32)+(last.box.y2>=1048);
 if((w*h>=.35*960*1080&&borders>=2)||(h>=.85*1080&&borders>=1)){
  m.kind=4;m.reason="NEAR_CROP_NOT_LATERAL_EXIT";m.confidence=.65;return;
 }
 // 使用不少于3个真实位置、覆盖不少于100ms的最小二乘拟合，
 // 分别拟合原始光学方位和图像x，不用两点猜测替代。
 std::vector<const FollowKeyframe*>samples;for(auto it=m.positions.rbegin();it!=m.positions.rend()&&samples.size()<8;it++)if(it->time<=last.time&&last.time-it->time<=1200000)samples.push_back(&*it);
 if(samples.size()<3||last.time-samples.back()->time<100000){m.reason="EXIT_SEQUENCE_TOO_SHORT";return;}
 bool same_eye=true,rays=true;double st=0,sx=0,sb=0,stt=0,stx=0,stb=0;unsigned n=(unsigned)samples.size();
 for(auto*k:samples){same_eye&=k->box.eye==last.box.eye;rays&=k->bearing_valid;double dt=((double)k->time-(double)last.time)/1e6,x=(k->box.x1+k->box.x2)*.5;st+=dt;sx+=x;sb+=k->bearing;stt+=dt*dt;stx+=dt*x;stb+=dt*k->bearing;}
 double denominator=n*stt-st*st;if(denominator<1e-9)return;double vx=(n*stx-st*sx)/denominator,omega=(n*stb-st*sb)/denominator;
 if(rays)m.velocity=std::clamp(omega,-6.,6.);
 if(!same_eye&&!rays){m.reason="EXIT_CROSS_EYE_CANONICAL_RAY_UNAVAILABLE";return;}
 const auto&first=*samples.back();double displacement=same_eye?((last.box.x1+last.box.x2-first.box.x1-first.box.x2)*.5):last.bearing-first.bearing;
 bool left=last.box.x1<=48&&(same_eye?(vx<=-40&&displacement<=-12):(omega>=.05&&displacement>=.02));
 bool right=last.box.x2>=912&&(same_eye?(vx>=40&&displacement>=12):(omega<=-.05&&displacement<=-.02));
 if(!left&&!right){m.reason="CENTER_MISS_OR_EDGE_WITHOUT_OUTWARD_SEQUENCE";return;}
 // 实际连续观测的大多数需支持该方向，
 // 排除稳定画面后单个噪声边缘点造成的误判。
 unsigned agreeing=0,total=0;for(size_t i=1;i<samples.size();i++){const auto&a=*samples[i],&b=*samples[i-1];double delta=same_eye?(b.box.x1+b.box.x2-a.box.x1-a.box.x2)*.5:b.bearing-a.bearing;if(std::fabs(delta)<(same_eye?2.:.003))continue;total++;if(same_eye?(left?delta<0:delta>0):(left?delta>0:delta<0))agreeing++;}
 if(total<2||agreeing*3<total*2){m.reason="EXIT_DIRECTION_SEQUENCE_INCONSISTENT";return;}
 m.valid=true;m.direction=left?1:-1;m.kind=left?1:2;m.confidence=std::min(.95,.55+.05*n);m.reason=left?"REAL_SEQUENCE_LEFT_EDGE_EXIT_RAW_UNCOMPENSATED":"REAL_SEQUENCE_RIGHT_EDGE_EXIT_RAW_UNCOMPENSATED";
}
static void follow_exit_json(std::ostringstream&o,const Tracker&tr,const Frame&f,uint64_t generated){
 const auto&m=tr.follow_exit;bool same=m.owner==tr.selected&&m.epoch==f.epoch;bool eligible=same&&tr.follow_policy&&m.source&&m.source<=f.time&&m.source<=generated&&generated-m.source<=2000000;
 o<<",\"follow_exit\":{\"valid\":"<<(eligible&&m.valid?"true":"false")<<",\"bearing_valid\":"<<(same&&m.bearing_valid?"true":"false")<<",\"direction\":"<<(eligible?m.direction:0)<<",\"kind\":"<<(same?m.kind:0)<<",\"source_us\":"<<quote(std::to_string(same?m.source:0))<<",\"frame_id\":"<<quote(std::to_string(same?m.frame:0))<<",\"eye\":"<<(same?m.eye:0)<<",\"bearing_rad\":";if(same&&m.bearing_valid)o<<m.bearing;else o<<"null";
 o<<",\"angular_velocity_rad_s\":";if(same&&m.bearing_valid)o<<m.velocity;else o<<"null";o<<",\"confidence\":"<<(eligible?m.confidence:0)<<",\"compensated\":false,\"reason\":"<<quote(!same?"NO_REAL_TARGET_HISTORY":!tr.follow_policy?"STATIC_MODE_EXIT_CONTROL_DISABLED":m.source&&generated>m.source+2000000?"EXIT_HINT_SOURCE_EXPIRED":m.reason)<<",\"keyframes\":[";
 bool first=true;if(same)for(const auto&k:m.keyframes){if(k.time>f.time||k.time>generated)continue;if(!first)o<<',';first=false;o<<"{\"frame_id\":"<<quote(std::to_string(k.frame))<<",\"source_us\":"<<quote(std::to_string(k.time))<<",\"eye\":"<<k.box.eye<<",\"bbox\":["<<k.box.x1<<','<<k.box.y1<<','<<k.box.x2<<','<<k.box.y2<<"],\"bearing_rad\":";if(k.bearing_valid)o<<k.bearing;else o<<"null";o<<",\"ray_direction\":";if(k.bearing_valid)o<<'['<<k.ray[0]<<','<<k.ray[1]<<','<<k.ray[2]<<']';else o<<"null";o<<'}';}o<<"]}";
}
