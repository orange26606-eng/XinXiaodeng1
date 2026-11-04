// 只跟踪选中目标的当前像素，工作量有界，不按检测分数设门限。
// 预测只提出搜索区域，只有不同源帧中有信息的真实
// 灰度图像块才能为该原始图设置observed=true。
template<class Sample> static GrayAppearance sampled_profile(const Box&b,Sample sample){
 GrayAppearance d;double sum=0,sq=0;std::array<float,256>v{};
 for(int y=0;y<16;y++)for(int x=0;x<16;x++){
  int xx=std::clamp((int)((b.x1+(b.x2-b.x1)*(.05+.90*(x+.5)/16))/4),0,239);
  int yy=std::clamp((int)((b.y1+(b.y2-b.y1)*(.05+.90*(y+.5)/16))/4),0,269);
  float z=sample(xx,yy,b.eye);v[y*16+x]=z;sum+=z;sq+=z*z;d.hist[(int)z/16]+=1.f/256;
 }
 d.mean=(float)(sum/256);d.contrast=(float)std::sqrt(std::max(0.,sq/256-d.mean*d.mean));d.valid=d.contrast>=8;
 for(int y=0;y<8;y++)for(int x=0;x<8;x++){double z=0;for(int dy=0;dy<2;dy++)for(int dx=0;dx<2;dx++)z+=v[(y*2+dy)*16+x*2+dx];d.cells[y*8+x]=(float)((z/4-d.mean)/std::max(12.f,d.contrast));}
 return d;
}
static GrayAppearance snapshot_profile(const SnapshotIdentity&h,const Box&b){
 if(h.gray_quarter.size()!=129600||b.eye<0||b.eye>1)return {};
 return sampled_profile(b,[&](int x,int y,int eye){return h.gray_quarter[(size_t)eye*64800+y*240+x];});
}
static GrayAppearance image_profile(const Frame&f,const Box&b){
 if(f.gray.size()!=2073600||b.eye<0||b.eye>1)return {};
 return sampled_profile(b,[&](int x,int y,int eye){unsigned sum=0;for(int dy=0;dy<4;dy++)for(int dx=0;dx<4;dx++)sum+=f.gray[(y*4+dy)*1920+eye*960+x*4+dx];return (uint8_t)((sum+8)/16);});
}
static double pixel_distance(const Track&t,const GrayAppearance&p){double cost=appearance_distance(t.pixel_reference,p);for(int i=0;i<t.pixel_template_count;i++)cost=std::min(cost,appearance_distance(t.pixel_templates[i],p));return cost;}
static GrayAppearance describe_snapshot(const SnapshotIdentity&h,const Box&b){return snapshot_profile(h,b);}
SnapshotIdentity capture_snapshot_identity(const Frame&f,const Tracker&t){
 SnapshotIdentity h;h.id=std::to_string(f.epoch)+"-"+std::to_string(f.id);h.time=f.time;for(const auto&v:t.tracks)if(v.observed){h.tracks.push_back(v.id);h.track_boxes[v.id]=v.last_observed_box;h.track_bindings[v.id]=v.binding_generation;}
 if(f.gray.size()!=2073600)return h;h.gray_quarter.resize(129600);
 for(int eye=0;eye<2;eye++)for(int y=0;y<270;y++)for(int x=0;x<240;x++){
  // 执行实际4×4均值采样，像素仍对应原始镜头和帧。
  unsigned sum=0;for(int dy=0;dy<4;dy++)for(int dx=0;dx<4;dx++)sum+=f.gray[(y*4+dy)*1920+eye*960+x*4+dx];
  h.gray_quarter[(size_t)eye*64800+y*240+x]=(uint8_t)((sum+8)/16);
 }return h;
}
int Tracker::select_roi(const Box&b,const GrayAppearance&profile,uint64_t frame,uint64_t time){
 if(!epoch||!frame||!time||b.eye<0||b.eye>1||b.x1<0||b.y1<0||b.x2>960||b.y2>1080||b.x2-b.x1<8||b.y2-b.y1<8)throw std::runtime_error("ROI_INVALID");
 for(auto&t:tracks){t.anchor={};t.anchor_locked=false;t.identity_reference={};t.template_count=t.template_next=0;t.identity_reason="UNLOCKED";t.identity_missed=0;t.last_identity_reject_us=0;}
 // 为显式选中目标分配历史前，只释放未选中目标的记录。
 if(tracks.size()>=MAX_TRACKS){auto victim=std::find_if(tracks.begin(),tracks.end(),[&](const Track&t){return t.id!=selected&&!t.observed;});if(victim==tracks.end())victim=std::find_if(tracks.begin(),tracks.end(),[&](const Track&t){return t.id!=selected;});if(victim!=tracks.end())tracks.erase(victim);}
 if(tracks.size()>=MAX_TRACKS)throw std::runtime_error("ROI_TRACK_CAPACITY");
 Track t;t.manual_seed_us=time;t.identity_evidence="MANUAL_ROI_SEED";t.id=next_id++;t.cls=80;t.task_class_id=80;t.observed_class_id=80;t.manual=true;t.box=t.last_observed_box=b;t.box.cls=t.last_observed_box.cls=80;t.box.score=t.last_observed_box.score=1;
 t.appearance=t.identity_reference=t.pixel_reference=profile;t.anchor_locked=true;t.anchor={80,box_area(b),box_aspect(b),0,0,false,time,epoch};t.template_time=time;t.observed_at=time;t.last_frame=frame;t.observed=false;t.status="PREDICTED";t.seen=1;
 auto m=measure(t.box);for(int k=0;k<4;k++)t.kf[k].x=m[k];t.identity_reason=profile.valid?"USER_ROI_TEMPLATE_AWAIT_CURRENT_IMAGE":"USER_ROI_LOW_TEXTURE_AWAIT_CURRENT_IMAGE";
 tracks.push_back(t);selected=t.id;anchor_owner=t.id;selection_lost=false;reid_time_us=0;reid_from=-1;return t.id;
}
static bool image_track_selected(Tracker&owner,Track&t,const Frame&f,const Config&cfg){
 (void)cfg;uint64_t begin=now_us();t.image_tracking_valid=false;t.image_match_error=1;t.image_match_margin=0;t.image_candidates=0;
 if(f.gray.size()!=2073600||!t.pixel_reference.valid||t.box.eye<0||t.box.eye>1||!t.observed_at||f.time<=t.observed_at){t.identity_reason="IMAGE_TRACKER_TEMPLATE_UNAVAILABLE_OR_NONNEW_SOURCE";return false;}
 double lost=(f.time-t.observed_at)/1e6;Box origin=t.box;double width=std::max(8.f,origin.x2-origin.x1),height=std::max(8.f,origin.y2-origin.y1);
 double cx=(origin.x1+origin.x2)*.5,cy=(origin.y1+origin.y2)*.5;
 // 选中目标可在旧锚点之外重新出现，搜索范围有界，
 // 不授予未经观测的运动许可，也不设置固定世界位置限制。
 bool projected=owner.reacquire_region_valid&&owner.reacquire_image_prediction_valid[t.box.eye]&&owner.reacquire_region_time<=f.time+100000;
 if(projected){cx=owner.reacquire_image_prediction_x[t.box.eye];cy=owner.reacquire_image_prediction_y[t.box.eye];}
 double radius=std::min(360.,std::max(24.,std::min(width,height)*.65)+std::min(160.,lost*100));
 radius=std::ceil(radius/4)*4;int stride=std::max(4,(int)std::ceil(radius/30));struct Hit{Box box;GrayAppearance appearance;double cost;};std::vector<Hit>hits;hits.reserve(4608);SnapshotIdentity current=capture_snapshot_identity(f,owner);
 auto evaluate=[&](double x,double y,double scale){
  if(t.image_candidates>=4608)return;double w=width*scale,h=height*scale;if(w<8||h<8||w>960||h>1080)return;
  Box b=origin;b.x1=(float)(x-w*.5);b.x2=(float)(x+w*.5);b.y1=(float)(y-h*.5);b.y2=(float)(y+h*.5);if(b.x1<0||b.y1<0||b.x2>960||b.y2>1080)return;
  auto d=snapshot_profile(current,b);t.image_candidates++;if(!d.valid)return;double cost=pixel_distance(t,d);if(!std::isfinite(cost))return;hits.push_back({b,d,cost});
 };
 for(int dy=-(int)radius;dy<=(int)radius;dy+=stride)for(int dx=-(int)radius;dx<=(int)radius;dx+=stride)evaluate(cx+dx,cy+dy,1.);
 evaluate(cx,cy,1.);
 if(hits.empty()){t.identity_reason="IMAGE_TRACKER_NO_TEXTURE_MATCH";t.image_tracking_ms=(now_us()-begin)/1000.;return false;}
 auto best_it=std::min_element(hits.begin(),hits.end(),[](const Hit&a,const Hit&b){return a.cost<b.cost;});Hit coarse=*best_it;
 double bx=(coarse.box.x1+coarse.box.x2)*.5,by=(coarse.box.y1+coarse.box.y2)*.5,bs=(coarse.box.x2-coarse.box.x1)/width;
 for(double scale:{bs*.86,bs*.94,bs,bs*1.08,bs*1.16})for(int dy=-stride;dy<=stride;dy+=2)for(int dx=-stride;dx<=stride;dx+=2)evaluate(bx+dx,by+dy,scale);
 auto winner=std::min_element(hits.begin(),hits.end(),[](const Hit&a,const Hit&b){return a.cost<b.cost;});Hit best=*winner;double second=1.;
 for(const auto&h:hits){double dx=((h.box.x1+h.box.x2)-(best.box.x1+best.box.x2))*.5,dy=((h.box.y1+h.box.y2)-(best.box.y1+best.box.y2))*.5;
  // 相邻搜索提案属于同一个物理匹配峰，不视为竞争身份。
  if(std::hypot(dx,dy)>=std::max(18.,std::min(width,height)*.55))second=std::min(second,h.cost);
 }
 t.image_match_error=best.cost;t.image_match_margin=second-best.cost;t.image_tracking_ms=(now_us()-begin)/1000.;
 if(best.cost>.235||second-best.cost<.025){t.identity_reason=best.cost>.235?"IMAGE_TRACKER_MATCH_WEAK":"IMAGE_TRACKER_COMPETING_PATCHES";return false;}
 t.pixel_measured=true;t.last_pixel_us=f.time;t.last_pixel_frame=f.id;
  if(!semantic_support(owner,t,f,cfg)){
   auto m=measure(best.box);for(int k=0;k<4;k++)t.kf[k].update(m[k]);t.box=best.box;
   t.reacquire_candidate_valid=true;t.reacquire_candidate_box=best.box;t.reacquire_candidate_time=f.time;t.reacquire_candidate_frame=f.id;t.reacquire_candidate_status="PIXEL_UNVERIFIED";t.reacquire_candidate_score=std::max(0.,1-best.cost);t.reacquire_candidate_reason="CURRENT_PIXEL_PATCH_SEMANTIC_SUPPORT_EXPIRED";t.identity_reason="IMAGE_TRACKER_SEMANTIC_SUPPORT_EXPIRED";return false;
  }
  auto m=measure(best.box);for(int k=0;k<4;k++)t.kf[k].update(m[k]);t.box=t.last_observed_box=best.box;t.appearance=describe(f,best.box);if(t.manual&&t.identity_accepts==0){t.identity_reference=t.appearance;t.template_count=t.template_next=0;}t.observed=t.image_tracking_valid=true;t.detector_observed=false;t.observed_at=f.time;t.last_frame=f.id;t.seen++;t.consecutive++;t.identity_missed=0;t.identity_accepts++;t.status=t.seen>=cfg.confirm?"CONFIRMED":"TENTATIVE";t.identity_reason="CURRENT_IMAGE_PATCH_MEASURED";owner.selection_lost=false;
 if(best.cost<.10&&second-best.cost>.05&&f.time>=t.pixel_template_time+500000){t.pixel_templates[t.pixel_template_next]=best.appearance;t.pixel_template_next=(t.pixel_template_next+1)%4;t.pixel_template_count=std::min(4,t.pixel_template_count+1);t.pixel_template_time=f.time;}
 // 图像块匹配形成新的方向与框，不形成新的深度。
 t.point.valid=t.coarse.valid=false;t.point.reason="IMAGE_TRACKED_WAIT_ORIGINAL_DEPTH";
 return true;
}
