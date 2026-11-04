// 检测分数用于关联排序。
void Mono::load(const std::string&path,int expected){
 std::ifstream f(path);if(!f)throw std::runtime_error("MONO_FILE_MISSING");std::map<std::string,double>v;std::string line;
 while(std::getline(f,line)){line=trim(line.substr(0,line.find('#')));if(line.empty())continue;auto at=line.find('=');if(at==std::string::npos)throw std::runtime_error("MONO_SYNTAX");std::string key=trim(line.substr(0,at)),value=trim(line.substr(at+1));char*end=nullptr;double x=std::strtod(value.c_str(),&end);if(end==value.c_str()||*end||!std::isfinite(x)||!v.emplace(key,x).second)throw std::runtime_error("MONO_VALUE_INVALID");}
 for(const char*k:{"eye","image_width","image_height","fx","fy","cx","cy","k1","k2","p1","p2","k3"})if(!v.count(k))throw std::runtime_error(std::string("MONO_KEY_MISSING_")+k);
 if(v["eye"]!=expected||v["image_width"]!=960||v["image_height"]!=1080||v["fx"]<=0||v["fy"]<=0||v["cx"]<0||v["cx"]>=960||v["cy"]<0||v["cy"]>=1080)throw std::runtime_error("MONO_EYE_OR_GEOMETRY_MISMATCH");
 eye=expected;fx=v["fx"];fy=v["fy"];cx=v["cx"];cy=v["cy"];D={v["k1"],v["k2"],v["p1"],v["p2"],v["k3"]};valid=true;reason="MEASURED_MONO_READY";
}
std::array<double,3>Mono::ray(double u,double v)const{
 if(!valid)throw std::runtime_error("MONO_NOT_LOADED");double xd=(u-cx)/fx,yd=(v-cy)/fy,x=xd,y=yd;
 for(int i=0;i<10;i++){double r=x*x+y*y,radial=1+D[0]*r+D[1]*r*r+D[4]*r*r*r;if(!std::isfinite(radial)||std::fabs(radial)<.1)throw std::runtime_error("MONO_UNDISTORT_DIVERGED");double tx=2*D[2]*x*y+D[3]*(r+2*x*x),ty=D[2]*(r+2*y*y)+2*D[3]*x*y;x=(xd-tx)/radial;y=(yd-ty)/radial;}
 if(!std::isfinite(x)||!std::isfinite(y)||std::fabs(x)>20||std::fabs(y)>20)throw std::runtime_error("MONO_RAY_INVALID");return{x,y,1};
}
void Extrinsics::load(const std::string&path){J j=JP(read(path,32768)).parse();if(j.at("version").num()!=1)throw std::runtime_error("EXTRINSICS_VERSION");double e=j.at("left_eye").num();if(e!=0&&e!=1)throw std::runtime_error("EXTRINSICS_EYE");left_eye=(int)e;auto values=[&](const char*key,auto&out){const J&a=j.at(key);if(a.type!='a'||a.values.size()!=out.size())throw std::runtime_error("EXTRINSICS_ARRAY");for(size_t i=0;i<out.size();i++)out[i]=a.values[i].num();};values("R",R);values("T",T);for(int i=0;i<3;i++)for(int k=0;k<3;k++){double d=0;for(int q=0;q<3;q++)d+=R[i*3+q]*R[k*3+q];if(std::fabs(d-(i==k))>.01)throw std::runtime_error("EXTRINSICS_ROTATION");}double b=std::hypot(std::hypot(T[0],T[1]),T[2]);if(b<=.005||b>=1)throw std::runtime_error("EXTRINSICS_BASELINE");valid=true;reason="MEASURED_STEREO_EXTRINSICS_READY";}
std::array<double,3>canonical_ray(const Mono&m,const Extrinsics&e,double u,double v){auto r=m.ray(u,v);if(!e.valid||m.eye==e.left_eye)return r;std::array<double,3>q{};for(int i=0;i<3;i++)for(int j=0;j<3;j++)q[i]+=e.R[j*3+i]*r[j];return q;}
EyeQuality assess_eye(const Frame&f,int eye,const Box*target){
 EyeQuality q;q.source_frame=f.id;q.source_us=f.time;if(f.gray.size()!=2073600||eye<0||eye>1)return q;int x0=target?std::clamp((int)target->x1,2,956):160,x1=target?std::clamp((int)target->x2,3,957):800,y0=target?std::clamp((int)target->y1,2,1076):180,y1=target?std::clamp((int)target->y2,3,1077):900;
 if(x1<=x0||y1<=y0){q.reason="TARGET_ROI_EMPTY";return q;}int step=std::max(1,std::max(x1-x0,y1-y0)/128),n=0;double sum=0,sq=0,lap=0,saturated=0,gx=0,gy=0;
 for(int y=y0;y<y1;y+=step)for(int x=x0;x<x1;x+=step){size_t i=(size_t)y*1920+eye*960+x;double a=f.gray[i],l=4*a-f.gray[i-1]-f.gray[i+1]-f.gray[i-1920]-f.gray[i+1920];sum+=a;sq+=a*a;lap+=l*l;double dx=(double)f.gray[i+1]-f.gray[i-1],dy=(double)f.gray[i+1920]-f.gray[i-1920];gx+=dx*dx;gy+=dy*dy;saturated+=(a<=3||a>=252);n++;}
 q.mean=sum/std::max(1,n);q.contrast=std::sqrt(std::max(0.,sq/std::max(1,n)-q.mean*q.mean));q.sharpness=lap/std::max(1,n);q.gradient_x=gx/std::max(1,n);q.gradient_y=gy/std::max(1,n);q.saturation_fraction=saturated/std::max(1,n);q.normalized_detail=q.sharpness/(q.contrast*q.contrast+1.0);
 // 清晰度指标会受纹理、噪声和运动影响，不能单独确认镜头对焦状态。
 q.focus_state=q.saturation_fraction>.90?"EXPOSURE_INVALID":q.contrast<8?"LOW_TEXTURE_FOCUS_UNKNOWN":q.normalized_detail<.05?"SOFT_OR_MOTION_BLUR_SUSPECT":"EDGE_DETAIL_PRESENT";
 if(saturated/std::max(1,n)>.90){q.reason="TARGET_OVER_OR_UNDER_EXPOSED";return q;}
 if(q.contrast>=8&&q.sharpness<3){q.reason="TARGET_BLURRED";return q;}
 q.usable=true;q.reason=q.contrast<8?"LOW_TEXTURE_FOCUS_UNKNOWN_USABLE":"TARGET_CLEAR_USABLE";return q;
}
void Tracker::update(const Frame&f,const std::vector<Box>&boxes,const std::vector<Point>&points,const Config&cfg){
 if(points.size()!=boxes.size()||boxes.size()>64)throw std::runtime_error("TRACK_POINT_COUNT_MISMATCH");if(f.epoch!=epoch)reset(f.epoch);if(last_time&&(f.id<=last_frame_id||f.time<=last_time))return;
 if(anchor_owner!=selected){for(auto&t:tracks){t.anchor={};t.anchor_locked=false;t.identity_reference={};t.template_count=t.template_next=0;t.identity_reason="UNLOCKED";t.identity_missed=0;t.last_identity_reject_us=0;}anchor_owner=selected;}
 for(auto&t:tracks)if(t.id==selected&&!t.anchor_locked&&t.observed){const Point&q=t.coarse;bool verified=q.world_valid||world_fresh(t.world,f);t.anchor={t.cls,box_area(t.last_observed_box),box_aspect(t.last_observed_box),q.world_valid?q.world_x:t.world.x,q.world_valid?q.world_y:t.world.y,verified,t.observed_at,epoch};t.identity_reference=t.appearance;t.anchor_locked=true;t.template_time=f.time;identity_event(*this,t,f.time,"OBJECT_IDENTITY_ANCHORED","USER_SELECTED_FIRST_OBSERVATION");}
 std::vector<GrayAppearance>descriptors;for(const auto&b:boxes)descriptors.push_back(describe(f,b));double dt=last_time?std::min(1.,(f.time-last_time)/1e6):.2;last_time=f.time;last_frame_id=f.id;
 for(auto&t:tracks){if(t.id==selected&&reacquire_lidar_valid&&reacquire_lidar_time<=f.time+100000&&f.time<=reacquire_lidar_time+800000)t.last_lidar_us=std::max(t.last_lidar_us,reacquire_lidar_time);t.observed=false;t.pixel_measured=false;t.image_tracking_valid=false;t.detector_observed=false;t.image_candidates=0;t.image_tracking_ms=0;t.point.valid=false;for(auto&k:t.kf)k.predict(dt);t.box=predicted(t);}
 struct Edge{int t,b;double score;AssociationEvidence evidence;bool follow_rebind=false;};std::vector<Edge>edges;std::vector<bool>used(boxes.size(),false);std::set<int>matched;
 for(int ti=0;ti<(int)tracks.size();ti++){
  auto&t=tracks[ti];if(t.retired)continue;std::vector<Edge>local;bool long_lost=f.time>t.observed_at+(uint64_t)cfg.lost_ms*1000;
  if(f.detection_result&&t.id==selected&&follow_policy&&!t.manual){bool replacement=false;int found=follow_continuation(*this,t,f,boxes,descriptors,cfg,replacement);if(found>=0){auto evidence=association_evidence(*this,t,f,boxes[found],descriptors[found],found);if(!replacement){evidence.quality=std::max(.6,evidence.quality);evidence.cost=1-evidence.quality;}edges.push_back({ti,found,11+evidence.quality,evidence,replacement});}continue;}
  if(f.detection_result&&t.id==selected&&(long_lost||t.identity_missed>=2)){
   int found=propose_reacquisition(*this,t,f,boxes,descriptors);if(found>=0)edges.push_back({ti,found,10+t.reacquire_candidate_score,t.association});continue;
  }
  if(f.detection_result){t.reacquire_candidate_valid=false;t.reacquire_candidate_hits=0;t.reacquire_candidate_status="NONE";}
  for(int bi=0;bi<(int)boxes.size();bi++){
   const auto&b=boxes[bi];if(b.eye!=t.box.eye)continue; // 换眼使用下方经过标定的综合比较。
   auto e=association_evidence(*this,t,f,b,descriptors[bi],bi);if(!e.plausible)continue;local.push_back({ti,bi,e.quality,e});
  }
  std::sort(local.begin(),local.end(),[](const Edge&a,const Edge&b){return a.score>b.score;});
  if(t.id==selected&&local.size()>1&&local[0].score-local[1].score<.08){identity_event(*this,t,f.time,"OBJECT_IDENTITY_UNCERTAIN","COMPETING_CURRENT_OBJECTS_KEEP_TASK");continue;}
  edges.insert(edges.end(),local.begin(),local.end());
 }
 edges=byte_association(tracks,boxes,edges,selected,f,cfg);
 for(auto e:edges){auto&t=tracks[e.t];if(used[e.b]||matched.count(t.id))continue;used[e.b]=true;matched.insert(t.id);if(e.follow_rebind)bind_follow_target(t,f,boxes[e.b]);t.observed_class_id=boxes[e.b].cls;t.cls=t.manual?80:boxes[e.b].cls;remember_detector(t,f,boxes[e.b]);auto m=measure(boxes[e.b]);for(int k=0;k<4;k++)t.kf[k].update(m[k]);t.association=e.evidence;t.last_detector_us=f.time;t.last_detector_frame=f.id;t.detector_confirmations++;t.identity_evidence="DETECTOR_SEMANTIC_OBSERVATION";t.appearance=descriptors[e.b];if(t.id!=selected||!t.pixel_reference.valid)t.pixel_reference=image_profile(f,boxes[e.b]);t.box=boxes[e.b];t.last_observed_box=t.box;t.observed=true;t.detector_observed=true;t.observed_at=f.time;t.last_frame=f.id;t.seen++;t.consecutive++;t.identity_missed=0;t.identity_accepts++;
 if(t.anchor_locked&&t.appearance.valid&&t.identity_reference.valid&&t.consecutive>=3&&f.time>=t.template_time+500000&&profile_distance(t,t.appearance)<=.30){bool novel=true;for(int i=0;i<t.template_count;i++)if(appearance_distance(t.templates[i],t.appearance)<.025)novel=false;if(novel){t.templates[t.template_next]=t.appearance;t.template_next=(t.template_next+1)%4;t.template_count=std::min(4,t.template_count+1);t.template_time=f.time;}}
 if(t.id==selected){selection_lost=false;t.identity_reason="SOFT_MULTI_CUE_CONTINUITY";}Point q=points[e.b];if(q.valid){q.source_frame=f.id;q.source_time=f.time;q.source_eye=t.box.eye;q.source_box=t.box;t.last_point=q;t.point=q;t.coarse=q;}else{t.coarse.valid=false;t.point.valid=false;}t.status=t.seen>=cfg.confirm?"CONFIRMED":"TENTATIVE";}
 tracks.erase(std::remove_if(tracks.begin(),tracks.end(),[&](const Track&t){return t.id!=selected&&f.time>t.observed_at+(uint64_t)cfg.expire_ms*1000;}),tracks.end());
 // 预测结果保留历史观测身份；新检测单独更新可见时间。

 for(size_t i=0;i<boxes.size();i++)if(!used[i]){if((int)tracks.size()>=cfg.max_tracks){auto victim=tracks.end();for(auto it=tracks.begin();it!=tracks.end();++it)if(it->id!=selected&&!it->observed&&(victim==tracks.end()||it->observed_at<victim->observed_at))victim=it;if(victim!=tracks.end())tracks.erase(victim);}if((int)tracks.size()>=cfg.max_tracks)continue;Track t;t.id=next_id++;t.observed_class_id=t.cls=boxes[i].cls;remember_detector(t,f,boxes[i]);t.box=t.last_observed_box=boxes[i];t.appearance=descriptors[i];t.pixel_reference=image_profile(f,t.box);auto m=measure(t.box);for(int k=0;k<4;k++)t.kf[k].x=m[k];t.last_detector_us=f.time;t.last_detector_frame=f.id;t.detector_confirmations=1;t.identity_evidence="DETECTOR_SEMANTIC_OBSERVATION";t.seen=t.consecutive=1;t.observed=true;t.detector_observed=true;t.observed_at=f.time;t.last_frame=f.id;t.last_point=t.point=t.coarse=points[i];if(t.point.valid){t.point.source_frame=t.last_point.source_frame=t.coarse.source_frame=f.id;t.point.source_time=t.last_point.source_time=t.coarse.source_time=f.time;t.point.source_eye=t.last_point.source_eye=t.coarse.source_eye=t.box.eye;t.point.source_box=t.last_point.source_box=t.coarse.source_box=t.box;}tracks.push_back(t);}
 for(auto&chosen:tracks)if(chosen.id==selected&&chosen.reacquire_candidate_time==f.time){double best=.70;for(const auto&raw:tracks)if(raw.observed&&raw.last_frame==f.id&&raw.box.eye==chosen.reacquire_candidate_box.eye){double overlap=iou(raw.box,chosen.reacquire_candidate_box);if(overlap>=best){best=overlap;chosen.reacquire_candidate_track_id=raw.id;}}}
 if(cfg.image_tracking)for(auto&t:tracks)if(t.id==selected&&!t.observed&&!t.retired&&!(t.reacquire_candidate_valid&&t.reacquire_candidate_time==f.time))image_track_selected(*this,t,f,cfg);
 for(auto&t:tracks)if(t.id==selected&&t.observed){t.reacquire_candidate_valid=false;t.reacquire_candidate_hits=0;if(t.reacquire_candidate_time!=f.time)t.reacquire_candidate_status="NONE";}
 for(auto&t:tracks)if(!t.observed){t.consecutive=0;if(f.detection_result)t.identity_missed++;t.coarse.valid=t.point.valid=false;t.point.reason="PREDICTED_NOT_MEASURED";t.status=f.time>t.observed_at+(uint64_t)cfg.lost_ms*1000?"LOST":"PREDICTED";if(t.id==selected){selection_lost=true;if(t.identity_reason.rfind("IMAGE_TRACKER_",0)!=0&&t.identity_reason.rfind("REACQUIRE_",0)!=0)t.identity_reason="MEMORY_RETAINED_AWAIT_CURRENT_ASSOCIATION";}}
}
bool handoff_eye(Tracker&tr,const Frame&f,Vision&vision){
 auto current=std::find_if(tr.tracks.begin(),tr.tracks.end(),[&](const Track&t){return t.id==tr.selected;});
 if(current==tr.tracks.end()){if(!vision.eyes[vision.active_eye].usable&&vision.eyes[1-vision.active_eye].usable){vision.active_eye=1-vision.active_eye;vision.eye_reason="CURRENT_EYE_UNUSABLE_OTHER_USABLE";return true;}vision.eye_reason="STICKY_CURRENT_EYE_USABLE";return false;}
 int eye=current->box.eye;vision.active_eye=eye;vision.eyes[eye]=assess_eye(f,eye,&current->last_observed_box);if(current->observed&&vision.eyes[eye].usable){vision.eye_reason="SELECTED_EYE_USABLE_KEEP";return false;}
 struct EyeChoice{Track*track;AssociationEvidence evidence;};std::vector<EyeChoice>choices;Tracker cues=tr;
 for(auto&candidate:tr.tracks){if(candidate.id==current->id||!candidate.observed||candidate.box.eye==eye)continue;auto q=assess_eye(f,candidate.box.eye,&candidate.box);if(!q.usable)continue;
  Frame geometry_frame;geometry_frame.id=f.id;geometry_frame.boxes={candidate.box};prepare_reacquisition_geometry(cues,geometry_frame,vision);
  auto e=association_evidence(cues,*current,f,candidate.box,candidate.appearance,0);if(!e.plausible)continue;choices.push_back({&candidate,e});vision.eyes[candidate.box.eye]=q;
 }
 std::sort(choices.begin(),choices.end(),[](const EyeChoice&a,const EyeChoice&b){return a.evidence.quality>b.evidence.quality;});
 if(choices.empty()||(choices.size()>1&&choices[0].evidence.quality-choices[1].evidence.quality<.08)){vision.eye_reason=choices.empty()?"NO_VERIFIED_OTHER_EYE_TARGET_KEEP_MEMORY":"OTHER_EYE_TARGET_AMBIGUOUS_KEEP_MEMORY";return false;}
 Track replacement=*choices.front().track;replacement.association=choices.front().evidence;int consumed=replacement.id;Track previous=*current;replacement.id=current->id;replacement.task_class_id=current->task_class_id;replacement.binding_generation=current->binding_generation;replacement.binding_source_us=current->binding_source_us;replacement.binding_reason=current->binding_reason;replacement.manual=current->manual;if(replacement.manual)replacement.cls=80;replacement.anchor=current->anchor;replacement.anchor_locked=current->anchor_locked;replacement.identity_reference=current->identity_reference;replacement.templates=current->templates;replacement.template_count=current->template_count;replacement.template_next=current->template_next;replacement.identity_reason="SAME_TARGET_OTHER_EYE_HANDOFF";replacement.identity_accepts=current->identity_accepts+1;
 // 保留每个仍被观测的原始表示可选，两槽位属于
 // 同一个已核验物体，仅选中任务编号随镜头切换。
 // 各槽位保留自身源帧、镜头、框及缓存距离的时间戳。
 previous.id=consumed;previous.anchor={};previous.anchor_locked=false;previous.identity_reference={};previous.template_count=previous.template_next=0;previous.identity_reason="SECONDARY_EYE_OBSERVATION_RETAINED";*current=replacement;*choices.front().track=previous;tr.selection_lost=false;vision.active_eye=replacement.box.eye;vision.eye_reason="CURRENT_EYE_UNUSABLE_SAME_TARGET_OTHER_EYE";return true;
}
