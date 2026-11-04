static void remember_detector(Track&t,const Frame&f,const Box&b){if(b.attributes.color_valid)t.attributes=b.attributes;if(!t.detector_history.empty()&&t.detector_history.back().frame==f.id)return;t.detector_history.push_back({b,f.id,f.time});while(t.detector_history.size()>8)t.detector_history.pop_front();}
static void bind_follow_target(Track&t,const Frame&f,const Box&b){
 if(t.binding_generation==UINT32_MAX)throw std::runtime_error("FOLLOW_BINDING_GENERATION_EXHAUSTED");++t.binding_generation;t.binding_source_us=f.time;t.binding_reason="FOLLOW_SAME_CLASS_CONTINUATION";
 t.attributes={};t.last_point=t.point=t.coarse=Point{};t.world={};t.detector_history.clear();t.templates={};t.template_count=t.template_next=0;t.pixel_templates={};t.pixel_template_count=t.pixel_template_next=0;t.last_lidar_us=0;t.reacquire_candidate_hits=0;t.reacquire_candidate_valid=false;t.anchor={};t.anchor_locked=false;t.identity_reference={};t.identity_missed=0;t.detector_confirmations=0;
 auto measured=measure(b);for(int k=0;k<4;k++){t.kf[k]=Axis{};t.kf[k].x=measured[k];}t.pixel_reference=image_profile(f,b);t.identity_reference=describe(f,b);t.pixel_template_time=f.time;
}
static int follow_continuation(Tracker&owner,Track&t,const Frame&f,const std::vector<Box>&boxes,const std::vector<GrayAppearance>&descriptors,const Config&cfg,bool&rebind){
 rebind=false;if(!owner.follow_policy||t.manual||!f.detection_result)return -1;
 int task=t.task_class_id>=0?t.task_class_id:owner.policy_task_class_id>=0?owner.policy_task_class_id:t.cls;
 // 当前真实图像块用于防止仍可见目标被其他目标占用，
 // 只作为连续性证据，不是重绑时必过的外观门限。
 Track measured=t;Tracker probe_owner;probe_owner.reacquire_lidar_valid=owner.reacquire_lidar_valid;probe_owner.reacquire_lidar_time=owner.reacquire_lidar_time;probe_owner.reacquire_region_valid=owner.reacquire_region_valid;probe_owner.reacquire_region_time=owner.reacquire_region_time;probe_owner.reacquire_image_prediction_valid=owner.reacquire_image_prediction_valid;probe_owner.reacquire_image_prediction_x=owner.reacquire_image_prediction_x;probe_owner.reacquire_image_prediction_y=owner.reacquire_image_prediction_y;
 if(cfg.image_tracking)image_track_selected(probe_owner,measured,f,cfg);
 if(measured.pixel_measured){int found=-1;double best=1e9;for(int i=0;i<(int)boxes.size();i++)if(boxes[i].eye==measured.box.eye){double dx=((boxes[i].x1+boxes[i].x2)-(measured.box.x1+measured.box.x2))*.5,dy=((boxes[i].y1+boxes[i].y2)-(measured.box.y1+measured.box.y2))*.5,d=std::hypot(dx,dy);if(d<=std::max(24.,std::min(measured.box.x2-measured.box.x1,measured.box.y2-measured.box.y1)*.35)&&d<best){best=d;found=i;}}if(found>=0)return found;if(measured.observed){t=std::move(measured);owner.selection_lost=false;return -3;}}
 struct Candidate{int index;double support;AssociationEvidence evidence;};std::vector<Candidate>candidates;
 for(int i=0;i<(int)boxes.size();i++)if(boxes[i].cls==task){auto e=association_evidence(owner,t,f,boxes[i],descriptors[i],i);bool spatial=e.motion>=.08&&e.reason!="OUTSIDE_EXPANDING_SPATIAL_REGION"&&e.reason!="CROSS_EYE_CANONICAL_GEOMETRY_UNAVAILABLE";if(!spatial)continue;double score=(e.available&2)?.65*e.motion+.35*e.region:e.motion;candidates.push_back({i,score,e});}
 if(std::any_of(candidates.begin(),candidates.end(),[&](const Candidate&c){return boxes[c.index].eye==t.box.eye;}))candidates.erase(std::remove_if(candidates.begin(),candidates.end(),[&](const Candidate&c){return boxes[c.index].eye!=t.box.eye;}),candidates.end());
 std::sort(candidates.begin(),candidates.end(),[](const Candidate&a,const Candidate&b){return a.support>b.support;});
 if(candidates.empty())return -1;if(candidates.size()>1&&candidates[0].support-candidates[1].support<.08){t.identity_reason="FOLLOW_SAME_CLASS_CANDIDATES_AMBIGUOUS";return -2;}
 const Box&candidate=boxes[candidates[0].index];double dx=((candidate.x1+candidate.x2)-(t.last_observed_box.x1+t.last_observed_box.x2))*.5,dy=((candidate.y1+candidate.y2)-(t.last_observed_box.y1+t.last_observed_box.y2))*.5;
 bool recent=t.last_detector_us&&t.last_detector_us<=f.time&&f.time-t.last_detector_us<=1500000;
 bool continuous=candidate.eye==t.last_observed_box.eye&&std::hypot(dx,dy)<=std::max(24.,std::min(t.last_observed_box.x2-t.last_observed_box.x1,t.last_observed_box.y2-t.last_observed_box.y1)*.35);
 t.association=candidates[0].evidence;rebind=!(recent&&continuous);return candidates[0].index;
}
