// 首个合理真实检测立即提供方向提示。
// 第二个独立且空间一致的观测恢复规范编号。
static int propose_reacquisition(Tracker&owner,Track&t,const Frame&f,const std::vector<Box>&boxes,const std::vector<GrayAppearance>&descriptors){
 struct Candidate{int index;AssociationEvidence evidence;};std::vector<Candidate>choices;
 for(int i=0;i<(int)boxes.size();i++){auto e=association_evidence(owner,t,f,boxes[i],descriptors[i],i);if(e.plausible)choices.push_back({i,e});}
 std::sort(choices.begin(),choices.end(),[](const Candidate&a,const Candidate&b){return a.evidence.quality>b.evidence.quality;});t.reacquire_candidate_valid=false;t.reacquire_candidate_track_id=-1;
 if(choices.empty()){t.reacquire_candidate_hits=0;t.reacquire_candidate_status="NONE";t.identity_reason=t.reacquire_candidate_reason="REACQUIRE_NO_SPATIALLY_PLAUSIBLE_DETECTION_KEEP_MEMORY";return -1;}
 t.association=choices[0].evidence;
 if(choices.size()>1&&choices[0].evidence.quality-choices[1].evidence.quality<.08){t.reacquire_candidate_hits=0;t.reacquire_candidate_status="AMBIGUOUS";t.identity_reason=t.reacquire_candidate_reason="REACQUIRE_MULTIPLE_PLAUSIBLE_OBJECTS_KEEP_MEMORY";return -1;}
 const Box&b=boxes[choices[0].index];double cx=(b.x1+b.x2)*.5,cy=(b.y1+b.y2)*.5,px=(t.reacquire_candidate_box.x1+t.reacquire_candidate_box.x2)*.5,py=(t.reacquire_candidate_box.y1+t.reacquire_candidate_box.y2)*.5;
 bool new_source=t.reacquire_candidate_time<f.time&&t.reacquire_candidate_frame<f.id&&f.time-t.reacquire_candidate_time<=1500000;
 bool spatial=t.reacquire_candidate_box.eye==b.eye?std::hypot(cx-px,cy-py)<=std::max(120.,std::sqrt(box_area(b))):owner.reacquire_cross_eye_geometry;
 bool continuing=t.reacquire_candidate_hits>0&&new_source&&spatial;
 t.reacquire_candidate_hits=continuing?t.reacquire_candidate_hits+1:1;t.reacquire_candidate_box=b;t.reacquire_candidate_time=f.time;t.reacquire_candidate_frame=f.id;t.reacquire_candidate_score=choices[0].evidence.quality;t.reacquire_candidate_valid=true;
 t.reacquire_candidate_status=t.reacquire_candidate_hits>=2?"CONFIRMED":"CANDIDATE";t.identity_reason=t.reacquire_candidate_reason=t.reacquire_candidate_hits>=2?"REACQUIRE_CONFIRMED_NEW_SOURCE_COMPOSITE":"REACQUIRE_FIRST_DETECTION_DIRECTION_ONLY";
 return t.reacquire_candidate_hits>=2?choices[0].index:-1;
}
