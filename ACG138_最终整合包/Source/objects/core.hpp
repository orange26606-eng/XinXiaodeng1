#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>
#include "object_anchor.hpp"
#include "../board/object_attributes.h"
namespace obj {
constexpr int OW=960,OH=1080,MAX_TRACKS=65;
struct Box {int cls=0,eye=0;float x1=0,y1=0,x2=0,y2=0,score=0;obj_attributes attributes{};};
struct Frame {bool detection_result=true;uint64_t epoch=0,id=0,time=0;std::vector<uint8_t>gray;std::vector<Box>boxes;};
struct Config {bool stereo=false,mount=false,image_tracking=true;int eye=0,confirm=2,lost_ms=3000,expire_ms=10000,max_tracks=65,disparities=64;double max_age_ms=1200,depth_max_age_ms=2000,pixel_bridge_ms=2000;int snapshot_jpeg_quality=80,snapshot_period_ms=250;double min_depth=.15,max_depth=5,scale=.25;AnchorLimits anchor;double size_box_small_m=.25,size_box_large_m=.45,size_bottle_small_m=.18,size_bottle_large_m=.28,size_cup_small_m=.10,size_cup_large_m=.18;int identity_reject_frames=10;std::string calibration,model_profile="coco80",classes_file;std::array<std::string,2>mono_files{};std::string extrinsics;};
Config load_config(const std::string&);
Frame read_frame(const std::string&);
Frame read_raw_frame(const std::string&);
struct Calibration {int w=0,h=0,left_eye=0;double baseline=0;std::array<double,16>Q{};std::array<double,9>R{};std::array<double,9>K{};std::array<double,5>D{};std::array<int,4>roi1{},roi2{};std::array<std::vector<float>,4>maps;void load(const std::string&);};
struct Depth {int w=0,h=0,row_begin=0,row_end=0;std::vector<float>z,disparity;std::vector<uint8_t>valid;};
Depth stereo(const Frame&,const Calibration&,const Config&);
Depth stereo_roi(const Frame&,const Calibration&,const Config&,const Box&);
// x、y、z为原光学相机坐标，world_*为独立可选的
// 已核验连续世界坐标元数据，associate()不填写这些字段。
struct CoarseCandidate {double range=0,error=0;int points=0;};
struct Point {std::array<CoarseCandidate,3>candidates{};int candidate_count=0;bool valid=false;double x=0,y=0,z=0,range=0,spread=0,error=0,fraction=0;int count=0;bool world_valid=false;double world_x=0,world_y=0;std::string reason="CALIBRATION_REQUIRED";uint64_t source_frame=0,source_time=0;int source_eye=0,geometry_eye=0;Box source_box;};
Point associate(const Box&,const Depth&,const Calibration&,const Config&);
struct Axis {double x=0,v=0,p00=100,p01=0,p11=100;void predict(double);void update(double);};
// 每检测固定取256个灰度样本，形成64个归一化单元及直方图。
// 初始参考不改变，四个受约束的视角模板不能替换它。
struct GrayAppearance {std::array<float,64>cells{};std::array<float,16>hist{};float mean=0,contrast=0;bool valid=false;};
struct WorldObservation {bool valid=false;double x=0,y=0,error=0;uint64_t frame=0,time=0;};
struct AssociationEvidence {double motion=0,region=0,appearance=0,class_shape=0,overlap=0,quality=0,cost=1;unsigned available=0;bool plausible=false;std::string reason="NO_ASSOCIATION";};
struct DetectorSource {Box box;uint64_t frame=0,time=0;};
struct SpatialSource {bool valid=false;int track_id=0,cls=0,eye=0;Box box;uint64_t frame=0,image_time=0,time=0;double x=0,y=0,error=0;};
struct Track {obj_attributes attributes{};int task_class_id=-1,observed_class_id=-1;uint32_t binding_generation=0;std::string binding_reason="INITIAL_DETECTION";uint64_t binding_source_us=0;std::deque<DetectorSource>detector_history;uint64_t last_detector_us=0,last_detector_frame=0,last_pixel_us=0,last_pixel_frame=0,last_lidar_us=0,manual_seed_us=0;unsigned detector_confirmations=0;bool pixel_measured=false;std::string identity_evidence="UNCONFIRMED";AssociationEvidence association;int reacquire_candidate_track_id=-1;std::string reacquire_candidate_reason="NONE";bool reacquire_candidate_valid=false;Box reacquire_candidate_box;uint64_t reacquire_candidate_frame=0,reacquire_candidate_time=0;int reacquire_candidate_hits=0;double reacquire_candidate_score=0;std::string reacquire_candidate_status="NONE";int id=0,cls=0,seen=0,depth_seen=0,consecutive=0,identity_missed=0;ObjectAnchor anchor;bool anchor_locked=false;uint64_t identity_rejections=0,identity_accepts=0,last_identity_event_us=0,last_identity_reject_us=0;std::string identity_reason="UNLOCKED";Box box,last_observed_box;std::array<Axis,4>kf;uint64_t observed_at=0,last_frame=0;bool observed=false,retired=false,manual=false,image_tracking_valid=false,detector_observed=false;double image_match_error=1,image_match_margin=0,image_tracking_ms=0;unsigned image_candidates=0;Point point,last_point,coarse;std::string status="TENTATIVE";GrayAppearance appearance,identity_reference,pixel_reference;std::array<GrayAppearance,4>pixel_templates{};int pixel_template_count=0,pixel_template_next=0;uint64_t pixel_template_time=0;std::array<GrayAppearance,4>templates{};int template_count=0,template_next=0;uint64_t template_time=0;WorldObservation world;};
struct FollowKeyframe {uint64_t frame=0,time=0;Box box;bool bearing_valid=false;double bearing=0;std::array<double,3>ray{};std::vector<uint8_t>roi;int priority=0;};
struct FollowExitMemory {int owner=-1;uint32_t generation=0;uint64_t epoch=0,processed_frame=0,processed_time=0;std::deque<FollowKeyframe>keyframes,positions;bool valid=false,bearing_valid=false;int direction=0,kind=0;double bearing=0,velocity=0,confidence=0;uint64_t source=0,frame=0;int eye=0;std::string reason="NO_REAL_TARGET_HISTORY";};
struct IdentityEvent{uint64_t time=0;int track=0;std::string code,reason;};
class Tracker {int anchor_owner=-1;int next_id=1;uint64_t epoch=0,last_time=0,last_frame_id=0;int reid_owner=-1,reid_candidate=-1,reid_hits=0;uint64_t reid_last_us=0;public:FollowExitMemory follow_exit;bool follow_policy=false,spatial_target_valid=false;int policy_task_class_id=-1;uint64_t policy_time=0,policy_binding_generation=0;double spatial_target_x=0,spatial_target_y=0,spatial_target_error=0;std::array<SpatialSource,16>spatial_sources{};unsigned spatial_source_count=0;std::array<bool,2>reacquire_image_prediction_valid{};std::array<double,2>reacquire_image_prediction_x{},reacquire_image_prediction_y{};std::array<double,64>reacquire_bearings{},reacquire_pitches{};std::array<bool,64>reacquire_geometry_valid{};uint64_t reacquire_geometry_frame=0,reacquire_region_time=0,reacquire_target_source_time=0,reacquire_region_after_context_us=0;bool reacquire_lidar_valid=false;uint64_t reacquire_lidar_time=0;double reacquire_lidar_bearing=0,reacquire_lidar_half_width=.35;bool reacquire_anchor_ray_valid=false,reacquire_cross_eye_geometry=false,reacquire_region_valid=false;double reacquire_anchor_bearing=0,reacquire_anchor_pitch=0,reacquire_region_bearing=0,reacquire_region_half_width=.35;std::string reacquire_region_context,reacquire_region_reason="REGION_NOT_PROVIDED";std::string world_context;std::deque<IdentityEvent>identity_events;uint64_t identity_event_total=0;std::vector<Track>tracks;int selected=-1;bool selection_lost=false;uint64_t reid_time_us=0;int reid_from=-1;void update(const Frame&,const std::vector<Box>&,const std::vector<Point>&,const Config&);void reset(uint64_t);int select_roi(const Box&,const GrayAppearance&,uint64_t,uint64_t);};
bool apply_world_bridge(const std::string&,Tracker&,uint64_t epoch,uint64_t frame_id,uint64_t now);
struct SnapshotIdentity {std::string id;uint64_t time=0;std::vector<int>tracks;std::map<int,Box>track_boxes;std::map<int,uint32_t>track_bindings;unsigned issued_eye_mask=3;std::vector<uint8_t>gray_quarter;};
SnapshotIdentity capture_snapshot_identity(const Frame&,const Tracker&);
struct Selection {uint64_t request=0;int track=-1;std::string state="CLEARED",reason="NO_SELECTION";};
void select_request(const std::string&,Tracker&,const std::deque<SnapshotIdentity>&,uint64_t,uint64_t,Selection&);
struct Mono {bool valid=false;int eye=0;double fx=0,fy=0,cx=0,cy=0;std::array<double,5>D{};std::string reason="MONO_FILE_MISSING";void load(const std::string&,int);std::array<double,3>ray(double,double)const;};
struct Extrinsics {bool valid=false;int left_eye=0;std::array<double,9>R{};std::array<double,3>T{};std::string reason="EXTRINSICS_FILE_MISSING";void load(const std::string&);};
struct EyeQuality {bool usable=false;double sharpness=0,contrast=0,mean=0;
 double gradient_x=0,gradient_y=0,saturation_fraction=0,normalized_detail=0;
 uint64_t source_frame=0,source_us=0;std::string focus_state="WAIT_FRAME",reason="WAIT_FRAME";};
struct Vision {std::string input_source="OBF1",pairing_reason="LEGACY_SELF_CONTAINED";uint64_t raw_updates=0,detector_updates=0,replayed_images=0,rejected_pairs=0,detector_source_id=0,detector_source_time=0;int active_eye=0;std::string eye_reason="INITIAL_REFERENCE_EYE";std::array<EyeQuality,2>eyes{};std::array<Mono,2>mono{};Extrinsics extrinsics;bool stereo_enabled=false,stereo_loaded=false;std::string stereo_reason="STEREO_DISABLED";uint64_t depth_frame=0,depth_time=0;double processing_ms=0;int roi_rows=0,full_rows=0;bool selected_roi=false;uint64_t submitted=0,replaced=0,completed=0;double snapshot_ms=0;uint64_t snapshot_submitted=0,snapshot_replaced=0,snapshot_completed=0,snapshot_bytes=0;std::string snapshot_reason="WAIT_SNAPSHOT";};
void update_follow_keyframes(Tracker&,const Frame&,const Vision&);
void prepare_reacquisition_geometry(Tracker&,const Frame&,const Vision&);
void prepare_reacquisition(Tracker&,const Frame&,const Vision&,const std::string&region_file,uint64_t now);
EyeQuality assess_eye(const Frame&,int,const Box* target=nullptr);
bool handoff_eye(Tracker&,const Frame&,Vision&);
std::array<double,3>canonical_ray(const Mono&,const Extrinsics&,double,double);
std::string state_json(const Frame&,const Tracker&,const Config&,bool,const std::string&,const Selection&,uint64_t,const std::string&image="",const Vision*vision=nullptr,int image_eye=-1);
std::string png_data(const Frame&,int eye);
std::string jpeg_data(const Frame&,int eye,int quality=80);
bool atomic_file(const std::string&,const std::string&);
uint64_t now_us();
const char*label(int);
const char*model_profile_name();size_t model_class_count();
}
