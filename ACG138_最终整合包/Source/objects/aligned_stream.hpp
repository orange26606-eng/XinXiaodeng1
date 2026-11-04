#pragma once
#include "core.hpp"
#include <algorithm>
#include <memory>
namespace obj {
// 源图固定最多20张，不建立无界接收队列。晚到的检测框
// 只作用于其所属OBF1源像素，再将之后真实到达的原始图像
// 按序送入选中目标的像素跟踪器，不修改检测框时间。
class AlignedStream {
 struct Entry{Frame frame;Tracker before;};std::vector<Entry>history;
 uint64_t base_time=0,last_detection_id=0,fence_time=0;int selected_fence=-1;uint32_t binding_fence=0;
 static void region_copy(Tracker&to,const Tracker&from){to.follow_policy=from.follow_policy;to.policy_task_class_id=from.policy_task_class_id;to.policy_time=from.policy_time;to.policy_binding_generation=from.policy_binding_generation;to.spatial_target_valid=from.spatial_target_valid;to.spatial_target_x=from.spatial_target_x;to.spatial_target_y=from.spatial_target_y;to.spatial_target_error=from.spatial_target_error;to.spatial_sources=from.spatial_sources;to.spatial_source_count=from.spatial_source_count;to.reacquire_image_prediction_valid=from.reacquire_image_prediction_valid;to.reacquire_image_prediction_x=from.reacquire_image_prediction_x;to.reacquire_image_prediction_y=from.reacquire_image_prediction_y;to.reacquire_lidar_valid=from.reacquire_lidar_valid;to.reacquire_lidar_time=from.reacquire_lidar_time;to.reacquire_lidar_bearing=from.reacquire_lidar_bearing;to.reacquire_lidar_half_width=from.reacquire_lidar_half_width;to.reacquire_region_valid=from.reacquire_region_valid;to.reacquire_region_time=from.reacquire_region_time;to.reacquire_target_source_time=from.reacquire_target_source_time;to.reacquire_region_bearing=from.reacquire_region_bearing;to.reacquire_region_half_width=from.reacquire_region_half_width;to.reacquire_region_context=from.reacquire_region_context;to.reacquire_region_after_context_us=from.reacquire_region_after_context_us;to.reacquire_region_reason=from.reacquire_region_reason;}
 void bound(){while(history.size()>20){base_time=history.front().frame.time;history.erase(history.begin());}}
public:
 Tracker tracker;Frame latest,detector_snapshot;Tracker detector_snapshot_tracks;bool snapshot_ready=false;
 uint64_t raw_updates=0,detector_updates=0,replayed_images=0,rejected=0,detector_source_time=0,detector_source_id=0;std::string pairing_reason="WAIT_RAW_FRAME";
 AlignedStream(){history.reserve(21);}
 size_t retained_frames()const{return history.size();}
 void reset(uint64_t epoch){tracker.reset(epoch);history.clear();latest={};latest.epoch=epoch;detector_snapshot={};snapshot_ready=false;base_time=last_detection_id=fence_time=0;selected_fence=-1;binding_fence=0;pairing_reason="SOURCE_EPOCH_RESET";}
 // 显式选择形成身份屏障，迟到的选择前任务
 // 不能回退当前选择或重新分配其规范目标编号。
 void source_checkpoint(){selected_fence=tracker.selected;binding_fence=0;for(const auto&t:tracker.tracks)if(t.id==tracker.selected)binding_fence=t.binding_generation;fence_time=latest.time;base_time=fence_time;history.clear();snapshot_ready=false;pairing_reason="IDENTITY_OR_EYE_SOURCE_CHECKPOINT";}
 void selection_fence(){uint32_t gen=0;for(const auto&t:tracker.tracks)if(t.id==tracker.selected)gen=t.binding_generation;if(tracker.selected==selected_fence&&gen==binding_fence)return;source_checkpoint();pairing_reason="USER_SELECTION_OR_BINDING_FENCE";}
 bool raw(Frame frame,const Config&cfg,const Vision&vision,const std::string&region,uint64_t now){
  if(frame.epoch!=latest.epoch)reset(frame.epoch);if(latest.time&&(frame.id<=latest.id||frame.time<=latest.time))return false;
  frame.detection_result=false;frame.boxes.clear();prepare_reacquisition(tracker,frame,vision,region,now);Entry entry;entry.before=tracker;entry.frame=std::move(frame);
  tracker.update(entry.frame,{}, {},cfg);update_follow_keyframes(tracker,entry.frame,vision);latest=entry.frame;history.push_back(std::move(entry));bound();++raw_updates;pairing_reason="RAW_CURRENT_IMAGE_MEASURED";return true;
 }
 bool detection(Frame detection,const Config&cfg,const Vision&vision,uint64_t now){
  if(!latest.time||detection.epoch!=latest.epoch){++rejected;pairing_reason="DETECTOR_EPOCH_NOT_RAW_EPOCH";return false;}
  if(detection.id<=last_detection_id)return false;
  if(detection.time<=fence_time||detection.time<=base_time){++rejected;last_detection_id=detection.id;pairing_reason="DETECTOR_BEFORE_RETAINED_HISTORY_OR_SELECTION_FENCE";return false;}
  if(detection.time>now||now-detection.time>(uint64_t)(cfg.max_age_ms*1000)){++rejected;last_detection_id=detection.id;pairing_reason="DETECTOR_SOURCE_EXPIRED";return false;}
  if(detection.id>latest.id){pairing_reason="DETECTOR_WAIT_MATCHING_RAW_SEQUENCE";return false;}
  auto position=std::lower_bound(history.begin(),history.end(),detection.id,[](const Entry&a,uint64_t id){return a.frame.id<id;});size_t index=(size_t)(position-history.begin());
  if(position!=history.end()&&position->frame.id==detection.id){
   if(position->frame.time!=detection.time||position->frame.gray!=detection.gray){++rejected;last_detection_id=detection.id;pairing_reason="DETECTOR_RAW_SOURCE_TIME_OR_PIXELS_MISMATCH";return false;}
   position->frame.boxes=std::move(detection.boxes);position->frame.detection_result=true;
  }else{
   // 最新原图文件跳过的图像仍可从OBF1自带的
   // 不可变原始像素恢复，不能配给下一张原图。
   if(index>=history.size()||(index&&detection.time<=history[index-1].frame.time)||detection.time>=history[index].frame.time){++rejected;last_detection_id=detection.id;pairing_reason="DETECTOR_SOURCE_ORDER_INVALID";return false;}
   Entry entry;entry.before=history[index].before;entry.frame=std::move(detection);entry.frame.detection_result=true;history.insert(history.begin()+(ptrdiff_t)index,std::move(entry));
  }
  Tracker previous=tracker,work=history[index].before;
  for(size_t i=index;i<history.size();i++){
   Tracker metadata=history[i].before;region_copy(work,metadata);if(work.reacquire_region_time>history[i].frame.time+100000)work.reacquire_region_valid=false;if(work.reacquire_lidar_time>history[i].frame.time+100000)work.reacquire_lidar_valid=false;
   prepare_reacquisition_geometry(work,history[i].frame,vision);history[i].before=work;
   std::vector<Point>points(history[i].frame.boxes.size());work.update(history[i].frame,history[i].frame.boxes,points,cfg);update_follow_keyframes(work,history[i].frame,vision);
   if(i==index){detector_snapshot=history[i].frame;detector_snapshot_tracks=work;snapshot_ready=true;}
   if(i>index)++replayed_images;
  }
  // 即使新检测触发同一规范目标历史的回放，异步双目结果
  // 仍保留原源编号和时间戳。
  for(auto&t:work.tracks)for(const auto&old:previous.tracks)if(t.id==old.id&&t.binding_generation==old.binding_generation&&old.last_point.source_time>t.last_point.source_time)t.last_point=old.last_point;
  tracker=std::move(work);latest=history.back().frame;last_detection_id=detector_snapshot.id;detector_source_id=detector_snapshot.id;detector_source_time=detector_snapshot.time;++detector_updates;pairing_reason="DETECTOR_ORIGINAL_SOURCE_PAIRED_RAW_REPLAYED";bound();return true;
 }
};
}
