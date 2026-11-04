#pragma once
#include "latest_mailbox.h"
#include <vector>
#include <thread>
#include <cstdio>
#include <algorithm>
struct PublicationView{uint64_t count=0,total_ns=0,max_ns=0,last_sequence=0,last_source_ns=0,last_age_ns=0;};
class PublicationTiming{std::mutex mutex;PublicationView view;public:void observe(uint64_t begin,uint64_t end,uint64_t sequence,uint64_t source){std::lock_guard<std::mutex>l(mutex);view.count++;view.total_ns+=end-begin;view.max_ns=std::max(view.max_ns,end-begin);view.last_sequence=sequence;view.last_source_ns=source;view.last_age_ns=end>=source?end-source:0;}PublicationView get(){std::lock_guard<std::mutex>l(mutex);return view;}};
struct RawFrameJob {uint64_t sequence=0,stamp=0;std::vector<uint8_t>gray;};
template<class Raw,class Vision>class RawFramePublisher {
 Raw raw;Vision vision;acg::LatestMailbox<RawFrameJob>queue;PublicationTiming timing_data;std::thread worker;bool enabled=false;
public:
 explicit RawFramePublisher(uint64_t epoch,bool allow=true):raw(epoch){enabled=allow&&(raw.enabled()||vision.enabled());if(enabled)worker=std::thread([this]{std::unique_ptr<RawFrameJob>j;while(queue.take(j)){uint64_t begin=ns();raw.publish(j->gray.data(),j->gray.size(),j->sequence,j->stamp);vision.publish(j->gray.data(),j->gray.size(),j->sequence,j->stamp);timing_data.observe(begin,ns(),j->sequence,j->stamp);}});}
 ~RawFramePublisher(){queue.stop();if(worker.joinable())worker.join();}
 bool submit(const uint8_t*gray,size_t bytes,uint64_t sequence,uint64_t stamp){if(!enabled)return false;if(!gray||bytes<2073600||!sequence||!stamp)return false;auto j=std::make_unique<RawFrameJob>();j->sequence=sequence;j->stamp=stamp;j->gray.assign(gray,gray+2073600);return queue.put(std::move(j));}
 acg::MailboxStats stats(){return queue.stats();}
 PublicationView timing(){return timing_data.get();}
};
struct DetectionPublishJob{uint64_t sequence=0,stamp=0,total=0;std::vector<acg::Box>boxes;std::vector<uint8_t>gray;};
template<class Exporter>class DetectionPublisher {
 bool allow;Exporter&exporter;acg::LatestMailbox<DetectionPublishJob>queue;PublicationTiming timing_data;std::thread worker;
public:
 explicit DetectionPublisher(Exporter&e,bool enabled=true):allow(enabled),exporter(e){worker=std::thread([this]{yolo_fps_writer fps{};yolo_fps_writer_init(&fps);uint64_t last_total=0;std::unique_ptr<DetectionPublishJob>j;while(queue.take(j)){uint64_t begin=ns();
  // 发布任务持有同一槽位的双眼图像与原始灰度。
  if(allow&&!j->gray.empty())exporter.publish(j->gray.data(),j->gray.size(),j->boxes,j->sequence,j->stamp);
  if(allow){printf("FRAME %llu detections=%zu\n",(unsigned long long)j->sequence,j->boxes.size());for(const auto&b:j->boxes)printf("%s class=%d score=%.6f box=(%.2f,%.2f,%.2f,%.2f)\n",label(b.cls),b.cls,b.score,b.x1,b.y1,b.x2,b.y2);fflush(stdout);}
  last_total=std::max(last_total,j->total);if(allow)yolo_fps_writer_frame(&fps,ns(),last_total);timing_data.observe(begin,ns(),j->sequence,j->stamp);
 }});}
 ~DetectionPublisher(){queue.stop();if(worker.joinable())worker.join();}
 bool submit(std::unique_ptr<DetectionPublishJob>j){return queue.put(std::move(j));}
 acg::MailboxStats stats(){return queue.stats();}
 PublicationView timing(){return timing_data.get();}
};
