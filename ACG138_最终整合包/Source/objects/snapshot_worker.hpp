#pragma once
#include "core.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
namespace obj {
struct SnapshotJob {Frame frame;Tracker tracker;Selection selection;Vision vision;bool calibrated=false;std::string reason;uint64_t revision=0;};
// 一个实际编码任务加一个最新待处理源，各眼JSON携带
// 自己的像素及元数据，按已提交镜头分别确认部分磁盘写入。
class SnapshotWorker {
 Config config;std::string path;std::thread thread;std::mutex mutex;std::condition_variable cv;
 bool quit=false,busy=false;uint64_t epoch=0,revision=0,submitted=0,replaced=0,completed=0,bytes=0,last_done_frame=0,last_committed_frame=0,last_committed_time=0;int selected_fence=-2;uint32_t binding_fence=0;double elapsed_ms=0;std::string reason="WAIT_SNAPSHOT";
 std::unique_ptr<SnapshotJob>pending;std::deque<SnapshotIdentity>published;
 static uint32_t generation(const Tracker&t){for(const auto&track:t.tracks)if(track.id==t.selected)return track.binding_generation;return 0;}
 void issued(SnapshotIdentity identity,unsigned mask){if(!mask)return;identity.issued_eye_mask=mask;identity.tracks.erase(std::remove_if(identity.tracks.begin(),identity.tracks.end(),[&](int id){auto it=identity.track_boxes.find(id);return it==identity.track_boxes.end()||!(mask&(1u<<it->second.eye));}),identity.tracks.end());if(published.size()>=40)published.pop_front();published.push_back(std::move(identity));}
 void run(){for(;;){std::unique_ptr<SnapshotJob>job;{std::unique_lock<std::mutex>lock(mutex);cv.wait(lock,[&]{return quit||pending;});if(quit)return;job=std::move(pending);busy=true;cv.notify_all();}
  auto begin=now_us();try{
   std::array<std::string,2>payload;for(int eye=0;eye<2;eye++){std::string image=jpeg_data(job->frame,eye,config.snapshot_jpeg_quality);payload[eye]=state_json(job->frame,job->tracker,config,job->calibrated,job->reason,job->selection,now_us(),image,&job->vision,eye);}
   SnapshotIdentity identity=capture_snapshot_identity(job->frame,job->tracker);std::lock_guard<std::mutex>lock(mutex);
   if(job->frame.epoch==epoch&&job->revision==revision&&job->frame.id>last_committed_frame&&job->frame.time>last_committed_time){
    unsigned mask=0;bool eyes=true;for(int eye=0;eye<2;eye++){auto filename=(std::filesystem::path(path).parent_path()/("objects-snapshot-eye"+std::to_string(eye)+".json")).string();if(atomic_file(filename,payload[eye]))mask|=1u<<eye;else eyes=false;}
    if(mask){last_committed_frame=job->frame.id;last_committed_time=job->frame.time;issued(std::move(identity),mask);}
    int active=job->vision.active_eye;if(active<0||active>1)throw std::runtime_error("SNAPSHOT_ACTIVE_EYE_INVALID");
    if(!eyes){reason="EYE_SNAPSHOT_PARTIAL_COMMIT";}else if(!atomic_file(path,payload[active])){reason="ACTIVE_SNAPSHOT_WRITE_FAILED_EYES_COMMITTED";}else{completed++;bytes=payload[0].size()+payload[1].size();last_done_frame=job->frame.id;reason="JPEG_SNAPSHOT_PUBLISHED";}
   }else reason=job->frame.epoch!=epoch?"SNAPSHOT_OLD_EPOCH_DISCARDED":job->revision!=revision?"SNAPSHOT_OLD_SELECTION_OR_BINDING_DISCARDED":"SNAPSHOT_DUPLICATE_OR_OLDER_SOURCE_DISCARDED";
   elapsed_ms=(now_us()-begin)/1000.;busy=false;cv.notify_all();
  }catch(const std::exception&e){std::lock_guard<std::mutex>lock(mutex);reason=e.what();busy=false;cv.notify_all();}
 }}
 public:SnapshotWorker(const Config&c,const std::string&p):config(c),path(p){thread=std::thread([this]{run();});}~SnapshotWorker(){{std::lock_guard<std::mutex>lock(mutex);quit=true;pending.reset();}cv.notify_all();thread.join();}
 void reset(uint64_t e){std::lock_guard<std::mutex>lock(mutex);epoch=e;++revision;pending.reset();published.clear();last_done_frame=last_committed_frame=last_committed_time=0;selected_fence=-2;binding_fence=0;}
 bool identity_fence(uint64_t e,const Tracker&t){std::lock_guard<std::mutex>lock(mutex);if(e!=epoch)return false;uint32_t g=generation(t);if(selected_fence==t.selected&&binding_fence==g)return false;selected_fence=t.selected;binding_fence=g;++revision;pending.reset();return true;}
 void submit(const Frame&f,const Tracker&t,const Selection&s,const Vision&v,bool calibrated,const std::string&r){auto job=std::make_unique<SnapshotJob>();job->frame=f;job->tracker=t;job->selection=s;job->vision=v;job->calibrated=calibrated;job->reason=r;std::lock_guard<std::mutex>lock(mutex);if(epoch!=f.epoch)return;if(selected_fence==-2){selected_fence=t.selected;binding_fence=generation(t);}if(selected_fence!=t.selected||binding_fence!=generation(t)){reason="SNAPSHOT_JOB_IDENTITY_MISMATCH";return;}if(f.id<=last_committed_frame||f.time<=last_committed_time||(pending&&f.id<=pending->frame.id)){reason="SNAPSHOT_DUPLICATE_OR_OLDER_SOURCE_DISCARDED";return;}job->revision=revision;if(pending)replaced++;pending=std::move(job);submitted++;cv.notify_all();}
 void collect(std::deque<SnapshotIdentity>&history,Vision&v,uint64_t now){std::lock_guard<std::mutex>lock(mutex);while(!published.empty()){auto photo=std::move(published.front());published.pop_front();if(history.empty()||history.back().id!=photo.id)history.push_back(std::move(photo));}while(history.size()>40||(!history.empty()&&now>history.front().time+8000000))history.pop_front();v.snapshot_ms=elapsed_ms;v.snapshot_submitted=submitted;v.snapshot_replaced=replaced;v.snapshot_completed=completed;v.snapshot_bytes=bytes;v.snapshot_reason=reason;}
 bool wait(uint64_t frame,unsigned ms){std::unique_lock<std::mutex>lock(mutex);return cv.wait_for(lock,std::chrono::milliseconds(ms),[&]{return last_done_frame>=frame||(!busy&&!pending);})&&last_done_frame>=frame;}
 bool processing(){std::lock_guard<std::mutex>lock(mutex);return busy;}
};
}
