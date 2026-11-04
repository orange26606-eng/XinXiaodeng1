/* 有界双目推理流水线，通过dlopen使用现有RKNN运行库。采集与预览独立执行，三个工作线程分别持有推理上下文；不发送电机命令，也不重复打开已被占用的采集节点。

 */
#include "rknn_api.h"
#include "yolo_math.h"
#include "model_profile.h"
#include "yolo_fps_writer.h"
#include "real_frames.h"
#include "vision_frame_export.h"
#include "object_frame_export.h"
#include "object_raw_export.h"
#include "latest_mailbox.h"
#include "capture_queue.h"
#include "attribute_label.h"
#include "../reference/imu_font_data.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <dlfcn.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/videodev2.h>
#include <mutex>
#include <memory>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
using namespace acg;
static volatile sig_atomic_t interrupted;
static void stop_signal(int signal){interrupted=signal;}
static uint64_t ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void check(bool ok,const char*text){if(!ok)throw std::runtime_error(text);}
static int ctl(int fd,unsigned long cmd,void*p){int r;do{r=ioctl(fd,cmd,p);}while(r<0&&errno==EINTR&&!interrupted);return r;}
struct Api {
    void *lib=nullptr;
    decltype(&rknn_init) init;decltype(&rknn_destroy) destroy;decltype(&rknn_query) query;
    decltype(&rknn_set_core_mask) core;decltype(&rknn_inputs_set) inputs;
    decltype(&rknn_run) run;decltype(&rknn_outputs_get) get;decltype(&rknn_outputs_release) release;
    template<class T>void symbol(T&function,const char*name){function=reinterpret_cast<T>(dlsym(lib,name));check(function!=nullptr,name);}
    Api(){lib=dlopen("librknnrt.so",RTLD_NOW|RTLD_LOCAL);check(lib,"Cannot load existing librknnrt.so");
        symbol(init,"rknn_init");symbol(destroy,"rknn_destroy");symbol(query,"rknn_query");symbol(core,"rknn_set_core_mask");symbol(inputs,"rknn_inputs_set");symbol(run,"rknn_run");symbol(get,"rknn_outputs_get");symbol(release,"rknn_outputs_release");}
    ~Api(){if(lib)dlclose(lib);}
};
#include "rknn_context_worker.h"
struct Capture {
    int fd=-1;std::vector<void*> maps;std::vector<size_t> sizes;
    explicit Capture(const char*path){
        fd=open(path,O_RDWR|O_NONBLOCK|O_CLOEXEC);check(fd>=0,"Capture open failed");
        try{
            v4l2_format f{};f.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;check(ctl(fd,VIDIOC_G_FMT,&f)==0,"Capture format query failed");
            check(f.fmt.pix.width==1920&&f.fmt.pix.height==1080&&f.fmt.pix.pixelformat==V4L2_PIX_FMT_NV12&&f.fmt.pix.bytesperline==1920,"Expected contiguous stereo NV12 1920x1080");
            unsigned matrix=f.fmt.pix.ycbcr_enc?(unsigned)f.fmt.pix.ycbcr_enc:(unsigned)V4L2_MAP_YCBCR_ENC_DEFAULT(f.fmt.pix.colorspace);
            check(matrix==V4L2_YCBCR_ENC_601&&f.fmt.pix.quantization!=V4L2_QUANTIZATION_FULL_RANGE,"Expected limited-range BT.601 NV12");
            v4l2_requestbuffers req{};req.count=4;req.type=f.type;req.memory=V4L2_MEMORY_MMAP;
            check(ctl(fd,VIDIOC_REQBUFS,&req)==0&&req.count>0&&req.count<=8,"Capture buffers unavailable");
            for(unsigned i=0;i<req.count;i++){
                v4l2_buffer b{};b.type=f.type;b.memory=V4L2_MEMORY_MMAP;b.index=i;check(ctl(fd,VIDIOC_QUERYBUF,&b)==0,"QUERYBUF failed");
                void*p=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);check(p!=MAP_FAILED,"Capture mmap failed");maps.push_back(p);sizes.push_back(b.length);check(ctl(fd,VIDIOC_QBUF,&b)==0,"QBUF failed");
            }
            v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;check(ctl(fd,VIDIOC_STREAMON,&type)==0,"Capture STREAMON failed");
        }catch(...){close_all();throw;}
    }
    void close_all(){if(fd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(fd,VIDIOC_STREAMOFF,&t);for(unsigned i=0;i<maps.size();i++)munmap(maps[i],sizes[i]);close(fd);fd=-1;}}
    ~Capture(){close_all();}
    uint64_t older_dropped=0,error_dropped=0;unsigned source_sequence=0;
    bool next(std::vector<uint8_t>&frame){
        pollfd p{fd,POLLIN,0};int rc=poll(&p,1,100);if(rc<0&&errno==EINTR)return false;check(rc>=0&&!(p.revents&(POLLERR|POLLHUP|POLLNVAL)),"Capture stopped");if(!(p.revents&POLLIN))return false;
        auto take=[&](CaptureFrame&o){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;int ret=ioctl(fd,VIDIOC_DQBUF,&b);if(ret<0){if(errno==EAGAIN)return CaptureIo::Again;if(errno==EINTR)return CaptureIo::Interrupted;return CaptureIo::Failed;}o={b.index,b.bytesused,b.sequence,(b.flags&V4L2_BUF_FLAG_ERROR)!=0};return CaptureIo::Frame;};
        auto put=[&](unsigned index){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;b.index=index;return ctl(fd,VIDIOC_QBUF,&b)==0;};
        auto result=drain_capture((unsigned)maps.size(),sizes.data(),frame.size(),take,put,[]{return interrupted!=0;});older_dropped+=result.stats.older_dropped;error_dropped+=result.stats.error_dropped;
        if(result.status==CaptureDrainStatus::Empty||result.status==CaptureDrainStatus::Stopped)return false;
        if(result.status!=CaptureDrainStatus::Ready)throw std::runtime_error(std::string("CAPTURE_LATEST_FAILED reason=")+capture_drain_reason_text(result.reason));
        memcpy(frame.data(),maps[result.frame.index],frame.size());source_sequence=result.frame.sequence;check(put(result.frame.index),"Capture selected QBUF failed");return true;
    }
};
struct Output {
    int fd=-1;
    explicit Output(const char*path){fd=open(path,O_WRONLY|O_NONBLOCK|O_CLOEXEC);check(fd>=0,"Output open failed");
        v4l2_format f{};f.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;f.fmt.pix.width=1920;f.fmt.pix.height=1080;f.fmt.pix.pixelformat=V4L2_PIX_FMT_NV12;f.fmt.pix.field=V4L2_FIELD_NONE;f.fmt.pix.bytesperline=1920;f.fmt.pix.sizeimage=1920*1080*3/2;f.fmt.pix.colorspace=V4L2_COLORSPACE_SMPTE170M;f.fmt.pix.ycbcr_enc=V4L2_YCBCR_ENC_601;f.fmt.pix.quantization=V4L2_QUANTIZATION_LIM_RANGE;
        if(ctl(fd,VIDIOC_S_FMT,&f)||f.fmt.pix.width!=1920||f.fmt.pix.height!=1080||f.fmt.pix.pixelformat!=V4L2_PIX_FMT_NV12){close(fd);fd=-1;throw std::runtime_error("Output layout rejected");}
        v4l2_streamparm p{};p.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;p.parm.output.timeperframe.numerator=1;p.parm.output.timeperframe.denominator=30;
        if(ctl(fd,VIDIOC_S_PARM,&p)||real_frames_configure(fd)){close(fd);fd=-1;throw std::runtime_error("Cannot disable loopback duplicate/timeout frames");}
    }
    ~Output(){if(fd>=0)close(fd);}
    void write_frame(const std::vector<uint8_t>&frame){
        uint64_t end=ns()+1000000000ULL;
        for(;;){ssize_t n=write(fd,frame.data(),frame.size());if(n==(ssize_t)frame.size())return;if(n<0&&(errno==EINTR||errno==EAGAIN)&&!interrupted&&ns()<end){poll(nullptr,0,1);continue;}throw std::runtime_error("Preview write failed");}
    }
};
static const char*label(int category);
static void annotate(std::vector<uint8_t>&nv12,const std::vector<Box>&boxes){
    auto pixel=[&](int x,int y){if(x>=0&&x<1920&&y>=0&&y<1080){nv12[y*1920+x]=190;unsigned uv=1920*1080+(y/2)*1920+(x&~1);nv12[uv]=54;nv12[uv+1]=34;}};
    for(const auto&b:boxes){int x1=std::clamp((int)b.x1,0,1919),x2=std::clamp((int)b.x2,0,1919),y1=std::clamp((int)b.y1,0,1079),y2=std::clamp((int)b.y2,0,1079);
        for(int thickness=0;thickness<3;thickness++){for(int x=x1;x<=x2;x++){pixel(x,y1+thickness);pixel(x,y2-thickness);}for(int y=y1;y<=y2;y++){pixel(x1+thickness,y);pixel(x2-thickness,y);}}
        draw_attribute_label(nv12,b,attribute_caption(label(b.cls),b));
    }
}
static const ModelProfile& active_profile(){static const ModelProfile p=ModelProfile::load();return p;}
#include "frame_publish_workers.h"
struct Slot {std::vector<uint8_t>pixels=std::vector<uint8_t>(1920*1080*3/2);uint64_t sequence=0,stamp=0;int remaining=0;std::vector<Box>results;};
static const char*label(int category){return active_profile().label(category);}
struct Pipeline {
 bool export_enabled;ObjectFrameExport objects_export;RawFramePublisher<ObjectRawExport,VisionFrameExport>sources;DetectionPublisher<ObjectFrameExport>publisher;
 std::mutex mutex;std::condition_variable cv;std::array<Slot,2>slots;std::deque<std::pair<int,int>>jobs;
 bool quit=false;std::atomic<bool>failed{false};std::vector<Box>latest;uint64_t latest_sequence=0,latest_stamp=0,completed=0;
 uint64_t timing_count=0,timing_start=0;std::array<uint64_t,6>timing_sum{},timing_max{};
 explicit Pipeline(bool allow=true):export_enabled(allow),sources(objects_export.session_epoch(),allow),publisher(objects_export,allow){}
 void worker(Context&context){
  uint64_t failure_sequence=0,failure_stamp=0;int failure_eye=-1;
  try{for(;;){int slot,eye;{std::unique_lock<std::mutex>lock(mutex);cv.wait(lock,[&]{return quit||!jobs.empty();});if(quit)return;slot=jobs.front().first;eye=jobs.front().second;jobs.pop_front();failure_sequence=slots[slot].sequence;failure_stamp=slots[slot].stamp;failure_eye=eye;}
   auto boxes=context.infer(slots[slot].pixels.data(),eye);for(auto&b:boxes)extract_color(slots[slot].pixels.data(),slots[slot].pixels.size(),b);uint64_t now=ns();std::unique_ptr<DetectionPublishJob>finished;std::array<uint64_t,6>sum{},maximum{};uint64_t samples=0,age=0;
   {std::lock_guard<std::mutex>lock(mutex);auto&s=slots[slot];s.results.insert(s.results.end(),boxes.begin(),boxes.end());
    if(s.remaining==1){finished=std::make_unique<DetectionPublishJob>();finished->sequence=s.sequence;finished->stamp=s.stamp;finished->boxes=s.results;if(export_enabled&&objects_export.enabled())finished->gray.assign(s.pixels.begin(),s.pixels.begin()+2073600);}
    if(--s.remaining==0){++completed;finished->total=completed;if(s.sequence>latest_sequence){latest_sequence=s.sequence;latest_stamp=s.stamp;latest=s.results;}}
    const auto&t=context.timing;std::array<uint64_t,6>times={t.preprocess,t.input,t.run,t.get,t.decode,t.release};for(unsigned k=0;k<6;k++){timing_sum[k]+=times[k];timing_max[k]=std::max(timing_max[k],times[k]);}++timing_count;if(!timing_start)timing_start=now;
    if(now-timing_start>=1000000000ULL){sum=timing_sum;maximum=timing_max;samples=timing_count;timing_sum.fill(0);timing_max.fill(0);timing_count=0;timing_start=now;age=now>=s.stamp?(now-s.stamp)/1000000:0;}
   }
   if(finished)publisher.submit(std::move(finished));
   if(samples){static const char*names[]={"preprocess","inputs_set","npu_run","outputs_get","decode","release"};fprintf(stderr,"MODEL_TIMING samples=%llu eye=%d source_age_ms=%llu preprocess_backend=%s",(unsigned long long)samples,eye,(unsigned long long)age,ACG_HAS_AARCH64_NEON?"NEON_EXACT":"SCALAR_EXACT");for(unsigned k=0;k<6;k++)fprintf(stderr," %s_avg_ms=%.3f %s_max_ms=%.3f",names[k],sum[k]/(double)samples/1e6,names[k],maximum[k]/1e6);fprintf(stderr,"\n");}
  }}catch(const std::exception&e){fprintf(stderr,"NPU_FAILURE source_frame=%llu eye=%d source_age_ms=%.3f reason=%s\n",(unsigned long long)failure_sequence,failure_eye,failure_stamp?(ns()-failure_stamp)/1e6:-1.,e.what());failed.store(true);}
 }
 bool submit(const std::vector<uint8_t>&frame,uint64_t sequence,uint64_t stamp){std::lock_guard<std::mutex>lock(mutex);for(unsigned i=0;i<slots.size();i++)if(slots[i].remaining==0){auto&s=slots[i];s.pixels=frame;s.sequence=sequence;s.stamp=stamp;s.remaining=2;s.results.clear();jobs.emplace_back(i,0);jobs.emplace_back(i,1);cv.notify_all();return true;}return false;}
 void shutdown(){std::lock_guard<std::mutex>lock(mutex);quit=true;cv.notify_all();}
};
int main(int argc,char**argv){
    std::string model_path,device,output,ready,benchmark_nv12;bool no_detect=false,probe=false,benchmark=false;unsigned iterations=120;
    for(int i=1;i<argc;i++){
        std::string a=argv[i];if(a=="--model"&&i+1<argc)model_path=argv[++i];else if(a=="--device"&&i+1<argc)device=argv[++i];else if(a=="--display-device"&&i+1<argc)output=argv[++i];else if(a=="--ready-file"&&i+1<argc)ready=argv[++i];else if(a=="--no-detect")no_detect=true;else if(a=="--probe")probe=true;else if(a=="--benchmark-pipeline")benchmark=true;else if(a=="--benchmark-nv12"&&i+1<argc)benchmark_nv12=argv[++i];else if(a=="--iterations"&&i+1<argc){char*end=nullptr;errno=0;unsigned long n=std::strtoul(argv[++i],&end,10);if(errno||!end||*end||!n||n>1000){fprintf(stderr,"iterations must1..1000\n");return 2;}iterations=(unsigned)n;}
        else if((a=="--frames"||a=="--log-every")&&i+1<argc)++i;else if(a=="--no-save"){}else{fprintf(stderr,"Usage: %s --model MODEL --device INPUT --display-device OUTPUT --ready-file PATH [--no-detect|--probe]\n",argv[0]);return 2;}
    }
    struct sigaction action{};action.sa_handler=stop_signal;sigemptyset(&action.sa_mask);sigaction(SIGINT,&action,nullptr);sigaction(SIGTERM,&action,nullptr);sigaction(SIGHUP,&action,nullptr);
    std::unique_ptr<Pipeline>pipeline_owner;std::vector<std::thread>workers;bool ready_owned=false;int result=0;
    try{
        pipeline_owner=std::make_unique<Pipeline>(!benchmark);Pipeline&pipeline=*pipeline_owner;
        std::unique_ptr<Api>api;std::vector<std::unique_ptr<Context>>contexts;
        struct JoinWorkers{Pipeline&p;std::vector<std::thread>&threads;~JoinWorkers(){p.shutdown();for(auto&t:threads)if(t.joinable())t.join();threads.clear();}} joiner{pipeline,workers};
        if(!no_detect){
            std::ifstream file(model_path,std::ios::binary);check((bool)file,"Cannot read YOLO11 model");std::vector<char>model((std::istreambuf_iterator<char>(file)),{});check(!model.empty()&&model.size()<256*1024*1024,"Invalid model size");
            api=std::make_unique<Api>();for(unsigned i=0;i<3;i++)contexts.push_back(std::make_unique<Context>(*api,model,i));
        }
        if(probe){check(!no_detect,"Probe requires detection");std::vector<uint8_t>gray(1920*1080*3/2,128);std::fill(gray.begin(),gray.begin()+1920*1080,126);for(auto&c:contexts)c->infer(gray.data(),0);puts("PROBE_OK: three independent contexts on NPU0/1/2");return 0;}
        if(benchmark){
            check(!no_detect&&!benchmark_nv12.empty(),"Benchmark requires --model and --benchmark-nv12; no cameras opened");std::ifstream in(benchmark_nv12,std::ios::binary);check((bool)in,"Benchmark NV12 open failed");std::vector<uint8_t>fixed((std::istreambuf_iterator<char>(in)),{});check(fixed.size()==1920*1080*3/2,"Benchmark NV12 must be exactly3110400bytes");
            for(auto&c:contexts)c->infer(fixed.data(),0);for(auto&c:contexts)workers.emplace_back([&pipeline,c=c.get()]{pipeline.worker(*c);});uint64_t begin=ns(),deadline=begin+600000000000ULL;
            for(unsigned n=0;n<iterations&&!interrupted;){if(pipeline.failed.load())throw std::runtime_error("Benchmark inference failed");if(ns()>deadline)throw std::runtime_error("Benchmark exceeded600seconds");if(pipeline.submit(fixed,n+1,ns()))++n;else std::this_thread::sleep_for(std::chrono::milliseconds(1));}
            for(;;){uint64_t done;{std::lock_guard<std::mutex>lock(pipeline.mutex);done=pipeline.completed;}if(done>=iterations||interrupted||pipeline.failed.load())break;if(ns()>deadline)throw std::runtime_error("Benchmark completion timeout");std::this_thread::sleep_for(std::chrono::milliseconds(1));}
            uint64_t end=ns(),done;{std::lock_guard<std::mutex>lock(pipeline.mutex);done=pipeline.completed;}pipeline.shutdown();for(auto&t:workers)if(t.joinable())t.join();workers.clear();
            fprintf(stdout,"BENCHMARK source=RECORDED_NV12 stereo_completed=%llu seconds=%.6f stereo_fps=%.3f eye_inferences=%llu capture_opened=0 preview_opened=0 object_exports=0 motor_commands=0 not_camera_fps=1\n",(unsigned long long)done,(end-begin)/1e9,done*1e9/(end-begin),(unsigned long long)(done*2));return interrupted?128+interrupted:pipeline.failed.load()?1:0;
        }
        check(!device.empty()&&!output.empty()&&device!=output&&!ready.empty(),"Missing distinct video nodes/readiness path");check(access(ready.c_str(),F_OK)!=0&&errno==ENOENT,"Readiness file already exists");
        Capture capture(device.c_str());Output preview(output.c_str());
        for(auto&context:contexts)workers.emplace_back([&pipeline,c=context.get()]{pipeline.worker(*c);});
        std::vector<uint8_t>frame(1920*1080*3/2);uint64_t sequence=0,skipped=0,last=ns(),period=last,last_sequence=0;std::array<uint64_t,4>stage_sum{},stage_max{};uint64_t stage_count=0;
        try{while(!interrupted&&!pipeline.failed.load()){
            uint64_t capture_begin=ns();if(!capture.next(frame)){check(ns()-last<10000000000ULL,"No fresh camera frame for 10 seconds");continue;}
            last=ns();++sequence;uint64_t copy_begin=last;pipeline.sources.submit(frame.data(),frame.size(),sequence,last);uint64_t copy_end=ns();
            if(!no_detect&&!pipeline.submit(frame,sequence,last))++skipped;
            std::vector<Box>overlay;{std::lock_guard<std::mutex>lock(pipeline.mutex);if(pipeline.latest_stamp&&last>=pipeline.latest_stamp&&last-pipeline.latest_stamp<=500000000ULL)overlay=pipeline.latest;}annotate(frame,overlay);
            uint64_t preview_begin=ns();preview.write_frame(frame);uint64_t preview_end=ns();std::array<uint64_t,4>stage_times={last-capture_begin,copy_end-copy_begin,preview_begin-copy_end,preview_end-preview_begin};for(unsigned k=0;k<4;k++){stage_sum[k]+=stage_times[k];stage_max[k]=std::max(stage_max[k],stage_times[k]);}stage_count++;
            if(!ready_owned){std::string tmp=ready+"."+std::to_string(getpid())+".tmp";std::ofstream f(tmp);check((bool)f,"Readiness create failed");f<<output<<'\n';f.close();check(!f.fail()&&rename(tmp.c_str(),ready.c_str())==0,"Readiness publish failed");ready_owned=true;fprintf(stderr,"YOLO11_PREVIEW_READY %s (real capture only)\n",output.c_str());}
            if(last-period>=1000000000ULL){uint64_t source=0,stamp=0,total=0;size_t count=0;{std::lock_guard<std::mutex>lock(pipeline.mutex);source=pipeline.latest_sequence;stamp=pipeline.latest_stamp;total=pipeline.completed;count=pipeline.latest.size();}auto raw=pipeline.sources.stats(),pub=pipeline.publisher.stats();fprintf(stderr,"VIDEO_DETECTION source_frame=%llu source_age_ms=%.1f boxes=%zu driver_sequence=%u driver_older_dropped=%llu driver_error_dropped=%llu raw_pending_limit=1 raw_replaced=%llu result_pending_limit=1 result_replaced=%llu result_old_rejected=%llu\n",(unsigned long long)source,stamp&&last>=stamp?(last-stamp)/1e6:-1.,count,capture.source_sequence,(unsigned long long)capture.older_dropped,(unsigned long long)capture.error_dropped,(unsigned long long)raw.replaced,(unsigned long long)pub.replaced,(unsigned long long)pub.rejected);fprintf(stderr,"PERF capture_fps=%.2f preview_fps=%.2f captured=%llu stereo_completed=%llu ai_skipped=%llu in_flight_limit=2\n",(sequence-last_sequence)*1e9/(last-period),(sequence-last_sequence)*1e9/(last-period),(unsigned long long)sequence,(unsigned long long)total,(unsigned long long)skipped);static const char*stage_names[]={"capture_wait_copy","raw_submit_copy","inference_submit_and_overlay","preview_write"};fprintf(stderr,"CAPTURE_TIMING selected_frames=%llu",(unsigned long long)stage_count);for(unsigned k=0;k<4;k++)fprintf(stderr," %s_avg_ms=%.3f %s_max_ms=%.3f",stage_names[k],stage_count?stage_sum[k]/(double)stage_count/1e6:0,stage_names[k],stage_max[k]/1e6);fprintf(stderr,"\n");auto raw_time=pipeline.sources.timing(),result_time=pipeline.publisher.timing();for(const auto&entry:{std::make_pair("raw",raw_time),std::make_pair("detections",result_time)}){const auto&t=entry.second;fprintf(stderr,"PUBLISH_TIMING kind=%s calls=%llu avg_ms=%.3f max_ms=%.3f source_frame=%llu source_us=%llu completed_source_age_ms=%.3f\n",entry.first,(unsigned long long)t.count,t.count?t.total_ns/(double)t.count/1e6:0,t.max_ns/1e6,(unsigned long long)t.last_sequence,(unsigned long long)(t.last_source_ns/1000),t.last_age_ns/1e6);}stage_sum.fill(0);stage_max.fill(0);stage_count=0;period=last;last_sequence=sequence;}

        }}catch(...){pipeline.shutdown();for(auto&t:workers)t.join();workers.clear();throw;}
        pipeline.shutdown();for(auto&t:workers)t.join();workers.clear();if(pipeline.failed.load())result=1;
    }catch(const std::exception&e){fprintf(stderr,"YOLO11: %s\n",e.what());result=1;}
    if(ready_owned)unlink(ready.c_str());return interrupted?128+interrupted:result;
}
