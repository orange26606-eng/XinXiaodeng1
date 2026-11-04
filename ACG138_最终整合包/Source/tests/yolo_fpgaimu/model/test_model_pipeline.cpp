#include <array>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <algorithm>
#include <atomic>
#include "yolo_math.h"
#include "model_profile.h"
#include "rknn_api.h"
#include "capture_queue.h"
#include "baseline_yolo_math.h"
static std::atomic<unsigned> assertions{0};
#define A(x) do{++assertions;if(!(x))throw std::runtime_error(#x);}while(0)
static uint64_t ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static void check(bool ok,const char*s){if(!ok)throw std::runtime_error(s);}
using namespace acg;
struct Api {
 unsigned classes=6,outputs=9,delay_ms=0;bool short_output=false,fail_release=false,bad_class=false;unsigned releases=0,destroys=0,runs=0;std::array<std::vector<float>,9>data;
 int init(rknn_context*c,void*,uint32_t,uint32_t,void*){*c=1;return 0;}int destroy(rknn_context){destroys++;return 0;}int core(rknn_context,rknn_core_mask){return 0;}
 int query(rknn_context,rknn_query_cmd cmd,void*p,uint32_t){
  if(cmd==RKNN_QUERY_SDK_VERSION){auto*v=(rknn_sdk_version*)p;std::strcpy(v->api_version,"INERT_TEST");std::strcpy(v->drv_version,"NO_DEVICE");return 0;}
  if(cmd==RKNN_QUERY_IN_OUT_NUM){*(rknn_input_output_num*)p={1,outputs};return 0;}
  auto*a=(rknn_tensor_attr*)p;unsigned i=a->index;*a={};a->index=i;a->fmt=RKNN_TENSOR_NCHW;a->type=RKNN_TENSOR_FLOAT16;
  if(cmd==RKNN_QUERY_INPUT_ATTR){a->n_dims=4;a->dims[0]=1;a->dims[1]=3;a->dims[2]=a->dims[3]=640;return 0;}
  if(outputs==1){a->n_dims=3;a->dims[0]=1;a->dims[1]=10;a->dims[2]=8400;a->n_elems=84000;data[0].assign(84000,0);return 0;}
  unsigned side=i/3==0?80:i/3==1?40:20,c=i%3==0?64:i%3==1?(bad_class?7:classes):1;a->n_dims=4;a->dims[0]=1;a->dims[1]=c;a->dims[2]=a->dims[3]=side;a->n_elems=c*side*side;a->size=a->n_elems*2;data[i].assign(a->n_elems,0);return 0;
 }
 int inputs(rknn_context,unsigned,const rknn_input*){return 0;}int run(rknn_context,void*){runs++;if(delay_ms)std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));return 0;}
 int get(rknn_context,unsigned n,rknn_output*out,void*){for(unsigned i=0;i<n;i++){out[i].buf=data[i].data();out[i].size=(uint32_t)(data[i].size()*sizeof(float));}if(short_output)out[0].size=4;return 0;}
 int release(rknn_context,unsigned,rknn_output*){releases++;return fail_release?-1:0;}
};
#include "rknn_context_worker.h"
struct yolo_fps_writer{};static void yolo_fps_writer_init(yolo_fps_writer*){}static void yolo_fps_writer_frame(yolo_fps_writer*,uint64_t,uint64_t){}
static const char*label(int){return "person";}
#include "frame_publish_workers.h"
static void env(const char*key,const char*value){
#ifdef _WIN32
 _putenv_s(key,value);
#else
 setenv(key,value,1);
#endif
}
static void test_math(){
 env("ACG_YOLO_PROFILE","custom6-new");env("ACG_OBJECT_CLASSES_FILE","");auto p=ModelProfile::load();A(p.labels.size()==6&&p.labels[2]=="cell_phone"&&p.labels[4]=="box");
 std::vector<uint8_t>nv(1920*1080*3/2),a(640*640*3),b(a.size());uint32_t v=9250;for(auto&x:nv){v=v*1664525+1013904223;x=(uint8_t)(v>>24);}InputPlan fast;acg_baseline::InputPlan reference;
 for(int eye=0;eye<2;eye++){fast.convert(nv.data(),eye,a.data());reference.convert(nv.data(),eye,b.data());for(size_t i=0;i<a.size();i++)A(a[i]==b[i]);}
 std::array<Tensor,9>heads;std::array<std::vector<float>,9>storage;for(unsigned i=0;i<9;i++){unsigned side=i/3==0?80:i/3==1?40:20,c=i%3==0?64:i%3==1?6:1;storage[i].assign(c*side*side,0);heads[i]={storage[i].data(),c,side,side,false};}
 unsigned cell=40*80+40;storage[1][4*80*80+cell]=.9f;storage[2][cell]=.9f;auto left=decode(heads,0,.1f,6),right=decode(heads,1,.1f,6);A(left.size()==1&&right.size()==1&&left[0].cls==4&&right[0].x1==left[0].x1+960);storage[2][cell]=0;A(decode(heads,0,.1f,6,false).size()==1);storage[2][cell]=.9f;
 bool rejected=false;try{decode(heads,0,.1f,80);}catch(...){rejected=true;}A(rejected);
 std::vector<float>decoded(84000,0);decoded[0]=320;decoded[8400]=320;decoded[16800]=100;decoded[25200]=100;decoded[(4+2)*8400]=.81f;auto boxes=decode_decoded(decoded.data(),decoded.size(),10,8400,false,0);A(boxes.size()==1&&boxes[0].cls==2&&boxes[0].score==.81f);
 // 不同类别重叠框有意保留。
 std::vector<Box>overlap={{1,1,30,30,.9f,0},{1,1,30,30,.8f,1},{1,1,30,30,.7f,0}};auto out=nms(overlap);A(out.size()==2&&out[0].cls==0&&out[1].cls==1);
}
static void test_context(){
 env("ACG_YOLO_PROFILE","custom6-new");Api api;std::vector<char>model(1024,0);std::vector<uint8_t>nv(1920*1080*3/2,128);
 {Context c(api,model,0);auto x=c.infer(nv.data(),0);A(x.empty()&&api.runs==1&&api.releases==1);api.short_output=true;bool error=false;try{c.infer(nv.data(),0);}catch(...){error=true;}A(error&&api.releases==2);api.short_output=false;api.fail_release=true;error=false;try{c.infer(nv.data(),1);}catch(...){error=true;}A(error&&api.releases==3);}
 A(api.destroys==1);Api bad;bad.bad_class=true;bool rejected=false;try{Context c(bad,model,1);}catch(...){rejected=true;}A(rejected&&bad.destroys==1);
 Api decoded;decoded.outputs=1;{Context c(decoded,model,2);A(c.infer(nv.data(),0).empty());}A(decoded.releases==1);
 env("ACG_YOLO_PROFILE","coco80");Api coco;coco.classes=80;{Context c(coco,model,0);A(c.infer(nv.data(),0).empty());}A(coco.releases==1);
 env("ACG_YOLO_PROFILE","custom6");Api original;original.outputs=1;{Context c(original,model,0);A(c.infer(nv.data(),0).empty());}A(original.releases==1);env("ACG_YOLO_PROFILE","custom6-new");
}
struct Observation{uint64_t seq,stamp;uint8_t pixel;};static std::mutex observed_mutex;static std::condition_variable observed_cv;static bool entered=false,unblock=false;static std::vector<Observation>raw_seen,vision_seen,detection_seen;
struct FakeRaw{explicit FakeRaw(uint64_t epoch){A(epoch==123);}bool enabled()const{return true;}void publish(const uint8_t*p,size_t n,uint64_t seq,uint64_t stamp){std::unique_lock<std::mutex>l(observed_mutex);A(n==2073600);raw_seen.push_back({seq,stamp,p[0]});if(seq==1){entered=true;observed_cv.notify_all();observed_cv.wait(l,[]{return unblock;});}}};
struct FakeVision{bool enabled()const{return true;}void publish(const uint8_t*p,size_t n,uint64_t seq,uint64_t stamp){std::lock_guard<std::mutex>l(observed_mutex);A(n==2073600);vision_seen.push_back({seq,stamp,p[0]});}};
struct FakeDetection{void publish(const uint8_t*p,size_t n,const std::vector<Box>&b,uint64_t seq,uint64_t stamp){std::unique_lock<std::mutex>l(observed_mutex);A(n==2073600&&b.size()==2&&b[0].x1==0&&b[1].x1==960);detection_seen.push_back({seq,stamp,p[0]});if(seq==1){entered=true;observed_cv.notify_all();observed_cv.wait(l,[]{return unblock;});}}};
static void wait_enter(){std::unique_lock<std::mutex>l(observed_mutex);A(observed_cv.wait_for(l,std::chrono::seconds(5),[]{return entered;}));}
static void let_go(){std::lock_guard<std::mutex>l(observed_mutex);unblock=true;observed_cv.notify_all();}
static void test_publishers(){
 std::vector<uint8_t>gray(2073600,1);
 {RawFramePublisher<FakeRaw,FakeVision>p(123);A(p.submit(gray.data(),gray.size(),1,100000));wait_enter();for(unsigned k=2;k<=40;k++){std::fill(gray.begin(),gray.end(),(uint8_t)k);A(p.submit(gray.data(),gray.size(),k,k*100000));}A(!p.submit(gray.data(),gray.size(),20,2000000));auto s=p.stats();A(s.submitted==40&&s.replaced==38&&s.rejected==1&&s.taken==1);let_go();}
 A(raw_seen.size()==2&&vision_seen.size()==2&&raw_seen[0].seq==1&&raw_seen[1].seq==40&&raw_seen[1].pixel==40&&raw_seen[1].stamp==4000000);for(unsigned i=0;i<2;i++)A(raw_seen[i].seq==vision_seen[i].seq&&raw_seen[i].stamp==vision_seen[i].stamp&&raw_seen[i].pixel==vision_seen[i].pixel);
 entered=unblock=false;FakeDetection e;
 {DetectionPublisher<FakeDetection>p(e);auto put=[&](unsigned k){auto j=std::make_unique<DetectionPublishJob>();j->sequence=k;j->stamp=k*100000;j->total=k;j->gray.assign(2073600,(uint8_t)k);j->boxes={{0,0,1,1,.5f,0},{960,0,961,1,.5f,0}};return p.submit(std::move(j));};A(put(1));wait_enter();A(put(5));A(!put(2));A(!put(4));A(put(9));A(!put(8));auto s=p.stats();A(s.submitted==3&&s.replaced==1&&s.rejected==3);let_go();}
 A(detection_seen.size()==2&&detection_seen[0].seq==1&&detection_seen[1].seq==9&&detection_seen[1].pixel==9&&detection_seen[1].stamp==900000);
}
static void test_capture(){
 size_t lengths[4]={100,100,100,100};std::vector<CaptureFrame>frames={{0,100,0xffffffffu,false},{1,0,0,true},{2,100,1,false},{3,100,2,false}};unsigned next=0;std::array<unsigned,4>requeued{};
 auto take=[&](CaptureFrame&f){if(next==frames.size())return CaptureIo::Again;f=frames[next++];return CaptureIo::Frame;};auto put=[&](unsigned i){requeued[i]++;return true;};auto result=drain_capture(4,lengths,100,take,put,[]{return false;});A(result.status==CaptureDrainStatus::Ready&&result.frame.index==3&&result.frame.sequence==2);A(result.stats.older_dropped==2&&result.stats.error_dropped==1&&result.stats.attempts==4);A(requeued[0]==1&&requeued[1]==1&&requeued[2]==1&&requeued[3]==0);A(put(3)&&requeued[3]==1);
 next=0;frames={{1,99,5,false}};requeued.fill(0);result=drain_capture(4,lengths,100,take,put,[]{return false;});A(result.status==CaptureDrainStatus::Failed&&result.reason==CaptureDrainReason::InvalidBytes&&requeued[1]==1);
}
static std::vector<Observation>pipeline_seen;
struct PipelineExporter{bool enabled()const{return true;}uint64_t session_epoch()const{return 123;}void publish(const uint8_t*p,size_t n,const std::vector<Box>&b,uint64_t seq,uint64_t stamp){std::lock_guard<std::mutex>l(observed_mutex);A(n==2073600&&b.size()==2);A((b[0].x1<960)!=(b[1].x1<960));A(p[0]==seq&&stamp==seq*100000);pipeline_seen.push_back({seq,stamp,p[0]});}};
struct PipelineRaw{explicit PipelineRaw(uint64_t){}bool enabled()const{return false;}bool publish(const uint8_t*,size_t,uint64_t,uint64_t){throw std::runtime_error("Unexpected disabled raw writer");}};
struct PipelineVision{bool enabled()const{return false;}void publish(const uint8_t*,size_t,uint64_t,uint64_t){throw std::runtime_error("Unexpected disabled AVF writer");}};
using ObjectFrameExport=PipelineExporter;using ObjectRawExport=PipelineRaw;using VisionFrameExport=PipelineVision;
#include "production_pipeline.inc"
static void test_actual_three_workers(){
 env("ACG_YOLO_PROFILE","custom6-new");std::array<Api,3>apis;apis[0].delay_ms=65;apis[1].delay_ms=55;apis[2].delay_ms=1;std::vector<char>model(1024,0);std::array<std::unique_ptr<Context>,3>contexts;
 for(unsigned k=0;k<3;k++){contexts[k]=std::make_unique<Context>(apis[k],model,k);unsigned cell=40*80+40;apis[k].data[1][cell]=.9f;apis[k].data[2][cell]=.9f;}
 {Pipeline pipeline;std::vector<std::thread>workers;for(auto&c:contexts)workers.emplace_back([&pipeline,c=c.get()]{pipeline.worker(*c);});
  std::vector<uint8_t>frame(1920*1080*3/2,128);std::fill(frame.begin(),frame.begin()+2073600,1);A(pipeline.submit(frame,1,100000));std::fill(frame.begin(),frame.begin()+2073600,2);A(pipeline.submit(frame,2,200000));A(!pipeline.submit(frame,3,300000));
  auto wait=[&](uint64_t target){uint64_t end=ns()+5000000000ULL;for(;;){{std::lock_guard<std::mutex>l(pipeline.mutex);if(pipeline.completed>=target)return;}A(ns()<end);std::this_thread::sleep_for(std::chrono::milliseconds(1));}};wait(2);
  {std::lock_guard<std::mutex>l(pipeline.mutex);A(pipeline.latest_sequence==2&&pipeline.latest_stamp==200000&&pipeline.latest.size()==2);}std::fill(frame.begin(),frame.begin()+2073600,3);A(pipeline.submit(frame,3,300000));wait(3);pipeline.shutdown();for(auto&t:workers)t.join();A(!pipeline.failed.load());A(pipeline.publisher.stats().rejected>=1);
 }
 A(pipeline_seen.size()==2&&pipeline_seen[0].seq==2&&pipeline_seen[1].seq==3);A(apis[0].runs+apis[1].runs+apis[2].runs==6);
 // 基准测试模式关闭图像导出。
 size_t before=pipeline_seen.size();{Pipeline pipeline(false);std::vector<std::thread>workers;for(auto&c:contexts)workers.emplace_back([&pipeline,c=c.get()]{pipeline.worker(*c);});std::vector<uint8_t>frame(1920*1080*3/2,128);A(pipeline.submit(frame,1,100000));uint64_t end=ns()+5000000000ULL;for(;;){{std::lock_guard<std::mutex>l(pipeline.mutex);if(pipeline.completed==1)break;}A(ns()<end);std::this_thread::sleep_for(std::chrono::milliseconds(1));}pipeline.shutdown();for(auto&t:workers)t.join();}A(pipeline_seen.size()==before);
}
int main(){try{test_math();test_context();test_publishers();test_capture();test_actual_three_workers();std::printf("PASS model pipeline %u assertions: true class/schema, RGB exact, single-release, bounded slow raw/result publishers, stale/outoforder rejection, paired source time, latest capture\n",assertions.load());return 0;}catch(const std::exception&e){std::fprintf(stderr,"FAIL %s\n",e.what());return 1;}}
