#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include "yolo_math.h"
#include "model_profile.h"
#include "attribute_label.h"
#include "object_frame_export.h"
#include "object_raw_export.h"
#include "latest_mailbox.h"
#include "core.hpp"
#include "json.hpp"
static unsigned assertions;
#define A(x) do{++assertions;if(!(x))throw std::runtime_error(#x);}while(0)
static uint64_t ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
using namespace acg;
struct yolo_fps_writer{};static void yolo_fps_writer_init(yolo_fps_writer*){}static void yolo_fps_writer_frame(yolo_fps_writer*,uint64_t,uint64_t){}
static const ModelProfile&active_profile(){static auto p=ModelProfile::load("custom6-new","",false);return p;}
static const char*label(int);
struct Context {struct Timing{uint64_t preprocess=0,input=0,run=0,get=0,decode=0,release=0;}timing;
 std::vector<acg::Box>infer(const uint8_t*p,int eye){if(p[0]==42&&eye==0)std::this_thread::sleep_for(std::chrono::milliseconds(30));else std::this_thread::sleep_for(std::chrono::milliseconds(1));return {{100.f+eye*960,100.f,400.f+eye*960,700.f,.8f,3}};}
};
#include "frame_publish_workers.h"
#include "production_pipeline.inc"
static void env(const char*k,const char*v){
#ifdef _WIN32
 _putenv_s(k,v);
#else
 setenv(k,v,1);
#endif
}
static void paint(std::vector<uint8_t>&p,int x0,int y0,int x1,int y1,int r,int g,int b){
 int yy=std::clamp(((66*r+129*g+25*b+128)>>8)+16,0,255),u=std::clamp(((-38*r-74*g+112*b+128)>>8)+128,0,255),v=std::clamp(((112*r-94*g-18*b+128)>>8)+128,0,255);
 for(int y=y0;y<y1;y++)for(int x=x0;x<x1;x++)p[y*1920+x]=(uint8_t)yy;
 for(int y=y0/2;y<(y1+1)/2;y++)for(int x=x0&~1;x<x1;x+=2){p[2073600+y*1920+x]=(uint8_t)u;p[2073600+y*1920+x+1]=(uint8_t)v;}
}
static std::string read(const std::string&path){std::ifstream f(path,std::ios::binary);return std::string(std::istreambuf_iterator<char>(f),{});}
static void write(const std::string&p,const std::string&s){std::ofstream f(p,std::ios::binary);f.write(s.data(),s.size());}
static void colors(){std::vector<uint8_t>p(3110400,128);acg::Box b{100,100,500,700,.8f,3};
 struct C{int r,g,b;unsigned color;}cases[]={{255,0,0,OBJ_COLOR_RED},{0,0,255,OBJ_COLOR_BLUE},{0,255,0,OBJ_COLOR_GREEN},{255,255,255,OBJ_COLOR_WHITE},{0,0,0,OBJ_COLOR_BLACK},{128,128,128,OBJ_COLOR_GRAY},{255,255,0,OBJ_COLOR_YELLOW}};
 for(auto c:cases){paint(p,0,0,1920,1080,c.r,c.g,c.b);extract_color(p.data(),p.size(),b);A(b.color_valid&&b.color==c.color&&b.color_samples==1024&&b.color_support>.95f);}
 paint(p,0,0,1920,1080,255,0,0);paint(p,100,100,500,700,0,0,255);extract_color(p.data(),p.size(),b);A(b.color==OBJ_COLOR_BLUE);A(attribute_caption("bottle",b)=="蓝色水瓶");
 auto original=p;draw_attribute_label(p,b,attribute_caption("bottle",b));A(p!=original);for(unsigned char c:attribute_caption("bottle",b))A(c!='%');
 unsigned count=0;const unsigned char*utf=(const unsigned char*)"红色水瓶";while(*utf){A(attribute_glyph(attribute_codepoint(utf)));++count;}A(count==4);
 b.x1=-1;extract_color(p.data(),p.size(),b);A(!b.color_valid);b.x1=100;b.x2=100;extract_color(p.data(),p.size(),b);A(!b.color_valid);
}
static void worker_contract(const std::string&out){env("ACG_OBJECT_FRAME",out.c_str());env("ACG_OBJECT_RAW_FRAME","");env("ACG_VISION_FRAME","");
 std::vector<uint8_t>one(3110400,128),two(3110400,128);paint(one,0,0,960,1080,255,0,0);paint(one,960,0,1920,1080,0,0,255);paint(two,0,0,960,1080,0,0,255);paint(two,960,0,1920,1080,255,0,0);one[0]=42;two[0]=43;
 {Pipeline pipeline(true);std::vector<std::thread>workers;std::array<Context,3>contexts;for(auto&c:contexts)workers.emplace_back([&pipeline,&c]{pipeline.worker(c);});A(pipeline.submit(one,1,1000000000));A(pipeline.submit(two,2,2000000000));one.assign(one.size(),0);two.assign(two.size(),0);
  uint64_t end=ns()+3000000000ULL;for(;;){uint64_t n;{std::lock_guard<std::mutex>g(pipeline.mutex);n=pipeline.completed;}if(n==2)break;A(ns()<end);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  {std::lock_guard<std::mutex>g(pipeline.mutex);A(pipeline.latest_sequence==2);A(pipeline.latest.size()==2);for(auto&b:pipeline.latest)A(b.color==(b.x1<960?OBJ_COLOR_BLUE:OBJ_COLOR_RED));}
  pipeline.shutdown();for(auto&t:workers)t.join();}
 auto frame=obj::read_frame(out);A(frame.id==2&&frame.time==2000000&&frame.boxes.size()==2);for(auto&b:frame.boxes){A(b.attributes.color_valid);A(std::string(b.attributes.color)==(b.eye==0?"blue":"red"));A(b.attributes.color_frame_id==2&&b.attributes.color_source_us==2000000&&b.attributes.color_eye==b.eye);}
 std::string binary=read(out);A((unsigned char)binary[3]==0x32);std::string old=binary.substr(0,64+2073600);old[3]=0x31;old[4]=1;for(unsigned k=0;k<2;k++)old+=binary.substr(64+2073600+k*64,32);write(out+".old",old);auto legacy=obj::read_frame(out+".old");A(legacy.boxes.size()==2&&!legacy.boxes[0].attributes.color_valid);
 binary[64+2073600+32]=(char)255;write(out+".badcolor",binary);auto degraded=obj::read_frame(out+".badcolor");A(degraded.boxes.size()==2&&!degraded.boxes[0].attributes.color_valid);
 auto config=out+".conf";write(config,"model_profile=custom6-new\n");auto cfg=obj::load_config(config);obj::Tracker tracker;std::vector<obj::Point>points(frame.boxes.size());tracker.update(frame,frame.boxes,points,cfg);A(tracker.tracks.size()==2);
 obj::Vision vision;for(int eye=0;eye<2;eye++){auto&m=vision.mono[eye];m.valid=true;m.eye=eye;m.fx=m.fy=800;m.cx=480;m.cy=540;}
 for(auto&t:tracker.tracks){auto&p=t.last_point;p.valid=true;p.source_frame=frame.id;p.source_time=frame.time;p.source_eye=p.geometry_eye=t.box.eye;p.source_box=t.box;p.z=t.box.eye==0?1.:.18;p.error=.01;}
 obj::Selection selection;auto state=obj::state_json(frame,tracker,cfg,true,"OK",selection,frame.time+1000,"",&vision);write(out+".state.json",state);
 auto fixture=obj::JP(state).parse();A(fixture.at("objects").values.size()==2);A(fixture.at("objects").values[0].at("attributes").at("size_valid").boolean);
 obj::Tracker color_tracking;auto red=frame.boxes[1];color_tracking.update(frame,{red},{obj::Point{}},cfg);color_tracking.selected=color_tracking.tracks[0].id;
 obj::Frame next=frame;next.id++;next.time+=100000;auto blue=red;blue.x1+=5;blue.x2+=5;std::snprintf(blue.attributes.color,sizeof blue.attributes.color,"blue");
 red.attributes.color_frame_id=blue.attributes.color_frame_id=next.id;red.attributes.color_source_us=blue.attributes.color_source_us=next.time;
 color_tracking.update(next,{blue,red},{obj::Point{},obj::Point{}},cfg);bool kept_red=false;for(const auto&t:color_tracking.tracks)if(t.id==color_tracking.selected)kept_red=t.observed&&std::string(t.attributes.color)=="red";A(kept_red);
 next.id++;next.time+=100000;color_tracking.update(next,{blue},{obj::Point{}},cfg);bool no_hard_veto=false;for(const auto&t:color_tracking.tracks)if(t.id==color_tracking.selected)no_hard_veto=t.observed;A(no_hard_veto);
 obj::Frame raw=frame;raw.id++;raw.time+=100000;raw.detection_result=false;raw.boxes.clear();tracker.update(raw,{}, {},cfg);for(auto&t:tracker.tracks)A(t.attributes.color_source_us==2000000);
}
static obj::J attribute_json(obj::Frame&f,obj::Track&t,obj::Vision&v,obj::Config&cfg){obj::Tracker tracker;tracker.tracks={t};obj::Selection selection;auto j=obj::JP(obj::state_json(f,tracker,cfg,true,"OK",selection,f.time+1000,"",&v)).parse();return j.at("objects").values.at(0).at("attributes");}
static void sizes(){obj::Config cfg;obj::Vision vision;vision.mono[0].valid=true;vision.mono[0].eye=0;vision.mono[0].fx=vision.mono[0].fy=800;vision.mono[0].cx=480;vision.mono[0].cy=540;obj::Frame f;f.epoch=1;f.id=10;f.time=1000000;f.gray.assign(2073600,128);
 obj::Track t;t.id=1;t.cls=4;t.observed=true;t.observed_at=f.time;t.last_frame=f.id;t.box={4,0,300,200,500,600,.9f};t.last_point.valid=true;t.last_point.source_frame=8;t.last_point.source_time=900000;t.last_point.source_eye=t.last_point.geometry_eye=0;t.last_point.source_box=t.box;t.last_point.z=1;t.last_point.error=.02;
 auto a=attribute_json(f,t,vision,cfg);A(a.at("size_valid").boolean&&a.at("size").str()=="large");A(std::abs(a.at("width_m").num()-.25)<.005);A(std::abs(a.at("height_m").num()-.5)<.005);A(a.at("size_source_us").str()=="900000");
 t.last_point.source_box={4,0,200,100,600,900,.9f};t.last_point.z=.25;auto b=attribute_json(f,t,vision,cfg);A(b.at("size").str()=="small");A(b.at("width_m").num()<a.at("width_m").num());
 t.last_point.valid=false;auto missing=attribute_json(f,t,vision,cfg);A(!missing.at("size_valid").boolean&&missing.at("size").str().empty());
 t.last_point.valid=true;t.last_point.source_box.x1=0;auto clipped=attribute_json(f,t,vision,cfg);A(!clipped.at("size_valid").boolean&&clipped.at("size_source").str()=="CLIPPED_VISIBLE_EXTENT");
 obj::Tracker full;for(int k=0;k<65;k++){auto q=t;q.id=k+1;full.tracks.push_back(q);}obj::Selection selection;auto all=obj::state_json(f,full,cfg,true,"OK",selection,f.time+1000,"",&vision);auto j=obj::JP(all).parse();A(j.at("objects").values.size()==65);printf("65_target_json_bytes=%zu\n",all.size());
}
int main(int argc,char**argv){try{A(argc==2);colors();worker_contract(argv[1]);sizes();printf("PASS attributes actual pipeline worker/publisher + OBF1/2 reader + tracker/source/provenance + NV12 colors + UTF8 render + metric sizes, assertions=%u\n",assertions);return 0;}catch(const std::exception&e){fprintf(stderr,"FAIL %s\n",e.what());return 1;}}
