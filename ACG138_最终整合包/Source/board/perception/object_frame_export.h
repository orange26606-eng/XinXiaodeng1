#ifndef ACG_OBJECT_FRAME_EXPORT_H
#define ACG_OBJECT_FRAME_EXPORT_H
#include "vision_frame_export.h"
#include "yolo_math.h"
#include <chrono>
#include <cmath>
#include <vector>
// OBF1保存同一源帧的原始Y平面与检测结果，发布后保持不变。
// 调用方须在remaining归零前、仍持有NPU槽时复制原始Y平面。
class ObjectFrameExport {
 std::string path,tmp;uint64_t epoch,last_stamp=0,last_sequence=0,last_warning=0;
 static void putf(unsigned char*p,float v){uint32_t n;std::memcpy(&n,&v,4);avf_put32(p,n);}
public:
 ObjectFrameExport(){const char*p=std::getenv("ACG_OBJECT_FRAME");if(p&&*p){path=p;tmp=path+".tmp";}epoch=(uint64_t)std::chrono::steady_clock::now().time_since_epoch().count();if(!epoch)epoch=1;}
 uint64_t session_epoch()const{return epoch;}
 bool enabled()const{return !path.empty();}
 void publish(const uint8_t*gray,size_t length,const std::vector<acg::Box>&boxes,uint64_t seq,uint64_t stamp_ns){
  if(path.empty()||length!=1920u*1080u||!seq||!stamp_ns||seq<=last_sequence||(last_stamp&&stamp_ns<=last_stamp))return;
  std::vector<unsigned char>records;records.reserve(64*64);unsigned per_eye[2]{};
  for(const auto&b:boxes){if(records.size()>=64*64)break;
   if(b.cls<0||b.cls>=80||!std::isfinite(b.score)||b.score<0||b.score>1||!std::isfinite(b.x1)||!std::isfinite(b.y1)||!std::isfinite(b.x2)||!std::isfinite(b.y2))continue;
   int eye=(b.x1+b.x2)*.5f>=960?1:0;float x1=b.x1-eye*960,x2=b.x2-eye*960;
   if(per_eye[eye]>=32)continue; // 单眼目标过多时也为另一眼保留结果容量。
   if(x1<0||x2>960||x2<=x1||b.y1<0||b.y2>1080||b.y2<=b.y1)continue;
   unsigned char record[64]{};avf_put32(record,(uint32_t)b.cls);avf_put32(record+4,(uint32_t)eye);
   putf(record+8,x1);putf(record+12,b.y1);putf(record+16,x2);putf(record+20,b.y2);putf(record+24,b.score);
   avf_put32(record+32,b.color);avf_put32(record+36,b.secondary_color);putf(record+40,b.color_support);avf_put32(record+44,b.color_valid?1:0);avf_put32(record+48,b.color_samples);
   records.insert(records.end(),record,record+64);++per_eye[eye];
  }
  unsigned char h[64]{};avf_put32(h,0x3246424f);avf_put32(h+4,2);avf_put32(h+8,64);avf_put32(h+12,1920);avf_put32(h+16,1080);avf_put32(h+20,1920);avf_put32(h+24,1920*1080);avf_put32(h+28,(uint32_t)(records.size()/64));avf_put64(h+32,epoch);avf_put64(h+40,seq);avf_put64(h+48,stamp_ns/1000);
  FILE*f=nullptr;
#ifdef _WIN32
  f=std::fopen(tmp.c_str(),"wb");
#else
  int fd=::open(tmp.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC|O_NOFOLLOW,0600);if(fd>=0){f=::fdopen(fd,"wb");if(!f)::close(fd);}
#endif
  bool ok=false;if(f){ok=std::fwrite(h,1,64,f)==64&&std::fwrite(gray,1,length,f)==length&&(records.empty()||std::fwrite(records.data(),1,records.size(),f)==records.size());if(std::fclose(f)!=0)ok=false;
#ifdef _WIN32
   // 仅主机测试使用此分支，板端使用下方POSIX原子重命名。
   if(ok){std::remove(path.c_str());ok=std::rename(tmp.c_str(),path.c_str())==0;}
#else
   if(ok)ok=std::rename(tmp.c_str(),path.c_str())==0;
#endif
  }
  if(ok){last_stamp=stamp_ns;last_sequence=seq;}
  else{std::remove(tmp.c_str());if(!last_warning||stamp_ns-last_warning>5000000000ULL){std::fprintf(stderr,"OBJECT_FRAME_EXPORT_FAILED path=%s (existing video continues)\n",path.c_str());last_warning=stamp_ns;}}
 }
};
#endif
