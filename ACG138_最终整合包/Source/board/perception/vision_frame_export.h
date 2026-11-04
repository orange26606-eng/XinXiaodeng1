#ifndef ACG_VISION_FRAME_EXPORT_H
#define ACG_VISION_FRAME_EXPORT_H
// 仅发布画框之前的最新灰度快照，时间戳为RK出队时刻，
// 不代表传感器曝光时刻。仅保留一个已发布文件和一个临时文件，不排队。
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif
inline void avf_put32(unsigned char*p,uint32_t n){for(unsigned i=0;i<4;i++)p[i]=(unsigned char)(n>>(8*i));}
inline void avf_put64(unsigned char*p,uint64_t n){for(unsigned i=0;i<8;i++)p[i]=(unsigned char)(n>>(8*i));}
inline void avf_header(unsigned char h[64],uint64_t seq,uint64_t us){
 std::memset(h,0,64);avf_put32(h,0x31465641);avf_put32(h+4,1);avf_put32(h+8,64);
 avf_put32(h+12,1920);avf_put32(h+16,1080);avf_put32(h+20,1920);avf_put32(h+24,1920*1080);
 avf_put32(h+28,0); // 物理左右镜头顺序需现场核对。
 avf_put64(h+32,seq);avf_put64(h+40,us);
}
class VisionFrameExport {
 std::string path,tmp;uint64_t last=0,last_warning=0;
public:
 VisionFrameExport(){const char*p=std::getenv("ACG_VISION_FRAME");if(p&&*p){path=p;tmp=path+".tmp";}}
 bool enabled()const{return !path.empty();}
 void publish(const uint8_t*gray,size_t length,uint64_t seq,uint64_t stamp_ns){
  if(path.empty()||length<1920u*1080u||(last&&stamp_ns-last<200000000ULL))return;
  last=stamp_ns;unsigned char h[64];avf_header(h,seq,stamp_ns/1000);
  FILE*f=nullptr;
#ifdef _WIN32
  f=std::fopen(tmp.c_str(),"wb");
#else
  int fd=::open(tmp.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC|O_NOFOLLOW,0600);
  if(fd>=0){f=::fdopen(fd,"wb");if(!f)::close(fd);}
#endif
  bool ok=false;
  if(f){ok=std::fwrite(h,1,64,f)==64&&std::fwrite(gray,1,1920*1080,f)==1920u*1080u;
   if(std::fclose(f)!=0)ok=false;
   if(ok)ok=std::rename(tmp.c_str(),path.c_str())==0;
  }
  if(!ok){std::remove(tmp.c_str());if(!last_warning||stamp_ns-last_warning>5000000000ULL){
   std::fprintf(stderr,"VISION_FRAME_EXPORT_FAILED path=%s (camera continues)\n",path.c_str());last_warning=stamp_ns;}}
 }
};
#endif
