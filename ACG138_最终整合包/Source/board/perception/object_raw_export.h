#pragma once
#include "vision_frame_export.h"
// ORF2由唯一采集者导出推理和画框之前的最新原始灰度。
// 64字节小端头：magic为0x3246524f，版本2、头长64、宽1920、高1080、步长1920，
// 载荷2073600字节、count为0，随后为epoch、源序号、源单调时刻及保留0。
class ObjectRawExport {
 std::string path,tmp;uint64_t epoch,last=0,last_warning=0;
public:
 explicit ObjectRawExport(uint64_t e):epoch(e){const char*p=std::getenv("ACG_OBJECT_RAW_FRAME");if(p&&*p){path=p;tmp=path+".tmp";}}
 bool enabled()const{return !path.empty();}
 bool publish(const uint8_t*gray,size_t length,uint64_t seq,uint64_t stamp_ns){
  if(path.empty()||length<2073600||!seq||seq<=last||!stamp_ns)return false;
  unsigned char h[64]{};avf_put32(h,0x3246524f);avf_put32(h+4,2);avf_put32(h+8,64);avf_put32(h+12,1920);avf_put32(h+16,1080);avf_put32(h+20,1920);avf_put32(h+24,2073600);avf_put64(h+32,epoch);avf_put64(h+40,seq);avf_put64(h+48,stamp_ns/1000);
  FILE*f=nullptr;
#ifdef _WIN32
  f=std::fopen(tmp.c_str(),"wb");
#else
  int fd=::open(tmp.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC|O_NOFOLLOW,0600);if(fd>=0){f=::fdopen(fd,"wb");if(!f)::close(fd);}
#endif
  bool ok=false;if(f){ok=std::fwrite(h,1,64,f)==64&&std::fwrite(gray,1,2073600,f)==2073600;if(std::fclose(f))ok=false;
#ifdef _WIN32
   if(ok){std::remove(path.c_str());ok=std::rename(tmp.c_str(),path.c_str())==0;}
#else
   if(ok)ok=std::rename(tmp.c_str(),path.c_str())==0;
#endif
  }
  if(ok)last=seq;else{std::remove(tmp.c_str());if(!last_warning||stamp_ns-last_warning>5000000000ULL){std::fprintf(stderr,"OBJECT_RAW_EXPORT_FAILED path=%s source_sequence=%llu\n",path.c_str(),(unsigned long long)seq);last_warning=stamp_ns;}}return ok;
 }
};
