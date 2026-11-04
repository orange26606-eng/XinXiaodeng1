#include "core.hpp"
#include "../board/perception/model_profile.h"

#include "json.hpp"

#include <algorithm>

#include <chrono>

#include <cmath>

#include <cstdio>

#include <cstring>

#include <filesystem>

#include <fstream>

#include <iomanip>

#include <limits>

#include <set>

#include <sstream>

#include <stdexcept>

#include <thread>

#ifdef _WIN32

#include <windows.h>

#else

#include <fcntl.h>

#include <sys/stat.h>

#include <unistd.h>

#endif

#define STB_IMAGE_WRITE_IMPLEMENTATION

#define STBI_WRITE_NO_STDIO

#include "../board/vendor/stb_image_write.h"

// 使用官方ByteTrack LAPJV固定版本，MIT许可保存在third_party。
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wunused-parameter"
namespace objlapjv {
#include "third_party/bytetrack/lapjv.cpp"
}
#pragma clang diagnostic pop
#undef NEW
#undef FREE
#undef SWAP_INDICES
#undef PRINTF
#undef PRINT_COST_ARRAY
#undef PRINT_INDEX_ARRAY
#undef ASSERT
#undef LARGE
#undef TRUE
#undef FALSE

namespace obj {
static acg::ModelProfile active_model=acg::ModelProfile::load("coco80","",false);
const char*model_profile_name(){return active_model.name.c_str();}
size_t model_class_count(){return active_model.labels.size();}

uint64_t now_us(){return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}

static std::string read(const std::string&p,size_t limit){

#ifndef _WIN32

 int fd=open(p.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);if(fd<0)throw std::runtime_error("FILE_UNAVAILABLE");struct stat st;if(fstat(fd,&st)||!S_ISREG(st.st_mode)||st.st_size<0||(uint64_t)st.st_size>limit){close(fd);throw std::runtime_error("FILE_SIZE_INVALID");}std::string b((size_t)st.st_size,'\0');size_t n=0;while(n<b.size()){ssize_t k=::read(fd,b.data()+n,b.size()-n);if(k<=0)break;n+=(size_t)k;}close(fd);if(n!=b.size())throw std::runtime_error("FILE_TRUNCATED");return b;

#else

 std::ifstream f(p,std::ios::binary|std::ios::ate);if(!f)throw std::runtime_error("FILE_UNAVAILABLE");auto n=f.tellg();if(n<0||(uint64_t)n>limit)throw std::runtime_error("FILE_SIZE_INVALID");std::string b((size_t)n,'\0');f.seekg(0);if(!f.read(b.data(),n))throw std::runtime_error("FILE_TRUNCATED");return b;

#endif

}

bool atomic_file(const std::string&p,const std::string&b){const std::string t=p+".tmp";

#ifndef _WIN32

 int fd=open(t.c_str(),O_WRONLY|O_CREAT|O_TRUNC|O_CLOEXEC|O_NOFOLLOW,0600);if(fd<0)return false;size_t n=0;while(n<b.size()){ssize_t k=::write(fd,b.data()+n,b.size()-n);if(k<=0)break;n+=(size_t)k;}int rc=close(fd);if(n!=b.size()||rc){unlink(t.c_str());return false;}if(rename(t.c_str(),p.c_str())){unlink(t.c_str());return false;}return true;

#else

 std::ofstream f(t,std::ios::binary);if(!f.write(b.data(),b.size()))return false;f.close();
 // Windows测试或显示读取者可能短暂禁止共享删除，
 // 有限次数重试并保留原子替换，Linux的rename流程不变。
 for(int attempt=0;attempt<20;attempt++){if(MoveFileExA(t.c_str(),p.c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH))return true;DWORD error=GetLastError();if(error!=ERROR_ACCESS_DENIED&&error!=ERROR_SHARING_VIOLATION)return false;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return false;

#endif

}

static uint32_t u32(const unsigned char*p){return(uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}

static uint64_t u64(const unsigned char*p){return(uint64_t)u32(p)|(uint64_t)u32(p+4)<<32;}

static float f32(const unsigned char*p){uint32_t x=u32(p);float f;std::memcpy(&f,&x,4);return f;}

static double f64(const unsigned char*p){uint64_t x=u64(p);double f;std::memcpy(&f,&x,8);return f;}

static std::string trim(std::string s){size_t a=s.find_first_not_of(" \r\n\t"),b=s.find_last_not_of(" \r\n\t");return a==std::string::npos?"":s.substr(a,b-a+1);}

Config load_config(const std::string&path){Config c;std::ifstream f(path);if(!f)throw std::runtime_error("CONFIG_UNAVAILABLE");std::string line;std::set<std::string>seen;while(std::getline(f,line)){line=trim(line.substr(0,line.find('#')));if(line.empty())continue;size_t p=line.find('=');if(p==std::string::npos)throw std::runtime_error("CONFIG_SYNTAX");std::string k=trim(line.substr(0,p)),v=trim(line.substr(p+1));if(!seen.insert(k).second)throw std::runtime_error("CONFIG_DUPLICATE_KEY");if(k=="model_profile"||k=="classes_file"){if(k=="model_profile")c.model_profile=v;else c.classes_file=v;continue;}if(k=="calibration_file"||k=="mono_eye0_file"||k=="mono_eye1_file"||k=="stereo_extrinsics_file"){if(k=="calibration_file")c.calibration=v;else if(k=="mono_eye0_file")c.mono_files[0]=v;else if(k=="mono_eye1_file")c.mono_files[1]=v;else c.extrinsics=v;continue;}const std::set<std::string>keys={"size_box_small_m","size_box_large_m","size_bottle_small_m","size_bottle_large_m","size_cup_small_m","size_cup_large_m","image_tracking_enabled","pixel_bridge_ms","depth_max_age_ms","snapshot_jpeg_quality","snapshot_period_ms","stereo_enabled","camera_to_body_verified","reference_eye","frame_max_age_ms","confirm_frames","track_lost_ms","track_expire_ms","max_tracks","min_depth_m","max_depth_m","disparity_count","scale","camera_x_m","camera_y_m","camera_z_m","camera_roll_deg","camera_pitch_deg","camera_yaw_deg","camera_delay_ms","anchor_area_ratio_min","anchor_area_ratio_max","anchor_aspect_tol","anchor_position_claim_m","identity_reject_frames"};if(!keys.count(k))throw std::runtime_error("CONFIG_UNKNOWN_KEY:"+k);char*e;double d=std::strtod(v.c_str(),&e);if(e==v.c_str()||*e||!std::isfinite(d))throw std::runtime_error("CONFIG_INVALID_NUMBER");if(k=="stereo_enabled"||k=="camera_to_body_verified"||k=="reference_eye"){if(d!=0&&d!=1)throw std::runtime_error("CONFIG_BOOLEAN");if(k=="stereo_enabled")c.stereo=d==1;else if(k=="camera_to_body_verified")c.mount=d==1;else c.eye=(int)d;}

 else if(k=="size_box_small_m")c.size_box_small_m=d;else if(k=="size_box_large_m")c.size_box_large_m=d;else if(k=="size_bottle_small_m")c.size_bottle_small_m=d;else if(k=="size_bottle_large_m")c.size_bottle_large_m=d;else if(k=="size_cup_small_m")c.size_cup_small_m=d;else if(k=="size_cup_large_m")c.size_cup_large_m=d;
 else if(k=="pixel_bridge_ms")c.pixel_bridge_ms=d;else if(k=="depth_max_age_ms")c.depth_max_age_ms=d;else if(k=="snapshot_jpeg_quality")c.snapshot_jpeg_quality=(int)d;else if(k=="snapshot_period_ms")c.snapshot_period_ms=(int)d;else if(k=="image_tracking_enabled"){if(d!=0&&d!=1)throw std::runtime_error("CONFIG_BOOLEAN");c.image_tracking=d==1;}
 else if(k=="frame_max_age_ms")c.max_age_ms=d;else if(k=="confirm_frames")c.confirm=(int)d;else if(k=="track_lost_ms")c.lost_ms=(int)d;else if(k=="track_expire_ms")c.expire_ms=(int)d;else if(k=="max_tracks")c.max_tracks=(int)d;else if(k=="min_depth_m")c.min_depth=d;else if(k=="max_depth_m")c.max_depth=d;else if(k=="disparity_count")c.disparities=(int)d;else if(k=="scale")c.scale=d;

 else if(k=="anchor_area_ratio_min")c.anchor.area_min=d;else if(k=="anchor_area_ratio_max")c.anchor.area_max=d;else if(k=="anchor_aspect_tol")c.anchor.aspect_tol=d;else if(k=="anchor_position_claim_m")c.anchor.position_claim_m=d;else if(k=="identity_reject_frames"){if(d!=std::floor(d))throw std::runtime_error("CONFIG_INTEGER");if(d<1||d>1000)throw std::runtime_error("CONFIG_ANCHOR_OUT_OF_RANGE");c.identity_reject_frames=(int)d;}

 if(k=="confirm_frames"||k=="track_lost_ms"||k=="track_expire_ms"||k=="max_tracks"||k=="disparity_count")if(d!=std::floor(d))throw std::runtime_error("CONFIG_INTEGER");}

 if(c.size_box_small_m<=0||c.size_box_large_m<=c.size_box_small_m||c.size_bottle_small_m<=0||c.size_bottle_large_m<=c.size_bottle_small_m||c.size_cup_small_m<=0||c.size_cup_large_m<=c.size_cup_small_m)throw std::runtime_error("CONFIG_SIZE_THRESHOLDS_INVALID");
 if(c.anchor.area_min<=0||c.anchor.area_min>1||c.anchor.area_max<1||c.anchor.area_max>10||c.anchor.aspect_tol<0||c.anchor.aspect_tol>2||c.anchor.position_claim_m<=0||c.anchor.position_claim_m>5||c.identity_reject_frames<1||c.identity_reject_frames>1000)throw std::runtime_error("CONFIG_ANCHOR_OUT_OF_RANGE");

 if(c.snapshot_period_ms<100||c.snapshot_period_ms>1000||c.pixel_bridge_ms<200||c.pixel_bridge_ms>5000||c.depth_max_age_ms<50||c.depth_max_age_ms>2000||c.snapshot_jpeg_quality<40||c.snapshot_jpeg_quality>95||c.max_age_ms<50||c.max_age_ms>2000||c.confirm<1||c.confirm>10||c.lost_ms<200||c.lost_ms>5000||c.expire_ms<c.lost_ms||c.expire_ms>10000||c.max_tracks<1||c.max_tracks>MAX_TRACKS||c.disparities<16||c.disparities>192||c.disparities%16||c.scale<.25||c.scale>.5||c.min_depth<=0||c.max_depth<=c.min_depth||c.max_depth>20)throw std::runtime_error("CONFIG_OUT_OF_RANGE");if(!c.calibration.empty()&&std::filesystem::path(c.calibration).is_relative())c.calibration=(std::filesystem::path(path).parent_path()/c.calibration).string();for(auto*file:{&c.mono_files[0],&c.mono_files[1],&c.extrinsics})if(!file->empty()&&std::filesystem::path(*file).is_relative())*file=(std::filesystem::path(path).parent_path()/ *file).string();if(!c.classes_file.empty()&&std::filesystem::path(c.classes_file).is_relative())c.classes_file=(std::filesystem::path(path).parent_path()/c.classes_file).string();active_model=acg::ModelProfile::load(c.model_profile,c.classes_file);return c;}

static Frame read_any_frame(const std::string&path,bool raw){
 std::string b=read(path,64+2073600+64*64);if(b.size()<64)throw std::runtime_error("FRAME_TRUNCATED");auto*p=(const unsigned char*)b.data();
 bool attributes=!raw&&u32(p)==0x3246424fu&&u32(p+4)==2u;
 if((!attributes&&(u32(p)!=(raw?0x3246524fu:0x3146424fu)||u32(p+4)!=(raw?2u:1u)))||u32(p+8)!=64||u32(p+12)!=1920||u32(p+16)!=1080||u32(p+20)!=1920||u32(p+24)!=2073600||u32(p+28)>64||u64(p+56))throw std::runtime_error("FRAME_LAYOUT_INVALID");
 unsigned n=u32(p+28),stride=attributes?64:32;if(raw&&n)throw std::runtime_error("RAW_FRAME_DETECTIONS_FORBIDDEN");if(b.size()!=64+2073600+n*stride)throw std::runtime_error("FRAME_SIZE_INVALID");
 Frame a;a.detection_result=!raw;a.epoch=u64(p+32);a.id=u64(p+40);a.time=u64(p+48);if(!a.epoch||!a.id||!a.time)throw std::runtime_error("FRAME_IDENTITY_INVALID");a.gray.assign(p+64,p+64+2073600);
 for(unsigned i=0;i<n;i++){auto*q=p+64+2073600+i*stride;Box x;x.cls=(int)u32(q);x.eye=(int)u32(q+4);x.x1=f32(q+8);x.y1=f32(q+12);x.x2=f32(q+16);x.y2=f32(q+20);x.score=f32(q+24);
 if(u32(q+28)||x.cls<0||x.cls>79||x.eye<0||x.eye>1||!std::isfinite(x.x1)||!std::isfinite(x.y1)||!std::isfinite(x.x2)||!std::isfinite(x.y2)||!std::isfinite(x.score)||x.x1<0||x.y1<0||x.x2>960||x.y2>1080||x.x2<=x.x1||x.y2<=x.y1||x.score<0||x.score>1)throw std::runtime_error("FRAME_BOX_INVALID");
 if(attributes){unsigned primary=u32(q+32),secondary=u32(q+36),flags=u32(q+44);float support=f32(q+40);auto&at=x.attributes;
  if(primary>0&&primary<14&&secondary<14&&(flags&1)&&!(flags&~1u)&&std::isfinite(support)&&support>=0&&support<=1){at.color_valid=1;std::snprintf(at.color,sizeof at.color,"%s",obj_color_name(primary));if(secondary)std::snprintf(at.secondary_color,sizeof at.secondary_color,"%s",obj_color_name(secondary));at.color_support=support;at.color_eye=x.eye;at.color_frame_id=a.id;at.color_source_us=a.time;}
 }
 a.boxes.push_back(x);}return a;
}

Frame read_frame(const std::string&path){return read_any_frame(path,false);}
Frame read_raw_frame(const std::string&path){return read_any_frame(path,true);}

void Calibration::load(const std::string&path){std::string b=read(path,12*1024*1024);if(b.size()<48)throw std::runtime_error("CALIBRATION_TRUNCATED");auto*p=(const unsigned char*)b.data();if(u32(p)!=0x3144534f||u32(p+4)!=1||u32(p+8)!=48||u32(p+12)!=960||u32(p+16)!=1080)throw std::runtime_error("CALIBRATION_LAYOUT");w=(int)u32(p+20);h=(int)u32(p+24);left_eye=(int)u32(p+28);baseline=f64(p+32);if(w<240||w>480||h<270||h>540||left_eye<0||left_eye>1||!std::isfinite(baseline)||baseline<=.005||baseline>=1||f64(p+40)!=0)throw std::runtime_error("CALIBRATION_GEOMETRY");size_t n=(size_t)w*h,expected=48+(16+9+9+5)*8+8*4+4*n*4;if(b.size()!=expected)throw std::runtime_error("CALIBRATION_SIZE");size_t off=48;auto doubles=[&](auto&a){for(double&v:a){v=f64(p+off);off+=8;if(!std::isfinite(v))throw std::runtime_error("CALIBRATION_NONFINITE");}};doubles(Q);doubles(R);doubles(K);doubles(D);if(K[0]<=0||K[4]<=0)throw std::runtime_error("CALIBRATION_FOCAL");for(int i=0;i<3;i++)for(int j=0;j<3;j++){double dot=0;for(int k=0;k<3;k++)dot+=R[i*3+k]*R[j*3+k];if(std::abs(dot-(i==j))>.01)throw std::runtime_error("CALIBRATION_ROTATION");}for(auto*a:{&roi1,&roi2})for(int&v:*a){v=(int)u32(p+off);off+=4;}for(auto&m:maps){m.resize(n);for(float&v:m){v=f32(p+off);off+=4;if(!std::isfinite(v)||std::abs(v)>100000)throw std::runtime_error("CALIBRATION_MAP");}}}

static uint8_t sample(const Frame&f,int eye,float x,float y){if(x<0||y<0||x>=959||y>=1079)return 0;int xx=(int)x,yy=(int)y;float a=x-xx,b=y-yy;const uint8_t*p=f.gray.data()+yy*1920+eye*960+xx;return(uint8_t)std::clamp((1-b)*((1-a)*p[0]+a*p[1])+b*((1-a)*p[1920]+a*p[1921]),0.f,255.f);}

static std::vector<uint32_t>census(const std::vector<uint8_t>&a,int w,int h){std::vector<uint32_t>c((size_t)w*h);for(int y=2;y<h-2;y++)for(int x=2;x<w-2;x++){uint32_t n=0;for(int v=-2;v<=2;v++)for(int u=-2;u<=2;u++)if(u||v)n=(n<<1)|(a[(y+v)*w+x+u]<a[y*w+x]);c[y*w+x]=n;}return c;}

static std::vector<float>sgm(const std::vector<uint8_t>&a,const std::vector<uint8_t>&b,int w,int h,int D,bool reverse){const int P1=3,P2=20;size_t N=(size_t)w*h;auto ca=census(a,w,h),cb=census(b,w,h);std::vector<uint8_t>costs(N*D,60);for(int y=0;y<h;y++)for(int x=0;x<w;x++){size_t i=(size_t)y*w+x;for(int d=0;d<D;d++){int xr=reverse?x+d:x-d;if(xr>=0&&xr<w)costs[i*D+d]=(uint8_t)(__builtin_popcount(ca[i]^cb[(size_t)y*w+xr])+std::abs((int)a[i]-(int)b[(size_t)y*w+xr])/16);}}std::vector<uint16_t>sum(N*D,0);std::vector<int>prev(D),cur(D);auto path=[&](int x,int y,int dx,int dy){std::fill(prev.begin(),prev.end(),0);while(x>=0&&x<w&&y>=0&&y<h){int mn=*std::min_element(prev.begin(),prev.end());size_t i=(size_t)y*w+x;for(int d=0;d<D;d++){int cost=costs[i*D+d];int v=std::min(prev[d],mn+P2);if(d)v=std::min(v,prev[d-1]+P1);if(d+1<D)v=std::min(v,prev[d+1]+P1);cur[d]=cost+v-mn;sum[i*D+d]+=(uint16_t)cur[d];}prev.swap(cur);x+=dx;y+=dy;}};for(int y=0;y<h;y++){path(0,y,1,0);path(w-1,y,-1,0);}for(int x=0;x<w;x++){path(x,0,0,1);path(x,h-1,0,-1);}std::vector<float>out(N,-1);for(int y=3;y<h-3;y++)for(int x=3;x<w-3;x++){size_t i=(size_t)y*w+x;int lo=255,hi=0;for(int v=-2;v<=2;v++)for(int u=-2;u<=2;u++){int q=a[(y+v)*w+x+u];lo=std::min(lo,q);hi=std::max(hi,q);}if(hi-lo<12)continue;auto*p=sum.data()+i*D;int best=0;for(int d=1;d<D;d++)if(p[d]<p[best])best=d;if(best<1||best>=D-1)continue;int second=10000;for(int d=0;d<D;d++)if(std::abs(d-best)>1)second=std::min(second,(int)p[d]);if(second-p[best]<std::max(4,(int)p[best]/12))continue;double den=2.*(p[best-1]+p[best+1]-2.*p[best]);double delta=den>0?(p[best-1]-p[best+1])/den:0;out[i]=(float)(best+std::clamp(delta,-.5,.5));}return out;}

static Depth stereo_rows(const Frame&f,const Calibration&c,const Config&cfg,int y0,int y1){
 Depth d;d.w=c.w;d.h=c.h;d.row_begin=y0;d.row_end=y1;
 if(cfg.disparities>=c.w-16)throw std::runtime_error("DISPARITY_EXCEEDS_WIDTH");
 if(std::abs((double)c.w/960-cfg.scale)>.002||std::abs((double)c.h/1080-cfg.scale)>.002)throw std::runtime_error("CALIBRATION_SCALE_MISMATCH");
 if(f.gray.size()!=2073600||y0<0||y1>c.h||y1-y0<8)throw std::runtime_error("STEREO_ROI_INVALID");
 size_t n=(size_t)c.w*c.h,work=(size_t)c.w*(y1-y0);std::vector<uint8_t>a(work),b(work);
 for(int y=y0;y<y1;y++)for(int x=0;x<c.w;x++){size_t i=(size_t)y*c.w+x,k=(size_t)(y-y0)*c.w+x;a[k]=sample(f,c.left_eye,c.maps[0][i],c.maps[1][i]);b[k]=sample(f,1-c.left_eye,c.maps[2][i],c.maps[3][i]);}
 auto left=sgm(a,b,c.w,y1-y0,cfg.disparities,false);auto right=sgm(b,a,c.w,y1-y0,cfg.disparities,true);
 d.z.assign(n,std::numeric_limits<float>::quiet_NaN());d.valid.assign(n,0);d.disparity.assign(n,-1);
 for(int y=y0;y<y1;y++)for(int x=0;x<c.w;x++){size_t i=(size_t)y*c.w+x,k=(size_t)(y-y0)*c.w+x;float disparity=left[k];d.disparity[i]=disparity;if(disparity<=0)continue;int xr=(int)std::lround(x-disparity);if(xr<0||xr>=c.w||right[(size_t)(y-y0)*c.w+xr]<=0||std::abs(disparity-right[(size_t)(y-y0)*c.w+xr])>1.5)continue;
 if(x<c.roi1[0]||x>=c.roi1[0]+c.roi1[2]||y<c.roi1[1]||y>=c.roi1[1]+c.roi1[3]||xr<c.roi2[0]||xr>=c.roi2[0]+c.roi2[2]||y<c.roi2[1]||y>=c.roi2[1]+c.roi2[3])continue;
 double den=c.Q[12]*x+c.Q[13]*y+c.Q[14]*disparity+c.Q[15],z=(c.Q[8]*x+c.Q[9]*y+c.Q[10]*disparity+c.Q[11])/den;if(!std::isfinite(z)||z<cfg.min_depth||z>cfg.max_depth)continue;d.z[i]=(float)z;d.valid[i]=1;}return d;
}
Depth stereo(const Frame&f,const Calibration&c,const Config&cfg){return stereo_rows(f,c,cfg,0,c.h);}
Depth stereo_roi(const Frame&f,const Calibration&c,const Config&cfg,const Box&box){
 // 仅裁剪校正图的行，保留原始全部列与视差范围，
 // 避免视差原点偏移，并保留20cm近距搜索。
 // 在原框周围加入20%环带，用于椅子前景与背景归属判断。
 int lo=c.h,hi=-1;int map=box.eye==c.left_eye?0:2;double pad=.20*(box.y2-box.y1);
 for(int y=0;y<c.h;y++)for(int x=0;x<c.w;x++){size_t i=(size_t)y*c.w+x;float u=c.maps[map][i],v=c.maps[map+1][i];if(u>=box.x1-.20*(box.x2-box.x1)&&u<=box.x2+.20*(box.x2-box.x1)&&v>=box.y1-pad&&v<=box.y2+pad){lo=std::min(lo,y);hi=std::max(hi,y);}}
 if(hi<lo)return stereo(f,c,cfg);lo=std::max(0,lo-8);hi=std::min(c.h,hi+9);return stereo_rows(f,c,cfg,lo,hi);
}

static double median(std::vector<double>v){if(v.empty())return 0;std::nth_element(v.begin(),v.begin()+v.size()/2,v.end());return v[v.size()/2];}

Point associate(const Box&box,const Depth&d,const Calibration&c,const Config&){auto mapped=[&](int i){if(box.eye==c.left_eye)return std::array<double,2>{c.maps[0][i],c.maps[1][i]};int xr=i%d.w-(int)std::lround(d.disparity[i]);if(xr<0||xr>=d.w)return std::array<double,2>{-10000,-10000};int j=(i/d.w)*d.w+xr;return std::array<double,2>{c.maps[2][j],c.maps[3][j]};};struct P{double z;int i;};std::vector<P>p;int all=0;for(int y=0;y<d.h;y++)for(int x=0;x<d.w;x++){int i=y*d.w+x;auto pixel=mapped(i);double u=pixel[0],v=pixel[1];if(u<box.x1||u>=box.x2||v<box.y1||v>=box.y2)continue;all++;if(d.valid[i])p.push_back({d.z[i],i});}Point out;out.count=(int)p.size();out.fraction=all?(double)p.size()/all:0;if(p.size()<30||out.fraction<.03){out.reason="DEPTH_INSUFFICIENT_POINTS";return out;}std::sort(p.begin(),p.end(),[](P a,P b){return a.z<b.z;});std::vector<std::pair<int,int>>groups;int start=0;for(int i=1;i<=(int)p.size();i++){if(i==(int)p.size()||p[i].z-p[i-1].z>std::max(.08,.04*p[i-1].z)){if(i-start>=std::max(20,(int)p.size()/12))groups.push_back({start,i});start=i;}}

 // 不能确认表面归属时仍保留有限候选，
 // 由雷达融合进一步排序，这些候选不标记为depth_valid。
 for(const auto&cg:groups){if(out.candidate_count==3)break;std::vector<double>rr;
  for(int k=cg.first;k<cg.second;k++){int ix=p[k].i;double v[4]={(double)(ix%d.w),(double)(ix/d.w),d.disparity[ix],1},rect[3]={},den=0;for(int j=0;j<4;j++)den+=c.Q[12+j]*v[j];if(!std::isfinite(den)||std::abs(den)<1e-10)continue;for(int k=0;k<3;k++)for(int j=0;j<4;j++)rect[k]+=c.Q[k*4+j]*v[j]/den;double raw[3]={};for(int i=0;i<3;i++)for(int j=0;j<3;j++)raw[i]+=c.R[j*3+i]*rect[j];double r=std::hypot(raw[0],raw[2]);if(std::isfinite(r)&&r>0&&r<=20)rr.push_back(r);}
  if(rr.size()<20)continue;double r=median(rr);std::vector<double>dev;for(double v:rr)dev.push_back(std::abs(v-r));double error=std::max(.15,std::max(.15*r,3*median(dev)));out.candidates[out.candidate_count++]={r,error,(int)rr.size()};
 }
 if(groups.size()!=1){out.reason="DEPTH_AMBIGUOUS_CLUSTERS";return out;}auto g=groups[0];int count=g.second-g.first;if(count<(int)p.size()*.75){out.reason="DEPTH_AMBIGUOUS_SUPPORT";return out;}std::vector<double>zs;for(int i=g.first;i<g.second;i++)zs.push_back(p[i].z);double z=median(zs);std::vector<double>dev;for(double a:zs)dev.push_back(std::abs(a-z));double mad=median(dev);out.spread=mad;if(mad>std::max(.04,z*.04)){out.reason="DEPTH_TOO_DISPERSED";return out;}

 // 椅子检测框可能包含透过空隙看到的背景墙面。

 // 目标深度需由分割结果或有效前景点支持。

 if(box.cls==56||box.cls==80){

  // 图像与深度检查：前景需覆盖多个内部网格，

  // 且其深度与框外环形区域的背景分离。

  // 框内外深度相近时，认为测距主要来自背景墙面。

  double bw=box.x2-box.x1,bh=box.y2-box.y1;std::set<int>tiles;std::vector<double>annulus;

  for(int i=g.first;i<g.second;i++){int ix=p[i].i;auto pixel=mapped(ix);double u=pixel[0],v=pixel[1];int tx=std::clamp((int)((u-box.x1)*3/bw),0,2),ty=std::clamp((int)((v-box.y1)*3/bh),0,2);tiles.insert(ty*3+tx);}

  for(int y=0;y<d.h;y++)for(int x=0;x<d.w;x++){int i=y*d.w+x;if(!d.valid[i])continue;auto pixel=mapped(i);double u=pixel[0],v=pixel[1];bool outer=u>=box.x1-.15*bw&&u<box.x2+.15*bw&&v>=box.y1-.15*bh&&v<box.y2+.15*bh;bool inner=u>=box.x1&&u<box.x2&&v>=box.y1&&v<box.y2;if(outer&&!inner)annulus.push_back(d.z[i]);}

  if(annulus.size()<50||tiles.size()<4||median(annulus)<z+std::max(.20,.12*z)){out.reason="DEPTH_TARGET_SUPPORT_UNVERIFIED";return out;}

 }

 const P&rep=p[(g.first+g.second)/2];double x=rep.i%d.w,y=rep.i/d.w,disp=d.disparity[rep.i];double v[4]={x,y,disp,1},rect[3]={};double den=0;for(int j=0;j<4;j++)den+=c.Q[12+j]*v[j];if(!std::isfinite(den)||std::abs(den)<1e-10){out.reason="DEPTH_PROJECTION_INVALID";return out;}for(int k=0;k<3;k++)for(int j=0;j<4;j++)rect[k]+=c.Q[k*4+j]*v[j]/den;double raw[3]={};for(int i=0;i<3;i++)for(int j=0;j<3;j++)raw[i]+=c.R[j*3+i]*rect[j];if(!std::isfinite(raw[0])||!std::isfinite(raw[1])||!std::isfinite(raw[2])||raw[2]<=0){out.reason="DEPTH_PROJECTION_INVALID";return out;}out.x=raw[0];out.y=raw[1];out.z=raw[2];out.range=std::hypot(std::hypot(out.x,out.y),out.z);out.error=std::max(.03,3*mad+z*z/(std::max(1.,c.K[0]*(double)c.w/960)*c.baseline));out.valid=true;out.count=count;out.source_eye=box.eye;out.geometry_eye=c.left_eye;out.source_box=box;out.reason="OBSERVED_SURFACE_VALID";return out;}

void Axis::predict(double dt){x+=v*dt;p00+=2*dt*p01+dt*dt*p11+4*dt*dt;p01+=dt*p11;p11+=4*dt;}

void Axis::update(double z){double s=p00+16,k0=p00/s,k1=p01/s,e=z-x;double a=p00,b=p01;x+=k0*e;v+=k1*e;p00=(1-k0)*a;p01=(1-k0)*b;p11-=k1*b;}

static std::array<double,4>measure(const Box&b){return{(b.x1+b.x2)/2.,(b.y1+b.y2)/2.,b.x2-b.x1,b.y2-b.y1};}

static Box predicted(const Track&t){

 Box b=t.box;double cx=t.kf[0].x,cy=t.kf[1].x,w=t.kf[2].x,h=t.kf[3].x;

 /* 预测框允许暂时出图，但保持有界和有效；仍标记为未观测，不生成测量值。

 */

 if(!std::isfinite(cx)||!std::isfinite(cy)||!std::isfinite(w)||!std::isfinite(h)){cx=(b.x1+b.x2)*.5;cy=(b.y1+b.y2)*.5;w=b.x2-b.x1;h=b.y2-b.y1;}

 w=std::clamp(w,1.,960.);h=std::clamp(h,1.,1080.);cx=std::clamp(cx,.5,959.5);cy=std::clamp(cy,.5,1079.5);

 b.x1=(float)std::clamp(cx-w/2,0.,959.);b.y1=(float)std::clamp(cy-h/2,0.,1079.);b.x2=(float)std::clamp(cx+w/2,(double)b.x1+1.,960.);b.y2=(float)std::clamp(cy+h/2,(double)b.y1+1.,1080.);return b;

}

static double iou(const Box&a,const Box&b){double in=std::max(0.f,std::min(a.x2,b.x2)-std::max(a.x1,b.x1))*std::max(0.f,std::min(a.y2,b.y2)-std::max(a.y1,b.y1));double sum=(a.x2-a.x1)*(a.y2-a.y1)+(b.x2-b.x1)*(b.y2-b.y1)-in;return sum>0?in/sum:0;}

static double box_area(const Box&b){return (b.x2-b.x1)*(b.y2-b.y1);}

static double box_aspect(const Box&b){return (b.x2-b.x1)/(b.y2-b.y1);}

static GrayAppearance describe(const Frame&f,const Box&b){

 GrayAppearance d;if(f.gray.size()!=2073600)return d;

 double sum=0,sq=0;std::array<float,256>v{};

 for(int y=0;y<16;y++)for(int x=0;x<16;x++){

  double u=b.x1+(b.x2-b.x1)*(.05+.90*(x+.5)/16),w=b.y1+(b.y2-b.y1)*(.05+.90*(y+.5)/16);

  int xx=std::clamp((int)u,0,959),yy=std::clamp((int)w,0,1079);float z=f.gray[yy*1920+b.eye*960+xx];

  v[y*16+x]=z;sum+=z;sq+=z*z;d.hist[(int)z/16]+=1.f/256;

 }

 d.mean=(float)(sum/256);d.contrast=(float)std::sqrt(std::max(0.,sq/256-d.mean*d.mean));d.valid=d.contrast>=8;

 for(int y=0;y<8;y++)for(int x=0;x<8;x++){double z=0;for(int dy=0;dy<2;dy++)for(int dx=0;dx<2;dx++)z+=v[(y*2+dy)*16+x*2+dx];d.cells[y*8+x]=(float)((z/4-d.mean)/std::max(12.f,d.contrast));}

 return d;

}

static double appearance_distance(const GrayAppearance&a,const GrayAppearance&b){

 if(!a.valid||!b.valid)return 1.;double cells=0,hist=0;

 for(int i=0;i<64;i++)cells+=std::min(3.f,std::fabs(a.cells[i]-b.cells[i]));

 for(int i=0;i<16;i++)hist+=std::fabs(a.hist[i]-b.hist[i]);

 return .70*cells/128+.25*hist/2+.05*std::fabs(a.mean-b.mean)/255;

}

static double profile_distance(const Track&t,const GrayAppearance&d){double v=appearance_distance(t.identity_reference,d);for(int i=0;i<t.template_count;i++)v=std::min(v,appearance_distance(t.templates[i],d));return v;}

static bool world_fresh(const WorldObservation&w,const Frame&f){return w.valid&&w.time<=f.time&&f.time-w.time<=1200000&&w.frame<=f.id&&f.id-w.frame<=12;}

[[maybe_unused]] static bool anchor_continuity(const Track&t,const Box&b){

 double ratio=box_area(b)/std::max(1.,box_area(t.last_observed_box));if(ratio<.16||ratio>6.25)return false;

 if(iou(t.box,b)>=.16)return true;

 double w=std::max(80.,double(t.last_observed_box.x2-t.last_observed_box.x1));

 double h=std::max(80.,double(t.last_observed_box.y2-t.last_observed_box.y1));

 return std::hypot(((b.x1+b.x2)-(t.box.x1+t.box.x2))*.5/w,((b.y1+b.y2)-(t.box.y1+t.box.y2))*.5/h)<=.50;

}

bool apply_world_bridge(const std::string&path,Tracker&tr,uint64_t epoch,uint64_t frame,uint64_t now){

 try{J j=JP(read(path,16384)).parse();uint64_t source=decimal(j.at("frame_id").str()),time=decimal(j.at("published_us").str());

  if(j.at("version").num()!=1||decimal(j.at("epoch").str())!=epoch||source>frame||frame-source>12||time>now||now-time>1200000||j.at("objects").type!='a'||j.at("objects").values.size()>65)return false;

  std::string context;const auto&cx=j.at("context");if(cx.type!='o')return false;

  for(const char*key:{"source","clock","timeline_id","origin_epoch"}){const auto&v=cx.at(key);if(v.type=='s')context+=quote(v.str());else if(v.type=='d')context+=std::to_string(v.num());else return false;context+='|';}

  std::vector<std::pair<int,WorldObservation>>pending;std::set<int>ids;

  for(const auto&o:j.at("objects").values){double n=o.at("track_id").num();if(n<1||n>INT32_MAX||n!=std::floor(n)||!ids.insert((int)n).second)return false;

   if(o.at("valid").type!='b')return false;if(!o.at("valid").boolean)continue;

   double x=o.at("x").num(),y=o.at("y").num(),e=o.at("error_m").num();if(std::fabs(x)>100000||std::fabs(y)>100000||e<0||e>.35)return false;

   uint64_t source_time=decimal(o.at("source_us").str());if(source_time>time||source_time>now||now-source_time>1200000)return false;

   pending.push_back({(int)n,{true,x,y,e,source,source_time}});

  }

  if(!tr.world_context.empty()&&tr.world_context!=context){for(auto&t:tr.tracks){t.world={};t.anchor.world_valid=false;if(t.anchor_locked)t.identity_reason="WORLD_CONTEXT_RESET_GRAY_IDENTITY_RETAINED";}}

  tr.world_context=context;

  for(const auto&v:pending)for(auto&t:tr.tracks)if(t.id==v.first&&t.last_frame==source&&t.observed_at<=time&&time-t.observed_at<=1200000){

   t.world=v.second;

   // 首次可靠位置可能晚于选择一帧，只有同一编号的

   // 真实观测到达后才冻结一次初始位置。

   if(t.anchor_locked&&!t.anchor.world_valid){t.anchor.world_valid=true;t.anchor.world_x=v.second.x;t.anchor.world_y=v.second.y;}

  }

  return true;

 }catch(...){return false;}

}

static void identity_event(Tracker&tr,Track&t,uint64_t now,const char*code,const char*why){

 // 累计重复拒绝帧数，相同原因日志及环形事件

 // 每目标每秒最多合并记录一次，环形记录不超过32项。

 t.identity_reason=why;

 if(std::strcmp(code,"OBJECT_IDENTITY_REJECTED")==0){

  if(t.last_identity_reject_us&&now>=t.last_identity_reject_us&&now-t.last_identity_reject_us<1000000)return;

  t.last_identity_reject_us=now;

 }

 t.last_identity_event_us=now;tr.identity_events.push_back({now,t.id,code,why});++tr.identity_event_total;

 while(tr.identity_events.size()>32)tr.identity_events.pop_front();

 std::fprintf(stderr,"%s track_id=%d reason=%s frame_time_us=%llu\n",code,t.id,why,(unsigned long long)now);

}

void Tracker::reset(uint64_t e){follow_exit=FollowExitMemory{};follow_policy=spatial_target_valid=false;policy_task_class_id=-1;policy_time=policy_binding_generation=0;spatial_source_count=0;reacquire_lidar_valid=false;reacquire_lidar_time=0;reacquire_geometry_frame=reacquire_region_after_context_us=0;reacquire_anchor_ray_valid=reacquire_region_valid=false;reacquire_geometry_valid.fill(false);reacquire_region_context.clear();reacquire_region_reason="REGION_NOT_PROVIDED";world_context.clear();tracks.clear();identity_events.clear();identity_event_total=0;anchor_owner=-1;next_id=1;epoch=e;last_time=last_frame_id=0;selected=-1;selection_lost=false;reid_owner=reid_candidate=-1;reid_hits=0;reid_last_us=reid_time_us=0;reid_from=-1;}

#include "composite_association.inc"
#include "image_tracking.inc"
#include "continuity_v4.inc"
#include "reacquisition.inc"
#include "byte_association.inc"
#include "tracking_s3.inc"
#include "reacquisition_bridge.inc"

void select_request(const std::string&path,Tracker&t,const std::deque<SnapshotIdentity>&history,uint64_t epoch,uint64_t now,Selection&selection){std::string bytes;try{bytes=read(path,4096);}catch(...){return;}try{J j=JP(bytes).parse();uint64_t req=decimal(j.at("request_id").str());if(req<=selection.request)return;Selection next;next.request=req;next.state="REJECTED";next.reason="SELECTION_INVALID";if(j.at("version").num()!=1)throw std::runtime_error("SELECTION_VERSION");auto op=j.at("operation").str();if(op=="clear"){for(auto&tr:t.tracks){tr.anchor={};tr.anchor_locked=false;tr.identity_reference={};tr.template_count=0;tr.identity_reason="UNLOCKED";tr.identity_missed=0;tr.last_identity_reject_us=0;}t.selected=-1;t.selection_lost=false;t.reid_time_us=0;t.reid_from=-1;next.state="CLEARED";next.reason="USER_CLEARED";selection=next;return;}if(op=="roi"){
 if(decimal(j.at("epoch").str())!=epoch)throw std::runtime_error("SELECTION_EPOCH_CHANGED");double ey=j.at("eye").num();if(ey!=0&&ey!=1)throw std::runtime_error("ROI_EYE_INVALID");const J&bb=j.at("bbox");if(bb.type!='a'||bb.values.size()!=4)throw std::runtime_error("ROI_BOX_INVALID");Box box{80,(int)ey,(float)bb.values[0].num(),(float)bb.values[1].num(),(float)bb.values[2].num(),(float)bb.values[3].num(),1};if(box.x1<0||box.y1<0||box.x2>960||box.y2>1080||box.x2-box.x1<8||box.y2-box.y1<8)throw std::runtime_error("ROI_BOX_INVALID");std::string snap=j.at("snapshot_id").str();const SnapshotIdentity*issued=nullptr;for(const auto&h:history)if(h.id==snap&&h.time<=now&&now-h.time<=8000000&&h.gray_quarter.size()==129600&&(h.issued_eye_mask&(1u<<(int)ey)))issued=&h;if(!issued)throw std::runtime_error("ROI_SNAPSHOT_EXPIRED_OR_IMAGE_UNAVAILABLE");size_t dash=snap.rfind('-');if(dash==std::string::npos||decimal(snap.substr(0,dash))!=epoch)throw std::runtime_error("ROI_SNAPSHOT_ID_INVALID");next.track=t.select_roi(box,describe_snapshot(*issued,box),decimal(snap.substr(dash+1)),issued->time);for(auto&track:t.tracks)if(track.id==next.track)track.manual_seed_us=now;next.state="SELECTED";next.reason="USER_ROI_LOCKED_AWAIT_CURRENT_PIXEL_MEASUREMENT";selection=next;return;
 }if(op!="select"||decimal(j.at("epoch").str())!=epoch)throw std::runtime_error("SELECTION_EPOCH_CHANGED");double id=j.at("track_id").num();if(id!=std::floor(id)||id<1||id>INT32_MAX)throw std::runtime_error("SELECTION_TRACK_INVALID");next.track=(int)id;std::string snap=j.at("snapshot_id").str();const SnapshotIdentity*issued_photo=nullptr;bool issued=false;for(auto&h:history)if(h.id==snap&&h.time<=now&&now-h.time<=8000000&&std::find(h.tracks.begin(),h.tracks.end(),next.track)!=h.tracks.end()){issued=true;issued_photo=&h;}if(!issued)throw std::runtime_error("SELECTION_SNAPSHOT_EXPIRED");auto i=std::find_if(t.tracks.begin(),t.tracks.end(),[&](const Track&a){return a.id==next.track;});if(i==t.tracks.end()||i->retired||!i->observed_at||i->observed_at>now||now-i->observed_at>2000000 )throw std::runtime_error("SELECTION_TRACK_NOT_OBSERVED");if(issued_photo&&issued_photo->track_bindings.count(next.track)&&issued_photo->track_bindings.at(next.track)!=i->binding_generation)throw std::runtime_error("SELECTION_BINDING_CHANGED_UPDATE_PHOTO");i->task_class_id=issued_photo&&issued_photo->track_boxes.count(next.track)?issued_photo->track_boxes.at(next.track).cls:i->observed_class_id>=0?i->observed_class_id:i->cls;for(auto&tr:t.tracks){tr.anchor={};tr.anchor_locked=false;tr.identity_reference={};tr.template_count=0;tr.identity_reason="UNLOCKED";tr.identity_missed=0;tr.last_identity_reject_us=0;}t.selected=next.track;t.selection_lost=false;t.reid_time_us=0;t.reid_from=-1;next.state="SELECTED";next.reason="USER_SELECTED_TRACK";selection=next;

 }catch(const std::exception&e){try{J j=JP(bytes).parse();uint64_t req=decimal(j.at("request_id").str());if(req>selection.request){selection.request=req;selection.state="REJECTED";selection.reason=e.what();selection.track=-1;}}catch(...){}}}

const char*label(int c){return active_model.label(c);}

static std::string b64(const std::string&b){static const char*a="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";std::string s;s.reserve((b.size()+2)/3*4);for(size_t i=0;i<b.size();i+=3){unsigned n=(unsigned char)b[i]<<16;if(i+1<b.size())n|=(unsigned char)b[i+1]<<8;if(i+2<b.size())n|=(unsigned char)b[i+2];s+=a[n>>18];s+=a[(n>>12)&63];s+=i+1<b.size()?a[(n>>6)&63]:'=';s+=i+2<b.size()?a[n&63]:'=';}return s;}

std::string png_data(const Frame&f,int eye){if(f.gray.size()!=2073600)return"";std::vector<uint8_t>a(OW*OH);for(int y=0;y<OH;y++)std::memcpy(a.data()+y*OW,f.gray.data()+y*1920+eye*OW,OW);std::string png;stbi_write_png_compression_level=1;auto write=[](void*ctx,void*p,int n){auto*s=(std::string*)ctx;if(s->size()+(size_t)n>1500000)throw std::runtime_error("PNG_LIMIT");s->append((char*)p,n);};if(!stbi_write_png_to_func(write,&png,OW,OH,1,a.data(),OW))throw std::runtime_error("PNG_ENCODE_FAILED");return"data:image/png;base64,"+b64(png);}
std::string jpeg_data(const Frame&f,int eye,int quality){if(f.gray.size()!=2073600||eye<0||eye>1)return"";std::vector<uint8_t>a(OW*OH);for(int y=0;y<OH;y++)std::memcpy(a.data()+y*OW,f.gray.data()+y*1920+eye*OW,OW);std::string jpeg;auto write=[](void*ctx,void*p,int n){auto*s=(std::string*)ctx;if(s->size()+(size_t)n>1500000)throw std::runtime_error("JPEG_LIMIT");s->append((char*)p,n);};if(!stbi_write_jpg_to_func(write,&jpeg,OW,OH,1,a.data(),std::clamp(quality,40,95)))throw std::runtime_error("JPEG_ENCODE_FAILED");return"data:image/jpeg;base64,"+b64(jpeg);}


#include "follow_exit_v5.inc"
#include "state_s3.inc"
}
