#pragma once
#include "../object_attributes.h"
#include "yolo_math.h"
#include <array>
#include <cmath>
#include <string>
namespace acg {
inline unsigned color_rgb(int r,int g,int b){
 int maximum=std::max(r,std::max(g,b)),minimum=std::min(r,std::min(g,b)),delta=maximum-minimum;
 if(maximum<42)return OBJ_COLOR_BLACK;
 if(delta<maximum*.17)return maximum>205?OBJ_COLOR_WHITE:OBJ_COLOR_GRAY;
 double h=delta==0?0:maximum==r?60.*(g-b)/delta:maximum==g?120.+60.*(b-r)/delta:240.+60.*(r-g)/delta;
 if(h<0)h+=360.;if(h<15||h>=345)return OBJ_COLOR_RED;
 if(h<48)return maximum<155?OBJ_COLOR_BROWN:OBJ_COLOR_ORANGE;
 if(h<75)return OBJ_COLOR_YELLOW;if(h<165)return OBJ_COLOR_GREEN;if(h<195)return OBJ_COLOR_CYAN;
 if(h<255)return OBJ_COLOR_BLUE;if(h<285)return OBJ_COLOR_PURPLE;return OBJ_COLOR_PINK;
}
// 从未经叠加的BT.601有限范围NV12中，按目标内侧32×32网格采样。
// 区域始终位于原始单眼目标框内，不使用叠加图或灰度输入。
inline void extract_color(const uint8_t*nv12,size_t bytes,Box&box){
 box.color=box.secondary_color=box.color_samples=0;box.color_support=0;box.color_valid=false;
 if(!nv12||bytes!=3110400||!std::isfinite(box.x1)||!std::isfinite(box.x2)||!std::isfinite(box.y1)||!std::isfinite(box.y2))return;
 int eye=(box.x1+box.x2)*.5>=960?1:0,left=eye*960;
 if(box.x1<left||box.x2>left+960||box.y1<0||box.y2>1080||box.x2<=box.x1||box.y2<=box.y1)return;
 float dx=box.x2-box.x1,dy=box.y2-box.y1;int x0=(int)(box.x1+.12f*dx),x1=(int)(box.x2-.12f*dx),y0=(int)(box.y1+.10f*dy),y1=(int)(box.y2-.10f*dy);
 if(x1<=x0||y1<=y0)return;std::array<unsigned,14>count{};
 auto clip=[](int n){return std::clamp(n,0,255);};
 for(int gy=0;gy<32;++gy)for(int gx=0;gx<32;++gx){int x=std::clamp(x0+(x1-x0)*(2*gx+1)/64,left,left+959),y=std::clamp(y0+(y1-y0)*(2*gy+1)/64,0,1079);
 int yy=std::max(0,(int)nv12[y*1920+x]-16),uv=2073600+(y/2)*1920+(x&~1),u=(int)nv12[uv]-128,v=(int)nv12[uv+1]-128;
 int r=clip((298*yy+409*v+128)>>8),g=clip((298*yy-100*u-208*v+128)>>8),b=clip((298*yy+516*u+128)>>8);
 ++count[color_rgb(r,g,b)];++box.color_samples;}
 unsigned first=0,second=0;for(unsigned i=1;i<14;++i)if(count[i]>count[first])first=i;
 for(unsigned i=1;i<14;++i)if(i!=first&&count[i]>count[second])second=i;
 box.color_support=count[first]/(float)box.color_samples;
 if(first&&box.color_support>=.38f){box.color=first;box.color_valid=true;if(second&&count[second]>=box.color_samples*.22f)box.secondary_color=second;}
}
inline const char*attribute_object_zh(const char*s){if(!std::strcmp(s,"person"))return "人";if(!std::strcmp(s,"cup"))return "杯子";if(!std::strcmp(s,"cell_phone")||!std::strcmp(s,"cell phone"))return "手机";if(!std::strcmp(s,"bottle"))return "水瓶";if(!std::strcmp(s,"box"))return "箱子";if(!std::strcmp(s,"remote"))return "遥控器";return s;}
inline std::string attribute_caption(const char*name,const Box&b){return std::string(b.color_valid?obj_color_zh(b.color):"")+attribute_object_zh(name);}
}
