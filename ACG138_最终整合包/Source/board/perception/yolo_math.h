#ifndef ACG_YOLO_MATH_H
#define ACG_YOLO_MATH_H
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include "neon_yuv8.h"
namespace acg {
struct Box {float x1,y1,x2,y2,score;int cls;unsigned color=0,secondary_color=0,color_samples=0;float color_support=0;bool color_valid=false;};
inline float iou(const Box&a,const Box&b){
    float intersection=std::max(0.f,std::min(a.x2,b.x2)-std::max(a.x1,b.x1))*std::max(0.f,std::min(a.y2,b.y2)-std::max(a.y1,b.y1));
    float area=std::max(0.f,a.x2-a.x1)*std::max(0.f,a.y2-a.y1)+std::max(0.f,b.x2-b.x1)*std::max(0.f,b.y2-b.y1)-intersection;
    return area>0?intersection/area:0;
}
inline std::vector<Box> nms(std::vector<Box> boxes,float threshold=.45f){
    std::sort(boxes.begin(),boxes.end(),[](const Box&a,const Box&b){return a.score>b.score;});
    if(boxes.size()>1024)boxes.resize(1024);
    std::vector<Box> keep;
    for(const Box&b:boxes){bool overlap=false;for(const Box&k:keep)if(k.cls==b.cls&&iou(k,b)>threshold){overlap=true;break;}if(!overlap)keep.push_back(b);if(keep.size()==128)break;}
    return keep;
}
inline float dfl(const float *data,unsigned step,unsigned bins=16){
    float maximum=data[0];for(unsigned k=1;k<bins;k++)maximum=std::max(maximum,data[k*step]);
    float sum=0,weighted=0;for(unsigned k=0;k<bins;k++){float e=std::exp(data[k*step]-maximum);sum+=e;weighted+=k*e;}return weighted/sum;
}
struct Tensor {const float*data=nullptr;unsigned channels=0,height=0,width=0;bool nhwc=false;
    float at(unsigned c,unsigned y,unsigned x)const{return data[nhwc?(y*width+x)*channels+c:(c*height+y)*width+x];}
};
inline std::vector<Box> decode(const std::array<Tensor,9>&out,int eye,float threshold=.25f,unsigned class_count=80,bool use_sum_gate=true){
    std::vector<Box> boxes;
    for(unsigned side:{80u,40u,20u}){
        const Tensor *reg=nullptr,*cls=nullptr,*sum=nullptr;
        for(const auto&t:out)if(t.width==side&&t.height==side){if(t.channels==64)reg=&t;else if(t.channels==class_count)cls=&t;else if(t.channels==1)sum=&t;}
        if(!reg||!cls||!sum)throw std::runtime_error("Expected 9 YOLO11 DFL/class/score_sum outputs");
        const float stride=640.f/side;
        for(unsigned y=0;y<side;y++)for(unsigned x=0;x<side;x++){
            if(use_sum_gate&&sum->at(0,y,x)<threshold)continue;
            float score=threshold;int category=-1;
            for(unsigned c=0;c<class_count;c++){float v=cls->at(c,y,x);if(v>score){score=v;category=(int)c;}}
            if(category<0)continue;
            float distances[4];
            for(unsigned s=0;s<4;s++){float values[16];for(unsigned k=0;k<16;k++)values[k]=reg->at(s*16+k,y,x);distances[s]=dfl(values,1)*stride;}
            constexpr float gain=640.f/1080.f;constexpr float pad=35.f;
            auto px=[eye](float value){return std::clamp((value-pad)/gain,0.f,960.f)+eye*960;};
            auto py=[](float value){return std::clamp(value/gain,0.f,1080.f);};
            float cx=(x+.5f)*stride,cy=(y+.5f)*stride;
            Box b{px(cx-distances[0]),py(cy-distances[1]),px(cx+distances[2]),py(cy+distances[3]),score,category};
            if(std::isfinite(score)&&score<=1&&b.x2>b.x1&&b.y2>b.y1)boxes.push_back(b);
        }
    }
    return nms(std::move(boxes));
}
// Ultralytics解码输出形状为[1,4+classes,8400]或其转置。
// xywh使用640像素补边坐标，类别概率已经过sigmoid处理。
inline std::vector<Box> decode_decoded(const float*data,size_t elements,unsigned channels,unsigned anchors,bool transposed,int eye,float threshold=.10f){
 if(!data||channels!=10||anchors!=8400||elements<(size_t)channels*anchors||eye<0||eye>1)throw std::runtime_error("CUSTOM6_OUTPUT_SHAPE_MISMATCH");
 auto at=[&](unsigned c,unsigned a){return data[transposed?(size_t)a*channels+c:(size_t)c*anchors+a];};std::vector<Box>boxes;boxes.reserve(128);
 for(unsigned a=0;a<anchors;a++){
  float score=threshold;int cls=-1;for(unsigned c=4;c<channels;c++){float v=at(c,a);if(std::isfinite(v)&&v>score&&v<=1){score=v;cls=(int)c-4;}}if(cls<0)continue;
  float x=at(0,a),y=at(1,a),w=at(2,a),h=at(3,a);if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(w)||!std::isfinite(h)||w<=0||h<=0)continue;
  constexpr float gain=640.f/1080.f,pad=35.f;auto px=[&](float v){return std::clamp((v-pad)/gain,0.f,960.f)+eye*960;};auto py=[&](float v){return std::clamp(v/gain,0.f,1080.f);};
  Box b{px(x-w/2),py(y-h/2),px(x+w/2),py(y+h/2),score,cls};if(b.x2>b.x1&&b.y2>b.y1)boxes.push_back(b);
 }
 return nms(std::move(boxes));
}
struct InputPlan {
    static constexpr int width=569,height=640,left=35;
    std::array<unsigned,width> xs{};std::array<unsigned,height> ys{};
    InputPlan(){for(unsigned x=0;x<width;x++)xs[x]=std::min(959u,x*960u/width);for(unsigned y=0;y<height;y++)ys[y]=std::min(1079u,y*1080u/height);}
    void convert(const uint8_t*nv12,int eye,uint8_t*rgb)const{
        std::fill(rgb,rgb+640*640*3,114);
        auto clip=[](int x){return(uint8_t)std::clamp(x,0,255);};
        for(unsigned y=0;y<height;y++){
            const uint8_t*yp=nv12+ys[y]*1920;const uint8_t*uv=nv12+1920*1080+(ys[y]/2)*1920;
            uint8_t*dst=rgb+(y*640+left)*3;
            unsigned x=0;
            for(;x+8<=width;x+=8){uint8_t yy8[8],u8[8],v8[8];for(unsigned k=0;k<8;k++){unsigned sx=xs[x+k]+eye*960;yy8[k]=yp[sx];u8[k]=uv[sx&~1u];v8[k]=uv[(sx&~1u)+1];}acg_neon_yuv8(yy8,u8,v8,dst+3*x);}
            for(;x<width;x++){
                unsigned sx=xs[x]+eye*960;int yy=std::max(0,(int)yp[sx]-16),u=(int)uv[sx&~1u]-128,v=(int)uv[(sx&~1u)+1]-128;
                dst[3*x]=clip((298*yy+409*v+128)>>8);dst[3*x+1]=clip((298*yy-100*u-208*v+128)>>8);dst[3*x+2]=clip((298*yy+516*u+128)>>8);
            }
        }
    }
};
}
#include "color_attributes.h"
#endif
