#pragma once
#include "attribute_font.h"
#include "color_attributes.h"
namespace acg {
inline uint32_t attribute_codepoint(const unsigned char*&s){unsigned c=*s++;if(c<128)return c;unsigned n=(c&224)==192?1:(c&240)==224?2:(c&248)==240?3:0;uint32_t v=c&((1u<<(6-n))-1);if(!n)return '?';for(unsigned i=0;i<n;i++){if(!*s||(*s&192)!=128)return '?';v=(v<<6)|(*s++&63);}return v;}
inline const AttributeGlyph*attribute_glyph(uint32_t c){for(const auto&g:attribute_glyphs)if(g.code==c)return &g;return nullptr;}
inline void draw_attribute_label(std::vector<uint8_t>&nv12,const Box&b,const std::string&caption){
 if(nv12.size()!=3110400)return;std::vector<const AttributeGlyph*>glyphs;int width=0,above=0,below=0;
 const auto*s=(const unsigned char*)caption.c_str();while(*s){const auto*g=attribute_glyph(attribute_codepoint(s));if(!g)g=attribute_glyph('?');glyphs.push_back(g);width+=g->advance;above=std::max(above,-g->top);below=std::max(below,g->top+g->height);}
 int eye=(b.x1+b.x2)*.5>=960?1:0;constexpr int pad=5;int plate_w=std::min(960,width+2*pad),plate_h=above+below+2*pad;
 int x0=std::clamp((int)b.x1,eye*960,eye*960+960-plate_w),y0=b.y1>=plate_h+3?(int)b.y1-plate_h-3:std::min(1080-plate_h,(int)b.y1+3);
 for(int y=y0;y<y0+plate_h;y++)std::fill(nv12.begin()+y*1920+x0,nv12.begin()+y*1920+x0+plate_w,235);
 for(int y=y0/2;y<(y0+plate_h+1)/2;y++)std::fill(nv12.begin()+2073600+y*1920+(x0&~1),nv12.begin()+2073600+y*1920+((x0+plate_w+1)&~1),128);
 int cursor=x0+pad,baseline=y0+pad+above;
 for(const auto*g:glyphs){for(int y=0;y<g->height;y++)for(int x=0;x<g->width;x++){unsigned a=attribute_font_pixels[g->offset+y*g->width+x];int px=cursor+g->left+x,py=baseline+g->top+y;if(a&&px>=x0&&px<x0+plate_w&&py>=y0&&py<y0+plate_h)nv12[py*1920+px]=(235*(255-a)+16*a+127)/255;}cursor+=g->advance;if(cursor>=x0+plate_w-pad)break;}
}
}
