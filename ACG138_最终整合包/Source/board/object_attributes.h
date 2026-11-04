#ifndef ACG_OBJECT_ATTRIBUTES_H
#define ACG_OBJECT_ATTRIBUTES_H
#include <stdint.h>
#include <string.h>
/* 共享可选JSON属性v1，不修改FPGA或共享传感器ABI。 */
typedef struct obj_attributes {
 char color[16],secondary_color[16],size[12],size_source[32];
 int color_valid,size_valid,color_eye;
 double color_support,size_support,width_m,height_m;
 uint64_t color_frame_id,color_source_us,size_frame_id,size_source_us;
} obj_attributes;
enum {OBJ_COLOR_UNKNOWN,OBJ_COLOR_RED,OBJ_COLOR_ORANGE,OBJ_COLOR_YELLOW,OBJ_COLOR_GREEN,OBJ_COLOR_CYAN,OBJ_COLOR_BLUE,OBJ_COLOR_PURPLE,OBJ_COLOR_PINK,OBJ_COLOR_BROWN,OBJ_COLOR_WHITE,OBJ_COLOR_GRAY,OBJ_COLOR_BLACK,OBJ_COLOR_MIXED};
static inline const char*obj_color_name(unsigned i){static const char*const n[]={"unknown","red","orange","yellow","green","cyan","blue","purple","pink","brown","white","gray","black","mixed"};return i<14?n[i]:n[0];}
static inline const char*obj_color_zh(unsigned i){static const char*const n[]={"","红色","橙色","黄色","绿色","青色","蓝色","紫色","粉色","棕色","白色","灰色","黑色","多色"};return i<14?n[i]:n[0];}
static inline unsigned obj_color_id(const char*s){if(!s)return 0;for(unsigned i=1;i<14;++i)if(!strcmp(s,obj_color_name(i)))return i;return 0;}
static inline int obj_attribute_match(const obj_attributes*a,const char*color,const char*size){return (!color||!*color||(a&&a->color_valid&&(!strcmp(a->color,color)||!strcmp(a->secondary_color,color))))&&(!size||!*size||(a&&a->size_valid&&!strcmp(a->size,size)));}
static inline int obj_attribute_color_match(const obj_attributes*a,const char*color){return !color||!*color||(a&&a->color_valid&&(!strcmp(a->color,color)||!strcmp(a->secondary_color,color)));}
static inline double obj_attribute_metric_extent(const obj_attributes*a,const char*category){return !a||!a->size_valid?0:category&&!strcmp(category,"bottle")?a->height_m:a->width_m>a->height_m?a->width_m:a->height_m;}
#endif
