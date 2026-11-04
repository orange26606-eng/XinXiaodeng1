#ifndef ACG_OBJECT_ATTRIBUTE_REQUEST_H
#define ACG_OBJECT_ATTRIBUTE_REQUEST_H
#include "../../../board/object_attributes.h"
#include <stddef.h>
/* 仅用于本地私有IPC，reserved6携带类别、颜色及大小，下划线表示空；不构造坐标、图片编号或电机命令。
 */
static inline int object_attribute_search_parse(const char*text,char*category,size_t nc,char*color,size_t np,char*size,size_t ns){
 if(!text||!category||!color||!size||nc<2||np<2||ns<2)return 0;
 const char*first=strchr(text,'|');if(!first)return 0;
 const char*second=strchr(first+1,'|');if(!second||strchr(second+1,'|'))return 0;
 size_t a=(size_t)(first-text),b=(size_t)(second-first-1),c=strlen(second+1);
 if(!a||a>=nc||!b||b>=np||!c||c>=ns)return 0;
 memcpy(category,text,a);category[a]=0;memcpy(color,first+1,b);color[b]=0;memcpy(size,second+1,c);size[c]=0;
 for(size_t i=0;i<a;++i)if(!((category[i]>='a'&&category[i]<='z')||category[i]=='_'||category[i]==' '))return 0;
 if(!strcmp(color,"_"))color[0]=0;if(!strcmp(size,"_"))size[0]=0;
 return (!color[0]||obj_color_id(color))&&(!size[0]||!strcmp(size,"small")||!strcmp(size,"medium")||!strcmp(size,"large"));
}
#endif
