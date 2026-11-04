/* 保留NV12像素值，并检查行填充边界。 */
#include "camera_swap_nv12.h"
#include <string.h>
int camera_swap_nv12(uint8_t *dst,size_t dst_bytes,const uint8_t *src,size_t src_bytes,
                     unsigned width,unsigned height,unsigned stride)
{
    if(!dst || !src || !width || !height || width>8192 || height>8192 ||
       width%4 || height%2 || stride<width || stride>65536) return -1;
    size_t rows=(size_t)height*3/2;
    if(src_bytes<rows*stride || dst_bytes<rows*width ||
       ((uintptr_t)dst<=(uintptr_t)src ? (uintptr_t)src-(uintptr_t)dst<rows*width :
                                      (uintptr_t)dst-(uintptr_t)src<rows*stride)) return -1;
    for(size_t row=0;row<rows;++row) {
        memcpy(dst+row*width,src+row*stride+width/2,width/2);
        memcpy(dst+row*width+width/2,src+row*stride,width/2);
    }
    return 0;
}
