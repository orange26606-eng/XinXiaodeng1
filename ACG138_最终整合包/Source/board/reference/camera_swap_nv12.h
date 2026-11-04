/* 在YOLO处理前交换双眼图像。 */
#ifndef CAMERA_SWAP_NV12_H
#define CAMERA_SWAP_NV12_H
#include <stddef.h>
#include <stdint.h>
/* 逐行复制NV12的Y平面和交错UV平面到紧凑缓冲区。宽度需为4的倍数，以保持每半幅图的色度对完整。源与目标不重叠，不镜像、不缩放、不转换颜色。

 */
int camera_swap_nv12(uint8_t *dst,size_t dst_bytes,const uint8_t *src,size_t src_bytes,
                     unsigned width,unsigned height,unsigned stride);
#endif
