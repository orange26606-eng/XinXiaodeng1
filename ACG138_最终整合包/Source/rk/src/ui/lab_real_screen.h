#ifndef ACG_LAB_REAL_SCREEN_H
#define ACG_LAB_REAL_SCREEN_H
#include "lab_real_runtime.h"
/* 借用视图同步消费，不执行IO、线程、控制或内存分配。 */
void lab_real_screen_draw(uint32_t *pixels,int width,int height,const lab_real_runtime_view *view);
#endif
