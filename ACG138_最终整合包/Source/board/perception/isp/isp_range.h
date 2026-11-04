#ifndef ISP_RANGE_H
#define ISP_RANGE_H
/* 全范围到有限范围NV12转换，系数219/224保持原查表的整数舍入结果。
 */
#include <stddef.h>
#include <stdint.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define ISP_RANGE_NEON 1
#else
#define ISP_RANGE_NEON 0
#endif

static inline void isp_range_u8(uint8_t *dst,const uint8_t *src,size_t count,uint8_t coefficient) {
    size_t i=0;
#if ISP_RANGE_NEON
    const uint8x8_t scale=vdup_n_u8(coefficient);
    const uint16x8_t bias=vdupq_n_u16(127);
    const uint16x8_t one=vdupq_n_u16(1);
    const uint8x16_t offset=vdupq_n_u8(16);
    for(;i+16<=count;i+=16) {
        const uint8x16_t pixels=vld1q_u8(src+i);
        uint16x8_t lo=vmlal_u8(bias,vget_low_u8(pixels),scale);
        uint16x8_t hi=vmlal_u8(bias,vget_high_u8(pixels),scale);
        /* n不大于255×224+127时，floor(n/255)等于(n+1+(n>>8))>>8，两次加法均小于65536。
 */
        lo=vaddq_u16(vaddq_u16(lo,one),vshrq_n_u16(lo,8));
        hi=vaddq_u16(vaddq_u16(hi,one),vshrq_n_u16(hi,8));
        const uint8x16_t limited=vaddq_u8(
            vcombine_u8(vshrn_n_u16(lo,8),vshrn_n_u16(hi,8)),offset);
        vst1q_u8(dst+i,limited);
    }
#else
    /* 在主机上模拟NEON所用的同一除255整数运算。 */
    for(;i+16<=count;i+=16)for(size_t lane=0;lane<16;lane++) {
        const unsigned n=(unsigned)src[i+lane]*coefficient+127u;
        dst[i+lane]=(uint8_t)(16u+((n+1u+(n>>8))>>8));
    }
#endif
    for(;i<count;i++)dst[i]=(uint8_t)(16u+((unsigned)src[i]*coefficient+127u)/255u);
}

static inline void isp_range_nv12(uint8_t *dst,const uint8_t *src,size_t y_size) {
    isp_range_u8(dst,src,y_size,219);
    isp_range_u8(dst+y_size,src+y_size,y_size/2,224);
}
#endif
