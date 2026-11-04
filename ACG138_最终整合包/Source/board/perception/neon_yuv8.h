#ifndef ACG_NEON_YUV8_H
#define ACG_NEON_YUV8_H
/* 精确的BT.601有限范围整数计算，纯C头文件支持独立交叉编译及离线指令仿真。
 */
#include <stdint.h>
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#define ACG_HAS_AARCH64_NEON 1
#else
#define ACG_HAS_AARCH64_NEON 0
#endif

static inline uint8_t acg_channel_scalar(int value) {
    /* 避免依赖负数有符号右移的实现差异，只对舍入后为正的值做除法，负值截为0。
 */
    value += 128;
    if (value <= 0) return 0;
    value /= 256;
    return (uint8_t)(value > 255 ? 255 : value);
}
static inline void acg_yuv8_scalar(const uint8_t *y, const uint8_t *u,
                                   const uint8_t *v, uint8_t *rgb) {
    for (unsigned i=0;i<8;i++) {
        int c=y[i]>16?(int)y[i]-16:0,du=(int)u[i]-128,dv=(int)v[i]-128;
        rgb[i*3]=acg_channel_scalar(298*c+409*dv);
        rgb[i*3+1]=acg_channel_scalar(298*c-100*du-208*dv);
        rgb[i*3+2]=acg_channel_scalar(298*c+516*du);
    }
}
#if ACG_HAS_AARCH64_NEON
static inline uint8x8_t acg_round_clip8(int32x4_t lo,int32x4_t hi) {
    /* SQRSHRUN在算术右移8位前加128，并将负值饱和为0；UQXTN完成255上限裁剪，累加值使用int32。
 */
    return vqmovn_u16(vcombine_u16(vqrshrun_n_s32(lo,8),vqrshrun_n_s32(hi,8)));
}
#endif
static inline void acg_neon_yuv8(const uint8_t *y,const uint8_t *u,
                                const uint8_t *v,uint8_t *rgb) {
#if ACG_HAS_AARCH64_NEON
    int16x8_t c=vreinterpretq_s16_u16(vmovl_u8(vqsub_u8(vld1_u8(y),vdup_n_u8(16))));
    int16x8_t du=vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(vld1_u8(u))),vdupq_n_s16(128));
    int16x8_t dv=vsubq_s16(vreinterpretq_s16_u16(vmovl_u8(vld1_u8(v))),vdupq_n_s16(128));
    int32x4_t lo=vmull_n_s16(vget_low_s16(c),298),hi=vmull_n_s16(vget_high_s16(c),298);
    int32x4_t rlo=vmlal_n_s16(lo,vget_low_s16(dv),409),rhi=vmlal_n_s16(hi,vget_high_s16(dv),409);
    int32x4_t glo=vmlal_n_s16(vmlal_n_s16(lo,vget_low_s16(du),-100),vget_low_s16(dv),-208);
    int32x4_t ghi=vmlal_n_s16(vmlal_n_s16(hi,vget_high_s16(du),-100),vget_high_s16(dv),-208);
    int32x4_t blo=vmlal_n_s16(lo,vget_low_s16(du),516),bhi=vmlal_n_s16(hi,vget_high_s16(du),516);
    uint8x8x3_t out;out.val[0]=acg_round_clip8(rlo,rhi);out.val[1]=acg_round_clip8(glo,ghi);out.val[2]=acg_round_clip8(blo,bhi);
    vst3_u8(rgb,out);
#else
    acg_yuv8_scalar(y,u,v,rgb);
#endif
}
#endif
