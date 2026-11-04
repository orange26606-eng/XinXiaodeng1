/* 使用相同实际NEON核心，不依赖Linux或NPU。 */
#include "neon_yuv8.h"
unsigned neon_probe(void) { return ACG_HAS_AARCH64_NEON; }
void neon_batch(const unsigned char *source,unsigned char *rgb,unsigned blocks) {
    for(unsigned i=0;i<blocks;i++)acg_neon_yuv8(source+i*24,source+i*24+8,source+i*24+16,rgb+i*24);
}
