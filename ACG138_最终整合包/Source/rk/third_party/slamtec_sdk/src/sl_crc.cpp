/* License and attribution: LICENSES/source_notices/notice_031.txt */






 /* License and attribution: LICENSES/source_notices/notice_018.txt */
























#include "sl_crc.h"  

namespace sl {namespace crc32 {
    
    static sl_u32 table[256];//crc32_table
    sl_u32 bitrev(sl_u32 input, sl_u16 bw)
    {
        sl_u16 i;
        sl_u32 var;
        var = 0;
        for (i = 0; i < bw; i++) {
            if (input & 0x01) {
                var |= 1 << (bw - 1 - i);
            }
            input >>= 1;
        }
        return var;
    }

    void init(sl_u32 poly)
    {
        sl_u16 i;
        sl_u16 j;
        sl_u32 c;

        poly = bitrev(poly, 32);
        for (i = 0; i < 256; i++) {
            c = i;
            for (j = 0; j < 8; j++) {
                if (c & 1)
                    c = poly ^ (c >> 1);
                else
                    c = c >> 1;
            }
            table[i] = c;
        }
    }

    sl_u32 cal(sl_u32 crc, void* input, sl_u16 len)
    {
        sl_u16 i;
        sl_u8 index;
        sl_u8* pch;
        pch = (unsigned char*)input;
        sl_u8 leftBytes = 4 - (len & 0x3);

        for (i = 0; i < len; i++) {
            index = (unsigned char)(crc^*pch);
            crc = (crc >> 8) ^ table[index];
            pch++;
        }

        for (i = 0; i < leftBytes; i++) {//zero padding
            index = (unsigned char)(crc ^ 0);
            crc = (crc >> 8) ^ table[index];
        }
        return crc ^ 0xffffffff;
    }

    sl_result getResult(sl_u8 *ptr, sl_u32 len) 
    {
        static sl_u8 tmp;
        if (tmp != 1) {
            init(0x4C11DB7);
            tmp = 1;
        }

        return cal(0xFFFFFFFF, ptr, len);
    }
}}