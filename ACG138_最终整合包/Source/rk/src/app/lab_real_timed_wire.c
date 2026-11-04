#include "lab_real_mpu.h"
#include "lab_codec.h"
#include <string.h>
static uint32_t be32(const uint8_t *p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static void put32(uint8_t *p,uint32_t n){for(int i=3;i>=0;--i){p[i]=(uint8_t)n;n>>=8;}}
static void put64(uint8_t *p,uint64_t n){put32(p,(uint32_t)(n>>32));put32(p+4,(uint32_t)n);}
size_t lab_real_timed_encode(uint8_t *out,size_t cap,const uint8_t *p,size_t bytes,uint64_t first,uint64_t returned,uint32_t span){
 if(!out||!p||bytes<SCAN_HEADER||bytes>SCAN_PACKET_MAX||cap<64+bytes||memcmp(p,"LS01",4)||!span||span>250000||first>returned||span>returned-first)return 0;
 memset(out,0,64);memcpy(out,"LT01",4);out[4]=1;out[5]=3;out[7]=64;put32(out+8,be32(p+8));put32(out+12,be32(p+12));put64(out+16,first);put64(out+24,returned);put32(out+32,span);put32(out+40,(uint32_t)bytes);memcpy(out+64,p,bytes);uint32_t crc=lab_crc32(out,60);put32(out+60,lab_crc32_extend(crc,p,bytes));return 64+bytes;
}
