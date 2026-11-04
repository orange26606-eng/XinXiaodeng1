/* 独立构造期望Y及UV行，检查保护区和填充。 */
#include "camera_swap_nv12.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
static void layout(unsigned w,unsigned h,unsigned stride) {
    size_t rows=(size_t)h*3/2,n=rows*stride,out=rows*w;
    uint8_t *s=malloc(n+16),*d=malloc(out+16),*back=malloc(out+16);
    CHECK(s && d && back);
    memset(s,0xEE,n+16);memset(d,0xAC,out+16);memset(back,0xBD,out+16);
    for(size_t y=0;y<rows;++y) for(unsigned x=0;x<w;++x)
        s[y*stride+x]=(uint8_t)((x<w/2 ? 19 : 137)+(x%(w/2))*7+y*3);
    CHECK(!camera_swap_nv12(d,out,s,n,w,h,stride));
    for(size_t y=0;y<rows;++y)for(unsigned x=0;x<w;++x) {
        unsigned source=x<w/2 ? x+w/2 : x-w/2;
        CHECK(d[y*w+x]==(uint8_t)((source<w/2 ? 19 : 137)+(source%(w/2))*7+y*3));
    }
    for(unsigned i=0;i<16;++i){CHECK(d[out+i]==0xAC);CHECK(s[n+i]==0xEE);}
    CHECK(!camera_swap_nv12(back,out,d,out,w,h,w));
    for(size_t y=0;y<rows;++y) CHECK(!memcmp(back+y*w,s+y*stride,w));
    memset(d,0xAC,out);
    CHECK(camera_swap_nv12(d,out-1,s,n,w,h,stride)==-1);
    CHECK(camera_swap_nv12(d,out,s,n-1,w,h,stride)==-1);
    for(size_t i=0;i<out;++i)CHECK(d[i]==0xAC);
    CHECK(camera_swap_nv12(s,n,s,n,w,h,stride)==-1);
    CHECK(camera_swap_nv12(s+1,n-1,s,n,w,h,stride)==-1);
    free(s);free(d);free(back);
}
int main(void) {
    layout(8,4,8);layout(12,6,16);layout(1920,1080,2048);
    uint8_t src[48]={0},dst[48];memset(dst,0xAB,sizeof(dst));
    CHECK(camera_swap_nv12(dst,48,src,48,6,4,8)==-1);
    CHECK(camera_swap_nv12(dst,48,src,48,8,3,8)==-1);
    CHECK(camera_swap_nv12(dst,48,src,48,8,4,7)==-1);
    CHECK(camera_swap_nv12(dst,48,NULL,48,8,4,8)==-1);
    for(unsigned i=0;i<48;++i)CHECK(dst[i]==0xAB);
    /* 显式UV顺序为U0,V0、U1,V1，再接U2,V2、U3,V3。 */
    uint8_t simple[]={0,1,2,3,4,5,6,7,10,11,12,13,14,15,16,17,20,80,21,81,22,82,23,83};
    uint8_t expected[]={4,5,6,7,0,1,2,3,14,15,16,17,10,11,12,13,22,82,23,83,20,80,21,81};
    CHECK(!camera_swap_nv12(dst,48,simple,sizeof(simple),8,2,8));
    CHECK(!memcmp(dst,expected,sizeof(expected)));
    printf("PASS NV12 camera swap: %u checks\n",checks);return 0;
}
