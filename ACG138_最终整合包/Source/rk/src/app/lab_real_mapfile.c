#include "lab_real_mapfile.h"
#include "lab_codec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static void put32(uint8_t *p,uint32_t n){for(int i=3;i>=0;--i){p[i]=(uint8_t)n;n>>=8;}}
static uint32_t get32(const uint8_t *p){return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];}
static void put64(uint8_t *p,uint64_t n){put32(p,(uint32_t)(n>>32));put32(p+4,(uint32_t)n);}
static uint64_t get64(const uint8_t *p){return ((uint64_t)get32(p)<<32)|get32(p+4);}
static void putf(uint8_t *p,double x){uint64_t n;memcpy(&n,&x,8);put64(p,n);}
static double getf(const uint8_t *p){uint64_t n=get64(p);double x;memcpy(&x,&n,8);return x;}
void lab_real_map_file_close(lab_real_map_file *f){if(f){free(f->owned_cells);memset(f,0,sizeof(*f));}}
lab_status lab_real_map_save(const char *path,const lab_real_grid_view *m,const lab_localization_config *c){return lab_real_map_save_ex(path,m,c,0);}
lab_status lab_real_map_save_ex(const char *path,const lab_real_grid_view *m,const lab_localization_config *c,uint32_t experimental){
 if(experimental>1||!path||!c||!lab_real_grid_valid(m)||!c->parameter_version)return LAB_ERR_ARGUMENT;
 uint8_t h[160]={0};memcpy(h,"ACGRMAP1",8);put32(h+8,experimental?2:1);put32(h+132,experimental);put32(h+12,sizeof(h));put32(h+16,m->width);put32(h+20,m->height);
 putf(h+24,m->resolution_m);putf(h+32,m->origin_x_m);putf(h+40,m->origin_y_m);put64(h+48,m->map_id);put64(h+56,m->map_version);put64(h+64,m->correction_version);
 put32(h+72,m->context.source);put32(h+76,m->context.clock);put32(h+80,m->context.timeline_id);put32(h+84,m->context.origin_epoch);put64(h+88,c->parameter_version);
 putf(h+96,c->mount_x_m);putf(h+104,c->mount_y_m);putf(h+112,c->mount_yaw_rad);size_t n=(size_t)m->width*m->height;put64(h+120,n);put32(h+128,lab_crc32(m->cells,n));put32(h+156,lab_crc32(h,156));
 FILE *f=fopen(path,"wbx");if(!f)return LAB_ERR_IO;int ok=fwrite(h,1,sizeof(h),f)==sizeof(h)&&fwrite(m->cells,1,n,f)==n;if(fclose(f))ok=0;return ok?LAB_OK:LAB_ERR_IO;
}
lab_status lab_real_map_load(const char *path,const lab_real_context *ctx,const lab_localization_config *c,lab_real_map_file *out){
 if(!path||!c||!out||!lab_real_context_valid(ctx))return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));FILE *f=fopen(path,"rb");if(!f)return LAB_ERR_IO;
 uint8_t h[160];lab_status s=LAB_ERR_TRUNCATED;int8_t *cells=NULL;
 if(fread(h,1,sizeof(h),f)!=sizeof(h))goto done;s=LAB_ERR_FORMAT;if(memcmp(h,"ACGRMAP1",8)||(get32(h+8)!=1&&get32(h+8)!=2)||get32(h+12)!=sizeof(h))goto done;
 s=LAB_ERR_CRC;if(lab_crc32(h,156)!=get32(h+156))goto done;s=LAB_ERR_FORMAT;uint32_t experimental=get32(h+132);if(experimental>1||(get32(h+8)==1&&experimental)||(get32(h+8)==2&&!experimental))goto done;for(unsigned i=136;i<156;++i)if(h[i])goto done;
 lab_real_grid_view m={.context=*ctx,.width=get32(h+16),.height=get32(h+20),.resolution_m=getf(h+24),.origin_x_m=getf(h+32),.origin_y_m=getf(h+40),.map_id=get64(h+48),.map_version=get64(h+56),.correction_version=get64(h+64),.readonly=1};
 if(!m.width||!m.height||m.width>LAB_MAP_SIDE_MAX||m.height>LAB_MAP_SIDE_MAX||get64(h+120)!=(uint64_t)m.width*m.height)goto done;
 lab_real_context original={get32(h+72),get32(h+76),get32(h+80),get32(h+84)};if(!lab_real_context_valid(&original))goto done;
 /* 导入后保留来源，测试地图不能变成实测地图。 */
 s=LAB_ERR_CONTEXT;if(ctx->source==LAB_REAL_HARDWARE&&original.source!=LAB_REAL_HARDWARE)goto done;
 s=LAB_ERR_VERSION;if(get64(h+88)!=c->parameter_version)goto done;
 s=LAB_ERR_ARGUMENT;double mx=getf(h+96),my=getf(h+104),ma=getf(h+112);if(!isfinite(mx)||!isfinite(my)||!isfinite(ma)||fabs(mx-c->mount_x_m)>1e-12||fabs(my-c->mount_y_m)>1e-12||fabs(ma-c->mount_yaw_rad)>1e-12)goto done;
 size_t n=(size_t)m.width*m.height;cells=malloc(n);s=LAB_ERR_NOMEM;if(!cells)goto done;s=LAB_ERR_TRUNCATED;if(fread(cells,1,n,f)!=n)goto done;s=LAB_ERR_FORMAT;if(fgetc(f)!=EOF||ferror(f))goto done;
 s=LAB_ERR_CRC;if(lab_crc32(cells,n)!=get32(h+128))goto done;m.cells=cells;s=LAB_ERR_FORMAT;if(!lab_real_grid_valid(&m))goto done;
 out->experimental_timing=experimental;out->view=m;out->owned_cells=cells;out->original_context=original;out->parameter_version=get64(h+88);out->mount_x_m=mx;out->mount_y_m=my;out->mount_yaw_rad=ma;cells=NULL;s=LAB_OK;
done:free(cells);fclose(f);return s;
}
