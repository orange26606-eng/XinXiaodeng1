#define _GNU_SOURCE
#include "lab_real_mpusvc.h"
#include "lab_codec.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#ifndef _WIN32
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#else
#include <process.h>
#endif
static FILE *read_regular(const char *path){
#ifndef _WIN32
 int fd=open(path,O_RDONLY|O_CLOEXEC|O_NOFOLLOW|O_NONBLOCK);if(fd<0)return NULL;struct stat st;if(fstat(fd,&st)||!S_ISREG(st.st_mode)){close(fd);return NULL;}FILE *f=fdopen(fd,"rb");if(!f)close(fd);else setvbuf(f,NULL,_IONBF,0);return f;
#else
 FILE *f=fopen(path,"rb");if(f)setvbuf(f,NULL,_IONBF,0);return f;
#endif
}
lab_status mpusvc_read(const char *path,mpusvc_view *out){
 if(!path||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));FILE *f=read_regular(path);if(!f)return LAB_ERR_IO;uint8_t b[768];lab_status s=LAB_ERR_TRUNCATED;if(fread(b,1,768,f)==768){s=fgetc(f)==EOF&&!ferror(f)?mpusvc_decode(out,b,768):LAB_ERR_FORMAT;}fclose(f);return s;
}
lab_status mpusvc_legacy_read(const char *path,struct odom_shared *out){
 if(!path||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));FILE *f=read_regular(path);if(!f)return LAB_ERR_IO;
 if(fseek(f,0,SEEK_END)||ftell(f)!=(long)sizeof(*out)){fclose(f);return LAB_ERR_VERSION;}lab_status s=LAB_AGAIN;
 for(unsigned i=0;i<5;++i){uint32_t a,b;struct odom_shared v;clearerr(f);
  if(fseek(f,(long)offsetof(struct odom_shared,sequence),SEEK_SET)||fread(&a,1,4,f)!=4){s=LAB_ERR_TRUNCATED;break;}if(a&1)continue;
  if(fseek(f,0,SEEK_SET)||fread(&v,1,sizeof(v),f)!=sizeof(v)||fseek(f,(long)offsetof(struct odom_shared,sequence),SEEK_SET)||fread(&b,1,4,f)!=4){s=LAB_ERR_TRUNCATED;break;}
  if(a!=b||a!=v.sequence||(b&1))continue;if(v.magic!=ODOM_MAGIC||v.version!=1||v.bytes!=sizeof(v)){s=LAB_ERR_VERSION;break;}
  const struct odom_state *p=&v.state;if(p->quality>ODOM_SLIP||p->points>ODOM_POINTS||p->calibrated>1||p->imu_used>1||p->slip>1||!isfinite(p->x)||!isfinite(p->y)||!isfinite(p->heading)||!isfinite(p->total_distance)||p->total_distance<0||!isfinite(p->uncertainty_m)||p->uncertainty_m<0){s=LAB_ERR_FORMAT;break;}
  *out=v;s=LAB_OK;break;
 }fclose(f);return s;
}
lab_status mpusvc_write_new(const char *path,const void *data,size_t n){if(!path||!data)return LAB_ERR_ARGUMENT;FILE *f=fopen(path,"wbx");if(!f)return LAB_ERR_IO;int ok=fwrite(data,1,n,f)==n;if(fclose(f))ok=0;return ok?LAB_OK:LAB_ERR_IO;}

static uint32_t hist_r32(const uint8_t*p){return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3];}
static uint64_t hist_r64(const uint8_t*p){return (uint64_t)hist_r32(p)<<32|hist_r32(p+4);}
static void hist_u32(uint8_t*p,uint32_t v){p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v;}
static void hist_u64(uint8_t*p,uint64_t v){hist_u32(p,(uint32_t)(v>>32));hist_u32(p+4,(uint32_t)v);}
static void history_header(uint8_t b[64],const mpusvc_history_writer*w){memset(b,0,64);memcpy(b,"MPUSHIS1",8);hist_u32(b+8,1);hist_u32(b+12,64);hist_u32(b+16,768);hist_u32(b+20,MPUSVC_HISTORY_CAPACITY);hist_u64(b+24,w->instance);hist_u64(b+32,w->first);hist_u64(b+40,w->last);hist_u32(b+48,w->count);hist_u32(b+52,w->config_crc);hist_u32(b+56,w->session);hist_u32(b+60,lab_crc32(b,60));}
lab_status mpusvc_history_create(mpusvc_history_writer*w,const char*path){
 if(!w||!path||strlen(path)>500)return LAB_ERR_ARGUMENT;memset(w,0,sizeof(*w));char temp[544];
#ifdef _WIN32
 unsigned long pid=(unsigned long)_getpid();
#else
 unsigned long pid=(unsigned long)getpid();
#endif
 if(snprintf(temp,sizeof(temp),"%s.new.%lu",path,pid)>=(int)sizeof(temp))return LAB_ERR_LIMIT;
 FILE*f=fopen(temp,"wbx");if(!f)return LAB_ERR_IO;uint8_t h[64];history_header(h,w);int bad=fwrite(h,1,64,f)!=64||fseek(f,(long)(64+MPUSVC_HISTORY_CAPACITY*768-1),SEEK_SET)||fputc(0,f)==EOF||fflush(f);if(fclose(f))bad=1;if(bad||rename(temp,path)){remove(temp);return LAB_ERR_IO;}f=fopen(path,"r+b");if(!f)return LAB_ERR_IO;setvbuf(f,NULL,_IONBF,0);w->file=f;return LAB_OK;
}
lab_status mpusvc_history_append(mpusvc_history_writer*w,const mpusvc_view*v){
 if(!w||!w->file||!v||!v->export_sequence)return LAB_ERR_ARGUMENT;uint8_t b[768],h[64];lab_status r=mpusvc_encode_timed(b,v);if(r!=LAB_OK)return r;
 if(w->count&&(v->instance!=w->instance||v->export_sequence!=w->last+1))return LAB_ERR_CONTEXT;
 mpusvc_history_writer next=*w;if(!next.count||next.config_crc!=v->config_crc||next.session!=v->session||next.parameter_version!=v->parameter_version){next.count=0;next.first=v->export_sequence;}
 next.instance=v->instance;next.parameter_version=v->parameter_version;next.config_crc=v->config_crc;next.session=v->session;next.last=v->export_sequence;if(next.count<MPUSVC_HISTORY_CAPACITY)++next.count;else ++next.first;
 history_header(h,&next);FILE*f=w->file;long offset=64+(long)((v->export_sequence-1)%MPUSVC_HISTORY_CAPACITY)*768;
 /* 先写记录行，再提交文件头。读取端核对两次文件头及行序号、CRC；若读到覆盖中的槽位则重试。 */
 if(fseek(f,offset,SEEK_SET)||fwrite(b,1,768,f)!=768||fflush(f)||fseek(f,0,SEEK_SET)||fwrite(h,1,64,f)!=64||fflush(f))return LAB_ERR_IO;*w=next;return LAB_OK;
}
void mpusvc_history_close(mpusvc_history_writer*w){if(w&&w->file){fclose(w->file);w->file=NULL;}}
lab_status mpusvc_history_next(const char*path,mpusvc_history_cursor*c,uint8_t bytes[768],mpusvc_view*v){
 if(!path||!c||!bytes||!v)return LAB_ERR_ARGUMENT;FILE*f=read_regular(path);if(!f)return LAB_ERR_IO;uint8_t a[64],b[64];lab_status r=LAB_AGAIN;
 if(fseek(f,0,SEEK_END)||ftell(f)!=(long)(64+MPUSVC_HISTORY_CAPACITY*768)||fseek(f,0,SEEK_SET)){r=LAB_ERR_FORMAT;goto done;}
 if(fread(a,1,64,f)!=64){r=LAB_ERR_TRUNCATED;goto done;}
 if(memcmp(a,"MPUSHIS1",8)||hist_r32(a+8)!=1||hist_r32(a+12)!=64||hist_r32(a+16)!=768||hist_r32(a+20)!=MPUSVC_HISTORY_CAPACITY){r=LAB_ERR_VERSION;goto done;}
 if(lab_crc32(a,60)!=hist_r32(a+60)){r=LAB_ERR_CRC;goto done;}
 uint64_t instance=hist_r64(a+24),first=hist_r64(a+32),last=hist_r64(a+40);uint32_t count=hist_r32(a+48);
 if(!count)goto done;if(count>MPUSVC_HISTORY_CAPACITY||!instance||!first||last<first||last-first+1!=count){r=LAB_ERR_FORMAT;goto done;}
 if(c->rows&&(c->config_crc!=hist_r32(a+52)||c->session!=hist_r32(a+56))){r=LAB_ERR_CONTEXT;goto done;}
 if(c->instance&&c->instance!=instance){r=LAB_ERR_CONTEXT;goto done;}if(c->sequence>last){r=LAB_ERR_TIME;goto done;}
 uint64_t wanted=c->sequence?c->sequence+1:first;if(wanted>last)goto done;uint64_t lost=0;if(wanted<first){lost=first-wanted;wanted=first;}
 if(fseek(f,64+(long)((wanted-1)%MPUSVC_HISTORY_CAPACITY)*768,SEEK_SET)||fread(bytes,1,768,f)!=768||fseek(f,0,SEEK_SET)||fread(b,1,64,f)!=64){r=LAB_ERR_TRUNCATED;goto done;}
 if(memcmp(a,b,64))goto done;r=mpusvc_decode(v,bytes,768);if(r!=LAB_OK){if(r==LAB_ERR_CRC||r==LAB_ERR_FORMAT||r==LAB_ERR_VERSION){c->instance=instance;c->sequence=wanted;c->overwritten+=lost;++c->rejected_rows;}goto done;}
 if(v->instance!=instance||v->export_sequence!=wanted){r=LAB_AGAIN;goto done;}
 if(v->config_crc!=hist_r32(a+52)||v->session!=hist_r32(a+56)||(c->rows&&v->parameter_version!=c->parameter_version)){r=LAB_ERR_CONTEXT;goto done;}
 if(c->header_last&&last>c->header_last+1)c->latest_version_jumps+=last-c->header_last-1;
 c->config_crc=v->config_crc;c->session=v->session;c->parameter_version=v->parameter_version;c->header_last=last;c->instance=instance;c->sequence=wanted;c->overwritten+=lost;++c->rows;r=LAB_OK;
 done:fclose(f);return r;
}
