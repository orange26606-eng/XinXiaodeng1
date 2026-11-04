#define _POSIX_C_SOURCE 200809L
#include "lab_real_session.h"
#include "lab_real_screen.h"
#include "lab_screen.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
typedef struct real_panel {lab_real_session session;uint64_t start,log_time,last_tick;int done;} real_panel;
static int tick(void *u,uint64_t now){real_panel *p=u;if(!p->start)p->start=now;if(now-p->start>=p->session.options.duration_us){p->done=1;return -1;}
 if(p->last_tick&&now==p->last_tick)return 0;p->last_tick=now;
 lab_status s=lab_real_session_tick(&p->session,now);if(!p->log_time||now-p->log_time>=1000000){lab_real_print_view(&p->session.view);p->log_time=now;}if(s<0&&s!=LAB_ERR_UNAVAILABLE){fprintf(stderr,"PANEL_TICK_FATAL status=%d reason=%s\n",(int)s,p->session.view.reason);return -1;}return 0;}
static void draw(void *u,uint32_t *pixels,int w,int h,uint64_t now){(void)now;real_panel *p=u;lab_real_screen_draw(pixels,w,h,&p->session.view);}
static void key(void *u,uint32_t code,uint64_t now){real_panel *p=u;if(code==46)lab_real_runtime_cancel(p->session.runtime,now);}
int main(int argc,char **argv){if(argc==2&&!strcmp(argv[1],"--help")){lab_real_usage();return 0;}
 if(argc==2&&!strcmp(argv[1],"--info")){puts("RK modules  integrated readonly/map/fixed-localization/shadow/mapping-shadow panel; output DISABLED; board untested");return 0;}
 lab_real_options o;char error[256];if(!lab_real_options_parse(&o,argc,argv,error,sizeof(error))){fprintf(stderr,"%s\n",error);return 2;}
 if(!o.run){puts("PREFLIGHT: no Wayland/source/socket/device opened; use explicit --run for future board check.");return 0;}
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);uint64_t stamp=(uint64_t)t.tv_sec*1000000+(uint64_t)t.tv_nsec/1000;
 lab_real_context ctx={LAB_REAL_HARDWARE,LAB_REAL_MONOTONIC,(uint32_t)(stamp^(stamp>>32))|1u,1};real_panel p={0};lab_status s=lab_real_session_open(&p.session,&o,&ctx,NULL);if(s!=LAB_OK){fprintf(stderr,"SOURCE_OR_MODULE_OPEN_FAILED: %s\n",lab_status_text(s));return 3;}
 int result=lab_wayland_run(o.output,&p,tick,draw,key);if(p.done)result=0;
 if(o.save_map[0]&&(s=lab_real_session_save(&p.session))!=LAB_OK){fprintf(stderr,"MAP_NOT_SAVED: %s\n",lab_status_text(s));result=6;}
 if(o.preview[0]&&!lab_real_write_preview(o.preview,&p.session.view))result=4;
 lab_real_session_close(&p.session);return result;
}
