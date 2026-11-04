#define _GNU_SOURCE
#include "scan_core.h"
#include "lab_screen.h"
#include "panel_outputs.h"
#include "xdg-shell-client-protocol.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <wayland-client.h>
static volatile sig_atomic_t running=1;
static int requested_visible=1;
void lab_wayland_set_visible(int visible){requested_visible=!!visible;}
static void stop(int s){(void)s;running=0;}
static uint64_t now_ms(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000+t.tv_nsec/1000000;}
static struct wl_compositor *compositor;
static struct wl_shm *shm;
static struct xdg_wm_base *shell;
static struct panel_outputs outputs;
static struct wl_seat *seat;
static struct wl_keyboard *keyboard;
static void *key_user;
static struct wl_pointer *pointer;static struct wl_touch *touch;
static double pointer_x,pointer_y;static int pointer_inside,active_touch=-1;
int lab_wayland_pointer_position(int*x,int*y){if(!pointer_inside)return 0;*x=(int)pointer_x;*y=(int)pointer_y;return 1;}
static void (*click_callback)(void *,int,int,uint32_t,uint64_t);
static void pointer_enter(void*d,struct wl_pointer*p,uint32_t serial,struct wl_surface*surface,wl_fixed_t x,wl_fixed_t y){(void)d;(void)p;(void)serial;(void)surface;pointer_inside=1;pointer_x=wl_fixed_to_double(x);pointer_y=wl_fixed_to_double(y);}
static void pointer_leave(void*d,struct wl_pointer*p,uint32_t serial,struct wl_surface*surface){(void)d;(void)p;(void)serial;(void)surface;pointer_inside=0;}
static void pointer_motion(void*d,struct wl_pointer*p,uint32_t time,wl_fixed_t x,wl_fixed_t y){(void)d;(void)p;(void)time;pointer_x=wl_fixed_to_double(x);pointer_y=wl_fixed_to_double(y);}
static void pointer_button(void*d,struct wl_pointer*p,uint32_t serial,uint32_t time,uint32_t button,uint32_t state){(void)d;(void)p;(void)serial;(void)time;if(state==WL_POINTER_BUTTON_STATE_PRESSED&&click_callback)click_callback(key_user,(int)pointer_x,(int)pointer_y,button==0x110?1u:2u,now_ms()*1000);}
static void pointer_axis(void*d,struct wl_pointer*p,uint32_t time,uint32_t axis,wl_fixed_t value){(void)d;(void)p;(void)time;(void)axis;(void)value;}
static void pointer_frame(void*d,struct wl_pointer*p){(void)d;(void)p;}
static void pointer_source(void*d,struct wl_pointer*p,uint32_t source){(void)d;(void)p;(void)source;}
static void pointer_stop(void*d,struct wl_pointer*p,uint32_t time,uint32_t axis){(void)d;(void)p;(void)time;(void)axis;}
static void pointer_discrete(void*d,struct wl_pointer*p,uint32_t axis,int32_t discrete){(void)d;(void)p;(void)axis;(void)discrete;}
static const struct wl_pointer_listener pointer_listener={.enter=pointer_enter,.leave=pointer_leave,.motion=pointer_motion,.button=pointer_button,.axis=pointer_axis,.frame=pointer_frame,.axis_source=pointer_source,.axis_stop=pointer_stop,.axis_discrete=pointer_discrete};
static void touch_down(void*d,struct wl_touch*t,uint32_t serial,uint32_t time,struct wl_surface*s,int32_t id,wl_fixed_t x,wl_fixed_t y){(void)d;(void)t;(void)serial;(void)time;(void)s;if(active_touch==-1){active_touch=id;if(click_callback)click_callback(key_user,wl_fixed_to_int(x),wl_fixed_to_int(y),1u,now_ms()*1000);}}
static void touch_up(void*d,struct wl_touch*t,uint32_t serial,uint32_t time,int32_t id){(void)d;(void)t;(void)serial;(void)time;if(id==active_touch)active_touch=-1;}
static void touch_motion(void*d,struct wl_touch*t,uint32_t time,int32_t id,wl_fixed_t x,wl_fixed_t y){(void)d;(void)t;(void)time;(void)id;(void)x;(void)y;}
static void touch_frame(void*d,struct wl_touch*t){(void)d;(void)t;}
static void touch_cancel(void*d,struct wl_touch*t){(void)d;(void)t;active_touch=-1;}
static const struct wl_touch_listener touch_listener={.down=touch_down,.up=touch_up,.motion=touch_motion,.frame=touch_frame,.cancel=touch_cancel};
static uint32_t seat_id;
static void (*key_callback)(void *,uint32_t,uint64_t);
static int configured,width=1280,height=720;
static void ping(void *d,struct xdg_wm_base *b,uint32_t serial){(void)d;xdg_wm_base_pong(b,serial);}
static const struct xdg_wm_base_listener shell_listener={.ping=ping};
static void keymap(void *d,struct wl_keyboard *k,uint32_t format,int32_t fd,uint32_t size){(void)d;(void)k;(void)format;(void)size;close(fd);}
static void key_enter(void *d,struct wl_keyboard *k,uint32_t serial,struct wl_surface *s,struct wl_array *keys){(void)d;(void)k;(void)serial;(void)s;(void)keys;}
static void key_leave(void *d,struct wl_keyboard *k,uint32_t serial,struct wl_surface *s){(void)d;(void)k;(void)serial;(void)s;}
static void key_event(void *d,struct wl_keyboard *k,uint32_t serial,uint32_t time,uint32_t key,uint32_t state){
 (void)d;(void)k;(void)serial;(void)time;if(state!=WL_KEYBOARD_KEY_STATE_PRESSED)return;
 if(key==1||key==16){running=0;return;}if(key_callback)key_callback(key_user,key,now_ms()*1000);
}
static void key_modifiers(void *d,struct wl_keyboard *k,uint32_t serial,uint32_t depressed,uint32_t latched,uint32_t locked,uint32_t group){(void)d;(void)k;(void)serial;(void)depressed;(void)latched;(void)locked;(void)group;}
static void key_repeat(void *d,struct wl_keyboard *k,int32_t rate,int32_t delay){(void)d;(void)k;(void)rate;(void)delay;}
static const struct wl_keyboard_listener keyboard_listener={.keymap=keymap,.enter=key_enter,.leave=key_leave,.key=key_event,.modifiers=key_modifiers,.repeat_info=key_repeat};
static void seat_capabilities(void *d,struct wl_seat *s,uint32_t caps){
 (void)d;
 if((caps&WL_SEAT_CAPABILITY_POINTER)&&!pointer){pointer=wl_seat_get_pointer(s);wl_pointer_add_listener(pointer,&pointer_listener,NULL);}
 if(!(caps&WL_SEAT_CAPABILITY_POINTER)&&pointer){wl_pointer_release(pointer);pointer=NULL;}
 if((caps&WL_SEAT_CAPABILITY_TOUCH)&&!touch){touch=wl_seat_get_touch(s);wl_touch_add_listener(touch,&touch_listener,NULL);}
 if(!(caps&WL_SEAT_CAPABILITY_TOUCH)&&touch){wl_touch_release(touch);touch=NULL;}
 if((caps&WL_SEAT_CAPABILITY_KEYBOARD)&&!keyboard){keyboard=wl_seat_get_keyboard(s);wl_keyboard_add_listener(keyboard,&keyboard_listener,NULL);}
 else if(!(caps&WL_SEAT_CAPABILITY_KEYBOARD)&&keyboard){wl_keyboard_release(keyboard);keyboard=NULL;}
}
static void seat_name(void *d,struct wl_seat *s,const char *name){(void)d;(void)s;(void)name;}
static const struct wl_seat_listener seat_listener={.capabilities=seat_capabilities,.name=seat_name};
static void global(void *data,struct wl_registry *registry,uint32_t id,const char *interface,uint32_t version){
    (void)data;
    if(!strcmp(interface,"wl_compositor"))compositor=wl_registry_bind(registry,id,&wl_compositor_interface,version<4?version:4);
    else if(!strcmp(interface,"wl_shm"))shm=wl_registry_bind(registry,id,&wl_shm_interface,1);
    else if(!strcmp(interface,"xdg_wm_base")){shell=wl_registry_bind(registry,id,&xdg_wm_base_interface,1);xdg_wm_base_add_listener(shell,&shell_listener,NULL);}
    else if(!strcmp(interface,"wl_seat")&&!seat&&version>=5){seat=wl_registry_bind(registry,id,&wl_seat_interface,5);seat_id=id;wl_seat_add_listener(seat,&seat_listener,NULL);}
    else panel_outputs_global(&outputs,registry,id,interface,version);
}
static void removed(void *d,struct wl_registry *r,uint32_t id){(void)d;(void)r;panel_outputs_remove(&outputs,id);if(id==seat_id){if(pointer){wl_pointer_release(pointer);pointer=NULL;}if(touch){wl_touch_release(touch);touch=NULL;}if(keyboard){wl_keyboard_release(keyboard);keyboard=NULL;}if(seat){wl_seat_release(seat);seat=NULL;}seat_id=0;}}
static const struct wl_registry_listener registry_listener={.global=global,.global_remove=removed};
static void top_configure(void *data,struct xdg_toplevel *top,int32_t w,int32_t h,struct wl_array *states){
    (void)data;(void)top;(void)states;if(w>=640&&w<=3840)width=w;if(h>=480&&h<=2160)height=h;
}
static void top_close(void *d,struct xdg_toplevel *t){(void)d;(void)t;running=0;}
static const struct xdg_toplevel_listener top_listener={.configure=top_configure,.close=top_close};
static void surface_configure(void *d,struct xdg_surface *s,uint32_t serial){(void)d;xdg_surface_ack_configure(s,serial);configured=1;}
static const struct xdg_surface_listener surface_listener={.configure=surface_configure};
struct buffer {struct wl_buffer *wl;uint32_t *pixels;size_t bytes;int busy,w,h;};
static void release(void *d,struct wl_buffer *b){(void)b;((struct buffer*)d)->busy=0;}
static const struct wl_buffer_listener buffer_listener={.release=release};
static void destroy_buffer(struct buffer *b){if(b->wl)wl_buffer_destroy(b->wl);if(b->pixels)munmap(b->pixels,b->bytes);memset(b,0,sizeof(*b));}
static int make_buffer(struct buffer *b,int w,int h){
    size_t bytes=(size_t)w*h*4;int fd=memfd_create("acg138-lidar-localmap",MFD_CLOEXEC);
    if(fd<0)return -1;if(ftruncate(fd,(off_t)bytes)){close(fd);return -1;}
    void *p=mmap(NULL,bytes,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);if(p==MAP_FAILED){close(fd);return -1;}
    struct wl_shm_pool *pool=wl_shm_create_pool(shm,fd,(int32_t)bytes);close(fd);
    if(!pool){munmap(p,bytes);return -1;}
    b->wl=wl_shm_pool_create_buffer(pool,0,w,h,w*4,WL_SHM_FORMAT_XRGB8888);wl_shm_pool_destroy(pool);
    if(!b->wl){munmap(p,bytes);return -1;}b->pixels=p;b->bytes=bytes;b->w=w;b->h=h;
    wl_buffer_add_listener(b->wl,&buffer_listener,b);return 0;
}

int lab_wayland_run(const char *output,void *user,int (*tick)(void *,uint64_t),void (*draw)(void *,uint32_t *,int,int,uint64_t),void (*key)(void *,uint32_t,uint64_t)){
 return lab_wayland_run_input(output,user,tick,draw,key,NULL);
}
int lab_wayland_run_input(const char *output,void *user,int (*tick)(void *,uint64_t),void (*draw)(void *,uint32_t *,int,int,uint64_t),void (*key)(void *,uint32_t,uint64_t),void (*click)(void *,int,int,uint32_t,uint64_t)){
 click_callback=click;
    if(!output||!tick||!draw)return 1;signal(SIGINT,stop);signal(SIGTERM,stop);
    running=1;configured=0;width=1280;height=720;key_callback=key;key_user=user;panel_outputs_init(&outputs,output);
    struct wl_display *display=wl_display_connect(NULL);
    if(!display){fprintf(stderr,"Wayland unavailable; no services started.\n");return 2;}
    struct wl_registry *registry=wl_display_get_registry(display);wl_registry_add_listener(registry,&registry_listener,NULL);
    if(wl_display_roundtrip(display)<0||wl_display_roundtrip(display)<0||wl_display_roundtrip(display)<0){wl_display_disconnect(display);return 2;}
    struct panel_output_selection selected={0};panel_outputs_update(&outputs,&selected);
    if(!compositor||!shm||!shell||(!selected.wl&&!selected.windowed)){
        fprintf(stderr,"Output unavailable: %s\n",output);for(unsigned i=0;i<PANEL_OUTPUT_LIMIT;++i)if(outputs.entries[i].present)fprintf(stderr,"  %s\n",outputs.entries[i].name);
        wl_display_disconnect(display);return 2;
    }
    struct wl_surface *surface=NULL;struct xdg_surface *xdg=NULL;struct xdg_toplevel *top=NULL;
    struct buffer buffers[2]={{0}};uint64_t last_draw=0;int result=0;
    while(running){
        while(wl_display_prepare_read(display)!=0)if(wl_display_dispatch_pending(display)<0){result=2;goto finish;}
        int want_write=0;if(wl_display_flush(display)<0){if(errno==EAGAIN)want_write=1;else{wl_display_cancel_read(display);result=2;break;}}
        struct pollfd fd={.fd=wl_display_get_fd(display),.events=(short)(POLLIN|(want_write?POLLOUT:0))};
        int ready=poll(&fd,1,25);if(ready<0){wl_display_cancel_read(display);if(errno==EINTR)continue;result=2;break;}
        if(fd.revents&POLLIN){if(wl_display_read_events(display)<0){result=2;break;}}else wl_display_cancel_read(display);
        if(fd.revents&(POLLERR|POLLHUP|POLLNVAL)){result=2;break;}
        if(wl_display_dispatch_pending(display)<0){result=2;break;}
        if(panel_outputs_update(&outputs,&selected)&&selected.wl&&surface){xdg_toplevel_set_fullscreen(top,selected.wl);wl_surface_commit(surface);}
        uint64_t now=now_ms();int updated=tick(user,now*1000);if(updated<0){result=2;break;}
        if(!requested_visible&&surface){
            /* 只销毁本应用表面，底层旧相机与IMU客户端保持运行，由合成器恢复显示。
 */
            xdg_toplevel_destroy(top);xdg_surface_destroy(xdg);wl_surface_destroy(surface);
            top=NULL;xdg=NULL;surface=NULL;configured=0;pointer_inside=0;
            fprintf(stderr,"HDMI_VIEW camera_imu (navigation surface hidden)\n");
        }
        if(requested_visible&&!surface){
            configured=0;surface=wl_compositor_create_surface(compositor);xdg=xdg_wm_base_get_xdg_surface(shell,surface);
            xdg_surface_add_listener(xdg,&surface_listener,NULL);top=xdg_surface_get_toplevel(xdg);xdg_toplevel_add_listener(top,&top_listener,NULL);
            xdg_toplevel_set_title(top,"ACG138 Navigation");xdg_toplevel_set_app_id(top,"cn.acg138.navigation");xdg_toplevel_set_min_size(top,640,480);
            if(selected.wl)xdg_toplevel_set_fullscreen(top,selected.wl);wl_surface_commit(surface);wl_display_flush(display);last_draw=0;
            fprintf(stderr,"HDMI_VIEW navigation (real navigation session)\n");
        }
        if(surface&&configured&&now-last_draw>=66){for(int i=0;i<2;++i){struct buffer *b=&buffers[i];if(b->busy)continue;
            if(b->w!=width||b->h!=height){destroy_buffer(b);if(make_buffer(b,width,height)){result=2;goto finish;}}
            draw(user,b->pixels,width,height,now*1000);b->busy=1;wl_surface_attach(surface,b->wl,0,0);wl_surface_damage(surface,0,0,INT_MAX,INT_MAX);wl_surface_commit(surface);last_draw=now;break;
        }}
    }
finish:
    for(int i=0;i<2;++i)destroy_buffer(&buffers[i]);if(top)xdg_toplevel_destroy(top);if(xdg)xdg_surface_destroy(xdg);if(surface)wl_surface_destroy(surface);panel_outputs_finish(&outputs);
    if(pointer){wl_pointer_release(pointer);pointer=NULL;}if(touch){wl_touch_release(touch);touch=NULL;}if(keyboard){wl_keyboard_release(keyboard);keyboard=NULL;}if(seat){wl_seat_release(seat);seat=NULL;}
    xdg_wm_base_destroy(shell);wl_shm_destroy(shm);wl_compositor_destroy(compositor);wl_registry_destroy(registry);wl_display_disconnect(display);
    compositor=NULL;shm=NULL;shell=NULL;return result;
}
