#ifndef ACG_LAB_SCREEN_H
#define ACG_LAB_SCREEN_H
#include "lab_router.h"
#include "lab_mapping.h"
#include "lab_autonomy.h"
typedef struct lab_screen_state {
 const lab_router *router;
 const lab_map_view *map;
 const lab_mapping_stats *mapping_stats;
 uint64_t now_us,timeline_now_us;
 double scan_hz,replay_speed;
 uint32_t accumulated,paused,recording,contract_preview,available_modules;
 const char *reason;
} lab_screen_state;
void lab_screen_draw(uint32_t *pixels,int width,int height,const lab_screen_state *state);
void lab_auto_screen_draw(uint32_t *pixels,int width,int height,const lab_auto_view *view,uint64_t time_us,int paused);
int lab_wayland_run(const char *output,void *user,int (*tick)(void *,uint64_t),
 void (*draw)(void *,uint32_t *,int,int,uint64_t),void (*key)(void *,uint32_t,uint64_t));
void lab_wayland_set_visible(int visible);
int lab_wayland_pointer_position(int *x,int *y);
int lab_wayland_run_input(const char *output,void *user,int (*tick)(void *,uint64_t),
 void (*draw)(void *,uint32_t *,int,int,uint64_t),void (*key)(void *,uint32_t,uint64_t),
 void (*click)(void *,int,int,uint32_t,uint64_t));
#endif
