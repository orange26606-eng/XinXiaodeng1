/* Generated from the packaged upstream Wayland XML. */
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <wayland-client.h>
struct wl_output;
extern const struct wl_interface wl_output_interface;
struct zxdg_output_manager_v1;
extern const struct wl_interface zxdg_output_manager_v1_interface;
struct zxdg_output_v1;
extern const struct wl_interface zxdg_output_v1_interface;
#define ZXDG_OUTPUT_MANAGER_V1_INTERFACE_VERSION 3
static inline void zxdg_output_manager_v1_set_user_data(struct zxdg_output_manager_v1 *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *zxdg_output_manager_v1_get_user_data(struct zxdg_output_manager_v1 *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define ZXDG_OUTPUT_MANAGER_V1_DESTROY_SINCE_VERSION 1
#define ZXDG_OUTPUT_MANAGER_V1_GET_XDG_OUTPUT_SINCE_VERSION 1
static inline void zxdg_output_manager_v1_destroy(struct zxdg_output_manager_v1 *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline struct zxdg_output_v1 * zxdg_output_manager_v1_get_xdg_output(struct zxdg_output_manager_v1 *o, struct wl_output * output){
return (struct zxdg_output_v1 *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&zxdg_output_v1_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, (struct wl_proxy*)output);
}
#define ZXDG_OUTPUT_V1_INTERFACE_VERSION 3
struct zxdg_output_v1_listener {
 void (*logical_position)(void *data, struct zxdg_output_v1 *object, int32_t x, int32_t y);
 void (*logical_size)(void *data, struct zxdg_output_v1 *object, int32_t width, int32_t height);
 void (*done)(void *data, struct zxdg_output_v1 *object);
 void (*name)(void *data, struct zxdg_output_v1 *object, const char * name);
 void (*description)(void *data, struct zxdg_output_v1 *object, const char * description);
};
static inline int zxdg_output_v1_add_listener(struct zxdg_output_v1 *o,const struct zxdg_output_v1_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void zxdg_output_v1_set_user_data(struct zxdg_output_v1 *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *zxdg_output_v1_get_user_data(struct zxdg_output_v1 *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define ZXDG_OUTPUT_V1_DESTROY_SINCE_VERSION 1
#define ZXDG_OUTPUT_V1_LOGICAL_POSITION_SINCE_VERSION 1
#define ZXDG_OUTPUT_V1_LOGICAL_SIZE_SINCE_VERSION 1
#define ZXDG_OUTPUT_V1_DONE_SINCE_VERSION 1
#define ZXDG_OUTPUT_V1_NAME_SINCE_VERSION 2
#define ZXDG_OUTPUT_V1_DESCRIPTION_SINCE_VERSION 2
static inline void zxdg_output_v1_destroy(struct zxdg_output_v1 *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
