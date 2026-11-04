/* Generated from the packaged upstream Wayland XML. */
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <wayland-client.h>
struct wl_surface;
extern const struct wl_interface wl_surface_interface;
struct wp_viewport;
extern const struct wl_interface wp_viewport_interface;
struct wp_viewporter;
extern const struct wl_interface wp_viewporter_interface;
#define WP_VIEWPORTER_INTERFACE_VERSION 1
enum wp_viewporter_error {
WP_VIEWPORTER_ERROR_VIEWPORT_EXISTS = 0,
};
static inline void wp_viewporter_set_user_data(struct wp_viewporter *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wp_viewporter_get_user_data(struct wp_viewporter *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WP_VIEWPORTER_DESTROY_SINCE_VERSION 1
#define WP_VIEWPORTER_GET_VIEWPORT_SINCE_VERSION 1
static inline void wp_viewporter_destroy(struct wp_viewporter *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline struct wp_viewport * wp_viewporter_get_viewport(struct wp_viewporter *o, struct wl_surface * surface){
return (struct wp_viewport *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&wp_viewport_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, (struct wl_proxy*)surface);
}
#define WP_VIEWPORT_INTERFACE_VERSION 1
enum wp_viewport_error {
WP_VIEWPORT_ERROR_BAD_VALUE = 0,
WP_VIEWPORT_ERROR_BAD_SIZE = 1,
WP_VIEWPORT_ERROR_OUT_OF_BUFFER = 2,
WP_VIEWPORT_ERROR_NO_SURFACE = 3,
};
static inline void wp_viewport_set_user_data(struct wp_viewport *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wp_viewport_get_user_data(struct wp_viewport *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WP_VIEWPORT_DESTROY_SINCE_VERSION 1
#define WP_VIEWPORT_SET_SOURCE_SINCE_VERSION 1
#define WP_VIEWPORT_SET_DESTINATION_SINCE_VERSION 1
static inline void wp_viewport_destroy(struct wp_viewport *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void wp_viewport_set_source(struct wp_viewport *o, wl_fixed_t x, wl_fixed_t y, wl_fixed_t width, wl_fixed_t height){
wl_proxy_marshal((struct wl_proxy*)o,1,x, y, width, height);
}
static inline void wp_viewport_set_destination(struct wp_viewport *o, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,2,width, height);
}
