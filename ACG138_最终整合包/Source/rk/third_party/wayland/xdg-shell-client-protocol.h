/* Generated from the packaged upstream Wayland XML. */
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <wayland-client.h>
struct wl_output;
extern const struct wl_interface wl_output_interface;
struct wl_seat;
extern const struct wl_interface wl_seat_interface;
struct wl_surface;
extern const struct wl_interface wl_surface_interface;
struct xdg_popup;
extern const struct wl_interface xdg_popup_interface;
struct xdg_positioner;
extern const struct wl_interface xdg_positioner_interface;
struct xdg_surface;
extern const struct wl_interface xdg_surface_interface;
struct xdg_toplevel;
extern const struct wl_interface xdg_toplevel_interface;
struct xdg_wm_base;
extern const struct wl_interface xdg_wm_base_interface;
#define XDG_WM_BASE_INTERFACE_VERSION 7
enum xdg_wm_base_error {
XDG_WM_BASE_ERROR_ROLE = 0,
XDG_WM_BASE_ERROR_DEFUNCT_SURFACES = 1,
XDG_WM_BASE_ERROR_NOT_THE_TOPMOST_POPUP = 2,
XDG_WM_BASE_ERROR_INVALID_POPUP_PARENT = 3,
XDG_WM_BASE_ERROR_INVALID_SURFACE_STATE = 4,
XDG_WM_BASE_ERROR_INVALID_POSITIONER = 5,
XDG_WM_BASE_ERROR_UNRESPONSIVE = 6,
};
struct xdg_wm_base_listener {
 void (*ping)(void *data, struct xdg_wm_base *object, uint32_t serial);
};
static inline int xdg_wm_base_add_listener(struct xdg_wm_base *o,const struct xdg_wm_base_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void xdg_wm_base_set_user_data(struct xdg_wm_base *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *xdg_wm_base_get_user_data(struct xdg_wm_base *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define XDG_WM_BASE_DESTROY_SINCE_VERSION 1
#define XDG_WM_BASE_CREATE_POSITIONER_SINCE_VERSION 1
#define XDG_WM_BASE_GET_XDG_SURFACE_SINCE_VERSION 1
#define XDG_WM_BASE_PONG_SINCE_VERSION 1
#define XDG_WM_BASE_PING_SINCE_VERSION 1
static inline void xdg_wm_base_destroy(struct xdg_wm_base *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline struct xdg_positioner * xdg_wm_base_create_positioner(struct xdg_wm_base *o){
return (struct xdg_positioner *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&xdg_positioner_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline struct xdg_surface * xdg_wm_base_get_xdg_surface(struct xdg_wm_base *o, struct wl_surface * surface){
return (struct xdg_surface *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,2,&xdg_surface_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, (struct wl_proxy*)surface);
}
static inline void xdg_wm_base_pong(struct xdg_wm_base *o, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,3,serial);
}
#define XDG_POSITIONER_INTERFACE_VERSION 7
enum xdg_positioner_error {
XDG_POSITIONER_ERROR_INVALID_INPUT = 0,
};
enum xdg_positioner_anchor {
XDG_POSITIONER_ANCHOR_NONE = 0,
XDG_POSITIONER_ANCHOR_TOP = 1,
XDG_POSITIONER_ANCHOR_BOTTOM = 2,
XDG_POSITIONER_ANCHOR_LEFT = 3,
XDG_POSITIONER_ANCHOR_RIGHT = 4,
XDG_POSITIONER_ANCHOR_TOP_LEFT = 5,
XDG_POSITIONER_ANCHOR_BOTTOM_LEFT = 6,
XDG_POSITIONER_ANCHOR_TOP_RIGHT = 7,
XDG_POSITIONER_ANCHOR_BOTTOM_RIGHT = 8,
};
enum xdg_positioner_gravity {
XDG_POSITIONER_GRAVITY_NONE = 0,
XDG_POSITIONER_GRAVITY_TOP = 1,
XDG_POSITIONER_GRAVITY_BOTTOM = 2,
XDG_POSITIONER_GRAVITY_LEFT = 3,
XDG_POSITIONER_GRAVITY_RIGHT = 4,
XDG_POSITIONER_GRAVITY_TOP_LEFT = 5,
XDG_POSITIONER_GRAVITY_BOTTOM_LEFT = 6,
XDG_POSITIONER_GRAVITY_TOP_RIGHT = 7,
XDG_POSITIONER_GRAVITY_BOTTOM_RIGHT = 8,
};
enum xdg_positioner_constraint_adjustment {
XDG_POSITIONER_CONSTRAINT_ADJUSTMENT_NONE = 0,
XDG_POSITIONER_CONSTRAINT_ADJUSTMENT_SLIDE_X = 1,
XDG_POSITIONER_CONSTRAINT_ADJUSTMENT_SLIDE_Y = 2,
XDG_POSITIONER_CONSTRAINT_ADJUSTMENT_FLIP_X = 4,
XDG_POSITIONER_CONSTRAINT_ADJUSTMENT_FLIP_Y = 8,
XDG_POSITIONER_CONSTRAINT_ADJUSTMENT_RESIZE_X = 16,
XDG_POSITIONER_CONSTRAINT_ADJUSTMENT_RESIZE_Y = 32,
};
static inline void xdg_positioner_set_user_data(struct xdg_positioner *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *xdg_positioner_get_user_data(struct xdg_positioner *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define XDG_POSITIONER_DESTROY_SINCE_VERSION 1
#define XDG_POSITIONER_SET_SIZE_SINCE_VERSION 1
#define XDG_POSITIONER_SET_ANCHOR_RECT_SINCE_VERSION 1
#define XDG_POSITIONER_SET_ANCHOR_SINCE_VERSION 1
#define XDG_POSITIONER_SET_GRAVITY_SINCE_VERSION 1
#define XDG_POSITIONER_SET_CONSTRAINT_ADJUSTMENT_SINCE_VERSION 1
#define XDG_POSITIONER_SET_OFFSET_SINCE_VERSION 1
#define XDG_POSITIONER_SET_REACTIVE_SINCE_VERSION 3
#define XDG_POSITIONER_SET_PARENT_SIZE_SINCE_VERSION 3
#define XDG_POSITIONER_SET_PARENT_CONFIGURE_SINCE_VERSION 3
static inline void xdg_positioner_destroy(struct xdg_positioner *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void xdg_positioner_set_size(struct xdg_positioner *o, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,1,width, height);
}
static inline void xdg_positioner_set_anchor_rect(struct xdg_positioner *o, int32_t x, int32_t y, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,2,x, y, width, height);
}
static inline void xdg_positioner_set_anchor(struct xdg_positioner *o, uint32_t anchor){
wl_proxy_marshal((struct wl_proxy*)o,3,anchor);
}
static inline void xdg_positioner_set_gravity(struct xdg_positioner *o, uint32_t gravity){
wl_proxy_marshal((struct wl_proxy*)o,4,gravity);
}
static inline void xdg_positioner_set_constraint_adjustment(struct xdg_positioner *o, uint32_t constraint_adjustment){
wl_proxy_marshal((struct wl_proxy*)o,5,constraint_adjustment);
}
static inline void xdg_positioner_set_offset(struct xdg_positioner *o, int32_t x, int32_t y){
wl_proxy_marshal((struct wl_proxy*)o,6,x, y);
}
static inline void xdg_positioner_set_reactive(struct xdg_positioner *o){
wl_proxy_marshal((struct wl_proxy*)o,7);
}
static inline void xdg_positioner_set_parent_size(struct xdg_positioner *o, int32_t parent_width, int32_t parent_height){
wl_proxy_marshal((struct wl_proxy*)o,8,parent_width, parent_height);
}
static inline void xdg_positioner_set_parent_configure(struct xdg_positioner *o, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,9,serial);
}
#define XDG_SURFACE_INTERFACE_VERSION 7
enum xdg_surface_error {
XDG_SURFACE_ERROR_NOT_CONSTRUCTED = 1,
XDG_SURFACE_ERROR_ALREADY_CONSTRUCTED = 2,
XDG_SURFACE_ERROR_UNCONFIGURED_BUFFER = 3,
XDG_SURFACE_ERROR_INVALID_SERIAL = 4,
XDG_SURFACE_ERROR_INVALID_SIZE = 5,
XDG_SURFACE_ERROR_DEFUNCT_ROLE_OBJECT = 6,
};
struct xdg_surface_listener {
 void (*configure)(void *data, struct xdg_surface *object, uint32_t serial);
};
static inline int xdg_surface_add_listener(struct xdg_surface *o,const struct xdg_surface_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void xdg_surface_set_user_data(struct xdg_surface *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *xdg_surface_get_user_data(struct xdg_surface *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define XDG_SURFACE_DESTROY_SINCE_VERSION 1
#define XDG_SURFACE_GET_TOPLEVEL_SINCE_VERSION 1
#define XDG_SURFACE_GET_POPUP_SINCE_VERSION 1
#define XDG_SURFACE_SET_WINDOW_GEOMETRY_SINCE_VERSION 1
#define XDG_SURFACE_ACK_CONFIGURE_SINCE_VERSION 1
#define XDG_SURFACE_CONFIGURE_SINCE_VERSION 1
static inline void xdg_surface_destroy(struct xdg_surface *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline struct xdg_toplevel * xdg_surface_get_toplevel(struct xdg_surface *o){
return (struct xdg_toplevel *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&xdg_toplevel_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline struct xdg_popup * xdg_surface_get_popup(struct xdg_surface *o, struct xdg_surface * parent, struct xdg_positioner * positioner){
return (struct xdg_popup *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,2,&xdg_popup_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, (struct wl_proxy*)parent, (struct wl_proxy*)positioner);
}
static inline void xdg_surface_set_window_geometry(struct xdg_surface *o, int32_t x, int32_t y, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,3,x, y, width, height);
}
static inline void xdg_surface_ack_configure(struct xdg_surface *o, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,4,serial);
}
#define XDG_TOPLEVEL_INTERFACE_VERSION 7
enum xdg_toplevel_error {
XDG_TOPLEVEL_ERROR_INVALID_RESIZE_EDGE = 0,
XDG_TOPLEVEL_ERROR_INVALID_PARENT = 1,
XDG_TOPLEVEL_ERROR_INVALID_SIZE = 2,
};
enum xdg_toplevel_resize_edge {
XDG_TOPLEVEL_RESIZE_EDGE_NONE = 0,
XDG_TOPLEVEL_RESIZE_EDGE_TOP = 1,
XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM = 2,
XDG_TOPLEVEL_RESIZE_EDGE_LEFT = 4,
XDG_TOPLEVEL_RESIZE_EDGE_TOP_LEFT = 5,
XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM_LEFT = 6,
XDG_TOPLEVEL_RESIZE_EDGE_RIGHT = 8,
XDG_TOPLEVEL_RESIZE_EDGE_TOP_RIGHT = 9,
XDG_TOPLEVEL_RESIZE_EDGE_BOTTOM_RIGHT = 10,
};
enum xdg_toplevel_state {
XDG_TOPLEVEL_STATE_MAXIMIZED = 1,
XDG_TOPLEVEL_STATE_FULLSCREEN = 2,
XDG_TOPLEVEL_STATE_RESIZING = 3,
XDG_TOPLEVEL_STATE_ACTIVATED = 4,
XDG_TOPLEVEL_STATE_TILED_LEFT = 5,
XDG_TOPLEVEL_STATE_TILED_RIGHT = 6,
XDG_TOPLEVEL_STATE_TILED_TOP = 7,
XDG_TOPLEVEL_STATE_TILED_BOTTOM = 8,
XDG_TOPLEVEL_STATE_SUSPENDED = 9,
XDG_TOPLEVEL_STATE_CONSTRAINED_LEFT = 10,
XDG_TOPLEVEL_STATE_CONSTRAINED_RIGHT = 11,
XDG_TOPLEVEL_STATE_CONSTRAINED_TOP = 12,
XDG_TOPLEVEL_STATE_CONSTRAINED_BOTTOM = 13,
};
enum xdg_toplevel_wm_capabilities {
XDG_TOPLEVEL_WM_CAPABILITIES_WINDOW_MENU = 1,
XDG_TOPLEVEL_WM_CAPABILITIES_MAXIMIZE = 2,
XDG_TOPLEVEL_WM_CAPABILITIES_FULLSCREEN = 3,
XDG_TOPLEVEL_WM_CAPABILITIES_MINIMIZE = 4,
};
struct xdg_toplevel_listener {
 void (*configure)(void *data, struct xdg_toplevel *object, int32_t width, int32_t height, struct wl_array * states);
 void (*close)(void *data, struct xdg_toplevel *object);
 void (*configure_bounds)(void *data, struct xdg_toplevel *object, int32_t width, int32_t height);
 void (*wm_capabilities)(void *data, struct xdg_toplevel *object, struct wl_array * capabilities);
};
static inline int xdg_toplevel_add_listener(struct xdg_toplevel *o,const struct xdg_toplevel_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void xdg_toplevel_set_user_data(struct xdg_toplevel *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *xdg_toplevel_get_user_data(struct xdg_toplevel *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define XDG_TOPLEVEL_DESTROY_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_PARENT_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_TITLE_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_APP_ID_SINCE_VERSION 1
#define XDG_TOPLEVEL_SHOW_WINDOW_MENU_SINCE_VERSION 1
#define XDG_TOPLEVEL_MOVE_SINCE_VERSION 1
#define XDG_TOPLEVEL_RESIZE_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_MAX_SIZE_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_MIN_SIZE_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_MAXIMIZED_SINCE_VERSION 1
#define XDG_TOPLEVEL_UNSET_MAXIMIZED_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_FULLSCREEN_SINCE_VERSION 1
#define XDG_TOPLEVEL_UNSET_FULLSCREEN_SINCE_VERSION 1
#define XDG_TOPLEVEL_SET_MINIMIZED_SINCE_VERSION 1
#define XDG_TOPLEVEL_CONFIGURE_SINCE_VERSION 1
#define XDG_TOPLEVEL_CLOSE_SINCE_VERSION 1
#define XDG_TOPLEVEL_CONFIGURE_BOUNDS_SINCE_VERSION 4
#define XDG_TOPLEVEL_WM_CAPABILITIES_SINCE_VERSION 5
static inline void xdg_toplevel_destroy(struct xdg_toplevel *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void xdg_toplevel_set_parent(struct xdg_toplevel *o, struct xdg_toplevel * parent){
wl_proxy_marshal((struct wl_proxy*)o,1,(struct wl_proxy*)parent);
}
static inline void xdg_toplevel_set_title(struct xdg_toplevel *o, const char * title){
wl_proxy_marshal((struct wl_proxy*)o,2,title);
}
static inline void xdg_toplevel_set_app_id(struct xdg_toplevel *o, const char * app_id){
wl_proxy_marshal((struct wl_proxy*)o,3,app_id);
}
static inline void xdg_toplevel_show_window_menu(struct xdg_toplevel *o, struct wl_seat * seat, uint32_t serial, int32_t x, int32_t y){
wl_proxy_marshal((struct wl_proxy*)o,4,(struct wl_proxy*)seat, serial, x, y);
}
static inline void xdg_toplevel_move(struct xdg_toplevel *o, struct wl_seat * seat, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,5,(struct wl_proxy*)seat, serial);
}
static inline void xdg_toplevel_resize(struct xdg_toplevel *o, struct wl_seat * seat, uint32_t serial, uint32_t edges){
wl_proxy_marshal((struct wl_proxy*)o,6,(struct wl_proxy*)seat, serial, edges);
}
static inline void xdg_toplevel_set_max_size(struct xdg_toplevel *o, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,7,width, height);
}
static inline void xdg_toplevel_set_min_size(struct xdg_toplevel *o, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,8,width, height);
}
static inline void xdg_toplevel_set_maximized(struct xdg_toplevel *o){
wl_proxy_marshal((struct wl_proxy*)o,9);
}
static inline void xdg_toplevel_unset_maximized(struct xdg_toplevel *o){
wl_proxy_marshal((struct wl_proxy*)o,10);
}
static inline void xdg_toplevel_set_fullscreen(struct xdg_toplevel *o, struct wl_output * output){
wl_proxy_marshal((struct wl_proxy*)o,11,(struct wl_proxy*)output);
}
static inline void xdg_toplevel_unset_fullscreen(struct xdg_toplevel *o){
wl_proxy_marshal((struct wl_proxy*)o,12);
}
static inline void xdg_toplevel_set_minimized(struct xdg_toplevel *o){
wl_proxy_marshal((struct wl_proxy*)o,13);
}
#define XDG_POPUP_INTERFACE_VERSION 7
enum xdg_popup_error {
XDG_POPUP_ERROR_INVALID_GRAB = 0,
};
struct xdg_popup_listener {
 void (*configure)(void *data, struct xdg_popup *object, int32_t x, int32_t y, int32_t width, int32_t height);
 void (*popup_done)(void *data, struct xdg_popup *object);
 void (*repositioned)(void *data, struct xdg_popup *object, uint32_t token);
};
static inline int xdg_popup_add_listener(struct xdg_popup *o,const struct xdg_popup_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void xdg_popup_set_user_data(struct xdg_popup *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *xdg_popup_get_user_data(struct xdg_popup *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define XDG_POPUP_DESTROY_SINCE_VERSION 1
#define XDG_POPUP_GRAB_SINCE_VERSION 1
#define XDG_POPUP_REPOSITION_SINCE_VERSION 3
#define XDG_POPUP_CONFIGURE_SINCE_VERSION 1
#define XDG_POPUP_POPUP_DONE_SINCE_VERSION 1
#define XDG_POPUP_REPOSITIONED_SINCE_VERSION 3
static inline void xdg_popup_destroy(struct xdg_popup *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void xdg_popup_grab(struct xdg_popup *o, struct wl_seat * seat, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,1,(struct wl_proxy*)seat, serial);
}
static inline void xdg_popup_reposition(struct xdg_popup *o, struct xdg_positioner * positioner, uint32_t token){
wl_proxy_marshal((struct wl_proxy*)o,2,(struct wl_proxy*)positioner, token);
}
