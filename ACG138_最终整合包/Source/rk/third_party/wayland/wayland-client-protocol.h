/* Generated from the packaged upstream Wayland XML. */
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <wayland-client-core.h>
struct wl_buffer;
extern const struct wl_interface wl_buffer_interface;
struct wl_callback;
extern const struct wl_interface wl_callback_interface;
struct wl_compositor;
extern const struct wl_interface wl_compositor_interface;
struct wl_data_device;
extern const struct wl_interface wl_data_device_interface;
struct wl_data_device_manager;
extern const struct wl_interface wl_data_device_manager_interface;
struct wl_data_offer;
extern const struct wl_interface wl_data_offer_interface;
struct wl_data_source;
extern const struct wl_interface wl_data_source_interface;
struct wl_display;
extern const struct wl_interface wl_display_interface;
struct wl_keyboard;
extern const struct wl_interface wl_keyboard_interface;
struct wl_output;
extern const struct wl_interface wl_output_interface;
struct wl_pointer;
extern const struct wl_interface wl_pointer_interface;
struct wl_region;
extern const struct wl_interface wl_region_interface;
struct wl_registry;
extern const struct wl_interface wl_registry_interface;
struct wl_seat;
extern const struct wl_interface wl_seat_interface;
struct wl_shell;
extern const struct wl_interface wl_shell_interface;
struct wl_shell_surface;
extern const struct wl_interface wl_shell_surface_interface;
struct wl_shm;
extern const struct wl_interface wl_shm_interface;
struct wl_shm_pool;
extern const struct wl_interface wl_shm_pool_interface;
struct wl_subcompositor;
extern const struct wl_interface wl_subcompositor_interface;
struct wl_subsurface;
extern const struct wl_interface wl_subsurface_interface;
struct wl_surface;
extern const struct wl_interface wl_surface_interface;
struct wl_touch;
extern const struct wl_interface wl_touch_interface;
#define WL_DISPLAY_INTERFACE_VERSION 1
enum wl_display_error {
WL_DISPLAY_ERROR_INVALID_OBJECT = 0,
WL_DISPLAY_ERROR_INVALID_METHOD = 1,
WL_DISPLAY_ERROR_NO_MEMORY = 2,
WL_DISPLAY_ERROR_IMPLEMENTATION = 3,
};
struct wl_display_listener {
 void (*error)(void *data, struct wl_display *object, void * object_id, uint32_t code, const char * message);
 void (*delete_id)(void *data, struct wl_display *object, uint32_t id);
};
static inline int wl_display_add_listener(struct wl_display *o,const struct wl_display_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_display_set_user_data(struct wl_display *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_display_get_user_data(struct wl_display *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_DISPLAY_SYNC_SINCE_VERSION 1
#define WL_DISPLAY_GET_REGISTRY_SINCE_VERSION 1
#define WL_DISPLAY_ERROR_SINCE_VERSION 1
#define WL_DISPLAY_DELETE_ID_SINCE_VERSION 1
static inline void wl_display_destroy(struct wl_display *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline struct wl_callback * wl_display_sync(struct wl_display *o){
return (struct wl_callback *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,&wl_callback_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline struct wl_registry * wl_display_get_registry(struct wl_display *o){
return (struct wl_registry *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&wl_registry_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
#define WL_REGISTRY_INTERFACE_VERSION 1
struct wl_registry_listener {
 void (*global)(void *data, struct wl_registry *object, uint32_t name, const char * interface, uint32_t version);
 void (*global_remove)(void *data, struct wl_registry *object, uint32_t name);
};
static inline int wl_registry_add_listener(struct wl_registry *o,const struct wl_registry_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_registry_set_user_data(struct wl_registry *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_registry_get_user_data(struct wl_registry *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_REGISTRY_BIND_SINCE_VERSION 1
#define WL_REGISTRY_GLOBAL_SINCE_VERSION 1
#define WL_REGISTRY_GLOBAL_REMOVE_SINCE_VERSION 1
static inline void wl_registry_destroy(struct wl_registry *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline void * wl_registry_bind(struct wl_registry *o, uint32_t name, const struct wl_interface *interface, uint32_t version){
return wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,interface,version,name, interface->name, version, NULL);
}
#define WL_CALLBACK_INTERFACE_VERSION 1
struct wl_callback_listener {
 void (*done)(void *data, struct wl_callback *object, uint32_t callback_data);
};
static inline int wl_callback_add_listener(struct wl_callback *o,const struct wl_callback_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_callback_set_user_data(struct wl_callback *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_callback_get_user_data(struct wl_callback *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_CALLBACK_DONE_SINCE_VERSION 1
static inline void wl_callback_destroy(struct wl_callback *o){wl_proxy_destroy((struct wl_proxy*)o);}
#define WL_COMPOSITOR_INTERFACE_VERSION 5
static inline void wl_compositor_set_user_data(struct wl_compositor *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_compositor_get_user_data(struct wl_compositor *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_COMPOSITOR_CREATE_SURFACE_SINCE_VERSION 1
#define WL_COMPOSITOR_CREATE_REGION_SINCE_VERSION 1
static inline void wl_compositor_destroy(struct wl_compositor *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline struct wl_surface * wl_compositor_create_surface(struct wl_compositor *o){
return (struct wl_surface *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,&wl_surface_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline struct wl_region * wl_compositor_create_region(struct wl_compositor *o){
return (struct wl_region *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&wl_region_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
#define WL_SHM_POOL_INTERFACE_VERSION 1
static inline void wl_shm_pool_set_user_data(struct wl_shm_pool *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_shm_pool_get_user_data(struct wl_shm_pool *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SHM_POOL_CREATE_BUFFER_SINCE_VERSION 1
#define WL_SHM_POOL_DESTROY_SINCE_VERSION 1
#define WL_SHM_POOL_RESIZE_SINCE_VERSION 1
static inline struct wl_buffer * wl_shm_pool_create_buffer(struct wl_shm_pool *o, int32_t offset, int32_t width, int32_t height, int32_t stride, uint32_t format){
return (struct wl_buffer *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,&wl_buffer_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, offset, width, height, stride, format);
}
static inline void wl_shm_pool_destroy(struct wl_shm_pool *o){
wl_proxy_marshal((struct wl_proxy*)o,1);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void wl_shm_pool_resize(struct wl_shm_pool *o, int32_t size){
wl_proxy_marshal((struct wl_proxy*)o,2,size);
}
#define WL_SHM_INTERFACE_VERSION 1
enum wl_shm_error {
WL_SHM_ERROR_INVALID_FORMAT = 0,
WL_SHM_ERROR_INVALID_STRIDE = 1,
WL_SHM_ERROR_INVALID_FD = 2,
};
enum wl_shm_format {
WL_SHM_FORMAT_ARGB8888 = 0,
WL_SHM_FORMAT_XRGB8888 = 1,
WL_SHM_FORMAT_C8 = 0x20203843,
WL_SHM_FORMAT_RGB332 = 0x38424752,
WL_SHM_FORMAT_BGR233 = 0x38524742,
WL_SHM_FORMAT_XRGB4444 = 0x32315258,
WL_SHM_FORMAT_XBGR4444 = 0x32314258,
WL_SHM_FORMAT_RGBX4444 = 0x32315852,
WL_SHM_FORMAT_BGRX4444 = 0x32315842,
WL_SHM_FORMAT_ARGB4444 = 0x32315241,
WL_SHM_FORMAT_ABGR4444 = 0x32314241,
WL_SHM_FORMAT_RGBA4444 = 0x32314152,
WL_SHM_FORMAT_BGRA4444 = 0x32314142,
WL_SHM_FORMAT_XRGB1555 = 0x35315258,
WL_SHM_FORMAT_XBGR1555 = 0x35314258,
WL_SHM_FORMAT_RGBX5551 = 0x35315852,
WL_SHM_FORMAT_BGRX5551 = 0x35315842,
WL_SHM_FORMAT_ARGB1555 = 0x35315241,
WL_SHM_FORMAT_ABGR1555 = 0x35314241,
WL_SHM_FORMAT_RGBA5551 = 0x35314152,
WL_SHM_FORMAT_BGRA5551 = 0x35314142,
WL_SHM_FORMAT_RGB565 = 0x36314752,
WL_SHM_FORMAT_BGR565 = 0x36314742,
WL_SHM_FORMAT_RGB888 = 0x34324752,
WL_SHM_FORMAT_BGR888 = 0x34324742,
WL_SHM_FORMAT_XBGR8888 = 0x34324258,
WL_SHM_FORMAT_RGBX8888 = 0x34325852,
WL_SHM_FORMAT_BGRX8888 = 0x34325842,
WL_SHM_FORMAT_ABGR8888 = 0x34324241,
WL_SHM_FORMAT_RGBA8888 = 0x34324152,
WL_SHM_FORMAT_BGRA8888 = 0x34324142,
WL_SHM_FORMAT_XRGB2101010 = 0x30335258,
WL_SHM_FORMAT_XBGR2101010 = 0x30334258,
WL_SHM_FORMAT_RGBX1010102 = 0x30335852,
WL_SHM_FORMAT_BGRX1010102 = 0x30335842,
WL_SHM_FORMAT_ARGB2101010 = 0x30335241,
WL_SHM_FORMAT_ABGR2101010 = 0x30334241,
WL_SHM_FORMAT_RGBA1010102 = 0x30334152,
WL_SHM_FORMAT_BGRA1010102 = 0x30334142,
WL_SHM_FORMAT_YUYV = 0x56595559,
WL_SHM_FORMAT_YVYU = 0x55595659,
WL_SHM_FORMAT_UYVY = 0x59565955,
WL_SHM_FORMAT_VYUY = 0x59555956,
WL_SHM_FORMAT_AYUV = 0x56555941,
WL_SHM_FORMAT_NV12 = 0x3231564e,
WL_SHM_FORMAT_NV21 = 0x3132564e,
WL_SHM_FORMAT_NV16 = 0x3631564e,
WL_SHM_FORMAT_NV61 = 0x3136564e,
WL_SHM_FORMAT_YUV410 = 0x39565559,
WL_SHM_FORMAT_YVU410 = 0x39555659,
WL_SHM_FORMAT_YUV411 = 0x31315559,
WL_SHM_FORMAT_YVU411 = 0x31315659,
WL_SHM_FORMAT_YUV420 = 0x32315559,
WL_SHM_FORMAT_YVU420 = 0x32315659,
WL_SHM_FORMAT_YUV422 = 0x36315559,
WL_SHM_FORMAT_YVU422 = 0x36315659,
WL_SHM_FORMAT_YUV444 = 0x34325559,
WL_SHM_FORMAT_YVU444 = 0x34325659,
WL_SHM_FORMAT_R8 = 0x20203852,
WL_SHM_FORMAT_R16 = 0x20363152,
WL_SHM_FORMAT_RG88 = 0x38384752,
WL_SHM_FORMAT_GR88 = 0x38385247,
WL_SHM_FORMAT_RG1616 = 0x32334752,
WL_SHM_FORMAT_GR1616 = 0x32335247,
WL_SHM_FORMAT_XRGB16161616F = 0x48345258,
WL_SHM_FORMAT_XBGR16161616F = 0x48344258,
WL_SHM_FORMAT_ARGB16161616F = 0x48345241,
WL_SHM_FORMAT_ABGR16161616F = 0x48344241,
WL_SHM_FORMAT_XYUV8888 = 0x56555958,
WL_SHM_FORMAT_VUY888 = 0x34325556,
WL_SHM_FORMAT_VUY101010 = 0x30335556,
WL_SHM_FORMAT_Y210 = 0x30313259,
WL_SHM_FORMAT_Y212 = 0x32313259,
WL_SHM_FORMAT_Y216 = 0x36313259,
WL_SHM_FORMAT_Y410 = 0x30313459,
WL_SHM_FORMAT_Y412 = 0x32313459,
WL_SHM_FORMAT_Y416 = 0x36313459,
WL_SHM_FORMAT_XVYU2101010 = 0x30335658,
WL_SHM_FORMAT_XVYU12_16161616 = 0x36335658,
WL_SHM_FORMAT_XVYU16161616 = 0x38345658,
WL_SHM_FORMAT_Y0L0 = 0x304c3059,
WL_SHM_FORMAT_X0L0 = 0x304c3058,
WL_SHM_FORMAT_Y0L2 = 0x324c3059,
WL_SHM_FORMAT_X0L2 = 0x324c3058,
WL_SHM_FORMAT_YUV420_8BIT = 0x38305559,
WL_SHM_FORMAT_YUV420_10BIT = 0x30315559,
WL_SHM_FORMAT_XRGB8888_A8 = 0x38415258,
WL_SHM_FORMAT_XBGR8888_A8 = 0x38414258,
WL_SHM_FORMAT_RGBX8888_A8 = 0x38415852,
WL_SHM_FORMAT_BGRX8888_A8 = 0x38415842,
WL_SHM_FORMAT_RGB888_A8 = 0x38413852,
WL_SHM_FORMAT_BGR888_A8 = 0x38413842,
WL_SHM_FORMAT_RGB565_A8 = 0x38413552,
WL_SHM_FORMAT_BGR565_A8 = 0x38413542,
WL_SHM_FORMAT_NV24 = 0x3432564e,
WL_SHM_FORMAT_NV42 = 0x3234564e,
WL_SHM_FORMAT_P210 = 0x30313250,
WL_SHM_FORMAT_P010 = 0x30313050,
WL_SHM_FORMAT_P012 = 0x32313050,
WL_SHM_FORMAT_P016 = 0x36313050,
WL_SHM_FORMAT_AXBXGXRX106106106106 = 0x30314241,
WL_SHM_FORMAT_NV15 = 0x3531564e,
WL_SHM_FORMAT_Q410 = 0x30313451,
WL_SHM_FORMAT_Q401 = 0x31303451,
WL_SHM_FORMAT_XRGB16161616 = 0x38345258,
WL_SHM_FORMAT_XBGR16161616 = 0x38344258,
WL_SHM_FORMAT_ARGB16161616 = 0x38345241,
WL_SHM_FORMAT_ABGR16161616 = 0x38344241,
};
struct wl_shm_listener {
 void (*format)(void *data, struct wl_shm *object, uint32_t format);
};
static inline int wl_shm_add_listener(struct wl_shm *o,const struct wl_shm_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_shm_set_user_data(struct wl_shm *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_shm_get_user_data(struct wl_shm *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SHM_CREATE_POOL_SINCE_VERSION 1
#define WL_SHM_FORMAT_SINCE_VERSION 1
static inline void wl_shm_destroy(struct wl_shm *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline struct wl_shm_pool * wl_shm_create_pool(struct wl_shm *o, int32_t fd, int32_t size){
return (struct wl_shm_pool *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,&wl_shm_pool_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, fd, size);
}
#define WL_BUFFER_INTERFACE_VERSION 1
struct wl_buffer_listener {
 void (*release)(void *data, struct wl_buffer *object);
};
static inline int wl_buffer_add_listener(struct wl_buffer *o,const struct wl_buffer_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_buffer_set_user_data(struct wl_buffer *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_buffer_get_user_data(struct wl_buffer *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_BUFFER_DESTROY_SINCE_VERSION 1
#define WL_BUFFER_RELEASE_SINCE_VERSION 1
static inline void wl_buffer_destroy(struct wl_buffer *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
#define WL_DATA_OFFER_INTERFACE_VERSION 3
enum wl_data_offer_error {
WL_DATA_OFFER_ERROR_INVALID_FINISH = 0,
WL_DATA_OFFER_ERROR_INVALID_ACTION_MASK = 1,
WL_DATA_OFFER_ERROR_INVALID_ACTION = 2,
WL_DATA_OFFER_ERROR_INVALID_OFFER = 3,
};
struct wl_data_offer_listener {
 void (*offer)(void *data, struct wl_data_offer *object, const char * mime_type);
 void (*source_actions)(void *data, struct wl_data_offer *object, uint32_t source_actions);
 void (*action)(void *data, struct wl_data_offer *object, uint32_t dnd_action);
};
static inline int wl_data_offer_add_listener(struct wl_data_offer *o,const struct wl_data_offer_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_data_offer_set_user_data(struct wl_data_offer *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_data_offer_get_user_data(struct wl_data_offer *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_DATA_OFFER_ACCEPT_SINCE_VERSION 1
#define WL_DATA_OFFER_RECEIVE_SINCE_VERSION 1
#define WL_DATA_OFFER_DESTROY_SINCE_VERSION 1
#define WL_DATA_OFFER_FINISH_SINCE_VERSION 3
#define WL_DATA_OFFER_SET_ACTIONS_SINCE_VERSION 3
#define WL_DATA_OFFER_OFFER_SINCE_VERSION 1
#define WL_DATA_OFFER_SOURCE_ACTIONS_SINCE_VERSION 3
#define WL_DATA_OFFER_ACTION_SINCE_VERSION 3
static inline void wl_data_offer_accept(struct wl_data_offer *o, uint32_t serial, const char * mime_type){
wl_proxy_marshal((struct wl_proxy*)o,0,serial, mime_type);
}
static inline void wl_data_offer_receive(struct wl_data_offer *o, const char * mime_type, int32_t fd){
wl_proxy_marshal((struct wl_proxy*)o,1,mime_type, fd);
}
static inline void wl_data_offer_destroy(struct wl_data_offer *o){
wl_proxy_marshal((struct wl_proxy*)o,2);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void wl_data_offer_finish(struct wl_data_offer *o){
wl_proxy_marshal((struct wl_proxy*)o,3);
}
static inline void wl_data_offer_set_actions(struct wl_data_offer *o, uint32_t dnd_actions, uint32_t preferred_action){
wl_proxy_marshal((struct wl_proxy*)o,4,dnd_actions, preferred_action);
}
#define WL_DATA_SOURCE_INTERFACE_VERSION 3
enum wl_data_source_error {
WL_DATA_SOURCE_ERROR_INVALID_ACTION_MASK = 0,
WL_DATA_SOURCE_ERROR_INVALID_SOURCE = 1,
};
struct wl_data_source_listener {
 void (*target)(void *data, struct wl_data_source *object, const char * mime_type);
 void (*send)(void *data, struct wl_data_source *object, const char * mime_type, int32_t fd);
 void (*cancelled)(void *data, struct wl_data_source *object);
 void (*dnd_drop_performed)(void *data, struct wl_data_source *object);
 void (*dnd_finished)(void *data, struct wl_data_source *object);
 void (*action)(void *data, struct wl_data_source *object, uint32_t dnd_action);
};
static inline int wl_data_source_add_listener(struct wl_data_source *o,const struct wl_data_source_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_data_source_set_user_data(struct wl_data_source *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_data_source_get_user_data(struct wl_data_source *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_DATA_SOURCE_OFFER_SINCE_VERSION 1
#define WL_DATA_SOURCE_DESTROY_SINCE_VERSION 1
#define WL_DATA_SOURCE_SET_ACTIONS_SINCE_VERSION 3
#define WL_DATA_SOURCE_TARGET_SINCE_VERSION 1
#define WL_DATA_SOURCE_SEND_SINCE_VERSION 1
#define WL_DATA_SOURCE_CANCELLED_SINCE_VERSION 1
#define WL_DATA_SOURCE_DND_DROP_PERFORMED_SINCE_VERSION 3
#define WL_DATA_SOURCE_DND_FINISHED_SINCE_VERSION 3
#define WL_DATA_SOURCE_ACTION_SINCE_VERSION 3
static inline void wl_data_source_offer(struct wl_data_source *o, const char * mime_type){
wl_proxy_marshal((struct wl_proxy*)o,0,mime_type);
}
static inline void wl_data_source_destroy(struct wl_data_source *o){
wl_proxy_marshal((struct wl_proxy*)o,1);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void wl_data_source_set_actions(struct wl_data_source *o, uint32_t dnd_actions){
wl_proxy_marshal((struct wl_proxy*)o,2,dnd_actions);
}
#define WL_DATA_DEVICE_INTERFACE_VERSION 3
enum wl_data_device_error {
WL_DATA_DEVICE_ERROR_ROLE = 0,
};
struct wl_data_device_listener {
 void (*data_offer)(void *data, struct wl_data_device *object, struct wl_data_offer * id);
 void (*enter)(void *data, struct wl_data_device *object, uint32_t serial, struct wl_surface * surface, wl_fixed_t x, wl_fixed_t y, struct wl_data_offer * id);
 void (*leave)(void *data, struct wl_data_device *object);
 void (*motion)(void *data, struct wl_data_device *object, uint32_t time, wl_fixed_t x, wl_fixed_t y);
 void (*drop)(void *data, struct wl_data_device *object);
 void (*selection)(void *data, struct wl_data_device *object, struct wl_data_offer * id);
};
static inline int wl_data_device_add_listener(struct wl_data_device *o,const struct wl_data_device_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_data_device_set_user_data(struct wl_data_device *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_data_device_get_user_data(struct wl_data_device *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_DATA_DEVICE_START_DRAG_SINCE_VERSION 1
#define WL_DATA_DEVICE_SET_SELECTION_SINCE_VERSION 1
#define WL_DATA_DEVICE_RELEASE_SINCE_VERSION 2
#define WL_DATA_DEVICE_DATA_OFFER_SINCE_VERSION 1
#define WL_DATA_DEVICE_ENTER_SINCE_VERSION 1
#define WL_DATA_DEVICE_LEAVE_SINCE_VERSION 1
#define WL_DATA_DEVICE_MOTION_SINCE_VERSION 1
#define WL_DATA_DEVICE_DROP_SINCE_VERSION 1
#define WL_DATA_DEVICE_SELECTION_SINCE_VERSION 1
static inline void wl_data_device_destroy(struct wl_data_device *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline void wl_data_device_start_drag(struct wl_data_device *o, struct wl_data_source * source, struct wl_surface * origin, struct wl_surface * icon, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,0,(struct wl_proxy*)source, (struct wl_proxy*)origin, (struct wl_proxy*)icon, serial);
}
static inline void wl_data_device_set_selection(struct wl_data_device *o, struct wl_data_source * source, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,1,(struct wl_proxy*)source, serial);
}
static inline void wl_data_device_release(struct wl_data_device *o){
wl_proxy_marshal((struct wl_proxy*)o,2);
wl_proxy_destroy((struct wl_proxy*)o);
}
#define WL_DATA_DEVICE_MANAGER_INTERFACE_VERSION 3
enum wl_data_device_manager_dnd_action {
WL_DATA_DEVICE_MANAGER_DND_ACTION_NONE = 0,
WL_DATA_DEVICE_MANAGER_DND_ACTION_COPY = 1,
WL_DATA_DEVICE_MANAGER_DND_ACTION_MOVE = 2,
WL_DATA_DEVICE_MANAGER_DND_ACTION_ASK = 4,
};
static inline void wl_data_device_manager_set_user_data(struct wl_data_device_manager *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_data_device_manager_get_user_data(struct wl_data_device_manager *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_DATA_DEVICE_MANAGER_CREATE_DATA_SOURCE_SINCE_VERSION 1
#define WL_DATA_DEVICE_MANAGER_GET_DATA_DEVICE_SINCE_VERSION 1
static inline void wl_data_device_manager_destroy(struct wl_data_device_manager *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline struct wl_data_source * wl_data_device_manager_create_data_source(struct wl_data_device_manager *o){
return (struct wl_data_source *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,&wl_data_source_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline struct wl_data_device * wl_data_device_manager_get_data_device(struct wl_data_device_manager *o, struct wl_seat * seat){
return (struct wl_data_device *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&wl_data_device_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, (struct wl_proxy*)seat);
}
#define WL_SHELL_INTERFACE_VERSION 1
enum wl_shell_error {
WL_SHELL_ERROR_ROLE = 0,
};
static inline void wl_shell_set_user_data(struct wl_shell *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_shell_get_user_data(struct wl_shell *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SHELL_GET_SHELL_SURFACE_SINCE_VERSION 1
static inline void wl_shell_destroy(struct wl_shell *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline struct wl_shell_surface * wl_shell_get_shell_surface(struct wl_shell *o, struct wl_surface * surface){
return (struct wl_shell_surface *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,&wl_shell_surface_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, (struct wl_proxy*)surface);
}
#define WL_SHELL_SURFACE_INTERFACE_VERSION 1
enum wl_shell_surface_resize {
WL_SHELL_SURFACE_RESIZE_NONE = 0,
WL_SHELL_SURFACE_RESIZE_TOP = 1,
WL_SHELL_SURFACE_RESIZE_BOTTOM = 2,
WL_SHELL_SURFACE_RESIZE_LEFT = 4,
WL_SHELL_SURFACE_RESIZE_TOP_LEFT = 5,
WL_SHELL_SURFACE_RESIZE_BOTTOM_LEFT = 6,
WL_SHELL_SURFACE_RESIZE_RIGHT = 8,
WL_SHELL_SURFACE_RESIZE_TOP_RIGHT = 9,
WL_SHELL_SURFACE_RESIZE_BOTTOM_RIGHT = 10,
};
enum wl_shell_surface_transient {
WL_SHELL_SURFACE_TRANSIENT_INACTIVE = 0x1,
};
enum wl_shell_surface_fullscreen_method {
WL_SHELL_SURFACE_FULLSCREEN_METHOD_DEFAULT = 0,
WL_SHELL_SURFACE_FULLSCREEN_METHOD_SCALE = 1,
WL_SHELL_SURFACE_FULLSCREEN_METHOD_DRIVER = 2,
WL_SHELL_SURFACE_FULLSCREEN_METHOD_FILL = 3,
};
struct wl_shell_surface_listener {
 void (*ping)(void *data, struct wl_shell_surface *object, uint32_t serial);
 void (*configure)(void *data, struct wl_shell_surface *object, uint32_t edges, int32_t width, int32_t height);
 void (*popup_done)(void *data, struct wl_shell_surface *object);
};
static inline int wl_shell_surface_add_listener(struct wl_shell_surface *o,const struct wl_shell_surface_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_shell_surface_set_user_data(struct wl_shell_surface *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_shell_surface_get_user_data(struct wl_shell_surface *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SHELL_SURFACE_PONG_SINCE_VERSION 1
#define WL_SHELL_SURFACE_MOVE_SINCE_VERSION 1
#define WL_SHELL_SURFACE_RESIZE_SINCE_VERSION 1
#define WL_SHELL_SURFACE_SET_TOPLEVEL_SINCE_VERSION 1
#define WL_SHELL_SURFACE_SET_TRANSIENT_SINCE_VERSION 1
#define WL_SHELL_SURFACE_SET_FULLSCREEN_SINCE_VERSION 1
#define WL_SHELL_SURFACE_SET_POPUP_SINCE_VERSION 1
#define WL_SHELL_SURFACE_SET_MAXIMIZED_SINCE_VERSION 1
#define WL_SHELL_SURFACE_SET_TITLE_SINCE_VERSION 1
#define WL_SHELL_SURFACE_SET_CLASS_SINCE_VERSION 1
#define WL_SHELL_SURFACE_PING_SINCE_VERSION 1
#define WL_SHELL_SURFACE_CONFIGURE_SINCE_VERSION 1
#define WL_SHELL_SURFACE_POPUP_DONE_SINCE_VERSION 1
static inline void wl_shell_surface_destroy(struct wl_shell_surface *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline void wl_shell_surface_pong(struct wl_shell_surface *o, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,0,serial);
}
static inline void wl_shell_surface_move(struct wl_shell_surface *o, struct wl_seat * seat, uint32_t serial){
wl_proxy_marshal((struct wl_proxy*)o,1,(struct wl_proxy*)seat, serial);
}
static inline void wl_shell_surface_resize(struct wl_shell_surface *o, struct wl_seat * seat, uint32_t serial, uint32_t edges){
wl_proxy_marshal((struct wl_proxy*)o,2,(struct wl_proxy*)seat, serial, edges);
}
static inline void wl_shell_surface_set_toplevel(struct wl_shell_surface *o){
wl_proxy_marshal((struct wl_proxy*)o,3);
}
static inline void wl_shell_surface_set_transient(struct wl_shell_surface *o, struct wl_surface * parent, int32_t x, int32_t y, uint32_t flags){
wl_proxy_marshal((struct wl_proxy*)o,4,(struct wl_proxy*)parent, x, y, flags);
}
static inline void wl_shell_surface_set_fullscreen(struct wl_shell_surface *o, uint32_t method, uint32_t framerate, struct wl_output * output){
wl_proxy_marshal((struct wl_proxy*)o,5,method, framerate, (struct wl_proxy*)output);
}
static inline void wl_shell_surface_set_popup(struct wl_shell_surface *o, struct wl_seat * seat, uint32_t serial, struct wl_surface * parent, int32_t x, int32_t y, uint32_t flags){
wl_proxy_marshal((struct wl_proxy*)o,6,(struct wl_proxy*)seat, serial, (struct wl_proxy*)parent, x, y, flags);
}
static inline void wl_shell_surface_set_maximized(struct wl_shell_surface *o, struct wl_output * output){
wl_proxy_marshal((struct wl_proxy*)o,7,(struct wl_proxy*)output);
}
static inline void wl_shell_surface_set_title(struct wl_shell_surface *o, const char * title){
wl_proxy_marshal((struct wl_proxy*)o,8,title);
}
static inline void wl_shell_surface_set_class(struct wl_shell_surface *o, const char * class_){
wl_proxy_marshal((struct wl_proxy*)o,9,class_);
}
#define WL_SURFACE_INTERFACE_VERSION 5
enum wl_surface_error {
WL_SURFACE_ERROR_INVALID_SCALE = 0,
WL_SURFACE_ERROR_INVALID_TRANSFORM = 1,
WL_SURFACE_ERROR_INVALID_SIZE = 2,
WL_SURFACE_ERROR_INVALID_OFFSET = 3,
};
struct wl_surface_listener {
 void (*enter)(void *data, struct wl_surface *object, struct wl_output * output);
 void (*leave)(void *data, struct wl_surface *object, struct wl_output * output);
};
static inline int wl_surface_add_listener(struct wl_surface *o,const struct wl_surface_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_surface_set_user_data(struct wl_surface *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_surface_get_user_data(struct wl_surface *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SURFACE_DESTROY_SINCE_VERSION 1
#define WL_SURFACE_ATTACH_SINCE_VERSION 1
#define WL_SURFACE_DAMAGE_SINCE_VERSION 1
#define WL_SURFACE_FRAME_SINCE_VERSION 1
#define WL_SURFACE_SET_OPAQUE_REGION_SINCE_VERSION 1
#define WL_SURFACE_SET_INPUT_REGION_SINCE_VERSION 1
#define WL_SURFACE_COMMIT_SINCE_VERSION 1
#define WL_SURFACE_SET_BUFFER_TRANSFORM_SINCE_VERSION 2
#define WL_SURFACE_SET_BUFFER_SCALE_SINCE_VERSION 3
#define WL_SURFACE_DAMAGE_BUFFER_SINCE_VERSION 4
#define WL_SURFACE_OFFSET_SINCE_VERSION 5
#define WL_SURFACE_ENTER_SINCE_VERSION 1
#define WL_SURFACE_LEAVE_SINCE_VERSION 1
static inline void wl_surface_destroy(struct wl_surface *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void wl_surface_attach(struct wl_surface *o, struct wl_buffer * buffer, int32_t x, int32_t y){
wl_proxy_marshal((struct wl_proxy*)o,1,(struct wl_proxy*)buffer, x, y);
}
static inline void wl_surface_damage(struct wl_surface *o, int32_t x, int32_t y, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,2,x, y, width, height);
}
static inline struct wl_callback * wl_surface_frame(struct wl_surface *o){
return (struct wl_callback *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,3,&wl_callback_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline void wl_surface_set_opaque_region(struct wl_surface *o, struct wl_region * region){
wl_proxy_marshal((struct wl_proxy*)o,4,(struct wl_proxy*)region);
}
static inline void wl_surface_set_input_region(struct wl_surface *o, struct wl_region * region){
wl_proxy_marshal((struct wl_proxy*)o,5,(struct wl_proxy*)region);
}
static inline void wl_surface_commit(struct wl_surface *o){
wl_proxy_marshal((struct wl_proxy*)o,6);
}
static inline void wl_surface_set_buffer_transform(struct wl_surface *o, int32_t transform){
wl_proxy_marshal((struct wl_proxy*)o,7,transform);
}
static inline void wl_surface_set_buffer_scale(struct wl_surface *o, int32_t scale){
wl_proxy_marshal((struct wl_proxy*)o,8,scale);
}
static inline void wl_surface_damage_buffer(struct wl_surface *o, int32_t x, int32_t y, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,9,x, y, width, height);
}
static inline void wl_surface_offset(struct wl_surface *o, int32_t x, int32_t y){
wl_proxy_marshal((struct wl_proxy*)o,10,x, y);
}
#define WL_SEAT_INTERFACE_VERSION 8
enum wl_seat_capability {
WL_SEAT_CAPABILITY_POINTER = 1,
WL_SEAT_CAPABILITY_KEYBOARD = 2,
WL_SEAT_CAPABILITY_TOUCH = 4,
};
enum wl_seat_error {
WL_SEAT_ERROR_MISSING_CAPABILITY = 0,
};
struct wl_seat_listener {
 void (*capabilities)(void *data, struct wl_seat *object, uint32_t capabilities);
 void (*name)(void *data, struct wl_seat *object, const char * name);
};
static inline int wl_seat_add_listener(struct wl_seat *o,const struct wl_seat_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_seat_set_user_data(struct wl_seat *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_seat_get_user_data(struct wl_seat *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SEAT_GET_POINTER_SINCE_VERSION 1
#define WL_SEAT_GET_KEYBOARD_SINCE_VERSION 1
#define WL_SEAT_GET_TOUCH_SINCE_VERSION 1
#define WL_SEAT_RELEASE_SINCE_VERSION 5
#define WL_SEAT_CAPABILITIES_SINCE_VERSION 1
#define WL_SEAT_NAME_SINCE_VERSION 2
static inline void wl_seat_destroy(struct wl_seat *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline struct wl_pointer * wl_seat_get_pointer(struct wl_seat *o){
return (struct wl_pointer *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,0,&wl_pointer_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline struct wl_keyboard * wl_seat_get_keyboard(struct wl_seat *o){
return (struct wl_keyboard *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&wl_keyboard_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline struct wl_touch * wl_seat_get_touch(struct wl_seat *o){
return (struct wl_touch *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,2,&wl_touch_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL);
}
static inline void wl_seat_release(struct wl_seat *o){
wl_proxy_marshal((struct wl_proxy*)o,3);
wl_proxy_destroy((struct wl_proxy*)o);
}
#define WL_POINTER_INTERFACE_VERSION 8
enum wl_pointer_error {
WL_POINTER_ERROR_ROLE = 0,
};
enum wl_pointer_button_state {
WL_POINTER_BUTTON_STATE_RELEASED = 0,
WL_POINTER_BUTTON_STATE_PRESSED = 1,
};
enum wl_pointer_axis {
WL_POINTER_AXIS_VERTICAL_SCROLL = 0,
WL_POINTER_AXIS_HORIZONTAL_SCROLL = 1,
};
enum wl_pointer_axis_source {
WL_POINTER_AXIS_SOURCE_WHEEL = 0,
WL_POINTER_AXIS_SOURCE_FINGER = 1,
WL_POINTER_AXIS_SOURCE_CONTINUOUS = 2,
WL_POINTER_AXIS_SOURCE_WHEEL_TILT = 3,
};
struct wl_pointer_listener {
 void (*enter)(void *data, struct wl_pointer *object, uint32_t serial, struct wl_surface * surface, wl_fixed_t surface_x, wl_fixed_t surface_y);
 void (*leave)(void *data, struct wl_pointer *object, uint32_t serial, struct wl_surface * surface);
 void (*motion)(void *data, struct wl_pointer *object, uint32_t time, wl_fixed_t surface_x, wl_fixed_t surface_y);
 void (*button)(void *data, struct wl_pointer *object, uint32_t serial, uint32_t time, uint32_t button, uint32_t state);
 void (*axis)(void *data, struct wl_pointer *object, uint32_t time, uint32_t axis, wl_fixed_t value);
 void (*frame)(void *data, struct wl_pointer *object);
 void (*axis_source)(void *data, struct wl_pointer *object, uint32_t axis_source);
 void (*axis_stop)(void *data, struct wl_pointer *object, uint32_t time, uint32_t axis);
 void (*axis_discrete)(void *data, struct wl_pointer *object, uint32_t axis, int32_t discrete);
 void (*axis_value120)(void *data, struct wl_pointer *object, uint32_t axis, int32_t value120);
};
static inline int wl_pointer_add_listener(struct wl_pointer *o,const struct wl_pointer_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_pointer_set_user_data(struct wl_pointer *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_pointer_get_user_data(struct wl_pointer *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_POINTER_SET_CURSOR_SINCE_VERSION 1
#define WL_POINTER_RELEASE_SINCE_VERSION 3
#define WL_POINTER_ENTER_SINCE_VERSION 1
#define WL_POINTER_LEAVE_SINCE_VERSION 1
#define WL_POINTER_MOTION_SINCE_VERSION 1
#define WL_POINTER_BUTTON_SINCE_VERSION 1
#define WL_POINTER_AXIS_SINCE_VERSION 1
#define WL_POINTER_FRAME_SINCE_VERSION 5
#define WL_POINTER_AXIS_SOURCE_SINCE_VERSION 5
#define WL_POINTER_AXIS_STOP_SINCE_VERSION 5
#define WL_POINTER_AXIS_DISCRETE_SINCE_VERSION 5
#define WL_POINTER_AXIS_VALUE120_SINCE_VERSION 8
static inline void wl_pointer_destroy(struct wl_pointer *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline void wl_pointer_set_cursor(struct wl_pointer *o, uint32_t serial, struct wl_surface * surface, int32_t hotspot_x, int32_t hotspot_y){
wl_proxy_marshal((struct wl_proxy*)o,0,serial, (struct wl_proxy*)surface, hotspot_x, hotspot_y);
}
static inline void wl_pointer_release(struct wl_pointer *o){
wl_proxy_marshal((struct wl_proxy*)o,1);
wl_proxy_destroy((struct wl_proxy*)o);
}
#define WL_KEYBOARD_INTERFACE_VERSION 8
enum wl_keyboard_keymap_format {
WL_KEYBOARD_KEYMAP_FORMAT_NO_KEYMAP = 0,
WL_KEYBOARD_KEYMAP_FORMAT_XKB_V1 = 1,
};
enum wl_keyboard_key_state {
WL_KEYBOARD_KEY_STATE_RELEASED = 0,
WL_KEYBOARD_KEY_STATE_PRESSED = 1,
};
struct wl_keyboard_listener {
 void (*keymap)(void *data, struct wl_keyboard *object, uint32_t format, int32_t fd, uint32_t size);
 void (*enter)(void *data, struct wl_keyboard *object, uint32_t serial, struct wl_surface * surface, struct wl_array * keys);
 void (*leave)(void *data, struct wl_keyboard *object, uint32_t serial, struct wl_surface * surface);
 void (*key)(void *data, struct wl_keyboard *object, uint32_t serial, uint32_t time, uint32_t key, uint32_t state);
 void (*modifiers)(void *data, struct wl_keyboard *object, uint32_t serial, uint32_t mods_depressed, uint32_t mods_latched, uint32_t mods_locked, uint32_t group);
 void (*repeat_info)(void *data, struct wl_keyboard *object, int32_t rate, int32_t delay);
};
static inline int wl_keyboard_add_listener(struct wl_keyboard *o,const struct wl_keyboard_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_keyboard_set_user_data(struct wl_keyboard *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_keyboard_get_user_data(struct wl_keyboard *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_KEYBOARD_RELEASE_SINCE_VERSION 3
#define WL_KEYBOARD_KEYMAP_SINCE_VERSION 1
#define WL_KEYBOARD_ENTER_SINCE_VERSION 1
#define WL_KEYBOARD_LEAVE_SINCE_VERSION 1
#define WL_KEYBOARD_KEY_SINCE_VERSION 1
#define WL_KEYBOARD_MODIFIERS_SINCE_VERSION 1
#define WL_KEYBOARD_REPEAT_INFO_SINCE_VERSION 4
static inline void wl_keyboard_destroy(struct wl_keyboard *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline void wl_keyboard_release(struct wl_keyboard *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
#define WL_TOUCH_INTERFACE_VERSION 8
struct wl_touch_listener {
 void (*down)(void *data, struct wl_touch *object, uint32_t serial, uint32_t time, struct wl_surface * surface, int32_t id, wl_fixed_t x, wl_fixed_t y);
 void (*up)(void *data, struct wl_touch *object, uint32_t serial, uint32_t time, int32_t id);
 void (*motion)(void *data, struct wl_touch *object, uint32_t time, int32_t id, wl_fixed_t x, wl_fixed_t y);
 void (*frame)(void *data, struct wl_touch *object);
 void (*cancel)(void *data, struct wl_touch *object);
 void (*shape)(void *data, struct wl_touch *object, int32_t id, wl_fixed_t major, wl_fixed_t minor);
 void (*orientation)(void *data, struct wl_touch *object, int32_t id, wl_fixed_t orientation);
};
static inline int wl_touch_add_listener(struct wl_touch *o,const struct wl_touch_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_touch_set_user_data(struct wl_touch *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_touch_get_user_data(struct wl_touch *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_TOUCH_RELEASE_SINCE_VERSION 3
#define WL_TOUCH_DOWN_SINCE_VERSION 1
#define WL_TOUCH_UP_SINCE_VERSION 1
#define WL_TOUCH_MOTION_SINCE_VERSION 1
#define WL_TOUCH_FRAME_SINCE_VERSION 1
#define WL_TOUCH_CANCEL_SINCE_VERSION 1
#define WL_TOUCH_SHAPE_SINCE_VERSION 6
#define WL_TOUCH_ORIENTATION_SINCE_VERSION 6
static inline void wl_touch_destroy(struct wl_touch *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline void wl_touch_release(struct wl_touch *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
#define WL_OUTPUT_INTERFACE_VERSION 4
enum wl_output_subpixel {
WL_OUTPUT_SUBPIXEL_UNKNOWN = 0,
WL_OUTPUT_SUBPIXEL_NONE = 1,
WL_OUTPUT_SUBPIXEL_HORIZONTAL_RGB = 2,
WL_OUTPUT_SUBPIXEL_HORIZONTAL_BGR = 3,
WL_OUTPUT_SUBPIXEL_VERTICAL_RGB = 4,
WL_OUTPUT_SUBPIXEL_VERTICAL_BGR = 5,
};
enum wl_output_transform {
WL_OUTPUT_TRANSFORM_NORMAL = 0,
WL_OUTPUT_TRANSFORM_90 = 1,
WL_OUTPUT_TRANSFORM_180 = 2,
WL_OUTPUT_TRANSFORM_270 = 3,
WL_OUTPUT_TRANSFORM_FLIPPED = 4,
WL_OUTPUT_TRANSFORM_FLIPPED_90 = 5,
WL_OUTPUT_TRANSFORM_FLIPPED_180 = 6,
WL_OUTPUT_TRANSFORM_FLIPPED_270 = 7,
};
enum wl_output_mode {
WL_OUTPUT_MODE_CURRENT = 0x1,
WL_OUTPUT_MODE_PREFERRED = 0x2,
};
struct wl_output_listener {
 void (*geometry)(void *data, struct wl_output *object, int32_t x, int32_t y, int32_t physical_width, int32_t physical_height, int32_t subpixel, const char * make, const char * model, int32_t transform);
 void (*mode)(void *data, struct wl_output *object, uint32_t flags, int32_t width, int32_t height, int32_t refresh);
 void (*done)(void *data, struct wl_output *object);
 void (*scale)(void *data, struct wl_output *object, int32_t factor);
 void (*name)(void *data, struct wl_output *object, const char * name);
 void (*description)(void *data, struct wl_output *object, const char * description);
};
static inline int wl_output_add_listener(struct wl_output *o,const struct wl_output_listener *l,void *data){return wl_proxy_add_listener((struct wl_proxy*)o,(void (**)(void))l,data);}
static inline void wl_output_set_user_data(struct wl_output *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_output_get_user_data(struct wl_output *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_OUTPUT_RELEASE_SINCE_VERSION 3
#define WL_OUTPUT_GEOMETRY_SINCE_VERSION 1
#define WL_OUTPUT_MODE_SINCE_VERSION 1
#define WL_OUTPUT_DONE_SINCE_VERSION 2
#define WL_OUTPUT_SCALE_SINCE_VERSION 2
#define WL_OUTPUT_NAME_SINCE_VERSION 4
#define WL_OUTPUT_DESCRIPTION_SINCE_VERSION 4
static inline void wl_output_destroy(struct wl_output *o){wl_proxy_destroy((struct wl_proxy*)o);}
static inline void wl_output_release(struct wl_output *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
#define WL_REGION_INTERFACE_VERSION 1
static inline void wl_region_set_user_data(struct wl_region *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_region_get_user_data(struct wl_region *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_REGION_DESTROY_SINCE_VERSION 1
#define WL_REGION_ADD_SINCE_VERSION 1
#define WL_REGION_SUBTRACT_SINCE_VERSION 1
static inline void wl_region_destroy(struct wl_region *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void wl_region_add(struct wl_region *o, int32_t x, int32_t y, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,1,x, y, width, height);
}
static inline void wl_region_subtract(struct wl_region *o, int32_t x, int32_t y, int32_t width, int32_t height){
wl_proxy_marshal((struct wl_proxy*)o,2,x, y, width, height);
}
#define WL_SUBCOMPOSITOR_INTERFACE_VERSION 1
enum wl_subcompositor_error {
WL_SUBCOMPOSITOR_ERROR_BAD_SURFACE = 0,
};
static inline void wl_subcompositor_set_user_data(struct wl_subcompositor *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_subcompositor_get_user_data(struct wl_subcompositor *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SUBCOMPOSITOR_DESTROY_SINCE_VERSION 1
#define WL_SUBCOMPOSITOR_GET_SUBSURFACE_SINCE_VERSION 1
static inline void wl_subcompositor_destroy(struct wl_subcompositor *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline struct wl_subsurface * wl_subcompositor_get_subsurface(struct wl_subcompositor *o, struct wl_surface * surface, struct wl_surface * parent){
return (struct wl_subsurface *)wl_proxy_marshal_constructor_versioned((struct wl_proxy*)o,1,&wl_subsurface_interface,wl_proxy_get_version((struct wl_proxy*)o),NULL, (struct wl_proxy*)surface, (struct wl_proxy*)parent);
}
#define WL_SUBSURFACE_INTERFACE_VERSION 1
enum wl_subsurface_error {
WL_SUBSURFACE_ERROR_BAD_SURFACE = 0,
};
static inline void wl_subsurface_set_user_data(struct wl_subsurface *o,void*d){wl_proxy_set_user_data((struct wl_proxy*)o,d);}
static inline void *wl_subsurface_get_user_data(struct wl_subsurface *o){return wl_proxy_get_user_data((struct wl_proxy*)o);}
#define WL_SUBSURFACE_DESTROY_SINCE_VERSION 1
#define WL_SUBSURFACE_SET_POSITION_SINCE_VERSION 1
#define WL_SUBSURFACE_PLACE_ABOVE_SINCE_VERSION 1
#define WL_SUBSURFACE_PLACE_BELOW_SINCE_VERSION 1
#define WL_SUBSURFACE_SET_SYNC_SINCE_VERSION 1
#define WL_SUBSURFACE_SET_DESYNC_SINCE_VERSION 1
static inline void wl_subsurface_destroy(struct wl_subsurface *o){
wl_proxy_marshal((struct wl_proxy*)o,0);
wl_proxy_destroy((struct wl_proxy*)o);
}
static inline void wl_subsurface_set_position(struct wl_subsurface *o, int32_t x, int32_t y){
wl_proxy_marshal((struct wl_proxy*)o,1,x, y);
}
static inline void wl_subsurface_place_above(struct wl_subsurface *o, struct wl_surface * sibling){
wl_proxy_marshal((struct wl_proxy*)o,2,(struct wl_proxy*)sibling);
}
static inline void wl_subsurface_place_below(struct wl_subsurface *o, struct wl_surface * sibling){
wl_proxy_marshal((struct wl_proxy*)o,3,(struct wl_proxy*)sibling);
}
static inline void wl_subsurface_set_sync(struct wl_subsurface *o){
wl_proxy_marshal((struct wl_proxy*)o,4);
}
static inline void wl_subsurface_set_desync(struct wl_subsurface *o){
wl_proxy_marshal((struct wl_proxy*)o,5);
}
