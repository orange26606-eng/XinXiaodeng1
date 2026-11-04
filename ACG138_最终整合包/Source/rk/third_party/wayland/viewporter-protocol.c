#include "viewporter-client-protocol.h"
static const struct wl_interface *wp_viewporter_requests_1_types[]={&wp_viewport_interface,&wl_surface_interface};
static const struct wl_message wp_viewporter_requests[]={
{"destroy","",NULL},
{"get_viewport","no",wp_viewporter_requests_1_types},
};
const struct wl_interface wp_viewporter_interface={"wp_viewporter",1,2,wp_viewporter_requests,0,NULL};
static const struct wl_interface *wp_viewport_requests_1_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wp_viewport_requests_2_types[]={NULL,NULL};
static const struct wl_message wp_viewport_requests[]={
{"destroy","",NULL},
{"set_source","ffff",wp_viewport_requests_1_types},
{"set_destination","ii",wp_viewport_requests_2_types},
};
const struct wl_interface wp_viewport_interface={"wp_viewport",1,3,wp_viewport_requests,0,NULL};
