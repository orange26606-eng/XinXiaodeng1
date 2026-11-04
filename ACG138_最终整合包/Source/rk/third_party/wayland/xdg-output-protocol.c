#include "xdg-output-client-protocol.h"
static const struct wl_interface *zxdg_output_manager_v1_requests_1_types[]={&zxdg_output_v1_interface,&wl_output_interface};
static const struct wl_message zxdg_output_manager_v1_requests[]={
{"destroy","",NULL},
{"get_xdg_output","no",zxdg_output_manager_v1_requests_1_types},
};
const struct wl_interface zxdg_output_manager_v1_interface={"zxdg_output_manager_v1",3,2,zxdg_output_manager_v1_requests,0,NULL};
static const struct wl_message zxdg_output_v1_requests[]={
{"destroy","",NULL},
};
static const struct wl_interface *zxdg_output_v1_events_0_types[]={NULL,NULL};
static const struct wl_interface *zxdg_output_v1_events_1_types[]={NULL,NULL};
static const struct wl_interface *zxdg_output_v1_events_3_types[]={NULL};
static const struct wl_interface *zxdg_output_v1_events_4_types[]={NULL};
static const struct wl_message zxdg_output_v1_events[]={
{"logical_position","ii",zxdg_output_v1_events_0_types},
{"logical_size","ii",zxdg_output_v1_events_1_types},
{"done","",NULL},
{"name","2s",zxdg_output_v1_events_3_types},
{"description","2s",zxdg_output_v1_events_4_types},
};
const struct wl_interface zxdg_output_v1_interface={"zxdg_output_v1",3,1,zxdg_output_v1_requests,5,zxdg_output_v1_events};
