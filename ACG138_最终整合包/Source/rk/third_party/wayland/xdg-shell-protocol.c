#include "xdg-shell-client-protocol.h"
static const struct wl_interface *xdg_wm_base_requests_1_types[]={&xdg_positioner_interface};
static const struct wl_interface *xdg_wm_base_requests_2_types[]={&xdg_surface_interface,&wl_surface_interface};
static const struct wl_interface *xdg_wm_base_requests_3_types[]={NULL};
static const struct wl_message xdg_wm_base_requests[]={
{"destroy","",NULL},
{"create_positioner","n",xdg_wm_base_requests_1_types},
{"get_xdg_surface","no",xdg_wm_base_requests_2_types},
{"pong","u",xdg_wm_base_requests_3_types},
};
static const struct wl_interface *xdg_wm_base_events_0_types[]={NULL};
static const struct wl_message xdg_wm_base_events[]={
{"ping","u",xdg_wm_base_events_0_types},
};
const struct wl_interface xdg_wm_base_interface={"xdg_wm_base",7,4,xdg_wm_base_requests,1,xdg_wm_base_events};
static const struct wl_interface *xdg_positioner_requests_1_types[]={NULL,NULL};
static const struct wl_interface *xdg_positioner_requests_2_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *xdg_positioner_requests_3_types[]={NULL};
static const struct wl_interface *xdg_positioner_requests_4_types[]={NULL};
static const struct wl_interface *xdg_positioner_requests_5_types[]={NULL};
static const struct wl_interface *xdg_positioner_requests_6_types[]={NULL,NULL};
static const struct wl_interface *xdg_positioner_requests_8_types[]={NULL,NULL};
static const struct wl_interface *xdg_positioner_requests_9_types[]={NULL};
static const struct wl_message xdg_positioner_requests[]={
{"destroy","",NULL},
{"set_size","ii",xdg_positioner_requests_1_types},
{"set_anchor_rect","iiii",xdg_positioner_requests_2_types},
{"set_anchor","u",xdg_positioner_requests_3_types},
{"set_gravity","u",xdg_positioner_requests_4_types},
{"set_constraint_adjustment","u",xdg_positioner_requests_5_types},
{"set_offset","ii",xdg_positioner_requests_6_types},
{"set_reactive","3",NULL},
{"set_parent_size","3ii",xdg_positioner_requests_8_types},
{"set_parent_configure","3u",xdg_positioner_requests_9_types},
};
const struct wl_interface xdg_positioner_interface={"xdg_positioner",7,10,xdg_positioner_requests,0,NULL};
static const struct wl_interface *xdg_surface_requests_1_types[]={&xdg_toplevel_interface};
static const struct wl_interface *xdg_surface_requests_2_types[]={&xdg_popup_interface,&xdg_surface_interface,&xdg_positioner_interface};
static const struct wl_interface *xdg_surface_requests_3_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *xdg_surface_requests_4_types[]={NULL};
static const struct wl_message xdg_surface_requests[]={
{"destroy","",NULL},
{"get_toplevel","n",xdg_surface_requests_1_types},
{"get_popup","n?oo",xdg_surface_requests_2_types},
{"set_window_geometry","iiii",xdg_surface_requests_3_types},
{"ack_configure","u",xdg_surface_requests_4_types},
};
static const struct wl_interface *xdg_surface_events_0_types[]={NULL};
static const struct wl_message xdg_surface_events[]={
{"configure","u",xdg_surface_events_0_types},
};
const struct wl_interface xdg_surface_interface={"xdg_surface",7,5,xdg_surface_requests,1,xdg_surface_events};
static const struct wl_interface *xdg_toplevel_requests_1_types[]={&xdg_toplevel_interface};
static const struct wl_interface *xdg_toplevel_requests_2_types[]={NULL};
static const struct wl_interface *xdg_toplevel_requests_3_types[]={NULL};
static const struct wl_interface *xdg_toplevel_requests_4_types[]={&wl_seat_interface,NULL,NULL,NULL};
static const struct wl_interface *xdg_toplevel_requests_5_types[]={&wl_seat_interface,NULL};
static const struct wl_interface *xdg_toplevel_requests_6_types[]={&wl_seat_interface,NULL,NULL};
static const struct wl_interface *xdg_toplevel_requests_7_types[]={NULL,NULL};
static const struct wl_interface *xdg_toplevel_requests_8_types[]={NULL,NULL};
static const struct wl_interface *xdg_toplevel_requests_11_types[]={&wl_output_interface};
static const struct wl_message xdg_toplevel_requests[]={
{"destroy","",NULL},
{"set_parent","?o",xdg_toplevel_requests_1_types},
{"set_title","s",xdg_toplevel_requests_2_types},
{"set_app_id","s",xdg_toplevel_requests_3_types},
{"show_window_menu","ouii",xdg_toplevel_requests_4_types},
{"move","ou",xdg_toplevel_requests_5_types},
{"resize","ouu",xdg_toplevel_requests_6_types},
{"set_max_size","ii",xdg_toplevel_requests_7_types},
{"set_min_size","ii",xdg_toplevel_requests_8_types},
{"set_maximized","",NULL},
{"unset_maximized","",NULL},
{"set_fullscreen","?o",xdg_toplevel_requests_11_types},
{"unset_fullscreen","",NULL},
{"set_minimized","",NULL},
};
static const struct wl_interface *xdg_toplevel_events_0_types[]={NULL,NULL,NULL};
static const struct wl_interface *xdg_toplevel_events_2_types[]={NULL,NULL};
static const struct wl_interface *xdg_toplevel_events_3_types[]={NULL};
static const struct wl_message xdg_toplevel_events[]={
{"configure","iia",xdg_toplevel_events_0_types},
{"close","",NULL},
{"configure_bounds","4ii",xdg_toplevel_events_2_types},
{"wm_capabilities","5a",xdg_toplevel_events_3_types},
};
const struct wl_interface xdg_toplevel_interface={"xdg_toplevel",7,14,xdg_toplevel_requests,4,xdg_toplevel_events};
static const struct wl_interface *xdg_popup_requests_1_types[]={&wl_seat_interface,NULL};
static const struct wl_interface *xdg_popup_requests_2_types[]={&xdg_positioner_interface,NULL};
static const struct wl_message xdg_popup_requests[]={
{"destroy","",NULL},
{"grab","ou",xdg_popup_requests_1_types},
{"reposition","3ou",xdg_popup_requests_2_types},
};
static const struct wl_interface *xdg_popup_events_0_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *xdg_popup_events_2_types[]={NULL};
static const struct wl_message xdg_popup_events[]={
{"configure","iiii",xdg_popup_events_0_types},
{"popup_done","",NULL},
{"repositioned","3u",xdg_popup_events_2_types},
};
const struct wl_interface xdg_popup_interface={"xdg_popup",7,3,xdg_popup_requests,3,xdg_popup_events};
