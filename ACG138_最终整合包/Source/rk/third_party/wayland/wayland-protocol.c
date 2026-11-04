#include "wayland-client-protocol.h"
static const struct wl_interface *wl_display_requests_0_types[]={&wl_callback_interface};
static const struct wl_interface *wl_display_requests_1_types[]={&wl_registry_interface};
static const struct wl_message wl_display_requests[]={
{"sync","n",wl_display_requests_0_types},
{"get_registry","n",wl_display_requests_1_types},
};
static const struct wl_interface *wl_display_events_0_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_display_events_1_types[]={NULL};
static const struct wl_message wl_display_events[]={
{"error","ous",wl_display_events_0_types},
{"delete_id","u",wl_display_events_1_types},
};
const struct wl_interface wl_display_interface={"wl_display",1,2,wl_display_requests,2,wl_display_events};
static const struct wl_interface *wl_registry_requests_0_types[]={NULL,NULL};
static const struct wl_message wl_registry_requests[]={
{"bind","un",wl_registry_requests_0_types},
};
static const struct wl_interface *wl_registry_events_0_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_registry_events_1_types[]={NULL};
static const struct wl_message wl_registry_events[]={
{"global","usu",wl_registry_events_0_types},
{"global_remove","u",wl_registry_events_1_types},
};
const struct wl_interface wl_registry_interface={"wl_registry",1,1,wl_registry_requests,2,wl_registry_events};
static const struct wl_interface *wl_callback_events_0_types[]={NULL};
static const struct wl_message wl_callback_events[]={
{"done","u",wl_callback_events_0_types},
};
const struct wl_interface wl_callback_interface={"wl_callback",1,0,NULL,1,wl_callback_events};
static const struct wl_interface *wl_compositor_requests_0_types[]={&wl_surface_interface};
static const struct wl_interface *wl_compositor_requests_1_types[]={&wl_region_interface};
static const struct wl_message wl_compositor_requests[]={
{"create_surface","n",wl_compositor_requests_0_types},
{"create_region","n",wl_compositor_requests_1_types},
};
const struct wl_interface wl_compositor_interface={"wl_compositor",5,2,wl_compositor_requests,0,NULL};
static const struct wl_interface *wl_shm_pool_requests_0_types[]={&wl_buffer_interface,NULL,NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_shm_pool_requests_2_types[]={NULL};
static const struct wl_message wl_shm_pool_requests[]={
{"create_buffer","niiiiu",wl_shm_pool_requests_0_types},
{"destroy","",NULL},
{"resize","i",wl_shm_pool_requests_2_types},
};
const struct wl_interface wl_shm_pool_interface={"wl_shm_pool",1,3,wl_shm_pool_requests,0,NULL};
static const struct wl_interface *wl_shm_requests_0_types[]={&wl_shm_pool_interface,NULL,NULL};
static const struct wl_message wl_shm_requests[]={
{"create_pool","nhi",wl_shm_requests_0_types},
};
static const struct wl_interface *wl_shm_events_0_types[]={NULL};
static const struct wl_message wl_shm_events[]={
{"format","u",wl_shm_events_0_types},
};
const struct wl_interface wl_shm_interface={"wl_shm",1,1,wl_shm_requests,1,wl_shm_events};
static const struct wl_message wl_buffer_requests[]={
{"destroy","",NULL},
};
static const struct wl_message wl_buffer_events[]={
{"release","",NULL},
};
const struct wl_interface wl_buffer_interface={"wl_buffer",1,1,wl_buffer_requests,1,wl_buffer_events};
static const struct wl_interface *wl_data_offer_requests_0_types[]={NULL,NULL};
static const struct wl_interface *wl_data_offer_requests_1_types[]={NULL,NULL};
static const struct wl_interface *wl_data_offer_requests_4_types[]={NULL,NULL};
static const struct wl_message wl_data_offer_requests[]={
{"accept","u?s",wl_data_offer_requests_0_types},
{"receive","sh",wl_data_offer_requests_1_types},
{"destroy","",NULL},
{"finish","3",NULL},
{"set_actions","3uu",wl_data_offer_requests_4_types},
};
static const struct wl_interface *wl_data_offer_events_0_types[]={NULL};
static const struct wl_interface *wl_data_offer_events_1_types[]={NULL};
static const struct wl_interface *wl_data_offer_events_2_types[]={NULL};
static const struct wl_message wl_data_offer_events[]={
{"offer","s",wl_data_offer_events_0_types},
{"source_actions","3u",wl_data_offer_events_1_types},
{"action","3u",wl_data_offer_events_2_types},
};
const struct wl_interface wl_data_offer_interface={"wl_data_offer",3,5,wl_data_offer_requests,3,wl_data_offer_events};
static const struct wl_interface *wl_data_source_requests_0_types[]={NULL};
static const struct wl_interface *wl_data_source_requests_2_types[]={NULL};
static const struct wl_message wl_data_source_requests[]={
{"offer","s",wl_data_source_requests_0_types},
{"destroy","",NULL},
{"set_actions","3u",wl_data_source_requests_2_types},
};
static const struct wl_interface *wl_data_source_events_0_types[]={NULL};
static const struct wl_interface *wl_data_source_events_1_types[]={NULL,NULL};
static const struct wl_interface *wl_data_source_events_5_types[]={NULL};
static const struct wl_message wl_data_source_events[]={
{"target","?s",wl_data_source_events_0_types},
{"send","sh",wl_data_source_events_1_types},
{"cancelled","",NULL},
{"dnd_drop_performed","3",NULL},
{"dnd_finished","3",NULL},
{"action","3u",wl_data_source_events_5_types},
};
const struct wl_interface wl_data_source_interface={"wl_data_source",3,3,wl_data_source_requests,6,wl_data_source_events};
static const struct wl_interface *wl_data_device_requests_0_types[]={&wl_data_source_interface,&wl_surface_interface,&wl_surface_interface,NULL};
static const struct wl_interface *wl_data_device_requests_1_types[]={&wl_data_source_interface,NULL};
static const struct wl_message wl_data_device_requests[]={
{"start_drag","?oo?ou",wl_data_device_requests_0_types},
{"set_selection","?ou",wl_data_device_requests_1_types},
{"release","2",NULL},
};
static const struct wl_interface *wl_data_device_events_0_types[]={&wl_data_offer_interface};
static const struct wl_interface *wl_data_device_events_1_types[]={NULL,&wl_surface_interface,NULL,NULL,&wl_data_offer_interface};
static const struct wl_interface *wl_data_device_events_3_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_data_device_events_5_types[]={&wl_data_offer_interface};
static const struct wl_message wl_data_device_events[]={
{"data_offer","n",wl_data_device_events_0_types},
{"enter","uoff?o",wl_data_device_events_1_types},
{"leave","",NULL},
{"motion","uff",wl_data_device_events_3_types},
{"drop","",NULL},
{"selection","?o",wl_data_device_events_5_types},
};
const struct wl_interface wl_data_device_interface={"wl_data_device",3,3,wl_data_device_requests,6,wl_data_device_events};
static const struct wl_interface *wl_data_device_manager_requests_0_types[]={&wl_data_source_interface};
static const struct wl_interface *wl_data_device_manager_requests_1_types[]={&wl_data_device_interface,&wl_seat_interface};
static const struct wl_message wl_data_device_manager_requests[]={
{"create_data_source","n",wl_data_device_manager_requests_0_types},
{"get_data_device","no",wl_data_device_manager_requests_1_types},
};
const struct wl_interface wl_data_device_manager_interface={"wl_data_device_manager",3,2,wl_data_device_manager_requests,0,NULL};
static const struct wl_interface *wl_shell_requests_0_types[]={&wl_shell_surface_interface,&wl_surface_interface};
static const struct wl_message wl_shell_requests[]={
{"get_shell_surface","no",wl_shell_requests_0_types},
};
const struct wl_interface wl_shell_interface={"wl_shell",1,1,wl_shell_requests,0,NULL};
static const struct wl_interface *wl_shell_surface_requests_0_types[]={NULL};
static const struct wl_interface *wl_shell_surface_requests_1_types[]={&wl_seat_interface,NULL};
static const struct wl_interface *wl_shell_surface_requests_2_types[]={&wl_seat_interface,NULL,NULL};
static const struct wl_interface *wl_shell_surface_requests_4_types[]={&wl_surface_interface,NULL,NULL,NULL};
static const struct wl_interface *wl_shell_surface_requests_5_types[]={NULL,NULL,&wl_output_interface};
static const struct wl_interface *wl_shell_surface_requests_6_types[]={&wl_seat_interface,NULL,&wl_surface_interface,NULL,NULL,NULL};
static const struct wl_interface *wl_shell_surface_requests_7_types[]={&wl_output_interface};
static const struct wl_interface *wl_shell_surface_requests_8_types[]={NULL};
static const struct wl_interface *wl_shell_surface_requests_9_types[]={NULL};
static const struct wl_message wl_shell_surface_requests[]={
{"pong","u",wl_shell_surface_requests_0_types},
{"move","ou",wl_shell_surface_requests_1_types},
{"resize","ouu",wl_shell_surface_requests_2_types},
{"set_toplevel","",NULL},
{"set_transient","oiiu",wl_shell_surface_requests_4_types},
{"set_fullscreen","uu?o",wl_shell_surface_requests_5_types},
{"set_popup","ouoiiu",wl_shell_surface_requests_6_types},
{"set_maximized","?o",wl_shell_surface_requests_7_types},
{"set_title","s",wl_shell_surface_requests_8_types},
{"set_class","s",wl_shell_surface_requests_9_types},
};
static const struct wl_interface *wl_shell_surface_events_0_types[]={NULL};
static const struct wl_interface *wl_shell_surface_events_1_types[]={NULL,NULL,NULL};
static const struct wl_message wl_shell_surface_events[]={
{"ping","u",wl_shell_surface_events_0_types},
{"configure","uii",wl_shell_surface_events_1_types},
{"popup_done","",NULL},
};
const struct wl_interface wl_shell_surface_interface={"wl_shell_surface",1,10,wl_shell_surface_requests,3,wl_shell_surface_events};
static const struct wl_interface *wl_surface_requests_1_types[]={&wl_buffer_interface,NULL,NULL};
static const struct wl_interface *wl_surface_requests_2_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_surface_requests_3_types[]={&wl_callback_interface};
static const struct wl_interface *wl_surface_requests_4_types[]={&wl_region_interface};
static const struct wl_interface *wl_surface_requests_5_types[]={&wl_region_interface};
static const struct wl_interface *wl_surface_requests_7_types[]={NULL};
static const struct wl_interface *wl_surface_requests_8_types[]={NULL};
static const struct wl_interface *wl_surface_requests_9_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_surface_requests_10_types[]={NULL,NULL};
static const struct wl_message wl_surface_requests[]={
{"destroy","",NULL},
{"attach","?oii",wl_surface_requests_1_types},
{"damage","iiii",wl_surface_requests_2_types},
{"frame","n",wl_surface_requests_3_types},
{"set_opaque_region","?o",wl_surface_requests_4_types},
{"set_input_region","?o",wl_surface_requests_5_types},
{"commit","",NULL},
{"set_buffer_transform","2i",wl_surface_requests_7_types},
{"set_buffer_scale","3i",wl_surface_requests_8_types},
{"damage_buffer","4iiii",wl_surface_requests_9_types},
{"offset","5ii",wl_surface_requests_10_types},
};
static const struct wl_interface *wl_surface_events_0_types[]={&wl_output_interface};
static const struct wl_interface *wl_surface_events_1_types[]={&wl_output_interface};
static const struct wl_message wl_surface_events[]={
{"enter","o",wl_surface_events_0_types},
{"leave","o",wl_surface_events_1_types},
};
const struct wl_interface wl_surface_interface={"wl_surface",5,11,wl_surface_requests,2,wl_surface_events};
static const struct wl_interface *wl_seat_requests_0_types[]={&wl_pointer_interface};
static const struct wl_interface *wl_seat_requests_1_types[]={&wl_keyboard_interface};
static const struct wl_interface *wl_seat_requests_2_types[]={&wl_touch_interface};
static const struct wl_message wl_seat_requests[]={
{"get_pointer","n",wl_seat_requests_0_types},
{"get_keyboard","n",wl_seat_requests_1_types},
{"get_touch","n",wl_seat_requests_2_types},
{"release","5",NULL},
};
static const struct wl_interface *wl_seat_events_0_types[]={NULL};
static const struct wl_interface *wl_seat_events_1_types[]={NULL};
static const struct wl_message wl_seat_events[]={
{"capabilities","u",wl_seat_events_0_types},
{"name","2s",wl_seat_events_1_types},
};
const struct wl_interface wl_seat_interface={"wl_seat",8,4,wl_seat_requests,2,wl_seat_events};
static const struct wl_interface *wl_pointer_requests_0_types[]={NULL,&wl_surface_interface,NULL,NULL};
static const struct wl_message wl_pointer_requests[]={
{"set_cursor","u?oii",wl_pointer_requests_0_types},
{"release","3",NULL},
};
static const struct wl_interface *wl_pointer_events_0_types[]={NULL,&wl_surface_interface,NULL,NULL};
static const struct wl_interface *wl_pointer_events_1_types[]={NULL,&wl_surface_interface};
static const struct wl_interface *wl_pointer_events_2_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_pointer_events_3_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_pointer_events_4_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_pointer_events_6_types[]={NULL};
static const struct wl_interface *wl_pointer_events_7_types[]={NULL,NULL};
static const struct wl_interface *wl_pointer_events_8_types[]={NULL,NULL};
static const struct wl_interface *wl_pointer_events_9_types[]={NULL,NULL};
static const struct wl_message wl_pointer_events[]={
{"enter","uoff",wl_pointer_events_0_types},
{"leave","uo",wl_pointer_events_1_types},
{"motion","uff",wl_pointer_events_2_types},
{"button","uuuu",wl_pointer_events_3_types},
{"axis","uuf",wl_pointer_events_4_types},
{"frame","5",NULL},
{"axis_source","5u",wl_pointer_events_6_types},
{"axis_stop","5uu",wl_pointer_events_7_types},
{"axis_discrete","5ui",wl_pointer_events_8_types},
{"axis_value120","8ui",wl_pointer_events_9_types},
};
const struct wl_interface wl_pointer_interface={"wl_pointer",8,2,wl_pointer_requests,10,wl_pointer_events};
static const struct wl_message wl_keyboard_requests[]={
{"release","3",NULL},
};
static const struct wl_interface *wl_keyboard_events_0_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_keyboard_events_1_types[]={NULL,&wl_surface_interface,NULL};
static const struct wl_interface *wl_keyboard_events_2_types[]={NULL,&wl_surface_interface};
static const struct wl_interface *wl_keyboard_events_3_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_keyboard_events_4_types[]={NULL,NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_keyboard_events_5_types[]={NULL,NULL};
static const struct wl_message wl_keyboard_events[]={
{"keymap","uhu",wl_keyboard_events_0_types},
{"enter","uoa",wl_keyboard_events_1_types},
{"leave","uo",wl_keyboard_events_2_types},
{"key","uuuu",wl_keyboard_events_3_types},
{"modifiers","uuuuu",wl_keyboard_events_4_types},
{"repeat_info","4ii",wl_keyboard_events_5_types},
};
const struct wl_interface wl_keyboard_interface={"wl_keyboard",8,1,wl_keyboard_requests,6,wl_keyboard_events};
static const struct wl_message wl_touch_requests[]={
{"release","3",NULL},
};
static const struct wl_interface *wl_touch_events_0_types[]={NULL,NULL,&wl_surface_interface,NULL,NULL,NULL};
static const struct wl_interface *wl_touch_events_1_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_touch_events_2_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_touch_events_5_types[]={NULL,NULL,NULL};
static const struct wl_interface *wl_touch_events_6_types[]={NULL,NULL};
static const struct wl_message wl_touch_events[]={
{"down","uuoiff",wl_touch_events_0_types},
{"up","uui",wl_touch_events_1_types},
{"motion","uiff",wl_touch_events_2_types},
{"frame","",NULL},
{"cancel","",NULL},
{"shape","6iff",wl_touch_events_5_types},
{"orientation","6if",wl_touch_events_6_types},
};
const struct wl_interface wl_touch_interface={"wl_touch",8,1,wl_touch_requests,7,wl_touch_events};
static const struct wl_message wl_output_requests[]={
{"release","3",NULL},
};
static const struct wl_interface *wl_output_events_0_types[]={NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_output_events_1_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_output_events_3_types[]={NULL};
static const struct wl_interface *wl_output_events_4_types[]={NULL};
static const struct wl_interface *wl_output_events_5_types[]={NULL};
static const struct wl_message wl_output_events[]={
{"geometry","iiiiissi",wl_output_events_0_types},
{"mode","uiii",wl_output_events_1_types},
{"done","2",NULL},
{"scale","2i",wl_output_events_3_types},
{"name","4s",wl_output_events_4_types},
{"description","4s",wl_output_events_5_types},
};
const struct wl_interface wl_output_interface={"wl_output",4,1,wl_output_requests,6,wl_output_events};
static const struct wl_interface *wl_region_requests_1_types[]={NULL,NULL,NULL,NULL};
static const struct wl_interface *wl_region_requests_2_types[]={NULL,NULL,NULL,NULL};
static const struct wl_message wl_region_requests[]={
{"destroy","",NULL},
{"add","iiii",wl_region_requests_1_types},
{"subtract","iiii",wl_region_requests_2_types},
};
const struct wl_interface wl_region_interface={"wl_region",1,3,wl_region_requests,0,NULL};
static const struct wl_interface *wl_subcompositor_requests_1_types[]={&wl_subsurface_interface,&wl_surface_interface,&wl_surface_interface};
static const struct wl_message wl_subcompositor_requests[]={
{"destroy","",NULL},
{"get_subsurface","noo",wl_subcompositor_requests_1_types},
};
const struct wl_interface wl_subcompositor_interface={"wl_subcompositor",1,2,wl_subcompositor_requests,0,NULL};
static const struct wl_interface *wl_subsurface_requests_1_types[]={NULL,NULL};
static const struct wl_interface *wl_subsurface_requests_2_types[]={&wl_surface_interface};
static const struct wl_interface *wl_subsurface_requests_3_types[]={&wl_surface_interface};
static const struct wl_message wl_subsurface_requests[]={
{"destroy","",NULL},
{"set_position","ii",wl_subsurface_requests_1_types},
{"place_above","o",wl_subsurface_requests_2_types},
{"place_below","o",wl_subsurface_requests_3_types},
{"set_sync","",NULL},
{"set_desync","",NULL},
};
const struct wl_interface wl_subsurface_interface={"wl_subsurface",1,6,wl_subsurface_requests,0,NULL};
