#ifndef PANEL_OUTPUTS_H
#define PANEL_OUTPUTS_H
#include "panel_output_policy.h"
#include <wayland-client.h>
#include "xdg-output-client-protocol.h"
#define PANEL_OUTPUT_LIMIT 16

struct panel_outputs;
struct panel_output_slot {
    struct panel_outputs *owner;
    struct wl_output *wl;
    struct zxdg_output_v1 *xdg;
    unsigned version, generation;
    size_t index;
};
struct panel_outputs {
    struct panel_output_entry entries[PANEL_OUTPUT_LIMIT];
    struct panel_output_slot slots[PANEL_OUTPUT_LIMIT];
    struct zxdg_output_manager_v1 *manager;
    uint32_t manager_id, current_id;
    unsigned next_generation, current_generation;
    const char *mode;
    int dirty, applied;
};
struct panel_output_selection {
    struct wl_output *wl;
    uint32_t id;
    const char *name;
    int windowed;
};
void panel_outputs_init(struct panel_outputs *state, const char *mode);
void panel_outputs_global(struct panel_outputs *state, struct wl_registry *registry,
                          uint32_t id, const char *interface, uint32_t version);
void panel_outputs_remove(struct panel_outputs *state, uint32_t id);
/* 输出事件分发后返回是否需更新全屏状态。 */

int panel_outputs_update(struct panel_outputs *state, struct panel_output_selection *selection);
void panel_outputs_finish(struct panel_outputs *state);
#endif
