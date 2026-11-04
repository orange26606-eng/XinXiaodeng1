#include "panel_outputs.h"
#include <stdio.h>
#include <string.h>

static void named(struct panel_output_slot *slot, const char *name)
{
    if (!slot->wl || !name) return;
    struct panel_output_entry *entry = &slot->owner->entries[slot->index];
    snprintf(entry->name, sizeof(entry->name), "%s", name);
    slot->owner->dirty = 1;
}
static void geometry(void *data, struct wl_output *output, int32_t x, int32_t y,
    int32_t width, int32_t height, int32_t subpixel, const char *make,
    const char *model, int32_t transform)
{
    (void)output; (void)x; (void)y; (void)width; (void)height; (void)subpixel;
    (void)make; (void)model; (void)transform;
    struct panel_output_slot *slot = data;
    if (slot->version < 2) {
        slot->owner->entries[slot->index].ready = 1;
        slot->owner->dirty = 1;
    }
}
static void mode(void *data, struct wl_output *output, uint32_t flags,
                 int32_t width, int32_t height, int32_t refresh)
{ (void)data; (void)output; (void)flags; (void)width; (void)height; (void)refresh; }
static void done(void *data, struct wl_output *output)
{
    (void)output; struct panel_output_slot *slot = data;
    slot->owner->entries[slot->index].ready = 1; slot->owner->dirty = 1;
}
static void scale(void *data, struct wl_output *output, int32_t factor)
{ (void)data; (void)output; (void)factor; }
static void name(void *data, struct wl_output *output, const char *value)
{ (void)output; named(data, value); }
static void description(void *data, struct wl_output *output, const char *value)
{ (void)data; (void)output; (void)value; }
static const struct wl_output_listener output_listener = {
    .geometry=geometry, .mode=mode, .done=done, .scale=scale,
    .name=name, .description=description
};
static void logical_position(void *data, struct zxdg_output_v1 *output, int32_t x, int32_t y)
{ (void)data; (void)output; (void)x; (void)y; }
static void logical_size(void *data, struct zxdg_output_v1 *output, int32_t w, int32_t h)
{ (void)data; (void)output; (void)w; (void)h; }
static void logical_done(void *data, struct zxdg_output_v1 *output)
{ (void)data; (void)output; }
static void logical_name(void *data, struct zxdg_output_v1 *output, const char *value)
{ (void)output; named(data, value); }
static void logical_description(void *data, struct zxdg_output_v1 *output, const char *value)
{ (void)data; (void)output; (void)value; }
static const struct zxdg_output_v1_listener logical_listener = {
    .logical_position=logical_position, .logical_size=logical_size, .done=logical_done,
    .name=logical_name, .description=logical_description
};
static void attach_logical(struct panel_outputs *state, size_t i)
{
    struct panel_output_slot *slot = &state->slots[i];
    if (state->manager && slot->wl && !slot->xdg) {
        slot->xdg = zxdg_output_manager_v1_get_xdg_output(state->manager, slot->wl);
        if (slot->xdg) zxdg_output_v1_add_listener(slot->xdg, &logical_listener, slot);
    }
}
static void clear_slot(struct panel_outputs *state, size_t i)
{
    struct panel_output_slot *slot = &state->slots[i];
    if (slot->xdg) zxdg_output_v1_destroy(slot->xdg);
    if (slot->wl) {
        if (slot->version >= WL_OUTPUT_RELEASE_SINCE_VERSION) wl_output_release(slot->wl);
        else wl_output_destroy(slot->wl);
    }
    memset(slot, 0, sizeof(*slot));
    memset(&state->entries[i], 0, sizeof(state->entries[i]));
    state->dirty = 1;
}
void panel_outputs_init(struct panel_outputs *state, const char *mode_value)
{
    memset(state, 0, sizeof(*state));
    state->mode = mode_value; state->dirty = 1;
}
void panel_outputs_global(struct panel_outputs *state, struct wl_registry *registry,
                          uint32_t id, const char *interface, uint32_t version)
{
    if (!strcmp(interface, "zxdg_output_manager_v1") && !state->manager) {
        state->manager = wl_registry_bind(registry, id, &zxdg_output_manager_v1_interface, version < 3 ? version : 3);
        state->manager_id = id;
        for (size_t i = 0; i < PANEL_OUTPUT_LIMIT; ++i) attach_logical(state, i);
    } else if (!strcmp(interface, "wl_output") && version >= 1) {
        size_t i;
        for (i = 0; i < PANEL_OUTPUT_LIMIT; ++i) if (!state->slots[i].wl) break;
        if (i == PANEL_OUTPUT_LIMIT) {
            fprintf(stderr, "Wayland output registry capacity exceeded; ignoring id=%u\n", id);
            return;
        }
        struct panel_output_slot *slot = &state->slots[i];
        slot->owner = state; slot->index = i; slot->version = version < 4 ? version : 4;
        slot->generation = ++state->next_generation;
        slot->wl = wl_registry_bind(registry, id, &wl_output_interface, slot->version);
        if (!slot->wl) { clear_slot(state, i); return; }
        state->entries[i].id = id; state->entries[i].present = 1;
        wl_output_add_listener(slot->wl, &output_listener, slot);
        attach_logical(state, i); state->dirty = 1;
    }
}
void panel_outputs_remove(struct panel_outputs *state, uint32_t id)
{
    if (state->manager && id == state->manager_id) {
        for (size_t i = 0; i < PANEL_OUTPUT_LIMIT; ++i) {
            if (state->slots[i].xdg) zxdg_output_v1_destroy(state->slots[i].xdg);
            state->slots[i].xdg = NULL;
        }
        zxdg_output_manager_v1_destroy(state->manager);
        state->manager = NULL; state->manager_id = 0; state->dirty = 1;
    }
    for (size_t i = 0; i < PANEL_OUTPUT_LIMIT; ++i)
        if (state->entries[i].present && state->entries[i].id == id) clear_slot(state, i);
}
int panel_outputs_update(struct panel_outputs *state, struct panel_output_selection *selection)
{
    if (!state->dirty && state->applied) return 0;
    state->dirty = 0;
    int index = panel_output_choose(state->entries, PANEL_OUTPUT_LIMIT, state->mode, state->current_id);
    uint32_t id = index >= 0 ? state->entries[index].id : 0;
    unsigned generation = index >= 0 ? state->slots[index].generation : 0;
    if (state->applied && id == state->current_id && generation == state->current_generation) return 0;
    state->applied = 1; state->current_id = id; state->current_generation = generation;
    selection->wl = index >= 0 ? state->slots[index].wl : NULL;
    selection->id = id;
    selection->name = index >= 0 ? state->entries[index].name : "";
    selection->windowed = index == -2;
    return 1;
}
void panel_outputs_finish(struct panel_outputs *state)
{
    for (size_t i = 0; i < PANEL_OUTPUT_LIMIT; ++i) clear_slot(state, i);
    if (state->manager) zxdg_output_manager_v1_destroy(state->manager);
    state->manager = NULL;
}
