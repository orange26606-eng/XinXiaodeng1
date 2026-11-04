#ifndef PANEL_OUTPUT_POLICY_H
#define PANEL_OUTPUT_POLICY_H
#include <stddef.h>
#include <stdint.h>

struct panel_output_entry {
    uint32_t id;
    int present, ready;
    char name[80];
};
/* Returns an array index, -1 while no eligible output exists, or -2 for
 * windowed mode. Only outputs announced by the compositor enter this array. */
int panel_output_choose(const struct panel_output_entry *entries, size_t count,
                        const char *mode, uint32_t current_id);
#endif
