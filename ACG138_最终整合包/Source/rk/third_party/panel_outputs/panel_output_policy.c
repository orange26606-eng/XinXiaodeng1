#include "panel_output_policy.h"
#include <string.h>
#include <strings.h>

static int priority(const char *name)
{
    if (!strncasecmp(name, "HDMI", 4)) return 3;
    if (!strncasecmp(name, "DSI", 3)) return 2;
    return 1;
}

int panel_output_choose(const struct panel_output_entry *entries, size_t count,
                        const char *mode, uint32_t current_id)
{
    if (mode && !strcmp(mode, "windowed")) return -2;
    int automatic = !mode || !*mode || !strcmp(mode, "auto");
    int best = -1, best_priority = -1;
    for (size_t i = 0; i < count; ++i) {
        const struct panel_output_entry *entry = &entries[i];
        if (!entry->present || !entry->ready) continue;
        if (!automatic) {
            if (!strcmp(entry->name, mode)) return (int)i;
            continue;
        }
        int rank = priority(entry->name);
        if (rank > best_priority ||
            (rank == best_priority && entry->id == current_id) ||
            (rank == best_priority && entries[best].id != current_id && entry->id < entries[best].id)) {
            best = (int)i;
            best_priority = rank;
        }
    }
    return best;
}
