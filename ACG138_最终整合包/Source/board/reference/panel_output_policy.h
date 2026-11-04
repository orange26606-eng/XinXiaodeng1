#ifndef PANEL_OUTPUT_POLICY_H
#define PANEL_OUTPUT_POLICY_H
#include <stddef.h>
#include <stdint.h>

struct panel_output_entry {
    uint32_t id;
    int present, ready;
    char name[80];
};
/* 返回输出数组索引，无符合条件的输出时返回-1，窗口模式返回-2；数组仅包含合成器公布的输出。
 */
int panel_output_choose(const struct panel_output_entry *entries, size_t count,
                        const char *mode, uint32_t current_id);
#endif
