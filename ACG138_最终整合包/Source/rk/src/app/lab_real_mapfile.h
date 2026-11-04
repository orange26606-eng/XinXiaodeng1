#ifndef W1_REAL_MAPFILE_H
#define W1_REAL_MAPFILE_H
#include "lab_localization.h"
/* REAL地图采用独立文件格式。 */
typedef struct lab_real_map_file {lab_real_grid_view view;lab_real_context original_context;uint32_t experimental_timing;uint64_t parameter_version;double mount_x_m,mount_y_m,mount_yaw_rad;int8_t *owned_cells;} lab_real_map_file;
lab_status lab_real_map_save_ex(const char *new_path,const lab_real_grid_view *map,const lab_localization_config *config,uint32_t experimental_timing);
lab_status lab_real_map_save(const char *new_path,const lab_real_grid_view *map,const lab_localization_config *config);
lab_status lab_real_map_load(const char *path,const lab_real_context *new_context,const lab_localization_config *config,lab_real_map_file *out);
void lab_real_map_file_close(lab_real_map_file *file);
#endif
