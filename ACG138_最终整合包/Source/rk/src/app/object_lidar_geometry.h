#ifndef ACG_OBJECT_LIDAR_GEOMETRY_H
#define ACG_OBJECT_LIDAR_GEOMETRY_H
#include "object_lidar.h"
typedef struct ol_config {
 double enabled,calibration_enabled,reference_eye,image_width,image_height;
 double fx,fy,cx,cy,k1,k2,p1,p2,k3,camera_to_body_verified;
 double camera_x_m,camera_y_m,camera_z_m,camera_roll_deg,camera_pitch_deg,camera_yaw_deg,camera_delay_ms;
 double lidar_z_m,lidar_height_verified,frame_max_age_ms,scan_max_age_ms,pair_max_delta_ms,confirm_frames;
 double cluster_gap_m,min_cluster_points,min_pixel_span,min_bbox_fraction,max_pose_uncertainty_m,max_time_uncertainty_ms;
 double temporal_gate_m,bbox_margin_px;
} ol_config;
int ol_config_load(const char*,ol_config*);
/* 按扫描历史校正地图坐标，将扫描点投影到历史图像。 */
int ol_project(const ol_config*,const passage_config*,const lab_pose*scan_pose,const lab_pose*image_pose,
 double range,double angle,double *u,double *v,double *map_x,double *map_y);
#endif
