#ifndef W2_LIVE_CAPTURE_CSV_H
#define W2_LIVE_CAPTURE_CSV_H
#include "live_sensor.h"
#include <stdio.h>
lab_status li_capture_csv_header(FILE *file);
lab_status li_capture_csv_pose(FILE *file,const li_pose_sample *sample);
#endif
