#include "live_capture_csv.h"
#include <inttypes.h>
lab_status li_capture_csv_header(FILE *f){
 if(!f)return LAB_ERR_ARGUMENT;
 return fputs("time_us,m1,m2,m3,m4,encoder_valid_mask,counts_per_revolution,session,origin_epoch,odom_x_m,odom_y_m,odom_yaw_rad,odom_quality,pose_quality,received_us,time_basis,encoder_mask_inferred,source_kind\n",f)<0?LAB_ERR_IO:LAB_OK;
}
lab_status li_capture_csv_pose(FILE *f,const li_pose_sample *s){
 if(!f||!s)return LAB_ERR_ARGUMENT;
 int n=fprintf(f,"%" PRIu64 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%u,%u,%u,%u,%.12g,%.12g,%.12g,%u,%u,%" PRIu64 ",host_receipt_proxy,%u,%s\n",
  s->meta.time_us,s->counts[0],s->counts[1],s->counts[2],s->counts[3],s->counts_valid_mask,
  s->counts_per_revolution,s->meta.producer_session,s->meta.context.origin_epoch,
  s->pose.x_m,s->pose.y_m,s->pose.yaw_rad,s->legacy_quality,s->pose.quality,s->meta.received_us,
  (s->meta.flags&LI_COUNTS_MASK_INFERRED)?1u:0u,s->meta.context.run_kind==LI_OFFLINE_TEST?"simulation":"unverified");
 return n<0?LAB_ERR_IO:LAB_OK;
}
