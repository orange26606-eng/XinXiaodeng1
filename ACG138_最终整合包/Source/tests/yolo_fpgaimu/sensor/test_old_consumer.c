#include "lab_real_mpusvc.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
int main(void){mpusvc_view a,b;assert(mpusvc_read("matched-v1.mpusnap",&a)==LAB_OK);assert(mpusvc_read("matched-v2.mpusnap",&b)==LAB_OK);assert(a.raw_received_us==1000000&&b.raw_received_us==1000000);assert(fabs(a.rpy[2]-.25)<1e-6&&fabs(b.rpy[2]-.25)<1e-6);assert(a.bias_ready&&a.counts_mask==15&&a.cal_samples==400);struct odom_shared m;assert(mpusvc_legacy_read("matched-legacy.shm",&m)==LAB_OK);assert(m.magic==ODOM_MAGIC&&m.bytes==4224&&m.state.session==a.session);puts("PASS exact old R81 consumer reads new matched FPGA v1/v2/legacy without ABI change");return 0;}
