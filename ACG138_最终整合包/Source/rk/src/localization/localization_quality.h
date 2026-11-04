#ifndef LOCALIZATION_QUALITY_H
#define LOCALIZATION_QUALITY_H
#include <math.h>
#include "r8_localization_policy.h"
/* 匹配比例与残差表示置信度，不直接决定地图是否可用；有限几何和足够方程仍必需，弱匹配只施加小校正，里程计保持连续。

 */
static inline double wl_quality_weight(unsigned n,unsigned used,unsigned close,double robust,double raw,double min_ratio,double max_rmse,unsigned min_points){
 if(!n||used<min_points||used>n||close>used||!isfinite(robust)||!isfinite(raw)||robust<0||raw<0||min_ratio<=0||max_rmse<=0)return 0;
 double w=1,parts[5]={(double)used/n/min_ratio,(double)close/used/R8_LOC_MIN_CLOSE_MATCHED_RATIO,(double)close/n/R8_LOC_MIN_CLOSE_ALL_RATIO,robust>0?max_rmse/robust:1,raw>0?R8_LOC_MAX_RAW_RMSE_M/raw:1};
 for(unsigned i=0;i<5;i++)w=fmin(w,parts[i]);
 return fmax(0,fmin(1,w*w));
}
static inline int wl_quality_accept(unsigned n,unsigned used,unsigned close,double robust,double raw,double min_ratio,double max_rmse,unsigned min_points,double correction_m,double correction_yaw,double max_correction_m,double max_correction_yaw){
 (void)max_correction_m;(void)max_correction_yaw;
 return isfinite(correction_m)&&isfinite(correction_yaw)&&wl_quality_weight(n,used,close,robust,raw,min_ratio,max_rmse,min_points)>0;
}
#endif
