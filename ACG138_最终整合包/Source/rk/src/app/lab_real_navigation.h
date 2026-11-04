#ifndef W1_REAL_NAVIGATION_H
#define W1_REAL_NAVIGATION_H
#include "lab_real_runtime.h"
#include "passage.h"
/* 只读接入导航数学后端，借用定位对象；需先关闭本绑定，再关闭定位。
 */
lab_status lab_real_navigation_create(lab_real_navigation_ops *out,
 const lab_real_context *context,const lab_localization_binding *localization,
 const lab_localization_config *geometry);
lab_status lab_real_navigation_create_profile(lab_real_navigation_ops*out,const lab_real_context*context,const lab_localization_binding*localization,const lab_localization_config*geometry,const passage_config*passage);
#endif
