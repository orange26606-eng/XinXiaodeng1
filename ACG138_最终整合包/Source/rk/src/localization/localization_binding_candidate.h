#ifndef ACG_W3_LOCALIZATION_BINDING_CANDIDATE_H
#define ACG_W3_LOCALIZATION_BINDING_CANDIDATE_H
/* 候选构建使用独立工厂符号；正常构建使用公开工厂接口。 */
#include "lab_localization.h"
lab_status w3_localization_candidate_create(lab_localization_binding *out,const lab_localization_config *config,const lab_real_context *context);
size_t w3_localization_allocated_bytes(const void *user);
#endif
