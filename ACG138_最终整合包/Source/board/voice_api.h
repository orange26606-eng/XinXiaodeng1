#ifndef ACG_SEMANTIC_API_H
#define ACG_SEMANTIC_API_H
#include "../rk/src/app/phone_goal_ipc.h"
enum {PHONE_SEMANTIC_TASK=13,PHONE_SEMANTIC_CANCEL=14,PHONE_SEMANTIC_START=15};
typedef struct semantic_spec {char mode[12],category[32],position[12];unsigned ordinal,reference;double distance;char color[16],size[12];} semantic_spec;
int semantic_payload(const char*,unsigned,phone_goal_message*,uint64_t*);
int semantic_search_pack(phone_goal_message*,const semantic_spec*);
int semantic_unpack(const phone_goal_message*,semantic_spec*);
#endif
