#ifndef ACG_V8_VOICE_MISSION_H
#define ACG_V8_VOICE_MISSION_H
#include "voice_api.h"
#include "../rk/src/app/object_readonly.h"
typedef struct voice_receipt {uint64_t published_us,request_id,receipt_id,epoch;int track,prepared,active,drive,enabled,pending;double v,w,pv,pw;char mode[12],state[32],reason[128],message[256],receipt_state[32],receipt_reason[128];} voice_receipt;
typedef struct voice_mission {semantic_spec spec;uint64_t id,floor,revision,last_sequence,epoch,selection_id,prepare_id,start_id,last_action_us,last_log_us;int active,linked,track,phase,prepared,search_requested,direct_follow,direct_static;unsigned attempts,duplicates;uint64_t input_frame_us,input_frame_id;int input_count,input_observed_persons;char input_kind[32],selection_reason[64];unsigned selection_category_count,selection_color_count,selection_metric_count;uint64_t selection_frame_id,selection_source_us;char session[33],stage[40],message[256];} voice_mission;
enum {VOICE_NONE=0,VOICE_SELECT=1,VOICE_PREPARE=2,VOICE_START=3,VOICE_SEARCH_PREPARE=4,VOICE_DIRECT_PREPARE=5,VOICE_DIRECT_STATIC_PREPARE=6};
int voice_receipt_parse(const char*,size_t,voice_receipt*);
int voice_mission_accept(voice_mission*,const phone_goal_message*,const char*,uint64_t,uint64_t);
void voice_mission_cancel(voice_mission*,uint64_t,const char*);
int voice_mission_step(voice_mission*,const obj_frame*,const obj_frame*,const voice_receipt*,uint64_t);
int voice_mission_control_message(const voice_mission*,phone_goal_message*);
void voice_mission_issued(voice_mission*,unsigned,uint64_t,uint64_t);
int voice_mission_json(const voice_mission*,const voice_receipt*,uint64_t,char*,size_t);
#endif
