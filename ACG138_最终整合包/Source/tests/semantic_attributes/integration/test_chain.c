#define main approach_fixture_main
#include "../approach/test_visual_docking.c"
#undef main
#include "voice_api.h"
#include "phone_goal_validation.h"
#include "object_request_admission.h"
#include <stdlib.h>
int main(int argc,char**argv){CHECK(argc==2);obj_frame *f=malloc(sizeof(*f));CHECK(f&&obj_frame_load(argv[1],f));
 const char*json="{\"seq\":1,\"task_id\":\"10\",\"mode\":\"GO\",\"category\":\"bottle\",\"position\":\"\",\"ordinal\":0,\"reference\":false,\"distance_m\":0.2,\"color\":\"red\",\"size\":\"small\"}";
 phone_goal_message m={0};uint64_t seq=0;CHECK(semantic_payload(json,PHONE_SEMANTIC_TASK,&m,&seq));semantic_spec spec;CHECK(semantic_unpack(&m,&spec)&&!strcmp(spec.color,"red")&&!strcmp(spec.size,"small"));CHECK(semantic_search_pack(&m,&spec)&&m.reserved==6);
 uint64_t n=f->frame_us;m.magic=PHONE_GOAL_MAGIC;m.version=7;m.operation=PHONE_NAV_OBJECT_START;m.sent_us=n;m.sequence=m.request_id=seq;memset(m.session,'1',32);CHECK(phone_goal_message_valid(&m,sizeof m,n));CHECK(!object_request_admission(&m,n,0));
 char category[64],wanted_color[16],size[12];CHECK(object_attribute_search_parse(m.association_id,category,sizeof category,wanted_color,sizeof wanted_color,size,sizeof size));
 object_dual_task t;CHECK(object_dual_task_prepare_search_attributes(&t,category,wanted_color,size,m.x_m,m.request_id));geometry(&t,1);CHECK(object_dual_task_start(&t,OBJECT_MODE_STATIC,0,0,n));lab_real_runtime_view v=view(n,0);lab_real_command c;
 CHECK(object_dual_task_tick(&t,NULL,f,&v,n,&c));CHECK(t.track_id==2&&t.search_bound&&t.near_hold&&c.linear_m_s==0&&c.angular_rad_s==0);CHECK(t.visual_dock.confirmations==1&&!t.arrived);object_route_target_v2 route;CHECK(object_dual_task_route_v2(&t,&v,n,&route)&&route.zero_required);
 CHECK(object_dual_task_tick(&t,NULL,f,&v,n+1000,&c));CHECK(t.visual_dock.confirmations==1&&!t.arrived);
 f->frame_id++;f->frame_us+=50000;f->generated_us=f->frame_us+1;for(int k=0;k<f->count;k++){obj_detection*d=&f->objects[k];d->observation_frame_id=f->frame_id;d->observation_time_us=f->frame_us;d->depth_frame_id=f->frame_id;d->depth_time_us=f->frame_us;}v=view(f->frame_us,0);
 CHECK(object_dual_task_tick(&t,NULL,f,&v,f->frame_us,&c));CHECK(t.arrived&&!t.active&&t.near_hold);object_dual_task_cancel(&t,"USER_CANCEL");CHECK(!object_dual_task_tick(&t,NULL,f,&v,f->frame_us+1000,&c));free(f);
 printf("PASS %u end-to-end C checks: actual producer JSON -> attributes parser -> semantic HTTP/unpack -> reserved6/session validation -> production static task -> correct red small bottle -> no-lidar near zero route -> independent-source completion -> cancel; fixture depth synthetic, no motors\n",checks);return 0;}
