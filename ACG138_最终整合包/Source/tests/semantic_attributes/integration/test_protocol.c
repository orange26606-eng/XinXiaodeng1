#include "object_readonly.h"
#include "semantic_observation.h"
#include "phone_goal_validation.h"
#include "object_request_admission.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(int argc,char**argv){assert(argc==3);char file[1024];obj_frame *f=malloc(sizeof(*f));assert(f);
 const char*names[]={"legacy","red","future_color","wrong_eye","pixel_size","historical_size","malformed_attribute","max65"};
 for(unsigned i=0;i<8;i++){snprintf(file,sizeof(file),"%s/%s.json",argv[1],names[i]);assert(obj_frame_load(file,f));assert(f->count==(i==7?65:1));const obj_attributes*a=&f->objects[0].attributes;
 if(i==0){assert(!a->color_valid&&!a->size_valid);}
 if(i==1||i==5||i==7){assert(a->color_valid&&!strcmp(a->color,"red")&&a->size_valid);assert(obj_attribute_match(a,"red","large"));}
 if(i==2||i==3||i==6){assert(!a->color_valid&&a->size_valid);}
 if(i==4){assert(a->color_valid&&!a->size_valid);}
 if(i==5){assert(a->size_frame_id==8&&a->size_source_us==800);}
 }
 assert(obj_frame_load(argv[2],f));assert(f->count==2&&f->frame_id==2&&f->frame_us==2000000);for(int k=0;k<2;k++){assert(f->objects[k].attributes.color_valid&&f->objects[k].attributes.size_valid);assert(f->objects[k].attributes.color_frame_id==2&&f->objects[k].attributes.color_source_us==2000000);}
 obj_frame*newer=malloc(sizeof(*newer));assert(newer);*newer=*f;newer->count=1;newer->frame_id=3;newer->frame_us=2000500;
 assert(!strcmp(newer->objects[0].attributes.color,"blue"));
 assert(semantic_observation_pick(newer,f,2001000,0,0,"bottle")==newer);
 assert(semantic_observation_pick_attributes(newer,f,2001000,0,0,"bottle","red","small")==f);
 assert(semantic_observation_pick_attributes(newer,f,2001000,0,0,"bottle","blue",NULL)==f);
 newer->objects[0].observation_time_us=2000500;newer->objects[0].attributes.color_source_us=2000500;assert(semantic_observation_pick_attributes(newer,f,2001000,0,0,"bottle","blue",NULL)==newer);
 obj_frame*middle=malloc(sizeof(*middle));assert(middle);*middle=*f;middle->frame_id=3;middle->frame_us=2050000;
 for(int k=0;k<2;k++){middle->objects[k].observation_time_us=2050000;middle->objects[k].attributes.color_source_us=2050000;strcpy(middle->objects[k].attributes.color,k?"blue":"red");}
 *newer=*f;newer->count=1;newer->objects[0]=f->objects[1];newer->frame_id=4;newer->frame_us=2100000;newer->objects[0].observation_time_us=2100000;
 assert(semantic_observation_pick_attributes(newer,middle,2101000,0,0,"bottle","red",NULL)==middle);
 middle->objects[0].attributes.size_valid=0;assert(semantic_observation_matches_attributes(middle,2101000,0,0,"bottle","red","large"));
 free(middle);free(newer);
 phone_goal_message m={0};m.magic=PHONE_GOAL_MAGIC;m.version=7;m.operation=PHONE_NAV_OBJECT_START;m.reserved=6;m.sent_us=1000000;m.sequence=m.request_id=1;m.x_m=.2;memset(m.session,'1',32);strcpy(m.association_id,"box|red|large");assert(phone_goal_message_valid(&m,sizeof m,1000001));assert(!object_request_admission(&m,1000001,0));
 m.session[0]='z';assert(!phone_goal_message_valid(&m,sizeof m,1000001));m.session[0]='1';m.padding[0]=1;assert(!phone_goal_message_valid(&m,sizeof m,1000001));m.padding[0]=0;
 char c[64],p[16],s[12];assert(object_attribute_search_parse(m.association_id,c,sizeof c,p,sizeof p,s,sizeof s));assert(!strcmp(c,"box")&&!strcmp(p,"red")&&!strcmp(s,"large"));
 strcpy(m.association_id,"bottle|blue|_");assert(phone_goal_message_valid(&m,sizeof m,1000001));
 strcpy(m.association_id,"box|red|large|bad");assert(!phone_goal_message_valid(&m,sizeof m,1000001));strcpy(m.association_id,"box|invented|large");assert(!phone_goal_message_valid(&m,sizeof m,1000001));
 strcpy(m.association_id,"box|red|large");m.object_track_id=1;assert(!phone_goal_message_valid(&m,sizeof m,1000001));m.object_track_id=0;
 m.reserved=4;strcpy(m.association_id,"bottle");assert(phone_goal_message_valid(&m,sizeof m,1000001));m.reserved=5;m.object_epoch=2;m.object_track_id=1;m.association_id[0]=0;assert(phone_goal_message_valid(&m,sizeof m,1000001));
 m.reserved=7;assert(phone_goal_message_valid(&m,sizeof m,1000001));m.object_epoch=0;assert(!phone_goal_message_valid(&m,sizeof m,1000001));m.object_epoch=2;m.reserved=8;assert(!phone_goal_message_valid(&m,sizeof m,1000001));m.reserved=7;
 m.sent_us=1;assert(!phone_goal_message_valid(&m,sizeof m,9000000));assert(object_request_admission(&m,9000000,0));
 free(f);puts("PASS actual JSON attrs degradation/65 objects, historical metric timestamps, reserved6+legacy4/5, malformed/revoked request bounds; no device IO");return 0;}
