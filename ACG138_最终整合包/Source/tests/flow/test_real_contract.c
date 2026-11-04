#include "lab_real_runtime.h"
#include "lab_codec.h"
#include "lab_real_screen.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned checks;
#define CHECK(x) do{++checks;assert(x);}while(0)
typedef struct fixture {
 lab_real_context context;lab_live_sample *events;unsigned count,at;
 lab_localization_view loc;int8_t cells[64];
 unsigned protect,apply,arm,stop,goals,cancels,nav_ticks;
 int bad_command,bad_output,bad_reset,reached,fail_stop,arm_wait,push_wait,pushes;uint32_t backend,caps;
} fixture;
static lab_status poll(void *u,uint64_t now,lab_live_sample *e){fixture *f=u;if(f->at==f->count||f->events[f->at].meta.received_us>now)return LAB_AGAIN;*e=f->events[f->at++];return LAB_OK;}
static lab_status health(void *u,uint64_t now,lab_live_health *h){fixture *f=u;memset(h,0,sizeof(*h));h->context=f->context;h->time_us=now;return LAB_OK;}
static lab_status reset(void *u,const lab_real_context *c){fixture *f=u;if(f->bad_reset)return LAB_ERR_IO;f->context=*c;f->loc.context=*c;f->loc.map.context=*c;f->at=f->count=0;return LAB_OK;}
static void close_fixture(void *u){(void)u;}
static lab_status push(void *u,const lab_live_sample *e){fixture *f=u;++f->pushes;if(f->push_wait)return LAB_ERR_UNAVAILABLE;
 if(e->type==LAB_EVENT_POSE){f->loc.pose=e->data.odometry;f->loc.quality=f->loc.pose.quality;f->loc.pose_time_us=e->meta.time_us;f->loc.has_pose=1;}
 if(e->type==LAB_EVENT_SCAN)f->loc.last_matched_scan_us=e->meta.time_us;return LAB_OK;}
static lab_status snapshot(void *u,uint64_t now,lab_localization_view *v){(void)now;*v=((fixture*)u)->loc;return LAB_OK;}
static lab_status pose_at(void *u,uint64_t t,lab_real_meta *m,lab_pose *p){fixture *f=u;memset(m,0,sizeof(*m));m->context=f->context;m->time_us=m->received_us=t;m->parameter_version=1;m->flags=3;m->provider=3;m->stream_id=3;m->stream_session=30;*p=f->loc.pose;return LAB_OK;}
static lab_status set_map(void *u,const lab_real_grid_view *m){((fixture*)u)->loc.map=*m;return LAB_OK;}
static lab_status initial(void *u,const lab_pose *p,uint64_t now){fixture *f=u;f->loc.pose=*p;f->loc.pose_time_us=now;return LAB_OK;}
static lab_status goal(void *u,const lab_real_goal *g,uint64_t now){(void)g;(void)now;++((fixture*)u)->goals;return LAB_OK;}
static lab_status cancel(void *u,uint64_t now){(void)now;++((fixture*)u)->cancels;return LAB_OK;}
static lab_status navstep(void *u,uint64_t now,const lab_localization_view *v,const lab_live_sample *s,lab_real_command *out){
 fixture *f=u;(void)v;(void)s;memset(out,0,sizeof(*out));out->context=f->context;out->linear_m_s=f->bad_command?NAN:.08;
 if(f->reached)return LAB_EOF;
 out->generated_us=now;out->expires_us=now+150000;out->sequence=++f->nav_ticks;return LAB_OK;}
static lab_status feedback(void *u,uint64_t now,lab_chassis_feedback *out){fixture *f=u;memset(out,0,sizeof(*out));out->context=f->context;out->time_us=now;out->backend=f->backend;out->capabilities=f->caps;return LAB_OK;}
static lab_status protect(void *u,const lab_chassis_health *h,const lab_real_command *in,lab_chassis_output *out){fixture *f=u;(void)h;++f->protect;memset(out,0,sizeof(*out));out->command=*in;out->command.lease=1;out->backend=f->backend;out->armed=in->enabled;if(f->bad_output)out->wheel_rpm[0]=NAN;return LAB_OK;}
static lab_status enable(void *u,uint32_t yes,uint64_t session,uint64_t now){(void)session;(void)now;fixture *f=u;f->arm+=yes?1:0;if(yes&&f->arm_wait){--f->arm_wait;return LAB_AGAIN;}return LAB_OK;}
static lab_status apply(void *u,const lab_chassis_output *out,uint64_t now){fixture *f=u;CHECK(lab_real_command_valid(&out->command,now));++f->apply;return LAB_OK;}
static lab_status stop(void *u,uint64_t now){(void)now;fixture *f=u;++f->stop;return f->fail_stop?LAB_ERR_IO:LAB_OK;}
static const lab_live_source_ops source_ops={LAB_REAL_API_VERSION,sizeof(lab_live_source_ops),poll,health,reset,close_fixture};
static const lab_localization_ops loc_ops={LAB_REAL_API_VERSION,sizeof(lab_localization_ops),push,snapshot,pose_at,set_map,initial,reset,close_fixture};
static const lab_chassis_ops chassis_ops={LAB_REAL_API_VERSION,sizeof(lab_chassis_ops),feedback,protect,enable,apply,stop,reset,close_fixture};
static void init(fixture *f,int hardware){memset(f,0,sizeof(*f));f->context=(lab_real_context){hardware?LAB_REAL_HARDWARE:LAB_REAL_TEST_FIXTURE,LAB_REAL_MONOTONIC,71,1};
 f->events=calloc(12,sizeof(*f->events));assert(f->events);f->loc.context=f->context;f->loc.state=LAB_REAL_LOC_TRACKING;f->loc.parameter_version=1;
 f->loc.map=(lab_real_grid_view){.context=f->context,.map_id=99,.map_version=1,.width=8,.height=8,.resolution_m=.05,.cells=f->cells};
 f->backend=hardware?LAB_CHASSIS_HARDWARE:LAB_CHASSIS_DRY_RUN;f->caps=7;}
static void events(fixture *f,uint64_t t,uint32_t seq){f->at=f->count=0;
 for(unsigned i=0;i<2;++i){lab_live_sample *e=&f->events[f->count++];memset(e,0,sizeof(*e));e->meta=(lab_real_meta){.context=f->context,.provider=i?LAB_REAL_ODOMETRY:LAB_REAL_USB,.stream_id=i+1,.stream_session=100+i,.sequence=seq,.flags=3,.time_us=t,.received_us=t,.parameter_version=1};
 e->type=i?LAB_EVENT_POSE:LAB_EVENT_SCAN;
 if(i)e->data.odometry=(lab_pose){.quality=LAB_POSE_VALID,.calibrated=1};
 else{e->data.scan=(struct scan_frame){.source=1,.flags=1,.session=100,.sequence=seq,.count=1};e->data.scan.points[0].distance_q2=4000;e->data.scan.points[0].quality=40;}}
}
static lab_real_runtime *open_run(fixture *f,uint32_t mode,int allow){lab_real_runtime_config c;lab_real_runtime_config_default(&c);c.mode=mode;c.allow_hardware_output=(uint32_t)allow;
 lab_real_bindings b={.sensors={f,&source_ops},.localization={f,&loc_ops},.chassis={f,&chassis_ops},.navigation={f,goal,cancel,navstep,reset}};
 lab_real_runtime *r=NULL;CHECK(lab_real_runtime_open(&r,&c,&f->context,&b)==LAB_OK);return r;}
static lab_real_runtime_view view(lab_real_runtime *r){lab_real_runtime_view v;CHECK(lab_real_runtime_view_get(r,&v)==LAB_OK);return v;}
static void set_goal(lab_real_runtime *r,uint64_t now){lab_real_goal g={99,1,0,.2,0,0};CHECK(lab_real_runtime_set_goal(r,&g,now)==LAB_OK);}
static void contracts(void){
 lab_real_context c={LAB_REAL_HARDWARE,LAB_REAL_MONOTONIC,1,1};CHECK(lab_real_context_valid(&c));c.source=LAB_REAL_RECORDING;CHECK(!lab_real_context_valid(&c));c.clock=LAB_REAL_REPLAY_CLOCK;CHECK(lab_real_context_valid(&c));
 fixture f;init(&f,0);events(&f,0,1);CHECK(lab_live_sample_valid(&f.events[0]));f.events[0].type=LAB_EVENT_TRUTH;CHECK(!lab_live_sample_valid(&f.events[0]));
 events(&f,0,1);f.events[0].meta.context.clock=LAB_REAL_UNMAPPED_CLOCK;CHECK(!lab_live_sample_valid(&f.events[0]));
 events(&f,0,1);f.events[0].meta.flags|=LAB_REAL_TIME_ESTIMATED;CHECK(!lab_live_sample_valid(&f.events[0]));f.events[0].meta.time_uncertainty_us=5000;CHECK(lab_live_sample_valid(&f.events[0]));
 CHECK(lab_real_grid_valid(&f.loc.map));f.cells[0]=101;CHECK(!lab_real_grid_valid(&f.loc.map));f.cells[0]=-1;CHECK(lab_real_grid_valid(&f.loc.map));
 lab_chassis_config cc;lab_chassis_config_default(&cc);CHECK(cc.backend==LAB_CHASSIS_DRY_RUN&&cc.max_wheel_rpm==80);
 /* v1编码器应拒绝真实数据元命名空间，不静默保存。 */
 lab_event *old=calloc(1,sizeof(*old));assert(old);old->meta.mode=0x524c;old->type=LAB_EVENT_SCAN;CHECK(!lab_event_valid(old));free(old);free(f.events);
}
static void absent_and_isolation(void){
 lab_real_runtime_config c;lab_real_runtime_config_default(&c);lab_real_context ctx={LAB_REAL_HARDWARE,LAB_REAL_MONOTONIC,1,1};lab_real_bindings b={0};lab_real_runtime *r=NULL;
 CHECK(lab_real_runtime_open(&r,&c,&ctx,&b)==LAB_OK);CHECK(lab_real_runtime_tick(r,0)==LAB_ERR_UNAVAILABLE);CHECK(view(r).missing_modules==1);CHECK(!view(r).hardware_calls);lab_real_runtime_close(r);
 for(uint32_t mode=1;mode<=4;++mode){fixture f;init(&f,0);events(&f,0,1);r=open_run(&f,mode,0);lab_status s=lab_real_runtime_tick(r,0);CHECK(s==LAB_OK||s==LAB_AGAIN);
  if(mode==LAB_REAL_SHADOW){set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_OK);CHECK(view(r).request.linear_m_s>.0);CHECK(f.protect==1);}
  CHECK(lab_real_runtime_enable(r,1,50000)==LAB_ERR_READONLY);lab_real_runtime_close(r);CHECK(!f.apply&&!f.arm&&!f.stop);free(f.events);}
 c.mode=LAB_REAL_NAVIGATION;c.allow_hardware_output=1;ctx.source=LAB_REAL_TEST_FIXTURE;CHECK(lab_real_runtime_open(&r,&c,&ctx,&b)==LAB_ERR_READONLY);
 ctx.source=LAB_REAL_RECORDING;ctx.clock=LAB_REAL_REPLAY_CLOCK;CHECK(lab_real_runtime_open(&r,&c,&ctx,&b)==LAB_ERR_READONLY);
}
static void time_reset_and_sessions(void){
 fixture f;init(&f,0);events(&f,0,1);lab_real_runtime *r=open_run(&f,LAB_REAL_SHADOW,0);CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);
 CHECK(lab_real_runtime_tick(r,50000)==LAB_OK);CHECK(lab_real_runtime_tick(r,50000)==LAB_ERR_TIME);CHECK(!view(r).request.linear_m_s);
 CHECK(lab_real_runtime_tick(r,60000)==LAB_ERR_TIME);lab_real_context c=f.context;CHECK(lab_real_runtime_reset(r,&c)==LAB_ERR_CONTEXT);++c.timeline_id;
 CHECK(lab_real_runtime_reset(r,&c)==LAB_OK);CHECK(!view(r).has_goal);events(&f,0,1);CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);
 events(&f,100000,1);CHECK(lab_real_runtime_tick(r,100000)==LAB_ERR_TIME);CHECK(view(r).rejected==1);
 events(&f,150000,2);f.events[0].meta.stream_session++;f.events[0].data.scan.session++;CHECK(lab_real_runtime_tick(r,150000)==LAB_ERR_CONTEXT);
 events(&f,160000,3);CHECK(lab_real_runtime_tick(r,160000)==LAB_ERR_CONTEXT);CHECK(!strcmp(view(r).reason,"CONTEXT_CHANGE_RESET_REQUIRED"));
 events(&f,200000,3);f.events[0].meta.parameter_version=2;CHECK(lab_real_runtime_tick(r,200000)==LAB_ERR_CONTEXT);
 lab_real_runtime_close(r);CHECK(!f.apply&&!f.arm&&!f.stop);free(f.events);
}
static void stale_map_and_bad_commands(void){fixture f;init(&f,0);events(&f,0,1);lab_real_runtime *r=open_run(&f,LAB_REAL_SHADOW,0);
 CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);f.bad_command=1;CHECK(lab_real_runtime_tick(r,50000)==LAB_ERR_ARGUMENT);CHECK(!view(r).request.linear_m_s);
 f.bad_command=0;CHECK(lab_real_runtime_tick(r,300000)==LAB_AGAIN);CHECK(!view(r).request.linear_m_s);
 events(&f,350000,2);CHECK(lab_real_runtime_tick(r,350000)==LAB_OK);f.loc.map.map_id=100;CHECK(lab_real_runtime_tick(r,400000)==LAB_AGAIN);CHECK(!view(r).has_goal&&f.cancels);
 lab_real_runtime_close(r);CHECK(!f.apply&&!f.arm&&!f.stop);free(f.events);}
static void hardware_stub_gate(void){
 /* 回调仅累计测试计数，不打开套接字、串口或电机。 */
 fixture f;init(&f,1);events(&f,0,1);lab_real_runtime *r=open_run(&f,LAB_REAL_NAVIGATION,1);
 CHECK(lab_real_runtime_enable(r,1,0)==LAB_AGAIN);CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);
 CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);CHECK(!strcmp(view(r).reason,"HARDWARE_DISABLED"));f.caps=1;
 CHECK(lab_real_runtime_enable(r,1,50000)==LAB_ERR_UNAVAILABLE);CHECK(!f.arm);f.caps=7;
 CHECK(lab_real_runtime_enable(r,1,50000)==LAB_OK);CHECK(f.arm==1);CHECK(lab_real_runtime_tick(r,100000)==LAB_OK);CHECK(f.apply==1);
 f.bad_output=1;CHECK(lab_real_runtime_tick(r,150000)==LAB_ERR_ARGUMENT);CHECK(f.apply==1&&f.stop==1&&!view(r).enabled);
 CHECK(lab_real_runtime_enable(r,1,150000)==LAB_AGAIN);lab_real_runtime_close(r);free(f.events);
}
static void uncertainty_version_and_terminal(void){
 fixture f;init(&f,0);events(&f,0,1);f.events[0].meta.time_uncertainty_us=300000;lab_real_runtime *r=open_run(&f,LAB_REAL_SHADOW,0);CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);
 events(&f,100000,2);CHECK(lab_real_runtime_tick(r,100000)==LAB_OK);f.loc.map.map_version=0;CHECK(lab_real_runtime_tick(r,150000)==LAB_ERR_CONTEXT);
 f.loc.map.map_version=2;CHECK(lab_real_runtime_tick(r,200000)==LAB_OK);f.loc.map.map_version=1;CHECK(lab_real_runtime_tick(r,250000)==LAB_ERR_CONTEXT);
 f.loc.map.map_version=3;f.reached=1;CHECK(lab_real_runtime_tick(r,300000)==LAB_OK);CHECK(!view(r).has_goal);CHECK(!strcmp(view(r).reason,"GOAL_REACHED"));CHECK(!f.apply&&!f.arm&&!f.stop);
 lab_real_runtime_close(r);free(f.events);
}
static void render_bounds(void){
 const int sizes[][2]={{640,480},{720,1280},{1280,720},{2560,1440}};lab_real_runtime_view v={0};v.context=(lab_real_context){LAB_REAL_TEST_FIXTURE,LAB_REAL_MONOTONIC,1,1};v.mode=LAB_REAL_SHADOW;strcpy(v.reason,"WAITING_FOR_MODULES");
 for(unsigned i=0;i<4;++i){size_t n=(size_t)sizes[i][0]*sizes[i][1];uint32_t *p=calloc(n+2,sizeof(*p));assert(p);p[0]=p[n+1]=0x12345678;lab_real_screen_draw(p+1,sizes[i][0],sizes[i][1],&v);CHECK(p[0]==0x12345678&&p[n+1]==0x12345678);free(p);}
}
static void stop_failure_is_visible(void){fixture f;init(&f,1);events(&f,0,1);lab_real_runtime *r=open_run(&f,LAB_REAL_NAVIGATION,1);
 CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);CHECK(lab_real_runtime_enable(r,1,50000)==LAB_OK);CHECK(lab_real_runtime_tick(r,100000)==LAB_OK);
 f.fail_stop=1;CHECK(lab_real_runtime_cancel(r,100000)==LAB_ERR_IO);CHECK(!view(r).enabled);CHECK(lab_real_runtime_tick(r,150000)==LAB_ERR_IO);CHECK(lab_real_runtime_enable(r,1,150000)==LAB_ERR_IO);lab_real_context next=f.context;++next.timeline_id;CHECK(lab_real_runtime_reset(r,&next)==LAB_ERR_IO);lab_real_runtime_close(r);free(f.events);
}

static lab_status observed(void *u,const lab_live_sample *s){(void)s;(*(unsigned*)u)++;return LAB_OK;}
static void async_and_alias(void){
 fixture f;init(&f,1);events(&f,0,1);lab_real_runtime *r=open_run(&f,LAB_REAL_NAVIGATION,1);
 CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);
 f.arm_wait=3;CHECK(lab_real_runtime_enable(r,1,50000)==LAB_AGAIN);CHECK(view(r).pending_enable&&!view(r).enabled);
 CHECK(lab_real_runtime_tick(r,100000)==LAB_AGAIN);CHECK(view(r).pending_enable);CHECK(!f.stop&&!f.apply);
 CHECK(lab_real_runtime_cancel(r,100000)==LAB_OK);CHECK(f.stop==1&&!view(r).pending_enable);lab_real_runtime_close(r);free(f.events);
 init(&f,1);events(&f,0,1);r=open_run(&f,LAB_REAL_NAVIGATION,1);
 CHECK(lab_real_runtime_tick(r,0)==LAB_AGAIN);set_goal(r,0);CHECK(lab_real_runtime_tick(r,50000)==LAB_AGAIN);f.arm_wait=2;
 CHECK(lab_real_runtime_enable(r,1,50000)==LAB_AGAIN);CHECK(lab_real_runtime_tick(r,100000)==LAB_AGAIN);CHECK(lab_real_runtime_tick(r,150000)==LAB_OK);CHECK(view(r).enabled&&!view(r).pending_enable&&f.apply==1);
 f.fail_stop=1;lab_real_context next=f.context;++next.timeline_id;CHECK(lab_real_runtime_reset(r,&next)==LAB_ERR_IO);CHECK(lab_real_runtime_tick(r,200000)==LAB_ERR_IO);lab_real_runtime_close(r);free(f.events);
 init(&f,0);events(&f,0,1);r=open_run(&f,LAB_REAL_MAPPING,0);unsigned count=0;CHECK(lab_real_runtime_observe(r,observed,&count)==LAB_OK);
 CHECK(lab_real_runtime_tick(r,0)==LAB_OK);CHECK(f.pushes==2&&count==2);
 f.at=0;f.count=1;f.events[0]=f.events[1];f.events[0].meta.sequence=2;f.events[0].meta.received_us=100;
 CHECK(lab_real_runtime_tick(r,100)==LAB_OK);CHECK(f.pushes==2&&count==3&&view(r).coalesced_pose_aliases==1);
 CHECK(lab_real_runtime_observe(r,observed,&count)==LAB_ERR_READONLY);
 events(&f,100000,3);f.push_wait=1;CHECK(lab_real_runtime_tick(r,100000)==LAB_OK);CHECK(view(r).localizer_waits==2);f.push_wait=0;
 events(&f,200000,4);CHECK(lab_real_runtime_tick(r,200000)==LAB_OK);CHECK(!view(r).hardware_calls);lab_real_runtime_close(r);free(f.events);
}
int main(void){async_and_alias();contracts();absent_and_isolation();time_reset_and_sessions();stale_map_and_bad_commands();hardware_stub_gate();uncertainty_version_and_terminal();render_bounds();stop_failure_is_visible();printf("PASS REAL RK contracts/orchestration checks=%u; all transport is test counters, no hardware IO\n",checks);return 0;}
