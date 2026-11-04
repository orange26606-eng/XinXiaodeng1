/* 普通模式使用R8.1，实物导航以obj101_符号单独编译，具有独立状态和策略。
 */
#define lab_real_navigation_create r81_navigation_create
#define lab_real_navigation_create_profile r81_navigation_create_profile
#include "lab_real_navigation_r81.inc"
#undef lab_real_navigation_create
#undef lab_real_navigation_create_profile

lab_status obj101_navigation_create_profile(lab_real_navigation_ops*,const lab_real_context*,const lab_localization_binding*,const lab_localization_config*,const passage_config*);

typedef struct merge_navigation {
 lab_real_navigation_ops base,object;
 unsigned object_mode;
 lab_status object_available;
} merge_navigation;
static lab_status merge_goal(void*u,const lab_real_goal*g,uint64_t now){merge_navigation*m=u;return m->base.goal(m->base.user,g,now);}
static lab_status merge_cancel(void*u,uint64_t now){merge_navigation*m=u;lab_status s=m->base.cancel(m->base.user,now);if(m->object.user)m->object.cancel(m->object.user,now);return s;}
static lab_status merge_step(void*u,uint64_t now,const lab_localization_view*v,const lab_live_sample*s,lab_real_command*out){merge_navigation*m=u;return m->base.step(m->base.user,now,v,s,out);}
static lab_status merge_reset(void*u,const lab_real_context*c){merge_navigation*m=u;lab_status s=m->base.reset(m->base.user,c);if(s!=LAB_OK)return s;m->object_mode=0;if(m->object.user)m->object_available=m->object.reset(m->object.user,c);return LAB_OK;}
static lab_status merge_view(void*u,lab_real_navigation_view*v){merge_navigation*m=u;lab_real_navigation_ops*b=m->object_mode&&m->object.user?&m->object:&m->base;return b->view(b->user,v);}
static void merge_close(void*u){merge_navigation*m=u;if(!m)return;m->base.close(m->base.user);if(m->object.user)m->object.close(m->object.user);free(m);}
static void merge_hint(void*u,const lab_chassis_feedback*f){merge_navigation*m=u;m->base.chassis_hint(m->base.user,f);if(m->object.user)m->object.chassis_hint(m->object.user,f);}
static lab_status merge_publish(void*u,uint64_t now,const lab_localization_view*v,lab_real_command*out){merge_navigation*m=u;return m->base.publish(m->base.user,now,v,out);}
static void merge_reduce(void*u,uint32_t reduced){merge_navigation*m=u;m->base.reduce_load(m->base.user,reduced);if(m->object.user)m->object.reduce_load(m->object.user,reduced);}
static lab_status merge_cruise(void*u,uint32_t enabled){merge_navigation*m=u;return m->base.set_cruise(m->base.user,enabled);}
static void merge_invalidate(void*u){merge_navigation*m=u;m->base.invalidate_command(m->base.user);if(m->object.user)m->object.invalidate_command(m->object.user);}
static void merge_observe(void*u,const lab_live_sample*s,uint64_t now){merge_navigation*m=u;m->base.observe_sample(m->base.user,s,now);if(m->object.user)m->object.observe_sample(m->object.user,s,now);}
static lab_status merge_local_step(void*u,uint64_t now,const lab_pose*p,uint64_t stamp,const lab_live_sample*s,const lab_real_command*desired,double cap,lab_real_command*out){merge_navigation*m=u;if(!m->object.user)return m->object_available;return m->object.local_step(m->object.user,now,p,stamp,s,desired,cap,out);}
static lab_status merge_local_publish(void*u,uint64_t now,const lab_pose*p,uint64_t stamp,lab_real_command*out){merge_navigation*m=u;if(!m->object.user)return m->object_available;return m->object.local_publish(m->object.user,now,p,stamp,out);}
static lab_status merge_local_mode(void*u,uint32_t mode){merge_navigation*m=u;if(mode>2)return LAB_ERR_ARGUMENT;if(!mode){m->object_mode=0;if(m->object.user)return m->object.local_mode(m->object.user,0);return LAB_OK;}if(!m->object.user)return m->object_available;lab_status s=m->object.local_mode(m->object.user,mode);if(s==LAB_OK)m->object_mode=mode;return s;}
static lab_status merge_local_target(void*u,const object_route_target_v2*t){merge_navigation*m=u;if(!m->object.user)return m->object_available;return m->object.local_target(m->object.user,t);}
lab_status lab_real_navigation_create_profile(lab_real_navigation_ops*out,const lab_real_context*c,const lab_localization_binding*l,const lab_localization_config*g,const passage_config*p){
 if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));merge_navigation*m=calloc(1,sizeof(*m));if(!m)return LAB_ERR_NOMEM;
 lab_status s=r81_navigation_create_profile(&m->base,c,l,g,p);if(s!=LAB_OK){free(m);return s;}
 m->object_available=obj101_navigation_create_profile(&m->object,c,l,g,p);
 if(m->object_available!=LAB_OK)fprintf(stderr,"OBJECT_MODULE_UNAVAILABLE status=%d; R81 original modes remain available\n",m->object_available);
 *out=(lab_real_navigation_ops){m,merge_goal,merge_cancel,merge_step,merge_reset,merge_view,merge_close,merge_hint,merge_publish,merge_reduce,merge_cruise,merge_invalidate,merge_observe,merge_local_step,merge_local_publish,merge_local_mode,merge_local_target};return LAB_OK;
}
lab_status lab_real_navigation_create(lab_real_navigation_ops*out,const lab_real_context*c,const lab_localization_binding*l,const lab_localization_config*g){passage_config p;passage_config_default(&p);return lab_real_navigation_create_profile(out,c,l,g,&p);}
