#include "s2_navigation_limits.h"
#include "continuous_policy.h"
#include "chassis_private.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* 真实候选沿用control_guard策略，不改仅仿真ABI，也不让真实输入经过模拟SIM上下文。
 */
struct ch_guard {
 ch_config c;ch_context context;ch_health health;ch_request request;ch_output output;
 uint64_t now,tick,arm_time,generation;uint32_t mode,session,armed;
 uint32_t have_clock,have_tick,have_health,have_request,have_sequence,last_sequence;
 uint32_t output_sequence,estop_input,estop_latched,clock_fault,health_blocked,expired;
 uint64_t reversal_until;uint32_t reversing;
};
static int same(ch_context a,ch_context b){return a.run_id==b.run_id&&a.origin_epoch==b.origin_epoch&&a.source==b.source;}
static int valid_context(const ch_context *c){return c&&c->run_id&&c->origin_epoch&&(c->source==CH_LIVE||c->source==CH_SIMULATION);}
static int positive(double x){return isfinite(x)&&x>0;}
static int newer(uint32_t next,uint32_t last){uint32_t d=next-last;return d&&d<UINT32_C(0x80000000);}
void ch_config_default(ch_config *c){
 if(!c)return;memset(c,0,sizeof(*c));for(int i=0;i<4;++i){c->diameter_m[i]=.0675;c->motor_polarity[i]=(i&1)?-1:1;}
 c->track_m=.29;c->max_linear_m_s=.10;c->max_angular_rad_s=S2_MAX_YAW_RAD_S;c->max_rpm=40;
 c->acceleration_m_s2=.2;c->angular_acceleration_rad_s2=.8;c->left_mask=5;
 c->command_age_us=150000;c->command_ttl_us=150000;c->health_age_us=150000;
 c->link_age_us=300000;c->pose_age_us=c->scan_age_us=250000;c->max_ramp_us=100000;c->config_id=1;
}
int ch_config_valid(const ch_config *c){
 if(!c||!positive(c->track_m)||c->track_m<.05||c->track_m>1||!positive(c->max_rpm)||c->max_rpm>327.67||
 !positive(c->max_linear_m_s)||c->max_linear_m_s>2||!positive(c->max_angular_rad_s)||c->max_angular_rad_s>10||
 !positive(c->acceleration_m_s2)||c->acceleration_m_s2>10||!positive(c->angular_acceleration_rad_s2)||c->angular_acceleration_rad_s2>50||
 !c->config_id||!c->command_age_us||c->command_age_us>CONT_COMMAND_US||!c->command_ttl_us||c->command_ttl_us>CONT_COMMAND_US||
 !c->health_age_us||c->health_age_us>1000000||!c->link_age_us||c->link_age_us>1000000||
 !c->pose_age_us||c->pose_age_us>1000000||!c->scan_age_us||c->scan_age_us>CONT_SCAN_US||
 !c->max_ramp_us||c->max_ramp_us>200000||c->left_mask>15||c->direct_targets>1)return 0;
 unsigned left=0;
 for(int i=0;i<4;++i){left+=(c->left_mask>>i)&1;
  if(!isfinite(c->diameter_m[i])||c->diameter_m[i]<.02||c->diameter_m[i]>.5||
     (c->motor_polarity[i]!=1&&c->motor_polarity[i]!=-1))return 0;}
 return left==2;
}
int ch_kinematics(const ch_config *c,double v,double w,ch_output *out){
 if(!out)return CH_ARGUMENT;memset(out,0,sizeof(*out));
 if(!ch_config_valid(c)||!isfinite(v)||!isfinite(w))return CH_ARGUMENT;
 double scale=1;
 if(fabs(v)>c->max_linear_m_s)scale=fmin(scale,c->max_linear_m_s/fabs(v));
 if(fabs(w)>c->max_angular_rad_s)scale=fmin(scale,c->max_angular_rad_s/fabs(w));
 for(int i=0;i<4;++i){double speed=v+(((c->left_mask>>i)&1)?-1:1)*w*c->track_m*.5;
  double rpm=speed*60/(3.14159265358979323846*c->diameter_m[i]);
  if(!isfinite(rpm))return CH_ARGUMENT;
  if(fabs(rpm)>c->max_rpm)scale=fmin(scale,c->max_rpm/fabs(rpm));}
 out->linear_m_s=v*scale;out->angular_rad_s=w*scale;out->limit_scale=scale;
 for(int i=0;i<4;++i){double speed=out->linear_m_s+(((c->left_mask>>i)&1)?-1:1)*out->angular_rad_s*c->track_m*.5;
  out->logical_rpm[i]=speed*60/(3.14159265358979323846*c->diameter_m[i]);
  out->electrical_rpm[i]=out->logical_rpm[i]*c->motor_polarity[i];}
 return CH_OK;
}
static void zero(ch_guard *g,uint64_t now){
 memset(&g->output,0,sizeof(g->output));g->output.context=g->context;g->output.time_us=g->output.expires_us=now;
 g->output.sequence=++g->output_sequence;g->output.mode=g->mode;g->output.session=g->session;
 g->output.generation=g->generation;g->output.armed=g->armed;
}
static void fence(ch_guard *g,uint64_t now){
 g->have_request=0;if(g->generation==UINT64_MAX){g->clock_fault=1;g->armed=0;}else ++g->generation;
 zero(g,now);
}
static int clock_at(ch_guard *g,uint64_t now,int tick){
 if((g->have_clock&&now<g->now)||(tick&&g->have_tick&&now<=g->tick)){
  g->clock_fault=1;g->armed=0;fence(g,g->now);g->output.reasons=CH_R_CLOCK;return CH_TIME;}
 g->now=now;g->have_clock=1;return g->clock_fault?CH_TIME:CH_OK;
}
static uint32_t health_reason(const ch_guard *g,uint64_t now){
 if(!g->have_health)return CH_R_HEALTH;
 const ch_health *h=&g->health;uint32_t r=0;
 if(now-h->time_us>=g->c.health_age_us)r|=CH_R_HEALTH;
 if(!h->link_ok||now-h->link_time_us>=g->c.link_age_us)r|=CH_R_LINK;
 if(g->mode==CH_NAVIGATION&&(!h->pose_valid||now-h->pose_time_us>=g->c.pose_age_us))r|=CH_R_POSE;
 if(!h->scan_valid||now-h->scan_time_us>=g->c.scan_age_us)r|=CH_R_SCAN;
 if(h->obstacle_stop)r|=CH_R_OBSTACLE;if(h->motor_fault)r|=CH_R_MOTOR;return r;
}
static void gate(ch_guard *g,uint64_t now,uint32_t reason){
 if(reason){if(!g->health_blocked)fence(g,now);else{g->have_request=0;zero(g,now);}g->health_blocked=1;}
 else g->health_blocked=0;
}
static ch_lease lease_of(const ch_guard *g){return (ch_lease){g->context,g->generation,g->mode,g->session};}
int ch_guard_open(ch_guard **out,const ch_config *c,const ch_context *ctx){
 if(!out)return CH_ARGUMENT;*out=NULL;if(!ch_config_valid(c)||!valid_context(ctx))return CH_ARGUMENT;
 ch_guard *g=calloc(1,sizeof(*g));if(!g)return CH_LIMIT;g->c=*c;g->context=*ctx;g->generation=1;
 zero(g,0);g->output.reasons=CH_R_OFF|CH_R_DISARMED;*out=g;return CH_OK;
}
int ch_guard_select(ch_guard *g,uint32_t mode,uint32_t session,uint64_t now){
 if(!g||mode>CH_NAVIGATION||(mode!=CH_OFF&&!session))return CH_ARGUMENT;
 if(clock_at(g,now,0)!=CH_OK)return CH_TIME;
 g->mode=mode;g->session=session;g->armed=0;g->have_sequence=0;g->expired=0;
 fence(g,now);g->output.reasons=CH_R_DISARMED|(mode==CH_OFF?CH_R_OFF:0);return CH_OK;
}
int ch_guard_health(ch_guard *g,const ch_health *h,uint64_t now){
 if(!g||!h)return CH_ARGUMENT;if(clock_at(g,now,0)!=CH_OK)return CH_TIME;
 if(!same(g->context,h->context))return CH_CONTEXT;
 if(h->link_ok>1||h->pose_valid>1||h->scan_valid>1||h->obstacle_stop>1||h->motor_fault>1)return CH_ARGUMENT;
 if(h->time_us>now||h->link_time_us>h->time_us||h->pose_time_us>h->time_us||h->scan_time_us>h->time_us||
    (g->have_health&&h->time_us<g->health.time_us))return CH_TIME;
 g->health=*h;g->have_health=1;uint32_t r=health_reason(g,now);gate(g,now,r);
 if(r)g->output.reasons=r|(!g->armed?CH_R_DISARMED:0);return CH_OK;
}
int ch_guard_arm(ch_guard *g,uint64_t now,ch_lease *out){
 if(out)memset(out,0,sizeof(*out));if(!g||!out)return CH_ARGUMENT;if(clock_at(g,now,0)!=CH_OK)return CH_TIME;
 uint32_t r=health_reason(g,now);if(g->mode==CH_OFF)r|=CH_R_OFF;if(g->estop_latched)r|=CH_R_ESTOP;
 if(r){g->output.reasons=r|CH_R_DISARMED;return CH_WAIT;}
 fence(g,now);if(g->clock_fault)return CH_LIMIT;g->armed=1;g->arm_time=now;g->have_sequence=0;g->health_blocked=0;g->expired=0;
 zero(g,now);g->output.reasons=CH_R_NO_COMMAND;*out=lease_of(g);return CH_OK;
}
int ch_guard_get_lease(const ch_guard *g,ch_lease *out){if(!g||!out)return CH_ARGUMENT;*out=lease_of(g);return CH_OK;}
int ch_guard_submit(ch_guard *g,const ch_request *r,uint64_t now){
 if(!g||!r)return CH_ARGUMENT;if(clock_at(g,now,0)!=CH_OK)return CH_TIME;
 if(!same(g->context,r->lease.context))return CH_CONTEXT;
 if(r->lease.generation!=g->generation||r->lease.mode!=g->mode||r->lease.session!=g->session)return CH_READONLY;
 uint32_t health=health_reason(g,now);gate(g,now,health);
 if(health||!g->armed||g->estop_latched||g->mode==CH_OFF)return CH_READONLY;
 if(!isfinite(r->linear_m_s)||!isfinite(r->angular_rad_s))return CH_ARGUMENT;
 if(r->time_us>now||r->time_us<g->arm_time||r->expires_us<=r->time_us||r->expires_us<=now||now-r->time_us>=g->c.command_age_us||
   (g->have_sequence&&(!newer(r->sequence,g->last_sequence)||r->time_us<g->request.time_us)))return CH_TIME;
 ch_request accepted=*r;if(accepted.expires_us-accepted.time_us>g->c.command_ttl_us)accepted.expires_us=accepted.time_us+g->c.command_ttl_us;
 if(accepted.expires_us<=now)return CH_TIME;g->request=accepted;g->have_request=g->have_sequence=1;g->expired=0;g->last_sequence=r->sequence;return CH_OK;
}
static double slew(double x,double target,double amount){if(x*target<0)target=0;return x+fmax(-amount,fmin(amount,target-x));}
int ch_guard_tick(ch_guard *g,uint64_t now,ch_output *out){
 if(out){memset(out,0,sizeof(*out));out->time_us=out->expires_us=now;if(g)out->context=g->context;}
 if(!g||!out)return CH_ARGUMENT;if(clock_at(g,now,1)!=CH_OK){*out=g->output;return CH_TIME;}
 uint64_t dt=g->have_tick?now-g->tick:0;g->tick=now;g->have_tick=1;
 uint32_t r=health_reason(g,now);gate(g,now,r);
 if(g->have_request&&now>=g->request.expires_us){fence(g,now);g->expired=1;}
 if(g->mode==CH_OFF)r|=CH_R_OFF;if(!g->armed)r|=CH_R_DISARMED;if(g->estop_latched)r|=CH_R_ESTOP;
 if(!g->have_request)r|=g->expired?CH_R_EXPIRED:CH_R_NO_COMMAND;
 if(r){zero(g,now);g->output.reasons=r;*out=g->output;return CH_WAIT;}
 ch_output next;int status=ch_kinematics(&g->c,g->request.linear_m_s,g->request.angular_rad_s,&next);
 if(status!=CH_OK){fence(g,now);*out=g->output;return status;}
 if(next.linear_m_s==0&&next.angular_rad_s==0){
  /* 有效停车请求仍保留受保护的原期限。zero()也用于故障及屏障，因此只有本接受请求路径继承请求有效期。

 */
  zero(g,now);g->output.expires_us=g->request.expires_us;
  *out=g->output;return CH_OK;
 }
 if(dt>g->c.max_ramp_us)dt=g->c.max_ramp_us;double seconds=(double)dt/1000000;
 double v=slew(g->output.linear_m_s,next.linear_m_s,g->c.acceleration_m_s2*seconds);
 double w=slew(g->output.angular_rad_s,next.angular_rad_s,g->c.angular_acceleration_rad_s2*seconds);
 if(g->c.direct_targets){
  int reverse=0;for(int i=0;i<4;i++)if(g->output.logical_rpm[i]*next.logical_rpm[i]<0)reverse=1;
  if(reverse&&!g->reversing){g->reversal_until=now+50000;g->reversing=1;}
  if(g->reversing&&now<g->reversal_until){zero(g,now);g->output.expires_us=g->request.expires_us;*out=g->output;return CH_OK;}
  g->reversing=0;v=next.linear_m_s;w=next.angular_rad_s;
 }
 status=ch_kinematics(&g->c,v,w,&next);if(status!=CH_OK){fence(g,now);*out=g->output;return status;}
 next.context=g->context;next.time_us=now;next.expires_us=g->request.expires_us;next.sequence=++g->output_sequence;
 next.mode=g->mode;next.session=g->session;next.generation=g->generation;next.armed=g->armed;
 g->output=next;*out=next;return CH_OK;
}
int ch_guard_estop(ch_guard *g,int active,uint64_t now){
 if(!g||(active!=0&&active!=1))return CH_ARGUMENT;
 if(active){g->estop_input=g->estop_latched=1;g->armed=0;fence(g,g->have_clock?g->now:now);g->output.reasons=CH_R_ESTOP|CH_R_DISARMED;}
 if(clock_at(g,now,0)!=CH_OK)return CH_TIME;g->estop_input=(uint32_t)active;return CH_OK;
}
int ch_guard_clear_estop(ch_guard *g,uint64_t now){
 if(!g)return CH_ARGUMENT;if(clock_at(g,now,0)!=CH_OK)return CH_TIME;if(g->estop_input)return CH_READONLY;
 g->estop_latched=0;g->armed=0;fence(g,now);g->output.reasons=CH_R_DISARMED;return CH_OK;
}
int ch_guard_reset(ch_guard *g,const ch_context *ctx){
 if(!g||!valid_context(ctx))return CH_ARGUMENT;if(same(g->context,*ctx))return CH_CONTEXT;
 ch_config c=g->c;uint32_t latch=g->estop_latched,input=g->estop_input;uint64_t generation=g->generation;
 memset(g,0,sizeof(*g));g->c=c;g->context=*ctx;g->estop_latched=latch;g->estop_input=input;g->generation=generation;
 fence(g,0);g->output.reasons=CH_R_OFF|CH_R_DISARMED|(latch?CH_R_ESTOP:0);return CH_OK;
}
int ch_guard_view(const ch_guard *g,ch_output *out){if(!g||!out)return CH_ARGUMENT;*out=g->output;return CH_OK;}
size_t ch_guard_allocated_bytes(const ch_guard *g){return g?sizeof(*g):0;}
void ch_guard_close(ch_guard *g){free(g);}
