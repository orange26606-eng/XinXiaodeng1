#include "chassis_binding.h"
#include "chassis_linux.h"
#include <math.h>
#include "s1_motion_policy.h"
#include "continuous_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct chassis_adapter {
 lab_chassis_config config;lab_real_context context;ch_config policy;ch_guard *guard;
 lab_chassis_output cached;ch_health health;ch_receiver model;ch_linux *transport;
 chassis_hardware_options hardware;ch_wire_status peer;
 uint64_t now,public_session,guard_public_session,peer_time_us,handshake_us,guard_armed_us;
 uint32_t prepared,enabled,phase,wire_session,wire_sequence,next_token,have_health,have_cached,have_peer;
 uint32_t selected,have_applied,applied_sequence;
 uint32_t clock_fault;
 uint32_t handshake_resyncs;uint64_t resync_peer_us,last_stale_diag_us;
} chassis_adapter;
static const lab_chassis_ops ops;
/* 使用显式有界配置语法，解析时不打开套接字。 */
static int endpoint_options(const lab_chassis_config *c,chassis_hardware_options *out){
 memset(out,0,sizeof(*out));
 if(c->reserved_transport||!memchr(c->endpoint,0,sizeof(c->endpoint)))return 0;
 if(c->backend==LAB_CHASSIS_DRY_RUN)return c->transport==LAB_CHASSIS_TRANSPORT_NONE&&!c->endpoint[0];
 if(c->transport!=LAB_CHASSIS_TRANSPORT_UDP||strncmp(c->endpoint,"ncv1://",7))return 0;
 const char *p=c->endpoint+7,*slash=strchr(p,'/');
 if(!slash||slash==p||slash-p>15)return 0;
 for(const char *s=p;s<slash;++s)if(!((*s>='a'&&*s<='z')||(*s>='A'&&*s<='Z')||(*s>='0'&&*s<='9')||*s=='_'||*s=='-'||*s=='.'))return 0;
 memcpy(out->interface_name,p,(size_t)(slash-p));
 const char suffix[]="/192.168.0.2:5012?config_id=";
 if(strncmp(slash,suffix,sizeof(suffix)-1))return 0;p=slash+sizeof(suffix)-1;
 if(!*p)return 0;uint32_t id=0;
 for(;*p;++p){if(*p<'0'||*p>'9')return 0;uint32_t d=(uint32_t)(*p-'0');if(id>(UINT32_MAX-d)/10)return 0;id=id*10+d;}
 out->expected_config_id=id;return id!=0;
}
static lab_status status(int r){switch(r){case CH_OK:return LAB_OK;case CH_WAIT:return LAB_AGAIN;case CH_ARGUMENT:return LAB_ERR_ARGUMENT;
 case CH_CONTEXT:return LAB_ERR_CONTEXT;case CH_TIME:return LAB_ERR_TIME;case CH_CRC:return LAB_ERR_CRC;case CH_FORMAT:return LAB_ERR_FORMAT;
 case CH_READONLY:return LAB_ERR_READONLY;case CH_LIMIT:return LAB_ERR_LIMIT;default:return LAB_ERR_IO;}}
static ch_context core_context(const lab_real_context *c){return (ch_context){c->timeline_id,c->origin_epoch,c->source==LAB_REAL_HARDWARE?CH_LIVE:CH_SIMULATION};}
static void reason(lab_chassis_output *out,const char *text){snprintf(out->reason,sizeof(out->reason),"%s",text);}
static int valid_clock(chassis_adapter *a,uint64_t now){if(now<a->now)a->clock_fault=1;if(a->clock_fault)return 0;a->now=now;return 1;}
static int next_wire(chassis_adapter *a,uint32_t *sequence){
 /* 接收端支持线上序号回绕，但不支持令牌分配耗尽。 */
 *sequence=++a->wire_sequence;return 1;
}
static ch_wire_command wire(chassis_adapter *a,uint8_t op){ch_wire_command c={0};c.op=op;c.mode=CH_NAVIGATION;c.session=a->wire_session;
 next_wire(a,&c.sequence);c.lease=a->peer.lease;c.config_id=a->hardware.expected_config_id?a->hardware.expected_config_id:1;return c;}
static lab_status ensure_transport(chassis_adapter *a){
 if(!a->prepared)return LAB_ERR_UNAVAILABLE;
 if(a->transport)return LAB_OK;
 ch_context ctx=core_context(&a->context);return status(ch_linux_open(&a->transport,a->hardware.interface_name,&ctx,a->hardware.expected_config_id));
}
static lab_status adapter_feedback(void *user,uint64_t now,lab_chassis_feedback *out){
 chassis_adapter *a=user;if(!a||!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));out->context=a->context;out->backend=a->config.backend;
 if(!valid_clock(a,now))return LAB_ERR_TIME;
 if(a->config.backend==LAB_CHASSIS_DRY_RUN){
  ch_receiver_sample(&a->model,now/1000,&a->peer);out->time_us=now;out->control_session=a->public_session;
  out->armed=a->model.armed;out->capabilities=LAB_CHASSIS_TARGET_RPM|LAB_CHASSIS_LOCAL_WATCHDOG;
  out->accepted_sequence=a->have_applied?a->applied_sequence:0;out->applied_sequence=0;
  snprintf(out->reason,sizeof(out->reason),"DRY_RUN_NO_PHYSICAL_FEEDBACK");return LAB_OK;
 }
 lab_status ready=ensure_transport(a);if(ready!=LAB_OK){snprintf(out->reason,sizeof(out->reason),"HARDWARE_NOT_PREPARED");return ready;}
 ch_wire_status s;int rc=ch_linux_poll(a->transport,now,&s);
 if(rc==CH_OK||rc==CH_READONLY){out->fpga_config_id=s.config_id;out->fpga_status_revision=s.revision;out->fpga_command_ttl_ms=s.max_ttl_ms;}
 if(rc==CH_OK){
  if(a->have_peer&&(s.state!=a->peer.state||s.reason!=a->peer.reason||s.fault_mask!=a->peer.fault_mask))
   fprintf(stderr,"FPGA_STATE_EVENT old_state=%u state=%u old_reason=%u reason=%u old_faults=%u faults=%u session=%u sequence=%u accepted=%u rejected=%u watchdogs=%u valid_mask=%u now_us=%llu\n",a->peer.state,s.state,a->peer.reason,s.reason,a->peer.fault_mask,s.fault_mask,s.session,s.sequence,s.accepted,s.rejected,s.watchdogs,s.valid_mask,(unsigned long long)now);
  a->peer=s;a->peer_time_us=now;a->have_peer=1;
 }
 else if(rc!=CH_WAIT){fprintf(stderr,"FPGA_FEEDBACK_FAILURE stage=POLL raw=%d now_us=%llu\n",rc,(unsigned long long)now);snprintf(out->reason,sizeof(out->reason),rc==CH_READONLY?"FPGA_POLICY_MISMATCH":"CAPABILITY_OR_STATUS_REJECTED");return status(rc);}
 if(!a->have_peer||now<a->peer_time_us||now-a->peer_time_us>=CONT_FEEDBACK_US){if(!a->last_stale_diag_us||now-a->last_stale_diag_us>=1000000){a->last_stale_diag_us=now;fprintf(stderr,"FPGA_FEEDBACK_FAILURE stage=STALE have_peer=%u age_us=%llu now_us=%llu\n",a->have_peer,(unsigned long long)(a->have_peer&&now>=a->peer_time_us?now-a->peer_time_us:UINT64_MAX),(unsigned long long)now);}snprintf(out->reason,sizeof(out->reason),"FPGA_STATUS_STALE");return LAB_AGAIN;}
 out->time_us=a->peer_time_us;out->control_session=a->public_session;out->capabilities=a->peer.capabilities&7;
 out->armed=a->peer.state==CH_RX_ARMED||a->peer.state==CH_RX_DRIVING;out->faults=a->peer.fault_mask|(a->peer.valid_mask==15?0u:256u);
 /* 线上ACK序号与公开请求序号不可互换。 */
 if(a->have_applied&&a->peer.sequence==a->wire_sequence)out->accepted_sequence=a->applied_sequence;
 /* NCV1状态只确认目标已接受，不确认下游PWM已施加，不能据ACK构造applied_sequence。
 */
 out->applied_sequence=0;
 memcpy(out->sonar_mm,a->peer.sonar_mm,sizeof(out->sonar_mm));out->sonar_valid=a->peer.sonar_valid;out->sonar_fresh=a->peer.sonar_fresh;out->sonar_near=a->peer.sonar_near;out->sonar_degraded=a->peer.sonar_degraded;out->sonar_front_stable=a->peer.sonar_front_stable;out->local_stop_sources=a->peer.local_stop_sources;out->sonar_pending=a->peer.sonar_pending;memcpy(out->sonar_age_bucket,a->peer.sonar_age_bucket,3);out->sonar_raw_zero_mask=a->peer.sonar_raw_zero_mask;
 out->sonar_front_sequence=a->peer.sonar_front_sequence;out->sonar_front_sequence_valid=a->peer.revision==S1_STATUS_REVISION;
 ch_wire_diagnostic diagnostic;uint64_t received=0;
 if(ch_linux_diagnostics(a->transport,now,&diagnostic,&received)==CH_OK){
  memcpy(out->sonar_raw_mm,diagnostic.raw_mm,sizeof(out->sonar_raw_mm));memcpy(out->effective_pwm,diagnostic.effective_pwm,sizeof(out->effective_pwm));
  memcpy(out->sonar_nonzero_sequence,diagnostic.nonzero_sequence,sizeof(out->sonar_nonzero_sequence));memcpy(out->sonar_raw_age_us,diagnostic.raw_age_us,sizeof(out->sonar_raw_age_us));
  out->sonar_source_uptime_ms=diagnostic.uptime_ms;out->sonar_diagnostic_time_us=received;out->sonar_raw_available=diagnostic.available_mask;out->sonar_diagnostic_flags=diagnostic.flags;out->sonar_observation_block=diagnostic.observation_block;out->sonar_diagnostic_zero_mask=diagnostic.zero_mask;out->sonar_diagnostic_fresh=diagnostic.fresh;out->sonar_diagnostic_near=diagnostic.near;out->sonar_diagnostic_pending=diagnostic.pending;
 }
 for(int i=0;i<4;++i)out->actual_rpm[i]=(double)a->peer.measured_centi[i]/100*a->config.drive_sign[i];
 if(a->peer.reason==CH_RX_ULTRASONIC_BLOCKED)snprintf(out->reason,sizeof(out->reason),"ULTRASONIC_PROTECTION_BLOCKED");
 else snprintf(out->reason,sizeof(out->reason),"NCV1_ACK_ONLY_REASON_%u",a->peer.reason);return LAB_OK;
}
static lab_status adapter_protect(void *user,const lab_chassis_health *h,const lab_real_command *request,lab_chassis_output *out){
 chassis_adapter *a=user;if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));
 if(a)a->have_cached=0;
 if(!a||!h||!request)return LAB_ERR_ARGUMENT;out->backend=a->config.backend;out->command=*request;
 out->command.linear_m_s=out->command.angular_rad_s=0;out->command.enabled=0;
 if(!lab_real_context_equal(&a->context,&h->context)||!lab_real_context_equal(&a->context,&request->context))return LAB_ERR_CONTEXT;
 if(a->context.source==LAB_REAL_RECORDING||a->context.clock!=LAB_REAL_MONOTONIC){reason(out,"REPLAY_OR_UNMAPPED_NO_EXECUTION");return LAB_ERR_READONLY;}
 uint64_t now=h->now_us;if(!valid_clock(a,now)||!lab_real_command_valid(request,now))return LAB_ERR_TIME;
 if(h->scan_valid>1||h->pose_valid>1||h->link_valid>1||h->obstacle_stop>1||h->motor_fault>1||h->estop>1)return LAB_ERR_ARGUMENT;
 ch_context ctx=core_context(&a->context);
 a->health=(ch_health){ctx,now,h->link_time_us,h->pose_time_us,h->scan_time_us,h->link_valid,h->pose_valid,h->scan_valid,h->obstacle_stop,h->motor_fault};
 a->have_health=1;
 uint64_t public_session=request->control_session;
 if(!a->selected||a->guard_public_session!=public_session){
  if(a->next_token==UINT32_MAX)return LAB_ERR_LIMIT;
  uint32_t token=++a->next_token;
  int r=ch_guard_select(a->guard,CH_NAVIGATION,token,now);if(r!=CH_OK)return status(r);
  a->guard_public_session=public_session;a->selected=1;
 }
 int r=ch_guard_health(a->guard,&a->health,now);if(r!=CH_OK)return status(r);
 r=ch_guard_estop(a->guard,(int)h->estop,now);if(r!=CH_OK)return status(r);
 ch_output view;ch_guard_view(a->guard,&view);ch_lease lease;
 if(!view.armed){r=ch_guard_arm(a->guard,now,&lease);if(r!=CH_OK){out->reasons=view.reasons;reason(out,"WAITING_FOR_SOFTWARE_ARM");return status(r);}a->guard_armed_us=now;}
 else ch_guard_get_lease(a->guard,&lease);
 if(request->generated_us<a->guard_armed_us){out->armed=1;out->fresh_command_required=1;reason(out,"POST_ARM_FRESH_COMMAND_REQUIRED");return LAB_AGAIN;}
 ch_request input={lease,request->generated_us,request->expires_us,request->sequence,request->linear_m_s,request->angular_rad_s};
 r=ch_guard_submit(a->guard,&input,now);if(r!=CH_OK){reason(out,"REQUEST_REJECTED");return status(r);}
 r=ch_guard_tick(a->guard,now,&view);out->reasons=view.reasons;out->armed=view.armed;
 if(r!=CH_OK){reason(out,"SOFTWARE_PROTECTION_STOP");a->have_cached=0;return status(r);}
 out->command=*request;out->command.linear_m_s=view.linear_m_s;out->command.angular_rad_s=view.angular_rad_s;
 out->command.expires_us=view.expires_us;out->command.lease=view.generation;
 for(int i=0;i<4;++i)out->wheel_rpm[i]=view.logical_rpm[i];
 reason(out,a->config.backend==LAB_CHASSIS_DRY_RUN?"DRY_RUN_PROTECTED":"SOFTWARE_PROTECTED_NOT_YET_APPLIED");
 a->cached=*out;a->have_cached=1;return LAB_OK;
}
static void adapter_diag(const chassis_adapter*a,const char*stage,int rc,uint64_t now){
 if(!a||a->config.backend!=LAB_CHASSIS_HARDWARE)return;
 fprintf(stderr,"CHASSIS_ADAPTER_DIAG stage=%s result=%d now_us=%llu phase=%u enabled=%u have_transport=%u have_peer=%u peer_age_us=%llu session_match=%u peer_session=%u wire_session=%u peer_seq=%u wire_seq=%u peer_state=%u peer_reason=%u peer_faults=%u peer_valid=%u\n",stage,rc,(unsigned long long)now,a->phase,a->enabled,(unsigned)(a->transport!=NULL),a->have_peer,(unsigned long long)(a->have_peer&&now>=a->peer_time_us?now-a->peer_time_us:UINT64_MAX),(unsigned)(a->have_peer&&a->peer.session==a->wire_session),a->peer.session,a->wire_session,a->peer.sequence,a->wire_sequence,a->peer.state,a->peer.reason,a->peer.fault_mask,a->peer.valid_mask);
}
static lab_status adapter_stop(void *user,uint64_t now){
 chassis_adapter *a=user;if(!a)return LAB_ERR_ARGUMENT;
 adapter_diag(a,"STOP_BEGIN",LAB_OK,now);
 a->enabled=0;a->phase=0;a->selected=0;a->have_cached=a->have_applied=0;
 if(a->guard)ch_guard_select(a->guard,CH_OFF,0,now);
 if(a->config.backend==LAB_CHASSIS_DRY_RUN){
  ch_receiver_health(&a->model,1,0,0,now/1000);
  if(a->model.claimed){a->peer=a->model.status;ch_wire_command c=wire(a,CH_STOP);return status(ch_receiver_accept(&a->model,&c,now/1000));}return LAB_OK;
 }
 if(!a->transport||!a->have_peer||!a->wire_session||a->peer.session!=a->wire_session){adapter_diag(a,"STOP_SKIPPED_NO_MATCHING_PEER_NOT_ACK",LAB_OK,now);return LAB_OK;}
 ch_wire_command c=wire(a,CH_STOP);int raw=ch_linux_send(a->transport,&c,now,now);lab_status mapped=status(raw);adapter_diag(a,"STOP_SEND_RESULT",mapped,now);return mapped;
}
static lab_status adapter_enable(void *user,uint32_t enabled,uint64_t session,uint64_t now){
 chassis_adapter *a=user;if(!a||enabled>1)return LAB_ERR_ARGUMENT;if(!enabled){adapter_diag(a,"DISABLE_CALL",LAB_OK,now);return adapter_stop(user,now);}
#define ENABLE_RETURN(result,stage) do{lab_status diag_result=(result);adapter_diag(a,(stage),diag_result,now);return diag_result;}while(0)
 adapter_diag(a,"ENABLE_BEGIN",LAB_OK,now);
 if(!session||a->context.source==LAB_REAL_RECORDING||a->context.clock!=LAB_REAL_MONOTONIC)ENABLE_RETURN(LAB_ERR_READONLY,"ENABLE_BAD_CONTEXT");
 if(!valid_clock(a,now))ENABLE_RETURN(LAB_ERR_TIME,"ENABLE_CLOCK_ERROR");
 if(a->enabled)ENABLE_RETURN(a->public_session==session?LAB_OK:LAB_ERR_READONLY,"ENABLE_ALREADY_ACTIVE");
 if(a->phase&&a->public_session!=session)ENABLE_RETURN(LAB_ERR_READONLY,"ENABLE_PUBLIC_SESSION_MISMATCH");
 if(a->have_health){int r=ch_guard_clear_estop(a->guard,now);if(r!=CH_OK)ENABLE_RETURN(status(r),"ENABLE_ESTOP_DENIED");}
 if(a->config.backend==LAB_CHASSIS_DRY_RUN){
  if(a->next_token==UINT32_MAX)return LAB_ERR_LIMIT;a->wire_session=++a->next_token;a->public_session=session;
  ch_receiver_health(&a->model,1,0,0,now/1000);a->peer=a->model.status;
  ch_wire_command c=wire(a,CH_CLAIM);int r=ch_receiver_accept(&a->model,&c,now/1000);if(r!=CH_OK)return status(r);
  a->peer=a->model.status;c=wire(a,CH_ARM);r=ch_receiver_accept(&a->model,&c,now/1000);if(r!=CH_OK)return status(r);
  a->peer=a->model.status;a->enabled=1;a->have_applied=0;return LAB_OK;
 }
 if(a->context.source!=LAB_REAL_HARDWARE||!a->prepared||a->config.protocol_version!=1)ENABLE_RETURN(LAB_ERR_UNAVAILABLE,"ENABLE_BACKEND_UNAVAILABLE");
 lab_chassis_feedback feedback;lab_status rc=adapter_feedback(a,now,&feedback);if(rc!=LAB_OK)ENABLE_RETURN(rc,"ENABLE_FEEDBACK_UNAVAILABLE");
 if(feedback.faults||(feedback.capabilities&7)!=7)ENABLE_RETURN(LAB_ERR_READONLY,"ENABLE_FEEDBACK_FAULT");
 if(a->phase&&now-a->handshake_us>=CONT_HANDSHAKE_US){adapter_stop(a,now);ENABLE_RETURN(LAB_ERR_TIME,"ENABLE_HANDSHAKE_TIMEOUT");}
 if(a->phase==0){
  if(feedback.armed)ENABLE_RETURN(LAB_ERR_READONLY,"ENABLE_FPGA_ALREADY_ARMED");if(a->next_token==UINT32_MAX)ENABLE_RETURN(LAB_ERR_LIMIT,"ENABLE_TOKEN_LIMIT");
  a->handshake_resyncs=0;a->resync_peer_us=0;a->wire_session=++a->next_token;a->public_session=session;ch_wire_command c=wire(a,CH_CLAIM);
  int r=ch_linux_send(a->transport,&c,now,now);if(r!=CH_OK)ENABLE_RETURN(status(r),"ENABLE_CLAIM_SEND_FAILED");a->phase=1;a->handshake_us=now;ENABLE_RETURN(LAB_AGAIN,"ENABLE_CLAIM_SENT_WAIT_ACK");
 }
 /* CLAIM拒绝可能与新租约发布竞争，仅对ARM前的租约竞争按新状态重试，原握手预算内最多两次；不重新同步已运动或被本地接管的控制者。

 */
 if(a->phase==1&&a->peer.reason==CH_RX_BAD_LEASE&&
    (a->peer.state==CH_RX_IDLE||a->peer.state==CH_RX_CLAIMED)&&
    !a->peer.local_stop_sources&&a->handshake_resyncs<2&&a->peer_time_us>a->resync_peer_us){
  ch_wire_command c=wire(a,CH_CLAIM);int raw=ch_linux_send(a->transport,&c,now,now);
  if(raw==CH_OK){++a->handshake_resyncs;a->resync_peer_us=a->peer_time_us;ENABLE_RETURN(LAB_AGAIN,"ENABLE_LEASE_RESYNC_CLAIM_SENT");}
  ENABLE_RETURN(status(raw),"ENABLE_LEASE_RESYNC_FAILED");
 }
 if(a->peer.session!=a->wire_session||a->peer.sequence!=a->wire_sequence)ENABLE_RETURN(LAB_AGAIN,"ENABLE_WAIT_SESSION_OR_SEQUENCE");
 if(a->phase==1&&a->peer.state==CH_RX_CLAIMED){
  ch_wire_command c=wire(a,CH_ARM);int r=ch_linux_send(a->transport,&c,now,now);if(r!=CH_OK)ENABLE_RETURN(status(r),"ENABLE_ARM_SEND_FAILED");a->phase=2;ENABLE_RETURN(LAB_AGAIN,"ENABLE_ARM_SENT_WAIT_ACK");
 }
 if(a->phase==2&&a->peer.state==CH_RX_ARMED){a->phase=0;a->enabled=1;a->have_applied=0;ENABLE_RETURN(LAB_OK,"ENABLE_ARMED");}
 ENABLE_RETURN(LAB_AGAIN,"ENABLE_WAIT_FPGA_STATE");
#undef ENABLE_RETURN
}
static int same_output(const lab_chassis_output *a,const lab_chassis_output *b){
 const lab_real_command *x=&a->command,*y=&b->command;
 if(!lab_real_context_equal(&x->context,&y->context)||x->generated_us!=y->generated_us||x->expires_us!=y->expires_us||x->lease!=y->lease||
 x->control_session!=y->control_session||x->sequence!=y->sequence||x->enabled!=y->enabled||x->motion_intent!=y->motion_intent||x->linear_m_s!=y->linear_m_s||x->angular_rad_s!=y->angular_rad_s||
 a->reasons!=b->reasons||a->armed!=b->armed||a->backend!=b->backend||a->fresh_command_required!=b->fresh_command_required||a->reserved)return 0;
 for(int i=0;i<4;++i)if(a->wheel_rpm[i]!=b->wheel_rpm[i])return 0;return 1;
}
static lab_status adapter_apply(void *user,const lab_chassis_output *output,uint64_t now){
 chassis_adapter *a=user;if(!a||!output)return LAB_ERR_ARGUMENT;
 if(!valid_clock(a,now))return LAB_ERR_TIME;
 if(!a->enabled||!a->have_cached||!same_output(output,&a->cached)||!output->command.enabled||!output->armed||output->reasons||
 !output->command.lease||output->command.control_session!=a->public_session)return LAB_ERR_READONLY;
 if(!lab_real_command_valid(&output->command,now)||now-output->command.generated_us>=a->config.max_command_age_us)return LAB_ERR_TIME;
 if(a->have_applied){uint32_t d=output->command.sequence-a->applied_sequence;if(!d||d>=UINT32_C(0x80000000))return LAB_ERR_TIME;}
 if(a->config.backend==LAB_CHASSIS_HARDWARE&&(a->context.source!=LAB_REAL_HARDWARE||!a->prepared||!a->have_peer||a->peer.fault_mask))return LAB_ERR_READONLY;
 uint64_t remaining=(output->command.expires_us-now)/1000;if(!remaining)return LAB_ERR_TIME;
 ch_wire_command c=wire(a,CH_DRIVE);c.ttl_ms=(uint16_t)(remaining>R6_COMMAND_TTL_MS?R6_COMMAND_TTL_MS:remaining);
 for(int i=0;i<4;++i){double rpm=output->wheel_rpm[i];if(!isfinite(rpm)||fabs(rpm)>a->config.max_wheel_rpm+1e-9)return LAB_ERR_ARGUMENT;
  c.rpm_centi[i]=(int16_t)(rpm*a->config.drive_sign[i]*100);}
 /* 先量化，再按最终机械轮速分类；换向等待输出合法零命令，不伪装非零意图心跳。
 */
 unsigned shape=s1_classify_lr_centi(c.rpm_centi[0],-(int)c.rpm_centi[1]);
 c.intent=shape==S1_ZERO?S1_ZERO:(uint8_t)output->command.motion_intent;
 if(!c.intent&&shape!=S1_ZERO)c.intent=(uint8_t)shape; /* 旧公共接口默认普通策略，不推断ESCAPE。 */
 if(!s1_intent_matches(c.intent,c.rpm_centi[0],-(int)c.rpm_centi[1]))return LAB_ERR_ARGUMENT;
 int r;
 if(a->config.backend==LAB_CHASSIS_DRY_RUN){a->peer=a->model.status;c.lease=a->peer.lease;r=ch_receiver_accept(&a->model,&c,now/1000);a->peer=a->model.status;}
 else r=ch_linux_send(a->transport,&c,output->command.expires_us,now);
 if(r==CH_OK){a->applied_sequence=output->command.sequence;a->have_applied=1;a->have_cached=0;}return status(r);
}
static lab_status adapter_reset(void *user,const lab_real_context *context){
 chassis_adapter *a=user;if(!a||!lab_real_context_valid(context))return LAB_ERR_ARGUMENT;
 if(context->source!=a->context.source||context->clock!=a->context.clock||lab_real_context_equal(context,&a->context))return LAB_ERR_CONTEXT;
 adapter_stop(a,a->now);ch_linux_close(a->transport);a->transport=NULL;a->context=*context;a->now=0;
 a->have_peer=a->have_health=a->selected=a->have_cached=a->have_applied=a->clock_fault=0;a->public_session=0;
 if(a->guard){ch_context c=core_context(context);int r=ch_guard_reset(a->guard,&c);if(r!=CH_OK)return status(r);}
 ch_receiver_init(&a->model,a->hardware.expected_config_id?a->hardware.expected_config_id:1);return LAB_OK;
}
static void adapter_close(void *user){chassis_adapter *a=user;if(a){adapter_stop(a,a->now);ch_linux_close(a->transport);ch_guard_close(a->guard);free(a);}}
static const lab_chassis_ops ops={LAB_REAL_API_VERSION,sizeof(lab_chassis_ops),adapter_feedback,adapter_protect,adapter_enable,adapter_apply,adapter_stop,adapter_reset,adapter_close};
lab_status lab_chassis_create(lab_chassis_binding *out,const lab_chassis_config *c,const lab_real_context *context){
 if(!out)return LAB_ERR_ARGUMENT;memset(out,0,sizeof(*out));
 if(!c||!lab_real_context_valid(context)||context->clock==LAB_REAL_UNMAPPED_CLOCK||c->reserved||
 (c->backend!=LAB_CHASSIS_DRY_RUN&&c->backend!=LAB_CHASSIS_HARDWARE)||!c->receive_watchdog_us||c->receive_watchdog_us>CONT_COMMAND_US||c->max_wheel_rpm>120)return LAB_ERR_ARGUMENT;
 if(c->backend==LAB_CHASSIS_HARDWARE&&(context->source!=LAB_REAL_HARDWARE||context->clock!=LAB_REAL_MONOTONIC||c->protocol_version!=1))return LAB_ERR_READONLY;
 chassis_hardware_options options;if(!endpoint_options(c,&options))return LAB_ERR_ARGUMENT;
 chassis_adapter *a=calloc(1,sizeof(*a));if(!a)return LAB_ERR_NOMEM;a->config=*c;a->context=*context;ch_config_default(&a->policy);
 a->hardware=options;a->prepared=c->backend==LAB_CHASSIS_HARDWARE;
 a->policy.direct_targets=c->direct_targets;a->policy.track_m=c->track_m;a->policy.left_mask=c->left_mask;a->policy.max_rpm=c->max_wheel_rpm;a->policy.max_linear_m_s=c->max_linear_m_s;
 a->policy.max_angular_rad_s=c->max_angular_rad_s;a->policy.command_age_us=c->max_command_age_us;
 a->policy.command_ttl_us=c->receive_watchdog_us<c->max_command_age_us?c->receive_watchdog_us:c->max_command_age_us;
 a->policy.health_age_us=CONT_FEEDBACK_US;a->policy.link_age_us=CONT_FEEDBACK_US;
 a->policy.scan_age_us=CONT_SCAN_US;a->policy.pose_age_us=CONT_ODOM_US; /* 与本集成的有界实时扫描策略一致，其他保护年龄保持不变。 */
 for(int i=0;i<4;++i){a->policy.diameter_m[i]=c->wheel_diameter_m[i];a->policy.motor_polarity[i]=c->drive_sign[i];}
 if(!ch_config_valid(&a->policy)){free(a);return LAB_ERR_ARGUMENT;}
 if(context->source!=LAB_REAL_RECORDING){ch_context ctx=core_context(context);int r=ch_guard_open(&a->guard,&a->policy,&ctx);if(r!=CH_OK){free(a);return status(r);}}
 ch_receiver_init(&a->model,1);out->user=a;out->ops=&ops;return LAB_OK;
}
lab_status chassis_binding_prepare(lab_chassis_binding *binding,const chassis_hardware_options *options){
 if(!binding||binding->ops!=&ops||!binding->user||!options||!options->interface_name[0]||!memchr(options->interface_name,0,sizeof(options->interface_name))||!options->expected_config_id)return LAB_ERR_ARGUMENT;
 chassis_adapter *a=binding->user;if(a->config.backend!=LAB_CHASSIS_HARDWARE||a->enabled||a->phase||a->transport)return LAB_ERR_READONLY;
 if(strcmp(options->interface_name,a->hardware.interface_name)||options->expected_config_id!=a->hardware.expected_config_id)return LAB_ERR_ARGUMENT;
 return LAB_OK;
}
size_t chassis_binding_private_bytes(const lab_chassis_binding *binding){
 if(!binding||binding->ops!=&ops||!binding->user)return 0;
 const chassis_adapter *a=binding->user;return sizeof(*a)+ch_guard_allocated_bytes(a->guard);
}
