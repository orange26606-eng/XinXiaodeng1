"""使用无设备传输桩测试实际AUTO HTTP事务。"""
from pathlib import Path
import hashlib,json,os,shutil,subprocess
from dev_paths import build_path,zig_path
R=Path(__file__).resolve().parents[1];B=build_path('final-auto-phone-tests')
for folder in ['board','rk/include','rk/src/app']:
 shutil.copytree(R/folder,B/folder,dirs_exist_ok=True)
s=(R/'board/phone_server.c').read_text(encoding='utf8')
handler='static void auto_switch_reply('+s.split('static void auto_switch_reply(',1)[1].split('static void motion_reply(',1)[0]
mode_body=s[s.index('    if(fpga_auto.active){',s.index('static void state_reply(')):s.index('    int rk_session=',s.index('static void state_reply('))]
harness=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "control_core.h"
#include "phone_auto_switch.h"
#include "phone_auto_sonar.h"
#include "phone_control_link.h"
#include "phone_json.h"
#include "s1_mode_state.h"
static struct phone_control control;static struct phone_control_link direct_link;
static phone_auto_switch auto_switch;static phone_sonar_auto fpga_auto;
static uint64_t navigation_request_sequence,now_us=10000000,generation=1;
static char navigation_request_session[33],control_directory[8]="test";
static int simulate=0,claiming=0,mutex,status_code,ipc_failure,wire_auto,wire_stop,nav_auto,checks;
static char captured[1024],mode[16]="drive";
static struct {int auto_mode,actual;unsigned nav_state,stop_reason,feedback_phase;} sensor;
static int fresh(uint64_t now){(void)now;return 1;}
static uint64_t micros(void){return now_us;}
static int pthread_mutex_lock(int*p){(void)p;return 0;}
static int pthread_mutex_unlock(int*p){(void)p;return 0;}
static int control_transport_ready(uint64_t n){(void)n;return 1;}
static int final_fault(void){return 0;}
static int phone_control_fence_revoke(const char*d,uint64_t*g){(void)d;*g=++generation;return 0;}
static int phone_control_fence_read(const char*d,uint64_t*g){(void)d;*g=generation;return 0;}
static int entropy(void*p,size_t n){memset(p,0x23,n);return 0;}
static char*test_getenv(const char*k){(void)k;return mode;}
#define getenv test_getenv
static void wire_send(unsigned op,unsigned motion){(void)motion;if(op==PHONE_AUTO)wire_auto++;if(op==PHONE_STOP){wire_stop++;phone_sonar_auto_cancel(&fpga_auto);}}
static int navigation_send_locked(phone_goal_message*m,const char*session,unsigned operation){(void)m;(void)session;if(operation==PHONE_NAV_AUTO_START)nav_auto++;return ipc_failure;}
static void nav_error_reply(int fd,int status,const char*op,const char*code,const char*message){(void)fd;(void)op;(void)message;status_code=status;snprintf(captured,sizeof captured,"%s",code);}
static void reply(int fd,int status,const char*type,const void*data,size_t bytes,const char*extra){(void)fd;(void)type;(void)extra;status_code=status;if(bytes>=sizeof captured)abort();memcpy(captured,data,bytes);captured[bytes]=0;}
static void phone_log_event(const char*a,int b,const char*c){(void)a;(void)b;(void)c;}
'''+handler+r'''
static s1_mode_state auto_status(uint64_t now){s1_mode_state rk={0};
'''+mode_body+r'''
return rk;}

#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d line%d response=%s\n",checks,__LINE__,captured);abort();}}while(0)
static const char*session="11111111111111111111111111111111";
static void reset(void){memset(&control,0,sizeof control);phone_control_session(&control,session,7,now_us/1000);phone_switch_clear(&auto_switch);phone_sonar_auto_cancel(&fpga_auto);memset(navigation_request_session,0,33);navigation_request_sequence=0;wire_auto=wire_stop=nav_auto=0;ipc_failure=claiming=0;direct_link.policy_mismatch=0;snprintf(mode,sizeof mode,"drive");}
static void prepare(void){auto_switch_reply(0,"{\"seq\":1}",session,1);CHECK(status_code==200&&auto_switch.valid&&wire_stop==1);}
static void consume(void){char body[160];snprintf(body,sizeof body,"{\"seq\":2,\"switch_token\":\"%s\"}",auto_switch.token);auto_switch_reply(0,body,session,0);}
int main(void){
 reset();prepare();uint64_t issued=auto_switch.issued_us;consume();CHECK(status_code==200&&fpga_auto.active&&control.autonomous);CHECK(wire_auto==1&&nav_auto==0);CHECK(fpga_auto.request_us==issued&&!strcmp(fpga_auto.session,session));CHECK(strstr(captured,"queued")&&strstr(captured,"auto_request_us"));
 consume();CHECK(status_code==409&&wire_auto==1);
 reset();prepare();snprintf(mode,sizeof mode,"preview");consume();CHECK(status_code==409&&!fpga_auto.active&&wire_auto==0);
 reset();prepare();snprintf(mode,sizeof mode,"cruise");consume();CHECK(status_code==200&&fpga_auto.active&&wire_auto==1);
 reset();direct_link.policy_mismatch=1;auto_switch_reply(0,"{\"seq\":1}",session,1);CHECK(status_code==409&&wire_auto==0);
 reset();prepare();generation++;consume();CHECK(status_code==409&&!fpga_auto.active);
 reset();ipc_failure=1;prepare();consume();CHECK(status_code==200&&wire_auto==1&&nav_auto==0);
 reset();control.last_rx_ms=0;auto_switch_reply(0,"{\"seq\":1}",session,1);CHECK(status_code==401&&wire_auto==0);
 reset();CHECK(phone_control_accept(&control,session,1,PHONE_AUTO,0,1,0,now_us/1000)==409);
 phone_sonar_auto_begin(&fpga_auto,session,9000,100);CHECK(!phone_sonar_auto_tick(&fpga_auto,session,1,1,0,99,299));CHECK(phone_sonar_auto_tick(&fpga_auto,session,1,1,0,299,300)==1);CHECK(phone_sonar_auto_tick(&fpga_auto,session,1,1,0,499,500)==1);CHECK(phone_sonar_auto_tick(&fpga_auto,session,1,1,0,699,700)==1);CHECK(phone_sonar_auto_tick(&fpga_auto,session,1,1,0,899,900)==-1);
 phone_sonar_auto_begin(&fpga_auto,session,9000,100);CHECK(!phone_sonar_auto_tick(&fpga_auto,session,1,1,1,101,110)&&fpga_auto.acknowledged);CHECK(!phone_sonar_auto_tick(&fpga_auto,session,1,1,1,102,500));CHECK(phone_sonar_auto_tick(&fpga_auto,session,1,1,0,103,510)==-1);CHECK(phone_sonar_auto_tick(&fpga_auto,session,0,1,1,104,520)==-1);
 phone_sonar_auto_cancel(&fpga_auto);CHECK(!phone_sonar_auto_tick(&fpga_auto,session,1,1,1,1000,1000));
 reset();phone_sonar_auto_begin(&fpga_auto,session,9900,100);sensor.auto_mode=0;
 s1_mode_state state=auto_status(100);CHECK(state.known&&state.intent_accepted&&!state.active&&state.waiting&&state.request_us==9900);
 sensor.auto_mode=1;sensor.actual=1;sensor.nav_state=1;state=auto_status(101);CHECK(state.active&&!state.waiting&&!strcmp(state.status,"ACTIVE"));
 phone_sonar_auto failed=fpga_auto;wire_send(PHONE_STOP,0);failed.active=0;failed.failed=1;fpga_auto=failed;
 state=auto_status(900);CHECK(state.known&&!state.active&&!state.intent_accepted&&state.request_us==9900&&!strcmp(state.status,"DENIED"));
 CHECK(strstr(state.reason,"FPGA未确认")!=NULL);
 fpga_auto.acknowledged=1;state=auto_status(901);CHECK(strstr(state.reason,"FPGA已退出")!=NULL);
 phone_sonar_auto_cancel(&fpga_auto);state=auto_status(902);CHECK(!state.known);
 phone_sonar_auto_begin(&fpga_auto,session,9910,903);CHECK(fpga_auto.active&&!fpga_auto.failed&&fpga_auto.request_us==9910);
 CHECK(strstr(phone_sonar_auto_reason(12,1,0,1),"近障")!=NULL);CHECK(strstr(phone_sonar_auto_reason(1,0,0,0),"确认")!=NULL);
 printf("PASS AUTO phone transaction checks=%d real-handler/no-device, token/session/fence/preview, FPGA-only launch, no-map-IPC, legacy isolation, retries/ACK/cancel\n",checks);return 0;
}
'''
(B/'test-auto-phone.c').write_text(harness,encoding='utf8')
env=dict(os.environ,TMP=str(B),TEMP=str(B),ZIG_GLOBAL_CACHE_DIR=str(B/'global'),ZIG_LOCAL_CACHE_DIR=str(B/'cache'))
logs=[]
for target,suffix in [('x86_64-windows-gnu','.exe'),('aarch64-linux-musl','-arm64')]:
 out=B/('test-auto-phone'+suffix)
 cmd=[zig_path(),'cc','-target',target,'-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation','-Wno-missing-field-initializers','-I',B/'board','-I',B/'rk/include','-I',B/'rk/src/app',B/'test-auto-phone.c',B/'board/control_core.c','-lm','-o',out]
 if suffix!='.exe':cmd+=['-static']
 p=subprocess.run(list(map(str,cmd)),env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);assert p.returncode==0,logs[-1]
 if suffix=='.exe':
  p=subprocess.run([str(out)],env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print(logs[-1][-1500:],flush=True);assert p.returncode==0
proof=dict(status='PASS',native_executed=True,ARM64_linked=True,motor_commands_sent=False,hardware_tested=False,sources={p.relative_to(R).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()for p in[R/'board/phone_server.c',R/'board/phone_auto_sonar.h',R/'board/control_core.c',R/'board/phone_auto_switch.h',R/'board/phone_json.h']},output='\n'.join(logs))
(R/'validation/auto-phone-tests.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2),encoding='utf8')
