from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import os,shutil,subprocess,json,hashlib,sys
r=Path(__file__).resolve().parents[1];a=_dev_path();b=a/'sensor-fpgaimu';b.mkdir(exist_ok=True);z=_zig_path()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs={p.relative_to(r).as_posix():sha(p)for folder in ['sensor/src','sensor/include','sensor/third_party','tests/yolo_fpgaimu/sensor']for p in(r/folder).rglob('*')if p.is_file()and p.suffix in{'.c','.h','.inc','.py'}}
for folder in ['sensor','tests/yolo_fpgaimu/sensor']:shutil.copytree(r/folder,b/folder,dirs_exist_ok=True)
shutil.copytree(r.parent/'acg138_r81_object_v10_1_20261103/sensor',b/'baseline_sensor',dirs_exist_ok=True)
env=dict(os.environ,TMP=str(b),TEMP=str(b),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'cache'))
inc=[x for d in ['sensor/include','sensor/src/app','sensor/src/live_io','sensor/third_party/odometry']for x in ['-I',b/d]]
flags=['-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers']
core=[b/'sensor'/p for p in ['src/common/lab_types.c','src/common/lab_codec.c','src/app/fpga_attitude.c','src/app/lab_real_mpusvc.c','src/app/lab_real_mpusvc_file.c','src/app/lab_real_mpusvc_vendor_odom.c','src/app/lab_real_mpusvc_vendor_fusion.c','src/app/lab_real_mpusvc_vendor_adapter.c']]
(b/'fixtures').mkdir(exist_ok=True)
logs=[];art={}
def run(cmd):
 p=subprocess.run(list(map(str,cmd)),cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print((p.stdout+p.stderr)[-8000:],flush=True);p.check_returncode()
for target,suffix in [('x86_64-windows-gnu','.exe'),('aarch64-linux-musl','-arm64')]:
 exe=b/('test-sensor-fpgaimu'+suffix);run([z,'cc','-target',target,*flags,*inc,*core,b/'tests/yolo_fpgaimu/sensor/test_sensor.c','-lm','-o',exe]);art[exe.name]=sha(exe)
 if suffix=='.exe':
  run([exe]);fixture_dir=r/'tests/yolo_fpgaimu/sensor/fixtures';fixture_dir.mkdir(exist_ok=True)
  for f in (b/'fixtures').iterdir():shutil.copy2(f,fixture_dir/f.name)
old_inc=[x for d in ['include','src/app','src/live_io','third_party/odometry']for x in ['-I',b/'baseline_sensor'/d]]
old_core=[b/'baseline_sensor'/p for p in ['src/common/lab_types.c','src/common/lab_codec.c','src/app/lab_real_mpusvc.c','src/app/lab_real_mpusvc_file.c','src/app/lab_real_mpusvc_vendor_odom.c','src/app/lab_real_mpusvc_vendor_fusion.c','src/app/lab_real_mpusvc_vendor_adapter.c']]
old_exe=b/'test-old-consumer.exe';run([z,'cc','-target','x86_64-windows-gnu',*flags,*old_inc,*old_core,b/'tests/yolo_fpgaimu/sensor/test_old_consumer.c','-lm','-o',old_exe]);run([old_exe]);art[old_exe.name]=sha(old_exe)
posix=(b/'sensor/src/app/lab_real_mpusvc_posix.c').read_text(encoding='utf8')
function='static void diagnostics('+posix.split('static void diagnostics(',1)[1].split('static void *publish_loop(',1)[0]
harness=r"""#include "lab_real_mpusvc_posix.h"
#include <stdatomic.h>
#define main underlying_test_main
#include "tests/yolo_fpgaimu/sensor/test_sensor.c"
#undef main
typedef struct service {const mpusvc_options*options;char diagnostic[32];atomic_uint_fast64_t timing_total;atomic_uint timing_consecutive,timing_window;int attitude_port_error,io_error;}service;
static char captured[8192];static int atomic_file(const char*p,const void*b,size_t n){(void)p;assert(n<sizeof captured);memcpy(captured,b,n);captured[n]=0;return 0;}
"""+function+r"""
int main(void){mpusvc_core*c=open_core(F6_MODE_AUTO);uint8_t p[100],f[100];raw_packet(p,1,50000,0,0,15);f6_packet(f,1,49000,0,.1,.2,.25);CHECK(mpusvc_push_attitude(c,f,100,1000000)==LAB_OK);CHECK(mpusvc_submit_raw(c,p,100,1000001,1000001)==LAB_OK);mpusvc_view v;mpusvc_snapshot(c,1001000,&v);mpusvc_options opt={.attitude_mode=F6_MODE_AUTO};service s={.options=&opt};atomic_init(&s.timing_total,0);atomic_init(&s.timing_consecutive,0);atomic_init(&s.timing_window,0);diagnostics(&s,&v,1001000);assert(!s.io_error);puts(captured);mpusvc_close(c);return 0;}
"""
(b/'test-diagnostics.c').write_text(harness,encoding='utf8');exe=b/'test-diagnostics.exe';run([z,'cc','-target','x86_64-windows-gnu',*flags,*inc,'-I',b,*core,b/'test-diagnostics.c','-lm','-o',exe]);run([exe]);parsed=json.loads(logs[-1].split('IMU_PAIR')[0].strip());assert parsed['source']=='FPGA6_ATTITUDE_ENCODER_ODOMETRY'and parsed['fpga_attitude']['paired']==1
for name,main in [('lab-mpu9250-service','lab_real_mpusvc_main.c'),('lab-mpu9250-info','lab_real_mpusvc_info.c')]:
 exe=b/name;run([z,'cc','-target','aarch64-linux-musl','-static',*flags,*inc,*core,b/'sensor/src/app/lab_real_mpusvc_posix.c',b/'sensor/src/app'/main,'-lm','-pthread','-o',exe]);shutil.copy2(exe,r/'runtime'/name);art[name]=sha(exe)
for rel,h in inputs.items():assert sha(r/rel)==h,('source changed',rel)
(r/'validation/sensor-fpgaimu-build.json').write_text(json.dumps({'status':'PASS','source_snapshot':inputs,'artifacts':art,'native_executed':True,'arm64_linked':True,'hardware_tested':False,'motor_commands_sent':False,'build_script_sha256':sha(Path(__file__)),'bounds':{'raw_pending_packets':16,'attitude_cache_entries':64,'raw_wait_us':12000,'attitude_source_max_age_us':50000,'attitude_first_rx_max_age_us':50000,'snapshot_bytes':768,'legacy_bytes':4224,'binary_wire_source_marker':False},'baseline_reader_inputs':{p.relative_to(r.parent/'acg138_r81_object_v10_1_20261103').as_posix():sha(p)for folder in ['src','include','third_party']for p in(r.parent/'acg138_r81_object_v10_1_20261103/sensor'/folder).rglob('*')if p.is_file()and p.suffix in{'.c','.h'}},'output':'\n'.join(logs)},ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print('PASS FPGA attitude sensor integration + ARM64 service/info')
