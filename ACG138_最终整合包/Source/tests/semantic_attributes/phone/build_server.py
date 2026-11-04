from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import os,shutil,subprocess,hashlib,json,re
r=Path(__file__).resolve().parents[3];a=_dev_path();b=a/'semantic-color-size-phone-server';b.mkdir(exist_ok=True)
z=_zig_path();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files=['voice_api.c','voice_json.c','voice_mission.c','phone_server.c','phone_control_link.c','video/video_service.c','video/h264_encoder.c','control_core.c','reference/dashboard_video.c','reference/robot_protocol.c','odometry.c','odometry_runtime.c','mpu9250/imu_fusion.c','mpu9250/mpu_adapter.c']
units=[r/'rk/src/app/tag_goal.c',r/'rk/src/app/object_json.c',*[r/'board'/f for f in files]];incdirs=[r/'board',r/'rk/include',r/'rk/src/app'];deps={}
def collect(f):
 rel=f.relative_to(r).as_posix()
 if rel in deps:return
 deps[rel]=sha(f)
 for name in re.findall(r'^\s*#\s*include\s*"([^"\n]+)"',f.read_text(encoding='utf8',errors='replace'),re.M):
  for candidate in [f.parent/name,*[d/name for d in incdirs]]:
   if candidate.is_file():collect(candidate.resolve());break
for f in units:collect(f.resolve())
for rel in deps:
 dest=b/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(r/rel,dest)
env=dict(os.environ,TMP=str(b),TEMP=str(b),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'cache'))
exe=b/'acg138-phone-server';cmd=[z,'cc','-target','aarch64-linux-gnu.2.28','-std=gnu11','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers','-pthread','-I',b/'rk/include','-I',b/'rk/src/app',*[b/f.relative_to(r) for f in units],'-lm','-ldl','-o',exe]
p=subprocess.run(list(map(str,cmd)),env=env,cwd=b,capture_output=True,text=True,encoding='utf8',errors='replace');print(p.stdout+p.stderr,flush=True);p.check_returncode()
for rel,digest in deps.items():assert sha(r/rel)==digest,('source changed while compiling',rel)
(r/'build').mkdir(exist_ok=True);shutil.copy2(exe,r/'build/acg138-phone-server-map-dev')
(r/'validation/semantic-phone-server.json').write_text(json.dumps({'status':'PASS','target':'aarch64-linux-gnu.2.28','sources':deps,'artifacts':{'build/acg138-phone-server-map-dev':sha(exe)},'source_file_count':len(deps),'compile_output':p.stdout+p.stderr,'device_tested':False,'motion_commands_sent':False,'stage':str(b)},indent=2)+'\n',encoding='utf8')
print('PASS semantic color/size phone server ARM64',sha(exe),flush=True)
