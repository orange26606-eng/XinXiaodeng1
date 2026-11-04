from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import os,shutil,subprocess,hashlib,json,sys
r=Path(__file__).resolve().parents[1]
a=_dev_path()
b=a/'rk-smooth-search-production';b.mkdir(parents=True,exist_ok=True)
zig=_zig_path()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
perception_only='--perception-only'in sys.argv or '--objects-only'in sys.argv
inputs={p.relative_to(r).as_posix():sha(p)for folder in (['board','objects'] if perception_only else ['rk/src','rk/include','rk/third_party','board'] if '--rk-only'in sys.argv else ['rk/src','rk/include','rk/third_party','board','objects'])for p in(r/folder).rglob('*')if p.is_file()and p.suffix in {'.c','.cpp','.h','.hpp','.inc'}}
if '--rk-only' in sys.argv:
 inputs={n:h for n,h in inputs.items() if not n.startswith('board/') or n in {'board/object_attributes.h','board/phone_shared.h','board/phone_control_fence.h'}}
for folder in ['rk','board','objects','tools']:
 shutil.copytree(r/folder,b/folder,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
(b/'build').mkdir(exist_ok=True);(b/'validation').mkdir(exist_ok=True);(b/'runtime/camera').mkdir(parents=True,exist_ok=True)
env=dict(os.environ,TMP=str(b/'build'),TEMP=str(b/'build'),ZIG_EXE=str(zig),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'cache'))
logs=[]
def run(args):
 cmd=list(map(str,args));print('RUN',cmd[-1],flush=True);p=subprocess.run(cmd,cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print((p.stdout+p.stderr)[-7000:],flush=True);p.check_returncode()
inc=[x for f in ['rk/include','rk/src/app','rk/src/ui','rk/src/live_io','rk/src/localization','rk/src/chassis','rk/src/navigation','rk/src/sim','rk/third_party/odometry']for x in ['-I',b/f]]
flags=['-DLAB_REAL_SNAPSHOT_SOURCE','-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers','-Wno-unused-function']
common=['rk/src/common/'+n+'.c'for n in ['lab_types','lab_codec','lab_live_contract','scan_core']]
units=common+['rk/src/live_io/'+n+'.c'for n in ['live_sensor','live_real_convert']]+['rk/src/localization/localization_'+n+'.c'for n in ['core','match','grid','binding']]+['rk/src/chassis/chassis_'+n+'.c'for n in ['core','wire','receiver','linux','binding']]
units+=['rk/src/app/'+n+'.c'for n in ['lab_real_runtime','lab_real_session','lab_real_mapfile','lab_real_navigation','lab_real_nav_generated','lab_real_time','lab_real_time_wire','lab_real_time_convert','tag_goal','object_dual_task','object_scene_fusion','object_radar_follow','object_scene_plan','object_json','object_map','object_lidar','object_lidar_geometry','object_navigation','lab_real_mpu','lab_real_timed_wire','lab_real_snapshot','lab_real_mpusvc','lab_real_mpusvc_file']]
units+=['rk/src/app/lab_real_'+family+'_vendor_'+n+'.c'for family in ['mpu','mpusvc']for n in ['odom','fusion','adapter']]
units+=['rk/src/ui/lab_real_screen.c','rk/src/navigation/passage.c','rk/src/navigation/passage_plan.c','rk/third_party/odometry/odometry.c']
units+=['rk/src/navigation/object_v101/'+n+'.c'for n in ['object_core','object_passage','object_passage_plan','object_binding']]
posix=['rk/src/live_io/'+n+'.c'for n in ['live_real_binding','live_shared_file','live_capture_csv']]+['rk/src/app/'+n+'.c'for n in ['lab_real_time_io','lab_real_time_source','lab_real_mpu_io','lab_real_snapshot_io']]
core=[b/x for x in units];posix=[b/x for x in posix]
if '--perception-only'not in sys.argv and '--objects-only'not in sys.argv:
 run([zig,'cc','-target','aarch64-linux-musl','-static',*flags,*inc,'-DLAB_REAL_POSIX_SOURCE',*core,*posix,b/'rk/src/app/lab_real_console.c','-lm','-pthread','-o',b/'rk/lab-real'])
 run([zig,'cc','-target','aarch64-linux-musl','-static',*flags,*inc,b/'rk/src/app/s5_manager_event.c','-pthread','-o',b/'rk/s5-manager-event'])
 way=b/'rk/third_party/wayland';panel=b/'rk/third_party/panel_outputs'
 run([zig,'cc','-target','aarch64-linux-gnu.2.28',*flags,*inc,'-DLAB_REAL_POSIX_SOURCE','-I',way,'-I',panel,*core,*posix,*[b/'rk/src/app'/x for x in ['live_nav_panel.c','phone_goal_socket.c','live_nav_ui.c','phone_nav_snapshot.c','phone_nav_lease.c']],b/'rk/src/ui/wayland_host.c',panel/'panel_outputs.c',panel/'panel_output_policy.c',way/'xdg-shell-protocol.c',way/'xdg-output-protocol.c','-L',b/'rk','-lwayland-client','-lm','-pthread','-o',b/'rk/live-map-nav-panel'])
 if '--rk-only'in sys.argv:
  names=['rk/lab-real','rk/s5-manager-event','rk/live-map-nav-panel']
 else:names=[]
else:names=[]
if '--rk-only'not in sys.argv:
 if '--objects-only'not in sys.argv:run([zig,'c++','-target','aarch64-linux-gnu.2.28','-std=c++17','-O3','-DNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-pthread',b/'board/perception/yolo11_pipeline.cpp','-ldl','-lm','-o',b/'runtime/camera/yolo11-pipeline'])
 run([zig,'c++','-target','aarch64-linux-musl','-static','-std=c++17','-O3','-DNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers','-Wno-unused-but-set-variable',b/'objects/core.cpp',b/'objects/main.cpp','-pthread','-o',b/'objects/acg138-object-perception'])
 names+=['objects/acg138-object-perception']
 if '--objects-only'not in sys.argv:names+=['runtime/camera/yolo11-pipeline']
 if not perception_only:names+=['rk/lab-real','rk/s5-manager-event','rk/live-map-nav-panel']
for rel,h in inputs.items():assert sha(r/rel)==h,('source changed',rel)
for name in names:(r/name).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(b/name,r/name)
proof={'status':'PASS','sources':inputs,'artifacts':{n:sha(r/n)for n in names},'commands_output':'\n'.join(logs),'native_executed':False,'arm64_linked':True,'board_tested':False,'motor_commands_sent':False}
report='new-objects-arm-build.json'if '--objects-only'in sys.argv else 'new-perception-build.json'if '--perception-only'in sys.argv else 'new-rk-build.json'
(r/'validation'/report).write_text(json.dumps(proof,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print('PASS merged ARM64 build',flush=True)
