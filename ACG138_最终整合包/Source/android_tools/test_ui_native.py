from pathlib import Path
import os,subprocess,tempfile
r=Path(__file__).resolve().parents[1]
zig=r.parent/'camera_imu_robot_v1_20fps_20261020/tools/zig-python/ziglang/zig.exe'
build=r/'build/ui';build.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env['ZIG_LOCAL_CACHE_DIR']=str(build/'zig-local');env['ZIG_GLOBAL_CACHE_DIR']=str(build/'zig-global')
sources=['rk/src/common/lab_live_contract.c','rk/src/navigation/passage.c','rk/src/app/phone_nav_snapshot.c','rk/src/app/live_nav_ui.c','tests/ui/test_flow_ui.c']
includes=['rk/include','rk/src/app','rk/src/navigation','rk/src/ui']
cmd=[str(zig),'cc','-target','x86_64-windows-gnu','-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers']
for p in includes:cmd+=['-I',str(r/p)]
cmd += [str(r/p) for p in sources]+['-lm','-o',str(build/'test-flow-ui.exe')]
subprocess.run(cmd,check=True,env=env)
with tempfile.TemporaryDirectory(prefix='acg_flow_ui_') as td:subprocess.run([str(build/'test-flow-ui.exe'),td,str(r/'validation/flow-hdmi-preview.ppm')],check=True,env=env)
