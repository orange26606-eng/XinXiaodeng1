from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import os,subprocess,shutil,json,hashlib
r=Path(__file__).resolve().parents[1];b=_dev_path('smooth-search-tests');b.mkdir(exist_ok=True)
z=_zig_path()
for n in ['board','rk/src','rk/include','runtime','tests/flow','tests/smooth_search']:shutil.copytree(r/n,b/n,dirs_exist_ok=True)
env=dict(os.environ,TMP=str(b),TEMP=str(b),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'cache'))
flags=['-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers','-Wno-unused-function'];inc=[x for d in ['rk/include','rk/src/app','rk/src/chassis','rk/src/ui','rk/src/navigation']for x in ['-I',str(b/d)]]
common=['rk/src/common/lab_types.c','rk/src/common/lab_codec.c','rk/src/common/lab_live_contract.c','rk/src/common/scan_core.c'];logs=[];art={}
def run(cmd):
 p=subprocess.run(list(map(str,cmd)),cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print((p.stdout+p.stderr)[-3500:],flush=True);p.check_returncode()
for name,src,extra in [('turn','tests/smooth_search/test_turn.c',['rk/src/app/object_dual_task.c']),('runtime','tests/smooth_search/runtime_turn.c',['rk/src/ui/lab_real_screen.c'])]:
 for target,suffix in [('x86_64-windows-gnu','.exe'),('aarch64-linux-musl','-arm64')]:
  out=b/(name+suffix);run([z,'cc','-target',target,'-Wl,--stack,16777216']+flags+inc+[b/src]+[b/f for f in common+extra]+['-lm','-o',out]) if suffix=='.exe' else run([z,'cc','-target',target,'-static']+flags+inc+[b/src]+[b/f for f in common+extra]+['-lm','-o',out])
  art[name+suffix]=hashlib.sha256(out.read_bytes()).hexdigest()
  if suffix=='.exe':run([out])
  else:(r/'release'/out.name).write_bytes(out.read_bytes())
sources={p.relative_to(r).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['rk/src/app','rk/src/navigation','tests/smooth_search']for p in (r/folder).rglob('*')if p.is_file() and p.suffix in ['.c','.h','.inc']}
(r/'validation/smooth-search-tests.json').write_text(json.dumps(dict(status='PASS',sources=sources,artifacts=art,output=''.join(logs),native_executed=True,arm64_linked=True,motor_IO=False,FPGA_modified=False),indent=2),encoding='utf8')
