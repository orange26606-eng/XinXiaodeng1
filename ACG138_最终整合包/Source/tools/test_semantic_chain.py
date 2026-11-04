from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import subprocess,os,json,hashlib,re
r=Path(__file__).resolve().parents[1];a=_dev_path();b=a/'semantic-integrated-chain';b.mkdir(exist_ok=True);z=_zig_path();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources=['tests/semantic_attributes/integration/test_chain.c','rk/src/app/object_dual_task.c','rk/src/app/object_json.c','rk/src/common/lab_types.c','rk/src/common/lab_live_contract.c','board/voice_api.c'];fixture=r/'tests/semantic_attributes/perception/fixtures/same-frame-color-metric-size.json'
inc=[r/'rk/include',r/'rk/src/app',r/'rk/src/navigation',r/'board'];inputs={}
def collect(p):
 p=p.resolve();n=p.relative_to(r.resolve()).as_posix()
 if n in inputs:return
 inputs[n]=sha(p)
 for name in re.findall(r'^\s*#\s*include\s*"([^"\n]+)"',p.read_text(encoding='utf8',errors='replace'),re.M):
  for q in [p.parent/name,*[d/name for d in inc]]:
   if q.is_file():collect(q);break
for n in sources:collect(r/n)
inputs[fixture.relative_to(r).as_posix()]=sha(fixture)
env=dict(os.environ,TMP=str(b),TEMP=str(b),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'cache'));logs=[];art={}
for target in ['x86_64-windows-gnu','aarch64-linux-musl']:
 exe=b/('chain.exe'if target.startswith('x86')else'chain-arm64');cmd=[z,'cc','-target',target,'-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation','-Wno-missing-field-initializers',*sum([['-I',d]for d in inc],[]),*[r/n for n in sources],'-lm','-o',exe]
 if target.startswith('x86'):cmd+=['-Wl,--stack,8388608']
 else:cmd+=['-static']
 p=subprocess.run(list(map(str,cmd)),env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);assert p.returncode==0,logs[-1]
 if target.startswith('x86'):
  p=subprocess.run([str(exe),str(fixture)],env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);assert p.returncode==0,logs[-1];print(logs[-1],flush=True)
 out=r/'build/semantic-chain'/exe.name;out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(exe.read_bytes());art[out.relative_to(r).as_posix()]=sha(out)
for n,h in inputs.items():assert sha(r/n)==h,('changed during test',n)
(r/'validation/semantic-chain.json').write_text(json.dumps(dict(status='PASS',sources=inputs,artifacts=art,native_executed=True,arm64_linked=True,synthetic_depth_explicit=True,motor_commands_sent=False,board_tested=False,output='\n'.join(logs)),indent=2)+'\n',encoding='utf8')
