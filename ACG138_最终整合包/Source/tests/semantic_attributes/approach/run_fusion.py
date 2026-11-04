from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import hashlib,json,os,shutil,subprocess
r=Path(__file__).resolve().parents[3];a=_dev_path();b=a/'semantic-static-fusion';b.mkdir(exist_ok=True)
for d in ['rk/include','rk/src/app','rk/src/common','rk/src/navigation','board']:shutil.copytree(r/d,b/d,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copy2(Path(__file__).with_name('test_static_fusion.c'),b/'test.c')
z=_zig_path();env=dict(os.environ,TMP=str(b),TEMP=str(b),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'cache'))
flags=['-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-misleading-indentation','-Wno-missing-field-initializers'];inc=[x for d in ['.','rk/include','rk/src/app','rk/src/navigation']for x in ['-I',b/d]]
sources=['rk/src/common/lab_types.c','rk/src/common/lab_live_contract.c','rk/src/common/scan_core.c','rk/src/app/object_json.c','rk/src/app/object_map.c','rk/src/navigation/passage.c'];inputs=sources+['rk/src/app/object_scene_fusion.c','rk/src/app/object_scene.h','rk/src/app/static_wall_v10.h','rk/src/app/follow_association_v5.h','board/object_attributes.h','rk/src/app/object_readonly.h','tests/semantic_attributes/approach/test_static_fusion.c','tests/semantic_attributes/approach/run_fusion.py'];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();snapshot={p:sha(r/p)for p in inputs};logs=[];artifacts={}
for target,suffix,extra in [('x86_64-windows-gnu','.exe',['-Wl,--stack,8388608']),('aarch64-linux-musl','-arm64',['-static'])]:
 exe=b/('test'+suffix);cmd=[z,'cc','-target',target,*extra,*flags,*inc,b/'test.c',*[b/p for p in sources],'-lm','-o',exe];p=subprocess.run(list(map(str,cmd)),cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print(logs[-1],flush=True);p.check_returncode()
 if suffix=='.exe':
  p=subprocess.run([str(exe)],cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print(logs[-1],flush=True);p.check_returncode()
 out=r/'build/semantic-fusion';out.mkdir(parents=True,exist_ok=True);shutil.copy2(exe,out/exe.name);artifacts[(out/exe.name).relative_to(r).as_posix()]=sha(exe)
assert snapshot=={p:sha(r/p)for p in inputs},'source changed during test'
(r/'validation/semantic-static-fusion.json').write_text(json.dumps({'status':'PASS','sources':snapshot,'artifacts':artifacts,'native_executed':True,'ARM64_linked':True,'stdout':'\n'.join(logs),'static_only_changes':True,'motor_commands_sent':False,'board_tested':False},indent=2)+'\n',encoding='utf8')
