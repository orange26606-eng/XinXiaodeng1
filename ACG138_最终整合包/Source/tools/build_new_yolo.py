from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import os,shutil,subprocess,json,hashlib,struct
r=Path(__file__).resolve().parents[1];a=_dev_path();b=a/'personnew-yolo-build';b.mkdir(exist_ok=True);z=_zig_path()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs={p.relative_to(r).as_posix():sha(p)for folder in ['board/perception','tests/yolo_fpgaimu/model']for p in(r/folder).rglob('*')if p.is_file()and p.suffix in {'.h','.cpp','.hpp'}}
for folder in ['board','tests/yolo_fpgaimu/model']:shutil.copytree(r/folder,b/folder,dirs_exist_ok=True)
source=(r/'board/perception/yolo11_pipeline.cpp').read_text(encoding='utf8')
slot=source[source.index('struct Slot {'):source.index('static const char*label(int category)',source.index('struct Slot {'))]
pipeline=source[source.index('struct Pipeline {'):source.index('int main(',source.index('struct Pipeline {'))]
(b/'tests/yolo_fpgaimu/model/production_pipeline.inc').write_bytes((slot+pipeline).encode('utf8'))
(b/'build').mkdir(exist_ok=True);env=dict(os.environ,TMP=str(b/'build'),TEMP=str(b/'build'),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'local'))
logs=[]
def run(args):
 p=subprocess.run(list(map(str,args)),cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print((p.stdout+p.stderr)[-2500:],flush=True);p.check_returncode();return p
flags=['-std=c++17','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers']
inc=['-I',b/'board/perception','-I',b/'tests/yolo_fpgaimu/model'];exe=b/'test-model.exe'
run([z,'c++','-target','x86_64-windows-gnu','-Wl,--stack,8388608',*flags,*inc,b/'tests/yolo_fpgaimu/model/test_model_pipeline.cpp','-o',exe]);native=run([exe]);arm=b/'test-model-arm64';run([z,'c++','-target','aarch64-linux-musl','-static',*flags,*inc,b/'tests/yolo_fpgaimu/model/test_model_pipeline.cpp','-pthread','-o',arm])
production=b/'yolo11-pipeline';run([z,'c++','-target','aarch64-linux-gnu.2.28','-std=c++17','-O3','-DNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-pthread',b/'board/perception/yolo11_pipeline.cpp','-ldl','-lm','-o',production])
raw=production.read_bytes();assert raw[:4]==b'\x7fELF'and struct.unpack_from('<H',raw,18)[0]==183
for rel,h in inputs.items():assert sha(r/rel)==h,('source changed during build',rel)
shutil.copy2(production,r/'runtime/camera/yolo11-pipeline');report={'status':'PASS','sources':inputs,'native_executed':True,'actual_three_worker_pipeline_extracted_verbatim':True,'ARM64_linked':True,'hardware_tested':False,'rk_fps_measured':False,'motor_commands_sent':False,'stdout':native.stdout,'artifacts':{'runtime/camera/yolo11-pipeline':sha(production)},'compile_and_test_output':'\n'.join(logs),'bounds':{'inference_slots':2,'NPU_contexts':3,'raw_pending':1,'raw_active':1,'result_pending':1,'result_active':1,'max_boxes_per_eye':128,'raw_queue_bytes':4147200,'capture_drain_max_buffers':8}}
(r/'validation/new-yolo-build.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8');print('PASS production latest model pipeline ARM64 and inert native timing/ownership tests',flush=True)
