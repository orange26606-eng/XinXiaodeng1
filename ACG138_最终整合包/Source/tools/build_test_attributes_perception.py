from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import os,hashlib,json,shutil,subprocess
r=Path(__file__).resolve().parents[1];a=_dev_path();b=a/'semantic-attributes-perception';b.mkdir(exist_ok=True);z=_zig_path()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs={p.relative_to(r).as_posix():sha(p)for folder in ['board/perception','objects','tests/semantic_attributes/perception']for p in(r/folder).rglob('*')if p.is_file()and p.suffix in {'.h','.cpp','.hpp','.inc'}}
inputs['board/object_attributes.h']=sha(r/'board/object_attributes.h')
inputs['tools/build_test_attributes_perception.py']=sha(Path(__file__))
for folder in ['board','objects','tests/semantic_attributes/perception']:shutil.copytree(r/folder,b/folder,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
s=(r/'board/perception/yolo11_pipeline.cpp').read_text(encoding='utf8');slot=s[s.index('struct Slot {'):s.index('struct Pipeline {')];pipeline=s[s.index('struct Pipeline {'):s.index('int main(',s.index('struct Pipeline {'))]
(b/'tests/semantic_attributes/perception/production_pipeline.inc').write_text(slot+pipeline,encoding='utf8')
(b/'temp').mkdir(exist_ok=True);env=dict(os.environ,TMP=str(b/'temp'),TEMP=str(b/'temp'),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'local'))
logs=[]
def run(args):
 p=subprocess.run(list(map(str,args)),cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace',timeout=360);logs.append(p.stdout+p.stderr);(r/'validation/attributes-perception-build.log').write_text('\n'.join(logs),encoding='utf8');print((p.stdout+p.stderr)[-3000:],flush=True);p.check_returncode();return p
flags=['-std=c++17','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers','-Wno-unused-but-set-variable','-Wno-unused-function']
inc=['-I',b/'board/perception','-I',b/'objects','-I',b/'tests/semantic_attributes/perception'];sources=[b/'tests/semantic_attributes/perception/test_attributes.cpp',b/'objects/core.cpp']
exe=b/'test-attributes.exe';run([z,'c++','-target','x86_64-windows-gnu','-Wl,--stack,8388608',*flags,*inc,*sources,'-o',exe]);native=run([exe,b/'attributes.obf2'])
arm=b/'test-attributes-arm64';run([z,'c++','-target','aarch64-linux-musl','-static',*flags,*inc,*sources,'-pthread','-o',arm])
yolo=b/'yolo11-pipeline';run([z,'c++','-target','aarch64-linux-gnu.2.28','-std=c++17','-O3','-DNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-pthread',b/'board/perception/yolo11_pipeline.cpp','-ldl','-lm','-o',yolo])
objects=b/'acg138-objects';run([z,'c++','-target','aarch64-linux-musl','-static','-std=c++17','-O3','-DNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers','-Wno-unused-but-set-variable',b/'objects/core.cpp',b/'objects/main.cpp','-pthread','-o',objects])
for n,h in inputs.items():assert sha(r/n)==h,('source changed during build',n)
fixtures=r/'tests/semantic_attributes/perception/fixtures';fixtures.mkdir(exist_ok=True)
for source,name in [(b/'attributes.obf2','same-frame-color.obf2'),(b/'attributes.obf2.state.json','same-frame-color-metric-size.json')]:shutil.copy2(source,fixtures/name)
(fixtures/'README.txt').write_text('Synthetic input: production NV12 color producer/worker/publisher and OBF2 reader, true state_json. Metric depths are explicit inert test inputs, NOT real camera measurements. Same exact source epoch/frame/eye/time in JSON and OBF2.\n',encoding='utf8')
out=r/'build/attributes-perception';out.mkdir(parents=True,exist_ok=True)
for f in [arm,yolo,objects]:shutil.copy2(f,out/f.name)
report={'status':'PASS','source_snapshot':inputs,'artifacts':{p.relative_to(r).as_posix():sha(p)for p in out.iterdir()if p.is_file()},'native_executed':True,'actual_production_pipeline_worker_and_publisher':True,'actual_objects_reader_and_tracker_and_json':True,'ARM64_linked':True,'stdout':native.stdout,'bounds':{'color_samples_per_detection':1024,'export_per_eye_detection_limit':32,'color_input_boxes_per_eye':128,'export_record_bytes':64,'OBF2_max_bytes':64+2073600+64*64,'frame_inference_slots':2,'publish_pending':1,'state_json_bytes':262144,'dimensions':'visible-plane approximation, not full3D metrology'},'fixture_sha256':{p.relative_to(r).as_posix():sha(p)for p in fixtures.iterdir()if p.is_file()},'physical_camera_tested':False,'new_fps_measured':False,'motor_commands_sent':False,'FPGA_changed':False}
(r/'validation/attributes-perception-build.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print('PASS semantic attribute perception native production tests + ARM64 production builds',flush=True)
