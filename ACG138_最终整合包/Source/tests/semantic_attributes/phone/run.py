from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import hashlib,json,os,shutil,subprocess
r=Path(__file__).resolve().parents[3]
a=_dev_path();b=a/'semantic-color-size-phone-tests';b.mkdir(exist_ok=True)
z=_zig_path();jdk=r.parent/'acg138_v11_odom_20261025/tools/local/python/jdk4py/java-runtime'
for folder in ['board','rk/include','rk/src/app','tests/semantic_attributes/phone']:shutil.copytree(r/folder,b/folder,dirs_exist_ok=True)
env=dict(os.environ,TMP=str(b),TEMP=str(b),ZIG_GLOBAL_CACHE_DIR=str(b/'global'),ZIG_LOCAL_CACHE_DIR=str(b/'cache'))
logs=[]
def run(cmd):
 p=subprocess.run(list(map(str,cmd)),cwd=b,env=env,capture_output=True,text=True,encoding='utf8',errors='replace');logs.append(p.stdout+p.stderr);print((p.stdout+p.stderr)[-3000:],flush=True);p.check_returncode();return p.stdout
incs=[x for f in ['board','rk/include','rk/src/app']for x in ['-I',b/f]]
for target,name in [('x86_64-windows-gnu','test.exe'),('aarch64-linux-musl','test-arm64')]:
 run([z,'cc','-target',target,'-std=c11','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-missing-field-initializers',*incs,b/'board/voice_api.c',b/'board/voice_mission.c',b/'tests/semantic_attributes/phone/test_attributes.c','-lm','-o',b/name])
 if name.endswith('.exe'):run([b/name])
classes=b/'classes';classes.mkdir(exist_ok=True)
run([jdk/'bin/java.exe','-Dfile.encoding=UTF-8','-jar',r.parent/'acg138_v11_odom_20261025/tools/local/ecj.jar','-source','8','-target','8','-encoding','UTF-8','-d',classes,r/'android/app/src/main/java/cn/acg138/rover/VoiceCommands.java',r/'tests/semantic_attributes/phone/TestAttributes.java'])
run([jdk/'bin/java.exe','-Dfile.encoding=UTF-8','-cp',classes,'cn.acg138.rover.TestAttributes'])
run(['node',r/'tests/semantic_attributes/phone/test_ui.js',r])
files=['board/voice_api.c','board/voice_api.h','board/voice_mission.c','board/voice_mission.h','board/voice_perception.h','rk/src/app/semantic_observation.h','board/phone_server.c','board/object_attributes.h','rk/src/app/object_readonly.h']
files += [p.relative_to(r).as_posix() for p in (r/'android/app/src/main').rglob('*') if p.is_file() and p.suffix in {'.java','.js','.xml'}]
files += [p.relative_to(r).as_posix() for p in (r/'tests/semantic_attributes/phone').glob('*') if p.is_file()]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
report={'status':'PASS','C_native_executed':True,'ARM64_test_linked':True,'Java_parser_executed':True,'actual_JS_executed':True,'sources':{p:sha(r/p) for p in files},'output':'\n'.join(logs),'board_tested':False,'motor_commands_sent':False,'stage':str(b)}
(r/'validation/semantic-phone-tests.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print('PASS semantic color/size phone tests')
