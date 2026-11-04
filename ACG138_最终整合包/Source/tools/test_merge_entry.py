from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import os,subprocess,hashlib,json
r=Path(__file__).resolve().parents[1];a=_dev_path();b=a/'semantic-color-size-entry';b.mkdir(exist_ok=True)
bash=Path('D:/tools/github/Git/bin/bash.exe')
(b/'begin').write_bytes((r/'runtime/begin').read_bytes())
(b/'robotctl').write_bytes(b'#!/bin/sh\nprintf "mode=%s cruise=%s action=%s app=%s phone=%s\\n" "$ROBOT_NAV_MODE" "$ACG_LIDAR_CRUISE" "$1" "$ROBOT_RUN_DIR" "$PHONE_RUN_DIR"\n')
env=os.environ.copy()
for k in ['ROBOT_NAV_MODE','ROBOT_RUN_DIR','PHONE_RUN_DIR','ACG_LIDAR_CRUISE']:env.pop(k,None)
results={}
for op,expected in [('drive','mode=drive cruise=0 action=start'),('cruise','mode=cruise cruise=1 action=start'),('status','action=status'),('stop','action=stop')]:
 p=subprocess.run([str(bash),str(b/'begin'),op],cwd=b,env=env,capture_output=True,text=True,encoding='utf8');p.check_returncode();assert expected in p.stdout,p.stdout;assert 'acg138-rk-smooth-search-app'in p.stdout and 'acg138-rk-smooth-search-phone'in p.stdout;results[op]=p.stdout.strip()
scripts={}
for file in(r/'runtime').rglob('*'):
 if not file.is_file():continue
 raw=file.read_bytes()
 if not raw.startswith(b'#!'):continue
 assert b'\r\n'not in raw,file
 p=subprocess.run([str(bash),'-n',str(file.resolve())],env=env,capture_output=True,text=True,encoding='utf8');assert not p.returncode,(file,p.stderr);scripts[file.relative_to(r).as_posix()]=hashlib.sha256(raw).hexdigest()
(r/'validation/semantic-entry.json').write_text(json.dumps({'status':'PASS','sources':scripts,'entry_results':results,'script_LF_verified':True,'services_started':False,'motor_commands_sent':False,'network_changed':False},indent=2)+'\n',encoding='utf8');print('PASS exact begin drive/cruise/status/stop and',len(scripts),'LF scripts')
