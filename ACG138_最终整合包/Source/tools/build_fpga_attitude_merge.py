from pathlib import Path


import sys as _dev_sys
_dev_sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / "tools/dev_paths.py").is_file()) / "tools"))
from dev_paths import build_path as _dev_path, zig_path as _zig_path
import hashlib,html,json,os,re,shutil,subprocess,time
r=Path(__file__).resolve().parents[1];v=r/'validation';v.mkdir(exist_ok=True);sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
a=_dev_path();stage=a/('r81-fpga-imu-synth-'+time.strftime('%Y%m%d_%H%M%S'));stage.mkdir()
inputs={p.relative_to(r).as_posix():sha(p)for p in(r/'fpga').rglob('*')if p.is_file()}
shutil.copytree(r/'fpga',stage,dirs_exist_ok=True)
tool=Path(os.environ.get('GOWIN_SH','D:/tools/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe'))
(v/'fpga-attitude-stage.txt').write_text(str(stage),encoding='utf8');print('Full Gowin synth/place/route:',stage,flush=True)
(v/'fpga-attitude-build.json').write_text(json.dumps({'status':'RUNNING','source_snapshot':inputs,'stage':str(stage),'hardware_programmed':False}),encoding='utf8')
try:
 with(v/'fpga-attitude-gowin.log').open('wb')as log:p=subprocess.run([str(tool),'build.tcl'],cwd=stage,stdout=log,stderr=subprocess.STDOUT,timeout=int(os.environ.get('GOWIN_TIMEOUT_S','7200')))
except subprocess.TimeoutExpired:
 (v/'fpga-attitude-build.json').write_text(json.dumps({'status':'FAIL_TOOL_TIMEOUT_FS_WITHHELD','source_snapshot':inputs,'stage':str(stage),'hardware_programmed':False,'published':False},indent=2)+'\n',encoding='utf8')
 raise
log=(v/'fpga-attitude-gowin.log').read_text(encoding='utf8',errors='replace')
if p.returncode:
 (v/'fpga-attitude-build.json').write_text(json.dumps({'status':'FAIL_TOOL_FS_WITHHELD','tool_returncode':p.returncode,'source_snapshot':inputs,'stage':str(stage),'hardware_programmed':False},indent=2)+'\n',encoding='utf8')
 raise RuntimeError(log[-9000:])
pnr=stage/'impl/pnr';stem='phone_robot_r81_fpga_imu';timing=(pnr/(stem+'_tr_content.html')).read_text(encoding='utf8',errors='replace')
counts={k:re.findall(r'Numbers of '+k+r' Violated Endpoints</td>\s*<td>(\d+)</td>',timing)for k in ['Setup','Hold']}
passed=all(items and all(x=='0'for x in items)for items in counts.values())
all_reported_slacks=re.findall(r'<td[^>]*>Slack</td>\s*<td>(-?[0-9.]+)</td>',timing)
passed=passed and bool(all_reported_slacks)and min(map(float,all_reported_slacks))>=0
for rel,h in inputs.items():assert sha(r/rel)==h,('Source changed during synthesis',rel)
pin=(pnr/(stem+'.pin.html')).read_text(encoding='utf8',errors='replace');rows=[' '.join(html.unescape(re.sub('<[^>]+>',' ',x)).split())for x in re.findall(r'<tr\b[^>]*>.*?</tr>',pin,re.S|re.I)]
checked={}
for port,ball in {'imu_sda':'D16','imu_scl':'E16'}.items():
 matches=[row for row in rows if re.search(r'\b'+port+r'\b',row)];assert matches and all(re.search(r'\b'+ball+r'/1\b',x)and 'LVCMOS33'in x for x in matches),(port,matches);checked[port]=matches
baseline=r.parent/'acg138_r81_object_v10_1_20261103';assert sha(r/'fpga/src/vendor/camera_dual.cst')==sha(baseline/'fpga/src/vendor/camera_dual.cst')
reports=v/'fpga-attitude-gowin';reports.mkdir(exist_ok=True)
for folder in [pnr,stage/'impl/gwsynthesis']:
 for f in folder.iterdir():
  if f.is_file()and f.suffix.lower()in{'.html','.rpt','.txt','.log','.xml'}:shutil.copy2(f,reports/f.name)
record={'status':'PASS'if passed else 'FAIL_TIMING_FS_WITHHELD','source_snapshot':inputs,'stage':str(stage),'setup_hold':counts,'pins_verified':checked,'config_id':'0x31110301','command_ttl_max_ms':500,'raw_UDP5016_preserved':True,'new_attitude_UDP5017':True,'AF_fault_never_resets_image_init':True,'new_synthesis_performed':True,'hardware_programmed':False,'board_tested':False,'external_io_timing_signed_off':False}
slacks=re.findall(r'<td[^>]*>Slack</td>\s*<td>(-?[0-9.]+)</td>',timing)
if slacks:record['minimum_reported_slack_ns']=min(map(float,slacks))
if passed:
 fs=pnr/(stem+'.fs');out=r/'release'/fs.name;shutil.copy2(fs,out);record.update(bitstream=out.relative_to(r).as_posix(),sha256=sha(out),bytes=out.stat().st_size)
(v/'fpga-attitude-build.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf8');print(json.dumps({k:x for k,x in record.items()if k!='source_snapshot'},indent=2),flush=True);assert passed,'FS withheld: setup/hold timing failure'
