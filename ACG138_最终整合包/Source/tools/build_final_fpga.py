"""发布位流前执行完整FPGA构建与检查。"""
from pathlib import Path
import hashlib,html,json,os,re,shutil,subprocess,time
from dev_paths import build_path

R=Path(__file__).resolve().parents[1];V=R/'validation';V.mkdir(exist_ok=True)
stage=build_path('final-auto-gowin-'+time.strftime('%Y%m%d-%H%M%S'))
for p in(R/'fpga').rglob('*'):
    if p.is_file():
        rel=p.relative_to(R/'fpga');q=stage/rel;q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs={p.relative_to(R).as_posix():sha(p)for p in(R/'fpga').rglob('*')if p.is_file()}
(V/'fpga-stage.txt').write_text(str(stage),encoding='utf8')
tool=os.environ.get('GOWIN_SH','D:/tools/Gowin/Gowin_V1.9.12.03_x64/IDE/bin/gw_sh.exe')
record=dict(status='RUNNING',source_snapshot=inputs,stage=str(stage),hardware_programmed=False)
(V/'fpga-build.json').write_text(json.dumps(record,indent=2),encoding='utf8')
print('Full synthesis/place/route:',stage,flush=True)
with(V/'fpga-build.log').open('wb')as log:
    proc=subprocess.run([tool,'build.tcl'],cwd=stage,stdout=log,stderr=subprocess.STDOUT,timeout=7200)
text=(V/'fpga-build.log').read_text(encoding='utf8',errors='replace')
stem='phone_robot_final_auto_nearstop';pnr=stage/'impl/pnr'
if proc.returncode:
    record.update(status='FAIL_TOOL_FS_WITHHELD',returncode=proc.returncode)
    (V/'fpga-build.json').write_text(json.dumps(record,indent=2),encoding='utf8')
    raise RuntimeError(text[-6000:])
timing=(pnr/(stem+'_tr_content.html')).read_text(encoding='utf8',errors='replace')
counts={k:re.findall(r'Numbers of '+k+r' Violated Endpoints</td>\s*<td>(\d+)</td>',timing)for k in ['Setup','Hold']}
slacks=[float(s)for s in re.findall(r'<td[^>]*>Slack</td>\s*<td>(-?[0-9.]+)</td>',timing)]
passed=bool(slacks)and min(slacks)>=0 and all(v and all(s=='0'for s in v)for v in counts.values())
for rel,h in inputs.items():assert sha(R/rel)==h,('SOURCE_CHANGED',rel)
baseline_pin_sha256='58a5fb28fc040396689fb30d3b4584dc31c6aec2becf9a8ddbe5af5b1ccf5358'
assert sha(R/'fpga/src/vendor/camera_dual.cst')==baseline_pin_sha256,'PIN_CONSTRAINT_CHANGED'
pin=(pnr/(stem+'.pin.html')).read_text(encoding='utf8',errors='replace')
rows=[' '.join(html.unescape(re.sub('<[^>]+>',' ',x)).split())for x in re.findall(r'<tr\b[^>]*>.*?</tr>',pin,re.S|re.I)]
pins={}
for name,ball in {'imu_sda':'D16','imu_scl':'E16'}.items():
    match=[row for row in rows if re.search(r'\b'+name+r'\b',row)]
    assert match and all(re.search(r'\b'+ball+r'/1\b',s)and'LVCMOS33'in s for s in match),(name,match)
    pins[name]=match
proof=V/'gowin';proof.mkdir(exist_ok=True)
for folder in [pnr,stage/'impl/gwsynthesis']:
    for p in folder.iterdir():
        if p.is_file()and p.suffix.lower()in{'.html','.rpt','.txt','.log','.xml'}:shutil.copy2(p,proof/p.name)
record.update(status='PASS'if passed else'FAIL_TIMING_FS_WITHHELD',setup_hold=counts,minimum_reported_slack_ns=min(slacks)if slacks else None,pins_verified=pins,external_io_timing_signed_off=False,config_id='0x31110301',command_ttl_ms=500,new_synthesis_performed=True)
if passed:
    fs=pnr/(stem+'.fs');dest=R/'release'/fs.name;dest.parent.mkdir(exist_ok=True);shutil.copy2(fs,dest)
    record.update(bitstream=dest.relative_to(R).as_posix(),sha256=sha(dest),bytes=dest.stat().st_size)
(V/'fpga-build.json').write_text(json.dumps(record,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in record.items()if k!='source_snapshot'},indent=2),flush=True)
assert passed,'Bitstream withheld because internal timing failed'
