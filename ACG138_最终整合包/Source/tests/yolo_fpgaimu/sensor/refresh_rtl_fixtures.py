"""原样提取实际GMII仿真载荷，不构造测量值。"""
from pathlib import Path
import json,hashlib
r=Path(__file__).resolve().parents[3];here=Path(__file__).parent
wire=r/'tests/yolo_fpgaimu/fpga/artifacts/vehicle-wire.hex'
packets=[bytes.fromhex(line.strip()) for line in wire.read_text().splitlines() if line.strip()]
raw=[];att=[]
for p in packets:
 frame=p[8:-4]
 if len(frame)<42:continue
 payload=frame[42:]
 if payload[:4]==b'I9E1':raw.append(payload)
 elif payload[:4]==b'F6A1':att.append(payload)
assert len(raw)==38 and len(att)==38 and all(len(x)==100 for x in raw+att)
body='/* Actual full-vehicle RTL simulation GMII payloads; not physical captures. */\n'
for name,values in [('rtl_raw',raw),('rtl_attitude',att)]:
 body+='static const unsigned char '+name+'['+str(len(values))+'][100]={\n'
 body+='\n'.join('{'+','.join('0x%02x'%b for b in x)+'},'for x in values)+'\n};\n'
(here/'rtl_fixtures.h').write_text(body,encoding='utf8')
sources=[wire,r/'tests/yolo_fpgaimu/fpga/artifacts/sample-attitude.bin']
(here/'rtl-fixture-provenance.json').write_text(json.dumps({'source':'FPGA full-vehicle RTL simulation (actual3896-bytecameraUDPstress and real nav_control_link status), not board','raw_frames':len(raw),'attitude_frames':len(att),'sources':{p.relative_to(r).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}},indent=2)+'\n',encoding='utf8')
print('Refreshed unchanged actual RAW/F6 payloads',hashlib.sha256(wire.read_bytes()).hexdigest())
