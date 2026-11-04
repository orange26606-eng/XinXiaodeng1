"""将实测单目NPZ导出为各眼运行内参，不生成安装外参。"""
from pathlib import Path
import argparse,json,hashlib,math
import numpy as np
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--physical-side-verified',action='store_true');a=p.parse_args()
with np.load(a.input,allow_pickle=False)as z:
 K=z['K'];D=z['D'].reshape(-1);rms=float(z['rms']);eye=int(z['eye'])
 if tuple(z['image_size'])!=(960,1080)or eye not in [0,1]or K.shape!=(3,3)or D.size!=5 or not np.isfinite(K).all()or not np.isfinite(D).all()or not math.isfinite(rms)or not 0<=rms<=2 or K[0,0]<=0 or K[1,1]<=0:raise SystemExit('Measured mono calibration invalid: require finite 960x1080 K/D, eye0/1, RMS<=2px')
 v={'calibration_enabled':1,'eye':eye,'reference_eye':eye,'image_width':960,'image_height':1080,'fx':K[0,0],'fy':K[1,1],'cx':K[0,2],'cy':K[1,2],**dict(zip(['k1','k2','p1','p2','k3'],D)),'rms_px':rms}
comment='# Measured mono intrinsics. '+('Physical eye0 LEFT / eye1 RIGHT confirmed by user 2026-11-01.' if a.physical_side_verified else 'Physical eye side UNVERIFIED.')+'\n'
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(comment+''.join(f'{k}={x:.14g}\n'for k,x in v.items()),encoding='utf8')
meta={'kind':'measured_mono_intrinsics','source_name':a.input.name,'source_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),'runtime_sha256':hashlib.sha256(a.output.read_bytes()).hexdigest(),'rms_px':rms,'eye':eye,'image_size':[960,1080],'physical_side_verified':a.physical_side_verified,'camera_to_body_verified':False,'hardware_accuracy_verified':False}
Path(str(a.output)+'.json').write_text(json.dumps(meta,indent=2)+'\n',encoding='utf8');print(json.dumps(meta,ensure_ascii=True))
