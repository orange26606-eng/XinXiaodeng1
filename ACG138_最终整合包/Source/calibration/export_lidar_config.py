#!/usr/bin/env python3
'导出已接受的单目标定内参。'
from pathlib import Path
import argparse,hashlib,json,math,sys

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--eye',type=int,choices=[0,1],required=True)
    a=p.parse_args()
    import numpy as np
    with np.load(a.input,allow_pickle=False) as z:
        K=z['K']; D=z['D'].reshape(-1); size=tuple(int(v) for v in z['image_size'])
        rms=float(z['rms']); eye=int(z['eye'])
    if size!=(960,1080) or K.shape!=(3,3) or not np.isfinite(K).all() or not np.isfinite(D).all():
        p.error('Expected finite measured per-eye960x1080 mono calibration')
    if eye!=a.eye:p.error('Requested eye differs from calibrated image index')
    if not math.isfinite(rms) or rms<0 or rms>2:p.error('Mono calibration must pass original RMS<=2px gate')
    if K[0,0]<=0 or K[1,1]<=0 or not (0<=K[0,2]<960 and 0<=K[1,2]<1080):p.error('Invalid intrinsics')
    if len(D)!=5:p.error('Only measured five-coefficient distortion is supported')
    root=Path(__file__).resolve().parents[1]
    text=(root/'runtime/object-lidar.conf').read_text(encoding='utf8')
    values={'enabled':'1','calibration_enabled':'1','reference_eye':str(a.eye),
            'fx':str(float(K[0,0])),'fy':str(float(K[1,1])),
            'cx':str(float(K[0,2])),'cy':str(float(K[1,2]))}
    values.update({k:str(float(v)) for k,v in zip(['k1','k2','p1','p2','k3'],D)})
    lines=[]
    for line in text.splitlines():
        k=line.split('=',1)[0]
        lines.append(k+'='+values[k] if k in values else line)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text('\n'.join(lines)+'\n',encoding='utf8')
    meta={'input':str(a.input),'input_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),
          'output_sha256':hashlib.sha256(a.output.read_bytes()).hexdigest(),'eye':a.eye,'rms_px':rms,
          'camera_mount_verified':False,'lidar_height_verified':False,
          'hardware_accuracy_verified':False,'note':'Intrinsics only; user must measure geometry and validate projected points before enabling mounts.'}
    a.output.with_suffix(a.output.suffix+'.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf8')
    print('Intrinsics exported. Camera mount and lidar height remain UNVERIFIED.')

if __name__=='__main__':main()
