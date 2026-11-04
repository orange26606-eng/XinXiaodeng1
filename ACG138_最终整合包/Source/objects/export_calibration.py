"""将实测双目标定NPZ导出为OSD1运行表，使用已核对的物理左眼编号。"""
from pathlib import Path
import argparse,hashlib,json,struct,sys
deps=Path(__file__).resolve().parents[2]/'acg138_apriltag_nav_20261029/calibration/.deps'
if deps.is_dir():sys.path.insert(0,str(deps))
import cv2 as cv
import numpy as np
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
p.add_argument('--left-eye',type=int,choices=[0,1],required=True);p.add_argument('--scale',type=float,default=.25)
p.add_argument('--extrinsics-output',type=Path,help='Optional canonical left-to-right optical geometry JSON')
p.add_argument('--physical-side-verified',action='store_true',help='Record an explicit user physical eye-order confirmation, not a fit assumption')
a=p.parse_args()
if not .25<=a.scale<=.5:raise SystemExit('Runtime supports scale .25.. .5 only')
with np.load(a.input,allow_pickle=False) as f:z={k:f[k].copy() for k in f.files}
meta_path=Path(str(a.input)+'.json')
rms=float(z['rms']) if 'rms' in z else float(json.loads(meta_path.read_text(encoding='utf8'))['rms_px']) if meta_path.is_file() else float('nan')
if not np.isfinite(rms) or not 0<=rms<=2:raise SystemExit('Measured stereo RMS required and must be <=2 pixels; recalibrate or supply its measured NPZ.json report')
if tuple(z['image_size'])!=(960,1080):raise SystemExit('Expected measured per-eye960x1080 calibration')
for n in ['K1','K2','D1','D2','R','T']:
 if not np.isfinite(z[n]).all():raise SystemExit('Nonfinite measured '+n)
if z['K1'].shape!=(3,3) or z['K2'].shape!=(3,3) or z['R'].shape!=(3,3) or z['T'].size!=3:raise SystemExit('Invalid measured matrix shape')
if not np.allclose(z['R']@z['R'].T,np.eye(3),atol=1e-5) or abs(np.linalg.det(z['R'])-1)>1e-5:raise SystemExit('Invalid measured rotation')
if any(z[k][0,0]<=0 or z[k][1,1]<=0 for k in ['K1','K2']):raise SystemExit('Invalid measured focal length')
baseline=float(np.linalg.norm(z['T']))
if not .005<baseline<1:raise SystemExit('Baseline outside supported5mm..1m')
size=(round(960*a.scale),round(1080*a.scale));Ks=[]
for n in ['K1','K2']:
 k=z[n].astype(float).copy();k[0,:]*=size[0]/960;k[1,:]*=size[1]/1080;Ks.append(k)
R1,R2,P1,P2,Q,roi1,roi2=cv.stereoRectify(Ks[0],z['D1'],Ks[1],z['D2'],size,z['R'],z['T'],flags=cv.CALIB_ZERO_DISPARITY,alpha=0)
if P2[0,3]>=0 or abs(P2[1,3])>abs(P2[0,3]):raise SystemExit('Expected horizontal physicalLEFT→RIGHT calibration')
# 使用原K和缩放后的输出P，使每个校正像素直接映射回
# 原960×1080输入，并用于原始检测ROI关联。
maps=[]
for K,D,R,P in [(z['K1'],z['D1'],R1,P1),(z['K2'],z['D2'],R2,P2)]:
 mx,my=cv.initUndistortRectifyMap(K,D,R,P,size,cv.CV_32FC1);maps +=[mx,my]
D=np.zeros(5);D[:min(5,z['D1'].size)]=z['D1'].ravel()[:5]
header=struct.pack('<8I2d',0x3144534f,1,48,960,1080,*size,a.left_eye,baseline,0.)
payload=header+b''.join(np.asarray(x,dtype='<f8').tobytes() for x in [Q,R1,z['K1'],D])+struct.pack('<8I',*roi1,*roi2)+b''.join(np.asarray(x,dtype='<f4').tobytes() for x in maps)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_bytes(payload)
meta={'format':'OSD1','source':str(a.input),'source_sha256':hashlib.sha256(a.input.read_bytes()).hexdigest(),'runtime_sha256':hashlib.sha256(payload).hexdigest(),'rms_px':rms,'max_rms_px':2,'quality':'EXPERIMENTAL_COARSE','original_size':[960,1080],'rectified_size':list(size),'scale':a.scale,'physical_left_image_eye':a.left_eye,'baseline_m':baseline,'time_source':'RK_DEQUEUE','exposure_synchronized':False,'camera_mount_verified':False,'hardware_accuracy_verified':False,'physical_side_verified':a.physical_side_verified,'algorithm':'runtime Census5x5 + four-path SGM, not OpenCV StereoSGBM','bytes':len(payload)}
Path(str(a.output)+'.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(meta,ensure_ascii=False,indent=2))

# R和T始终从原始物理左眼光学坐标映射到右眼
# 光学坐标，即X_right=R×X_left+T，与当前显示眼选择无关。
if a.extrinsics_output:
 extra={'version':1,'left_eye':a.left_eye,'right_eye':1-a.left_eye,'R':z['R'].reshape(-1).tolist(),'T':z['T'].reshape(-1).tolist(),
        'K1':z['K1'].reshape(-1).tolist(),'D1':z['D1'].reshape(-1).tolist(),'K2':z['K2'].reshape(-1).tolist(),'D2':z['D2'].reshape(-1).tolist(),
        'baseline_m':baseline,'rms_px':rms,'physical_side_verified':a.physical_side_verified,'physical_side_confirmation':'User confirms eye0 LEFT and eye1 RIGHT on 2026-11-01' if a.physical_side_verified else 'UNVERIFIED',
        'source_sha256':meta['source_sha256'],'source_name':a.input.name,'runtime_sha256':meta['runtime_sha256'],'camera_to_body_verified':False,'exposure_synchronized':False,'hardware_accuracy_verified':False}
 a.extrinsics_output.parent.mkdir(parents=True,exist_ok=True)
 a.extrinsics_output.write_text(json.dumps(extra,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
