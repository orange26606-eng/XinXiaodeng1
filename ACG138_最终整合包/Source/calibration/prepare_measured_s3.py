'检查实测双目图像并拟合标定参数。'
from pathlib import Path
import concurrent.futures, hashlib, json, time, sys, argparse
import cv2 as cv
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source-project',type=Path,default=ROOT.parent/'acg138_object_memory_nav_20261031')
parser.add_argument('--output-dir',type=Path,default=ROOT/'calibration/measured')
parser.add_argument('--cached-corners-only',action='store_true',help='Refit from delivered measured corner NPZ; does not claim original PNG verification')
args=parser.parse_args();SOURCE=args.source_project
OUT=args.output_dir
OUT.mkdir(parents=True,exist_ok=True)
cv.setNumThreads(1)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
reports=[];mono=[]
for eye in [0,1]:
 path=(OUT if args.cached_corners_only else SOURCE/'calibration')/f'mono_eye{eye}_20261031.npz';report=Path(str(path)+'.json')
 j=json.loads(report.read_text(encoding='utf8'));reports.append(j)
 assert j['kind']=='measured_mono_calibration' and j['eye']==eye and j['rms_px']<=2
 if not args.cached_corners_only:
  for n,digest in j['input_sha256'].items():assert sha(SOURCE/n)==digest,n
 (OUT/path.name).write_bytes(path.read_bytes());(OUT/report.name).write_bytes(report.read_bytes())
 with np.load(path,allow_pickle=False)as z:mono.append({k:z[k].copy()for k in z.files})
common=sorted(set(Path(n).name for n in reports[0]['accepted'])&set(Path(n).name for n in reports[1]['accepted']))
assert len(common)>=10
paths=[(SOURCE/'calibration/data_combined_20261031'/f'eye{eye}'/name)for name in common for eye in [0,1]]
cache=OUT/'detected-paired-corners.npz'
previous_path=OUT/'stereo-initial-fit.npz.json'
previous=json.loads(previous_path.read_text(encoding='utf8')) if previous_path.exists() else {}
if args.cached_corners_only:
 assert cache.exists() and previous.get('source_sha256'),'Delivered measured cache/provenance missing'
started=time.monotonic()
def detect(path):
 b=np.fromfile(path,dtype=np.uint8);im=cv.imdecode(b,cv.IMREAD_GRAYSCALE)
 assert im.shape==(1080,960),path
 flags=cv.CALIB_CB_NORMALIZE_IMAGE|cv.CALIB_CB_EXHAUSTIVE
 ok,c=cv.findChessboardCornersSB(im,(9,6),flags)
 if not ok:ok,c=cv.findChessboardCornersSB(cv.createCLAHE(3,(8,8)).apply(im),(9,6),flags)
 if not ok:
  flags=cv.CALIB_CB_ADAPTIVE_THRESH|cv.CALIB_CB_NORMALIZE_IMAGE|cv.CALIB_CB_FILTER_QUADS
  ok,c=cv.findChessboardCorners(im,(9,6),flags)
  if not ok:ok,c=cv.findChessboardCorners(cv.addWeighted(im,2.,cv.GaussianBlur(im,(0,0),1.5),-1.,0),(9,6),flags)
  if ok:cv.cornerSubPix(im,c,(5,5),(-1,-1),(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,.001))
 return c.astype(np.float32)if ok else None
rejected=[];flipped=[]
if cache.exists():
 if previous.get('corner_cache_sha256'):assert sha(cache)==previous['corner_cache_sha256'],'Measured corner cache changed'
 with np.load(cache,allow_pickle=False)as z:
  common=list(z['names']);left=list(z['left']);right=list(z['right'])
else:
 with concurrent.futures.ThreadPoolExecutor(max_workers=4)as pool:corners=list(pool.map(detect,paths))
 left=[];right=[];good=[]
 for name,a,b in zip(common,corners[0::2],corners[1::2]):
  if a is None or b is None:rejected.append(name);continue
  # 无方向标记的棋盘可能使角点编号反向，比较去中心后的
  # 观测网格以统一物理角点对应关系。
  ac=a-a.mean(axis=0);bc=b-b.mean(axis=0)
  if np.mean((ac-bc[::-1])**2)<np.mean((ac-bc)**2):b=b[::-1].copy();flipped.append(name)
  good.append(name);left.append(a);right.append(b)
 common=good
 np.savez_compressed(cache,names=np.array(common),left=np.array(left),right=np.array(right))
assert len(common)>=10
obj=np.zeros((9*6,3),np.float32);obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.025
K1,D1=mono[0]['K'].copy(),mono[0]['D'].copy();K2,D2=mono[1]['K'].copy(),mono[1]['D'].copy()
rms,K1,D1,K2,D2,R,T,E,F=cv.stereoCalibrate([obj]*len(left),left,right,K1,D1,K2,D2,(960,1080),flags=cv.CALIB_FIX_INTRINSIC,criteria=(cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,100,1e-7))
R1,R2,P1,P2,Q,roi1,roi2=cv.stereoRectify(K1,D1,K2,D2,(960,1080),R,T,flags=cv.CALIB_ZERO_DISPARITY,alpha=0)
rows=[]
for a,b in zip(left,right):
 ar=cv.undistortPoints(a,K1,D1,R=R1,P=P1);br=cv.undistortPoints(b,K2,D2,R=R2,P=P2)
 rows.extend(np.abs(ar[:,0,1]-br[:,0,1]).tolist())
np.savez_compressed(OUT/'stereo-initial-fit.npz',K1=K1,D1=D1,K2=K2,D2=D2,R=R,T=T,R1=R1,R2=R2,P1=P1,P2=P2,Q=Q,roi1=np.array(roi1),roi2=np.array(roi2),image_size=np.array([960,1080]),baseline_m=float(np.linalg.norm(T)),rms=rms)
result={'kind':'measured_stereo_initial_fit','status':'PASS'if np.isfinite(rms)and rms<=2 and .005<float(np.linalg.norm(T))<1 else 'NEEDS_REVIEW','rms_px':rms,'baseline_m':float(np.linalg.norm(T)),'T':T.tolist(),'R':R.tolist(),'P2':P2.tolist(),'rectified_vertical_error_median_px':float(np.median(rows)),'rectified_vertical_error_p95_px':float(np.percentile(rows,95)),'pairs':len(common),'names':common,'source_sha256':previous['source_sha256'] if args.cached_corners_only else {str(p.relative_to(SOURCE)):sha(p)for p in paths},'original_pngs_verified_this_fit':not args.cached_corners_only,'source_project':str(SOURCE),'corner_cache_sha256':sha(cache),'input_mono_sha256':{f'mono_eye{i}_20261031.npz':sha(OUT/f'mono_eye{i}_20261031.npz')for i in [0,1]},'elapsed_s':time.monotonic()-started,'opencv':cv.__version__,'physical_eye_order_verified':True,'physical_eye_order_confirmation':'User confirms eye0 physical LEFT, eye1 RIGHT on 2026-11-01','corner_detection_rejected':rejected,'corner_order_reversed':flipped,'hardware_accuracy_verified':False,'exposure_synchronized':False}
(OUT/'stereo-initial-fit.npz.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items()if k not in ['source_sha256','names']},ensure_ascii=False,indent=2))
