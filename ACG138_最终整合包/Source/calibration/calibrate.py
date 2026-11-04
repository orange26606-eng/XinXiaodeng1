#!/usr/bin/env python3
"""电脑端AVF1提取、实测相机标定及离线双目评估，不打开视频设备或UDP接收端，不发送电机命令，不猜测焦距。

"""
from __future__ import annotations
import argparse, glob, hashlib, json, math, struct, subprocess, sys, time
from pathlib import Path

# 本地测试依赖通过环境配置指定。
_deps = Path(__file__).resolve().parent / '.deps'
if _deps.is_dir(): sys.path.insert(0, str(_deps))
import cv2 as cv
import numpy as np

AVF = struct.Struct('<8I4Q')
MAGIC = 0x31465641

def write_json(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf8')

def image(path):
    # 某些OpenCV版本的imread不能读取Windows非ASCII路径。
    im = cv.imdecode(np.fromfile(str(path), dtype=np.uint8), cv.IMREAD_GRAYSCALE)
    if im is None: raise ValueError('Unreadable image: '+str(path))
    return im

def save_image(path, im):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    ok, b = cv.imencode(path.suffix or '.png', im)
    if not ok: raise ValueError('Image encode failed')
    b.tofile(str(path))

def parse_avf(data):
    if len(data)<AVF.size: raise ValueError('Truncated AVF1 header')
    magic, ver, hb, w, h, stride, payload, order, seq, stamp, r0, r1 = AVF.unpack_from(data)
    if (magic,ver,hb,w,h,stride,payload)!=(MAGIC,1,64,1920,1080,1920,2073600):
        raise ValueError('Unsupported AVF1 layout; expected 1920x1080 contiguous gray Y')
    if order not in (0,1,2) or r0 or r1 or not seq or not stamp:
        raise ValueError('Invalid AVF1 identity/time/reserved fields')
    if len(data)!=hb+payload: raise ValueError('Truncated or trailing AVF1 payload')
    gray=np.frombuffer(data,dtype=np.uint8,offset=hb).reshape(h,stride)[:,:w].copy()
    return gray,dict(sequence=seq,frame_time_us=stamp,time_source='RK_DEQUEUE',eye_order=order,
                     exposure_synchronized=False,width_per_eye=960,height=1080)

def extract_bytes(data, out):
    gray, meta=parse_avf(data); out=Path(out); out.mkdir(parents=True,exist_ok=True)
    stem=f'{meta["sequence"]:012d}'
    meta_file=out/'metadata'/(stem+'.json')
    if meta_file.exists():
        old=json.loads(meta_file.read_text(encoding='utf8'))
        if old.get('frame_time_us')!=meta['frame_time_us']:
            raise ValueError('Frame sequence reused across sessions; choose a new dataset output folder')
    for i in (0,1): save_image(out/f'eye{i}'/(stem+'.png'),gray[:,i*960:(i+1)*960])
    write_json(meta_file,meta)
    return meta

def extract(args):
    files=sorted(glob.glob(args.input))
    if not files: raise ValueError('No AVF1 files matched')
    seen=set()
    for p in files:
        b=Path(p).read_bytes(); _,m=parse_avf(b)
        if m['sequence'] in seen: raise ValueError('Duplicate frame sequence in input')
        seen.add(m['sequence']);extract_bytes(b,args.output)
    print(f'Extracted {len(files)} paired frames; eye index is image order, not verified physical side.')

def capture(args):
    """原子读取本地文件，或每个样本执行一次显式只读SSH获取。"""
    if args.count<1 or args.count>200 or not .2<=args.interval<=60: raise ValueError('Invalid capture limits')
    seen=set();start=time.monotonic()
    while len(seen)<args.count and time.monotonic()-start<args.timeout:
        if args.ssh:
            if not args.source.startswith('/') or any(c not in '/._-abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789' for c in args.source):
                raise ValueError('SSH source must be an absolute simple path')
            # 电脑端不经shell，不内置或保存密码，使用正常SSH认证。
            r=subprocess.run(['ssh',args.ssh,'cat',args.source],capture_output=True,timeout=30,check=True)
            data=r.stdout
        else: data=Path(args.source).read_bytes()
        _,m=parse_avf(data)
        if m['sequence'] not in seen:
            extract_bytes(data,args.output);seen.add(m['sequence']);print(f'Captured {len(seen)}/{args.count} sequence={m["sequence"]}',flush=True)
        if len(seen)<args.count: time.sleep(args.interval)
    if len(seen)<args.count: raise ValueError('Capture timeout; repeated file reads are not new samples')

def object_points(cols,rows,square):
    if cols<3 or rows<3 or cols>30 or rows>30 or not .001<=square<=.2: raise ValueError('Invalid checkerboard dimensions')
    p=np.zeros((cols*rows,3),np.float32);p[:,:2]=np.mgrid[0:cols,0:rows].T.reshape(-1,2)*square
    return p

def find_board(im,cols,rows):
    ok,c=cv.findChessboardCornersSB(im,(cols,rows),cv.CALIB_CB_NORMALIZE_IMAGE|cv.CALIB_CB_EXHAUSTIVE)
    return c.astype(np.float32) if ok else None

def files_for(pattern):
    p=sorted(Path(x) for x in glob.glob(pattern))
    if not p: raise ValueError('No images matched '+pattern)
    if len(p)>1000: raise ValueError('At most 1000 source images per calibration')
    return p

def board_views(files,cols,rows):
    points=[];names=[];rejected=[];size=None
    for p in files:
        im=image(p); s=(im.shape[1],im.shape[0])
        if size is None:size=s
        if s!=size:raise ValueError('Mixed input resolutions')
        c=find_board(im,cols,rows)
        if c is None:rejected.append(str(p));continue
        points.append(c);names.append(str(p))
    return points,names,rejected,size

def validate_camera(K,D,size,rms,max_rms):
    if not np.isfinite(K).all() or not np.isfinite(D).all() or not math.isfinite(rms):raise ValueError('Non-finite calibration')
    w,h=size
    if K[0,0]<=0 or K[1,1]<=0 or not (0<=K[0,2]<w and 0<=K[1,2]<h):raise ValueError('Implausible intrinsics')
    if rms>max_rms:raise ValueError(f'Calibration RMS {rms:.4f}px exceeds acceptance {max_rms}px; inspect samples')

def export_config(path,K,D,size,eye):
    if tuple(size)!=(960,1080):raise ValueError('Runtime config export requires exact per-eye 960x1080 calibration')
    d=np.zeros(5);d[:min(5,D.size)]=D.ravel()[:5]
    s=['# Measured intrinsics only. Verify camera pose before allowing navigation.',
       'calibration_enabled=1',f'eye={eye}','image_width=960','image_height=1080','tag_size_m=0.150',
       f'fx={K[0,0]:.12g}',f'fy={K[1,1]:.12g}',f'cx={K[0,2]:.12g}',f'cy={K[1,2]:.12g}']
    s += [f'{k}={v:.12g}' for k,v in zip(['k1','k2','p1','p2','k3'],d)]
    s += ['frame_max_age_ms=500','camera_to_body_verified=0','camera_x_m=0','camera_y_m=0','camera_z_m=0',
          'camera_roll_deg=0','camera_pitch_deg=0','camera_yaw_deg=0',
          '# Zero RPY optical->body mapping is (z,-x,-y); verify measured transform first.']
    Path(path).parent.mkdir(parents=True,exist_ok=True);Path(path).write_text('\n'.join(s)+'\n',encoding='utf8')

def mono(args):
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    obj=object_points(args.cols,args.rows,args.square_m)
    pts,names,rejected,size=board_views(files_for(args.images),args.cols,args.rows)
    if len(pts)<10:raise ValueError(f'Need at least 10 usable varied views, found {len(pts)}')
    rms,K,D,rvecs,tvecs=cv.calibrateCamera([obj]*len(pts),pts,size,None,None)
    validate_camera(K,D,size,rms,args.max_rms)
    per=[]
    for p,r,t in zip(pts,rvecs,tvecs):
        proj,_=cv.projectPoints(obj,r,t,K,D);per.append(float(np.sqrt(np.mean(np.sum((p-proj)**2,axis=2)))))
    np.savez_compressed(args.output,K=K,D=D,image_size=np.array(size),eye=args.eye,rms=rms)
    write_json(str(args.output)+'.json',dict(kind='measured_mono_calibration',rms_px=rms,per_view_rms_px=per,
        image_size=size,eye=args.eye,cols=args.cols,rows=args.rows,square_m=args.square_m,
        K=K.tolist(),D=D.ravel().tolist(),accepted=names,rejected=rejected,
        input_sha256={n:hashlib.sha256(Path(n).read_bytes()).hexdigest() for n in names},
        board_geometry_verified_by_user=False,camera_to_body_verified=False,hardware_accuracy_verified=False))
    if args.config:export_config(args.config,K,D,size,args.eye)
    print(f'MONO accepted={len(pts)} rejected={len(rejected)} RMS={rms:.4f}px. Independent ruler validation still required.')

def load_mono(path):
    with np.load(path,allow_pickle=False) as z:
        return z['K'].copy(),z['D'].copy(),tuple(int(v) for v in z['image_size'])

def paired_paths(a,b):
    aa={p.stem:p for p in files_for(a)};bb={p.stem:p for p in files_for(b)}
    if aa.keys()!=bb.keys():raise ValueError('Paired image stems differ; never silently pair by sorted index')
    return [(aa[k],bb[k]) for k in sorted(aa)]

def stereo(args):
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    K1,D1,size=load_mono(args.left_calib);K2,D2,size2=load_mono(args.right_calib)
    if size!=size2:raise ValueError('Left/right calibration resolution differs')
    obj=object_points(args.cols,args.rows,args.square_m);p1=[];p2=[];names=[];reject=[]
    for left,right in paired_paths(args.left,args.right):
        l,r=image(left),image(right)
        if (l.shape[1],l.shape[0])!=size or r.shape!=l.shape:raise ValueError('Stereo calibration image size mismatch')
        a,b=find_board(l,args.cols,args.rows),find_board(r,args.cols,args.rows)
        if a is None or b is None:reject.append([str(left),str(right)]);continue
        p1.append(a);p2.append(b);names.append([str(left),str(right)])
    if len(p1)<10:raise ValueError('Need at least 10 valid paired checkerboard views')
    rms,K1,D1,K2,D2,R,T,E,F=cv.stereoCalibrate([obj]*len(p1),p1,p2,K1,D1,K2,D2,size,
        criteria=(cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,100,1e-7),flags=cv.CALIB_FIX_INTRINSIC)
    if not math.isfinite(rms) or rms>args.max_rms:raise ValueError(f'Stereo RMS too high: {rms}')
    baseline=float(np.linalg.norm(T))
    if not .005<baseline<1.0:raise ValueError('Baseline outside 5mm..1m; check measured square units and pairing')
    R1,R2,P1,P2,Q,roi1,roi2=cv.stereoRectify(K1,D1,K2,D2,size,R,T,flags=cv.CALIB_ZERO_DISPARITY,alpha=0)
    # 仅支持水平双目，需核对物理左右镜头。
    if abs(P2[1,3])>abs(P2[0,3]) or P2[0,3]>=0:raise ValueError('Expected physical LEFT then RIGHT horizontal pair; verify eye mapping')
    row=[]
    for a,b in zip(p1,p2):
        ar=cv.undistortPoints(a,K1,D1,R=R1,P=P1);br=cv.undistortPoints(b,K2,D2,R=R2,P=P2)
        row.extend(np.abs(ar[:,0,1]-br[:,0,1]).tolist())
    np.savez_compressed(args.output,K1=K1,D1=D1,K2=K2,D2=D2,R=R,T=T,R1=R1,R2=R2,P1=P1,P2=P2,Q=Q,
                        roi1=np.array(roi1),roi2=np.array(roi2),image_size=np.array(size),baseline_m=baseline,rms=rms)
    write_json(str(args.output)+'.json',dict(kind='measured_stereo_calibration',rms_px=rms,baseline_m=baseline,
        rectified_vertical_error_median_px=float(np.median(row)),rectified_vertical_error_p95_px=float(np.percentile(row,95)),
        image_size=size,accepted=names,rejected=reject,exposure_synchronized=False,
        hardware_accuracy_verified=False,warning='A joined frame does not prove simultaneous exposures.'))
    print(f'STEREO pairs={len(p1)} RMS={rms:.4f}px baseline={baseline:.5f}m rowP95={np.percentile(row,95):.3f}px')

def depth_arrays(left,right,z,min_m=.15,max_m=5.,num_disp=128):
    size=tuple(int(v) for v in z['image_size'])
    if left.shape!=right.shape or (left.shape[1],left.shape[0])!=size:raise ValueError('Depth input/calibration resolution mismatch')
    if num_disp<16 or num_disp%16 or num_disp>=size[0]-16:raise ValueError('num-disparities must be multiple16 and less than width-16')
    if not 0<min_m<max_m<=100:raise ValueError('Invalid metric range')
    def rect(im,K,D,R,P):
        mx,my=cv.initUndistortRectifyMap(K,D,R,P,size,cv.CV_32FC1)
        return cv.remap(im,mx,my,cv.INTER_LINEAR)
    a=rect(left,z['K1'],z['D1'],z['R1'],z['P1']);b=rect(right,z['K2'],z['D2'],z['R2'],z['P2'])
    def matcher(mind):return cv.StereoSGBM_create(minDisparity=mind,numDisparities=num_disp,blockSize=5,
        P1=8*25,P2=32*25,disp12MaxDiff=1,uniquenessRatio=10,speckleWindowSize=100,speckleRange=2)
    dl=matcher(0).compute(a,b).astype(np.float32)/16.;dr=matcher(-num_disp).compute(b,a).astype(np.float32)/16.
    xyz=cv.reprojectImageTo3D(dl,z['Q']);depth=xyz[:,:,2]
    yy,xx=np.indices(dl.shape);xr=np.rint(xx-dl).astype(np.int32);inside=(xr>=0)&(xr<size[0]);xr=np.clip(xr,0,size[0]-1)
    consistent=np.abs(dl+dr[yy,xr])<=1.5
    mask=inside&consistent&(dl>0)&np.isfinite(depth)&(depth>=min_m)&(depth<=max_m)
    roi=np.zeros_like(mask)
    x,y,w,h=(int(v) for v in z['roi1']);roi[y:y+h,x:x+w]=True;mask &=roi
    x,y,w,h=(int(v) for v in z['roi2']);mask &= (xr>=x)&(xr<x+w)&(yy>=y)&(yy<y+h)
    depth=np.where(mask,depth,np.nan).astype(np.float32)
    return depth,mask,dl,a,b

def depth(args):
    if args.reference_m is not None and (not math.isfinite(args.reference_m) or args.reference_m<=0):raise ValueError('reference-m must be positive')
    left,right=image(args.left),image(args.right)
    t=time.perf_counter()
    with np.load(args.calibration,allow_pickle=False) as file: z={key:file[key].copy() for key in file.files}
    if (left.shape[1],left.shape[0])!=tuple(z['image_size']) or right.shape!=left.shape:raise ValueError('Input/calibration size mismatch before resizing')
    if not .25<=args.scale<=1:raise ValueError('scale must be .25..1')
    if args.scale!=1:
        old=tuple(int(v) for v in z['image_size']);size=tuple(int(round(v*args.scale)) for v in old)
        sx,sy=size[0]/old[0],size[1]/old[1]
        for key in ['K1','K2']:
            z[key][0,:]*=sx;z[key][1,:]*=sy
        result=cv.stereoRectify(z['K1'],z['D1'],z['K2'],z['D2'],size,z['R'],z['T'],flags=cv.CALIB_ZERO_DISPARITY,alpha=0)
        for key,value in zip(['R1','R2','P1','P2','Q','roi1','roi2'],result):z[key]=value
        z['image_size']=size;left=cv.resize(left,size,interpolation=cv.INTER_AREA);right=cv.resize(right,size,interpolation=cv.INTER_AREA)
    d,mask,disp,a,b=depth_arrays(left,right,z,args.min_m,args.max_m,args.num_disparities)
    elapsed=time.perf_counter()-t;out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(str(out)+'.npz',depth_m=d,valid=mask,disparity_px=disp)
    v=np.zeros(d.shape,np.uint8);v[mask]=np.clip(255*(1-d[mask]/args.max_m),0,255).astype(np.uint8)
    color=cv.applyColorMap(v,cv.COLORMAP_TURBO);color[~mask]=0;save_image(str(out)+'-depth.png',color)
    joined=np.hstack([a,b]);joined=cv.cvtColor(joined,cv.COLOR_GRAY2BGR)
    for y in range(0,joined.shape[0],40):cv.line(joined,(0,y),(joined.shape[1]-1,y),(0,255,0),1)
    save_image(str(out)+'-rectified.png',joined)
    stats=dict(valid_fraction=float(mask.mean()),processing_s=elapsed,valid_pixels=int(mask.sum()),
        depth_source='measured_stereo_Q',invalid_is_unknown=True,hardware_accuracy_verified=False,scale=args.scale,
        image_size=[int(v) for v in z['image_size']],time_source='RK_DEQUEUE',exposure_synchronized=False)
    if args.roi:
        x,y,w,h=args.roi
        if x<0 or y<0 or w<1 or h<1 or x+w>d.shape[1] or y+h>d.shape[0]:raise ValueError('ROI outside image')
        vals=d[y:y+h,x:x+w];vals=vals[np.isfinite(vals)];stats['roi_valid_fraction']=float(len(vals)/(w*h))
    else:vals=d[mask]
    if len(vals):
        med=float(np.median(vals));stats.update(median_depth_m=med,depth_mad_m=float(np.median(np.abs(vals-med))))
        if args.reference_m:
            stats.update(reference_m=args.reference_m,absolute_error_m=abs(med-args.reference_m),relative_error=abs(med-args.reference_m)/args.reference_m)
    write_json(str(out)+'.json',stats);print(json.dumps(stats,ensure_ascii=False))

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    x=sub.add_parser('extract');x.add_argument('--input',required=True);x.add_argument('--output',required=True);x.set_defaults(func=extract)
    x=sub.add_parser('capture');x.add_argument('--source',required=True);x.add_argument('--ssh');x.add_argument('--output',required=True)
    x.add_argument('--count',type=int,default=1);x.add_argument('--interval',type=float,default=1);x.add_argument('--timeout',type=float,default=180);x.set_defaults(func=capture)
    def board(x):
        x.add_argument('--cols',type=int,default=9);x.add_argument('--rows',type=int,default=6);x.add_argument('--square-m',type=float,default=.025)
        x.add_argument('--max-rms',type=float,default=2.0);x.add_argument('--output',required=True)
    x=sub.add_parser('mono');board(x);x.add_argument('--images',required=True);x.add_argument('--eye',type=int,choices=(0,1),required=True);x.add_argument('--config');x.set_defaults(func=mono)
    x=sub.add_parser('stereo');board(x)
    for name in ['left','right','left-calib','right-calib']:x.add_argument('--'+name,required=True)
    x.set_defaults(func=stereo)
    x=sub.add_parser('depth')
    for name in ['left','right','calibration','output']:x.add_argument('--'+name,required=True)
    x.add_argument('--num-disparities',type=int,default=128);x.add_argument('--min-m',type=float,default=.15);x.add_argument('--max-m',type=float,default=5)
    x.add_argument('--scale',type=float,default=.5,help='Rescale both images and recompute rectification/Q; default per-eye480x540')
    x.add_argument('--roi',type=int,nargs=4);x.add_argument('--reference-m',type=float);x.set_defaults(func=depth)
    args=p.parse_args()
    try:args.func(args)
    except (ValueError,OSError,cv.error,subprocess.SubprocessError) as e:p.exit(2,'ERROR: '+str(e)+'\n')

if __name__=='__main__':main()
