#!/usr/bin/env python3
"""单姿态采集并立即检查棋盘。板端相机程序运行后，从工程根目录执行python calibration/reshoot_check.py。每种姿态保持约2秒，工具获取一帧并按capture方式保存，同时报告两眼角点是否检出、倾斜角和模糊程度。






"""
import argparse, glob, math, ntpath, subprocess, sys
import numpy as np
import cv2 as cv

def newest(base, eye):
    files = glob.glob(base + f'/eye{eye}/*.png')
    if not files: return None
    return max(files, key=lambda p: ntpath.basename(p))

def load(p):
    return cv.imdecode(np.fromfile(p, dtype=np.uint8), cv.IMREAD_GRAYSCALE)

def check(path, K, obj):
    im = load(path)
    if im is None: return 'UNREADABLE', None, 0.0
    blur = cv.Laplacian(im, cv.CV_64F).var()
    ok, c = cv.findChessboardCornersSB(im, (9, 6), cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_EXHAUSTIVE)
    clahe = False
    if not ok:
        cla = cv.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        ok, c = cv.findChessboardCornersSB(cla.apply(im), (9, 6), cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_EXHAUSTIVE)
        clahe = bool(ok)
    if not ok:
        # 传统角点算法可补充SB算法漏检的低对比度或较软图像。
        ok, c = cv.findChessboardCorners(im, (9, 6), cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FILTER_QUADS)
        if ok and len(c) != 9 * 6: ok = False
    if not ok:
        ok, c = cv.findChessboardCorners(cv.addWeighted(im, 2.0, cv.GaussianBlur(im, (0, 0), 1.5), -1.0, 0), (9, 6), cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_FILTER_QUADS)
        if ok and len(c) != 9 * 6: ok = False
    if not ok:
        ok, c = cv.findChessboardCornersSB(cv.rotate(im, cv.ROTATE_90_CLOCKWISE), (9, 6), cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_EXHAUSTIVE)
        if ok: return 'PORTRAIT', None, blur
    if not ok:
        ok, c = cv.findChessboardCornersSB(cv.rotate(im, cv.ROTATE_180), (9, 6), cv.CALIB_CB_NORMALIZE_IMAGE | cv.CALIB_CB_EXHAUSTIVE)
        if ok: return 'UPSIDE-DOWN', None, blur
    if not ok: return 'NOT-FOUND', None, blur
    ret, rv, tv = cv.solvePnP(obj, c, K, None, flags=cv.SOLVEPNP_ITERATIVE)
    if not ret: return 'SOLVE-FAIL', None, blur
    R, _ = cv.Rodrigues(rv)
    tilt = math.degrees(math.acos(min(1.0, abs(R[2, 2]))))  # 实际倾角为棋盘法向与光轴的夹角。
    if clahe: return 'OK-LOW-CONTRAST', tilt, blur
    x, y, w, h = cv.boundingRect(c.astype(np.int32))
    return 'OK', tilt, blur

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ssh', default='root@192.168.232.116')
    ap.add_argument('--source', default='/run/acg138-object-memory-nav-sensors/vision-frame.bin')
    ap.add_argument('--output', default='calibration/data_20261031_reshoot')
    args = ap.parse_args()
    r = subprocess.run([sys.executable, 'calibration/calibrate.py', 'capture',
                        '--ssh', args.ssh, '--source', args.source,
                        '--output', args.output, '--count', '1', '--timeout', '20'],
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    if r.stdout.strip(): print(r.stdout.strip())
    if r.stderr.strip(): print(r.stderr.strip())
    if r.returncode != 0:
        print('CAPTURE FAILED - check that the board-side camera program is running (vision-frame.bin updating).')
        sys.exit(r.returncode)
    obj = np.zeros((9 * 6, 3), np.float32)
    obj[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.025
    K = np.array([[2400, 0, 480], [0, 2400, 540], [0, 0, 1]], np.float64)
    f = ntpath.basename(newest(args.output, 0))
    results = {}
    for eye in (0, 1):
        status, tilt, blur = check(f'{args.output}/eye{eye}/{f}', K, obj)
        results[eye] = (status, tilt, blur)
    print(f'[{f}]')
    for eye in (0, 1):
        status, tilt, blur = results[eye]
        if status in ('OK', 'OK-LOW-CONTRAST'):
            note = ' (low contrast, will auto-enhance)' if status == 'OK-LOW-CONTRAST' else ''
            if tilt < 10: verdict = 'frontal anchor, keep (only a few needed)'
            elif tilt <= 45: verdict = 'good tilt'
            elif tilt <= 55: verdict = 'strong but usable'
            else: verdict = 'TOO TILTED, reduce angle'
            print(f'  eye{eye}: OK  tilt={tilt:.0f}deg -> {verdict}{note}')
        elif status == 'PORTRAIT':
            print(f'  eye{eye}: PORTRAIT - board is rotated 90deg (tall). Rotate it back; in-plane roll of ~30deg is enough.')
        elif status == 'UPSIDE-DOWN':
            print(f'  eye{eye}: UPSIDE-DOWN - board is flipped 180deg (upside down). Flip it back and retake this pose.')
        else:
            print(f'  eye{eye}: {status} -> NO CHECKERBOARD in this image (the tool only looks for the black-white checkerboard, not general objects). Keep the board centered in the preview, then capture.')
    both_ok = all(results[e][0] in ('OK', 'OK-LOW-CONTRAST') for e in (0, 1))
    tilts = [results[e][1] or 0 for e in (0, 1)]
    if both_ok and all(t <= 55 for t in tilts):
        print('PASS - keep this one, next pose.')
    else:
        print('RETAKE THIS POSE.')

if __name__ == '__main__':
    main()
