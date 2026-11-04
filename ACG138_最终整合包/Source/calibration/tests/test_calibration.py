import argparse, importlib.util, json, math, struct, sys, tempfile, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import calibrate as c
import make_printables as p
cv,np=c.cv,c.np

class Tests(unittest.TestCase):
    def test_avf1_and_rejection(self):
        raw=np.zeros((1080,1920),np.uint8);raw[:,960:]=200
        header=c.AVF.pack(c.MAGIC,1,64,1920,1080,1920,2073600,0,42,1000000,0,0)
        im,m=c.parse_avf(header+raw.tobytes());self.assertEqual(m['sequence'],42);self.assertEqual(int(im[0,960]),200)
        self.assertEqual(m['time_source'],'RK_DEQUEUE');self.assertFalse(m['exposure_synchronized'])
        with self.assertRaises(ValueError):c.parse_avf(header+raw.tobytes()[:-1])
        with self.assertRaises(ValueError):c.parse_avf(b'BADD'+header[4:]+raw.tobytes())

    def test_checkerboard_dimensions(self):
        self.assertEqual(10*25,250)
        self.assertEqual(7*25,175)
        self.assertEqual(9*6,54)

    def test_mono_metric_geometry_and_export(self):
        K=np.array([[800.,0,480],[0,810,540],[0,0,1]])
        D=np.array([-.08,.01,.001,-.001,0.])
        obj=c.object_points(9,6,.025);pts=[];objs=[]
        for i in range(20):
            r=np.array([.15*math.sin(i),.3*math.cos(i*.7),.1*math.sin(i*.4)])
            t=np.array([-.12+.025*(i%4),-.08+.025*(i%3),.55+.05*(i%6)])
            q,_=cv.projectPoints(obj,r,t,K,D);pts.append(q.astype(np.float32));objs.append(obj)
        rms,k,d,_,_=cv.calibrateCamera(objs,pts,(960,1080),None,None)
        self.assertLess(rms,.001);self.assertLess(abs(k[0,0]-800),.1)
        qa=ROOT/'qa';qa.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=qa) as tmp:
            path=Path(tmp)/'apriltag.conf';c.export_config(path,k,d,(960,1080),0)
            text=path.read_text();self.assertIn('calibration_enabled=1',text);self.assertIn('camera_to_body_verified=0',text)
            with self.assertRaises(ValueError):c.export_config(path,k,d,(1920,1080),0)

    def test_stereo_metric_depth_and_invalid_mask(self):
        w,h=640,360;f=400.;baseline=.06;distance=1.5;shift=int(f*baseline/distance)
        K=np.array([[f,0,w/2],[0,f,h/2],[0,0,1.]])
        D=np.zeros(5);R=np.eye(3);T=np.array([-baseline,0,0])
        R1,R2,P1,P2,Q,roi1,roi2=cv.stereoRectify(K,D,K,D,(w,h),R,T,flags=cv.CALIB_ZERO_DISPARITY,alpha=0)
        z=dict(K1=K,D1=D,K2=K,D2=D,R1=R1,R2=R2,P1=P1,P2=P2,Q=Q,roi1=roi1,roi2=roi2,image_size=[w,h])
        rng=np.random.default_rng(9250);left=cv.GaussianBlur(rng.integers(0,256,(h,w),dtype=np.uint8),(3,3),0)
        right=np.zeros_like(left);right[:,:-shift]=left[:,shift:]
        depth,mask,disp,_,_=c.depth_arrays(left,right,z,.15,5,64)
        self.assertGreater(mask.mean(),.5);self.assertAlmostEqual(float(np.nanmedian(depth)),distance,delta=.03)
        self.assertTrue(np.isnan(depth[~mask]).all())
        blank=np.full_like(left,127);_,m,_,_,_=c.depth_arrays(blank,blank,z,.15,5,64)
        self.assertLess(m.mean(),.05)

    def test_stereo_calibration_from_known_paired_points(self):
        K=np.array([[800.,0,480],[0,805,540],[0,0,1]])
        D=np.zeros(5);obj=c.object_points(9,6,.025);left=[];right=[]
        for i in range(16):
            r=np.array([.15*math.sin(i),.2*math.cos(i),.1*math.sin(i*.3)])
            t=np.array([-.12+.03*(i%4),-.07+.02*(i%3),.6+.05*(i%5)])
            left.append(cv.projectPoints(obj,r,t,K,D)[0].astype(np.float32))
            right.append(cv.projectPoints(obj,r,t+np.array([-.06,0,0]),K,D)[0].astype(np.float32))
        result=cv.stereoCalibrate([obj]*len(left),left,right,K,D,K.copy(),D.copy(),(960,1080),flags=cv.CALIB_FIX_INTRINSIC)
        self.assertLess(result[0],.001);self.assertAlmostEqual(float(np.linalg.norm(result[6])),.06,places=5)

if __name__=='__main__':unittest.main(verbosity=2)
