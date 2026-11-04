"""用几何生成的双目棋盘视图测试完整命令行流程，不访问硬件。"""
import importlib,math,subprocess,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import calibrate as c
cv,np=c.cv,c.np

class Tests(unittest.TestCase):
    def test_cli_calibration_and_depth(self):
        root=ROOT/'qa/synthetic-cli';root.mkdir(parents=True,exist_ok=True)
        K=np.array([[850.,0,480],[0,860,540],[0,0,1.]])
        source=np.full((420,600),255,np.uint8)
        for y in range(7):
            for x in range(10):
                if (x+y)%2==0:source[y*60:(y+1)*60,x*60:(x+1)*60]=0
        # 物体坐标原点位于首个内角点，与标定约定一致。
        outer=np.array([[-.025,-.025,0],[.225,-.025,0],[.225,.15,0],[-.025,.15,0]],np.float32)
        src=np.array([[0,0],[600,0],[600,420],[0,420]],np.float32)
        for i in range(14):
            rv=np.array([.24*math.sin(i*.9),.28*math.cos(i*.6),.12*math.sin(i*.4)])
            tv=np.array([-.1+.03*(i%4),-.055+.025*(i%3),.52+.035*(i%5)])
            for eye in (0,1):
                projected=cv.projectPoints(outer,rv,tv+np.array([-.06*eye,0,0]),K,np.zeros(5))[0].reshape(4,2)
                H=cv.getPerspectiveTransform(src,projected.astype(np.float32))
                im=cv.warpPerspective(source,H,(960,1080),borderValue=255)
                c.save_image(root/f'eye{eye}/{i:03d}.png',im)
        def run(*args):
            r=subprocess.run([sys.executable,str(ROOT/'calibrate.py'),*map(str,args)],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stdout+r.stderr);return r.stdout
        for eye in (0,1):
            args=['mono','--images',root/f'eye{eye}/*.png','--eye',eye,'--output',root/f'eye{eye}.npz']
            if eye==0:args+=['--config',root/'intrinsics.conf']
            print(run(*args))
        print(run('stereo','--left',root/'eye0/*.png','--right',root/'eye1/*.png',
                  '--left-calib',root/'eye0.npz','--right-calib',root/'eye1.npz','--output',root/'stereo.npz'))
        print(run('depth','--left',root/'eye0/000.png','--right',root/'eye1/000.png',
                  '--calibration',root/'stereo.npz','--output',root/'depth','--scale',.5,'--num-disparities',64))
        with np.load(root/'stereo.npz',allow_pickle=False) as z:self.assertAlmostEqual(float(z['baseline_m']),.06,delta=.003)
        self.assertTrue((root/'depth-depth.png').is_file())

if __name__=='__main__':unittest.main(verbosity=2)
