"""按物理尺寸核对PDF绘图路径，并检查渲染后的图案识别。"""
from pathlib import Path
import sys,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'.deps'))
import calibrate as c
from pypdf import PdfReader
from pypdf.generic import ContentStream

class Tests(unittest.TestCase):
    def test_physical_rectangles(self):
        for stem,cell,extent,page in [
          ('tag36h11_id0_blackedge150mm_A4',18.75,(150,150),(210,297)),
          ('tag36h11_id1_blackedge150mm_A4',18.75,(150,150),(210,297)),
          ('checkerboard_9x6_inner_25mm_A4_landscape',25,(250,175),(297,210))]:
            r=PdfReader(ROOT/'printables'/(stem+'.pdf'));self.assertEqual(len(r.pages),1);p=r.pages[0]
            self.assertAlmostEqual(float(p.mediabox.width)*25.4/72,page[0],places=3)
            self.assertAlmostEqual(float(p.mediabox.height)*25.4/72,page[1],places=3)
            paths=[]
            for values,op in ContentStream(p.get_contents(),r).operations:
                if op!=b're':continue
                x,y,w,h=[float(v)*25.4/72 for v in values]
                if abs(w-page[0])<.001 and abs(h-page[1])<.001:continue
                self.assertAlmostEqual(w,cell,places=3);self.assertAlmostEqual(h,cell,places=3);paths.append((x,y,w,h))
            self.assertGreater(len(paths),20)
            self.assertAlmostEqual(max(x+w for x,y,w,h in paths)-min(x for x,y,w,h in paths),extent[0],places=3)
            self.assertAlmostEqual(max(y+h for x,y,w,h in paths)-min(y for x,y,w,h in paths),extent[1],places=3)

    def test_rendered_printed_pattern_identity(self):
        cv=c.cv
        for i in (0,1):
            png=ROOT/'qa'/f'tag36h11_id{i}_blackedge150mm_A4.png'
            self.assertTrue(png.exists(),'Render PDFs with pdftoppm before visual QA')
            im=c.image(png)
            ids=cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)).detectMarkers(im)[1]
            self.assertIsNotNone(ids);self.assertEqual(ids.ravel().tolist(),[i])
        im=c.image(ROOT/'qa/checkerboard_9x6_inner_25mm_A4_landscape.png')
        self.assertIsNotNone(c.find_board(im,9,6))

if __name__=='__main__':unittest.main(verbosity=2)
