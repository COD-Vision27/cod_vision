import hashlib,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from overlay import generate
class OverlayTests(unittest.TestCase):
    def test_original_io_files_remain_unchanged(self):
        root=Path(__file__).resolve().parents[2]/'sp_vision_25'
        originals={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in [root/'io/gimbal/gimbal.hpp',root/'io/gimbal/gimbal.cpp']}
        with tempfile.TemporaryDirectory() as folder:
            generate(root,folder)
            self.assertIn('latest_pose() const',(Path(folder)/'io/gimbal/gimbal.hpp').read_text())
            self.assertIn('latest_pose_time_ = t',(Path(folder)/'io/gimbal/gimbal.cpp').read_text())
        self.assertEqual(originals,{p:hashlib.sha256(p.read_bytes()).hexdigest() for p in originals})
if __name__=='__main__':unittest.main()
