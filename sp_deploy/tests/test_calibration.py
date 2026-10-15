#!/usr/bin/env python3
"""Independent known geometry, held-out PnP, degeneracy and deployment gates."""
import argparse,copy,json,math,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import cv2 as cv
import numpy as np
import calibration as cal
import deploy
BOARD={'type':'chessboard','cols':9,'rows':6,'spacing_m':.035}
K=np.array([[850.,0,640],[0,860,480],[0,0,1]])
D=np.zeros(5)

def q(R):
    rv=cv.Rodrigues(R)[0].ravel();a=np.linalg.norm(rv)
    return np.r_[math.cos(a/2),rv/a*math.sin(a/2)].tolist() if a else [1,0,0,0]

def fixtures(root):
    root=Path(root);intr=root/'intrinsics';hand=root/'handeye';intr.mkdir(parents=True);hand.mkdir()
    rng=np.random.default_rng(947);obj=cal.board_points(BOARD);samples=[]
    for i in range(35):
        rv=rng.uniform(-.35,.35,3);rv[2]*=.4;t=np.array([rng.uniform(-.55,.28),rng.uniform(-.38,.18),rng.uniform(1.0,1.3)])
        points=cv.projectPoints(obj,rv,t,K,D)[0].reshape(-1,2)
        points+=rng.normal(0,.03,points.shape)
        samples.append({'corners_px':points.tolist()})
    meta={'schema':'sp_capture_v1','kind':'synthetic','image_size':[1280,960],'R_gimbal2imubody':np.eye(3).flatten().tolist(),'samples':samples}
    cal.dump(intr/'dataset.json',meta)
    X=cal.transform([[0,0,1],[-1,0,0],[0,-1,0]],[.085,.013,.046])
    Y=cal.transform(X[:3,:3],[1.4,.14,.11]);S=cv.Rodrigues(np.array([.4,.1,-.2]))[0];samples=[]
    for i in range(30):
        G=cal.transform(cv.Rodrigues(np.array([0,rng.uniform(-.20,.20),rng.uniform(-.30,.30)]))[0],[0,0,0])
        B=np.linalg.inv(X)@np.linalg.inv(G)@Y
        points=cv.projectPoints(obj,cv.Rodrigues(B[:3,:3])[0],B[:3,3],K,D)[0].reshape(-1,2)
        points+=rng.normal(0,.015,points.shape)
        samples.append({'corners_px':points.tolist(),'quaternion_wxyz':q(S@G[:3,:3]@S.T)})
    meta.update({'R_gimbal2imubody':S.flatten().tolist(),'samples':samples});cal.dump(hand/'dataset.json',meta)
    return intr,hand,X,S

class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.root=Path(cls.tmp.name)
        cls.intr,cls.hand,cls.X,cls.S=fixtures(cls.root)
        cls.i=cal.calibrate_camera(cls.intr,BOARD)
        cls.h=cal.calibrate_handeye(cls.hand,BOARD,cls.i,True)
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_intrinsics_recover_known_camera_and_holdout(self):
        self.assertEqual(self.i['status'],'PASS');recovered=np.array(self.i['config']['camera_matrix']).reshape(3,3)
        self.assertLess(np.max(np.abs(recovered-K)),1.0);self.assertGreaterEqual(self.i['holdout_count'],4)
        self.assertLess(self.i['holdout_p95_px'],.2)
    def test_handeye_metres_direction_and_nonidentity_imu_basis(self):
        self.assertEqual(self.h['status'],'PASS');R=np.array(self.h['config']['R_camera2gimbal']).reshape(3,3)
        self.assertLess(cal.angle(R.T@self.X[:3,:3]),.2)
        self.assertLess(np.linalg.norm(np.array(self.h['config']['t_camera2gimbal'])-self.X[:3,3]),.003)
    def test_rotation_only_requires_explicit_fixed_center(self):
        with self.assertRaisesRegex(ValueError,'translation is missing'):cal.calibrate_handeye(self.hand,BOARD,self.i)
    def test_single_axis_rotation_rejected(self):
        X=self.X;Y=cal.transform(X[:3,:3],[1.4,.14,.11]);gs=[cal.transform(cv.Rodrigues(np.array([0.,0.,a]))[0],[0,0,0]) for a in np.linspace(-.4,.4,15)]
        bs=[np.linalg.inv(X)@np.linalg.inv(g)@Y for g in gs]
        with self.assertRaisesRegex(ValueError,'degenerate'):cal.handeye_from_poses(gs,bs,[0,5,10])
    def test_bad_quaternion_and_reflection_rejected(self):
        with self.assertRaises(ValueError):cal.quaternion([0,0,0,0])
        with self.assertRaises(ValueError):cal.rotation(np.diag([1,1,-1]))
    def test_board_motion_fails_residual(self):
        folder=self.root/'moving-board';folder.mkdir(exist_ok=True);meta=json.loads((self.hand/'dataset.json').read_text())
        for index,sample in enumerate(meta['samples']):
            if index%3==0:sample['corners_px']=(np.array(sample['corners_px'])+np.array([70,0])).tolist()
        cal.dump(folder/'dataset.json',meta);report=cal.calibrate_handeye(folder,BOARD,self.i,True)
        self.assertEqual(report['status'],'FAIL')
    def test_real_corner_detector_on_rendered_checkerboard(self):
        image=np.full((960,1280),160,np.uint8);obj=cal.board_points(BOARD);rv=np.array([.1,-.15,.03]);t=np.array([-.16,-.1,1.0])
        for row in range(-1,BOARD['rows']):
            for col in range(-1,BOARD['cols']):
                square=np.array([[col,row,0],[col+1,row,0],[col+1,row+1,0],[col,row+1,0]],np.float32)*BOARD['spacing_m']
                pts=cv.projectPoints(square,rv,t,K,D)[0].reshape(-1,2).astype(np.int32)
                cv.fillConvexPoly(image,pts,0 if (col+row)%2==0 else 255)
        folder=self.root/'rendered';folder.mkdir(exist_ok=True);cv.imwrite(str(folder/'1.png'),image)
        cal.dump(folder/'dataset.json',{'schema':'sp_capture_v1','kind':'synthetic','image_size':[1280,960],'samples':[{'image':'1.png'}]})
        _,_,_,views,rejected,_=cal.dataset(folder,BOARD)
        self.assertEqual(len(views),1);self.assertFalse(rejected)
    def test_synthetic_reports_cannot_activate_hardware(self):
        p=deploy.profile(Path(__file__).resolve().parents[1]/'profiles/hikrobot-uart44.example.json');p['R_gimbal2imubody']=self.S.flatten().tolist()
        with self.assertRaisesRegex(ValueError,'synthetic'):deploy.validate_reports(self.i,self.h,p,hardware=True)
    def test_handeye_intrinsics_provenance_cannot_be_mixed(self):
        p=deploy.profile(Path(__file__).resolve().parents[1]/'profiles/hikrobot-uart44.example.json');p['R_gimbal2imubody']=self.S.flatten().tolist();wrong=copy.deepcopy(self.i);wrong['config']['camera_matrix'][0]+=1
        with self.assertRaisesRegex(ValueError,'different intrinsics'):deploy.validate_reports(wrong,self.h,p)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--export',type=Path);args,remaining=parser.parse_known_args()
    if args.export:
        args.export.mkdir(parents=True,exist_ok=True);i,h,X,S=fixtures(args.export);intr=cal.calibrate_camera(i,BOARD);hand=cal.calibrate_handeye(h,BOARD,intr,True)
        p=deploy.profile(Path(__file__).resolve().parents[1]/'profiles/hikrobot-uart44.example.json');p['board']=BOARD;p['R_gimbal2imubody']=S.flatten().tolist()
        cal.dump(args.export/'intrinsics.json',intr);cal.dump(args.export/'handeye.json',hand);cal.dump(args.export/'profile.json',p)
    unittest.main(argv=[sys.argv[0],*remaining])
