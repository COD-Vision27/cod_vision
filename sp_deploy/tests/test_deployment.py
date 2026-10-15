#!/usr/bin/env python3
"""No hardware: release integrity, activation gates and rollback semantics."""
import copy,json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_calibration import fixtures,BOARD
import calibration as cal
import deploy

class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        i,h,_,S=fixtures(self.root/'data');self.intr=cal.calibrate_camera(i,BOARD);self.hand=cal.calibrate_handeye(h,BOARD,self.intr,True)
        self.p=deploy.profile(Path(__file__).resolve().parents[1]/'profiles/hikrobot-uart44.example.json');self.p['R_gimbal2imubody']=S.flatten().tolist();self.p['muzzle_offset_gimbal_m']=[0,0,0]
        self.qual=self.root/'qualification.json';cal.dump(self.qual,{'schema':'sp_sim_accuracy_v1','status':'PASS','expected_cases':3,
            'cases':[{'base':'standard3','scenario':s,'status':'PASS','shots':200,'conservative_hit_rate':.95,'wilson_95_lower':.91} for s in ['static','spin','move']]})
    def tearDown(self):self.tmp.cleanup()
    def candidate(self,name,hardware=False):
        release=self.root/'prefix/releases'/name;release.mkdir(parents=True)
        intr=copy.deepcopy(self.intr);hand=copy.deepcopy(self.hand)
        if hardware:
            intr['kind']='hardware';hand['kind']='hardware'
            import hashlib
            hand['intrinsics_sha256']=hashlib.sha256(json.dumps(intr,sort_keys=True).encode()).hexdigest()
        # Mock report labels here only test filesystem transactions, not real calibration.
        cal.dump(release/'calibration/intrinsics.json',intr);cal.dump(release/'calibration/handeye.json',hand);cal.dump(release/'config/profile.json',self.p)
        cal.dump(release/'manifest.json',{'files':{str(f.relative_to(release)):cal.digest(f) for f in release.rglob('*') if f.is_file()}})
        return release
    def test_synthetic_candidate_cannot_replace_current(self):
        release=self.candidate('synthetic')
        with self.assertRaisesRegex(ValueError,'synthetic'):deploy.activate(self.root/'prefix',release,self.qual)
        self.assertFalse((self.root/'prefix/current').exists())
    def test_activation_previous_and_rollback_leave_both_releases(self):
        prefix=self.root/'prefix';first=self.candidate('first',True);second=self.candidate('second',True)
        deploy.activate(prefix,first,self.qual);deploy.activate(prefix,second,self.qual)
        self.assertEqual((prefix/'current').resolve(),second);self.assertEqual((prefix/'previous').resolve(),first)
        deploy.activate(prefix,(prefix/'previous').resolve(),self.qual)
        self.assertEqual((prefix/'current').resolve(),first);self.assertTrue(second.is_dir())
    def test_modified_release_is_rejected_before_current_swap(self):
        first=self.candidate('first',True);second=self.candidate('second',True);deploy.activate(self.root/'prefix',first,self.qual)
        (second/'config/profile.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'integrity'):deploy.activate(self.root/'prefix',second,self.qual)
        self.assertEqual((self.root/'prefix/current').resolve(),first)
    def test_runtime_translation_is_shifted_to_muzzle_without_changing_report(self):
        raw=copy.deepcopy(self.hand['config']['t_camera2gimbal'])
        self.p['muzzle_offset_gimbal_m']=[.08,0,.01]
        cfg=deploy.config(self.p,self.intr,self.hand)
        import numpy as np
        self.assertTrue(np.allclose(cfg['t_camera2gimbal'],np.array(raw)-[.08,0,.01]))
        self.assertEqual(raw,self.hand['config']['t_camera2gimbal'])
    def test_unknown_muzzle_origin_cannot_activate_hardware(self):
        release=self.candidate('first',True);p=copy.deepcopy(self.p);p['muzzle_offset_gimbal_m']=None
        cal.dump(release/'config/profile.json',p)
        cal.dump(release/'manifest.json',{'files':{str(f.relative_to(release)):cal.digest(f) for f in release.rglob('*') if f.is_file() and f.name!='manifest.json'}})
        with self.assertRaisesRegex(ValueError,'muzzle offset'):deploy.activate(self.root/'prefix',release,self.qual)

    def test_insufficient_accuracy_cannot_activate(self):
        release=self.candidate('first',True);q=json.loads(self.qual.read_text());q['cases'][0]['shots']=20;cal.dump(self.qual,q)
        with self.assertRaisesRegex(ValueError,'accuracy gate'):deploy.activate(self.root/'prefix',release,self.qual)
        self.assertFalse((self.root/'prefix/current').exists())

if __name__=='__main__':unittest.main()
