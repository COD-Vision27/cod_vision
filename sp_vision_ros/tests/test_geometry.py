#!/usr/bin/env python3
import importlib.util
import unittest
from pathlib import Path
import numpy as np

script=Path(__file__).resolve().parents[1]/'scripts/derive_config.py'
spec=importlib.util.spec_from_file_location('derive',script)
derive=importlib.util.module_from_spec(spec); spec.loader.exec_module(derive)

class GeometryTests(unittest.TestCase):
    def test_camera_and_projectile_have_same_forward_axis(self):
        g=derive.derive(); basis=np.array(g['gun_basis_bevy'])
        shot=derive.quat_matrix(g['shot_rotation_xyzw'])
        np.testing.assert_allclose(basis@np.array([0.,0.,-1.]),shot@np.array([0.,1.,0.]),atol=1e-12)

    def test_extrinsic_reconstructs_world_points_under_arbitrary_body_rotation(self):
        g=derive.derive(); basis=np.array(g['gun_basis_bevy']); r=np.array(g['R_camera2gimbal']);t=np.array(g['t_camera2gimbal'])
        view=np.array(g['camera_marker_bevy']);muzzle=np.array(g['muzzle_marker_bevy'])
        for angles in [(0,0,0),(.3,-.2,.1),(2.8,.5,-.4)]:
            a,b,c=angles
            rx=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
            ry=np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
            rz=np.array([[np.cos(c),-np.sin(c),0],[np.sin(c),np.cos(c),0],[0,0,1]])
            world=ry@rx@rz; body=np.array([1.,.4,3.])
            for pc in [np.array([.2,-.3,5.]),np.array([-1.,1.,8.])]:
                actual=derive.ALIGN@(body+world@view+world@basis@derive.OPTICAL@pc)
                q=derive.ALIGN@world@basis@derive.ALIGN.T
                recovered=derive.ALIGN@(body+world@muzzle)+q@(r@pc+t)
                np.testing.assert_allclose(actual,recovered,atol=1e-12)

    def test_projection_resolution_fov_and_zero_distortion(self):
        g=derive.derive();k=np.array(g['camera_matrix'])
        self.assertEqual((g['width'],g['height']),(1440,1080))
        self.assertAlmostEqual(k[0,0],k[1,1],places=10)
        self.assertAlmostEqual(k[0,0],1303.67528,places=4)
        self.assertEqual(g['bullet_speed'],25)

if __name__=='__main__': unittest.main(verbosity=2)
