"""Host-validation and telemetry regressions; no builds, SSH, or simulator needed."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import aim_lab
import host_preflight
import remote_worker


class ValidationTests(unittest.TestCase):
    def test_keeps_render_and_vision_fps_distinct(self):
        result=aim_lab.performance_summary({'frames':25,'wall_seconds':10},
            [{'event':'vision','processing_ms':4},{'event':'vision','processing_ms':8}],
            [{'event':'plan','control':True,'observation_age_s':.02},
             {'event':'plan','control':False,'observation_age_s':999},
             {'type':'state','sim_time':1},{'type':'state','sim_time':9}],
            'Using Device #0: Intel - Arc\nProject FPS: 60 (16.66 mspf)\nProject FPS: 40 (25 mspf)\n')
        self.assertEqual(result['vision_processed_fps'],2.5)
        self.assertEqual(result['render_fps_median'],50)
        self.assertEqual(result['processing_ms_p50'],6)
        self.assertEqual(result['observation_age_ms_p50'],20)
        self.assertEqual(result['simulation_seconds_per_wall_second'],.8)
        self.assertFalse(result['software_renderer'])

    def test_missing_measurement_is_not_success_or_zero(self):
        result=aim_lab.performance_summary({},[],[],'')
        self.assertIsNone(result['render_fps_median'])
        self.assertIsNone(result['software_renderer'])
        self.assertIsNone(result['vision_processed_fps'])
        self.assertIsNone(result['observation_age_ms_p95'])
        result=aim_lab.performance_summary({},[],[],'OpenGL - Using Device: Mesa - llvmpipe')
        self.assertTrue(result['software_renderer'])

    def test_local_run_requires_explicit_opt_in(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(aim_lab,'ROOT',Path(folder)), patch.dict('os.environ',{},clear=True):
                with patch.object(sys,'argv',['aim_lab.py']):
                    with self.assertRaisesRegex(RuntimeError,'pass --local'):
                        aim_lab.main()
                with patch.object(sys,'argv',['aim_lab.py','--local']):
                    with self.assertRaisesRegex(RuntimeError,'requires DISPLAY'):
                        aim_lab.main()

    def test_validate_prepares_before_build_and_real_graphics(self):
        steps=remote_worker.commands(dict(action='validate',python='python3',env={},jobs=2,
            skip_build=False,scenario='static',seconds=15,command=[]))
        self.assertEqual(steps[0],['python3','tools/host_preflight.py'])
        self.assertEqual(steps[1],['python3','rm_aim_sim/tools/setup_godot.py'])
        self.assertEqual(steps[3],['bash','sp_vision_sim/build.sh'])
        self.assertTrue(any('tools/aim_lab.py' in command for command in steps))
        self.assertFalse(any('cargo' in argument for command in steps for argument in command))

    def test_preflight_lists_missing_dependency_without_installing(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'.zimiao-remote-worker').touch()
            def cmake_probe(argv,**kwargs):
                source=Path(argv[argv.index('-S')+1])
                script=(source/'CMakeLists.txt').read_text()
                self.assertIn('\nproject(worker_preflight',script)
                self.assertIn('OpenVINO',script)
                build=Path(argv[argv.index('-B')+1]);build.mkdir()
                (build/'packages.txt').write_text(''.join(
                    p+'='+('no' if p=='OpenVINO' else 'yes')+'\n' for p in host_preflight.PACKAGES))
                return subprocess.CompletedProcess(argv,0,'','')
            with patch.object(host_preflight,'ROOT',root), \
                 patch.object(host_preflight.platform,'system',return_value='Linux'), \
                 patch.object(host_preflight.platform,'machine',return_value='x86_64'), \
                 patch.object(host_preflight.platform,'platform',return_value='Linux test worker'), \
                 patch.object(host_preflight.shutil,'which',side_effect=lambda name:'/usr/bin/'+name), \
                 patch.object(host_preflight.subprocess,'run',side_effect=cmake_probe), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(host_preflight.main(),2)
            report=json.loads((root/'sp_vision_ros/artifacts/host-preflight.json').read_text())
            self.assertEqual(report['missing'],['OpenVINO'])
            self.assertEqual(report['status'],'FAIL')

if __name__=='__main__':
    unittest.main()
