"""Host-validation and telemetry regressions; no builds, SSH, or simulator needed."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch, Mock

import aim_lab
import host_preflight


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


    def test_interactive_duration_and_frame_cap(self):
        defaults=aim_lab.parse_args([])
        self.assertEqual((defaults.seconds,defaults.max_fps),(0,0))
        timed=aim_lab.parse_args(['--seconds','30','--max-fps','144'])
        self.assertEqual((timed.seconds,timed.max_fps),(30,144))
        command=aim_lab.engine_command(Path('/engine'),Path('/config.cfg'),144)
        self.assertEqual(command[command.index('--max-fps')+1],'144')
        self.assertIn('--disable-vsync',command)
        unlimited=aim_lab.engine_command(Path('/engine'),Path('/config.cfg'),0)
        self.assertEqual(unlimited[unlimited.index('--max-fps')+1],'0')
        for options in (['--seconds','-1'],['--seconds','nan'],['--seconds','inf'],['--max-fps','-1']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                aim_lab.parse_args(options)

    def test_closing_window_stops_unlimited_client(self):
        aim=Mock(returncode=0);aim.poll.return_value=None
        sim=Mock();sim.poll.return_value=0
        stop=Mock()
        self.assertEqual(aim_lab.wait_for_run(aim,sim,stop),(0,'window_closed'))
        stop.assert_called_once_with(aim)

    def test_ctrl_c_stops_client_and_engine_crash_fails(self):
        aim=Mock(returncode=0);aim.poll.return_value=None
        sim=Mock();sim.poll.return_value=None
        stop=Mock()
        with patch.object(aim_lab.time,'sleep',side_effect=KeyboardInterrupt):
            self.assertEqual(aim_lab.wait_for_run(aim,sim,stop),(0,'interrupted'))
        stop.assert_called_once_with(aim)
        sim.poll.return_value=1
        with self.assertRaisesRegex(RuntimeError,'Godot exited'):
            aim_lab.wait_for_run(aim,sim,stop)

    def test_performance_streams_across_session_reset(self):
        packets=iter([{'type':'state','session_id':'one','sim_time':1},
                      {'type':'state','session_id':'one','sim_time':3},
                      {'type':'state','session_id':'two','sim_time':0},
                      {'type':'state','session_id':'two','sim_time':1}])
        result=aim_lab.performance_summary({'wall_seconds':6},iter([]),packets,'')
        self.assertEqual(result['simulation_seconds_per_wall_second'],.5)


    def test_preflight_lists_missing_dependency_without_installing(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
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
            report=json.loads((root/'artifacts/local-preflight.json').read_text())
            self.assertEqual(report['missing'],['OpenVINO'])
            self.assertEqual(report['status'],'FAIL')

if __name__=='__main__':
    unittest.main()
