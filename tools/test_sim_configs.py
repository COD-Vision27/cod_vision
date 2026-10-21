"""Configuration workflow regressions; no ROS, compiler, GPU or SSH required."""
import importlib.util
from pathlib import Path
import tempfile
import contextlib
import io
import unittest
from unittest.mock import patch
from unittest.mock import Mock
import yaml
import remote_worker
import sim_configs
import sim_view

spec = importlib.util.spec_from_file_location('derive_configs',
    sim_configs.ROOT / 'sp_vision_ros/scripts/derive_config.py')
derive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(derive)


class ConfigWorkflowTests(unittest.TestCase):
    def test_view_uses_config_mode_unless_explicitly_overridden(self):
        for override in [[], ['--scenario', 'move']]:
            worker = Mock()
            worker.stdout = [sim_view.MARKER + '{"display": ":1"}\n']
            worker.wait.return_value = 0
            worker.poll.return_value = 0
            with patch('sys.argv', ['sim_view.py', '--view', 'remote', *override]), \
                 patch.object(sim_view.subprocess, 'Popen', return_value=worker) as spawn, \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(sim_view.main(), 0)
            argv = spawn.call_args.args[0]
            if override:
                self.assertEqual(argv[argv.index('--scenario') + 1], 'move')
            else:
                self.assertNotIn('--scenario', argv)

    def test_view_forwards_unlimited_duration_and_custom_frame_cap(self):
        worker=Mock();worker.stdout=[sim_view.MARKER+'{"display": ":1"}\n']
        worker.wait.return_value=0;worker.poll.return_value=0
        for options,seconds,fps in [([],0,0),(['--seconds','30','--max-fps','144'],30,144)]:
            with patch('sys.argv',['sim_view.py','--view','remote',*options]), \
                 patch.object(sim_view.subprocess,'Popen',return_value=worker) as spawn, \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(sim_view.main(),0)
            argv=spawn.call_args.args[0]
            self.assertEqual(float(argv[argv.index('--seconds')+1]),seconds)
            self.assertEqual(int(argv[argv.index('--max-fps')+1]),fps)

    def test_build_and_test_do_not_regenerate_tuned_inputs(self):
        for action in ['build', 'test', 'closed-loop', 'aim', 'sim']:
            commands = remote_worker.commands(dict(action=action, python='python3', jobs=2,
                env={}, skip_build=False, scenario='static', seconds=25, command=[]))
            self.assertFalse(any('derive_config.py' in arg for argv in commands for arg in argv))

    def test_generate_refuses_existing_tuning_and_preserves_toml(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'candidate.yaml'
            toml = Path(folder) / 'simulator.toml'
            toml.write_text('# manual tuning: keep this\n')
            out.write_text('tracker_max_dt: 0.23\n')
            with self.assertRaises(FileExistsError):
                derive.generate(out)
            self.assertEqual(out.read_text(), 'tracker_max_dt: 0.23\n')
            out.unlink()
            derive.generate(out)
            self.assertEqual(toml.read_text(), '# manual tuning: keep this\n')

    def test_geometry_refresh_keeps_algorithm_tuning(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'tuned.yaml'
            original = (sim_configs.CONFIGS / 'standard3_high_rate.yaml').read_text()
            out.write_text(original)
            toml = Path(folder) / 'custom.toml'
            toml.write_text((sim_configs.CONFIGS / 'simulator.toml').read_text().replace('fov = 45.0', 'fov = 50.0'))
            before = yaml.safe_load(original)
            geometry = derive.refresh_geometry(out, toml)
            after = yaml.safe_load(out.read_text())
            geometry_keys = {'camera_matrix', 'R_camera2gimbal', 't_camera2gimbal',
                             'distort_coeffs', 'sim_width', 'sim_height', 'sim_bullet_speed'}
            self.assertEqual({k:v for k,v in before.items() if k not in geometry_keys},
                             {k:v for k,v in after.items() if k not in geometry_keys})
            self.assertNotEqual(before['camera_matrix'], after['camera_matrix'])
            self.assertEqual(after['tracker_max_dt'], before['tracker_max_dt'])
            self.assertEqual(after['fire_yaw_scale'], before['fire_yaw_scale'])
            self.assertEqual(geometry['fov_y_degrees'], 50)

    def test_custom_inputs_must_be_in_synchronized_workspace(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'configs').mkdir()
            custom = root / 'configs/custom.yaml'
            custom.touch()
            (root / 'artifacts').mkdir()
            excluded = root / 'artifacts/test.yaml'
            excluded.touch()
            with patch.object(sim_configs, 'ROOT', root):
                self.assertEqual(sim_configs.resolve_config('configs/custom.yaml', custom, '.yaml'), custom)
                with self.assertRaises(ValueError):
                    sim_configs.resolve_config(str(excluded), custom, '.yaml')
                with self.assertRaises(ValueError):
                    sim_configs.resolve_config(__file__, custom, '.yaml')


if __name__ == '__main__':
    unittest.main()
