"""Ensure public entrypoints execute locally and need no private worker files."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import aim_lab
import local_run


class LocalExecutionTests(unittest.TestCase):
    def test_default_aim_is_local_without_worker_marker(self):
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(aim_lab, 'ROOT', Path(folder)), patch.dict('os.environ', {}, clear=True), \
                 patch.object(sys, 'argv', ['aim_lab.py']):
                with self.assertRaisesRegex(RuntimeError, 'requires DISPLAY'):
                    aim_lab.main()

    def test_test_plan_has_local_build_and_all_scenarios(self):
        steps = local_run.commands('test')
        self.assertEqual(steps[0], ['bash', 'sp_vision_sim/build.sh'])
        cases = [s[s.index('--scenario') + 1] for s in steps if '--scenario' in s]
        self.assertEqual(cases, ['static', 'spin', 'move'])
        self.assertTrue(any('run_tests.gd' in ' '.join(s) for s in steps))
        self.assertTrue(any('inference_reuse_test' in s[0] for s in steps))
        self.assertFalse(any(s[0] in ('ssh', 'rsync') for s in steps))

    def test_dry_run_does_not_execute_or_download(self):
        with patch.object(local_run.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(local_run.main(['test', '--dry-run']), 0)
        run.assert_not_called()

    def test_local_failures_stop_remaining_commands(self):
        with patch.object(local_run.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()):
            run.return_value.returncode = 7
            self.assertEqual(local_run.main(['test']), 7)
        self.assertEqual(run.call_count, 1)
        self.assertEqual(run.call_args.kwargs['env']['SP_BUILD_JOBS'], '1')

    def test_public_editor_tasks_use_local_entrypoint(self):
        tasks = json.loads((local_run.ROOT / '.vscode/tasks.json').read_text())['tasks']
        for task in tasks:
            self.assertEqual(task['args'][0], 'tools/local_run.py')
            self.assertNotIn('--host', task['args'])

    def test_tuning_check_does_not_overwrite_config(self):
        self.assertEqual(local_run.commands('check')[0][-1], 'sp_vision_25/tools/check_tuned_sources.py')
        self.assertFalse(any('derive_config.py' in argument for s in local_run.commands('test') for argument in s))


if __name__ == '__main__':
    unittest.main()
