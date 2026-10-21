"""Local training planning must not contact a worker or fabricate absent assets."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('local_training_pipeline', ROOT/'sp_training/armor_training/pipeline.py')
pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline)


class LocalTrainingTests(unittest.TestCase):
    def test_public_plan_is_local_and_waits_for_real_assets(self):
        plan, _, config = pipeline.load_plan(ROOT/'sp_training/configs/pipeline.json')
        self.assertEqual(plan['execution'], 'local_only')
        self.assertEqual(config['execution']['required_role'], 'local')
        self.assertFalse(plan['ready_to_execute'])

    def test_missing_assets_never_start_production(self):
        with patch.dict('os.environ', {'SP_ARMOR_EXECUTION_ROLE': 'local'}), \
             patch.object(pipeline.subprocess, 'run') as run:
            with self.assertRaisesRegex(RuntimeError, 'awaits real assets'):
                pipeline.run_pipeline(ROOT/'sp_training/configs/pipeline.json', execute=True)
        run.assert_not_called()

    def test_nonlocal_execution_location_is_rejected(self):
        config = json.loads((ROOT/'sp_training/configs/pipeline.json').read_text())
        config['execution']['location'] = 'external'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'pipeline.json'
            path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, 'local machine'):
                pipeline.load_plan(path)


if __name__ == '__main__':
    unittest.main()
