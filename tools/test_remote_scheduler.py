"""Lightweight protocol regressions; no project build, GPU, ROS or SSH needed."""
import json
from pathlib import Path
import select
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import remote_run
import remote_worker


class SchedulerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="zimiao-scheduler-")
        self.root = Path(self.temp.name) / "workspace"
        self.processes = []

    def tearDown(self):
        for proc in self.processes:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill(); proc.wait()
            for stream in (proc.stdin, proc.stdout):
                if stream and not stream.closed:
                    stream.close()
        self.temp.cleanup()

    def worker(self, timeout=2):
        proc = subprocess.Popen([sys.executable, "-u", str(Path(remote_worker.__file__)),
                                 str(self.root), str(timeout)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.processes.append(proc)
        return proc

    def ready(self, proc):
        self.assertTrue(select.select([proc.stdout], [], [], 5)[0], "Worker did not respond")
        line = proc.stdout.readline()
        self.assertTrue(line.startswith(remote_run.READY), line)

    def job(self, proc, code, run_id="test-run"):
        job = {"id": run_id, "action": "run", "python": sys.executable, "jobs": 2,
               "env": {}, "command": [sys.executable, "-c", code]}
        proc.stdin.write(json.dumps(job) + "\n"); proc.stdin.flush()
        return self.root / "artifacts/remote-runs" / run_id

    def finish(self, proc):
        # Keep stdin open while running, as the real SSH scheduler does.
        output = proc.stdout.read()
        code = proc.wait(timeout=5)
        return code, output

    def test_failure_and_changed_artifacts(self):
        proc = self.worker(); self.ready(proc)
        artifacts = self.root / "sp_vision_ros/artifacts"
        artifacts.mkdir(parents=True)
        (artifacts / "old_PASS.json").write_text("old")
        out = self.job(proc, "from pathlib import Path; "
                       "Path('sp_vision_ros/artifacts/new.json').write_text('new'); "
                       "print('remote output'); raise SystemExit(7)")
        code, output = self.finish(proc)
        self.assertEqual(code, 7)
        self.assertIn("remote output", output)
        self.assertEqual(json.loads((out / "result.json").read_text())["exit_code"], 7)
        self.assertTrue((out / "project-artifacts/new.json").exists())
        self.assertFalse((out / "project-artifacts/old_PASS.json").exists())

    def test_lock_and_release(self):
        first = self.worker(); self.ready(first)
        second = self.worker(timeout=0.1)
        code, output = self.finish(second)
        self.assertEqual(code, 1)
        self.assertIn("lock timeout", output)
        first.stdin.close(); self.assertEqual(first.wait(timeout=5), 130)
        third = self.worker(); self.ready(third)
        self.job(third, "print('ok')")
        self.assertEqual(self.finish(third)[0], 0)

    def test_refuses_existing_directory(self):
        self.root.mkdir()
        (self.root / "valuable.txt").write_text("keep")
        proc = self.worker()
        code, output = self.finish(proc)
        self.assertEqual(code, 1)
        self.assertIn("non-worker directory", output)
        self.assertEqual((self.root / "valuable.txt").read_text(), "keep")

    def test_disconnect_stops_process_group(self):
        proc = self.worker(); self.ready(proc)
        self.job(proc, "import time; print('started', flush=True); time.sleep(60)")
        self.assertIn("Running:", proc.stdout.readline())
        self.assertEqual(proc.stdout.readline().strip(), "started")
        proc.stdin.close()
        self.assertEqual(proc.wait(timeout=10), 130)
        result = json.loads((self.root / "artifacts/remote-runs/test-run/result.json").read_text())
        self.assertEqual(result["exit_code"], 130)
        self.assertIn("interrupted", result["error"])

    @unittest.skipUnless(shutil.which("rsync"), "rsync required")
    def test_sync_filters_and_cache_preservation(self):
        source = Path(self.temp.name) / "source"
        source.mkdir(); self.root.mkdir()
        inputs = ["src/edited.py", "sp_vision_ros/artifacts/sp_vision_baseline.json",
                  "sp_vision_ros/artifacts/old_result.json", ".ssh/id_ed25519",
                  ".env", "bevy/target/local-binary", "sp_vision_ros/.runtime/local-env"]
        for name in inputs:
            file = source / name; file.parent.mkdir(parents=True, exist_ok=True); file.write_text("source")
        protected = ["bevy/target/remote-binary", "sp_vision_ros/.runtime/remote-env",
                     "artifacts/remote-runs/old.json", ".zimiao-remote-worker"]
        for name in protected:
            file = self.root / name; file.parent.mkdir(parents=True, exist_ok=True); file.write_text("keep")
        (self.root / "deleted-source.py").touch()
        args = ["rsync", "-a", "--delete", "--safe-links",
                "--include=/sp_vision_ros/artifacts/",
                "--include=/sp_vision_ros/artifacts/sp_vision_baseline.json"]
        args += [f"--exclude={pattern}" for pattern in remote_run.EXCLUDES]
        subprocess.run(args + [str(source) + "/", str(self.root) + "/"], check=True)
        self.assertTrue((self.root / inputs[0]).exists())
        self.assertTrue((self.root / inputs[1]).exists())
        for name in inputs[2:]:
            self.assertFalse((self.root / name).exists(), name)
        for name in protected:
            self.assertEqual((self.root / name).read_text(), "keep", name)
        self.assertFalse((self.root / "deleted-source.py").exists())

    def test_command_arguments_are_preserved(self):
        argv = ["echo", "space and ' quote", "$(touch unwanted)"]
        self.assertEqual(remote_worker.commands({"action": "run", "python": "python3", "command": argv}), [argv])


if __name__ == "__main__":
    unittest.main()
