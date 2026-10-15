"""SSH-side protocol worker. Loaded by remote_run.py; not a local test entrypoint."""
import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time


def commands(job):
    python = job["python"]
    # The tuning agent keeps a separate reviewed manifest; retain the original
    # integration baseline and do not silently rewrite it after tuning.
    tuned_manifest = Path('sp_vision_25/docs/tuning-source-manifest.json')
    integrity = ('sp_vision_25/tools/check_tuned_sources.py' if tuned_manifest.is_file()
                 else 'sp_vision_ros/scripts/check_sp_unchanged.py')
    checks = [[python, "sp_vision_ros/tests/test_geometry.py"],
              [python, integrity]]
    # Configuration is a user input. Rebuilding must not overwrite tuning.
    build = [["bash", "sp_vision_ros/scripts/build.sh"]]
    ros_prefix = "source sp_vision_ros/scripts/env.sh && "
    def ros(argv):
        return ["bash", "-c", ros_prefix + shlex.join(argv)]
    def graphics(argv):
        # Check at execution time, after earlier build/unit-test steps have run.
        return ["bash", "-c",
                'if [[ -n "${DISPLAY:-}" ]]; then exec "$@"; '
                'elif command -v xvfb-run >/dev/null; then exec xvfb-run -a "$@"; '
                'else echo "No DISPLAY or xvfb-run. Supply a valid remote DISPLAY or install xvfb; '
                'a working Vulkan driver is required." >&2; exit 1; fi',
                "zimiao-display", *argv]
    action = job["action"]
    if action == "check":
        return checks
    if action == "offline":
        return [["cmake", "-S", "sp_vision_ros", "-B", "sp_vision_ros/build-offline",
                 "-DSP_BUILD_ROS=OFF", "-DCMAKE_BUILD_TYPE=Release",
                 "-DOpenVINO_DIR=" + job["env"].get("OpenVINO_DIR", "/opt/intel/openvino_2024.6.0/runtime/cmake")],
                ["cmake", "--build", "sp_vision_ros/build-offline", "-j", str(job["jobs"])],
                ["ctest", "--test-dir", "sp_vision_ros/build-offline", "--output-on-failure"]] + checks
    if action == "build":
        return build
    if action == "run":
        return [job["command"]]
    if action in {"sim", "aim"}:
        launch = ["bash", f"sp_vision_ros/scripts/run_{action}.sh"] + job["command"]
        if action == "sim":
            launch = graphics(launch)
        return ([] if job["skip_build"] else build) + [launch]
    steps = [] if job["skip_build"] else build
    if action == "test":
        steps += checks + [ros([python, "sp_vision_ros/tests/ros_smoke.py"]),
                           ["bash", "-c", ros_prefix + "cd bevy_robomaster_simulator && "
                            "cargo test --no-default-features --features ros2 --bin daedalus -j " + str(job["jobs"])]]
    scenarios = ["static", "spin", "move"] if job["scenario"] == "all" else [job["scenario"]]
    for scenario in scenarios:
        steps.append(graphics(ros([python, "sp_vision_ros/tests/closed_loop.py", "--scenario", scenario,
                                   "--seconds", str(job["seconds"])])))
    return steps


def snapshot(root):
    folder = root / "sp_vision_ros" / "artifacts"
    return {p.name: (p.stat().st_mtime_ns, p.stat().st_size) for p in folder.glob("*") if p.is_file()}


def stop_child(child):
    if child and child.poll() is None:
        os.killpg(child.pid, signal.SIGINT)
        try:
            child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()


def doctor(root, job, env):
    tools = {name: shutil.which(name, path=env["PATH"]) for name in
             ["rsync", "python3", "cmake", "ctest", "cargo", "rustc", "cc", "c++", "xvfb-run", "vulkaninfo"]}
    probe = subprocess.run([job["python"], "-c",
                            "import importlib.util,json; print(json.dumps({n: importlib.util.find_spec(n) is not None "
                            "for n in ['numpy','cv2','rclpy']}))"], env=env, text=True,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    gpu = None
    if shutil.which("nvidia-smi", path=env["PATH"]):
        try:
            info = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"],
                                  env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
            gpu = {"exit_code": info.returncode, "output": info.stdout.strip()}
        except subprocess.TimeoutExpired:
            gpu = {"error": "nvidia-smi timed out"}
    openvino = env.get("OpenVINO_DIR", "/opt/intel/openvino_2024.6.0/runtime/cmake")
    return {"hostname": os.uname().nodename, "cpu_count": os.cpu_count(),
            "workspace": str(root), "tools": tools, "python_probe": probe.stdout.strip(),
            "python_probe_exit": probe.returncode, "display": env.get("DISPLAY"),
            "ros_humble": Path("/opt/ros/humble/setup.bash").exists(),
            "ros_setups": {name: str(Path('/opt/ros') / name / 'setup.bash')
                           for name in ['humble', 'jazzy']
                           if (Path('/opt/ros') / name / 'setup.bash').exists()},
            "local_ros_runtime": (root / "sp_vision_ros/.runtime/setup.bash").exists(),
            "openvino_dir": openvino, "openvino_exists": Path(openvino).is_dir(), "nvidia_gpu": gpu,
            "note": "doctor checks availability; it does not prove full builds or rendering work"}


def main():
    root = Path(sys.argv[1]).expanduser()
    if not root.is_absolute():
        root = Path.home() / root
    root = root.resolve()
    if root == Path.home() or root == Path("/") or not str(root).isascii():
        raise RuntimeError("Use a dedicated ASCII worker directory, not HOME or filesystem root")
    root.parent.mkdir(parents=True, exist_ok=True)
    # A separate lock file survives rsync --delete and protects all job phases.
    with (root.parent / (root.name + ".lock")).open("a") as lock:
        deadline = time.monotonic() + float(sys.argv[2])
        announced = False
        while True:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if not announced:
                    print("Another remote job is running; waiting for workspace lock...", flush=True)
                    announced = True
                if time.monotonic() >= deadline:
                    raise RuntimeError("Remote workspace lock timeout")
                # Exit if the scheduler disappears while waiting.
                if select.select([sys.stdin], [], [], 1)[0] and not sys.stdin.readline():
                    return 130
        marker = root / ".zimiao-remote-worker"
        if root.exists() and not marker.exists() and any(root.iterdir()):
            raise RuntimeError("Refusing to synchronize into an existing non-worker directory")
        root.mkdir(exist_ok=True)
        marker.touch()
        print("ZIMIAO_WORKER_READY " + json.dumps({"root": str(root)}), flush=True)
        payload = sys.stdin.readline()
        if not payload:
            return 130
        job = json.loads(payload)
        # The marker is deliberately excluded from synchronization by the scheduler.
        marker.touch()
        os.chdir(root)
        out = root / "artifacts" / "remote-runs" / job["id"]
        out.mkdir(parents=True)
        env = os.environ.copy()
        env["PATH"] = str(Path.home() / ".cargo/bin") + ":" + env.get("PATH", "")
        env.update(job["env"])
        env["SP_BUILD_JOBS"] = str(job["jobs"])
        env["CARGO_BUILD_JOBS"] = str(job["jobs"])
        before = snapshot(root)
        status = {"id": job["id"], "action": job["action"], "hostname": os.uname().nodename,
                  "started": datetime.datetime.now().astimezone().isoformat(), "steps": [], "exit_code": 1}
        child = None
        def interrupted(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupted)
        signal.signal(signal.SIGHUP, interrupted)
        try:
            if job["action"] == "doctor":
                report = doctor(root, job, env)
                (out / "doctor.json").write_text(json.dumps(report, indent=2) + "\n")
                print(json.dumps(report, indent=2), flush=True)
                status["exit_code"] = report["python_probe_exit"]
            else:
                with (out / "job.log").open("w") as log:
                    for argv in commands(job):
                        print("Running: " + shlex.join(argv), flush=True)
                        log.write("$ " + shlex.join(argv) + "\n"); log.flush()
                        child = subprocess.Popen(argv, env=env, start_new_session=True,
                                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        # Relay output separately so the main thread notices SSH disconnects
                        # even while a compiler is silent.
                        disconnected = threading.Event()
                        def relay():
                            try:
                                for line in child.stdout:
                                    log.write(line); log.flush()
                                    print(line, end="", flush=True)
                            except (BrokenPipeError, OSError):
                                disconnected.set()
                        thread = threading.Thread(target=relay, daemon=True)
                        thread.start()
                        try:
                            while child.poll() is None:
                                if disconnected.wait(0.25):
                                    raise KeyboardInterrupt
                                if select.select([sys.stdin], [], [], 0)[0] and not os.read(sys.stdin.fileno(), 1):
                                    raise KeyboardInterrupt
                        finally:
                            stop_child(child)
                            thread.join(timeout=5)
                        code = child.returncode
                        status["steps"].append({"argv": argv, "exit_code": code})
                        status["exit_code"] = code
                        if code:
                            break
        except KeyboardInterrupt:
            status["exit_code"] = 130
            status["error"] = "Remote job interrupted or SSH output disconnected"
        except Exception as exc:
            status["error"] = str(exc)
            print(f"Worker failed: {exc}", file=sys.stderr, flush=True)
        finally:
            stop_child(child)
            # Return only files written by this run, so old PASS reports are not misattributed.
            changed = snapshot(root)
            collected = out / "project-artifacts"
            for name, stat in changed.items():
                if before.get(name) != stat and name != "sp_vision_baseline.json":
                    collected.mkdir(exist_ok=True)
                    shutil.copy2(root / "sp_vision_ros/artifacts" / name, collected / name)
            status["finished"] = datetime.datetime.now().astimezone().isoformat()
            (out / "result.json").write_text(json.dumps(status, indent=2) + "\n")
        return status["exit_code"] if status["exit_code"] >= 0 else 128 - status["exit_code"]


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError) as exc:
        print(f"Worker setup failed: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
