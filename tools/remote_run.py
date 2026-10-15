#!/usr/bin/env python3
"""Synchronize and schedule this workspace on an SSH worker, never locally."""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
READY = "ZIMIAO_WORKER_READY "
EXCLUDES = [
    ".git/", ".ssh/", ".aws/", ".codex/", ".agents/", ".env", ".env.*", ".zimiao-remote-worker",
    "*.pem", "*.key", "__pycache__/", "*.pyc", ".venv/", "venv/",
    "node_modules/", "target/", "build/", "build-*/", "install/",
    "log/", "logs/", ".runtime/", "/artifacts/",
    # The integrity baseline is an input, although it lives in artifacts.
    "/sp_vision_ros/artifacts/***", "imgs/", "dataset/",
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("ZIMIAO_REMOTE_HOST", "192.168.1.102"))
    parser.add_argument("--user", default=os.environ.get("ZIMIAO_REMOTE_USER", "wuxian"))
    parser.add_argument("--port", type=int, default=22)
    parser.add_argument("--identity", help="SSH private key path (never synchronized)")
    parser.add_argument("--remote-dir", default=".cache/zimiao-remote/workspace",
                        help="Dedicated ASCII directory, relative to remote home or absolute")
    parser.add_argument("--python", default="python3", help="Remote Python for project tests")
    parser.add_argument("--jobs", type=int, default=8, help="Remote build concurrency")
    parser.add_argument("--env", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--scenario", choices=["static", "spin", "move", "all"], default="all")
    parser.add_argument("--seconds", type=float, default=25)
    parser.add_argument("--lock-timeout", type=float, default=600,
                        help="Seconds to wait for another remote job")
    parser.add_argument("--dry-run", action="store_true", help="Print plan without connecting")
    parser.add_argument("action", choices=["doctor", "check", "offline", "build", "test",
                                          "closed-loop", "sim", "aim", "run"])
    parser.add_argument("command", nargs=argparse.REMAINDER, help="Arguments for run/sim/aim, after --")
    args = parser.parse_args()
    args.command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not re.fullmatch(r"[a-zA-Z0-9_.-]+", args.user) or not re.fullmatch(r"[a-zA-Z0-9_.-]+", args.host):
        parser.error("Invalid SSH host/user")
    if not 1 <= args.port <= 65535 or args.jobs < 1 or args.seconds <= 0 or args.lock_timeout < 0:
        parser.error("Invalid port, concurrency, duration or lock timeout")
    if not re.fullmatch(r"[a-zA-Z0-9_./~-]+", args.remote_dir):
        parser.error("remote-dir must be an ASCII path")
    if args.action == "run" and not args.command:
        parser.error("run requires a command after --")
    if args.command and args.action not in {"run", "sim", "aim"}:
        parser.error("This action does not accept command arguments")
    args.environment = {}
    for item in args.env:
        name, sep, value = item.partition("=")
        if not sep or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
            parser.error("--env requires KEY=VALUE")
        args.environment[name] = value
    return args


def main():
    args = parse_args()
    run_id = datetime.datetime.now().astimezone().strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
    job = {"id": run_id, "action": args.action, "python": args.python,
           "jobs": args.jobs, "env": args.environment, "skip_build": args.skip_build,
           "scenario": args.scenario, "seconds": args.seconds, "command": args.command}
    if args.dry_run:
        print(json.dumps({"target": f"{args.user}@{args.host}", "directory": args.remote_dir,
                          "job": job, "excludes": EXCLUDES, "local_fallback": False}, indent=2))
        return 0
    for program in ("ssh", "rsync"):
        if not shutil.which(program):
            raise RuntimeError(f"Local dependency missing: {program}")
    output = ROOT / "artifacts" / "remote" / run_id
    output.mkdir(parents=True)
    ssh = ["ssh", "-p", str(args.port), "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
           "-o", "StrictHostKeyChecking=yes", "-o", "ServerAliveInterval=15",
           "-o", "ServerAliveCountMax=3"]
    if args.identity:
        ssh += ["-i", str(Path(args.identity).expanduser())]
    target = f"{args.user}@{args.host}"
    transport = shlex.join(ssh)
    source = (ROOT / "tools" / "remote_worker.py").read_text()
    remote_command = shlex.join(["python3", "-u", "-c", source, args.remote_dir, str(args.lock_timeout)])
    session = None
    remote_root = None
    result = 1
    phase = "connect"
    try:
        with (output / "scheduler.log").open("w") as log:
            print(f"Remote job {run_id}: {target} ({args.action})", flush=True)
            session = subprocess.Popen(ssh + [target, remote_command], stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            # The worker holds a remote lock throughout synchronization and execution.
            for line in session.stdout:
                log.write(line); log.flush()
                if line.startswith(READY):
                    remote_root = json.loads(line[len(READY):])["root"]
                    break
                print(line, end="", flush=True)
            if remote_root is None:
                result = session.wait() or 1
                raise RuntimeError("SSH worker did not become ready. Check login/key and scheduler.log")
            phase = "sync"
            sync = ["rsync", "-az", "--delete", "--safe-links", "--protect-args", "-e", transport,
                    "--include=/sp_vision_ros/artifacts/",
                    "--include=/sp_vision_ros/artifacts/sp_vision_baseline.json"]
            sync += [f"--exclude={pattern}" for pattern in EXCLUDES]
            print("Synchronizing sources; preserving remote build caches...", flush=True)
            subprocess.run(sync + [str(ROOT) + "/", f"{target}:{remote_root}/"], check=True)
            phase = "execute"
            session.stdin.write(json.dumps(job) + "\n"); session.stdin.flush()
            for line in session.stdout:
                print(line, end="", flush=True)
                log.write(line); log.flush()
            result = session.wait()
            session.stdin.close()
    finally:
        if session is not None and session.poll() is None:
            # Closing input aborts before dispatch; worker handles disconnect during execution.
            if not session.stdin.closed:
                session.stdin.close()
            session.terminate()
            try:
                session.wait(timeout=10)
            except subprocess.TimeoutExpired:
                session.kill(); session.wait()
        if remote_root and phase == "execute":
            print("Retrieving remote results...", flush=True)
            fetch = subprocess.run(["rsync", "-az", "--protect-args", "-e", transport,
                                    f"{target}:{remote_root}/artifacts/remote-runs/{run_id}/", str(output) + "/"])
            if fetch.returncode:
                print("Artifact retrieval failed; results remain on the worker.", file=sys.stderr)
                if result == 0:
                    result = fetch.returncode
        (output / "dispatch.json").write_text(json.dumps(
            {"target": target, "action": args.action, "remote_root": remote_root,
             "phase": phase, "exit_code": result}, indent=2) + "\n")
        print(f"Local logs/results: {output}", flush=True)
    return result


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Remote dispatch interrupted.", file=sys.stderr)
        raise SystemExit(130)
    except (RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        print(f"Remote dispatch failed: {exc}\nNo project build/test was started locally.", file=sys.stderr)
        raise SystemExit(1)
