#!/usr/bin/env python3
"""Build and test this workspace on the current computer."""
import argparse
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def commands(action, *, skip_build=False, scenario='all', seconds=25, max_fps=0):
    python = sys.executable
    checks = [[python, 'sp_vision_25/tools/check_tuned_sources.py']]
    if action == 'doctor':
        return [[python, 'tools/host_preflight.py']]
    if action == 'check':
        return checks
    if action == 'build':
        return [['bash', 'sp_vision_sim/build.sh']]
    if action == 'sim':
        return [['bash', 'rm_aim_sim/run.sh', '--disable-vsync', '--max-fps', str(max_fps)]]
    steps = [] if skip_build else [['bash', 'sp_vision_sim/build.sh']]
    if action == 'test':
        steps += checks + [
            ['bash', 'rm_aim_sim/run.sh', '--headless', '--script', 'res://tests/run_tests.gd'],
            [python, 'rm_aim_sim/tests/integration.py', '--godot', str(ROOT/'rm_aim_sim/.tools/godot')],
        ]
    for case in ['static', 'spin', 'move'] if scenario == 'all' else [scenario]:
        steps.append([python, 'tools/aim_lab.py', '--scenario', case, '--seconds', str(seconds),
                      '--max-fps', str(max_fps)])
    if action == 'test':
        case = 'static' if scenario == 'all' else scenario
        steps.append(['sp_vision_25/build-tuning/inference_reuse_test',
                      'sp_vision_ros/configs/standard3.yaml',
                      f'artifacts/local-aim/aim-lab-{case}-first.png'])
    return steps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['doctor', 'check', 'build', 'test', 'closed-loop', 'sim'])
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--scenario', choices=['static', 'spin', 'move', 'all'], default='all')
    parser.add_argument('--seconds', type=float, default=25)
    parser.add_argument('--max-fps', type=int, default=0)
    parser.add_argument('--dry-run', action='store_true', help='Print commands without executing them')
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds <= 0 or args.max_fps < 0:
        parser.error('Test duration must be positive and FPS limit nonnegative')
    env = os.environ.copy()
    env.setdefault('SP_BUILD_JOBS', '1')
    for command in commands(args.action, skip_build=args.skip_build, scenario=args.scenario,
                            seconds=args.seconds, max_fps=args.max_fps):
        print(shlex.join(command), flush=True)
        if args.dry_run:
            continue
        result = subprocess.run(command, cwd=ROOT, env=env)
        if result.returncode:
            return result.returncode if result.returncode > 0 else 128-result.returncode
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
