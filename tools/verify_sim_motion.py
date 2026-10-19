#!/usr/bin/env python3
"""Remote GPU integration check for TOML-controlled target motion (no auto-aim)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'sp_vision_ros/artifacts'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12)
    args = parser.parse_args()
    if args.seconds < 10:
        parser.error('At least 10 seconds per case required')
    OUT.mkdir(exist_ok=True)
    base = (ROOT / 'sp_vision_ros/configs/simulator.toml').read_text().split('[scenario]')[0]
    cases = [('positive', 3.0), ('negative', -2.0), ('stopped', 0.0), ('inherited', None), ('move', None)]
    report = {'status': 'INCOMPLETE', 'cases': []}
    for name, rate in cases:
        config = base
        if name == 'inherited':
            config = re.sub(r'^rotation_speed\s*=.*$', 'rotation_speed = 1.5', config, flags=re.M)
        config += '\n[scenario]\nmode = "' + ('move' if name == 'move' else 'spin') + '"\n'
        if rate is not None:
            config += f'spin_speed_rad_s = {rate}\n'
        if name == 'move':
            config += ('target_distance_m = 5.0\ninitial_pitch_deg = -7.0\n'
                       'target_lateral_m = 0.1\nmove_amplitude_m = 0.4\n'
                       'move_peak_speed_mps = 0.32\nmove_start_delay_s = 1.0\n')
        path = OUT / f'motion-{name}.toml'
        path.write_text(config)
        env = dict(os.environ, SP_SIM_CONFIG=str(path), DAEDALUS_AUTO_AIM='0',
                   RUST_LOG='daedalus=info,bevy_render=info,wgpu=warn')
        env.pop('DAEDALUS_SP_SCENARIO', None)  # The config must select the mode.
        log = OUT / f'motion-{name}.log'
        child = None
        try:
            with log.open('w') as handle:
                child = subprocess.Popen(['bash', 'sp_vision_ros/scripts/run_sim.sh',
                    '--duration-secs', str(args.seconds)], cwd=ROOT, env=env,
                    stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
                code = child.wait(timeout=args.seconds + 45)
        finally:
            if child and child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        text = re.sub(r'\x1b\[[0-9;]*m', '', log.read_text())
        row = {'case': name, 'exit_code': code, 'config_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
               'gpu': 'NVIDIA GeForce RTX 4060' in text}
        valid = code == 0 and row['gpu']
        if name != 'move':
            samples = [tuple(map(float, pair)) for pair in re.findall(
                r'target_rate=([-\d.]+) rad/s actual_rate=([-\d.]+)', text)]
            expected = 1.5 if rate is None else rate
            row.update(expected_rate=expected, tail_samples=samples[-3:])
            valid = valid and len(samples) >= 3 and all(
                abs(target - expected) < 0.01 and abs(actual - expected) < 0.01
                for target, actual in samples[-3:])
        else:
            samples = [tuple(map(float, triple)) for triple in re.findall(
                r'x=([-\d.]+) m vx=([-\d.]+) m/s desired_x=([-\d.]+) m', text)]
            row['samples'] = samples
            valid = valid and len(samples) >= 5 and all(-0.301 <= s[2] <= 0.501 for s in samples)
            valid = valid and max((s[2] for s in samples), default=0) - min((s[2] for s in samples), default=0) > 0.2
            valid = valid and any(abs(s[1]) > 0.05 for s in samples)
            valid = valid and 'target_distance_m: 5.0' in text and 'initial_pitch_deg: -7.0' in text
        row['status'] = 'PASS' if valid else 'FAIL'
        report['cases'].append(row)
        report['status'] = ('PASS' if all(r['status'] == 'PASS' for r in report['cases']) else 'FAIL') if len(report['cases']) == len(cases) else 'INCOMPLETE'
        (OUT / 'motion-config-validation.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(row), flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
