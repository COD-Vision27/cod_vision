#!/usr/bin/env python3
"""Compare SP algorithm parameters using the existing, unmodified remote ROS fixture.

Run through tools/remote_run.py after sourcing sp_vision_ros/scripts/env.sh.
No simulator physics, rendering, control gates, target paths or truth inputs change.
"""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
ART = ROOT / 'sp_vision_ros/artifacts'
PROFILES = {
    'candidate': {},
    'baseline': {},
    'reuse': {'reuse_infer_request': True},
    'reuse_gap200': {'reuse_infer_request': True, 'tracker_max_dt': 0.2},
    'reuse_window2': {'reuse_infer_request': True, 'fire_yaw_scale': 2.0, 'fire_armor_fraction': 0.5},
    'reuse_gap200_window2': {'reuse_infer_request': True, 'tracker_max_dt': 0.2, 'fire_yaw_scale': 2.0, 'fire_armor_fraction': 0.5},
    'window2': {'fire_yaw_scale': 2.0, 'fire_armor_fraction': 0.5},
    'gap200_window2': {'tracker_max_dt': 0.2, 'fire_yaw_scale': 2.0, 'fire_armor_fraction': 0.5},
    'gap200_window3': {'tracker_max_dt': 0.2, 'fire_yaw_scale': 3.0, 'fire_armor_fraction': 0.5},
    'gap200_noise25_100': {'tracker_max_dt': 0.2, 'acceleration_variance': 25, 'angular_acceleration_variance': 100},
    'gap200_noise25_25': {'tracker_max_dt': 0.2, 'acceleration_variance': 25, 'angular_acceleration_variance': 25},
    'prior': {'initial_radius_variance': 0.01, 'initial_dr_variance': 0.01, 'initial_dz_variance': 0.0025},
    'prior_gap150': {'initial_radius_variance': 0.01, 'initial_dr_variance': 0.01, 'initial_dz_variance': 0.0025, 'tracker_max_dt': 0.15},
    'prior_gap200': {'initial_radius_variance': 0.01, 'initial_dr_variance': 0.01, 'initial_dz_variance': 0.0025, 'tracker_max_dt': 0.2},
    'prior_gap200_q1m': {'initial_radius_variance': 0.01, 'initial_dr_variance': 0.01, 'initial_dz_variance': 0.0025, 'tracker_max_dt': 0.2, 'Q_yaw': [1e6, 0], 'Q_pitch': [1e6, 0]},
    'prior_gap200_q30m': {'initial_radius_variance': 0.01, 'initial_dr_variance': 0.01, 'initial_dz_variance': 0.0025, 'tracker_max_dt': 0.2, 'Q_yaw': [3e7, 0], 'Q_pitch': [3e7, 0]},
    'prior_gap200_fire7': {'initial_radius_variance': 0.01, 'initial_dr_variance': 0.01, 'initial_dz_variance': 0.0025, 'tracker_max_dt': 0.2, 'fire_thresh': 0.007},
    'gap150': {'tracker_max_dt': 0.15},
    'gap200': {'tracker_max_dt': 0.2},
    'fire7': {'fire_thresh': 0.007},
    'gap200_fire7': {'tracker_max_dt': 0.2, 'fire_thresh': 0.007},
    'gap200_q1m': {'tracker_max_dt': 0.2, 'Q_yaw': [1e6, 0], 'Q_pitch': [1e6, 0]},
    'gap200_q30m': {'tracker_max_dt': 0.2, 'Q_yaw': [3e7, 0], 'Q_pitch': [3e7, 0]},
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def protected_hashes():
    paths = list((ROOT / 'bevy_robomaster_simulator/src').rglob('*'))
    paths += list((ROOT / 'bevy_robomaster_simulator/assets').rglob('*'))
    paths += [ROOT / 'bevy_robomaster_simulator/config.toml',
              ROOT / 'sp_vision_ros/src/ros_node.cpp',
              ROOT / 'sp_vision_ros/tests/closed_loop.py']
    return {str(p.relative_to(ROOT)): sha(p) for p in paths if p.is_file()}


def wilson(k, n):
    if not n:
        return 0.0
    z = 1.95996398454
    p = k / n
    return (p + z*z/(2*n) - z*math.sqrt(p*(1-p)/n + z*z/(4*n*n))) / (1+z*z/n)


def summarize(result, diagnostics, aim_log, seconds):
    data = [json.loads(line) for line in diagnostics.read_text().splitlines()]
    vision = [r for r in data if r['event'] == 'vision']
    plans = [r for r in data if r['event'] == 'plan']
    stamps = sorted(r['stamp_ns'] for r in vision)
    gaps = [(b-a)/1e9 for a, b in zip(stamps, stamps[1:])]
    ages = sorted(r['age_ms'] for r in vision)
    log = aim_log.read_text()
    return {
        **result, 'duration_seconds': seconds,
        'shots_per_second': result['shots'] / seconds,
        'enemy_hits_per_second': result['enemy_hit_shots'] / seconds,
        'conservative_hit_rate': result['enemy_hit_shots'] / result['shots'] if result['shots'] else 0,
        'wilson_95_lower': wilson(result['enemy_hit_shots'], result['shots']),
        'tracker_states': dict(collections.Counter(r['tracker'] for r in vision)),
        'control_fraction': sum(r['control'] for r in plans) / max(1, len(plans)),
        'fire_fraction': sum(r['fire'] for r in plans) / max(1, len(plans)),
        'frame_gaps_over_100ms': sum(g > .1 for g in gaps),
        'frame_gaps_over_200ms': sum(g > .2 for g in gaps),
        'frame_age_p50_ms': ages[len(ages)//2] if ages else None,
        'frame_age_p95_ms': ages[min(len(ages)-1, int(len(ages)*.95))] if ages else None,
        'large_dt_resets': log.count('Large dt:'),
        'divergence_resets': log.count('Target diverged!'),
        'convergence_resets': log.count('Bad Converge Found!'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profiles', nargs='+', choices=PROFILES, default=['baseline', 'gap200'])
    parser.add_argument('--scenarios', nargs='+', choices=['static', 'spin', 'move'], default=['static', 'spin', 'move'])
    parser.add_argument('--base', choices=['standard3', 'standard4'], default='standard3')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--candidate-config', type=Path, help='Exact YAML to evaluate as profile candidate')
    parser.add_argument('--report-output', type=Path, help='Workspace-relative summary path')
    args = parser.parse_args()
    if 'candidate' in args.profiles and not args.candidate_config:
        parser.error('candidate profile requires --candidate-config')
    if not (ROOT / '.zimiao-remote-worker').exists():
        parser.error('Use the SSH scheduler: this harness runs only in its remote workspace.')
    if args.seconds < 30 or args.repeats < 1:
        parser.error('At least 30 seconds and one repeat required')
    ART.mkdir(exist_ok=True)
    tag = time.strftime('%Y%m%d-%H%M%S')
    before = protected_hashes()
    source_hashes = {str(p.relative_to(ROOT)): sha(p) for p in (ROOT / 'sp_vision_25/tasks/auto_aim').rglob('*') if p.is_file()}
    base = ROOT / 'sp_vision_ros/config' / ('simulator.yaml' if args.base == 'standard3' else 'simulator4.yaml')
    rows = []
    report_path = (ROOT / args.report_output).resolve() if args.report_output else ART / f'tuning-{tag}.json'
    for profile in args.profiles:
        config = ART / f'tuning-{tag}-{args.base}-{profile}.yaml'
        candidate = (ROOT / args.candidate_config).resolve() if profile == 'candidate' else base
        text = candidate.read_text()
        for key, value in PROFILES[profile].items():
            line = f'{key}: {json.dumps(value)}'
            text = re.sub(rf'^{key}:.*$', line, text, flags=re.M) if re.search(rf'^{key}:', text, re.M) else text + '\n' + line + '\n'
        config.write_text(text)
        for repeat in range(args.repeats):
            for scenario in args.scenarios:
                label = f'tuning-{tag}-{args.base}-{profile}-{repeat+1}-{scenario}'
                print(f'{label}: {args.seconds}s', flush=True)
                env = os.environ.copy()
                env['SP_AIM_CONFIG'] = str(config)
                result_path = ART / f'{scenario}_result.json'
                previous = result_path.stat().st_mtime_ns if result_path.exists() else None
                code = subprocess.call([sys.executable, str(ROOT / 'sp_vision_ros/tests/closed_loop.py'),
                                        '--scenario', scenario, '--seconds', str(args.seconds)], env=env)
                if not result_path.exists() or result_path.stat().st_mtime_ns == previous:
                    raise RuntimeError(f'{label}: no fresh result, exit {code}')
                result = json.loads(result_path.read_text())
                row = summarize(result, ART / f'{scenario}_diagnostics.jsonl', ART / f'{scenario}_aim.log', args.seconds)
                row.update(profile=profile, base=args.base, repeat=repeat+1, overrides=PROFILES[profile],
                           exit_code=code, config_sha256=sha(config), evidence_prefix=label,
                           config_source=str(candidate.relative_to(ROOT)), config_source_sha256=sha(candidate))
                rows.append(row)
                for suffix in ['result.json', 'diagnostics.jsonl', 'metrics.jsonl', 'aim.log', 'sim.log', 'last_frame.png']:
                    source = ART / f'{scenario}_{suffix}'
                    if source.exists():
                        shutil.copy2(source, ART / f'{label}-{suffix}')
                unchanged = before == protected_hashes()
                report_path.write_text(json.dumps({'schema': 'sp_parameter_tuning_v1', 'tag': tag,
                    'source_hashes': source_hashes, 'fixture_hashes': before, 'fixture_unchanged': unchanged,
                    'base_config_sha256': sha(base), 'cases': rows,
                    'expected_cases': len(args.profiles)*len(args.scenarios)*args.repeats,
                    'complete': len(rows) == len(args.profiles)*len(args.scenarios)*args.repeats}, indent=2) + '\n')
                print(json.dumps(row), flush=True)
                if code or not unchanged:
                    raise RuntimeError(f'{label}: fixture changed or closed loop failed')
    print(f'Report: {report_path}', flush=True)


if __name__ == '__main__':
    main()
