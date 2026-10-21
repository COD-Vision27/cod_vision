#!/usr/bin/env python3
"""Offline shot timing, in simulation seconds; no simulator or ROS required.

Rolling TTK is an optimistic HP/damage conversion, not a referee simulation:
no regeneration, buffs, heat, normal-impact-speed or per-armor debounce model.
Only hits from rounds launched at/after the window start are eligible.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics


def summarize(events, hp=200, damage=20):
    if not math.isfinite(hp) or not math.isfinite(damage) or hp <= 0 or damage <= 0:
        raise ValueError('HP and damage must be finite and positive')
    shots = {e['shot_id']: e['launch_timestamp_ns'] for e in events
             if e.get('event') == 'shot_fired' and e.get('kind') == 'projectile'}
    hits = {}
    for e in events:
        if (e.get('event') == 'armor_hit' and e.get('kind') == 'projectile'
                and e.get('team') == 'blue' and e.get('armor_label') == 3
                and e['shot_id'] in shots):
            sid, timestamp = e['shot_id'], e['hit_timestamp_ns']
            if timestamp < shots[sid]:
                raise ValueError('impact precedes launch')
            hits[sid] = min(hits.get(sid, timestamp), timestamp)
    ordered = sorted(shots, key=lambda sid: (shots[sid], sid))
    needed = math.ceil(hp / damage)
    windows = []
    for i, sid in enumerate(ordered):
        eligible = sorted((hits[s], s) for s in ordered[i:] if s in hits)
        if len(eligible) >= needed:
            end, last = eligible[needed - 1]
            windows.append({'start_shot_id': sid, 'last_hit_shot_id': last,
                            'launch_ns': shots[sid], 'impact_ns': end,
                            'seconds': (end - shots[sid]) / 1e9})
    launch_times = sorted(shots.values())
    gaps = [(b-a)/1e9 for a, b in zip(launch_times, launch_times[1:])]
    gates = [e for e in events if e.get('event') == 'sp_fire_gate']
    gate_counts = {
        'valid_plans': len(gates),
        'blocked_planner': sum(not e['requested'] for e in gates),
        'blocked_alignment': sum(e['requested'] and not e['aligned'] for e in gates),
        'blocked_rate': sum(e['requested'] and e['aligned'] and not e['rate_allowed'] for e in gates),
        'allowed': sum(e['rate_allowed'] for e in gates),
    } if gates else None
    return {'hp': hp, 'damage': damage, 'hits_required': needed,
            'shots': len(shots), 'unique_enemy_hits': len(hits),
            'hit_rate': len(hits)/len(shots) if shots else None,
            'shot_interval_median_s': statistics.median(gaps) if gaps else None,
            'shot_interval_min_s': min(gaps) if gaps else None,
            'first_shot_to_first_hit_s': (min(hits.values())-min(shots.values()))/1e9 if hits else None,
            'first_burst_ttk': windows[0] if windows else None,
            'best_rolling_ttk': min(windows, key=lambda w: w['seconds']) if windows else None,
            'eligible_windows': len(windows), 'fire_gates': gate_counts}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metrics', nargs='+', type=Path)
    parser.add_argument('--hp', type=float, nargs='+', default=[150, 200, 400])
    parser.add_argument('--damage', type=float, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for path in args.metrics:
        data = path.read_bytes()
        events = [json.loads(line) for line in data.splitlines() if line.strip()]
        rows.append({'metrics': str(path.resolve()), 'sha256': hashlib.sha256(data).hexdigest(),
                     'cases': [summarize(events, hp, args.damage) for hp in args.hp]})
    report = {'schema': 'shot_timing_v1', 'clock': 'simulation',
              'model': 'HP/damage conversion only; ignores referee/heat/buff mechanics', 'runs': rows}
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
