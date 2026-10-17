#!/usr/bin/env python3
"""Check accuracy, exact candidate provenance and paired throughput after the run."""
import hashlib
import json
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[2]


def main():
    path = ROOT / 'sp_vision_ros/artifacts/final_tuning_comparison.json'
    report = json.loads(path.read_text())
    rows = report['cases']
    candidate = [r for r in rows if r['profile'] == 'candidate']
    baseline = [r for r in rows if r['profile'] == 'baseline']
    scenarios = {'static', 'spin', 'move'}
    complete = (report['complete'] and len(rows) == 6 and len(candidate) == len(baseline) == 3
                and {r['scenario'] for r in candidate} == scenarios
                and {r['scenario'] for r in baseline} == scenarios)
    tested_path = ROOT / 'sp_vision_25/configs/simulator_high_rate.yaml'
    tested_sha = hashlib.sha256(tested_path.read_bytes()).hexdigest()
    hardware = yaml.safe_load((ROOT/'sp_vision_25/configs/standard3_high_rate.yaml').read_text())
    original = yaml.safe_load((ROOT/'sp_vision_25/configs/standard3.yaml').read_text())
    fixture = yaml.safe_load((ROOT/'sp_vision_ros/config/simulator.yaml').read_text())
    tested = yaml.safe_load(tested_path.read_text())
    overlay_keys = {'tracker_max_dt', 'fire_yaw_scale', 'fire_armor_fraction'}
    profile_preserved = ({k:v for k,v in hardware.items() if k not in overlay_keys} == original and
                         {k:v for k,v in tested.items() if k not in overlay_keys} == fixture and
                         all(hardware[k] == tested[k] for k in overlay_keys))
    accurate = all(r['shots'] >= 100 and r['conservative_hit_rate'] >= .95 and
                   r['wilson_95_lower'] >= .90 and r['stale_stop'] and r['exit_code'] == 0 and
                   r['config_sha256'] == tested_sha for r in candidate)
    comparison = []
    for scene in sorted(scenarios):
        old = next((r for r in baseline if r['scenario'] == scene), None)
        new = next((r for r in candidate if r['scenario'] == scene), None)
        if old and new:
            comparison.append({'scenario': scene,
                'baseline_shots_s': old['shots_per_second'], 'candidate_shots_s': new['shots_per_second'],
                'baseline_hits_s': old['enemy_hits_per_second'], 'candidate_hits_s': new['enemy_hits_per_second'],
                'hit_throughput_change': new['enemy_hits_per_second']/old['enemy_hits_per_second']-1,
                'candidate_shots': new['shots'], 'candidate_hits': new['enemy_hit_shots'],
                'candidate_hit_rate': new['conservative_hit_rate'], 'candidate_wilson_lower': new['wilson_95_lower']})
    old_min = min((r['enemy_hits_per_second'] for r in baseline), default=0)
    new_min = min((r['enemy_hits_per_second'] for r in candidate), default=0)
    improves = new_min > old_min > 0
    baseline_manifest = json.loads((ROOT / 'sp_vision_ros/artifacts/sp_vision_baseline.json').read_text())
    changed = {name: {'before': expected, 'after': hashlib.sha256((ROOT/name).read_bytes()).hexdigest()}
               for name, expected in baseline_manifest.items()
               if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected}
    passed = complete and accurate and report['fixture_unchanged'] and improves and profile_preserved
    result = {'status': 'PASS' if passed else 'FAIL', 'comparison_report': str(path.relative_to(ROOT)),
        'comparison_report_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'thresholds': {'min_shots_per_candidate_case': 100, 'min_hit_rate': .95, 'min_wilson_95_lower': .90},
        'objective': 'maximize worst-scene confirmed enemy hits per second, subject to accuracy limits',
        'complete': complete, 'accuracy_pass': accurate, 'fixture_unchanged': report['fixture_unchanged'],
        'hardware_profile_preserved': profile_preserved,
        'algorithm_overlay': {k:hardware[k] for k in sorted(overlay_keys)},
        'worst_scene_improves': improves, 'baseline_worst_hits_s': old_min, 'candidate_worst_hits_s': new_min,
        'comparison': comparison, 'tested_yaml_sha256': tested_sha,
        'hardware_yaml_sha256': hashlib.sha256((ROOT/'sp_vision_25/configs/standard3_high_rate.yaml').read_bytes()).hexdigest(),
        'changed_original_sp_files': changed, 'source_hashes': report['source_hashes']}
    output = ROOT / 'sp_vision_ros/artifacts/high_rate_qualification.json'
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ['changed_original_sp_files', 'source_hashes']}, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
