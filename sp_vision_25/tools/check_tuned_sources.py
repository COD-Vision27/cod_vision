#!/usr/bin/env python3
"""Verify the reviewed tuning snapshot without replacing the original SP baseline."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
manifest_path = ROOT/'sp_vision_25/docs/tuning-source-manifest.json'
manifest = json.loads(manifest_path.read_text())
# Keep the historical baseline intact; newer reviewed revisions extend its hashes.
extension_path = ROOT/'sp_vision_25/docs/planner-performance-manifest.json'
if extension_path.exists():
    extension = json.loads(extension_path.read_text())
    if extension['base_manifest_sha256'] != hashlib.sha256(manifest_path.read_bytes()).hexdigest():
        raise SystemExit('Planner revision refers to a different historical source manifest')
    manifest['files'].update(extension['files'])
privacy_path = ROOT/'sp_vision_25/docs/privacy-source-manifest.json'
if privacy_path.exists():
    privacy = json.loads(privacy_path.read_text())
    if privacy['base_manifest_sha256'] != hashlib.sha256(manifest_path.read_bytes()).hexdigest():
        raise SystemExit('Privacy revision refers to a different source manifest')
    if any(not (name.endswith('.md') or '/.vscode/' in name
                or name == 'sp_vision_25/tools/check_tuned_sources.py') for name in privacy['files']):
        raise SystemExit('Privacy-only revision must not change algorithm source hashes')
    manifest['files'].update(privacy['files'])
changed = [name for name, digest in manifest['files'].items()
           if not (ROOT/name).is_file() or hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest]
result = {'status': 'FAIL' if changed else 'PASS', 'files_checked': len(manifest['files']),
          'planner_revision': extension_path.name if extension_path.exists() else None,
          'privacy_revision': privacy_path.name if privacy_path.exists() else None,
          'authorized_original_sp_changes': manifest['authorized_original_sp_changes'], 'unexpected_changes': changed}
output = ROOT/'sp_vision_ros/artifacts/tuned_source_integrity.json'
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
raise SystemExit(bool(changed))
