#!/usr/bin/env python3
"""Verify the reviewed tuning snapshot without replacing the original SP baseline."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
manifest = json.loads((ROOT/'sp_vision_25/docs/tuning-source-manifest.json').read_text())
changed = [name for name, digest in manifest['files'].items()
           if not (ROOT/name).is_file() or hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest]
result = {'status': 'FAIL' if changed else 'PASS', 'files_checked': len(manifest['files']),
          'authorized_original_sp_changes': manifest['authorized_original_sp_changes'], 'unexpected_changes': changed}
output = ROOT/'sp_vision_ros/artifacts/tuned_source_integrity.json'
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
raise SystemExit(bool(changed))
