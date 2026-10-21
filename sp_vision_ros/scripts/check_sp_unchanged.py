#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parents[2]
baseline=json.loads((root/'sp_vision_ros/tests/fixtures/sp_vision_baseline.json').read_text())
changed=[name for name,digest in baseline.items() if not (root/name).exists() or hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest]
result={'status':'PASS' if not changed else 'FAIL','files_checked':len(baseline),'changed':changed}
output=root/'artifacts/sp_vision_integrity.json'
output.parent.mkdir(exist_ok=True)
output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(bool(changed))
