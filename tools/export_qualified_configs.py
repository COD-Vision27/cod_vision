#!/usr/bin/env python3
"""Export the exact configuration bytes proven by the qualification report."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
ART=ROOT/'sp_vision_ros/artifacts'
report=json.loads((ART/'accuracy_qualification.json').read_text())
if report['status']!='PASS':raise SystemExit('qualification has not passed')
for base,name in [('standard3','simulator.yaml'),('standard4','simulator4.yaml')]:
    source=ROOT/f'sp_vision_ros/configs/{base}.yaml'
    expected=report['source_hashes'].get(f'sp_vision_ros/configs/{base}.yaml',report['source_hashes'].get('sp_vision_ros/config/'+name))
    actual=hashlib.sha256(source.read_bytes()).hexdigest()
    if actual!=expected:raise SystemExit('current config differs from actual qualified config: '+base)
    (ART/f'qualified-{base}.yaml').write_bytes(source.read_bytes())
for name in ['geometry.json','simulator.toml']:
    (ART/name).write_bytes((ROOT/'sp_vision_ros/configs'/name).read_bytes())
(ART/'qualified_config_validation.json').write_text(json.dumps({'status':'PASS','source_hashes':report['source_hashes']},indent=2)+'\n')
print('PASS: exact qualified configuration files exported')
