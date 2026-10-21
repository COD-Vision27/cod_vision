#!/usr/bin/env python3
"""One-time retirement of the user-authorized Bevy worker checkout/cache."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

root=Path(__file__).resolve().parents[1]
archive=Path.home()/'.cache/zimiao-archives/20261020-bevy-retired'
if not (root/'.zimiao-remote-worker').is_file():
    raise SystemExit('This cleanup is restricted to the managed remote workspace.')
manifest=json.loads((archive/'zimiao-retirement-manifest.json').read_text())
for name,digest in manifest.items():
    path=archive/name
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
        raise SystemExit('Archive verification failed: '+name)
old=root/'bevy_robomaster_simulator'
size=int(subprocess.check_output(['du','-sb',str(old)],text=True).split()[0]) if old.exists() else 0
if old.is_symlink(): raise SystemExit('Refusing unexpected simulator symlink')
if old.exists(): shutil.rmtree(old)
report={'archive':str(archive),'archive_sha256_files_verified':len(manifest),
        'remote_removed_bytes':size,'old_simulator_exists':old.exists()}
(root/'sp_vision_ros/artifacts/bevy-retirement-remote.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
for name in ['zimiao-godot-4.7.2.zip','zimiao-godot-complete.zip',*[f'zimiao-godot.part{i}' for i in range(5)]]:
    (Path('/tmp')/name).unlink(missing_ok=True)
