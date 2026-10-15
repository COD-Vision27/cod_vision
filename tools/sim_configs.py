"""Shared source configuration paths for local dispatch and the SSH worker."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / 'sp_vision_ros/configs'
PROFILES = ('standard3', 'standard4', 'standard3_high_rate')


def resolve_config(value, default, suffix):
    path = Path(value).expanduser() if value else Path(default)
    if not path.is_absolute():
        path = ROOT / path
    path = path.resolve()
    if not path.is_relative_to(ROOT) or path.suffix != suffix or not path.is_file():
        raise ValueError(f'Expected an existing workspace {suffix} file: {path}')
    if any(part in {'artifacts', 'target', 'install', '.runtime', '.git', '.venv'}
           or part.startswith('build') for part in path.relative_to(ROOT).parts):
        raise ValueError(f'Configuration must be in a synchronized source directory: {path}')
    return path
