#!/usr/bin/env python3
"""Audit tracked files for private paths, secret files and optional private tokens."""
import argparse
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--private-patterns', type=Path,
                        help='Ignored JSON string list containing machine-specific tokens to forbid')
    args = parser.parse_args()
    tokens = json.loads(args.private_patterns.read_text()) if args.private_patterns else []
    names = subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode().split('\0')
    findings = []
    private_path = re.compile(r'/(?:home|Users)/[^/\s"\x27]+/')
    key_header = re.compile(r'-----BEGIN (?:OPENSSH |RSA |EC |DSA )?PRIVATE KEY-----')
    for name in filter(None, names):
        path = ROOT / name
        parts = Path(name).parts
        if any(p in {'.local', '.ssh', '.aws', '.venv', 'CMakeFiles', 'artifacts'} or p.startswith('build-') for p in parts):
            findings.append((name, 'private/generated directory tracked'))
        if path.suffix in {'.pem', '.key'} or '.local.' in path.name or path.name == '.env':
            findings.append((name, 'private configuration tracked'))
        if path.is_symlink() or not path.is_file():
            continue
        data = path.read_bytes()
        if b'\0' in data:
            continue
        try:
            text = data.decode('utf-8')
        except UnicodeDecodeError:
            continue
        if private_path.search(text) or key_header.search(text):
            findings.append((name, 'personal path or private key'))
        if any(token and token.casefold() in text.casefold() for token in tokens):
            findings.append((name, 'private token (value redacted)'))
    for name, reason in findings:
        print(f'{name}: {reason}')
    print(f'Public tree audit: {"FAIL" if findings else "PASS"}')
    return bool(findings)


if __name__ == '__main__':
    raise SystemExit(main())
