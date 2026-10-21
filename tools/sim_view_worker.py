#!/usr/bin/env python3
"""Remote-only visible simulator session, called through remote_run.py."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from sim_configs import CONFIGS, PROFILES, resolve_config

ROOT = Path(__file__).resolve().parents[1]
MARKER = "ZIMIAO_SIM_VIEW "


def desktop_environment():
    candidates = []
    # The SSH environment normally lacks the logged-in desktop's DISPLAY.
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            if proc.joinpath('comm').read_text().strip() not in {'gnome-shell', 'gnome-session-b'}:
                continue
            values = dict(item.split(b'=', 1) for item in proc.joinpath('environ').read_bytes().split(b'\0') if b'=' in item)
            candidates.append({key: values[key.encode()].decode() for key in ('DISPLAY', 'XAUTHORITY') if key.encode() in values})
        except (OSError, ValueError):
            continue
    if os.environ.get('DISPLAY'):
        candidates.insert(0, {key: os.environ[key] for key in ('DISPLAY', 'XAUTHORITY') if key in os.environ})
    for auth in (Path(f'/run/user/{os.getuid()}/gdm/Xauthority'), Path.home() / '.Xauthority'):
        if auth.is_file():
            candidates.extend({'DISPLAY': ':' + sock.name[1:], 'XAUTHORITY': str(auth)}
                              for sock in Path('/tmp/.X11-unix').glob('X[0-9]*'))
    for candidate in candidates:
        if not candidate.get('DISPLAY'):
            continue
        env = dict(os.environ, **candidate)
        probe = subprocess.run(['xdotool', 'getdisplaygeometry'], env=env, capture_output=True, timeout=5)
        if probe.returncode == 0:
            return env
    raise RuntimeError('No accessible target desktop. Log in to the target graphical desktop, or use --view local.')


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_port(port, process):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'Viewer service exited with status {process.returncode}')
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=0.2):
                return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError(f'Viewer service did not start on loopback port {port}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view', choices=['local', 'remote'], required=True)
    parser.add_argument('--scenario', choices=['static', 'spin', 'move'])
    parser.add_argument('--base', choices=PROFILES, default='standard3')
    parser.add_argument('--aim-config')
    parser.add_argument('--sim-config')
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--inside-display', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 86400:
        parser.error('--seconds must be between 10 and 86400')
    for path in (ROOT / 'rm_aim_sim/.tools/godot',
                 ROOT / 'sp_vision_ros/build-ros-ascii/sp_sim_ipc'):
        if not path.is_file():
            raise RuntimeError('Remote build missing. Run: python3 tools/remote_run.py build')
    if args.view == 'local' and not args.inside_display:
        os.execvp('xvfb-run', ['xvfb-run', '-a', '-s', '-screen 0 1280x800x24 -nolisten tcp',
                             sys.executable, '-u', str(Path(__file__).resolve()), *sys.argv[1:], '--inside-display'])
    env = desktop_environment() if args.view == 'remote' else dict(os.environ)
    aim_config = resolve_config(args.aim_config, CONFIGS / f'{args.base}.yaml', '.yaml')
    sim_config = resolve_config(args.sim_config, ROOT / 'rm_aim_sim/configs/default.cfg', '.cfg')
    env['SP_AIM_CONFIG'] = str(aim_config)
    env['SP_SIM_CONFIG'] = str(sim_config)
    children = []
    handles = []
    log_dir = ROOT / 'sp_vision_ros/artifacts'
    log_dir.mkdir(exist_ok=True)
    selected = {kind: {'path': str(path.relative_to(ROOT)),
                      'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                for kind, path in [('aim', aim_config), ('simulator', sim_config)]}
    (log_dir / 'active-configs.json').write_text(json.dumps(selected, indent=2) + '\n')
    (log_dir / 'active-aim.yaml').write_bytes(aim_config.read_bytes())
    (log_dir / 'active-simulator.cfg').write_bytes(sim_config.read_bytes())

    def start(argv, name):
        handle = (log_dir / name).open('w')
        handles.append(handle)
        child = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        children.append(child)
        return child

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        port = None
        if args.view == 'local':
            vnc_port = free_port()
            vnc = start(['x11vnc', '-display', env['DISPLAY'], '-localhost', '-listen', '127.0.0.1', '-no6',
                         '-nopw', '-forever', '-shared',
                         '-rfbport', str(vnc_port), '-noxdamage', '-quiet'], 'viewer-vnc.log')
            wait_port(vnc_port, vnc)
            port = free_port()
            web = start(['websockify', '--web=/usr/share/novnc', f'127.0.0.1:{port}',
                         f'127.0.0.1:{vnc_port}'], 'viewer-web.log')
            wait_port(port, web)
        print(MARKER + json.dumps({'view': args.view, 'display': env['DISPLAY'], 'web_port': port,
                                  'scenario': args.scenario, 'base': args.base,
                                  'configs': selected}), flush=True)
        # Keep simulator/algorithm output in the scheduler log, including the GPU adapter.
        pair = subprocess.Popen([sys.executable, 'tools/aim_lab.py', '--seconds', str(args.seconds),
                                 '--scenario', args.scenario or 'static', '--aim-config',str(aim_config),
                                 '--sim-config',str(sim_config)],
                                cwd=ROOT, env=env, start_new_session=True)
        children.append(pair)
        while pair.poll() is None:
            if any(child.poll() is not None for child in children[:-1]):
                raise RuntimeError('Viewer service stopped before the simulation finished; see viewer logs.')
            time.sleep(0.2)
        return pair.returncode
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'Visible remote simulation failed: {exc}', file=sys.stderr)
        raise SystemExit(1)
