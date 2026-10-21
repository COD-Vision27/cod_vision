#!/usr/bin/env python3
"""Run on the SSH target; show the simulation in a local browser or target desktop."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
import urllib.request
import webbrowser
from sim_configs import CONFIGS, PROFILES, resolve_config

ROOT = Path(__file__).resolve().parents[1]
MARKER = 'ZIMIAO_SIM_VIEW '


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view', choices=['local', 'remote'], default='local')
    parser.add_argument('--scenario', choices=['static', 'spin', 'move'], help='Override scenario.mode in simulator Godot CFG')
    parser.add_argument('--base', '--profile', choices=PROFILES, default='standard3')
    parser.add_argument('--aim-config', help='Custom YAML path relative to the workspace root')
    parser.add_argument('--sim-config', help='Custom simulator Godot CFG path relative to the workspace root')
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--host', default=os.environ.get('ZIMIAO_REMOTE_HOST', 'wuxian-remote'))
    parser.add_argument('--user', default=os.environ.get('ZIMIAO_REMOTE_USER', 'wuxian'))
    parser.add_argument('--port', type=int, help='SSH port override; otherwise honor ~/.ssh/config')
    parser.add_argument('--identity')
    parser.add_argument('--local-port', type=int, default=6080)
    parser.add_argument('--no-open', action='store_true', help='Print browser URL without opening it')
    parser.add_argument('--verbose', action='store_true', help='Stream all simulator/algorithm diagnostics')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 86400 or not 1 <= args.local_port <= 65535 or (args.port is not None and not 1 <= args.port <= 65535):
        parser.error('Invalid duration or port')
    if any(not re.fullmatch(r'[a-zA-Z0-9_.-]+', value) for value in (args.host, args.user)):
        parser.error('Invalid host/user')
    aim_config = resolve_config(args.aim_config, CONFIGS / f'{args.base}.yaml', '.yaml')
    sim_config = resolve_config(args.sim_config, ROOT / 'rm_aim_sim/configs/default.cfg', '.cfg')
    if args.view == 'local':
        with socket.socket() as probe:
            try:
                probe.bind(('127.0.0.1', args.local_port))
            except OSError:
                parser.error('Local viewer port is busy; choose another --local-port (e.g. 6081)')
    common = ['--host', args.host, '--user', args.user]
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
           '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3']
    if args.port is not None:
        common += ['--port', str(args.port)]
        ssh += ['-p', str(args.port)]
    if args.identity:
        common += ['--identity', args.identity]
        ssh += ['-i', str(Path(args.identity).expanduser())]
    command = [sys.executable, '-u', str(ROOT / 'tools/remote_run.py'), *common,
               'run', '--', 'python3', '-u', 'tools/sim_view_worker.py', '--view', args.view,
               '--base', args.base, '--seconds', str(args.seconds),
               '--aim-config', str(aim_config.relative_to(ROOT)),
               '--sim-config', str(sim_config.relative_to(ROOT))]
    if args.scenario:
        command += ['--scenario', args.scenario]
    scheduler = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, start_new_session=True)
    tunnel = None
    ready = False
    try:
        for line in scheduler.stdout:
            if args.verbose or any(word in line for word in (
                    'Remote job ', 'Synchronizing ', 'Running:', MARKER, 'AdapterInfo',
                    'Retrieving ', 'Local logs/results:', 'failed', 'Failed', 'ERROR', 'Error',
                    'Another remote job', 'Auto-aim exited')):
                print(line, end='', flush=True)
            if tunnel and tunnel.poll() is not None:
                raise RuntimeError('SSH display tunnel disconnected')
            if line.startswith(MARKER):
                state = json.loads(line[len(MARKER):])
                if args.view == 'remote':
                    print(f"仿真窗口将在目标机桌面 {state['display']} 显示。", flush=True)
                else:
                    tunnel = subprocess.Popen(ssh + ['-N', '-o', 'ExitOnForwardFailure=yes', '-L',
                        f'127.0.0.1:{args.local_port}:127.0.0.1:{int(state["web_port"])}',
                        f'{args.user}@{args.host}'])
                    url = f'http://127.0.0.1:{args.local_port}/vnc.html?autoconnect=1&resize=scale&reconnect=0'
                    deadline = time.monotonic() + 15
                    while True:
                        if tunnel.poll() is not None:
                            raise RuntimeError('SSH display tunnel failed')
                        try:
                            with urllib.request.urlopen(url, timeout=1) as response:
                                if response.status == 200:
                                    break
                        except OSError:
                            if time.monotonic() >= deadline:
                                raise RuntimeError('Local browser viewer did not become ready')
                            time.sleep(0.2)
                    print(f'本地查看目标机仿真：{url}', flush=True)
                    if not args.no_open:
                        webbrowser.open(url)
                ready = True
        status = scheduler.wait()
        if status == 0 and not ready:
            raise RuntimeError('Remote command finished without a visible display')
        return status
    finally:
        if scheduler.poll() is None:
            # Let the scheduler close its SSH protocol and retrieve final logs.
            scheduler.send_signal(signal.SIGINT)
            try:
                remaining, _ = scheduler.communicate(timeout=35)
                if remaining:
                    print(remaining, end='', flush=True)
            except subprocess.TimeoutExpired:
                os.killpg(scheduler.pid, signal.SIGKILL)
                scheduler.wait()
        if tunnel and tunnel.poll() is None:
            tunnel.terminate()
            try:
                tunnel.wait(timeout=5)
            except subprocess.TimeoutExpired:
                tunnel.kill()
                tunnel.wait()


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('已停止远端仿真和本地查看连接。', file=sys.stderr)
        raise SystemExit(130)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'Visible remote simulation failed: {exc}', file=sys.stderr)
        raise SystemExit(1)
