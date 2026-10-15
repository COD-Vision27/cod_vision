#!/usr/bin/env python3
"""Test the restored simulator without changing its source, assets or config."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'bevy_robomaster_simulator'
OUT = ROOT / 'sp_vision_ros/artifacts'

def hashes():
    paths = [SIM / 'Cargo.toml', SIM / 'Cargo.lock', SIM / 'config.toml']
    for folder in ['src', 'crates', 'rm_interfaces', 'assets']:
        paths.extend(p for p in (SIM/folder).rglob('*') if p.is_file())
    return {str(p.relative_to(SIM)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths)}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--seconds', type=float, default=45)
    parser.add_argument('--jobs', type=int, default=8)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    before = hashes()
    report = {'variant': 'restored-original-no-ros', 'hostname': os.uname().nodename,
              'seconds': args.seconds, 'source_before': before, 'steps': []}
    env = os.environ.copy()
    env['PATH'] = str(Path.home()/'.cargo/bin')+':'+env['PATH']
    for key in ['DAEDALUS_SP_SCENARIO', 'DAEDALUS_AUTO_AIM', 'DAEDALUS_METRICS_PATH']:
        env.pop(key, None)
    try:
        if not args.skip_build:
            for cmd in [['cargo','test','--locked','--no-default-features','--bin','daedalus','-j',str(args.jobs)],
                        ['cargo','build','--locked','--no-default-features','--bin','daedalus','-j',str(args.jobs)]]:
                print('Running:', ' '.join(cmd), flush=True)
                r = subprocess.run(cmd, cwd=SIM, env=env)
                report['steps'].append({'command':cmd, 'exit_code':r.returncode})
                if r.returncode:
                    raise RuntimeError('Original simulator build/tests failed')
        for key, cmd in [('gpu',['nvidia-smi']),('vulkan',['vulkaninfo','--summary'])]:
            r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30)
            report[key] = {'exit_code':r.returncode, 'output':r.stdout+r.stderr}
        rustlib = subprocess.check_output(['rustc','--print','target-libdir'], env=env, text=True).strip()
        env['LD_LIBRARY_PATH'] = str(SIM/'target/debug/deps')+':'+rustlib+':'+env.get('LD_LIBRARY_PATH','')
        env['BEVY_ASSET_ROOT'] = str(SIM)
        env['RUST_LOG'] = 'daedalus=info,bevy_diagnostic=info,bevy_render=info,wgpu=warn'
        # Read the untouched original configuration in the original directory.
        cmd = [str(SIM/'target/debug/daedalus'),'--duration-secs',str(args.seconds)]
        prefix = [] if env.get('DISPLAY') or env.get('WAYLAND_DISPLAY') else ['xvfb-run','-a']
        report['launch'] = prefix+cmd
        log_path = OUT/'baseline_simulator.log'
        lines = []
        started = time.monotonic()
        child = subprocess.Popen(prefix+cmd,cwd=SIM,env=env,start_new_session=True,
                                 stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        def relay():
            with log_path.open('w') as log:
                for line in child.stdout:
                    lines.append((time.monotonic(),line)); log.write(line); log.flush()
                    print(line,end='',flush=True)
        reader = threading.Thread(target=relay,daemon=True);reader.start()
        screenshot_steps = 0
        try:
            while child.poll() is None:
                if time.monotonic()-started > args.seconds+240:
                    raise RuntimeError('Original scene run exceeded wall-clock timeout')
                loaded = next((t for t,l in lines if 'scene loaded' in l),None)
                if loaded and screenshot_steps < 3 and time.monotonic()-loaded>5*(screenshot_steps+1) and not prefix:
                    ids = subprocess.run(['xdotool','search','--pid',str(child.pid)],env=env,
                                         capture_output=True,text=True)
                    if ids.returncode==0 and ids.stdout.strip():
                        subprocess.run(['xdotool','windowfocus','--sync',ids.stdout.splitlines()[0]],
                                       env=env,capture_output=True,text=True,timeout=5)
                        if screenshot_steps:
                            subprocess.run(['xdotool','key','F3'],
                                           env=env,capture_output=True,text=True)
                            time.sleep(1)
                        key = subprocess.run(['xdotool','key','F2'],
                                             env=env,capture_output=True,text=True)
                        report['screenshot_key_exit'] = key.returncode
                    screenshot_steps+=1
                time.sleep(.25)
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            reader.join(timeout=10)
        text = ''.join(l for _,l in lines)
        report['run_exit_code'] = child.returncode
        report['scene_loaded'] = 'scene loaded' in text
        report['hardware_adapter'] = 'RTX 4060' in text
        report['fps_samples'] = [float(x) for x in re.findall(r'fps[^\n]*?\(avg\s+([\d.]+)',text)]
        report['scene_load_wall_seconds'] = next((t-started for t,l in lines if 'scene loaded' in l),None)
        report['wall_seconds'] = time.monotonic()-started
        report['screenshots'] = []
        for image in SIM.glob('screenshot-*.png'):
            if image.stat().st_mtime >= time.time()-report['wall_seconds']-2:
                shutil.copy2(image,OUT/('baseline_'+image.name))
                report['screenshots'].append('baseline_'+image.name)
        report['status'] = 'PASS' if child.returncode==0 and report['scene_loaded'] and report['hardware_adapter'] else 'FAIL'
    except Exception as e:
        report['status'] = 'FAIL';report['error'] = str(e)
    finally:
        report['source_unchanged'] = hashes()==before
        if not report['source_unchanged']:report['status']='FAIL'
        (OUT/'baseline_result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ['source_before','gpu','vulkan']},indent=2),flush=True)
    return 0 if report['status']=='PASS' else 1

if __name__=='__main__':
    raise SystemExit(main())
