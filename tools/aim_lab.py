#!/usr/bin/env python3
"""Godot + real SP pixels closed loop: remote_run.py, or explicit --local."""
import argparse
import hashlib
import gzip
import json
import math
import os
from pathlib import Path
import shutil
import signal
import re
import statistics
import subprocess
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def performance_summary(report, vision, packets, engine_log):
    """Keep renderer, completed vision, and requested control rates distinct."""
    def percentile(values, fraction):
        if not values:
            return None
        values=sorted(values)
        offset=(len(values)-1)*fraction
        low=int(offset)
        return values[low]+(values[min(low+1,len(values)-1)]-values[low])*(offset-low)
    render=[float(v) for v in re.findall(r'^Project FPS: ([0-9.]+)',engine_log,re.M)]
    device=next((line for line in engine_log.splitlines() if 'Using Device' in line),'')
    software=bool(re.search(r'llvmpipe|lavapipe|softpipe|swiftshader',device,re.I)) if device else None
    # Stream full state records: unlimited sessions can produce large histories.
    ages=[]
    previous=None
    sim_elapsed=0
    state_count=0
    for packet in packets:
        if packet.get('event')=='plan' and packet.get('control'):
            ages.append(1000*packet['observation_age_s'])
        if packet.get('type')=='state':
            current=(packet.get('publisher_id'),packet.get('session_id'),packet['sim_time'])
            if previous is not None and current[:2]==previous[:2]:
                sim_elapsed+=max(0,current[2]-previous[2])
            previous=current
            state_count+=1
    processing=[p['processing_ms'] for p in vision if p.get('event')=='vision']
    wall=report.get('wall_seconds',0)
    return {'vision_processed_fps':report.get('frames',0)/wall if wall>0 else None,
            'render_fps_median':statistics.median(render) if render else None,
            'render_fps_samples':len(render),'renderer_device':device or None,
            'software_renderer':software,'processing_ms_p50':percentile(processing,.5),
            'processing_ms_p95':percentile(processing,.95),
            'observation_age_ms_p50':percentile(ages,.5),'observation_age_ms_p95':percentile(ages,.95),
            'simulation_seconds_per_wall_second':sim_elapsed/wall if state_count>1 and wall>0 else None,
            'wall_seconds':wall,'note':'Whole-run metrics include startup and post-kill frames; 100 Hz is the requested control cadence, not measured render/vision FPS.'}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--local', action='store_true', help='Explicitly run on this computer instead of the SSH worker')
    p.add_argument('--scenario', choices=['static','spin','move'], default='static')
    p.add_argument('--seconds', type=float, default=0, help='Run duration in seconds; 0 or omitted runs until stopped')
    p.add_argument('--max-fps', type=int, default=0, help='Maximum render FPS; 0 or omitted means uncapped')
    p.add_argument('--aim-config', default='sp_vision_ros/configs/standard3.yaml')
    p.add_argument('--sim-config', default='rm_aim_sim/configs/default.cfg')
    p.add_argument('--health', type=int, help='Explicit test override; default preserves competition preset')
    p.add_argument('--output', default='aim-lab')
    args = p.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds<0 or args.max_fps<0 or (args.health is not None and not 1 <= args.health <= 10000):
        p.error('Duration and max FPS must be nonnegative; health must be in [1,10000]')
    return args


def engine_command(engine, config, max_fps):
    return [str(engine),'--disable-vsync','--max-fps',str(max_fps),'--print-fps',
            '--rendering-method','mobile','--rendering-driver','vulkan',
            '--path',str(ROOT/'rm_aim_sim'),'--','--remote','--config='+str(config)]


def wait_for_run(aim, sim, stop):
    try:
        while aim.poll() is None:
            sim_code=sim.poll()
            if sim_code is not None:
                stop(aim)
                if sim_code:
                    raise RuntimeError('Godot exited with status '+str(sim_code))
                return aim.returncode,'window_closed'
            time.sleep(.1)
    except KeyboardInterrupt:
        stop(aim)
        return aim.returncode,'interrupted'
    return aim.returncode,'completed'


def jsonl_records(path):
    with path.open() as source:
        for line in source:
            yield json.loads(line)


def main():
    args=parse_args()
    if not args.local and not (ROOT/'.zimiao-remote-worker').is_file():
        raise RuntimeError('Use tools/remote_run.py closed-loop, or pass --local to explicitly run on this computer.')
    if not os.environ.get('DISPLAY'):
        raise RuntimeError('Real camera requires DISPLAY. Use a graphical desktop or xvfb-run; headless has no pixels.')
    out = ROOT/('artifacts/local-aim' if args.local else 'sp_vision_ros/artifacts')
    out.mkdir(parents=True,exist_ok=True)
    name = Path(args.output).name+'-'+args.scenario
    prefix = out/name
    config = prefix.with_suffix('.cfg')
    channel = 'sp_'+uuid.uuid4().hex
    engine = ROOT/'rm_aim_sim/.tools/godot'
    binary = ROOT/'sp_vision_ros/build-ros-ascii/sp_sim_ipc'
    if not engine.is_file():
        raise RuntimeError('Godot missing: run python3 rm_aim_sim/tools/setup_godot.py first.')
    if not binary.is_file():
        raise RuntimeError('SP client missing: build with bash sp_vision_sim/build.sh first.')
    text = (ROOT/args.sim_config).read_text()
    # ConfigFile allows a section once; change existing keys rather than appending duplicates.
    def replace(key, value):
        nonlocal text
        text, count = re.subn(r'^'+re.escape(key)+r'=.*$',key+'='+value,text,flags=re.M)
        if count != 1: raise RuntimeError('Config requires one '+key+' field')
    replace('channel','"'+channel+'"')
    replace('translate',str(args.scenario=='move').lower())
    replace('rotate',str(args.scenario=='spin').lower())
    if args.health is not None: replace('max_health',str(args.health))
    config.write_text(text)
    provenance = {
        'aim_config': str(Path(args.aim_config)),
        'sim_config': str(Path(args.sim_config)),
        'aim_config_sha256':hashlib.sha256((ROOT/args.aim_config).read_bytes()).hexdigest(),
        'effective_sim_config':text,
        'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
        'engine_sha256':hashlib.sha256(engine.read_bytes()).hexdigest(),
        'upstream':(ROOT/'rm_aim_sim/UPSTREAM.md').read_text(),
        'scenario':args.scenario,'seconds':args.seconds,'max_fps':args.max_fps,'execution':'local' if args.local else 'remote',
    }
    Path(str(prefix)+'-provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    # A cancelled startup must not reuse the previous run's score or screenshot.
    for suffix in ['-summary.json','-result.json','-first.png']:
        Path(str(prefix)+suffix).unlink(missing_ok=True)
    sim = aim = None
    handles = []
    def stop(child):
        if child and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try: child.wait(timeout=8)
            except subprocess.TimeoutExpired: child.kill();child.wait()
    def interrupted(signum, frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    signal.signal(signal.SIGINT,interrupted)
    try:
        handle=(out/(name+'-godot.log')).open('w');handles.append(handle)
        sim = subprocess.Popen(engine_command(engine,config,args.max_fps),
            stdout=handle,stderr=subprocess.STDOUT,cwd=ROOT)
        state_path=Path('/dev/shm/rm_aim_sim_'+channel)/'state.json'
        deadline=time.monotonic()+20
        while not state_path.exists():
            if sim.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Godot did not publish; see '+str(handle.name))
            time.sleep(.05)
        handle=(out/(name+'-sp.log')).open('w');handles.append(handle)
        aim=subprocess.Popen([str(binary),str(ROOT/args.aim_config),channel,str(args.seconds),str(prefix)],
            stdout=handle,stderr=subprocess.STDOUT,cwd=ROOT)
        code,termination=wait_for_run(aim,sim,stop)
        report_path=Path(str(prefix)+'-summary.json')
        if not report_path.exists():
            if termination in ('window_closed','interrupted'):
                print('Stopped before SP produced a report; see '+str(prefix)+'-sp.log',flush=True)
                return 0
            raise RuntimeError('SP exited without a report; see '+str(prefix)+'-sp.log')
        report=json.loads(report_path.read_text())
        final=report.get('final',{})
        stats=final.get('stats',{})
        first_shot=None
        states_path=Path(str(prefix)+'-states.jsonl')
        # Only the final session belongs to the reported health/death time (R resets it).
        for packet in jsonl_records(states_path):
            if packet.get('session_id')!=final.get('session_id'):
                continue
            for event in packet.get('events',[]):
                if event['type']=='shot':
                    first_shot=event['time'] if first_shot is None else min(first_shot,event['time'])
        death=final.get('health',{}).get('death_time',-1)
        ttk=death-first_shot if death>=0 and first_shot is not None else None
        summary={'ttk_first_shot_s':ttk,'first_shot_sim_time':first_shot,
            'death_sim_time':death,'exit_code':code,'termination':termination,'scenario':args.scenario,'frames':report.get('frames'),
            'errors':report.get('errors'),'stats':stats,'health':final.get('health'),
            'hit_per_fired':stats.get('hit',0)/max(1,stats.get('fired',0)),
            'inference_num_threads':report.get('inference_num_threads'),
            'opencv_threads':report.get('opencv_threads'),
            'source':'real RGB -> SP YOLO -> PnP -> Tracker -> Planner -> local IPC'}
        summary['performance']=performance_summary(report,
            jsonl_records(Path(str(prefix)+'-vision.jsonl')),jsonl_records(states_path),
            (out/(name+'-godot.log')).read_text(errors='replace'))
        Path(str(prefix)+'-result.json').write_text(json.dumps(summary,indent=2)+'\n')
        # Keep full per-frame/per-shot evidence without accumulating large local logs.
        for suffix in ['-states.jsonl','-vision.jsonl']:
            raw=Path(str(prefix)+suffix)
            with raw.open('rb') as source, gzip.open(str(raw)+'.gz','wb',compresslevel=1) as compressed:
                shutil.copyfileobj(source,compressed)
            raw.unlink()
        print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
        if code: return code
        if termination in ('window_closed','interrupted'): return 0
        return 0 if stats.get('hit',0)>0 else 2
    finally:
        stop(aim);stop(sim)
        for handle in handles: handle.close()
        config.unlink(missing_ok=True)
        # This unique channel has no live publisher after wait(); remove only its own mailbox.
        shutil.rmtree('/dev/shm/rm_aim_sim_'+channel,ignore_errors=True)

if __name__=='__main__':
    raise SystemExit(main())
