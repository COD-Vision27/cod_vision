#!/usr/bin/env python3
"""Godot + real SP pixels closed loop: remote_run.py, or explicit --local."""
import argparse
import hashlib
import gzip
import json
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
    ages=[1000*p['observation_age_s'] for p in packets if p.get('event')=='plan' and p.get('control')]
    processing=[p['processing_ms'] for p in vision if p.get('event')=='vision']
    wall=report.get('wall_seconds',0)
    states=[p for p in packets if p.get('type')=='state']
    sim_elapsed=states[-1]['sim_time']-states[0]['sim_time'] if len(states)>1 else None
    return {'vision_processed_fps':report.get('frames',0)/wall if wall>0 else None,
            'render_fps_median':statistics.median(render) if render else None,
            'render_fps_samples':len(render),'renderer_device':device or None,
            'software_renderer':software,'processing_ms_p50':percentile(processing,.5),
            'processing_ms_p95':percentile(processing,.95),
            'observation_age_ms_p50':percentile(ages,.5),'observation_age_ms_p95':percentile(ages,.95),
            'simulation_seconds_per_wall_second':sim_elapsed/wall if sim_elapsed is not None and wall>0 else None,
            'wall_seconds':wall,'note':'Whole-run metrics include startup and post-kill frames; 100 Hz is the requested control cadence, not measured render/vision FPS.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--local', action='store_true', help='Explicitly run on this computer instead of the SSH worker')
    p.add_argument('--scenario', choices=['static','spin','move'], default='static')
    p.add_argument('--seconds', type=float, default=25)
    p.add_argument('--aim-config', default='sp_vision_ros/configs/standard3.yaml')
    p.add_argument('--sim-config', default='rm_aim_sim/configs/default.cfg')
    p.add_argument('--health', type=int, help='Explicit test override; default preserves competition preset')
    p.add_argument('--output', default='aim-lab')
    args = p.parse_args()
    if not 1 <= args.seconds <= 86400 or (args.health is not None and not 1 <= args.health <= 10000):
        p.error('Invalid duration/health')
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
        'scenario':args.scenario,'seconds':args.seconds,'execution':'local' if args.local else 'remote',
    }
    Path(str(prefix)+'-provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
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
        sim = subprocess.Popen([str(engine),'--disable-vsync','--max-fps','120','--print-fps','--rendering-method','mobile','--rendering-driver','vulkan',
            '--path',str(ROOT/'rm_aim_sim'),'--',
            '--remote','--config='+str(config)],stdout=handle,stderr=subprocess.STDOUT,cwd=ROOT)
        state_path=Path('/dev/shm/rm_aim_sim_'+channel)/'state.json'
        deadline=time.monotonic()+20
        while not state_path.exists():
            if sim.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Godot did not publish; see '+str(handle.name))
            time.sleep(.05)
        handle=(out/(name+'-sp.log')).open('w');handles.append(handle)
        aim=subprocess.Popen([str(binary),str(ROOT/args.aim_config),channel,str(args.seconds),str(prefix)],
            stdout=handle,stderr=subprocess.STDOUT,cwd=ROOT)
        code=aim.wait()
        report=json.loads(Path(str(prefix)+'-summary.json').read_text()) if Path(str(prefix)+'-summary.json').exists() else {}
        final=report.get('final',{})
        stats=final.get('stats',{})
        events = {}
        packets=[]
        for line in Path(str(prefix)+'-states.jsonl').read_text().splitlines():
            packet=json.loads(line)
            packets.append(packet)
            for event in packet.get('events',[]):
                events[(packet['session_id'],event['type'],event.get('id'))]=event
        shots=[e['time'] for e in events.values() if e['type']=='shot']
        death=final.get('health',{}).get('death_time',-1)
        first_shot=min(shots) if shots else None
        ttk=death-first_shot if death>=0 and first_shot is not None else None
        summary={'ttk_first_shot_s':ttk,'first_shot_sim_time':first_shot,
            'death_sim_time':death,'exit_code':code,'scenario':args.scenario,'frames':report.get('frames'),
            'errors':report.get('errors'),'stats':stats,'health':final.get('health'),
            'hit_per_fired':stats.get('hit',0)/max(1,stats.get('fired',0)),
            'inference_num_threads':report.get('inference_num_threads'),
            'opencv_threads':report.get('opencv_threads'),
            'source':'real RGB -> SP YOLO -> PnP -> Tracker -> Planner -> local IPC'}
        vision=[json.loads(line) for line in Path(str(prefix)+'-vision.jsonl').read_text().splitlines()]
        summary['performance']=performance_summary(report,vision,packets,
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
        return 0 if stats.get('hit',0)>0 else 2
    finally:
        stop(aim);stop(sim)
        for handle in handles: handle.close()
        config.unlink(missing_ok=True)
        # This unique channel has no live publisher after wait(); remove only its own mailbox.
        shutil.rmtree('/dev/shm/rm_aim_sim_'+channel,ignore_errors=True)

if __name__=='__main__':
    raise SystemExit(main())
