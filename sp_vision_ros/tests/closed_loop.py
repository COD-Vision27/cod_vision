#!/usr/bin/env python3
"""Run the actual Bevy render -> ROS -> SP neural/EKF/MPC -> ROS -> projectile loop."""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rm_interfaces.msg import SpGimbalPlan

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scenario',choices=['static','spin','move'],default='static')
    parser.add_argument('--seconds',type=float,default=25);args=parser.parse_args()
    artifacts=ROOT/'artifacts';artifacts.mkdir(exist_ok=True)
    metrics=artifacts/f'{args.scenario}_metrics.jsonl'
    env=os.environ.copy();env['DAEDALUS_SP_SCENARIO']=args.scenario;env['DAEDALUS_METRICS_PATH']=str(metrics)
    selected={}
    for key,default in [('SP_AIM_CONFIG','configs/standard3.yaml'),('SP_SIM_CONFIG','configs/simulator.toml')]:
        path=Path(env.get(key,default))
        if not path.is_absolute():path=ROOT/path
        path=path.resolve();env[key]=str(path)
        selected[key]={'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    env['RUST_LOG']='daedalus=info,bevy_render=info,bevy_diagnostic=info,wgpu=warn,bevy_gltf=warn'
    rclpy.init();node=rclpy.create_node('sp_closed_loop_observer')
    reports=[];plans=[];latest_image=[]
    subs=[node.create_subscription(String,'/sp_vision/sim/diagnostics',lambda m:reports.append(json.loads(m.data)),qos_profile_sensor_data),
          node.create_subscription(SpGimbalPlan,'/sp_vision/sim/plan',lambda m:plans.append(m),qos_profile_sensor_data),
          node.create_subscription(Image,'/image_raw',lambda m:latest_image.__setitem__(slice(None),[m]),qos_profile_sensor_data)]
    handles=[];children=[]
    try:
        start=time.monotonic()
        sim_log=(artifacts/f'{args.scenario}_sim.log').open('w');handles.append(sim_log)
        sim=subprocess.Popen(['bash',str(ROOT/'scripts/run_sim.sh'),'--duration-secs',str(args.seconds)],env=env,stdout=sim_log,stderr=subprocess.STDOUT)
        children.append(sim)
        # Let asynchronously loaded dynamic robots settle before observing their motion.
        warmup=5.0
        until=time.monotonic()+warmup
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            if sim.poll() is not None: raise RuntimeError('simulator exited during warmup')
        aim_log=(artifacts/f'{args.scenario}_aim.log').open('w');handles.append(aim_log)
        aim=subprocess.Popen(['bash',str(ROOT/'scripts/run_aim.sh')],env=env,stdout=aim_log,stderr=subprocess.STDOUT)
        children.insert(0,aim)
        deadline=start+args.seconds+90
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if children[0].poll() is not None: raise RuntimeError('SP node exited; inspect aim log')
            if children[1].poll() is not None: break
        else: raise RuntimeError('simulator exceeded startup/run timeout')
        if children[1].returncode!=0: raise RuntimeError(f'simulator exited {children[1].returncode}; inspect sim log')
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        stale_stop=bool(plans) and all(not p.control and not p.fire for p in plans[-10:])
        vision=[r for r in reports if r.get('event')=='vision'];controlled=[p for p in plans if p.control]
        events=[json.loads(line) for line in metrics.read_text().splitlines()] if metrics.exists() else []
        shots=[e for e in events if e.get('event')=='shot_fired'];hits=[e for e in events if e.get('event')=='armor_hit']
        enemy_hits={e['shot_id'] for e in hits if e.get('kind')=='projectile' and e.get('team')=='blue' and e.get('armor_label')==3}
        completed={e['shot_id']:e['outcome'] for e in events if e.get('event')=='shot_finished' and e.get('kind')=='projectile'}
        completed_hits=sum(outcome=='armor_hit' for outcome in completed.values())
        result={'scenario':args.scenario,'aim_config':env['SP_AIM_CONFIG'],'configs':selected,'warmup_seconds':warmup,'move_reference_amplitude_m':0.8 if args.scenario=='move' else None,'sim_exit':children[1].returncode,'wall_seconds':time.monotonic()-start,
                'vision_frames':len(vision),'frames_with_armors':sum(r['armors']>0 for r in vision),
                'controlled_plans':len(controlled),'fire_plans':sum(p.fire for p in plans),
                'shots':len(shots),'armor_hits':len(hits),'enemy_hit_shots':len(enemy_hits),
                'completed_shots':len(completed),'completed_armor_hit_shots':completed_hits,
                'completed_hit_rate':completed_hits/len(completed) if completed else None,
                'pending_shots':len(shots)-len(completed),'stale_stop':stale_stop,
                'max_yaw_acc':max((abs(p.yaw_acc) for p in controlled),default=0),
                'max_pitch_acc':max((abs(p.pitch_acc) for p in controlled),default=0)}
        result['status']='PASS' if vision and controlled and shots and enemy_hits and stale_stop else 'FAIL'
        (artifacts/f'{args.scenario}_result.json').write_text(json.dumps(result,indent=2)+'\n')
        with (artifacts/f'{args.scenario}_diagnostics.jsonl').open('w') as file:
            for report in reports:file.write(json.dumps(report)+'\n')
        if latest_image:
            try:
                import cv2,numpy as np
                im=latest_image[0];pixels=np.frombuffer(bytes(im.data),np.uint8).reshape(im.height,im.step)[:,:im.width*3].reshape(im.height,im.width,3)
                if im.encoding=='rgb8':pixels=cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(artifacts/f'{args.scenario}_last_frame.png'),pixels)
            except ImportError:pass
        print(json.dumps(result,indent=2))
        if result['status']!='PASS':raise SystemExit(1)
    finally:
        for child in children:
            if child.poll() is None:child.send_signal(signal.SIGINT)
        for child in children:
            try:child.wait(timeout=10)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
        for log in handles:log.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
