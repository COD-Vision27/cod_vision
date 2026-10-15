#!/usr/bin/env python3
"""Derive the actual robot-camera geometry from the simulator, without touching SP."""
import argparse
import hashlib
import json
import math
import re
import struct
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ALIGN = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
OPTICAL = np.diag([1., -1., -1.])

def glb(path):
    data = path.read_bytes()
    if data[:4] != b'glTF':
        raise ValueError('Not a GLB')
    size, kind = struct.unpack_from('<II', data, 12)
    if kind != 0x4e4f534a:
        raise ValueError('Missing GLB JSON')
    return json.loads(data[20:20+size])

def quat_matrix(q):
    x,y,z,w = np.array(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def derive(simulator=ROOT/'bevy_robomaster_simulator', sim_config=None):
    doc = glb(simulator/'assets/vehicle.glb')
    nodes = {n.get('name'):n for n in doc['nodes']}
    view, shot = nodes['CAM_DIRECTION'], nodes['SHOT_DIRECTION']
    # update_camera_follow ignores CAM_DIRECTION rotation: it uses SHOT_DIRECTION * Rx(pi/2).
    rx90 = np.array([[1.,0,0], [0,0,-1.], [0,1.,0]])
    basis = quat_matrix(shot['rotation']) @ rx90
    t = ALIGN @ basis.T @ (np.array(view['translation'])-np.array(shot['translation']))
    r = ALIGN @ OPTICAL
    # Read only the numeric scalar values we need; no TOML dependency is necessary.
    sections, section = {}, ''
    selected = sim_config or ROOT/'sp_vision_ros/configs/simulator.toml'
    for line in Path(selected).read_text().splitlines():
        clean = line.split('#',1)[0].strip()
        if clean.startswith('['):
            section = clean.strip('[]')
        elif '=' in clean:
            key,value = clean.split('=',1); sections[(section,key.strip())] = value.strip()
    width = int(sections.get(('capture.color','width'),'1440'))
    height = int(sections.get(('capture.color','height'),'1080'))
    fov = float(sections.get(('camera','fov'),'45'))
    speed = float(sections.get(('projectile','speed'),'25'))
    if width <= 0 or height <= 0 or not 0 < fov < 180:
        raise ValueError('Invalid simulator camera size/FOV')
    if not 10 <= speed <= 25:
        raise ValueError('Unmodified SP Planner accepts 10..25 m/s; choose a matching simulator projectile speed')
    # Bevy stores FOV as f32; reproduce its quantization before computing intrinsics.
    fov_rad = float(np.float32(np.float32(fov)*np.float32(math.pi/180)))
    focal = height/(2*math.tan(fov_rad/2))
    k = np.array([[focal,0,width/2],[0,focal,height/2],[0,0,1.]])
    return dict(R_camera2gimbal=r.tolist(), t_camera2gimbal=t.tolist(), camera_matrix=k.tolist(),
                width=width,height=height,fov_y_degrees=fov,bullet_speed=speed,
                camera_marker_bevy=view['translation'],muzzle_marker_bevy=shot['translation'],
                shot_rotation_xyzw=shot['rotation'],gun_basis_bevy=basis.tolist(),
                origin='muzzle (same launch origin as projectile_launch)',
                vehicle_sha256=hashlib.sha256((simulator/'assets/vehicle.glb').read_bytes()).hexdigest())

def generate(out, base='standard3', *, force=False, sim_config=None):
    out = Path(out)
    if out.exists() and not force:
        raise FileExistsError(f'{out} already exists; use --force only to deliberately replace tuning')
    g = derive(sim_config=sim_config)
    text = (ROOT/'sp_vision_25/configs'/f'{base}.yaml').read_text()
    # Keep the algorithm parameters, omit all hardware transport sections.
    text = text.split('#####-----cboard')[0]
    updates = {'enemy_color':'"blue"','device':'CPU','yaw_offset':'0','pitch_offset':'0',
               'high_speed_delay_time':'0.0','low_speed_delay_time':'0.0',
               'camera_matrix':json.dumps(np.array(g['camera_matrix']).flatten().tolist()),
               'distort_coeffs':'[0, 0, 0, 0, 0]',
               'R_camera2gimbal':json.dumps(np.array(g['R_camera2gimbal']).flatten().tolist()),
               't_camera2gimbal':json.dumps(g['t_camera2gimbal'])}
    for key,val in updates.items():
        text = re.sub(rf'^{key}:.*$',f'{key}: {val}',text,flags=re.M)
    text = '\n'.join(line for line in text.splitlines() if not line.startswith('# 相机同') and not line.startswith('# 标定') and not line.startswith('# 重投影'))
    # Remove identical duplicated confidence entry present in standard4.
    seen = False; lines=[]
    for line in text.splitlines():
        if line.startswith('min_confidence:'):
            if seen: continue
            seen=True
        lines.append(line)
    text='\n'.join(lines)+'\n'
    text += f'''\n# Generated from simulator geometry; model paths use the ROS sp_vision_root parameter.
fire_thresh: {0.0035 if base == 'standard3' else 0.003}
max_yaw_acc: 50
Q_yaw: [9e6, 0]
R_yaw: [1]
max_pitch_acc: 100
Q_pitch: [9e6, 0]
R_pitch: [1]
sim_width: {g['width']}
sim_height: {g['height']}
sim_bullet_speed: {g['bullet_speed']}
image_topic: /image_raw
camera_info_topic: /camera_info
frame_tf_topic: /sp_vision/sim/frame_tf
plan_topic: /sp_vision/sim/plan
diagnostics_topic: /sp_vision/sim/diagnostics
plan_period_ms: 10
max_frame_age_ms: 1000
target_timeout_ms: 250
command_timeout_s: 0.25
allow_fire: true
'''
    out.parent.mkdir(parents=True,exist_ok=True); out.write_text(text)
    geometry = out.parent/'geometry.json'
    if force or not geometry.exists():
        geometry.write_text(json.dumps(g,indent=2)+'\n')
    # simulator.toml is maintained by the user, never overwritten by generation.
    return g

def refresh_geometry(out, sim_config=None):
    """Refresh camera/ballistic geometry without resetting algorithm tuning."""
    out = Path(out)
    text = out.read_text()
    g = derive(sim_config=sim_config)
    updates = {key: np.array(g[key]).flatten().tolist()
               for key in ['camera_matrix', 'R_camera2gimbal', 't_camera2gimbal']}
    updates.update(distort_coeffs=[0, 0, 0, 0, 0], sim_width=g['width'],
                   sim_height=g['height'], sim_bullet_speed=g['bullet_speed'])
    for key, value in updates.items():
        text, count = re.subn(rf'^{key}:.*$', f'{key}: {json.dumps(value)}', text, flags=re.M)
        if count != 1:
            raise ValueError(f'Expected exactly one {key} in {out}')
    out.write_text(text)
    (out.parent/'geometry.json').write_text(json.dumps(g,indent=2)+'\n')
    return g

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--base',choices=['standard3','standard4'],default='standard3')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--sim-config',type=Path,default=ROOT/'sp_vision_ros/configs/simulator.toml')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--force',action='store_true',help='Deliberately replace existing algorithm YAML and geometry metadata')
    mode.add_argument('--geometry-only',action='store_true',help='Update only geometry; preserve all algorithm tuning')
    args=parser.parse_args()
    output=args.output or ROOT/'sp_vision_ros/configs'/f'{args.base}.yaml'
    result=refresh_geometry(output,args.sim_config) if args.geometry_only else generate(output,args.base,force=args.force,sim_config=args.sim_config)
    print(json.dumps(result,indent=2))
