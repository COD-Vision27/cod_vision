#!/usr/bin/env python3
"""Long-run, conservative simulator accuracy gate; real rendered images only."""
import argparse,hashlib,json,math,os,shutil,subprocess,sys,time
from pathlib import Path
from sim_configs import CONFIGS, PROFILES
ROOT=Path(__file__).resolve().parents[1]
ART=ROOT/'sp_vision_ros/artifacts'

def wilson(k,n):
    if not n:return 0.0
    z=1.95996398454;p=k/n
    return (p+z*z/(2*n)-z*math.sqrt(p*(1-p)/n+z*z/(4*n*n)))/(1+z*z/n)

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=180);p.add_argument('--repeats',type=int,default=1)
    p.add_argument('--bases',nargs='+',choices=PROFILES,default=['standard3','standard4'])
    p.add_argument('--scenarios',nargs='+',choices=['static','spin','move'],default=['static','spin','move'])
    p.add_argument('--min-shots',type=int,default=100);p.add_argument('--min-hit-rate',type=float,default=.90)
    p.add_argument('--min-wilson-lower',type=float,default=.85);a=p.parse_args()
    if a.seconds<30 or a.repeats<1 or a.min_shots<1 or not 0<a.min_hit_rate<=1 or not 0<a.min_wilson_lower<=1:p.error('invalid acceptance limits')
    tag=time.strftime('%Y%m%d-%H%M%S');rows=[]
    source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'sp_vision_ros/src/ros_node.cpp',ROOT/'bevy_robomaster_simulator/src/ros2/plugin.rs',ROOT/'bevy_robomaster_simulator/src/systems/gimbal_pid.rs',CONFIGS/'simulator.toml',*[CONFIGS/f'{base}.yaml' for base in a.bases]]}
    for base in a.bases:
        for scenario in a.scenarios:
            shots=hits=0;valid=True;runs=[]
            for repeat in range(a.repeats):
                label=f'{base}-{scenario}-{repeat+1}'
                print(f'Accuracy case {label}: {a.seconds}s',flush=True)
                env=os.environ.copy();env['SP_AIM_CONFIG']=str(CONFIGS/f'{base}.yaml')
                before={p.name:p.stat().st_mtime_ns for p in ART.glob(f'{scenario}_*') if p.is_file()}
                cmd=[sys.executable,str(ROOT/'sp_vision_ros/tests/closed_loop.py'),'--scenario',scenario,'--seconds',str(a.seconds)]
                code=subprocess.call(cmd,env=env)
                result_path=ART/f'{scenario}_result.json'
                if code:valid=False
                result=json.loads(result_path.read_text()) if result_path.exists() and result_path.stat().st_mtime_ns!=before.get(result_path.name) else {'status':'FAIL','exit_code':code}
                shots+=result.get('shots',0);hits+=result.get('enemy_hit_shots',0)
                runs.append(result)
                for suffix in ['result.json','metrics.jsonl','diagnostics.jsonl','last_frame.png','aim.log','sim.log']:
                    source=ART/f'{scenario}_{suffix}'
                    if source.exists() and source.stat().st_mtime_ns!=before.get(source.name):shutil.copy2(source,ART/f'accuracy-{tag}-{label}-{suffix}')
            lower=wilson(hits,shots);rate=hits/shots if shots else 0
            passed=valid and shots>=a.min_shots and rate>=a.min_hit_rate and lower>=a.min_wilson_lower
            rows.append({'base':base,'scenario':scenario,'shots':shots,'unique_enemy_hit_shots':hits,
                'conservative_hit_rate':rate,'wilson_95_lower':lower,'status':'PASS' if passed else 'FAIL','runs':runs})
            report={'schema':'sp_sim_accuracy_v1','tag':tag,'thresholds':{'min_shots':a.min_shots,'min_hit_rate':a.min_hit_rate,'min_wilson_lower':a.min_wilson_lower},
                'source_hashes':source_hashes,'expected_cases':len(a.bases)*len(a.scenarios),'cases':rows,'status':'PASS' if len(rows)==len(a.bases)*len(a.scenarios) and all(r['status']=='PASS' for r in rows) else 'INCOMPLETE' if len(rows)<len(a.bases)*len(a.scenarios) else 'FAIL'}
            (ART/'accuracy_qualification.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps({k:v for k,v in rows[-1].items() if k!='runs'}),flush=True)
    return 0 if report['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
