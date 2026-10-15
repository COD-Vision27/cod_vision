#!/usr/bin/env python3
"""Isolated build, measured calibration, versioned deployment and rollback."""
import argparse,fcntl,datetime,hashlib,json,os,platform,re,shlex,shutil,signal,subprocess,sys,time,uuid
from pathlib import Path
if __name__=='__main__' and sys.argv[1:]==['deps']:
    subprocess.run(['sudo','apt-get','update'],check=True)
    subprocess.run(['sudo','apt-get','install','-y','build-essential','cmake','pkg-config','libusb-1.0-0-dev',
        'libopencv-dev','libeigen3-dev','libfmt-dev','libspdlog-dev','libyaml-cpp-dev','nlohmann-json3-dev',
        'python3-numpy','python3-opencv','python3-yaml','usbutils'],check=True)
    raise SystemExit(0)
import yaml
from overlay import generate as io_overlay
from calibration import calibrate_camera,calibrate_handeye,digest,dump,rotation
import numpy as np
ROOT=Path(__file__).resolve().parent
SP=ROOT.parent/'sp_vision_25'


def profile(path):
    p=json.loads(Path(path).read_text())
    if p.get('schema')!='sp_deploy_profile_v1' or p.get('base') not in ['standard3','standard4']:raise ValueError('invalid deployment profile')
    if p.get('camera_name') not in ['hikrobot','mindvision']:raise ValueError('original Camera supports hikrobot/mindvision only')
    if not re.fullmatch(r'[0-9a-fA-F]{4}:[0-9a-fA-F]{4}',p.get('vid_pid','')):raise ValueError('invalid USB VID:PID')
    if not Path(p['serial_port']).is_absolute() or p['baudrate'] not in [115200,460800,921600]:raise ValueError('invalid serial path or baudrate')
    if not 10<=p['bullet_speed_mps']<=25 or p['enemy_color'] not in ['red','blue']:raise ValueError('invalid bullet speed or enemy color')
    if not 0<float(p['exposure_ms'])<=100:raise ValueError('invalid exposure')
    rotation(p['R_gimbal2imubody'])
    return p


def config(p,intr=None,hand=None):
    cfg=yaml.safe_load((SP/'configs'/f"{p['base']}.yaml").read_text())
    cfg.update({'camera_name':p['camera_name'],'vid_pid':p['vid_pid'],'exposure_ms':p['exposure_ms'],
        'device':p.get('device','CPU'),'gain':p.get('gain',10),'gamma':p.get('gamma',1),'R_gimbal2imubody':p['R_gimbal2imubody'],
        'enemy_color':p['enemy_color'],'yaw_offset':p.get('yaw_offset_deg',0),'pitch_offset':p.get('pitch_offset_deg',0),
        'use_can':False,'com_port':p['serial_port'],'gimbal_baudrate':p['baudrate'],'uart_mode':'auto_aim',
        'uart_bullet_speed':p['bullet_speed_mps'],'imu_timeout_ms':3000,'imu_wait_ms':100,'imu_max_age_ms':200,
        'serial_imu':{'port':p['serial_port'],'baudrate':p['baudrate'],'timeout_ms':500,'reconnect_ms':1000,
            'protocol':'uart44','frame_size':44,'frame_stride':44,'header':[83,80],'tail':[13],
            'rpy_offsets':[23,19,27],'float_endian':'little','angle_unit':'degrees','velocity_unit':'degrees',
            'rpy_signs':p.get('rpy_signs',[1,1,1]),'rpy_bias_deg':p.get('rpy_bias_deg',[0,0,0]),'inverse':False,
            'max_abs_angle_deg':36000,'tx':{'protocol':'sp44','header':[83,80],'tail':[13],'angle_unit':'degrees','write_timeout_ms':20}}})
    if intr:
        cfg.update(intr['config']);cfg['deploy_image_width'],cfg['deploy_image_height']=intr['image_size']
    if hand:
        cfg.update(hand['config'])
        if p.get('muzzle_offset_gimbal_m') is not None:
            cfg['t_camera2gimbal']=(np.array(hand['config']['t_camera2gimbal'])-np.array(p['muzzle_offset_gimbal_m'])).tolist()
    for key in ['classify_model','yolov5_model_path','yolov8_model_path','yolo11_model_path']:
        if key in cfg:
            name=Path(cfg[key]);full=(SP/name).resolve()
            if not full.is_relative_to(SP.resolve()) or not full.is_file():raise ValueError('model missing/outside source: '+str(name))
    return cfg


def validate_reports(intr,hand,p,hardware=False):
    if intr.get('schema')!='sp_intrinsics_v1' or hand.get('schema')!='sp_handeye_v1' or intr['status']!='PASS' or hand['status']!='PASS':raise ValueError('calibration did not pass')
    if hand['intrinsics_sha256']!=hashlib.sha256(json.dumps(intr,sort_keys=True).encode()).hexdigest():raise ValueError('handeye report was made with different intrinsics')
    k=np.array(intr['config']['camera_matrix']).reshape(3,3);d=np.array(intr['config']['distort_coeffs'])
    if not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or d.shape!=(5,) or not np.isfinite(d).all():raise ValueError('invalid camera parameters')
    rotation(hand['config']['R_camera2gimbal']);rotation(hand['config']['R_gimbal2imubody'])
    t=np.array(hand['config']['t_camera2gimbal'])
    if t.shape!=(3,) or not np.isfinite(t).all() or np.linalg.norm(t)>=1:raise ValueError('invalid camera offset in metres')
    if not np.allclose(p['R_gimbal2imubody'],hand['config']['R_gimbal2imubody']):raise ValueError('profile IMU basis differs from calibration')
    if hardware and (intr['kind']!='hardware' or hand['kind']!='hardware'):raise ValueError('synthetic calibration cannot activate a real deployment')
    offset=p.get('muzzle_offset_gimbal_m')
    if hardware and offset is None:raise ValueError('measured muzzle offset relative to the calibration frame is required')
    if offset is not None and (np.asarray(offset).shape!=(3,) or not np.isfinite(offset).all() or np.linalg.norm(offset)>=1):raise ValueError('invalid muzzle offset in metres')


def run(argv,**kwargs):
    print('Running: '+shlex.join(map(str,argv)),flush=True);subprocess.run(list(map(str,argv)),check=True,**kwargs)

def build(jobs=8):
    folder=ROOT/'build';io_overlay(SP,folder/'sp-overlay');run(['cmake','-S',ROOT,'-B',folder,'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_POLICY_VERSION_MINIMUM=3.5','-DOpenVINO_DIR='+os.environ.get('OpenVINO_DIR','/opt/intel/openvino_2024.6.0/runtime/cmake')])
    run(['cmake','--build',folder,'--target','sp_capture','sp_real_aim','sp_deploy_protocol_test','sp_live_pose_test','-j',jobs])
    run(['ctest','--test-dir',folder,'-R','original_gimbal_protocol|real_control_policy|live_received_pose','--output-on-failure'])
    run([sys.executable,ROOT.parent/'sp_vision_ros/scripts/check_sp_unchanged.py'])
    return folder


def install_dependencies():
    run(['sudo','apt-get','update'])
    run(['sudo','apt-get','install','-y','build-essential','cmake','pkg-config','libusb-1.0-0-dev',
        'libopencv-dev','libeigen3-dev','libfmt-dev','libspdlog-dev','libyaml-cpp-dev','nlohmann-json3-dev',
        'python3-numpy','python3-opencv','python3-yaml','usbutils'])

def doctor(p=None,hardware=False):
    report={'platform':platform.platform(),'machine':platform.machine(),'programs':{x:shutil.which(x) for x in ['cmake','c++','pkg-config','lsusb']},
        'python_modules':{'numpy':np.__version__,'yaml':yaml.__version__},'openvino_exists':Path('/opt/intel/openvino_2024.6.0/runtime/cmake').is_dir()}
    missing=[x for x in ['cmake','c++','pkg-config'] if report['programs'][x] is None]
    if not report['openvino_exists']:missing.append('OpenVINO 2024.6.0 development/runtime')
    for name in ['opencv4','libusb-1.0']:
        result=subprocess.run(['pkg-config','--exists',name]) if report['programs']['pkg-config'] else None
        if result is None or result.returncode:missing.append(name+' development files')
    if hardware:
        if p is None:raise ValueError('hardware doctor requires profile')
        device=Path(p['serial_port']);report['serial_exists']=device.is_char_device();report['serial_rw']=device.is_char_device() and os.access(device,os.R_OK|os.W_OK)
        if not report['serial_rw']:missing.append('serial device missing or not readable/writable')
        found=subprocess.run(['lsusb','-d',p['vid_pid']],capture_output=True,text=True) if report['programs']['lsusb'] else None
        report['matching_cameras']=found.stdout.splitlines() if found else []
        if len(report['matching_cameras'])!=1:missing.append('exactly one matching USB camera is required by original camera selection')
    report['missing']=missing;report['status']='PASS' if not missing else 'FAIL';return report


def verify(release):
    release=Path(release).resolve();manifest=json.loads((release/'manifest.json').read_text())
    for name,expected in manifest['files'].items():
        path=(release/name).resolve()
        if not path.is_relative_to(release) or not path.is_file() or digest(path)!=expected:raise ValueError('release integrity failed: '+name)
    return manifest


def prepare(prefix,p,intr,hand,jobs=8,do_build=True):
    validate_reports(intr,hand,p)
    folder=build(jobs) if do_build else ROOT/'build'
    release=Path(prefix).resolve()/'releases'/(datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]);release.mkdir(parents=True)
    for name in ['bin','lib','config','calibration','run']: (release/name).mkdir()
    for name in ['sp_capture','sp_real_aim']:shutil.copy2(folder/name,release/'bin'/name)
    ov=Path('/opt/intel/openvino_2024.6.0/runtime/lib/intel64')
    if not ov.is_dir():raise ValueError('OpenVINO runtime libraries missing')
    shutil.copytree(ov,release/'lib',dirs_exist_ok=True)
    arch='amd64' if platform.machine()=='x86_64' else 'arm64'
    for vendor in ['hikrobot','mindvision']:
        for lib in (SP/'io'/vendor/'lib'/arch).glob('*.so*'):shutil.copy2(lib,release/'lib'/lib.name)
    for lib in Path('/opt/intel/openvino_2024.6.0/runtime/3rdparty').rglob('libtbb*.so*'):
        shutil.copy2(lib,release/'lib'/lib.name)
    linked=subprocess.run(['ldd',str(folder/'sp_real_aim')],check=True,capture_output=True,text=True).stdout
    if 'not found' in linked:raise ValueError('linked library missing: '+linked)
    for path in re.findall(r'=>\s+(/\S+)',linked):
        if 'libtbb' in Path(path).name:shutil.copy2(path,release/'lib'/Path(path).name)
    cfg=config(p,intr,hand)
    for key in ['classify_model','yolov5_model_path','yolov8_model_path','yolo11_model_path']:
        if key in cfg:
            rel=Path(cfg[key]);dest=release/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(SP/rel,dest)
            if rel.suffix=='.xml':
                weights=rel.with_suffix('.bin')
                if not (SP/weights).is_file():raise ValueError('OpenVINO weights missing: '+str(weights))
                shutil.copy2(SP/weights,release/weights)
    (release/'config/robot.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    dump(release/'calibration/intrinsics.json',intr);dump(release/'calibration/handeye.json',hand);dump(release/'config/profile.json',p)
    launcher=release/'bin/run'
    launcher.write_text('''#!/usr/bin/env bash
set -eo pipefail
SP_RELEASE_DIR="$(cd "$(dirname "$0")/.." && pwd -P)"
export LD_LIBRARY_PATH="$SP_RELEASE_DIR/lib:${LD_LIBRARY_PATH:-}"
SP_DEPLOY_PREFIX="$(cd "$SP_RELEASE_DIR/../.." && pwd -P)"
SP_RUN_DIR="$SP_DEPLOY_PREFIX/runtime/$(basename "$SP_RELEASE_DIR")"
mkdir -p "$SP_RUN_DIR" "$SP_DEPLOY_PREFIX/logs/$(basename "$SP_RELEASE_DIR")"
if [[ ! -e "$SP_RUN_DIR/assets" ]]; then ln -s "$SP_RELEASE_DIR/assets" "$SP_RUN_DIR/assets"; fi
if [[ ! -e "$SP_RUN_DIR/logs" ]]; then ln -s "$SP_DEPLOY_PREFIX/logs/$(basename "$SP_RELEASE_DIR")" "$SP_RUN_DIR/logs"; fi
cd "$SP_RUN_DIR"
case "${1:-}" in
  --help|--self-test) exec "$SP_RELEASE_DIR/bin/sp_real_aim" "$1" ;;
  --model-check) exec "$SP_RELEASE_DIR/bin/sp_real_aim" --model-check "$SP_RELEASE_DIR/config/robot.yaml" ;;
esac
exec "$SP_RELEASE_DIR/bin/sp_real_aim" "$SP_RELEASE_DIR/config/robot.yaml" "$@"
''');launcher.chmod(0o755)
    env=os.environ.copy();env['LD_LIBRARY_PATH']=str(release/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    run([release/'bin/sp_real_aim','--self-test'],env=env);run([release/'bin/sp_capture','--help'],env=env)
    run([release/'bin/sp_real_aim','--model-check',release/'config/robot.yaml'],env=env,cwd=release)
    (release/'run').rmdir() # Writable logs are outside immutable release contents.
    manifest={'schema':'sp_release_v1','id':release.name,'platform':platform.system(),'machine':platform.machine(),
        'profile_base':p['base'],'runtime':'original SP perception/planning + build-only Gimbal fresh-pose overlay; UART44 wire unchanged',
        'uart_acceleration_transmitted':False,'default_control':False,'default_fire':False,
        'source_hashes':{str(f.relative_to(ROOT.parent)):digest(f) for f in [ROOT/'src/real_aim.cpp',ROOT/'src/capture.cpp',ROOT/'overlay.py',SP/'io/gimbal/gimbal.hpp',SP/'io/gimbal/gimbal.cpp']},
        'files':{str(f.relative_to(release)):digest(f) for f in release.rglob('*') if f.is_file()}}
    dump(release/'manifest.json',manifest);verify(release);return release


def activate(prefix,release,qualification):
    prefix=Path(prefix).resolve();release=Path(release).resolve()
    if not release.is_relative_to(prefix/'releases'):raise ValueError('release is outside deployment prefix')
    verify(release);p=json.loads((release/'config/profile.json').read_text());intr=json.loads((release/'calibration/intrinsics.json').read_text());hand=json.loads((release/'calibration/handeye.json').read_text())
    validate_reports(intr,hand,p,hardware=True)
    q=json.loads(Path(qualification).read_text())
    if q.get('schema')!='sp_sim_accuracy_v1' or q.get('status')!='PASS' or q['expected_cases']!=len(q['cases']):raise ValueError('complete high-accuracy simulation qualification required')
    selected=[c for c in q['cases'] if c['base']==p['base']]
    if {c['scenario'] for c in selected}!={'static','spin','move'} or any(c['status']!='PASS' or c['shots']<100 or c['conservative_hit_rate']<.90 or c['wilson_95_lower']<.85 for c in selected):raise ValueError('profile base has not passed the accuracy gate')
    current=prefix/'current'
    if current.exists() and not current.is_symlink():raise ValueError('current must be managed symlink')
    if current.is_symlink():
        previous=prefix/'.previous-new';previous.unlink(missing_ok=True);previous.symlink_to(os.readlink(current));os.replace(previous,prefix/'previous')
    pending=prefix/'.current-new';pending.unlink(missing_ok=True);pending.symlink_to(release.relative_to(prefix));os.replace(pending,current)
    with (prefix/'activation-history.jsonl').open('a') as f:f.write(json.dumps({'release':release.name,'at':datetime.datetime.now().isoformat(),'qualification_sha256':digest(qualification)})+'\n')
    return release


def supervise(argv,env=None,timeout=None):
    child=subprocess.Popen(list(map(str,argv)),env=env,start_new_session=True)
    try:return child.wait(timeout=timeout)
    except (KeyboardInterrupt,subprocess.TimeoutExpired):
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=3)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        return 130 if timeout is None else 124


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='action',required=True)
    d=sub.add_parser('doctor');d.add_argument('--profile',type=Path);d.add_argument('--hardware',action='store_true')
    sub.add_parser('deps')
    b=sub.add_parser('build');b.add_argument('--jobs',type=int,default=8)
    c=sub.add_parser('capture');c.add_argument('--profile',type=Path,required=True);c.add_argument('--output',type=Path,required=True);c.add_argument('--seconds',type=float,default=60);c.add_argument('--period-ms',type=int,default=500);c.add_argument('--imu',action='store_true')
    for action in ['calibrate','prepare','deploy']:
        p=sub.add_parser(action);p.add_argument('--profile',type=Path,required=True);p.add_argument('--fixed-center',action='store_true')
        if action in ['calibrate','deploy']:
            p.add_argument('--intrinsics-data',type=Path,required=True);p.add_argument('--handeye-data',type=Path,required=True)
        else:p.add_argument('--intrinsics',type=Path,required=True);p.add_argument('--handeye',type=Path,required=True)
        if action=='calibrate':p.add_argument('--output',type=Path,required=True)
        else:p.add_argument('--prefix',type=Path,required=True);p.add_argument('--jobs',type=int,default=8);p.add_argument('--skip-build',action='store_true')
        if action=='deploy':p.add_argument('--qualification',type=Path,required=True)
    a=sub.add_parser('activate');a.add_argument('--prefix',type=Path,required=True);a.add_argument('--release',type=Path,required=True);a.add_argument('--qualification',type=Path,required=True)
    for action in ['launch','rollback','verify']:
        p=sub.add_parser(action);p.add_argument('--prefix',type=Path,required=True)
        if action=='launch':p.add_argument('--control',action='store_true');p.add_argument('--fire',action='store_true')
        if action=='rollback':p.add_argument('--qualification',type=Path,required=True)
    args=parser.parse_args()
    if args.action=='deps':install_dependencies();return 0
    if args.action=='doctor':
        result=doctor(profile(args.profile) if args.profile else None,args.hardware);print(json.dumps(result,indent=2));return 0 if result['status']=='PASS' else 1
    if args.action=='build':build(args.jobs);return 0
    if args.action=='capture':
        p=profile(args.profile);output=args.output.resolve()
        if output.exists() and any(output.iterdir()):raise ValueError('capture output must be new/empty')
        cfg=ROOT/'build/capture.yaml';cfg.parent.mkdir(exist_ok=True);cfg.write_text(yaml.safe_dump(config(p),sort_keys=False))
        binary=ROOT/'build/sp_capture'
        if not binary.exists():build()
        return supervise([binary,cfg,output,args.seconds,args.period_ms]+(['imu'] if args.imu else []),timeout=args.seconds+20)
    if args.action in ['calibrate','prepare','deploy']:
        p=profile(args.profile)
        if args.action in ['calibrate','deploy']:
            output=args.output if args.action=='calibrate' else args.prefix/'calibration'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
            intr=calibrate_camera(args.intrinsics_data,p['board']);dump(output/'intrinsics.json',intr)
            if intr['status']!='PASS':raise ValueError('intrinsic quality gate failed: '+str(output/'intrinsics.json'))
            hand=calibrate_handeye(args.handeye_data,p['board'],intr,args.fixed_center);dump(output/'handeye.json',hand)
            if hand['status']!='PASS':raise ValueError('handeye quality gate failed: '+str(output/'handeye.json'))
            if args.action=='calibrate':print(output.resolve());return 0
        else:intr=json.loads(args.intrinsics.read_text());hand=json.loads(args.handeye.read_text())
        release=prepare(args.prefix,p,intr,hand,args.jobs,not args.skip_build)
        if args.action=='deploy':activate(args.prefix,release,args.qualification)
        print('Release: '+str(release));return 0
    if args.action=='activate':print(activate(args.prefix,args.release,args.qualification));return 0
    prefix=args.prefix.resolve();current=prefix/'current'
    if args.action=='rollback':print(activate(prefix,(prefix/'previous').resolve(),args.qualification));return 0
    manifest=verify(current)
    if args.action=='verify':print(json.dumps(manifest,indent=2));return 0
    if args.fire and not args.control:raise ValueError('--fire requires --control')
    p=json.loads((current/'config/profile.json').read_text());hardware=doctor(p,True)
    if hardware['status']!='PASS':raise ValueError('hardware preflight failed: '+json.dumps(hardware))
    logs=prefix/'logs';logs.mkdir(exist_ok=True)
    with (prefix/'.launch.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise ValueError('a managed hardware process is already running')
        return supervise([current/'bin/run']+(['--control'] if args.control else [])+(['--fire'] if args.fire else []))

if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,KeyError,subprocess.CalledProcessError) as e:print('Deployment stopped: '+str(e),file=sys.stderr);raise SystemExit(1)
