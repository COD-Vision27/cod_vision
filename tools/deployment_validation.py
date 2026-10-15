#!/usr/bin/env python3
"""Remote-only package test with clearly synthetic calibration; never opens hardware."""
import json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'sp_deploy'))
import deploy
import calibration as cal

def main():
    data=ROOT/'sp_deploy/.runtime/validation-data'
    if not data.exists():
        subprocess.run([sys.executable,str(ROOT/'sp_deploy/tests/test_calibration.py'),'--export',str(data)],check=True)
    subprocess.run([sys.executable,'-m','unittest','discover','-s',str(ROOT/'sp_deploy/tests'),'-p','test_*.py','-v'],check=True)
    intr=json.loads((data/'intrinsics.json').read_text());hand=json.loads((data/'handeye.json').read_text());p=deploy.profile(data/'profile.json')
    cli_cal=data/'cli-calibration'
    subprocess.run([sys.executable,str(ROOT/'sp_deploy/deploy.py'),'calibrate','--profile',str(data/'profile.json'),
        '--intrinsics-data',str(data/'intrinsics'),'--handeye-data',str(data/'handeye'),'--fixed-center','--output',str(cli_cal)],check=True)
    cli_prefix=ROOT/'sp_deploy/.runtime/cli-validation-deployment'
    attempt=subprocess.run([sys.executable,str(ROOT/'sp_deploy/deploy.py'),'deploy','--profile',str(data/'profile.json'),
        '--intrinsics-data',str(data/'intrinsics'),'--handeye-data',str(data/'handeye'),'--fixed-center','--prefix',str(cli_prefix),
        '--qualification',str(ROOT/'sp_vision_ros/artifacts/accuracy_qualification.json'),'--skip-build'],capture_output=True,text=True)
    if attempt.returncode==0 or 'synthetic' not in attempt.stderr or (cli_prefix/'current').exists():
        raise RuntimeError('one-command pipeline did not reject synthetic deployment: '+attempt.stderr)

    prefix=ROOT/'sp_deploy/.runtime/validation-deployment'
    release=deploy.prepare(prefix,p,intr,hand,do_build=False);deploy.verify(release)
    alias=prefix/'probe-link';alias.unlink(missing_ok=True);alias.symlink_to(release.relative_to(prefix))
    try:
        subprocess.run([str(alias/'bin/run'),'--self-test'],check=True)
        if not (prefix/'runtime'/release.name/'assets').is_symlink():raise RuntimeError('launcher did not resolve release through alias')
    finally:alias.unlink(missing_ok=True)
    rejected=False
    try:deploy.activate(prefix,release,ROOT/'sp_vision_ros/artifacts/accuracy_qualification.json')
    except ValueError as e:
        if 'synthetic' not in str(e):raise
        rejected=True
    if not rejected or (prefix/'current').exists():raise RuntimeError('synthetic calibration was activated')
    report={'status':'PASS','release':str(release),'calibration_kind':'synthetic','intrinsic_rms_px':intr['rms_px'],
        'handeye_translation_p95_m':hand['translation_p95_m'],'handeye_rotation_p95_deg':hand['rotation_p95_deg'],
        'checks':['16 Python calibration/deployment checks','CLI calibration','one-command pipeline rejects synthetic activation','copy XML and BIN models','packaged OpenVINO model inference',
            'native control policy','launcher resolves symlink release','release checksums','synthetic activation rejected'],
        'not_executed':['real camera capture','real IMU synchronization','real handeye calibration','real gimbal control','real firing']}
    cal.dump(ROOT/'sp_vision_ros/artifacts/deployment_validation.json',report);print(json.dumps(report,indent=2))
    return 0
if __name__=='__main__':raise SystemExit(main())
