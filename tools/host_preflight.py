#!/usr/bin/env python3
"""Check local Linux build prerequisites without installing packages."""
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('OpenCV', 'Eigen3', 'fmt', 'spdlog', 'yaml-cpp', 'OpenVINO', 'nlohmann_json')


def main():
    missing=[]
    if platform.system()!='Linux' or platform.machine() not in ('x86_64','AMD64'):
        missing.append('Linux x86_64 (current engine bootstrap platform)')
    commands={name:shutil.which(name) for name in ('python3','cmake','ctest','c++','make')}
    missing += [name for name,path in commands.items() if path is None]
    if not os.environ.get('DISPLAY') and not shutil.which('xvfb-run'):
        missing.append('DISPLAY or xvfb-run (real camera capture needs graphical rendering)')
    openvino=os.environ.get('OpenVINO_DIR','/opt/intel/openvino_2024.6.0/runtime/cmake')
    report={'platform':platform.platform(),'cpu_count':os.cpu_count(),'tools':commands,
            'display':os.environ.get('DISPLAY'),'openvino_dir':openvino,
            'disk_free_gib':round(shutil.disk_usage(ROOT).free/2**30,2)}
    # Configure only: dependency discovery and a tiny compiler identification probe.
    # No SP sources, models, or simulator are compiled/run at this stage.
    if commands['cmake'] and commands['c++']:
        with tempfile.TemporaryDirectory(prefix='zimiao-preflight-') as folder:
            path=Path(folder)
            lines=['cmake_minimum_required(VERSION 3.16)',
                   'project(worker_preflight LANGUAGES CXX)',
                   'find_package(Threads REQUIRED)',
                   'file(WRITE "${CMAKE_BINARY_DIR}/packages.txt" "")']
            for package in PACKAGES:
                components=' COMPONENTS Runtime' if package=='OpenVINO' else ''
                lines += [f'find_package({package} QUIET{components})',
                          f'if({package}_FOUND)',
                          f'  file(APPEND "${{CMAKE_BINARY_DIR}}/packages.txt" "{package}=yes\\n")',
                          'else()',
                          f'  file(APPEND "${{CMAKE_BINARY_DIR}}/packages.txt" "{package}=no\\n")',
                          'endif()']
            (path/'CMakeLists.txt').write_text('\n'.join(lines)+'\n')
            probe=subprocess.run(['cmake','-S',str(path),'-B',str(path/'build'),
                                  '-DOpenVINO_DIR='+openvino],capture_output=True,text=True,timeout=60)
            package_file=path/'build/packages.txt'
            packages=dict(line.split('=',1) for line in package_file.read_text().splitlines()) if package_file.exists() else {}
            report['cmake_packages']=packages
            missing += [package for package in PACKAGES if packages.get(package)!='yes']
            if probe.returncode:
                missing.append('CMake compiler/dependency configuration')
                report['cmake_diagnostics']=(probe.stdout+probe.stderr)[-8000:]
    report['missing']=missing
    report['status']='FAIL' if missing else 'PASS'
    report['note']='PASS checks prerequisites only; Vulkan device and real pixels are verified by the graphical closed loop.'
    out=ROOT/'artifacts/local-preflight.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
    if missing:
        print('Prepare the listed dependencies, then rerun doctor. No packages were installed.',flush=True)
    return 2 if missing else 0


if __name__=='__main__':
    raise SystemExit(main())
