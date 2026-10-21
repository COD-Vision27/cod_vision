#!/usr/bin/env python3
"""Observe the simulator's image transport only; never launch an aiming node.

Run on the remote host after sourcing sp_vision_ros/scripts/env.sh. This script
does not build software, change scene parameters, or publish any ROS messages.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
ART = ROOT / 'sp_vision_ros/artifacts'


def isolate_ros_domain():
    # env.sh sets the normal application's domain before this script runs.
    # Do not inherit that domain into this independent observer.
    os.environ['ROS_DOMAIN_ID'] = '90'


def distribution(values):
    """Linear-interpolated percentiles; preserve negative ages as evidence."""
    values = sorted(values)
    def percentile(q):
        position = (len(values) - 1) * q
        low = int(position)
        high = min(low + 1, len(values) - 1)
        return values[low] + (values[high] - values[low]) * (position - low)
    return {'count': len(values), 'p50': percentile(.5) if values else None,
            'p95': percentile(.95) if values else None}


def image_summary(frames):
    capture = [(b['stamp_ns'] - a['stamp_ns']) / 1e6 for a, b in zip(frames, frames[1:])]
    received = [(b['received_monotonic_ns'] - a['received_monotonic_ns']) / 1e6
                for a, b in zip(frames, frames[1:])]
    ages = [(f['received_ros_ns'] - f['stamp_ns']) / 1e6 for f in frames if f['stamp_ns'] > 0]
    span = (frames[-1]['received_monotonic_ns'] - frames[0]['received_monotonic_ns']) / 1e9 if len(frames) > 1 else 0
    return {'images': len(frames), 'observed_span_seconds': span,
            'receive_fps': (len(frames) - 1) / span if span else None,
            'capture_interval_ms': distribution(capture),
            'receive_interval_ms': distribution(received), 'frame_age_ms': distribution(ages),
            'nonpositive_capture_intervals': sum(x <= 0 for x in capture),
            'negative_frame_ages': sum(x < 0 for x in ages),
            'zero_image_stamps': sum(f['stamp_ns'] == 0 for f in frames),
            'image_formats': sorted({(f['width'], f['height'], f['encoding']) for f in frames})}


def source_hashes(config):
    sim = ROOT / 'bevy_robomaster_simulator'
    paths = {config, ROOT / 'tools/capture_performance.py',
             ROOT / 'sp_vision_ros/scripts/run_sim.sh', ROOT / 'sp_vision_ros/scripts/env.sh'}
    for filename in ['Cargo.toml', 'Cargo.lock', 'config.toml']:
        if (sim / filename).is_file():
            paths.add(sim / filename)
    for folder in ['src', 'crates', 'rm_interfaces']:
        paths.update(p for p in (sim / folder).rglob('*') if p.is_file())
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def gpu_snapshot():
    try:
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=10)
        return {'exit_code': result.returncode, 'output': result.stdout + result.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'error': str(error)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-config', type=Path, default=ROOT / 'sp_vision_ros/configs/simulator.toml')
    parser.add_argument('--seconds', type=float, default=30)
    args = parser.parse_args()
    if not (ROOT / '.zimiao-remote-worker').is_file():
        parser.error('Run through tools/remote_run.py on the remote worker; no local simulation.')
    if not math.isfinite(args.seconds) or args.seconds <= 5:
        parser.error('--seconds must be finite and greater than the 5-second startup exclusion')
    config = args.sim_config.resolve()
    if not config.is_file():
        parser.error(f'configuration does not exist: {config}')
    ART.mkdir(parents=True, exist_ok=True)
    (ART / 'capture-performance-sim.log').write_text('')
    # Keep the observer and its child away from the normal domain (25).
    # The scheduler may explicitly select another isolated domain.
    isolate_ros_domain()
    report = {'schema': 'capture_performance_v1', 'hostname': os.uname().nodename,
              'ros_domain_id': os.environ['ROS_DOMAIN_ID'],
              'seconds_requested': args.seconds, 'topic': '/image_raw',
              'auto_aim_forced': False, 'launches_aim_or_hardware': False,
              'startup_exclusion_seconds_from_first_image': 5,
              'config': {'path': str(config), 'sha256': hashlib.sha256(config.read_bytes()).hexdigest()},
              'clock_method': 'Image header and rclpy SYSTEM_TIME for ages; monotonic receive times for intervals.',
              'capture_interval_caveat': 'Header-stamp spacing, not an independently measured exposure interval.',
              'source_scope': 'Simulator src/crates/interfaces and Cargo/config; benchmark, launcher and ROS environment scripts.',
              'status': 'FAIL', 'timed_out': False, 'cleanup_signals': [], 'sim_exit': None}
    before = source_hashes(config)
    report['source_hashes_before'] = before
    frames, lines, failures = [], [], []
    child = node = None
    ros = None
    reader = None
    initialized = False
    started = time.monotonic()
    try:
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        ros = rclpy
        ros.init()
        initialized = True
        node = ros.create_node('capture_performance_observer')
        system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        def receive(message):
            frames.append({'stamp_ns': message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                           'received_ros_ns': system_clock.now().nanoseconds,
                           'received_monotonic_ns': time.monotonic_ns(),
                           'width': message.width, 'height': message.height, 'encoding': message.encoding})
        subscription = node.create_subscription(Image, '/image_raw', receive, qos_profile_sensor_data)
        env = os.environ.copy()
        env['SP_SIM_CONFIG'] = str(config)
        env['DAEDALUS_AUTO_AIM'] = '0'  # Deliberately no command-line opt-in.
        # Observe exactly the selected TOML; do not inherit scenario overrides.
        inherited = sorted(key for key in env if key.startswith('DAEDALUS_') and key != 'DAEDALUS_AUTO_AIM')
        for key in inherited:
            env.pop(key)
        report['removed_inherited_override_names'] = inherited
        env['RUST_LOG'] = 'daedalus=info,bevy_render=info,bevy_diagnostic=info,wgpu=warn,bevy_gltf=warn'
        report['ros_domain_id'] = env.get('ROS_DOMAIN_ID', '0')
        cmd = ['bash', str(ROOT / 'sp_vision_ros/scripts/run_sim.sh'), '--duration-secs', str(args.seconds)]
        if not env.get('DISPLAY') and not env.get('WAYLAND_DISPLAY'):
            cmd = ['xvfb-run', '-a', *cmd]
        report['command'] = cmd
        report['gpu_start'] = gpu_snapshot()
        child = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, start_new_session=True)
        def consume_log():
            with (ART / 'capture-performance-sim.log').open('w') as handle:
                for line in child.stdout:
                    lines.append(line)
                    handle.write(line)
                    handle.flush()
                    if 'Auto-aim subscription is now ENABLED' in line:
                        failures.append('Auto-aim subscription unexpectedly enabled; stopping observation.')
        reader = threading.Thread(target=consume_log, daemon=True)
        reader.start()
        deadline = time.monotonic() + args.seconds + 120
        while child.poll() is None:
            ros.spin_once(node, timeout_sec=.05)
            if failures:
                raise RuntimeError(failures[0])
            if time.monotonic() > deadline:
                report['timed_out'] = True
                raise RuntimeError('simulator exceeded requested duration plus 120-second startup allowance')
        drain_until = time.monotonic() + .5
        while time.monotonic() < drain_until:
            ros.spin_once(node, timeout_sec=.05)
        if child.returncode != 0:
            raise RuntimeError(f'simulator exited {child.returncode}; inspect capture-performance-sim.log')
        if not frames:
            raise RuntimeError('no image_raw messages received')
        report['status'] = 'PASS'
    except Exception as error:
        report['error'] = str(error)
    finally:
        if child is not None:
            for sig, timeout in [(signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)]:
                if child.poll() is not None:
                    break
                try:
                    os.killpg(child.pid, sig)
                    report['cleanup_signals'].append(sig.name)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
            report['sim_exit'] = child.poll()
            report['cleanup_complete'] = child.poll() is not None
            if reader is not None:
                reader.join(timeout=5)
                report['log_reader_complete'] = not reader.is_alive()
            child.stdout.close()
        else:
            report['cleanup_complete'] = True
        if node is not None:
            node.destroy_node()
        if initialized:
            ros.shutdown()
        report['gpu_end'] = gpu_snapshot()
        log = ''.join(lines)
        report['gpu_adapters'] = re.findall(r'AdapterInfo\s*\{[^\n]+', log)
        fps = [(float(current), float(avg)) for current, avg in re.findall(
            r'\bfps\s*:\s*([\d.]+)\s*\(avg\s+([\d.]+)\)', log)]
        report['renderer_fps'] = {'method': 'Bevy diagnostic FPS samples; exclude first 10 samples, not a timed window. Current value and diagnostic moving average reported separately.',
                                  'all_samples': len(fps), 'excluded_samples': min(10, len(fps)),
                                  'current_after_first_10': distribution([x for x, _ in fps[10:]]),
                                  'diagnostic_average_after_first_10': distribution([x for _, x in fps[10:]])}
        report['images_all'] = image_summary(frames)
        first = frames[0]['received_monotonic_ns'] if frames else 0
        steady = [f for f in frames if f['received_monotonic_ns'] >= first + 5_000_000_000]
        report['images_after_startup'] = image_summary(steady)
        report['frames'] = frames
        report['wall_seconds'] = time.monotonic() - started
        after = source_hashes(config)
        report['source_unchanged'] = before == after
        report['source_changed_paths'] = sorted(p for p in before.keys() | after.keys() if before.get(p) != after.get(p))
        if (not report['source_unchanged'] or failures or not report['cleanup_complete']
                or not report.get('log_reader_complete', child is None) or len(steady) < 2):
            report['status'] = 'FAIL'
        (ART / 'capture-performance.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k not in {'source_hashes_before', 'frames', 'gpu_start', 'gpu_end'}}, indent=2), flush=True)
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
