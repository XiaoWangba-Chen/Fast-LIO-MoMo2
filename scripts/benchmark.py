#!/usr/bin/env python3
"""Real rosbag2 ablation runner. Each variant uses a new estimator process."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time
import yaml

VARIANTS = {
    'lio': {'backend.enabled': False, 'degeneracy.enabled': False},
    'degeneracy': {'backend.enabled': False, 'degeneracy.enabled': True},
    'wheel_frontend': {'fusion.mode': 'frontend', 'wheel.enabled': True},
    'gnss_frontend': {'fusion.mode': 'frontend', 'gnss.enabled': True},
    'stereo_frontend': {'fusion.mode': 'frontend', 'stereo.enabled': True},
    'all_frontend': {'fusion.mode': 'frontend', 'wheel.enabled': True, 'gnss.enabled': True, 'stereo.enabled': True},
    'all_backend': {'fusion.mode': 'backend', 'wheel.enabled': True, 'gnss.enabled': True, 'stereo.enabled': True},
    'all_backend_no_loop': {'fusion.mode': 'backend', 'wheel.enabled': True, 'gnss.enabled': True,
                            'stereo.enabled': True, 'backend.loop_enabled': False},
}

def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=30)
        except subprocess.TimeoutExpired: process.kill(); process.wait()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag'); p.add_argument('--config', required=True, help='Calibrated complete rig YAML')
    p.add_argument('--sensors', nargs='+', choices=['wheel','gnss','stereo'], help='Override calibrated sensor availability explicitly')
    p.add_argument('--output', default='results/ablations'); p.add_argument('--variant', choices=list(VARIANTS), action='append')
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]; output = Path(args.output).resolve(); output.mkdir(parents=True, exist_ok=True)
    artifacts=[root/'build/fast_lio/fastlio_mapping',*sorted((root/'build/fast_lio').glob('libfastlio_*.so'))]
    build_hashes={str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest() for path in artifacts}
    configuration=yaml.safe_load(Path(args.config).read_text())['/**']['ros__parameters']
    available=set(args.sensors if args.sensors is not None else [s for s in ['wheel','gnss','stereo'] if configuration.get(s+'.enabled',configuration.get(s,{}).get('enabled',False))])
    for name in args.variant or VARIANTS:
        folder = output/name
        if folder.exists(): raise FileExistsError(folder)
        folder.mkdir()
        requested={s for s in ['wheel','gnss','stereo'] if VARIANTS[name].get(s+'.enabled',False)}
        if requested and not requested.intersection(available):
            (folder/'skipped.json').write_text(json.dumps({'reason':'No requested calibrated sensors available','requested':sorted(requested),'available':sorted(available)},indent=2))
            continue
        override = {'backend.enabled': True, 'backend.loop_enabled': True, 'degeneracy.enabled': True,
                    'wheel.enabled': False, 'gnss.enabled': False, 'stereo.enabled': False,
                    'output.directory': str(folder/'export'), **VARIANTS[name]}
        for sensor in set(['wheel','gnss','stereo'])-available:override[sensor+'.enabled']=False
        params = folder/'overrides.yaml'; params.write_text(yaml.safe_dump({'/**': {'ros__parameters': override}}))
        with (folder/'estimator.log').open('w') as log, (folder/'recorder.log').open('w') as record_log, (folder/'playback.log').open('w') as play_log:
            estimator = subprocess.Popen([str(root/'build/fast_lio/fastlio_mapping'), '--ros-args',
                '--params-file', args.config, '--params-file', str(params), '-p', 'use_sim_time:=true'], stdout=log, stderr=subprocess.STDOUT)
            recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(folder/'odometry'),
                '/odometry/local', '/odometry/imu', '/odometry/global', '/diagnostics'], stdout=record_log, stderr=subprocess.STDOUT)
            playback = None
            try:
                time.sleep(2)
                if estimator.poll() is not None or recorder.poll() is not None: raise RuntimeError('Startup failed; inspect logs')
                playback = subprocess.Popen(['ros2', 'bag', 'play', args.bag, '--clock', '100', '--rate', '1.0'], stdout=play_log, stderr=subprocess.STDOUT)
                peak_rss = 0; start = time.monotonic()
                while playback.poll() is None:
                    if estimator.poll() is not None: raise RuntimeError('Estimator exited during playback')
                    for line in Path(f'/proc/{estimator.pid}/status').read_text().splitlines():
                        if line.startswith('VmRSS:'): peak_rss = max(peak_rss, int(line.split()[1]))
                    time.sleep(.2)
                time.sleep(1)
                (folder/'run.json').write_text(json.dumps({'variant': name, 'playback_exit': playback.returncode,
                    'wall_seconds': time.monotonic()-start, 'peak_rss_kib': peak_rss, 'synthetic': False, 'build_sha256':build_hashes, 'enabled_sensors':sorted(s for s in available if override.get(s+'.enabled',False))}, indent=2))
                if playback.returncode: raise RuntimeError('Playback failed')
            finally:
                if playback: stop(playback)
                stop(estimator); stop(recorder)

if __name__ == '__main__': main()
