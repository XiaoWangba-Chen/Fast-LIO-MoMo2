#!/usr/bin/env python3
"""Collect completed real runs with conditional position metrics and raw counters."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', nargs='+', help='Directories containing benchmark variants')
    parser.add_argument('--reference', required=True)
    parser.add_argument('--transforms', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    scripts = Path(__file__).resolve().parent
    root = scripts.parent
    provenance = ('Official ugv_parking00 positions; body_imu reference-point assumption; '
                  'orientation convention and independence unverified; GNSS disabled')
    report = {
        'reference_source': provenance,
        'alignment': 'SE3; scale fixed to 1; position only',
        'profile': {'mapping_workers': 2, 'max_iteration': 3, 'cube_side_length_m': 1000,
                    'stereo_max_clones': 6, 'reliable_inputs': True},
        'limitations': ['Original ROS1 binary migration comparison not run',
                        'Real GNSS fusion disabled: antenna lever arm unavailable',
                        'ENWIDE bag unavailable (HTTP 429)',
                        'Synthetic latency test is separate from real bag timing'],
        'provenance_note': ('Source hashes describe collection-time files. New runs record build artifact hashes. '
                            'Loop-enabled backend runs were repeated after the Scan Context yaw fix; '
                            'retained local-frontend and loop-disabled metrics use paths unaffected by that fix.'),
        'source_sha256': {}, 'runs': {}}
    for path in [root/'CMakeLists.txt', root/'config/fusionportable_v2.yaml',
                 root/'src/laserMapping.cpp', *sorted((root/'src/fusion').glob('*.cpp'))]:
        report['source_sha256'][str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for collection in args.results:
        folder = Path(collection)
        for variant in sorted(folder.iterdir()):
            if not variant.is_dir() or (variant/'skipped.json').exists():
                continue
            run = json.loads((variant/'run.json').read_text())
            if run['playback_exit'] != 0:
                raise RuntimeError(f'Unsuccessful playback: {variant}')
            topic = '/odometry/global' if variant.name.startswith('all_backend') else '/odometry/local'
            subprocess.run([sys.executable, str(scripts/'summarize_run.py'), str(variant/'odometry'),
                            '--output', str(variant/'timing.json')], check=True, stdout=subprocess.DEVNULL)
            common = ['--position-only', '--estimate-to-imu', args.transforms,
                      '--reference-source', provenance]
            subprocess.run([sys.executable, str(scripts/'evaluate.py'), str(variant/'odometry'),
                            args.reference, '--bag-topic', topic, *common,
                            '--output', str(variant/'metrics_position.json')],
                           check=True, stdout=subprocess.DEVNULL)
            item = {'run': run, 'metric_topic': topic,
                    'online_position': json.loads((variant/'metrics_position.json').read_text()),
                    'timing': json.loads((variant/'timing.json').read_text())}
            if topic == '/odometry/global':
                subprocess.run([sys.executable, str(scripts/'evaluate.py'), str(variant/'odometry'),
                                args.reference, '--bag-topic', '/odometry/local', *common,
                                '--output', str(variant/'metrics_local_control.json')],
                               check=True, stdout=subprocess.DEVNULL)
                item['local_control_position'] = json.loads((variant/'metrics_local_control.json').read_text())
            trajectory = variant/'export/trajectory.tum'
            if trajectory.exists() and trajectory.stat().st_size:
                subprocess.run([sys.executable, str(scripts/'evaluate.py'), str(trajectory),
                                args.reference, *common, '--output', str(variant/'metrics_final.json')],
                               check=True, stdout=subprocess.DEVNULL)
                item['final_optimized_position'] = json.loads((variant/'metrics_final.json').read_text())
            report['runs'][f'{folder.name}/{variant.name}'] = item
            print(f'{variant}: ATE {item["online_position"]["ate_rmse_m"]:.4f} m', flush=True)
    Path(args.output).write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
