#!/usr/bin/env python3
"""Replay a real ROS2 bag twice: map export, then automatic fixed-map localization.

Position differences are consistency against the map-building run, not ground truth.
Requires an isolated ROS_DOMAIN_ID and calibrated LiDAR/IMU config.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--offset', type=float, default=0., help='Localization replay start offset, seconds')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    output=Path(args.output).resolve(); output.mkdir(parents=True, exist_ok=False)
    rclpy.init(); node=rclpy.create_node('fastlio_localization_replay')
    global_poses=[]; states=[]
    def odometry(m):
        p=m.pose.pose.position
        global_poses.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z])
    def diagnostic(m):
        for s in m.status:
            if s.name=='fastlio/localization':
                states.append({'time':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                               'phase':s.message, **{v.key:v.value for v in s.values}})
    subscriptions=[node.create_subscription(Odometry,'/odometry/global',odometry,1000),
                   node.create_subscription(DiagnosticArray,'/diagnostics',diagnostic,1000)]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    def run_stage(name,localize=False):
        global_poses.clear(); states.clear(); estimator=None; playback=None
        with (output/(name+'.log')).open('w') as log, (output/(name+'-playback.log')).open('w') as play_log:
            try:
                command=[str(root/'build/fast_lio/fastlio_mapping'),'--ros-args','--params-file',args.config,
                         '-p','use_sim_time:=true','-p','wheel.enabled:=false','-p','stereo.enabled:=false',
                         '-p','gnss.enabled:=false','-p','backend.loop_enabled:=false',
                         '-p',f'output.directory:={output}/export']
                if localize:command+=['-p','localization.enabled:=true','-p','backend.enabled:=false',
                                      '-p',f'localization.map_directory:={output}/export/map_bundle']
                estimator=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                spin(2)
                if estimator.poll() is not None:raise RuntimeError(name+' startup failed')
                command=['ros2','bag','play',args.bag,'--clock','100','--rate','1.0',
                         '--topics','/points_raw','/imu/data']
                if localize and args.offset:command+=['--start-offset',str(args.offset)]
                print('Starting '+name,flush=True)
                playback=subprocess.Popen(command,stdout=play_log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+300
                while playback.poll() is None:
                    rclpy.spin_once(node,timeout_sec=.01)
                    if estimator.poll() is not None:raise RuntimeError(name+' estimator exited')
                    if time.monotonic()>deadline:raise TimeoutError('Playback exceeded 300 seconds')
                if playback.returncode:raise RuntimeError(name+' playback failed')
                spin(1)
                if not localize:
                    client=node.create_client(Trigger,'/fastlio_mapping/save')
                    if not client.wait_for_service(timeout_sec=2):raise RuntimeError('No save service')
                    future=client.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node,future,timeout_sec=30)
                    if not future.done() or not future.result().success:raise RuntimeError('Map save failed')
                    node.destroy_client(client)
                return {'poses':list(global_poses),'states':list(states)}
            finally:
                stop(playback); stop(estimator); spin(.3)
    report={'synthetic':False,'independent_traversal':False,'offset_seconds':args.offset,'passed':False,
            'comparison':'Consistency against map-building trajectory; not ground truth'}
    try:
        mapping=run_stage('mapping')
        bundle=output/'export/map_bundle'
        hashes=lambda:{str(p.relative_to(bundle)):hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in bundle.rglob('*') if p.is_file()}
        before=hashes(); localized=run_stage('localization',True)
        poses=np.asarray(localized['poses']); reference=np.loadtxt(output/'export/trajectory.tum',ndmin=2)
        report['global_messages']=len(poses)
        report['states']=localized['states']
        report['map_unchanged']=before==hashes()
        if len(poses):
            expected=np.column_stack([np.interp(poses[:,0],reference[:,0],reference[:,i]) for i in range(1,4)])
            errors=np.linalg.norm(poses[:,1:4]-expected,axis=1)
            report.update(first_global_stamp=float(poses[0,0]),
                          position_consistency_rmse_m=float(np.sqrt(np.mean(errors**2))),
                          position_consistency_p95_m=float(np.percentile(errors,95)))
        report['passed']=bool(len(poses)>100 and report['map_unchanged'] and
                              any(s['phase']=='tracking' for s in localized['states']) and
                              report.get('position_consistency_p95_m',float('inf'))<1.)
        (output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='states'},indent=2))
        if not report['passed']:raise RuntimeError('Real replay acceptance failed; inspect report and logs')
    finally:
        (output/'report.json').write_text(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
