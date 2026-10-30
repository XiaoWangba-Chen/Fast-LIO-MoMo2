#!/usr/bin/env python3
"""Build/save/restart/localize a synthetic static scene. Not an accuracy benchmark."""
from array import array
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import yaml
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from ros_smoke import stamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/fastlio-localization-smoke.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    process = None
    rclpy.init()
    node = rclpy.create_node('fastlio_localization_smoke')
    local, glob, transforms, diagnostics, maps = [], [], [], [], []
    subscriptions = [node.create_subscription(typ, topic, dst.append, 1000)
                     for typ, topic, dst in [(Odometry, '/odometry/local', local),
                                            (Odometry, '/odometry/global', glob),
                                            (TFMessage, '/tf', transforms),
                                            (DiagnosticArray, '/diagnostics', diagnostics)]]
    imu_pub = node.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2, '/points_raw', qos_profile_sensor_data)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    coords = []
    for a in np.linspace(-4, 4, 45):
        for b in np.linspace(-2, 2, 25):
            coords.extend([(6., float(a), float(b)), (float(a), 6., float(b)),
                           (float(a), float(b), -2.)])
    packed = b''.join(struct.pack('<fffffH', x, y, z, 100., i*.1/(len(coords)-1), i%16)
                      for i, (x, y, z) in enumerate(coords))
    fields = [PointField(name=n, offset=o, datatype=t, count=1)
              for n, o, t in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                              ('intensity', 12, 7), ('time', 16, 7), ('ring', 20, 4)]]

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)

    def feed(seconds, hooks=None):
        start, epoch = time.monotonic(), time.time()
        for i in range(round(seconds*200)):
            while time.monotonic() < start+i*.005:
                rclpy.spin_once(node, timeout_sec=.001)
            if hooks and i in hooks:
                hooks[i]()
            imu = Imu(); imu.header.stamp=stamp(epoch+i*.005)
            imu.header.frame_id='imu_link'; imu.linear_acceleration.z=9.81
            imu_pub.publish(imu)
            if i%20 == 0:
                cloud = PointCloud2(); cloud.header.stamp=imu.header.stamp
                cloud.header.frame_id='lidar_link'; cloud.height=1; cloud.width=len(coords)
                cloud.fields=fields; cloud.point_step=22; cloud.row_step=22*len(coords)
                cloud.is_dense=True; cloud.data=array('B', packed); cloud_pub.publish(cloud)
            rclpy.spin_once(node, timeout_sec=0)
        spin(.5)

    def stop():
        nonlocal process
        if process is not None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            process = None
        spin(.3)

    def save():
        client = node.create_client(Trigger, '/fastlio_mapping/save')
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done(), 'Save timed out'
        result = future.result(); node.destroy_client(client)
        return result

    with tempfile.TemporaryDirectory(prefix='fastlio-localization-smoke-') as tmp:
        tmp = Path(tmp)
        config = yaml.safe_load((root/'config/slam.yaml').read_text())
        p = config['/**']['ros__parameters']
        p['preprocess'].update(lidar_type=2, scan_line=16, timestamp_unit=0)
        p['point_filter_num']=1
        p['backend']['loop_enabled']=False
        p['output']['directory']=str(tmp/'export')
        p['extrinsics.imu_T_base.translation']=[.3,-.2,.1]
        p['extrinsics.imu_T_base.quaternion']=[0.,0.,0.,1.]
        sensor_config=tmp/'sensor.yaml'; sensor_config.write_text(yaml.safe_dump(config))
        logs=[]
        try:
            log = open(tmp/'mapping.log', 'w'); logs.append(log)
            process=subprocess.Popen([str(root/'build/fast_lio/fastlio_mapping'), '--ros-args',
                                      '--params-file', str(sensor_config)], stdout=log, stderr=subprocess.STDOUT)
            spin(2); assert process.poll() is None, 'Mapping startup failed'
            feed(4); assert local and glob, 'Mapping produced no odometry'
            assert save().success, 'Map bundle export failed'
            stop()
            bundle=tmp/'export/map_bundle'
            files=list(bundle.rglob('*'))
            before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()}
            assert (bundle/'manifest.txt').exists()
            local.clear(); glob.clear(); transforms.clear(); diagnostics.clear()
            # Exercise installed launch/config and late-subscriber transient-local map delivery.
            log = open(tmp/'localization.log', 'w'); logs.append(log)
            process=subprocess.Popen(['ros2', 'launch', 'fast_lio', 'localization.launch.py',
                                      f'config:={sensor_config}', f'map_directory:={bundle}'],
                                     stdout=log, stderr=subprocess.STDOUT)
            spin(2); assert process.poll() is None, 'Localization startup failed'
            subscriptions.append(node.create_subscription(PointCloud2, '/map/global', maps.append,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
            spin(.5)
            assert not glob, 'Global odometry published before localization'
            assert not any(t.header.frame_id=='map' for m in transforms for t in m.transforms)
            automatic_count=[]
            def manual_seed():
                automatic_count.append(len(glob))
                assert glob, 'Automatic localization failed before the manual seed'
                initial=PoseWithCovarianceStamped(); initial.header.frame_id='map'
                initial.pose.pose.position.x=.3; initial.pose.pose.position.y=-.2
                initial.pose.pose.position.z=.1; initial.pose.pose.orientation.w=1.
                initial_pub.publish(initial); glob.clear()
            # Keep IMU continuous across the manual seed, so a session reset
            # cannot accidentally replace the manual path with automatic recovery.
            feed(7, {1000: manual_seed})
            assert automatic_count and automatic_count[0]>0
            assert glob and local and maps, 'No localization or retained map output'
            statuses=[s for d in diagnostics for s in d.status if s.name=='fastlio/localization']
            assert any(s.message=='tracking' for s in statuses), 'Never reached tracking'
            assert any(t.header.frame_id=='map' and t.child_frame_id=='odom'
                       for m in transforms for t in m.transforms), 'Missing global transform'
            expected=np.array([.3,-.2,.1])
            for m in glob:
                pos=m.pose.pose.position
                assert np.linalg.norm(np.array([pos.x,pos.y,pos.z])-expected)<.15, 'Wrong global base_link pose'
                cov=np.asarray(m.pose.covariance).reshape(6,6)
                assert np.isfinite(cov).all() and np.linalg.eigvalsh(cov).min()>-1e-7
            assert glob, 'Manual base_link initial pose did not recover tracking'
            assert all(int(v.value)==0 for status in statuses for v in status.values
                       if v.key=='session'), 'Unexpected estimator reset during manual seed test'
            assert not save().success, 'Localization allowed overwriting the fixed map'
            stop()
            after={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in bundle.rglob('*') if f.is_file()}
            assert before==after, 'Fixed map changed'
            report={'passed':True, 'synthetic':True, 'map_roundtrip':True,
                    'automatic_localization':True, 'manual_base_link_pose':True,
                    'fixed_map_unchanged':True, 'retained_map_received':True}
            Path(args.output).write_text(json.dumps(report, indent=2)); print(json.dumps(report, indent=2))
        finally:
            stop()
            for log in logs: log.close()
            for name in ['mapping', 'localization']:
                path=tmp/(name+'.log')
                if path.exists(): Path(args.output+'.'+name+'.log').write_text(path.read_text())
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
