#!/usr/bin/env python3
"""Synthetic ROS2 integration test. This does NOT measure dataset accuracy."""
import argparse
from array import array
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField, Image, CameraInfo, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time


def stamp(t):
    ns = round(t*1e9)
    return Time(sec=ns//10**9, nanosec=ns%10**9)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['frontend', 'backend'], default='backend')
    parser.add_argument('--seconds', type=float, default=6.)
    parser.add_argument('--invalid-imu', action='store_true')
    parser.add_argument('--scan-overlap-ms', type=float, default=0.)
    parser.add_argument('--clock-reset', action='store_true')
    parser.add_argument('--imu-gap', action='store_true')
    parser.add_argument('--faults', action='store_true', help='Inject wheel slip, GNSS jump, black images and LiDAR loss')
    parser.add_argument('--output', default='/tmp/fastlio-smoke.json')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    binary = root/'build/fast_lio/fastlio_mapping'
    temp = tempfile.TemporaryDirectory(prefix='fastlio-smoke-')
    log = open(Path(temp.name)/'node.log', 'w')
    command = [str(binary), '--ros-args', '--params-file', str(root/'config/slam.yaml'),
               '-p', f'fusion.mode:={args.mode}', '-p', 'preprocess.lidar_type:=2',
               '-p', 'preprocess.scan_line:=16', '-p', 'preprocess.timestamp_unit:=0',
               '-p', 'point_filter_num:=1', '-p', 'backend.loop_enabled:=false',
               '-p', 'stereo.enabled:=true', '-p', 'wheel.enabled:=true',
               '-p', 'gnss.enabled:=true', '-p', 'gnss.yaw_known:=true',
               '-p', 'degeneracy.enabled:=true', '-p', f'output.directory:={temp.name}/export']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=os.environ)
    rclpy.init()
    node = rclpy.create_node('fastlio_smoke')
    pubs = {name: node.create_publisher(typ, topic, qos_profile_sensor_data) for name, typ, topic in [
        ('imu', Imu, '/imu/data'), ('lidar', PointCloud2, '/points_raw'),
        ('left', Image, '/camera/left/image_raw'), ('right', Image, '/camera/right/image_raw'),
        ('wheel', TwistWithCovarianceStamped, '/wheel/twist'), ('fix', NavSatFix, '/gps/fix')]}
    info_pubs = [node.create_publisher(CameraInfo, f'/camera/{side}/camera_info', 10)
                 for side in ['left', 'right']]
    fast, local, glob, paths, diagnostics, transforms = [], [], [], [], [], []
    sent, latencies = {}, []
    epoch = time.time()
    def receive(destination, msg):
        destination.append(msg)
        if destination is fast:
            index = round((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9-epoch)*200)
            if index in sent: latencies.append(1000*(time.monotonic()-sent[index]))
    subscriptions = [node.create_subscription(typ, topic, lambda msg, dst=dst: receive(dst, msg), 1000)
                     for typ, topic, dst in [(Odometry, '/odometry/imu', fast),
                     (Odometry, '/odometry/local', local), (Odometry, '/odometry/global', glob),
                     (PathMsg, '/path/optimized', paths), (TFMessage, '/tf', transforms), (DiagnosticArray, '/diagnostics', diagnostics)]]
    rng = np.random.default_rng(42)
    left = np.zeros((240, 320), dtype=np.uint8)
    for x, y in rng.integers([20, 20], [300, 220], size=(150, 2)):
        left[y:y+4, x:x+4] = rng.integers(100, 255)
    right = np.zeros_like(left); right[:, :-8] = left[:, 8:]
    coords = []
    for a in np.linspace(-4, 4, 25):
        for b in np.linspace(-2, 2, 15):
            coords.extend([(6., float(a), float(b)), (float(a), 6., float(b)),
                           (float(a), float(b), -2.)])
    packed = b''.join(struct.pack('<fffffH', x, y, z, 100., i*(.1+args.scan_overlap_ms*.001)/(len(coords)-1), i%16)
                      for i, (x, y, z) in enumerate(coords))
    fields = [PointField(name=name, offset=offset, datatype=typ, count=1)
              for name, offset, typ in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                       ('intensity', 12, 7), ('time', 16, 7), ('ring', 20, 4)]]
    try:
        discovery = time.monotonic()+1.5
        while time.monotonic()<discovery:
            rclpy.spin_once(node, timeout_sec=.02)
        if process.poll() is not None:
            raise RuntimeError('Estimator exited during startup')
        parameters = node.create_client(SetParameters, '/fastlio_mapping/set_parameters')
        assert parameters.wait_for_service(timeout_sec=2.), 'Parameter service missing'
        request = SetParameters.Request()
        request.parameters = [Parameter(name='fusion.mode', value=ParameterValue(type=4, string_value='frontend' if args.mode=='backend' else 'backend'))]
        future = parameters.call_async(request); rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
        assert future.done() and not future.result().results[0].successful, 'Fusion mode changed during execution'
        start = time.monotonic(); epoch = time.time()
        count = 0
        while count <= round(args.seconds*200):
            target = start + count*.005
            while time.monotonic()<target:
                rclpy.spin_once(node, timeout_sec=min(.001, max(0., target-time.monotonic())))
            t = epoch+count*.005-(2. if args.clock_reset and count>=600 else 0.)
            imu = Imu(); imu.header.stamp=stamp(t); imu.header.frame_id='imu_link'
            imu.linear_acceleration.z=9.81
            if args.invalid_imu and count==300: imu.linear_acceleration.x=float('nan')
            sent[round((t-epoch)*200)] = time.monotonic()
            if not (args.imu_gap and 3.2<count*.005<3.4): pubs['imu'].publish(imu)
            wheel = TwistWithCovarianceStamped(); wheel.header=imu.header; wheel.header.frame_id='base_link'
            wheel.twist.covariance[0]=.04
            if args.faults and 1.5<count*.005<1.65: wheel.twist.twist.linear.x=1000.
            pubs['wheel'].publish(wheel)
            if count%20 == 0:
                cloud = PointCloud2(); cloud.header.stamp=stamp(t); cloud.header.frame_id='lidar_link'
                cloud.height=1; cloud.width=len(coords); cloud.fields=fields
                cloud.point_step=22; cloud.row_step=22*len(coords); cloud.is_dense=True
                cloud.data=array('B', packed)
                if not (args.faults and 2.2<count*.005<3.): pubs['lidar'].publish(cloud)
                for i, pixels in enumerate([left, right]):
                    info = CameraInfo(); info.header.stamp=stamp(t); info.header.frame_id='camera_left_optical'
                    info.width=320; info.height=240; info.distortion_model='plumb_bob'; info.d=[0.]*5
                    info.k=[300.,0.,160.,0.,300.,120.,0.,0.,1.]
                    info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
                    info.p=[300.,0.,160.,-60.*i,0.,300.,120.,0.,0.,0.,1.,0.]
                    info_pubs[i].publish(info)
                    image=Image(); image.header=info.header; image.width=320; image.height=240
                    image.encoding='mono8'; image.step=320
                    image.data=array('B', (np.zeros_like(pixels) if args.faults and 1.7<count*.005<2. else pixels).tobytes())
                    pubs['left' if i==0 else 'right'].publish(image)
                fix=NavSatFix(); fix.header=imu.header; fix.status.status=0
                fix.latitude=22.+(.01 if args.faults and 1.5<count*.005<1.7 else 0.); fix.longitude=114.; fix.altitude=10.
                fix.position_covariance_type=2; fix.position_covariance=[.1,0.,0.,0.,.1,0.,0.,0.,.2]
                pubs['fix'].publish(fix)
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0)
            count += 1
        drain = time.monotonic()+1.
        while time.monotonic()<drain:
            rclpy.spin_once(node, timeout_sec=.01)
        stamps = [m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in fast]
        diag = {kv.key: kv.value for msg in diagnostics for status in msg.status for kv in status.values}
        boundaries=[0]+[i+1 for i,(a,b) in enumerate(zip(stamps,stamps[1:])) if b<a-1.]+[len(stamps)]
        segments=[stamps[a:b] for a,b in zip(boundaries,boundaries[1:]) if b>a]
        expected=sum(round((segment[-1]-segment[0])*200)+1 for segment in segments)
        monotonic_sessions=all(all(b>a for a,b in zip(segment,segment[1:])) for segment in segments)
        report = {'synthetic': True, 'faults_injected': args.faults, 'invalid_imu_injected':args.invalid_imu, 'scan_overlap_ms': args.scan_overlap_ms, 'mode_immutable': True, 'clock_reset': args.clock_reset, 'imu_gap': args.imu_gap, 'mode': args.mode, 'imu_published': count,
                  'fast_received': len(fast), 'local_received': len(local), 'global_received': len(glob),
                  'fast_valid_window_ratio': len(fast)/expected if expected else 0,
                  'fast_monotonic': all(b>a for a,b in zip(stamps, stamps[1:])),
                  'fast_monotonic_per_session': monotonic_sessions, 'output_sessions': len(segments),
                  'fast_latency_p95_ms': float(np.percentile(latencies,95)) if latencies else None,
                  'diagnostics': diag,
                  'local_final_position': [local[-1].pose.pose.position.x,
                                           local[-1].pose.pose.position.y,
                                           local[-1].pose.pose.position.z] if local else None}
        report['passed']=False
        Path(args.output).write_text(json.dumps(report, indent=2))
        assert local and fast and glob and paths, report
        if not args.clock_reset and not args.imu_gap:
            assert len(local)>=(args.seconds-.5-(.8 if args.faults else 0))*10, 'Unexpected loss of local LiDAR-rate output'
        if args.invalid_imu: assert int(diag.get('imu.invalid','0'))>=1, 'Non-finite IMU was not rejected'
        assert monotonic_sessions, report
        if args.clock_reset: assert len(segments)>=2 and int(diag.get('session','0'))>=1, report
        else: assert report['fast_monotonic'], report
        if not args.imu_gap: assert report['fast_valid_window_ratio']>=.99, report
        else:
            assert any(b-a>.1 for a,b in zip(stamps,stamps[1:])), 'IMU outage did not suspend output'
            assert stamps[-1]>epoch+4.8 and int(diag.get('session','0'))>=1, 'No recovery after IMU outage'
        for sensor in ['wheel', 'gnss', 'stereo']:
            assert int(diag.get(sensor+'.accepted', '0'))>0, report
        assert latencies and report['fast_latency_p95_ms']<20., report
        assert any(t.header.frame_id=='odom' and t.child_frame_id=='base_link' for m in transforms for t in m.transforms), 'Missing local TF'
        assert any(t.header.frame_id=='map' and t.child_frame_id=='odom' for m in transforms for t in m.transforms), 'Missing global TF'
        for msg in local+glob+fast:
            for values in [msg.pose.covariance,msg.twist.covariance]:
                covariance=np.asarray(values).reshape(6,6)
                assert np.isfinite(covariance).all() and np.linalg.eigvalsh(covariance).min()>-1e-7, 'Invalid odometry covariance'
        if args.mode=='frontend':
            for sensor in ['wheel', 'gnss', 'visual']:
                assert int(diag.get('backend.'+sensor+'_factors', '0'))==0, 'Duplicate backend fusion'
            if args.faults:
                assert int(diag['wheel.rejected'])>0 and int(diag['gnss.rejected'])>0, report
        else:
            for sensor in ['wheel', 'gnss', 'visual']:
                assert int(diag.get('backend.'+sensor+'_factors', '0'))>0, report
        save = node.create_client(Trigger, '/fastlio_mapping/save')
        assert save.wait_for_service(timeout_sec=1.)
        saved = save.call_async(Trigger.Request()); rclpy.spin_until_future_complete(node, saved, timeout_sec=5.)
        assert saved.done() and saved.result().success, 'Map export failed'
        report['map_exported']=(Path(temp.name)/'export/map.pcd').exists()
        assert report['map_exported'], 'Map PCD absent'
        report['passed']=True
        Path(args.output).write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        log.close()
        Path(args.output+'.node.log').write_text((Path(temp.name)/'node.log').read_text())
        node.destroy_node(); rclpy.shutdown(); temp.cleanup()

if __name__ == '__main__':
    main()
