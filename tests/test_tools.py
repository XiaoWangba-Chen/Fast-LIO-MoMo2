#!/usr/bin/env python3
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import dataset
import evaluate

class ConfigurationTests(unittest.TestCase):
    def test_ros2_profiles_preserve_legacy_launch_controls(self):
        import xml.etree.ElementTree as ET
        import yaml
        for name in ['avia','horizon','mid360','ouster64','velodyne','marsim']:
            data=yaml.safe_load((ROOT/'config'/(name+'.yaml')).read_text())['/**']['ros__parameters']
            launch=ET.parse(ROOT/'launch'/('mapping_'+name+'.launch')).getroot()
            for parameter in launch.findall('param'):
                key=parameter.get('name');value=parameter.get('value')
                if key not in ['max_iteration','point_filter_num','filter_size_surf','filter_size_map']:continue
                self.assertAlmostEqual(float(data[key]),float(value),msg=name+':'+key)
            self.assertGreater(float(data['cube_side_length']),3*float(data['mapping']['det_range']),name)

class MetricsTests(unittest.TestCase):
    def test_se3_alignment_does_not_fit_scale(self):
        t = np.arange(0, 5, .05)
        positions = np.column_stack((t, np.sin(t), np.cos(.7*t)))
        reference = np.column_stack((t, positions, np.tile([0.,0.,0.,1.], (len(t),1))))
        R = Rotation.from_euler('z', .4).as_matrix()
        estimate = reference.copy(); estimate[:,1:4] = (R@positions.T).T+[3.,2.,1.]
        estimate[:,4:8] = Rotation.from_matrix(np.tile(R,(len(t),1,1))).as_quat()
        metrics = evaluate.evaluate(estimate, reference)
        self.assertLess(metrics['ate_rmse_m'],1e-10)
        self.assertGreater(metrics['rpe_pairs'],0)
        estimate[:,1:4]*=2
        self.assertGreater(evaluate.evaluate(estimate, reference)['ate_rmse_m'],.1)

    def test_position_only_reference_omits_rotation_metrics(self):
        t=np.arange(0,3,.05)
        trajectory=np.column_stack((t,t,np.sin(t),np.cos(t),np.tile([0.,0.,0.,1.],(len(t),1))))
        metrics=evaluate.evaluate(trajectory,trajectory,position_only=True)
        self.assertIsNone(metrics['rpe_rotation_rmse_deg'])
        self.assertIsNone(metrics['rpe_translation_rmse_m'])
        self.assertLess(metrics['rpe_position_displacement_rmse_m'],1e-10)

    def test_latest_estimate_of_an_epoch_is_used(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'trajectory.tum'
            rows=np.array([[0,0,0,0,0,0,0,1],[1,10,0,0,0,0,0,1],[1,1,0,0,0,0,0,1],[2,2,0,0,0,0,0,1]],dtype=float)
            np.savetxt(path,rows)
            result=evaluate.load(path)
            self.assertEqual(len(result),3);self.assertEqual(result[1,1],1)

    def test_no_overlap_rejected(self):
        trajectory = np.array([[t,0.,0.,0.,0.,0.,0.,1.] for t in [0,1,2]])
        other = trajectory.copy();other[:,0]+=10
        with self.assertRaises(ValueError):evaluate.evaluate(trajectory, other)

    def test_encoder_wrap_and_variance(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'ticks.csv';out=Path(tmp)/'wheel.csv'
            path.write_text('t,left,right\n0,65530,65530\n1,4,4\n')
            dataset.wheel(types.SimpleNamespace(input=path,output=out,radius=1.,ticks_per_revolution=100.,counter_modulus=65536,variance=.04))
            data=np.loadtxt(out,delimiter=',',skiprows=1,ndmin=2)
            self.assertAlmostEqual(data[0,1],.2*np.pi)
            self.assertEqual(data[0,2],.04)

@unittest.skipUnless(importlib.util.find_spec('rosbags'), 'Install scripts/requirements.txt for bag tests')
class BagTests(unittest.TestCase):
    def test_ros1_to_ros2_and_end_stamped_cloud(self):
        from rosbags.rosbag1 import Writer
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores,get_typestore
        from rosbags.convert.converter import default_message
        store=get_typestore(Stores.ROS1_NOETIC)
        with tempfile.TemporaryDirectory() as tmp:
            bag=Path(tmp)/'input.bag';output=Path(tmp)/'output'
            with Writer(bag) as writer:
                imu=default_message(store,'sensor_msgs/msg/Imu');imu.header.stamp.sec=1
                ic=writer.add_connection('/imu','sensor_msgs/msg/Imu',typestore=store)
                writer.write(ic,10**9,store.serialize_ros1(imu,'sensor_msgs/msg/Imu'))
                cloud=default_message(store,'sensor_msgs/msg/PointCloud2')
                cloud.header.stamp.sec=1;cloud.header.stamp.nanosec=100000000
                cloud.width=2;cloud.height=1;cloud.point_step=16;cloud.row_step=32
                Field=store.types['sensor_msgs/msg/PointField']
                cloud.fields=[Field('x',0,7,1),Field('y',4,7,1),Field('z',8,7,1),Field('t',12,6,1)]
                data=np.zeros(2,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('t','<u4')])
                data['x']=[1,2];data['t']=[0,100000000]
                cloud.data=np.frombuffer(data.tobytes(),dtype=np.uint8)
                cc=writer.add_connection('/cloud','sensor_msgs/msg/PointCloud2',typestore=store)
                writer.write(cc,1100000000,store.serialize_ros1(cloud,'sensor_msgs/msg/PointCloud2'))
            wheel=Path(tmp)/'wheel.csv';wheel.write_text('timestamp,speed,variance\n1.0,0.5,0.04\n')
            args=types.SimpleNamespace(input=bag,output=output,topic=['/imu:=/imu/data','/cloud:=/points_raw'],seconds=0,point_time_field='t',point_time_unit='ns',lidar_header='end',wheel_csv=wheel,decompress_images=False)
            with contextlib.redirect_stdout(io.StringIO()):dataset.convert(args)
            with AnyReader([output]) as reader:
                conn=next(c for c in reader.connections if c.topic=='/points_raw')
                _,_,raw=next(reader.messages(connections=[conn]));converted=reader.deserialize(raw,conn.msgtype)
                self.assertEqual(converted.header.stamp.sec,1)
                self.assertEqual(converted.header.stamp.nanosec,0)
                self.assertIn('time',[f.name for f in converted.fields])
                points=np.ndarray(2,dtype=np.dtype({'names':['time'],'formats':['<f4'],'offsets':[16],'itemsize':22}),buffer=converted.data)
                self.assertAlmostEqual(points['time'][1],.1,places=6)
                wc=next(c for c in reader.connections if c.topic=='/wheel/twist')
                _,_,raw=next(reader.messages(connections=[wc]));w=reader.deserialize(raw,wc.msgtype)
                self.assertEqual(w.header.frame_id,'base_link');self.assertEqual(w.twist.twist.linear.x,.5)
                with contextlib.redirect_stdout(io.StringIO()) as text:dataset.inspect(types.SimpleNamespace(input=output))
                self.assertEqual(len(json.loads(text.getvalue())['topics']),3)

if __name__=='__main__':unittest.main()
