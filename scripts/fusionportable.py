#!/usr/bin/env python3
"""Import FusionPortableV2's published UGV calibration and wheel counter model.

Sources:
https://github.com/fusionportable/calibration_files/tree/master/20230426_calib
https://github.com/fusionportable/fusionportable_dataset_tools/tree/main/calibration/encoder_calc
"""
import argparse
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def transform(data, target):
    q=np.asarray(data['quaternion_sensor_'+target]['data'],dtype=float)
    t=np.asarray(data['translation_sensor_'+target]['data'],dtype=float)
    if q.shape!=(4,) or t.shape!=(3,):raise ValueError('Missing calibrated transform '+target)
    pose=np.eye(4);pose[:3,:3]=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();pose[:3,3]=t
    return pose


def calibrate(args):
    import cv2
    source=Path(args.input);output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    data={name:yaml.safe_load((source/(name+'.yaml')).read_text())
          for name in ['body_imu','ouster00','frame_cam00','frame_cam01']}
    left,right=data['frame_cam00'],data['frame_cam01']
    K1=np.array(left['camera_matrix']['data']).reshape(3,3)
    K2=np.array(right['camera_matrix']['data']).reshape(3,3)
    D1=np.array(left['distortion_coefficients']['data']);D2=np.array(right['distortion_coefficients']['data'])
    q=np.asarray(right['quaternion_stereo']['data']);R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
    T=np.asarray(right['translation_stereo']['data'],dtype=float)
    size=(int(left['image_width']),int(left['image_height']))
    R1,R2,P1,P2,_,_,_=cv2.stereoRectify(K1,D1,K2,D2,size,R,T,flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    for camera,rectification,projection,name in [(left,R1,P1,'left'),(right,R2,P2,'right')]:
        camera['rectification_matrix']['data']=rectification.reshape(-1).tolist()
        camera['projection_matrix']['data']=projection.reshape(-1).tolist()
        (output/(name+'.yaml')).write_text(yaml.safe_dump(camera,sort_keys=False))
    imu_T_lidar=np.linalg.inv(transform(data['ouster00'],'body_imu'))
    imu_T_camera=np.linalg.inv(transform(left,'body_imu'))
    imu_T_base=transform(data['body_imu'],'mini_hercules_wheel')
    imu=data['body_imu'];rate=float(imu['update_rate'])
    params={
        'common.lid_topic':'/points_raw','common.imu_topic':'/imu/data',
        'common.time_sync_en':False,'common.reliable_inputs':True,'common.time_offset_lidar_to_imu':0.,
        'preprocess.lidar_type':2,'preprocess.timestamp_unit':0,'preprocess.scan_line':128,
        'preprocess.scan_rate':10,'preprocess.blind':.5,'point_filter_num':3,
        'max_iteration':3,'cube_side_length':1000.,'filter_size_surf':.3,'filter_size_map':.3,'mapping.fov_degree':360.,'mapping.det_range':100.,
        'mapping.extrinsic_est_en':False,'mapping.extrinsic_T':imu_T_lidar[:3,3].tolist(),
        'mapping.extrinsic_R':imu_T_lidar[:3,:3].reshape(-1).tolist(),
        'mapping.gyr_cov':float(imu['gyroscope_noise_density'])**2*rate,
        'mapping.acc_cov':float(imu['accelerometer_noise_density'])**2*rate,
        'mapping.b_gyr_cov':float(imu['gyroscope_random_walk'])**2*rate,
        'mapping.b_acc_cov':float(imu['accelerometer_random_walk'])**2*rate,
        'stereo.left_time_offset':float(left['timeshift_sensor_body_imu']),
        'stereo.right_time_offset':float(right['timeshift_sensor_body_imu']),
        'stereo.max_clones':6,'fusion.mode':'backend','wheel.enabled':True,'stereo.enabled':True,'gnss.enabled':False,
        'backend.enabled':True,'backend.loop_enabled':True,'pcd_save.pcd_save_en':False,
        'publish.path_en':False,'publish.scan_publish_en':False,
    }
    for name,pose in [('imu_T_base',imu_T_base),('imu_T_camera',imu_T_camera)]:
        params['extrinsics.'+name+'.translation']=pose[:3,3].tolist()
        params['extrinsics.'+name+'.quaternion']=Rotation.from_matrix(pose[:3,:3]).as_quat().tolist()
    (output/'rig.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False))
    (output/'transforms.yaml').write_text(yaml.safe_dump({
        'imu_T_base':imu_T_base.tolist(),'imu_T_camera':imu_T_camera.tolist(),
        'imu_T_lidar':imu_T_lidar.tolist(),'imu_T_3dm':transform(imu,'3dm_imu').tolist()},sort_keys=False))
    print('Imported UGV calibration. GNSS stays disabled: published antenna lever arms are unspecified.')


def wheel(args):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores,get_typestore
    rows=[]
    with AnyReader([Path(p) for p in args.input],default_typestore=get_typestore(Stores.ROS1_NOETIC)) as reader:
        conns=[c for c in reader.connections if c.topic==args.topic]
        if not conns:raise ValueError('Encoder topic absent')
        for c,_,raw in reader.messages(connections=conns):
            msg=reader.deserialize(raw,c.msgtype)
            if len(msg.position)<2:raise ValueError('Encoder JointState requires left and right position counters')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            rows.append((stamp,float(msg.position[0]),float(msg.position[1])))
    rows=np.asarray(rows,dtype=float)
    if len(rows)<2 or not np.isfinite(rows).all():raise ValueError('Invalid encoder series')
    dt=np.diff(rows[:,0]);counts=np.diff(rows[:,1:3],axis=0)
    # The published reference implementation uses this non-integer wrap period.
    counts[counts>1024*1024]-=4294967.296;counts[counts< -1024*1024]+=4294967.296
    valid=(dt>0)&(dt<=.1)
    radius=np.array([args.left_radius,args.right_radius])
    velocity=(counts*2*np.pi*radius/1000).mean(axis=1)/np.maximum(dt,1e-9)
    valid&=np.isfinite(velocity)&(np.abs(velocity)<args.max_speed)
    # An interval-derived speed is observed at its midpoint.
    result=np.column_stack(((rows[1:,0]+rows[:-1,0])*.5,velocity,np.full(len(dt),args.variance)))
    np.savetxt(args.output,result[valid],delimiter=',',header='timestamp,speed,variance',comments='')
    print('Wheel samples:',int(valid.sum()),'rejected:',int((~valid).sum()))


def reference(args):
    poses=np.loadtxt(args.input,ndmin=2)
    transforms=yaml.safe_load(Path(args.transforms).read_text())
    imu_T_base=np.asarray(transforms['imu_T_base'])
    if args.frame=='base':ref_T_base=np.eye(4)
    elif args.frame=='imu':ref_T_base=imu_T_base
    elif args.frame=='lidar':ref_T_base=np.linalg.inv(np.asarray(transforms['imu_T_lidar']))@imu_T_base
    else:ref_T_base=np.linalg.inv(np.asarray(transforms['imu_T_3dm']))@imu_T_base
    for row in poses:
        T=np.eye(4);T[:3,:3]=Rotation.from_quat(row[4:8]).as_matrix();T[:3,3]=row[1:4]
        T=T@ref_T_base;row[1:4]=T[:3,3];row[4:8]=Rotation.from_matrix(T[:3,:3]).as_quat()
    np.savetxt(args.output,poses,fmt='%.9f')


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(required=True)
    c=sub.add_parser('calibrate');c.add_argument('input');c.add_argument('output');c.set_defaults(run=calibrate)
    c=sub.add_parser('wheel');c.add_argument('input',nargs='+');c.add_argument('--output',required=True)
    c.add_argument('--topic',default='/mini_hercules/encoder');c.add_argument('--left-radius',type=float,default=.217)
    c.add_argument('--right-radius',type=float,default=.217);c.add_argument('--variance',type=float,default=.04)
    c.add_argument('--max-speed',type=float,default=5.);c.set_defaults(run=wheel)
    c=sub.add_parser('reference');c.add_argument('input');c.add_argument('output');c.add_argument('--transforms',required=True)
    c.add_argument('--frame',choices=['imu','base','lidar','3dm'],required=True);c.set_defaults(run=reference)
    args=p.parse_args();args.run(args)

if __name__=='__main__':main()
