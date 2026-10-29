#!/usr/bin/env python3
"""SE(3)-aligned TUM trajectory evaluation. Scale is never fitted."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load(path, topic=None, position_only=False):
    if topic:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores,get_typestore
        with AnyReader([Path(path)],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
            rows = []
            conns = [c for c in reader.connections if c.topic == topic]
            if not conns:
                raise ValueError(f'Missing odometry topic {topic}')
            for c, _, raw in reader.messages(connections=conns):
                msg = reader.deserialize(raw, c.msgtype)
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                rows.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                             p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            data = np.asarray(rows)
    else:
        data = np.loadtxt(path, comments='#', ndmin=2)
    if position_only and data.ndim==2 and data.shape[1]==4:
        data=np.column_stack((data,np.tile([0.,0.,0.,1.],(len(data),1))))
    if data.ndim != 2 or data.shape[1] != 8 or len(data) < 3 or not np.all(np.isfinite(data)):
        raise ValueError('Expected at least three finite TUM poses: t x y z qx qy qz qw')
    if np.any(np.diff(data[:, 0]) < 0):
        raise ValueError('Trajectory contains a session reset; evaluate sessions separately')
    # Backend factors can revise one epoch several times. Keep its final estimate.
    data=data[np.r_[np.diff(data[:,0])!=0,True]]
    if len(data)<3:raise ValueError('Fewer than three distinct pose epochs')
    return data


def estimate_at_imu(estimate,imu_T_base):
    base_T_imu=np.linalg.inv(np.asarray(imu_T_base,dtype=float).reshape(4,4))
    result=estimate.copy();rotations=Rotation.from_quat(result[:,4:8]).as_matrix()
    result[:,1:4]+=np.einsum('nij,j->ni',rotations,base_T_imu[:3,3])
    result[:,4:8]=Rotation.from_matrix(rotations@base_T_imu[:3,:3]).as_quat()
    return result


def evaluate(estimate, reference, max_gap=.1, interval=1., position_only=False):
    t = estimate[:, 0]
    idx = np.searchsorted(reference[:, 0], t)
    valid = (t >= reference[0,0]) & (t <= reference[-1,0])
    clipped = np.clip(idx, 1, len(reference)-1)
    # Exact reference timestamps do not require a short neighboring interval.
    exact = np.abs(reference[clipped, 0]-t) < 1e-6
    valid &= exact | ((reference[clipped, 0]-reference[clipped-1, 0]) <= max_gap+1e-9)
    est = estimate[valid]
    if len(est) < 3:
        raise ValueError('Insufficient timestamp overlap with reference')
    times = est[:, 0]
    ref_positions = np.column_stack([np.interp(times, reference[:, 0], reference[:, i])
                                     for i in range(1, 4)])
    ref_rotations = Slerp(reference[:, 0], Rotation.from_quat(reference[:, 4:8]))(times).as_matrix()
    x, y = est[:, 1:4], ref_positions
    u, _, vt = np.linalg.svd((x-x.mean(0)).T@(y-y.mean(0)))
    correction = np.eye(3); correction[2, 2] = np.linalg.det(vt.T@u.T)
    alignment = vt.T@correction@u.T
    offset = y.mean(0)-alignment@x.mean(0)
    aligned = (alignment@x.T).T+offset
    errors = np.linalg.norm(aligned-y, axis=1)
    rotations = alignment@Rotation.from_quat(est[:, 4:8]).as_matrix()
    pairs = np.searchsorted(times, times+interval)
    rpe_t, rpe_r = [], []
    for i, j in enumerate(pairs):
        if j >= len(times) or abs(times[j]-times[i]-interval) > max_gap:
            continue
        dp = aligned[j]-aligned[i] if position_only else rotations[i].T@(aligned[j]-aligned[i])
        gt = y[j]-y[i] if position_only else ref_rotations[i].T@(y[j]-y[i])
        rpe_t.append(np.linalg.norm(dp-gt))
        delta = (rotations[i].T@rotations[j]).T@(ref_rotations[i].T@ref_rotations[j])
        rpe_r.append(Rotation.from_matrix(delta).magnitude()*180/np.pi)
    return {'alignment': 'SE3', 'scale': 1.0, 'associated_poses': len(est),
            'association_fraction': len(est)/len(estimate),
            'duration_seconds': float(times[-1]-times[0]),
            'ate_rmse_m': float(np.sqrt(np.mean(errors**2))),
            'ate_median_m': float(np.median(errors)),
            'rpe_interval_s': interval, 'rpe_pairs': len(rpe_t),
            'position_only_reference': position_only,
            'rpe_translation_rmse_m': float(np.sqrt(np.mean(np.square(rpe_t)))) if rpe_t and not position_only else None,
            'rpe_position_displacement_rmse_m': float(np.sqrt(np.mean(np.square(rpe_t)))) if rpe_t and position_only else None,
            'rpe_rotation_rmse_deg': float(np.sqrt(np.mean(np.square(rpe_r)))) if rpe_r and not position_only else None}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('estimate'); p.add_argument('reference'); p.add_argument('--bag-topic')
    p.add_argument('--max-gap', type=float, default=.1); p.add_argument('--interval', type=float, default=1.)
    p.add_argument('--estimate-to-imu', help='YAML containing imu_T_base; convert published base_link poses to IMU poses')
    p.add_argument('--position-only', action='store_true', help='Reference has only t,x,y,z; omit relative-pose and rotation errors')
    p.add_argument('--reference-source', required=True, help='Document independent GT or shared GNSS/INS provenance')
    p.add_argument('--output', default='metrics.json')
    args = p.parse_args()
    estimate=load(args.estimate,args.bag_topic)
    if args.estimate_to_imu:
        import yaml
        estimate=estimate_at_imu(estimate,yaml.safe_load(Path(args.estimate_to_imu).read_text())['imu_T_base'])
    report = evaluate(estimate, load(args.reference, position_only=args.position_only), args.max_gap, args.interval, args.position_only)
    report['reference_source'] = args.reference_source
    report['estimate_point']='imu' if args.estimate_to_imu else 'published_pose_origin'
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

if __name__ == '__main__':
    main()
