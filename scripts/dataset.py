#!/usr/bin/env python3
"""Inspect, download and normalize real bags without ROS1 installation.

Cloud conversion requires an explicit timestamp convention. No timestamp is
fabricated from azimuth or a nominal scan rate. The selected point field must
already contain per-point acquisition offsets.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import urllib.request
import numpy as np
import yaml


def checksum(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def download(args):
    catalog = yaml.safe_load(Path(args.catalog).read_text())
    entry = next(e for e in catalog['sources'][args.source]['files'] if e['name'] == args.name)
    if not entry['bytes']:
        raise ValueError('Unknown file size: inspect official metadata and populate bytes before download')
    destination = Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    used = sum(p.stat().st_size for p in destination.rglob('*') if p.is_file())
    if used + entry['bytes'] > catalog['budget_bytes']:
        raise ValueError('Dataset budget would exceed 30 GB')
    if shutil.disk_usage(destination).free < 2 * entry['bytes']:
        raise ValueError('Insufficient space for archive and extraction')
    output = destination / entry['name']
    partial = output.with_suffix(output.suffix + '.partial')
    if output.exists():
        raise FileExistsError(output)
    if 'drive.google.com' in entry['url']:
        import gdown
        if gdown.download(entry['url'], str(partial), quiet=False, resume=True) is None:
            raise RuntimeError('Official Google Drive download unavailable')
    else:
        with urllib.request.urlopen(entry['url'], timeout=60) as src, partial.open('wb') as dst:
            while block := src.read(1024 * 1024):
                dst.write(block)
                if dst.tell() > entry['bytes']:
                    raise ValueError('Response exceeds declared size')
    if partial.stat().st_size != entry['bytes']:
        raise ValueError('Downloaded file size differs from official metadata')
    digest = checksum(partial)
    if entry.get('sha256') and digest != entry['sha256']:
        raise ValueError('SHA256 mismatch')
    partial.rename(output)
    # A locally computed hash is provenance, not an independently verified hash.
    print(json.dumps({'path': str(output), 'sha256_local': digest,
                      'sha256_verified': bool(entry.get('sha256'))}, indent=2))


def inspect(args):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    with AnyReader([Path(args.input)], default_typestore=get_typestore(Stores.ROS2_HUMBLE if Path(args.input).is_dir() else Stores.ROS1_NOETIC)) as reader:
        topics = []
        for c in reader.connections:
            first = next(reader.messages(connections=[c]), None)
            sample = {}
            if first:
                msg = reader.deserialize(first[2], c.msgtype)
                if hasattr(msg, 'fields'):
                    sample['point_fields'] = [{'name': f.name, 'datatype': f.datatype,
                                              'offset': f.offset} for f in msg.fields]
                if hasattr(msg, 'header'):
                    sample['frame_id'] = msg.header.frame_id
            topics.append({'topic': c.topic, 'type': c.msgtype, 'count': c.msgcount, **sample})
        print(json.dumps({'start_ns': reader.start_time, 'end_ns': reader.end_time,
                          'topics': topics}, indent=2))


def normalize_cloud(msg, store, field, unit, convention):
    sizes = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    endian = '>' if msg.is_bigendian else '<'
    fields = {f.name: f for f in msg.fields}
    for name in ['x', 'y', 'z', field]:
        if name not in fields:
            raise ValueError(f'Required point field absent: {name}')
    dtype = np.dtype({'names': list(fields),
                      'formats': [endian + sizes[f.datatype] for f in fields.values()],
                      'offsets': [f.offset for f in fields.values()], 'itemsize': msg.point_step})
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step)).reshape(-1)
    units = {'s': 1., 'ms': 1e-3, 'us': 1e-6, 'ns': 1e-9}
    times = points[field].astype(np.float64) * units[unit]
    if not len(times) or not np.all(np.isfinite(times)):
        raise ValueError('Missing or invalid per-point offsets')
    if times.min() < 0 or times.max() > 1:
        raise ValueError('Expected relative acquisition offsets in [0,1] seconds')
    order = np.argsort(times, kind='stable')
    minimum, maximum = float(times.min()), float(times.max())
    stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    if convention == 'end':
        stamp_ns -= round((maximum - minimum) * 1e9)
    elif convention == 'start':
        stamp_ns += round(minimum * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    packed = np.zeros(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                         ('intensity', '<f4'), ('time', '<f4'), ('ring', '<u2')])
    for name in ['x', 'y', 'z', 'intensity', 'ring']:
        if name in fields:
            packed[name] = points[name][order]
    packed['time'] = (times[order] - minimum).astype(np.float32)
    PointField = store.types['sensor_msgs/msg/PointField']
    msg.fields = [PointField(name, packed.dtype.fields[name][1], 4 if name == 'ring' else 7, 1)
                  for name in packed.dtype.names]
    msg.height, msg.width = 1, len(points)
    msg.point_step, msg.row_step = packed.dtype.itemsize, packed.nbytes
    msg.is_bigendian = False
    msg.data = np.frombuffer(packed.tobytes(), dtype=np.uint8)
    return msg


def convert(args):
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore
    from rosbags.convert.converter import migrate_message, default_message
    store = get_typestore(Stores.ROS2_HUMBLE)
    mappings = dict(pair.split(':=', 1) for pair in args.topic)
    cache = {}
    wheel_rows = np.loadtxt(args.wheel_csv, delimiter=',', skiprows=1, ndmin=2) if args.wheel_csv else np.empty((0,3))
    if len(wheel_rows) and (wheel_rows.shape[1]!=3 or not np.isfinite(wheel_rows).all() or np.any(np.diff(wheel_rows[:,0])<=0) or np.any(wheel_rows[:,2]<=0)):
        raise ValueError('Wheel CSV must contain increasing timestamp,speed,positive_variance')
    with AnyReader([Path(args.input)], default_typestore=get_typestore(Stores.ROS2_HUMBLE if Path(args.input).is_dir() else Stores.ROS1_NOETIC)) as reader:
        connections = [c for c in reader.connections if c.topic in mappings]
        missing = set(mappings) - {c.topic for c in connections}
        if missing:
            raise ValueError(f'Missing requested topics: {missing}')
        if not connections:
            raise ValueError('Select explicit --topic source:=destination mappings')
        with Writer(Path(args.output), version=8) as writer:
            converted = {}
            camera_info={};camera_connections={}
            for mapping in getattr(args,'camera_info',[]):
                image_topic,filename=mapping.split(':=',1)
                data=yaml.safe_load(Path(filename).read_text());info=default_message(store,'sensor_msgs/msg/CameraInfo')
                info.width=int(data['image_width']);info.height=int(data['image_height']);info.distortion_model=data['distortion_model']
                for attr,key in [('k','camera_matrix'),('d','distortion_coefficients'),('r','rectification_matrix'),('p','projection_matrix')]:
                    setattr(info,attr,np.asarray(data[key]['data'],dtype=np.float64))
                camera_info[image_topic]=info
                camera_connections[image_topic]=writer.add_connection(image_topic.rsplit('/',1)[0]+'/camera_info','sensor_msgs/msg/CameraInfo',typestore=store)
            wheel_connection = writer.add_connection('/wheel/twist', 'geometry_msgs/msg/TwistWithCovarianceStamped', typestore=store) if len(wheel_rows) else None
            wheel_index = 0
            def write_wheel(row):
                msg = default_message(store, 'geometry_msgs/msg/TwistWithCovarianceStamped')
                timestamp = round(row[0]*1e9)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(timestamp,10**9)
                msg.header.frame_id='base_link';msg.twist.twist.linear.x=float(row[1]);msg.twist.covariance[0]=row[2]
                writer.write(wheel_connection,timestamp,store.serialize_cdr(msg,'geometry_msgs/msg/TwistWithCovarianceStamped'))
            for c in connections:
                if c.msgtype not in store.types:
                    raise ValueError(f'Unsupported custom type {c.msgtype}; adapt explicitly first')
                destination_type = 'sensor_msgs/msg/Image' if args.decompress_images and c.msgtype=='sensor_msgs/msg/CompressedImage' else c.msgtype
                converted[c.id] = writer.add_connection(mappings[c.topic], destination_type, typestore=store)
            first_ns = reader.start_time
            for c, timestamp, raw in reader.messages(connections=connections):
                if args.seconds and timestamp > first_ns + args.seconds * 1e9:
                    break
                while wheel_index<len(wheel_rows) and round(wheel_rows[wheel_index,0]*1e9)<=timestamp:
                    if wheel_rows[wheel_index,0]*1e9>=first_ns: write_wheel(wheel_rows[wheel_index])
                    wheel_index+=1
                source = reader.deserialize(raw, c.msgtype)
                msg = migrate_message(reader.typestore, store, c.msgtype, c.msgtype, cache, source)
                if c.msgtype == 'sensor_msgs/msg/PointCloud2' and args.point_time_field:
                    msg = normalize_cloud(msg, store, args.point_time_field, args.point_time_unit,
                                          args.lidar_header)
                if args.decompress_images and c.msgtype=='sensor_msgs/msg/CompressedImage':
                    import cv2
                    image=cv2.imdecode(msg.data,cv2.IMREAD_GRAYSCALE)
                    if image is None: raise ValueError('Invalid compressed image')
                    out=default_message(store,'sensor_msgs/msg/Image');out.header=msg.header
                    out.height,out.width=image.shape;out.encoding='mono8';out.step=out.width
                    out.data=np.frombuffer(image.tobytes(),dtype=np.uint8);msg=out
                writer.write(converted[c.id], timestamp, store.serialize_cdr(msg, converted[c.id].msgtype))
                if converted[c.id].topic in camera_info:
                    info=camera_info[converted[c.id].topic];info.header=msg.header
                    writer.write(camera_connections[converted[c.id].topic],timestamp,store.serialize_cdr(info,'sensor_msgs/msg/CameraInfo'))
    print('Converted. Normalized clouds use lidar_type=2 and timestamp_unit=0.')


def wheel(args):
    # CSV timestamp,left_ticks,right_ticks; output is independent encoder speed.
    data = np.loadtxt(args.input, delimiter=',', skiprows=1, ndmin=2)
    dt = np.diff(data[:, 0])
    if np.any(dt <= 0):
        raise ValueError('Encoder timestamps must increase strictly')
    ticks = np.diff(data[:, 1:3], axis=0)
    if args.counter_modulus:
        ticks = (ticks + args.counter_modulus/2) % args.counter_modulus - args.counter_modulus/2
    distance = ticks * (2*np.pi*args.radius/args.ticks_per_revolution)
    speed = distance.mean(axis=1)/dt
    np.savetxt(args.output, np.column_stack((data[1:, 0], speed,
                                           np.full(len(speed), args.variance))),
               delimiter=',', header='timestamp,speed,variance', comments='')


def calibration(args):
    # Deliberately explicit: input transforms must already have documented directions.
    data = yaml.safe_load(Path(args.input).read_text())
    from scipy.spatial.transform import Rotation
    output = {}
    for name in ['imu_T_base', 'imu_T_camera', 'imu_T_gnss', 'imu_T_lidar']:
        matrix = np.asarray(data[name], dtype=float).reshape(4, 4)
        rotation = matrix[:3, :3]
        if not np.allclose(matrix[3], [0, 0, 0, 1]) or not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-5) or np.linalg.det(rotation) < .999:
            raise ValueError(f'{name} must be a rigid transform')
        if name == 'imu_T_lidar':
            output['mapping.extrinsic_T'] = matrix[:3, 3].tolist()
            output['mapping.extrinsic_R'] = rotation.reshape(-1).tolist()
        else:
            output['extrinsics.'+name+'.translation'] = matrix[:3, 3].tolist()
            output['extrinsics.'+name+'.quaternion'] = Rotation.from_matrix(rotation).as_quat().tolist()
    Path(args.output).write_text(yaml.safe_dump({'/**': {'ros__parameters': output}}, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('download'); p.add_argument('--catalog', default=str(Path(__file__).with_name('datasets.yaml')))
    p.add_argument('--source', required=True); p.add_argument('--name', required=True); p.add_argument('--output', default='data'); p.set_defaults(run=download)
    p = sub.add_parser('inspect'); p.add_argument('input'); p.set_defaults(run=inspect)
    p = sub.add_parser('convert'); p.add_argument('input'); p.add_argument('output'); p.add_argument('--topic', action='append', default=[])
    p.add_argument('--seconds', type=float, default=0); p.add_argument('--point-time-field')
    p.add_argument('--wheel-csv'); p.add_argument('--decompress-images', action='store_true')
    p.add_argument('--camera-info', action='append', default=[], help='output_image_topic:=camera_calibration.yaml')
    p.add_argument('--point-time-unit', choices=['s', 'ms', 'us', 'ns'], default='ns')
    p.add_argument('--lidar-header', choices=['start', 'end'], default='start'); p.set_defaults(run=convert)
    p = sub.add_parser('wheel'); p.add_argument('input'); p.add_argument('output'); p.add_argument('--radius', type=float, required=True)
    p.add_argument('--ticks-per-revolution', type=float, required=True); p.add_argument('--variance', type=float, default=.04)
    p.add_argument('--counter-modulus', type=int); p.set_defaults(run=wheel)
    p = sub.add_parser('calibration'); p.add_argument('input'); p.add_argument('output'); p.set_defaults(run=calibration)
    args = parser.parse_args(); args.run(args)

if __name__ == '__main__':
    main()
