#!/usr/bin/env python3
"""Publish standard ROS camera-calibration YAML as CameraInfo at image timestamps."""
import argparse
from pathlib import Path
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def read_info(path):
    data = yaml.safe_load(Path(path).read_text())
    msg = CameraInfo()
    msg.width, msg.height = int(data['image_width']), int(data['image_height'])
    msg.distortion_model = data.get('distortion_model', 'plumb_bob')
    for field, key, size in [('k', 'camera_matrix', 9), ('r', 'rectification_matrix', 9),
                             ('p', 'projection_matrix', 12)]:
        values = [float(v) for v in data[key]['data']]
        if len(values) != size:
            raise ValueError(f'{key} requires {size} entries')
        setattr(msg, field, values)
    msg.d = [float(v) for v in data['distortion_coefficients']['data']]
    return msg


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--left', required=True); p.add_argument('--right', required=True)
    p.add_argument('--left-image', default='/camera/left/image_raw')
    p.add_argument('--right-image', default='/camera/right/image_raw')
    p.add_argument('--left-info', default='/camera/left/camera_info')
    p.add_argument('--right-info', default='/camera/right/camera_info')
    args = p.parse_args()
    rclpy.init(); node = rclpy.create_node('fastlio_camera_calibration')
    subscriptions = []
    for path, image_topic, info_topic in [(args.left, args.left_image, args.left_info),
                                          (args.right, args.right_image, args.right_info)]:
        info = read_info(path)
        publisher = node.create_publisher(CameraInfo, info_topic, 10)
        def callback(image, info=info, publisher=publisher):
            if image.width != info.width or image.height != info.height:
                node.get_logger().error('Image dimensions do not match calibration')
                return
            info.header = image.header; publisher.publish(info)
        subscriptions.append(node.create_subscription(Image, image_topic, callback, qos_profile_sensor_data))
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
