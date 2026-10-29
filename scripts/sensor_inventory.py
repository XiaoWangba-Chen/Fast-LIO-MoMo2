#!/usr/bin/env python3
"""Inspect raw IMU, GNSS and static transforms without ROS1."""
import argparse
import itertools
import json
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores,get_typestore

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--output',required=True)
    args=p.parse_args();report={'gnss':{},'imu':{},'static_transforms':{}}
    with AnyReader([Path(args.bag)],default_typestore=get_typestore(Stores.ROS1_NOETIC)) as reader:
        for c in reader.connections:
            if c.msgtype not in ['sensor_msgs/msg/Imu','sensor_msgs/msg/NavSatFix','tf2_msgs/msg/TFMessage']:continue
            count=500 if c.msgtype=='sensor_msgs/msg/NavSatFix' else 100 if c.msgtype=='tf2_msgs/msg/TFMessage' else 1
            messages=[reader.deserialize(raw,c.msgtype) for _,_,raw in itertools.islice(reader.messages(connections=[c]),count)]
            if not messages:continue
            if c.msgtype=='sensor_msgs/msg/Imu':
                m=messages[0];report['imu'][c.topic]={'frame':m.header.frame_id,'acceleration':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]}
            elif c.msgtype=='sensor_msgs/msg/NavSatFix':
                first=messages[0];report['gnss'][c.topic]={'frame':first.header.frame_id,'samples':len(messages),'status_counts':{str(s):sum(m.status.status==s for m in messages) for s in set(m.status.status for m in messages)},'first_covariance':list(first.position_covariance),'covariance_type':int(first.position_covariance_type)}
            else:
                for m in messages:
                    for t in m.transforms:
                        report['static_transforms'][t.header.frame_id+'->'+t.child_frame_id]={'translation':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],'quaternion':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]}
    Path(args.output).write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
