#!/usr/bin/env python3
"""Summarize ROS2 output timing, queues and fusion audit counters from a run."""
import argparse
import json
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores,get_typestore

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--output',required=True)
    args=p.parse_args();streams={};diagnostics=[]
    with AnyReader([Path(args.bag)],default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        for connection,_,raw in reader.messages():
            m=reader.deserialize(raw,connection.msgtype)
            if connection.msgtype=='nav_msgs/msg/Odometry':
                streams.setdefault(connection.topic,[]).append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
            elif connection.msgtype=='diagnostic_msgs/msg/DiagnosticArray':
                for status in m.status:
                    if status.name=='fastlio/estimation':diagnostics.append({v.key:v.value for v in status.values})
    report={'streams':{},'diagnostics_samples':len(diagnostics)}
    for name,times in streams.items():
        t=np.asarray(times);dt=np.diff(t);positive=dt[dt>1e-8]
        report['streams'][name]={'messages':len(t),'start':float(t[0]),'end':float(t[-1]),
            'duration_s':float(t[-1]-t[0]),'backwards':int((dt<0).sum()),'same_epoch_revisions':int((dt==0).sum()),
            'median_rate_hz':float(1/np.median(positive)) if len(positive) else None,
            'max_interval_s':float(positive.max()) if len(positive) else None}
    for name in ['processing_ms','state_lag_ms','lidar_queue','imu_queue','aux_queue']:
        values=np.asarray([float(d[name]) for d in diagnostics if name in d])
        if len(values):report[name]={'p50':float(np.median(values)),'p95':float(np.percentile(values,95)),
            'maximum':float(values.max()),'last':float(values[-1]),'tail_mean':float(values[-20:].mean())}
    if diagnostics:
        report['final_counters']=diagnostics[-1]
        report['severe_lidar_samples']=sum(d.get('degeneracy.severe')=='1' for d in diagnostics)
        report['geometrically_degraded_samples']=sum(d.get('degeneracy.degraded')=='1' for d in diagnostics)
    report['no_sustained_backlog']=(report.get('lidar_queue',{}).get('tail_mean',1e9)<=2 and report.get('state_lag_ms',{}).get('p95',1e9)<=300)
    Path(args.output).write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
