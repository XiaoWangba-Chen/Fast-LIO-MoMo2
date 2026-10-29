#!/usr/bin/env python3
"""Plot the recorded position-only validation; never fit trajectory scale."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate import load,estimate_at_imu

NAMES={'lio':'LIO','degeneracy':'Degeneracy','wheel_frontend':'Wheel / front',
       'stereo_frontend':'Stereo / front','all_frontend':'Wheel+stereo / front',
       'all_backend':'Wheel+stereo / back','all_backend_no_loop':'Back / loop off'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('results');p.add_argument('reference')
    p.add_argument('--transforms',required=True);p.add_argument('--output',required=True);args=p.parse_args()
    root=Path(args.results);reference=load(args.reference)
    transform=yaml.safe_load(Path(args.transforms).read_text())['imu_T_base']
    fig,(trajectory,bar)=plt.subplots(1,2,figsize=(13,5.5))
    labels,errors,colors=[],[],[]
    first,last=float('inf'),float('-inf')
    for index,(name,label) in enumerate(NAMES.items()):
        folder=root/name;metrics=json.loads((folder/'metrics_position.json').read_text())
        topic='/odometry/global' if name.startswith('all_backend') else '/odometry/local'
        estimate=estimate_at_imu(load(folder/'odometry',topic),transform)
        t=estimate[:,0];first=min(first,t[0]);last=max(last,t[-1])
        target=np.column_stack([np.interp(t,reference[:,0],reference[:,i]) for i in range(1,4)])
        source=estimate[:,1:4];u,_,vt=np.linalg.svd((source-source.mean(0)).T@(target-target.mean(0)))
        correction=np.eye(3);correction[2,2]=np.linalg.det(vt.T@u.T);rotation=vt.T@correction@u.T
        aligned=(rotation@(source-source.mean(0)).T).T+target.mean(0)
        color=plt.get_cmap('tab10')(index);trajectory.plot(aligned[:,0],aligned[:,1],linewidth=1,label=label,color=color)
        labels.append(label);errors.append(metrics['ate_rmse_m']);colors.append(color)
    # Show only the portion actually replayed, not the rest of the full reference.
    window=reference[(reference[:,0]>=first)&(reference[:,0]<=last)]
    trajectory.plot(window[:,1],window[:,2],'k--',linewidth=1.4,label='Provided reference')
    trajectory.set_aspect('equal',adjustable='datalim');trajectory.set_xlabel('Reference-frame x / m');trajectory.set_ylabel('Reference-frame y / m')
    trajectory.grid(alpha=.25);trajectory.legend(fontsize=7,loc='best')
    positions=np.arange(len(labels));bars=bar.barh(positions,errors,color=colors);bar.set_yticks(positions,labels);bar.invert_yaxis();bar.set_xlabel('Position ATE RMSE / m');bar.grid(axis='x',alpha=.25)
    for rect,value in zip(bars,errors):bar.text(value+.02,rect.get_y()+rect.get_height()/2,f'{value:.3f}',va='center',fontsize=8)
    bar.set_xlim(0,max(errors)*1.25)
    fig.suptitle('FusionPortableV2 ugv_parking00: first 60 s, SE(3) alignment, scale = 1')
    fig.text(.5,.015,'Conditional position comparison: reference point assumed IMU; reference orientation and independence unverified. GNSS disabled.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.04,1,.95));fig.savefig(args.output,dpi=150);plt.close(fig)

if __name__=='__main__':main()
