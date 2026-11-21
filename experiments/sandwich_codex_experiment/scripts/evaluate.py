"""Summarize recorded evidence, without inferring missing tactile forces."""
import csv
import json
from pathlib import Path
import sys
import math
from collections import defaultdict, Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

out=Path(sys.argv[1]);report=json.loads((out/'result.json').read_text())
poses=list(csv.DictReader((out/'object_poses.csv').open()))
fig,ax=plt.subplots(2,1,figsize=(10,7),sharex=True)
for n in ['bread_bottom','tomato','cheese','bread_top']:
    rows=[r for r in poses if r['item']==n]
    ts=[float(r['time_s']) for r in rows]
    ax[0].plot(ts,[float(r['y']) for r in rows],label=n)
    ax[1].plot(ts,[float(r['z']) for r in rows],label=n)
ax[0].set_ylabel('World Y (m)');ax[1].set_ylabel('World Z (m)');ax[1].set_xlabel('Simulation time (s)')
ax[0].legend(ncol=2);ax[0].set_title(out.name+(': PASS' if report['success'] else ': FAIL')+' — measured Gazebo ingredient poses')
for a in ax:a.grid(alpha=.3)
fig.tight_layout();fig.savefig(out/'ingredient_motion.png',dpi=150);plt.close(fig)
feedback=out/'controller_feedback.csv'
summary=dict(success=report['success'],failure=report.get('error'),
    force_measurement_available=False,contact_evidence='collision names and positions; bridged wrench arrays absent',
    attachment=False,transfers=[])
finger_counts=Counter();last_fingers=defaultdict(dict);wrench_rows=0;contact_rows=0
strict_unwanted=Counter()
with (out/'contacts.csv').open() as f:
    for row in csv.DictReader(f):
        d=json.loads(row['contact_json']);contact_rows+=1;wrench_rows+=bool(d['wrenches'])
        if row['sensor'] in ['table','source_plate','destination_plate','cheese_rest','tomato_rest']:
            for robot in [row['collision1'],row['collision2']]:
                if robot.startswith('gen3::') and not robot.startswith('gen3::base_link::'):
                    strict_unwanted[(row['stage'],robot,row['sensor'])]+=1
        for side in ['left','right']:
            if side+'_finger_tip' in row['collision1']+row['collision2']:
                key=(row['sensor'],row['stage']);finger_counts[(row['sensor'],row['stage'],side)]+=1
                last_fingers[key][side]=float(row['time_s'])
summary['force_measurement_available']=wrench_rows>0
summary['contact_rows']=contact_rows;summary['contact_rows_with_wrenches']=wrench_rows
summary['strict_unwanted_robot_support_contact_rows']=sum(strict_unwanted.values())
summary['strict_unwanted_robot_support_pairs']=[dict(stage=k[0],robot_collision=k[1],support=k[2],rows=v) for k,v in strict_unwanted.items()]
summary['final_stack_tilt_rad']={n:2*math.acos(min(1.,math.sqrt(p['qz']**2+p['qw']**2))) for n,p in report.get('final_ingredient_poses',{}).items()}
if feedback.exists():
    rows=list(csv.DictReader(feedback.open()));arm=[abs(float(r['error_rad'])) for r in rows if r['joint'].startswith('joint_')]
    summary['arm_tracking_error_rad']=dict(max=max(arm),p95=float(np.percentile(arm,95))) if arm else None
for r in report['transfers']:
    item=dict(item=r['item'],passed=r['passed'],bilateral_before_lift=r.get('bilateral_before_lift',False))
    if 'lifted' in r:item['observed_lift_m']=r['lifted']['z']-r['initial']['z']
    for k in ['xy_error_m','z_error_m','tilt_rad','released','settle_position_range_m']:item[k]=r.get(k)
    stage=r['item']+':transport';ts=[float(p['time_s']) for p in poses if p['stage']==stage]
    finger=last_fingers[(r['item'],stage)]
    item['bilateral_contact_at_transport_end']=bool(ts) and all(side in finger and max(ts)-finger[side]<.2 for side in ['left','right'])
    summary['transfers'].append(item)
rate=out/'runtime.csv'
if rate.exists():
    rows=list(csv.DictReader(rate.open()))
    summary['observed_average_real_time_factor']=(float(rows[-1]['time_s'])-float(rows[0]['time_s']))/(float(rows[-1]['wall_monotonic_s'])-float(rows[0]['wall_monotonic_s']))
(out/'evidence_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
if strict_unwanted:raise SystemExit('Independent recorded-contact verification failed')
