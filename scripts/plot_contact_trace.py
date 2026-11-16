#!/usr/bin/env python3
"""Plot measured cup translation/height from a completed trial, including failures."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]


def main():
    report=json.loads((ROOT/'outputs/contact_trial.json').read_text())
    with (ROOT/'outputs/contact_trace.csv').open() as f:
        rows=[r for r in csv.DictReader(f) if r['cup_y_m'] and r['cup_z_m']]
    start=float(rows[0]['time_s']);y0=float(rows[0]['cup_y_m']);z0=float(rows[0]['cup_z_m'])
    t=[float(r['time_s'])-start for r in rows]
    y=[(float(r['cup_y_m'])-y0)*1000 for r in rows]
    z=[(float(r['cup_z_m'])-z0)*1000 for r in rows]
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,constrained_layout=True)
    axes[0].plot(t,y,color='#2563eb',label='Measured cup displacement')
    axes[0].axhline(300,color='#374151',linestyle='--',linewidth=1,label='Programmed 300 mm target')
    axes[0].set_ylabel('Cup +Y displacement (mm)');axes[0].legend(loc='upper left')
    axes[1].plot(t,z,color='#b45309');axes[1].set_ylabel('Cup center height change (mm)')
    axes[1].set_xlabel('Simulated time since first logged state (s)')
    for stage,color in [('drag','#2563eb'),('release','#16a34a'),('retract','#6b7280')]:
        points=[tt for tt,r in zip(t,rows) if r['stage']==stage]
        if points:
            for ax in axes:ax.axvspan(min(points),max(points),alpha=.08,color=color)
            axes[0].text((min(points)+max(points))/2,.03,stage,ha='center',fontsize=8,
                         transform=axes[0].get_xaxis_transform())
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('Programmed Gazebo contact trial — '+('PASS' if report.get('success') else 'FAIL'))
    fig.savefig(ROOT/'outputs/contact_motion.png',dpi=160)
    plt.close(fig)


if __name__=='__main__':main()
