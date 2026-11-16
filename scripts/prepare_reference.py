#!/usr/bin/env python3
"""Normalize trusted drag pixels into a labelled 30 cm tabletop target reference.

This does not recover camera geometry, issue commands, or produce expert actions.
"""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    src = ROOT / 'data/raw/human_demonstration_phases.csv'
    with src.open(newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    drag = [r for r in rows if r['phase_approx'] == 'Drag'
            and 'unreliable' not in r['cup_quality_note'] and float(r['template_score']) >= 0.7]
    if len(drag) < 10:
        raise ValueError('Too few trusted drag samples')
    pixels = np.array([[float(r['cup_x_full_px']), float(r['cup_y_full_px'])] for r in drag])
    # Median-filter image noise, then project onto the observed 2D drag direction.
    smooth = np.array([np.median(pixels[max(0,i-2):min(len(pixels),i+3)],axis=0)
                       for i in range(len(pixels))])
    direction = smooth[-1] - smooth[0]
    if np.linalg.norm(direction) < 20:
        raise ValueError('Unresolved cup displacement in selected interval')
    progress = np.maximum.accumulate(np.clip((smooth - smooth[0]) @ direction / (direction @ direction),0,1))
    progress[0],progress[-1] = 0,1
    times = np.array([float(r['time_s']) for r in drag])
    output = []
    for row in rows:
        t = float(row['time_s'])
        p = float(np.interp(t,times,progress))
        approach = np.clip((t-1.9)/(4.25-1.9),0,1)
        approach = approach*approach*(3-2*approach)
        z = 0.75 - 0.135*approach
        close = np.clip((t-4.25)/(5.35-4.25),0,1)
        opening = np.clip((t-7.9)/(8.35-7.9),0,1)
        q = 0.1 + 0.3*close*(1-opening)
        if t >= 8.35:
            retract = np.clip((t-8.35)/(10.4-8.35),0,1)
            z = 0.615+0.135*retract*retract*(3-2*retract)
        output.append(dict(source_frame=row['frame'],source_time_s=t,phase_approx=row['phase_approx'],
            target_x_m=0.55,target_y_m=0.30*p,target_z_m=float(z),
            qx=1.0,qy=0.0,qz=0.0,qw=0.0,gripper_driver_rad=float(q),
            coordinate_frame='world',target_link='end_effector_link',
            source_quality=row['cup_quality_note'],
            representation='heuristic_target_reference_not_robot_actions'))
    out = ROOT / 'data/derived/tabletop_reference.csv'
    with out.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(output[0]));writer.writeheader();writer.writerows(output)
    meta = dict(source=str(src),source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
        selected_drag_samples=len(drag),source_drag_interval_s=[float(times[0]),float(times[-1])],
        requested_displacement_m=0.30,world_direction=[0,1,0],
        normalization='5-sample median; 2D endpoint projection; clip and monotonic progress; scale to 0.30 m',
        excluded='All samples outside Drag are excluded from cup-motion estimation, including the 9.5 s jump',
        limitations=['No homography/camera calibration','No recovered human 6DoF pose',
                    'Approach/retract geometry, gripper targets and orientation are robot-specific assumptions',
                    'Not collision-checked or executed; not an expert dataset'],
        nominal_human_time_preserved=True,robot_execution_time_scale=None)
    (ROOT / 'data/derived/reference_metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta,indent=2))


if __name__=='__main__':main()
