#!/usr/bin/env python3
"""Evaluate raw model estimates and compare phase-relative motion, never poses."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
from geometry import angular_distance
from extract import write_csv

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'outputs/hand3d'
ACTIVE = ['Approach','Grip','Drag','Hold','Release','Retract']


def read_csv(path):
    with path.open(newline='',encoding='utf-8-sig') as f:return list(csv.DictReader(f))


def quaternion(row):return np.array([float(row[f'palm_q{a}']) for a in 'xyzw'])


def stats(values):
    a=np.asarray(values,dtype=float);a=a[np.isfinite(a)]
    if not len(a):return {'n':0}
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(np.max(a)))


def groups_of_missing(frames, active_only=True):
    runs=[];run=[]
    for row in frames:
        missing=int(row['hand_count'])==0 and (not active_only or row['phase_approx'] in ACTIVE)
        if missing:run.append(row)
        elif run:
            runs.append(run);run=[]
    if run:runs.append(run)
    return [dict(start_s=float(x[0]['time_s']),end_s=float(x[-1]['time_s']),frames=len(x),
                 duration_s=len(x)/30) for x in runs]


def primary_rows(folder):
    return {int(r['frame']):r for r in read_csv(folder/'hand_pose_flexion.csv') if r['is_primary']=='True'}


def robot_fk(rows, urdf):
    robot=ET.parse(urdf).getroot()
    by_child={j.find('child').get('link'):j for j in robot.findall('joint')}
    chain=[];link='end_effector_link'
    while link in by_child:
        j=by_child[link];chain.append(j);link=j.find('parent').get('link')
    chain.reverse();poses=[]
    for row in rows:
        transform=np.eye(4)
        for joint in chain:
            origin=joint.find('origin');local=np.eye(4)
            if origin is not None:
                local[:3,:3]=Rotation.from_euler('xyz',[float(v) for v in origin.get('rpy','0 0 0').split()]).as_matrix()
                local[:3,3]=[float(v) for v in origin.get('xyz','0 0 0').split()]
            transform=transform@local
            if joint.get('type') in ('revolute','continuous'):
                axis=np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                rot=np.eye(4);rot[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*float(row[joint.get('name')])).as_matrix()
                transform=transform@rot
        poses.append(transform)
    return np.asarray(poses)


def main():
    frames=read_csv(OUT/'video/frames.csv');primary=primary_rows(OUT/'video')
    metadata=json.loads((OUT/'video/metadata.json').read_text())
    static=primary_rows(OUT/'static')
    old=read_csv(ROOT/'data/raw/2026-11-15 20-59-05_hand_landmarks.csv')
    old_frames={int(r['frame']) for r in old}
    old_groups={}
    for row in old:
        old_groups.setdefault((int(row['frame']),row['hand_index']),{})[int(row['landmark_index'])]=[
            float(row['x_px']),float(row['y_px'])]
    landmark_data=np.load(OUT/'video/landmarks.npz');image_repeatability=[]
    for frame,is_primary,image in zip(landmark_data['frame'],landmark_data['primary'],landmark_data['image']):
        if not is_primary:continue
        pixels=image[:,:2]*[metadata['width'],metadata['height']]
        candidates=[g for (f,h),g in old_groups.items() if f==frame and len(g)==21]
        if not candidates:continue
        match=min(candidates,key=lambda g:np.linalg.norm(pixels[0]-g[0]))
        image_repeatability.extend(np.linalg.norm(pixels-np.array([match[i] for i in range(21)]),axis=1))
    diagnostics=[];last=None;quaternion_sign_changes=0
    for f in frames:
        index=int(f['frame']);row=dict(primary[index]);valid=row.get('orientation_valid')=='True'
        row.update(raw_step_angle_deg='',raw_angular_speed_deg_s='',world_wrist_step_model_mm='',
                   width_step_percent='',image_wrist_step_px='',orientation_jump=False,
                   image_jump=False,shape_jump=False,label_flip=False,identity_boundary=False,
                   static_agreement_deg='',filtered_step_angle_deg='')
        if valid:
            q=quaternion(row)
            contiguous=last is not None and index==int(last['frame'])+1 and row['track_id']==last['track_id']
            row['identity_boundary']=not contiguous
            if contiguous:
                prev=quaternion(last);dt=float(row['time_s'])-float(last['time_s'])
                step=angular_distance(prev,q);row['raw_step_angle_deg']=step
                row['raw_angular_speed_deg_s']=step/dt
                row['orientation_jump']=step>30
                row['label_flip']=row['handedness_raw']!=last['handedness_raw']
                if np.dot(q,prev)<0:quaternion_sign_changes+=1
                row['world_wrist_step_model_mm']=1000*np.linalg.norm([
                    float(row[f'wrist_mp_{a}_model_m'])-float(last[f'wrist_mp_{a}_model_m']) for a in 'xyz'])
                row['image_wrist_step_px']=float(np.hypot(float(row['image_wrist_x_px'])-float(last['image_wrist_x_px']),
                    float(row['image_wrist_y_px'])-float(last['image_wrist_y_px'])))
                row['image_jump']=row['image_wrist_step_px']>.08*np.hypot(metadata['width'],metadata['height'])
                row['width_step_percent']=100*abs(float(row['palm_width_model_m'])/float(last['palm_width_model_m'])-1)
                row['shape_jump']=row['width_step_percent']>25
            last=row.copy()
            other=static.get(index)
            if other and other.get('orientation_valid')=='True':
                distance=np.hypot(float(row['image_wrist_x_px'])-float(other['image_wrist_x_px']),
                                  float(row['image_wrist_y_px'])-float(other['image_wrist_y_px']))
                if distance < 80:row['static_agreement_deg']=angular_distance(q,quaternion(other))
        else:last=None
        diagnostics.append(row)

    # A separate display-only filter: no missing-data filling and no filtering
    # across identity changes, label changes or raw orientation jumps.
    history=[];previous_q=None;previous_filtered=None
    for row in diagnostics:
        if row.get('orientation_valid')!='True':history=[];previous_q=None;previous_filtered=None;continue
        if row['identity_boundary'] or row['orientation_jump'] or row['label_flip']:
            history=[];previous_q=None;previous_filtered=None
        q=quaternion(row)
        if previous_q is not None and np.dot(q,previous_q)<0:q=-q
        for a,v in zip('xyzw',q):row[f'palm_q{a}_hemisphere_continuous']=v
        previous_q=q;history.append(q);history=history[-5:]
        smoothed=Rotation.from_quat(np.array(history)).mean().as_quat()
        if np.dot(smoothed,q)<0:smoothed=-smoothed
        for a,v in zip('xyzw',smoothed):row[f'palm_q{a}_display_filtered']=v
        if previous_filtered is not None:row['filtered_step_angle_deg']=angular_distance(previous_filtered,smoothed)
        previous_filtered=smoothed
    write_csv(OUT/'primary_pose_quality.csv',diagnostics)
    valid=[r for r in diagnostics if r.get('orientation_valid')=='True']
    active=[f for f in frames if f['phase_approx'] in ACTIVE]
    phase_summary=[]
    for phase in ['Idle']+ACTIVE:
        ff=[f for f in frames if f['phase_approx']==phase]
        rr=[r for r in diagnostics if r['phase_approx']==phase and r.get('orientation_valid')=='True']
        steps=[r['raw_step_angle_deg'] for r in rr if r['raw_step_angle_deg']!='']
        agreements=[r['static_agreement_deg'] for r in rr if r['static_agreement_deg']!='']
        width=np.array([float(r['palm_width_model_m']) for r in rr])
        phase_summary.append(dict(phase=phase,total_frames=len(ff),new_detected=sum(int(f['hand_count'])>0 for f in ff),
            old_detected=sum(int(f['frame']) in old_frames for f in ff),valid_orientation_frames=len(rr),
            static_detected=sum(static[int(f['frame'])].get('detected')=='True' for f in ff),
            raw_step_median_deg=stats(steps).get('median',''),raw_step_p95_deg=stats(steps).get('p95',''),
            jumps_over_30_deg=sum(bool(r['orientation_jump']) for r in rr),
            static_disagreement_median_deg=stats(agreements).get('median',''),
            static_disagreement_p95_deg=stats(agreements).get('p95',''),
            palm_width_cv_percent=float(100*width.std()/width.mean()) if len(width) else ''))
    write_csv(OUT/'phase_quality.csv',phase_summary)

    # Recorded robot forward kinematics: uses model origins and measured joints,
    # not a fit to the human landmarks. Only relative-change scalars are compared.
    baseline=ROOT/'outputs/successful_baseline'
    robot=read_csv(baseline/'contact_trace.csv')
    report=json.loads((baseline/'contact_trial.json').read_text())
    fk=robot_fk(robot,ROOT/'outputs/robot.urdf')
    rq=Rotation.from_matrix(fk[:,:3,:3]).as_quat()
    robot_rows=[]
    for r,t,q in zip(robot,fk,rq):
        out=dict(time_s=r['time_s'],stage=r['stage'],gripper_driver_rad=r['gripper_rad'],
                 cup_x_m=r['cup_x_m'],cup_y_m=r['cup_y_m'],cup_z_m=r['cup_z_m'])
        for a,v in zip('xyz',t[:3,3]):out[f'ee_world_{a}_m']=v
        for a,v in zip('xyzw',q):out[f'ee_q{a}']=v
        robot_rows.append(out)
    write_csv(OUT/'robot_baseline_fk.csv',robot_rows)
    drag_indices=[i for i,r in enumerate(robot) if r['stage']=='drag']
    robot_angles=[angular_distance(rq[drag_indices[0]],rq[i]) for i in drag_indices]
    human_drag=[r for r in diagnostics if r['phase_approx']=='Drag' and r.get('orientation_valid')=='True']
    first_by_track={};human_angles=[]
    for row in human_drag:
        initial=first_by_track.setdefault(row['track_id'],quaternion(row))
        human_angles.append(angular_distance(initial,quaternion(row)))

    summary=dict(video=metadata,old_detected_frames=len(old_frames),new_detected_frames=metadata['detected_frames'],
        old_vs_new_image_landmark_repeatability_px=stats(image_repeatability),
        active_frames=len(active),active_detected=sum(int(f['hand_count'])>0 for f in active),
        valid_primary_orientation_frames=len(valid),missing_active_runs=groups_of_missing(frames),
        raw_orientation_step_deg=stats([r['raw_step_angle_deg'] for r in valid if r['raw_step_angle_deg']!='']),
        display_filtered_orientation_step_deg=stats([r['filtered_step_angle_deg'] for r in valid if r['filtered_step_angle_deg']!='']),
        temporal_vs_static_disagreement_deg=stats([r['static_agreement_deg'] for r in valid if r['static_agreement_deg']!='']),
        raw_orientation_jump_frames=[dict(frame=r['frame'],time_s=r['time_s'],phase=r['phase_approx'],
            angle_deg=r['raw_step_angle_deg']) for r in valid if r['orientation_jump']],
        handedness_label_flips=sum(bool(r['label_flip']) for r in valid),
        quaternion_representation_sign_changes=quaternion_sign_changes,
        primary_identity_boundaries=[dict(frame=r['frame'],time_s=r['time_s']) for r in valid if r['identity_boundary']],
        image_jump_frames=[r['frame'] for r in valid if r['image_jump']],
        palm_width_jump_frames=[r['frame'] for r in valid if r['shape_jump']],
        multiple_hand_frames=[int(f['frame']) for f in frames if int(f['hand_count'])>1],
        max_rotation_orthogonality_error=max(float(r['orthogonality_error']) for r in valid),
        max_rotation_determinant_error=max(abs(float(r['rotation_determinant'])-1) for r in valid),
        phases=phase_summary,static_metadata=json.loads((OUT/'static/metadata.json').read_text()),
        comparison=dict(baseline_report=str(baseline/'contact_trial.json'),baseline_success=report['success'],
            baseline_trace_sha256=hashlib.sha256((baseline/'contact_trace.csv').read_bytes()).hexdigest(),
            robot_model_sha256=hashlib.sha256((ROOT/'outputs/robot.urdf').read_bytes()).hexdigest(),
            human_drag_duration_s=7.0667-5.3667,robot_drag_duration_s=report['drag']['duration_s'],
            human_drag_orientation_change_deg=stats(human_angles),robot_drag_orientation_change_deg=stats(robot_angles),
            cup_displacement_m=report['cup_displacement_m'],
            limitation='Unaligned human model and robot world frames; phase-relative qualitative comparison only. No human-to-robot pose error is identifiable.'))
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    plot(diagnostics,phase_summary,human_drag,human_angles,robot,drag_indices,robot_angles)
    print(json.dumps({k:v for k,v in summary.items() if k not in ['video','static_metadata','phases']},indent=2))


def plot(rows,phases,human_drag,human_angles,robot,indices,robot_angles):
    fig,ax=plt.subplots(4,1,figsize=(13,12),sharex=True,constrained_layout=True)
    times=[float(r['time_s']) for r in rows]
    ax[0].step(times,[int(r['detected']=='True') for r in rows],where='mid',label='Primary detection')
    ax[0].set_ylabel('Detected');ax[0].set_ylim(-.1,1.2)
    ax[1].plot(times,[r['raw_step_angle_deg'] if r['raw_step_angle_deg']!='' else np.nan for r in rows],label='Raw')
    ax[1].plot(times,[r['filtered_step_angle_deg'] if r['filtered_step_angle_deg']!='' else np.nan for r in rows],label='Display filter',alpha=.8)
    ax[1].axhline(30,color='red',ls='--',label='Jump flag (30 deg/frame)');ax[1].legend();ax[1].set_ylabel('Rotation step (deg)')
    ax[2].plot(times,[r['static_agreement_deg'] if r['static_agreement_deg']!='' else np.nan for r in rows])
    ax[2].set_ylabel('Video vs static (deg)')
    for finger in ['thumb','index','middle','ring','little']:
        key=f'{finger}_{"mcp" if finger=="thumb" else "pip"}_bend_deg'
        ax[3].plot(times,[float(r[key]) if r.get(key) else np.nan for r in rows],label=finger)
    ax[3].legend(ncol=5);ax[3].set_ylabel('Geometric bend (deg)');ax[3].set_xlabel('Source video time (s)')
    for a in ax:
        a.grid(alpha=.2);a.axvspan(5.35,7.1,color='#16a34a',alpha=.08)
    fig.suptitle('MediaPipe hand-centered estimates — missing/jump data retained; green = approximate drag')
    fig.savefig(OUT/'tracking_quality.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(1,2,figsize=(13,4.5),constrained_layout=True)
    hp=np.array([float(r['time_s']) for r in human_drag]);hp=(hp-hp[0])/(hp[-1]-hp[0])
    rt=np.array([float(robot[i]['time_s']) for i in indices]);rp=(rt-rt[0])/(rt[-1]-rt[0])
    ax[0].plot(hp,human_angles,label='Human palm, relative to segment start')
    ax[0].plot(rp,robot_angles,label='Robot EE, relative to drag start')
    ax[0].set_ylabel('Relative rotation magnitude (deg)');ax[0].legend()
    phases_data=read_csv(ROOT/'data/raw/human_demonstration_phases.csv')
    drag=[r for r in phases_data if r['phase_approx']=='Drag']
    x=np.array([float(r['cup_x_full_px']) for r in drag]);p=(x[0]-x)/(x[0]-x[-1])
    tp=np.array([float(r['time_s']) for r in drag]);tp=(tp-tp[0])/(tp[-1]-tp[0])
    yy=np.array([float(robot[i]['cup_y_m']) for i in indices]);yy=(yy-yy[0])/.3
    ax[1].plot(tp,p,label='Human cup IMAGE progress (normalized)')
    ax[1].plot(rp,yy,label='Robot cup +Y / requested 0.30 m')
    ax[1].set_ylabel('Normalized cup progress');ax[1].legend()
    for a in ax:a.set_xlabel('Fraction of each drag interval (different durations)');a.grid(alpha=.2)
    fig.suptitle('Qualitative phase comparison — no spatial calibration or pose-error comparison')
    fig.savefig(OUT/'robot_comparison.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
