#!/usr/bin/env python3
"""Offline legacy MediaPipe image+world landmark extraction, with missing rows.

Run from any directory using the dedicated .venv-hand3d interpreter. This script
does not import ROS, issue commands, modify inputs, or train a model.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import mediapipe as mp
import numpy as np
from scipy.optimize import linear_sum_assignment
from geometry import palm_pose, finger_bends

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VIDEO = ROOT / 'data/raw/2026-11-15 20-59-05.mkv'


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: '' if isinstance(v, (float, np.floating)) and not np.isfinite(v) else v
                             for k, v in row.items()})


def optional_value(point, field):
    return getattr(point, field) if point.HasField(field) else ''


def extract(video, out, static=False):
    out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open {video}')
    fps = cap.get(cv2.CAP_PROP_FPS)
    width, height = int(cap.get(3)), int(cap.get(4))
    phase_file = ROOT / 'data/raw/human_demonstration_phases.csv'
    with phase_file.open(newline='') as stream:
        phases = {int(r['frame']): r for r in csv.DictReader(stream)}
    landmark_rows, pose_rows, frame_rows, arrays = [], [], [], []
    tracks = {}
    next_track = 0
    previous_primary = None
    count = 0
    start = time.monotonic()
    # No asynchronous callbacks: every decoded frame is processed once.
    with mp.solutions.hands.Hands(static_image_mode=static, max_num_hands=2,
            model_complexity=1, min_detection_confidence=.5, min_tracking_confidence=.5) as hands:
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            index = count
            t = index / fps
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            result = hands.process(rgb)
            images = result.multi_hand_landmarks or []
            worlds = result.multi_hand_world_landmarks or []
            if len(images) != len(worlds):
                raise RuntimeError(f'Image/world hand count mismatch at frame {index}')
            detections = []
            for hand_index, (image, world, handedness) in enumerate(zip(images, worlds, result.multi_handedness or [])):
                image_xyz = np.array([[p.x, p.y, p.z] for p in image.landmark])
                world_xyz = np.array([[p.x, p.y, p.z] for p in world.landmark])
                if image_xyz.shape != (21, 3) or world_xyz.shape != (21, 3):
                    raise RuntimeError('Incomplete landmark output')
                category = handedness.classification[0]
                detections.append(dict(image=image_xyz, world=world_xyz, label=category.label,
                    score=float(category.score), hand_index=hand_index, raw_world=world))

            active = [(k, v) for k, v in tracks.items() if index - v['frame'] <= round(fps*.25)]
            assignments = {}
            if active and detections:
                costs = np.array([[np.linalg.norm(v['wrist'] - d['image'][0, :2]) for d in detections]
                                  for _, v in active])
                ii, jj = linear_sum_assignment(costs)
                for i, j in zip(ii, jj):
                    if costs[i, j] <= .2:
                        assignments[j] = active[i][0]
            for j, detection in enumerate(detections):
                if j not in assignments:
                    assignments[j] = next_track
                    next_track += 1
                track = assignments[j]
                detection['track'] = track
                tracks[track] = dict(frame=index, wrist=detection['image'][0, :2])

            phase = phases.get(index, {})
            primary, selection = None, 'missing'
            if detections:
                if phase.get('wrist_x_px') and phase.get('wrist_y_px'):
                    old_wrist = np.array([float(phase['wrist_x_px'])/width, float(phase['wrist_y_px'])/height])
                    primary = min(range(len(detections)), key=lambda j: np.linalg.norm(detections[j]['image'][0,:2]-old_wrist))
                    selection = 'nearest_existing_image_wrist'
                elif previous_primary is not None:
                    primary = min(range(len(detections)), key=lambda j: np.linalg.norm(detections[j]['image'][0,:2]-previous_primary))
                    selection = 'nearest_previous_image_wrist_unverified'
                else:
                    primary = int(np.argmax([d['score'] for d in detections]))
                    selection = 'handedness_score_fallback_not_detection_confidence'
                previous_primary = detections[primary]['image'][0, :2]
            frame_rows.append(dict(frame=index, time_s=t, phase_approx=phase.get('phase_approx',''),
                hand_count=len(detections), primary_hand_index=primary if primary is not None else '',
                primary_track_id=detections[primary]['track'] if primary is not None else '',
                primary_selection=selection))

            if not detections:
                pose_rows.append(dict(frame=index,time_s=t,phase_approx=phase.get('phase_approx',''),
                    detected=False,is_primary=True,track_id='',orientation_valid=False,
                    invalid_reason='no_detection', tracking_confidence='', handedness_score=''))
            for j, d in enumerate(detections):
                common = dict(frame=index,time_s=t,hand_index=j,track_id=d['track'],
                    handedness_raw=d['label'],handedness_score=d['score'],is_primary=j==primary)
                for k, (image, world) in enumerate(zip(d['image'], d['world'])):
                    point = d['raw_world'].landmark[k]
                    landmark_rows.append(dict(**common,landmark_index=k,
                        image_x_norm=image[0],image_y_norm=image[1],image_z_relative=image[2],
                        image_x_px=image[0]*width,image_y_px=image[1]*height,
                        mp_world_x_model_m=world[0],mp_world_y_model_m=world[1],mp_world_z_model_m=world[2],
                        world_visibility=optional_value(point,'visibility'),world_presence=optional_value(point,'presence')))
                row = dict(**common,phase_approx=phase.get('phase_approx',''),detected=True,
                    primary_selection=selection if j==primary else '',tracking_confidence='',
                    image_wrist_x_px=d['image'][0,0]*width,image_wrist_y_px=d['image'][0,1]*height,
                    coordinate_status='hand_centered_model_estimate_not_calibrated_world')
                for name, xyz in [('wrist',d['world'][0]),('palm',d['world'][[0,5,9,13,17]].mean(axis=0))]:
                    for axis, value in zip('xyz',xyz):row[f'{name}_mp_{axis}_model_m']=value
                try:
                    center, rotation, quat, sine = palm_pose(d['world'])
                    row.update(orientation_valid=True,invalid_reason='',palm_axis_sine=sine,
                        rotation_determinant=float(np.linalg.det(rotation)),
                        orthogonality_error=float(np.linalg.norm(rotation.T@rotation-np.eye(3))))
                    for k, value in zip('xyzw',quat):row[f'palm_q{k}']=value
                    for a in range(3):
                        for b in range(3):row[f'palm_R{a}{b}']=rotation[a,b]
                except ValueError as error:
                    row.update(orientation_valid=False,invalid_reason=str(error))
                row.update(finger_bends(d['world']))
                row['palm_width_model_m'] = float(np.linalg.norm(d['world'][5]-d['world'][17]))
                row['index_middle_bone_model_m'] = float(np.linalg.norm(d['world'][6]-d['world'][5]))
                pose_rows.append(row)
                arrays.append(dict(frame=index,hand_index=j,track_id=d['track'],primary=j==primary,
                                   image=d['image'],world=d['world']))
            count += 1
            if count % 60 == 0:
                print(f'{"static" if static else "video"}: {count} frames',flush=True)
    cap.release()
    write_csv(out/'landmarks.csv',landmark_rows)
    write_csv(out/'hand_pose_flexion.csv',pose_rows)
    write_csv(out/'frames.csv',frame_rows)
    np.savez_compressed(out/'landmarks.npz',frame=np.array([a['frame'] for a in arrays]),
        hand_index=np.array([a['hand_index'] for a in arrays]),track_id=np.array([a['track_id'] for a in arrays]),
        primary=np.array([a['primary'] for a in arrays]),
        image=np.array([a['image'] for a in arrays]),world=np.array([a['world'] for a in arrays]))
    model_dir=Path(mp.__file__).parent/'modules'
    models={str(p.relative_to(model_dir)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in model_dir.rglob('*.tflite') if 'hand_landmark_full' in p.name or 'palm_detection_full' in p.name}
    metadata=dict(video=str(video.resolve()),source_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
        mediapipe_version=mp.__version__,python=sys.version,opencv_version=cv2.__version__,
        frames=count,fps=fps,width=width,height=height,static_image_mode=static,
        max_num_hands=2,model_complexity=1,min_detection_confidence=.5,min_tracking_confidence=.5,
        inference_elapsed_s=time.monotonic()-start,detected_frames=sum(r['hand_count']>0 for r in frame_rows),
        hand_instances=len(arrays),track_segments=next_track,model_sha256=models,
        pipeline_applies_horizontal_flip=False,source_mirror_status='unverified',
        handedness_convention='Raw MediaPipe labels; anatomical side and original mirroring unverified',
        translation='Model hand-center-relative XYZ in nominal meters; NOT camera/world translation',
        orientation='Constructed palm frame relative to model-output axes; NOT calibrated robot orientation',
        confidence='Handedness score only; detection/tracking and landmark confidence unavailable in this API',
        tracking_association='2D wrist Hungarian association <=0.2 normalized units, max gap 0.25 s; not identity ground truth',
        smoothing='None during extraction; raw outputs retained')
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--video',type=Path,default=DEFAULT_VIDEO)
    parser.add_argument('--out',type=Path,default=ROOT/'outputs/hand3d/video')
    parser.add_argument('--static',action='store_true')
    args=parser.parse_args()
    extract(args.video,args.out,args.static)
