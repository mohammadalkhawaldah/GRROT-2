"""Extra dataset checks beyond supplied checksum validator; read-only inputs."""
import collections
import csv
import hashlib
import json
import math
import platform
import shutil
import subprocess
from scene import ROOT

rows=list(csv.DictReader(next((ROOT/'data/mediapipe').glob('*.csv')).open()))
groups=collections.defaultdict(list);times=collections.defaultdict(set)
for row in rows:
    groups[int(row['frame']),row['hand_index']].append(int(row['landmark_index']))
    times[int(row['frame'])].add(float(row['time_s']))
video=next((ROOT/'data/raw').glob('*.mkv'))
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-count_frames','-select_streams','v:0',
    '-show_entries','stream=width,height,r_frame_rate,nb_read_frames','-of','json',str(video)],text=True))
frames=int(probe['streams'][0]['nb_read_frames']);detected=set(times)
checks=dict(complete_landmark_groups=all(sorted(v)==list(range(21)) for v in groups.values()),
    one_timestamp_per_frame=all(len(v)==1 for v in times.values()),
    finite_coordinates=all(math.isfinite(float(row[k])) for row in rows for k in ['time_s','x_px','y_px','z_relative_not_meters']),
    timestamps_match_30fps=all(abs(t-f/30)<.002 for f,ts in times.items() for t in ts))
result=dict(checks=checks,passed=all(checks.values()),video=probe,detected_frames=len(detected),
    coverage=len(detected)/frames,hand_groups=len(groups),missing_frames=sorted(set(range(frames))-detected),
    frames_with_two_hands=sum(sum(k[0]==f for k in groups)>1 for f in detected),
    out_of_image_landmark_rows=sum(not(0<=float(r['x_px'])<1920 and 0<=float(r['y_px'])<1080) for r in rows),
    confidence_columns_present=False,occluded_points_are_model_estimates=True,
    calibrated=False,world_landmarks_available=False,architecture=platform.machine(),
    input_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'data').rglob('*') if p.is_file()},
    environment=dict(ros='humble',gazebo='Fortress',free_disk_bytes=shutil.disk_usage(ROOT).free,
        robot='Kinova Gen3 7DoF + Robotiq 2F85',baseline_camera='No rendering camera sensor in baseline model',
        package_changes='none',domain=62,partition='sandwich_experiment'))
(ROOT/'outputs/dataset_audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ['input_sha256','missing_frames']},indent=2))
raise SystemExit(0 if result['passed'] else 1)
