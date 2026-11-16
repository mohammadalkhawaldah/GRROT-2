#!/usr/bin/env python3
"""Copy the five supplied source files locally and validate the handoff schemas.

Original data is never modified. Existing local copies must match the source.
The report distinguishes image-space measurements from metric robot actions.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
FILES = ['2026-11-15 20-59-05.mkv', '2026-11-15 20-59-05_hand_landmarks.csv',
         '2026-11-15 20-59-05_hand_preview.mp4', 'cup_rough_scale_5cm.csv',
         'human_demonstration_phases.csv']
SCHEMAS = {
    FILES[1]: ['frame', 'time_s', 'hand_index', 'handedness', 'landmark_index',
               'x_px', 'y_px', 'z_relative_not_meters'],
    FILES[3]: ['time_s', 'phase_approx', 'cup_dx_px', 'cup_dy_px', 'rough_dx_cm',
               'rough_dy_image_cm', 'template_score', 'measurement_status'],
    FILES[4]: ['frame', 'time_s', 'cup_x_px', 'cup_y_px', 'cup_x_full_px', 'cup_y_full_px',
               'template_score', 'phase_approx', 'cup_quality_note', 'wrist_x_px', 'wrist_y_px',
               'pinch_mid_x_px', 'pinch_mid_y_px', 'normalized_pinch_gap', 'hand_detected',
               'gripper_state_verified', 'measurement_units'],
}


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()


def inspect_csv(path):
    with path.open(newline='', encoding='utf-8-sig') as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames or []
        rows = list(reader)
    missing = sorted(set(SCHEMAS[path.name]) - set(columns))
    if missing or not rows:
        raise ValueError(f'{path.name}: missing columns {missing}, rows={len(rows)}')
    times = [float(r['time_s']) for r in rows]
    if not all(math.isfinite(t) for t in times) or any(b < a for a, b in zip(times, times[1:])):
        raise ValueError(f'{path.name}: nonfinite or nonmonotonic timestamps')
    result = dict(columns=columns, rows=len(rows), time_range_s=[min(times), max(times)])
    if 'landmark_index' in columns:
        groups = collections.defaultdict(list)
        for row in rows:
            groups[(int(row['frame']), row['hand_index'])].append(int(row['landmark_index']))
        result['detected_frames'] = len({key[0] for key in groups})
        result['hand_frame_groups'] = len(groups)
        result['incomplete_landmark_groups'] = sum(sorted(v) != list(range(21)) for v in groups.values())
        if result['incomplete_landmark_groups']:
            raise ValueError(f'{path.name}: incomplete/duplicate 21-landmark groups')
    if 'phase_approx' in columns:
        result['phase_row_counts'] = dict(collections.Counter(r['phase_approx'] for r in rows))
    if 'gripper_state_verified' in columns:
        result['gripper_state_values'] = sorted({r['gripper_state_verified'] for r in rows})
        result['measurement_units'] = sorted({r['measurement_units'] for r in rows})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=ROOT / 'data/raw')
    args = parser.parse_args()
    dest_dir = ROOT / 'data/raw'
    dest_dir.mkdir(parents=True, exist_ok=True)
    report = {'source_directory': str(args.source), 'files': {}, 'errors': [],
              'coordinate_status': 'Image XY and relative/model Z; no calibrated human 6DoF pose',
              'metric_status': 'Rough tabletop scale only; not camera calibration',
              'robot_action_dataset': False, 'all_files_validated': False}
    for name in FILES:
        src, dest = args.source / name, dest_dir / name
        try:
            source_hash = digest(src)
            if src.resolve() != dest.resolve():
                if dest.exists():
                    if digest(dest) != source_hash:
                        raise ValueError('Existing local file differs; refusing to overwrite')
                else:
                    # A failed drive read leaves a clearly named partial copy.
                    part = dest.with_name(dest.name + '.partial')
                    with src.open('rb') as inp, part.open('wb') as out:
                        for data in iter(lambda: inp.read(1024 * 1024), b''):
                            out.write(data)
                    if digest(part) != source_hash:
                        raise ValueError('Copied file checksum differs from source')
                    part.rename(dest)
            result = {'local_path': str(dest), 'sha256': source_hash, 'bytes': dest.stat().st_size}
            if dest.suffix == '.csv':
                result.update(inspect_csv(dest))
            else:
                probe = subprocess.run(['ffprobe', '-v', 'error', '-show_entries',
                    'format=duration:stream=codec_type,width,height,r_frame_rate,nb_frames',
                    '-of', 'json', str(dest)], capture_output=True, text=True, timeout=30, check=True)
                result['video_probe'] = json.loads(probe.stdout)
            report['files'][name] = result
        except Exception as exc:
            report['errors'].append({'file': name, 'error': f'{type(exc).__name__}: {exc}'})
    report['all_files_validated'] = not report['errors']
    (ROOT / 'outputs/demo_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['all_files_validated'] else 1)


if __name__ == '__main__':
    main()
