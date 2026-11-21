#!/usr/bin/env python3
"""Dependency-free preflight checks for the sandwich Codex experiment."""
from pathlib import Path
import csv, sys, hashlib
root=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent.parent
checks={
'data/raw/2026-11-21 11-02-55.mkv':None,
'data/mediapipe/2026-11-21 11-02-55_hand_landmarks.csv':['frame','time_s','hand_index','handedness','landmark_index','x_px','y_px','z_relative_not_meters'],
'data/derived/sandwich_hand_object_timeline_10hz.csv':['frame','time_s','hand_grasp_x_px','hand_grasp_y_px','tomato_detected','phase'],
'data/derived/sandwich_ingredient_events_review.csv':['ingredient','event','time_s_approx','confidence'],
'data/derived/sandwich_object_keyframes_review.csv':['ingredient','location','x_px_approx','y_px_approx'],
}
errors=[]
for rel,fields in checks.items():
 p=root/rel
 if not p.is_file(): errors.append('MISSING '+rel); continue
 print('OK',rel,p.stat().st_size,'bytes')
 if fields:
  with p.open(newline='',encoding='utf-8-sig') as f:
   rdr=csv.DictReader(f); cols=rdr.fieldnames or []
   if any(c not in cols for c in fields): errors.append('BAD HEADER '+rel)
   count=sum(1 for _ in rdr)
  print('   rows:',count)
  if not count: errors.append('EMPTY CSV '+rel)
mp=root/'data/mediapipe/2026-11-21 11-02-55_hand_landmarks.csv'
if mp.exists():
 with mp.open(newline='',encoding='utf-8-sig') as f:
  r=csv.DictReader(f)
  frames=set(); n=0; lms=set()
  for x in r: frames.add(int(x['frame']));lms.add(int(x['landmark_index']));n+=1
 print('   mediapipe: rows=',n,'unique detected frames=',len(frames),'landmark IDs=',sorted(lms))
 if n!=32067 or len(frames)!=1476 or lms != set(range(21)):
  errors.append('MEDIAPIPE METRICS CHANGED: investigate before claiming same source')
# Validate packaged sha256 manifest, which excludes the manifest itself
manifest=root/'MANIFEST_SHA256.txt'
if manifest.exists():
 for line in manifest.read_text().splitlines():
  if not line.strip(): continue
  digest,rel=line.split('  ',1)
  p=root/rel
  if not p.exists(): errors.append('CHECKSUM FILE MISSING '+rel);continue
  h=hashlib.sha256()
  with p.open('rb') as f:
   for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
  if h.hexdigest()!=digest:errors.append('CHECKSUM FAILED '+rel)
 print('SHA256 manifest verified' if not errors else 'One or more integrity issues')
for e in errors: print('ERROR:',e,file=sys.stderr)
print('VALIDATION', 'PASSED' if not errors else 'FAILED')
sys.exit(1 if errors else 0)
