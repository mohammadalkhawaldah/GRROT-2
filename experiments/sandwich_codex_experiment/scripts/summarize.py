"""Collect actual completed attempts and check cup/input preservation."""
from pathlib import Path
import hashlib
import json
from scene import ROOT,BASE
from physics_signature import physics_signature

runs=[]
for p in sorted((ROOT/'outputs').glob('*/result.json')):
    d=json.loads(p.read_text())
    runs.append(dict(run=p.parent.name,success=d['success'],failed_stage=d.get('failed_stage'),
        error=d.get('error'),completed_transfers=sum(t['passed'] for t in d['transfers']),
        transfers=[dict(item=t['item'],passed=t['passed'],lift_m=t['lifted']['z']-t['initial']['z'] if 'lifted' in t else None,
            xy_error_m=t.get('xy_error_m'),z_error_m=t.get('z_error_m'),released=t.get('released')) for t in d['transfers']],
        unintended_robot_support_contacts=len(d['unwanted_robot_support_contacts'])))
baseline=json.loads((ROOT/'outputs/baseline_hashes.json').read_text())
changed=[f for f,h in baseline.items() if hashlib.sha256((BASE/f).read_bytes()).hexdigest()!=h]
preserved=dict(checked_files=len(baseline),changed=changed)
(ROOT/'outputs/baseline_preservation.json').write_text(json.dumps(preserved,indent=2)+'\n')
originals=json.loads((ROOT/'outputs/dataset_audit.json').read_text())['input_sha256']
inputs_changed=[f for f,h in originals.items() if hashlib.sha256((ROOT/f).read_bytes()).hexdigest()!=h]
full=[r for r in runs if r['run'].startswith('full_nominal')]
aborted=[dict(run=p.parent.name,**json.loads(p.read_text())) for p in sorted((ROOT/'outputs').glob('*/aborted.json'))]
core_files=['scripts/trial.py','scripts/scene.py','config/controllers.yaml','worlds/sandwich.sdf']
core_hashes={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in core_files}
world_physics=physics_signature(ROOT/'worlds/sandwich.sdf')
corrected=[]
for r in full:
    p=ROOT/'outputs'/r['run']/'source_hashes.json'
    hashes=json.loads(p.read_text()) if p.exists() else {}
    # A rendering-only asset update does not invalidate measured physics trials.
    # Keep the command/acceptance code and controller YAML exact; compare every
    # nonvisual SDF element rather than the scene generator's source text.
    world=p.parent/'run_sources/worlds/sandwich.sdf'
    robot=p.parent/'robot.urdf'
    if (all(hashes.get(f)==core_hashes[f] for f in ['scripts/trial.py','config/controllers.yaml'])
            and world.exists() and physics_signature(world)==world_physics
            and robot.exists() and hashlib.sha256(robot.read_bytes()).digest()==hashlib.sha256((ROOT/'outputs/robot.urdf').read_bytes()).digest()):
        corrected.append(r)
max_final_xy=0.
independent={}
for r in corrected:
    d=json.loads((ROOT/'outputs'/r['run']/'result.json').read_text())
    for p in d.get('final_ingredient_poses',{}).values():max_final_xy=max(max_final_xy,((p['x']-.55)**2+(p['y']-.30)**2)**.5)
    evidence=ROOT/'outputs'/r['run']/'evidence_summary.json'
    independent[r['run']]=json.loads(evidence.read_text()).get('strict_unwanted_robot_support_contact_rows') if evidence.exists() else None
summary=dict(runs=runs,nominal_full_attempts=len(full),nominal_full_successes=sum(r['success'] and r['completed_transfers']==4 for r in full),
    interrupted_attempts=aborted,
    corrected_nominal_runs=[r['run'] for r in corrected],corrected_nominal_successes=sum(r['success'] for r in corrected),
    corrected_core_hashes=core_hashes,max_final_xy_error_corrected_m=max_final_xy,
    corrected_world_physics_sha256=world_physics,rendering_excluded_from_physics_comparison=True,
    independent_unwanted_support_contact_rows=independent,
    required_nominal_resets=3,repeatability_gate_passed=len(corrected)>=3 and all(r['success'] and r['completed_transfers']==4 and independent[r['run']]==0 for r in corrected),
    baseline=preserved,inputs_changed=inputs_changed,learning=False,artificial_attachment=False,
    contact_forces_available=False,human_coordinates_calibrated=False,
    limits=['Rigid thick ingredient proxies; assumed geometry/mass/friction/poses',
        'No tactile-force verification or hardware transfer validation',
        'No learned autonomy or exact calibrated human trajectory reproduction'])
(ROOT/'outputs/experiment_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='runs'},indent=2))
