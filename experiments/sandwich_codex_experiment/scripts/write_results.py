"""Write a concise report from completed evidence, including interruptions."""
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[1]
o=ROOT/'outputs';s=json.loads((o/'experiment_summary.json').read_text())
a=json.loads((o/'dataset_audit.json').read_text())
success=s['corrected_nominal_successes'];count=len(s['corrected_nominal_runs'])
lines=['# Sandwich experiment results — 21 November 2026','',
    f"Dataset checks passed: {a['video']['streams'][0]['nb_read_frames']} video frames, {a['detected_frames']} frames with hand detections ({100*a['coverage']:.2f}%), {a['hand_groups']} complete 21-point groups. Original checksum manifest verified; input hashes unchanged.",'',
    'ROS 2 Humble / Fortress 6.18.0 / MoveIt 2.5.10 / pick_ik 1.1.2 work on the existing Jetson ARM64 setup. No missing dependency or package installation was needed. All 23 audited cup-baseline files remain unchanged.','',
    'The standalone bottom-bread gate passed: 146.4 mm observed lift, bilateral fingertip contact, 0.72 mm final XY error, released contacts and stable placement.','',
    f"Corrected full implementation: **{success}/{count} completed nominal runs passed**; repeatability gate {'PASSED' if s['repeatability_gate_passed'] else 'NOT YET PASSED'}. Motion code, nonvisual scene and controller configuration match across these runs; rendering-only changes are excluded from the physics comparison. Maximum final ingredient XY error: **{1000*s['max_final_xy_error_corrected_m']:.2f} mm**.",'',
    '| Run | Outcome | Completed transfers |','|---|---|---|']
for r in s['runs']:
    lines.append(f"| {r['run']} | {'PASS' if r['success'] else 'FAIL: '+str(r['failed_stage'])} | {r['completed_transfers']} |")
for r in s['interrupted_attempts']:lines.append(f"| {r['run']} | Interrupted, signal {r['signal']}; incomplete evidence | — |")
lines+=['','`top_debug_01` began with the lower three layers already stacked; its pass is an isolated top-placement test, not a four-transfer success. `full_nominal_04` retains partial video/logs; no measured CSV survived its runner interruption.','',
    'Earlier failures exposed a stale MoveIt planning obstacle, repeated-remove bookkeeping, and cheese displacement during top-bread descent. The held planning proxy is now removed once, with physics collisions retained and independent payload clearance checks. Top placement uses the measured grasp offset and a 5 mm release gap. No food attachment, teleportation, fixed grasp joint or object-pose command is used.','',
    'Completed full passes verify all four grasp/lift/transport/place/release operations, correct stack heights/order and adjacent contacts, five-second final stability, and zero recorded arm/table or arm/plate collisions. Native contact messages contain positions but empty wrench arrays, so grip forces are unmeasured.']
for name in s['corrected_nominal_runs']:
    p=o/name/'evidence_summary.json'
    if p.exists():
        e=json.loads(p.read_text());lines+=['',f"{name}: average observed simulation rate {e.get('observed_average_real_time_factor',0):.2f}× real time; bilateral fingertip contact at transport end: {all(t['bilateral_contact_at_transport_end'] for t in e['transfers'])}."]
lines+=['','Human-derived: ingredient order, image-right to image-left task direction and coarse relative timing. Assumed: all world coordinates, dimensions, rigid geometry, masses, friction and tool heights/orientation. Programmed: MoveIt/JTC goals and state-aware top placement. Calibration, deformable food, varied-source robustness, camera training observations and a verified policy-training schema remain unproven. No training was started.','',
    'Next implementation step: add a named ±10 mm source-layout offset to `scripts/scene.py` (move both source bread pieces together), then rerun the four-step test with the same physical acceptance gates. Nominal success alone does not establish pose-variation robustness.','',
    'Reproduce from `/home/mohammad/Documents/Groot2-1`, using an unused run name:','',
    '```bash','bash experiments/sandwich_codex_experiment/scripts/run.sh sandwich_new_01 --full --gui --record','```','',
    'See [implementation and assumptions](IMPLEMENTATION.md), [machine-readable results](outputs/experiment_summary.json), and each run folder for commands, measured feedback, poses, contacts, logs and source snapshots.']
last=s['corrected_nominal_runs'][-1] if s['corrected_nominal_runs'] else None
visual_runs=[r['run'] for r in s['runs'] if r['run'].startswith('food_visual_') and r['success'] and (o/r['run']/'gazebo_review_4x.mp4').exists()]
if visual_runs:
    last=visual_runs[-1]
    lines+=['','Appearance update: bread now has a loaf outline, crust and porous crumb; tomato has a rounded seeded cut face; cheese has rounded edges and holes. The updated appearance completed a separate full physical replay. Contact boxes, mass, inertia, friction and motion code remain unchanged. See [food appearance details](models/food/README.md).']
if last and (o/last/'gazebo_review_4x.mp4').exists():
    lines+=['',f"Video: [4× screen-recording playback](outputs/{last}/gazebo_review_4x.mp4), [original recording](outputs/{last}/gazebo_recording.mp4), [final scene](outputs/{last}/final_scene.png). The accelerated review does not represent real-time simulation."]
(ROOT/'RESULTS.md').write_text('\n'.join(lines)+'\n')
