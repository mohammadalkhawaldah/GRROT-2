"""Fresh owned world per run. Never edits baseline source or generated outputs."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import shutil
import hashlib
import re
from scene import ROOT,BASE,generate
sys.path.insert(0,str(BASE/'scripts'))
import prepare_robot
from verify_contact import stop

def main():
    generate(top_debug='--top-only' in sys.argv);prepare_robot.ROOT=ROOT;prepare_robot.prepare()
    robot=ET.parse(ROOT/'outputs/robot.urdf')
    robot.find("joint[@name='base_joint']/origin").set('xyz','0 0 0.401')
    robot.write(ROOT/'outputs/robot.urdf',encoding='unicode',xml_declaration=True)
    out=ROOT/'outputs'/sys.argv[1];out.mkdir(parents=True,exist_ok=False)
    snapshot=out/'run_sources';snapshot.mkdir()
    source_hashes={}
    for directory in ['scripts','launch','config','worlds','models']:
        paths=(ROOT/directory).rglob('*') if directory=='models' else (ROOT/directory).glob('*')
        for p in paths:
            if not p.is_file():continue
            rel=p.relative_to(ROOT);destination=snapshot/rel;destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(p,destination);source_hashes[str(rel)]=hashlib.sha256(p.read_bytes()).hexdigest()
    shutil.copy2(ROOT/'outputs/robot.urdf',out/'robot.urdf')
    (out/'source_hashes.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
    env={**os.environ,'ROS_DOMAIN_ID':'62','IGN_PARTITION':'sandwich_experiment',
         'ROS_LOG_DIR':str(out/'ros_logs'),'CUP_IK_SOLVER':'pick_ik'}
    processes=[];logs=[];recorder=None
    try:
        for name in ['simulation','planning']:
            log=(out/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(['ros2','launch',str(ROOT/'launch'/(name+'.launch.py'))],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env))
        if '--gui' in sys.argv:
            log=(out/'gui.log').open('w');logs.append(log)
            processes.append(subprocess.Popen(['ign','gazebo','-g','--gui-config',str(ROOT/'config/visible_gui.config')],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env={**env,'QT_QPA_PLATFORM':'xcb'}))
            time.sleep(25)
        if '--record' in sys.argv:
            if '--gui' not in sys.argv:raise ValueError('--record requires --gui')
            windows=subprocess.check_output(['xwininfo','-root','-tree'],env=env,text=True)
            match=re.search(r'^\s*(0x[0-9a-fA-F]+) "Gazebo":.*?\s(\d+)x(\d+)[+-]\d+[+-]\d+',windows,re.MULTILINE)
            if not match:raise RuntimeError('Gazebo window not found for recording')
            wid,width,height=match.groups()
            (out/'capture_window.json').write_text(json.dumps(dict(window_id=wid,width=int(width),height=int(height),
                source='native Gazebo GUI X11 window',playback_speed=1),indent=2)+'\n')
            log=(out/'recording.log').open('w');logs.append(log)
            recorder=subprocess.Popen(['ffmpeg','-nostdin','-v','warning','-f','x11grab','-framerate','8',
                '-window_id',str(int(wid,16)),'-video_size',width+'x'+height,'-draw_mouse','0',
                '-i',env.get('DISPLAY',':1')+'.0','-vf','scale=1280:-2',
                '-c:v','libx264','-preset','ultrafast','-crf','28','-pix_fmt','yuv420p',
                str(out/'gazebo_recording.mp4')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        code=subprocess.run(['python3',str(ROOT/'scripts/trial.py'),str(out)]+[a for a in ['--full','--top-only'] if a in sys.argv],env=env,timeout=900).returncode
        if code==0:
            # Include the independent, exact-name contact audit in the ordinary
            # one-run CLI as well as the multi-run verification harness.
            with (out/'evaluation.log').open('w') as validation_log:
                verified=subprocess.run(['python3',str(ROOT/'scripts/evaluate.py'),str(out)],
                    stdout=validation_log,stderr=subprocess.STDOUT,env=env)
            report=json.loads((out/'result.json').read_text())
            report['controller_trial_success']=report['success']
            report['recorded_contact_verification_passed']=verified.returncode==0
            if verified.returncode:
                report['success']=False;report['error']='Recorded-contact verification failed; see evaluation.log'
            (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
            code=verified.returncode
        return code
    finally:
        if recorder is not None:stop(recorder)
        for p in reversed(processes):stop(p)
        for f in logs:f.close()

if __name__=='__main__':raise SystemExit(main())
