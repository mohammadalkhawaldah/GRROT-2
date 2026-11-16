#!/usr/bin/env python3
"""Open native Gazebo, replay the contact trial, and retain the window afterward."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from prepare_robot import prepare
from verify_contact import stop

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--wait-for-start',action='store_true',help='Wait for outputs/visible_demo.start')
    args=parser.parse_args()
    # Preserve the previously measured successful run before replay writes logs.
    archive=ROOT/'outputs/successful_baseline'
    if not archive.exists():
        archive.mkdir()
        for name in ['contact_trial.json','contact_trace.csv','contact_commands.json','contact_motion.png']:
            src=ROOT/'outputs'/name
            if src.exists():shutil.copy2(src,archive/name)
    prepare()
    processes=[];logs=[]
    try:
        env={**os.environ,'CUP_IK_SOLVER':'pick_ik','QT_QPA_PLATFORM':'xcb'}
        for script,name in [('run_headless.sh','visible_server.log'),('run_planning.sh','visible_planning.log')]:
            log=(ROOT/'outputs'/name).open('w');logs.append(log)
            processes.append(subprocess.Popen(['bash',str(ROOT/'scripts'/script)],env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        log=(ROOT/'outputs/visible_gui.log').open('w');logs.append(log)
        gui=subprocess.Popen(['ign','gazebo','-g','-v','3','--gui-config',str(ROOT/'config/visible_gui.config')],
            env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(gui)
        print('Gazebo GUI starting; scene and planner running.',flush=True)
        if args.wait_for_start:
            trigger=ROOT/'outputs/visible_demo.start'
            while not trigger.exists():
                if gui.poll() is not None:raise RuntimeError('Gazebo GUI exited before replay')
                time.sleep(.25)
            trigger.unlink()
        else:
            # Give the desktop time to render before robot motion begins.
            for _ in range(40):
                if gui.poll() is not None:raise RuntimeError('Gazebo GUI exited before replay')
                time.sleep(.25)
        print('Replaying grasp, drag, release and retract.',flush=True)
        result=subprocess.run(['python3',str(ROOT/'scripts/contact_trial.py')],timeout=600)
        if (ROOT/'outputs/contact_trial.json').exists():
            shutil.copy2(ROOT/'outputs/contact_trial.json',ROOT/'outputs/visible_demo_result.json')
        print(f'Replay finished with exit code {result.returncode}; keeping Gazebo open. Close its window to stop.',flush=True)
        gui.wait()
        return result.returncode
    finally:
        for process in reversed(processes):stop(process)
        for log in logs:log.close()


if __name__=='__main__':raise SystemExit(main())
