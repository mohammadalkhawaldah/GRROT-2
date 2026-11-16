#!/usr/bin/env python3
"""Run one programmed contact trial in a fresh owned world and planner."""
import os
from pathlib import Path
import signal
import subprocess

ROOT=Path(__file__).resolve().parents[1]
from prepare_robot import prepare


def stop(process):
    if process.poll() is not None:return
    process.send_signal(signal.SIGINT)
    try:process.wait(timeout=25)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait()


def main():
    # Ensure both processes load the same generated model, even after a model
    # change. The server launch regenerates identical content atomically.
    prepare()
    processes=[]
    with (ROOT/'outputs/contact_headless.log').open('w') as server_log, \
         (ROOT/'outputs/contact_planning.log').open('w') as planner_log:
        try:
            for script,log in [('run_headless.sh',server_log),('run_planning.sh',planner_log)]:
                processes.append(subprocess.Popen(['bash',str(ROOT/'scripts'/script)],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
                    env={**os.environ,'CUP_IK_SOLVER':'pick_ik'}))
            return subprocess.run(['python3',str(ROOT/'scripts/contact_trial.py')],timeout=480).returncode
        finally:
            for process in reversed(processes):stop(process)


if __name__=='__main__':raise SystemExit(main())
