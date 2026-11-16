#!/usr/bin/env python3
"""Own one isolated launch and always terminate its process group after testing."""
import os
from pathlib import Path
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    result = 1
    with (ROOT / 'outputs/headless.log').open('w') as log:
        launch = subprocess.Popen(['bash', str(ROOT / 'scripts/run_headless.sh')],
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            test = subprocess.run(['python3', str(ROOT / 'scripts/smoke_test.py')], timeout=300)
            result = test.returncode
        finally:
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)
                try:
                    launch.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGTERM)
                    try:
                        launch.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(launch.pid, signal.SIGKILL)
                        launch.wait()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
