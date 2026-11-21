"""Own one trial independently of an interactive command session."""
from pathlib import Path
import json
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
name=sys.argv[1]
if Path(name).name!=name or (ROOT/'outputs'/name).exists():raise ValueError('Use a fresh run name')
start=time.monotonic()
with (ROOT/'outputs'/(name+'.console.log')).open('w') as log:
    result=subprocess.run(['bash',str(ROOT/'scripts/run.sh'),*sys.argv[1:]],stdout=log,stderr=subprocess.STDOUT)
out=ROOT/'outputs'/name
(out/'job_status.json').write_text(json.dumps(dict(returncode=result.returncode,elapsed_wall_s=time.monotonic()-start),indent=2)+'\n')
if result.returncode==0:
    subprocess.run(['python3',str(ROOT/'scripts/evaluate.py'),str(out)],check=True)
subprocess.run(['python3',str(ROOT/'scripts/summarize.py')],check=True)
