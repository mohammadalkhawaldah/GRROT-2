"""Three gated fresh-world full trials, stopping at the first failure."""
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]

def main():
    names=sys.argv[1:]
    if len(names)!=3:raise ValueError('Provide three unused full_nominal_* run names')
    for name in names:
        if Path(name).name!=name or not name.startswith('full_nominal_') or (ROOT/'outputs'/name).exists():
            raise ValueError('Use fresh full_nominal_* names: '+name)
    for i,name in enumerate(names):
        args=['bash',str(ROOT/'scripts/run.sh'),name,'--full']
        if i==2:args+=['--gui','--record']
        with (ROOT/'outputs'/(name+'.console.log')).open('w') as log:
            result=subprocess.run(args,stdout=log,stderr=subprocess.STDOUT)
        path=ROOT/'outputs'/name/'result.json'
        report=json.loads(path.read_text()) if path.exists() else {}
        print(json.dumps(dict(run=name,runner_returncode=result.returncode,success=report.get('success'),error=report.get('error'))),flush=True)
        if result.returncode or not report.get('success'):return 1
        subprocess.run(['python3',str(ROOT/'scripts/evaluate.py'),str(path.parent)],check=True)
        subprocess.run(['python3',str(ROOT/'scripts/summarize.py')],check=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
