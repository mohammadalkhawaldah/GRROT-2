#!/usr/bin/env python3
"""Cache official APT packages in parallel, verifying signed-index SHA256 values.

This does not install anything. Install the resulting debs using apt-get.
"""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import shutil
import urllib.request

import apt

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ['ros-humble-gz-ros2-control', 'ros-humble-joint-state-broadcaster',
            'ros-humble-joint-trajectory-controller', 'ros-humble-gripper-controllers',
            'ros-humble-kortex-description', 'ros-humble-robotiq-controllers',
            'ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config', 'ros-humble-pick-ik']


def main():
    cache = apt.Cache()
    for name in PACKAGES:
        cache[name].mark_install(auto_fix=True, auto_inst=True, from_user=True)
    if any(p.marked_delete or (p.is_installed and p.marked_upgrade) for p in cache.get_changes()):
        raise RuntimeError('Refusing a plan that deletes or upgrades installed packages')
    out = ROOT / 'vendor/debs'
    out.mkdir(parents=True, exist_ok=True)
    records = []
    tasks = []
    for package in cache.get_changes():
        v = package.candidate
        filename = Path(v.filename).name
        dest = out / filename
        checksum = v.record['SHA256']
        record = dict(name=package.name, version=v.version, architecture=v.architecture,
                      uri=v.uri, size=v.size, sha256=checksum, filename=filename)
        records.append(record)
        cached = Path('/var/cache/apt/archives') / filename
        if cached.exists() and hashlib.sha256(cached.read_bytes()).hexdigest() == checksum:
            shutil.copyfile(cached, dest)
        if dest.exists() and hashlib.sha256(dest.read_bytes()).hexdigest() == checksum:
            continue
        # Bound each request so one slow connection cannot hold up the large mesh package.
        for start in range(0, v.size, 262144):
            end = min(v.size - 1, start + 262143)
            tasks.append((record, start, end))

    def download(task):
        rec, start, end = task
        part = out / (rec['filename'] + f'.part{start}')
        if part.exists() and part.stat().st_size == end - start + 1:
            return
        req = urllib.request.Request(rec['uri'], headers={'Range': f'bytes={start}-{end}'})
        for attempt in range(3):
            try:
                with urllib.request.urlopen(req, timeout=60) as response:
                    data = response.read()
                    if response.status == 200 and start == 0 and len(data) == rec['size']:
                        part.write_bytes(data[:end + 1])
                    elif response.status == 206 and len(data) == end - start + 1:
                        part.write_bytes(data)
                    else:
                        raise RuntimeError(f'Unexpected range response for {rec["name"]}')
                return
            except Exception:
                if attempt == 2:
                    raise

    if not records:
        print('Requested packages are already installed; existing manifest preserved', flush=True)
        return
    print(f'Fetching {len(tasks)} chunks for {len(records)} packages', flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
        for i, _ in enumerate(pool.map(download, tasks), 1):
            if i % 10 == 0:
                print(f'Downloaded {i}/{len(tasks)} chunks', flush=True)
    for rec in records:
        dest = out / rec['filename']
        if not dest.exists():
            with dest.open('wb') as f:
                for start in range(0, rec['size'], 262144):
                    f.write((out / (rec['filename'] + f'.part{start}')).read_bytes())
        if hashlib.sha256(dest.read_bytes()).hexdigest() != rec['sha256']:
            raise RuntimeError(f'SHA256 mismatch: {dest.name}')
    (ROOT / 'docs/dependency_manifest.json').write_text(json.dumps(records, indent=2) + '\n')
    print(f'Verified {len(records)} packages; no installation performed', flush=True)


if __name__ == '__main__':
    main()
