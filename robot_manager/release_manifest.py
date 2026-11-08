#!/usr/bin/env python3
"""Generate a release manifest after deployment preparation, never from API requests."""
import argparse
import hashlib
import json
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--root',required=True)
p.add_argument('--build-version',required=True)
a=p.parse_args()
root=Path(a.root).resolve(strict=True)
output=root/'release.json'
if output.exists():raise SystemExit('release.json already exists; use a new release directory')
files={}
for path in sorted(root.rglob('*')):
    if path.is_symlink() and (path.is_dir() or not path.resolve().is_relative_to(root)):
        raise SystemExit('release must contain actual directories and no external symlinks: '+str(path))
    if not path.is_file():continue
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):value.update(chunk)
    files[str(path.relative_to(root))]=value.hexdigest()
if not files:raise SystemExit('empty release')
output.write_text(json.dumps(dict(build_version=a.build_version,management_protocol=1,control_protocol=2,files=files),indent=2)+'\n')
