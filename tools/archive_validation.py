#!/usr/bin/env python3
"""Archive software validation evidence outside source, excluding runtime secrets.

Run only after validation finishes. Installation files are fingerprinted, not
copied; deployment packaging and hardware acceptance are separate activities.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args])


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--exit-code', type=int, required=True)
    args = parser.parse_args()
    prefix = Path(args.prefix).resolve()
    output = Path(args.output).resolve()
    if output == ROOT or ROOT in output.parents:
        parser.error('evidence must be outside the source tree')
    if output == prefix.parent or output in prefix.parents or prefix.parent in output.parents:
        parser.error('evidence must be separate from the temporary output directory')
    log = Path(str(prefix) + '-full.log')
    install = Path(str(prefix) + '-install')
    if not log.is_file() or not install.is_dir():
        parser.error('full log and install directory must exist')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'source.patch').write_bytes(git('diff', '--binary', 'HEAD'))
    source_paths = git('ls-files', '--cached', '--others', '--exclude-standard', '-z').decode().split('\0')
    source_hashes = {name: digest(ROOT / name) for name in source_paths
                     if name and (ROOT / name).is_file()}
    write_json(output / 'source-sha256.json', source_hashes)
    untracked = git('ls-files', '--others', '--exclude-standard', '-z').decode().split('\0')
    for name in untracked:
        path = ROOT / name
        if name and path.is_file():
            target = output / 'untracked-source' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
    # Only explicit result/log evidence is copied. Never copy manager.json,
    # token files, profile bundles, or state journals from integration fixtures.
    evidence = []
    for path in sorted(prefix.parent.glob(prefix.name + '*')):
        if path.is_file() and path.suffix in ('.log', '.json'):
            evidence.append(path)
        elif path.is_dir() and ('-manager-' in path.name or path.name.endswith('-roslogs')):
            evidence.extend(p for p in path.rglob('*') if p.is_file()
                            and (p.suffix == '.log' or p.name == 'result.json'))
    for path in evidence:
        target = output / 'results' / path.relative_to(prefix.parent)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    write_json(output / 'install-sha256.json', {
        str(path.relative_to(install)): digest(path)
        for path in sorted(install.rglob('*')) if path.is_file()})
    write_json(output / 'run.json', {
        'archived_utc': datetime.now(timezone.utc).isoformat(),
        'command': ['bash', 'tools/validate_ros2.sh', str(prefix), 'full'],
        'exit_code': args.exit_code,
        'head': git('rev-parse', 'HEAD').decode().strip(),
        'status': git('status', '--short').decode(),
        'submodules': git('submodule', 'status', '--recursive').decode(),
        'submodule_status': git('submodule', 'foreach', '--recursive', 'git status --porcelain').decode(),
        'install': str(install),
        'scope': 'full offline software validation; no physical devices or deployment acceptance',
        'source_state': 'working tree snapshot; no commit or release created',
    })
    write_json(output / 'SHA256SUMS.json', {
        str(path.relative_to(output)): digest(path)
        for path in sorted(output.rglob('*')) if path.is_file()})
    print(output)


if __name__ == '__main__':
    main()
