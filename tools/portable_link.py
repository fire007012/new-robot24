#!/usr/bin/env python3
"""Export a stopped Robot24 deployment; check it after relocation without starting ROS."""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile

SOURCE = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def copy_tree(source, target):
    # Materialize external/symlink-install files, so the result cannot reference source/build.
    shutil.copytree(source, target, symlinks=False,
                    ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))


def verify_deployment(install, config):
    result = subprocess.run([sys.executable, '-B', str(install / 'manager/deployment_check.py'),
                             '--config', str(config)], capture_output=True, text=True)
    if result.returncode:
        raise ValueError('deployment verification failed:\n' + result.stdout + result.stderr)


def pack(args):
    config_path = args.config.resolve(strict=True)
    config = read(config_path)
    if config.get('enable_config_transactions', False):
        raise ValueError('active configuration transactions are not supported by this exporter')
    resolve = lambda key: (config_path.parent / config[key]).resolve(strict=True)
    state = resolve('state_dir')
    install = args.install.resolve(strict=True)
    if resolve('setup') != install / 'setup.bash' or resolve('build_manifest') != install / 'release.json':
        raise ValueError('--install must match the configured setup and release manifest')
    if not (install / 'manager/manager.py').is_file():
        raise ValueError('install/manager missing; export a complete linked deployment')
    output = args.output.absolute()
    if output.exists() or output.is_symlink():
        raise ValueError('output already exists')
    for directory in (install, state, resolve('config_root')):
        if output.is_relative_to(directory):
            raise ValueError('output must be outside deployment input directories')
    # Acquire the same lock as manager, and hold it through the complete snapshot.
    with (state / 'lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        data = read(state / 'state.json')
        if (data.get('resources') != [] or data.get('software_state') != 'stopped'
                or any(op.get('state') in ('accepted', 'running')
                       for op in data.get('operations', {}).values())):
            raise ValueError('stop/reconcile all resources before export')
        verify_deployment(install, config_path)
        profiles = resolve('config_root')
        for path in profiles.glob('*.json'):
            for key in ('baseline', 'video_config'):
                value = read(path).get(key)
                if value and (Path(value).is_absolute() or
                              not (profiles / value).resolve().is_relative_to(profiles)):
                    raise ValueError(f'{path.name}: {key} must be relative inside config_root')
        with tempfile.TemporaryDirectory(prefix='robot24-export-') as temporary:
            root = Path(temporary) / 'robot24-portable'
            root.mkdir(mode=0o700)
            copy_tree(install, root / 'install')
            runtime = root / 'runtime'
            runtime.mkdir(mode=0o700)
            copy_tree(profiles, runtime / 'profiles')
            copy_tree(state, runtime / 'state')
            shutil.copy2(resolve('token_file'), runtime / 'manager.token')
            (runtime / 'manager.token').chmod(0o600)
            config.update(config_root='profiles', state_dir='state', token_file='manager.token',
                          setup='../install/setup.bash', build_manifest='../install/release.json')
            if args.advertise_host:
                host = ipaddress.IPv4Address(args.advertise_host)
                if host.is_unspecified or host.is_multicast:
                    raise ValueError('advertise-host must be a reachable unicast IPv4 address')
                config['control_endpoint']['host'] = str(host)
                config['bind'] = config['bridge_bind'] = '127.0.0.1' if host.is_loopback else '0.0.0.0'
                config['network_policy_confirmed'] = not host.is_loopback
            write(runtime / 'manager.json', config)
            launcher = root / 'run_manager.sh'
            launcher.write_text('''#!/usr/bin/env bash
set -eo pipefail
robot_bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export ROS_HOME="$robot_bundle_dir/runtime/ros-home"
export ROS_LOG_DIR="$robot_bundle_dir/runtime/ros-logs"
exec /usr/bin/python3 -B "$robot_bundle_dir/install/manager/manager.py" --config "$robot_bundle_dir/runtime/manager.json"
''')
            launcher.chmod(0o755)
            shutil.copy2(__file__, root / 'portable_link.py')
            shutil.copy2(SOURCE / 'tools/install_ros2_dependencies.sh', root / 'install_dependencies.sh')
            shutil.copy2(SOURCE / 'docs/上下位机联动迁移说明.md', root / 'README.md')
            write(root / 'platform.json', dict(machine=platform.machine(),
                  os_release=platform.freedesktop_os_release(), ros='humble',
                  build_version=config['build_version']))
            # Removing bytecode/materializing links changes the release; fingerprint the actual payload.
            (root / 'install/release.json').unlink()
            subprocess.run([sys.executable, '-B', str(root / 'install/manager/release_manifest.py'),
                            '--root', str(root / 'install'), '--build-version', config['build_version']], check=True)
            check(root, dependencies=False)
            output.parent.mkdir(parents=True, exist_ok=True)
            fd, temporary_archive = tempfile.mkstemp(dir=output.parent, prefix='.robot24-')
            try:
                os.close(fd)
                with tarfile.open(temporary_archive, 'w:gz') as archive:
                    archive.add(root, arcname=root.name)
                # Do not overwrite a destination created during export.
                os.link(temporary_archive, output)
            finally:
                os.unlink(temporary_archive)
    digest = sha256(output)
    print(json.dumps(dict(archive=str(output), sha256=digest, hardware_started=False), indent=2))


def sha256(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def check(root, dependencies=True):
    root = root.resolve(strict=True)
    expected = read(root / 'platform.json')
    actual = platform.freedesktop_os_release()
    if (expected['machine'] != platform.machine() or
            any(expected['os_release'].get(k) != actual.get(k) for k in ('ID', 'VERSION_ID'))):
        raise ValueError('OS/CPU differs from export host; rebuild on the target machine')
    verify_deployment(root / 'install', root / 'runtime/manager.json')
    if dependencies:
        # A clean environment prevents the original workspace from masking missing dependencies.
        env = {k: v for k, v in os.environ.items() if k not in (
            'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
            'LD_LIBRARY_PATH', 'PYTHONPATH', 'COLCON_CURRENT_PREFIX')}
        subprocess.run(['/bin/bash', '-c', '''set -eo pipefail
source /opt/ros/humble/setup.bash
source "$1/install/local_setup.bash"
ros2 pkg prefix "${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" >/dev/null
for robot_pkg in robot_bringup host_bridge robot_hardware; do
  robot_prefix="$(ros2 pkg prefix "$robot_pkg")"
  [[ "$robot_prefix" == "$1/install/"* ]] || exit 1
done
exec /usr/bin/python3 -B "$1/portable_link.py" libraries --root "$1"
''', 'robot24-check', str(root)], env=env, check=True)
    print('PASS: deployment files verified' + ('; ROS packages and ELF dependencies verified' if dependencies else ''))


def libraries(root):
    failures = []
    for path in (root / 'install').rglob('*'):
        if not path.is_file():
            continue
        with path.open('rb') as stream:
            magic = stream.read(4)
        if magic != b'\x7fELF':
            continue
        result = subprocess.run(['ldd', str(path)], capture_output=True, text=True)
        missing = [line.strip() for line in (result.stdout + result.stderr).splitlines() if 'not found' in line]
        for line in result.stdout.splitlines():
            if '=>' not in line:
                continue
            resolved = line.split('=>', 1)[1].rsplit(' (', 1)[0].strip()
            if resolved.startswith('/') and not any(
                    Path(resolved).is_relative_to(base)
                    for base in (root / 'install', Path('/opt'), Path('/usr'), Path('/lib'), Path('/lib64'))):
                missing.append('external non-system dependency: ' + resolved)
        if missing:
            failures.append(dict(file=str(path.relative_to(root)), missing=missing))
    if failures:
        raise ValueError(json.dumps(failures, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    export = sub.add_parser('pack')
    export.add_argument('--install', type=Path, required=True)
    export.add_argument('--config', type=Path, required=True)
    export.add_argument('--output', type=Path, required=True)
    export.add_argument('--advertise-host', help='target lower-computer IPv4 as reachable from Qt')
    for command in ('check', 'libraries'):
        sub.add_parser(command).add_argument('--root', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    try:
        if args.command == 'pack':
            pack(args)
        elif args.command == 'check':
            check(args.root)
        else:
            libraries(args.root)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        parser.exit(1, str(error) + '\n')


if __name__ == '__main__':
    main()
