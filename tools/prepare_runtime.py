#!/usr/bin/env python3
"""Prepare a username-independent local runtime directory for terminal run_manager.sh."""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys


def compute_revision(profile: dict) -> str:
    body = {k: v for k, v in profile.items() if k != 'config_revision'}
    payload = json.dumps(body, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()
    return hashlib.sha256(payload).hexdigest()


def ensure_release_manifest(install_dir: Path, source_root: Path, requested_version: str = '') -> str:
    manifest_path = install_dir / 'release.json'
    if manifest_path.is_file():
        data = json.loads(manifest_path.read_text())
        version = data.get('build_version')
        if isinstance(version, str) and version:
            return version
    version = requested_version or 'robot24-local-dev'
    subprocess.run(
        [
            sys.executable,
            '-B',
            str(source_root / 'robot_manager/release_manifest.py'),
            '--root',
            str(install_dir),
            '--build-version',
            version,
        ],
        check=True,
    )
    return version


def seed_profiles(profiles_dir: Path, source_root: Path, build_version: str) -> None:
    profiles_dir.mkdir(parents=True, exist_ok=True)
    hardware_cfg = source_root / 'robot_hardware/config'
    baseline_map = {
        'commissioning-full-registered.yaml': hardware_cfg / 'full_registered_baseline.yaml',
        'commissioning-arm-registered.yaml': hardware_cfg / 'arm_registered_baseline.yaml',
    }
    for target_name, src_path in baseline_map.items():
        dst_path = profiles_dir / target_name
        if not dst_path.exists() and src_path.is_file():
            shutil.copy2(src_path, dst_path)

    mock_path = profiles_dir / 'mock.json'
    if not mock_path.exists():
        mock_profile = {
            'profile_id': 'mock',
            'mode': 'mock',
            'enable_servo': True,
            'build_version': build_version,
            'video': False,
            'video_required': False,
            'required_modules': ['core', 'bridge'],
            'optional_modules': [],
            'devices': [],
            'startup_timeout': 120,
        }
        mock_profile['config_revision'] = compute_revision(mock_profile)
        mock_path.write_text(json.dumps(mock_profile, indent=2, ensure_ascii=False) + '\n')
    else:
        mock_profile = json.loads(mock_path.read_text())
        if mock_profile.get('build_version') != build_version:
            mock_profile['build_version'] = build_version
            mock_profile['config_revision'] = compute_revision(mock_profile)
            mock_path.write_text(json.dumps(mock_profile, indent=2, ensure_ascii=False) + '\n')

    hw_path = profiles_dir / 'hardware.json'
    baseline_file = 'commissioning-full-registered.yaml'
    if not (profiles_dir / baseline_file).is_file() and (profiles_dir / 'commissioning-arm-registered.yaml').is_file():
        baseline_file = 'commissioning-arm-registered.yaml'
    if not hw_path.exists() and (profiles_dir / baseline_file).is_file():
        hw_profile = {
            'profile_id': 'hardware',
            'mode': 'hardware',
            'robot_id': 'robot24',
            'enable_servo': True,
            'build_version': build_version,
            'video': False,
            'video_required': False,
            'required_modules': ['core', 'bridge'],
            'optional_modules': [],
            'baseline': baseline_file,
            'devices': ['can0', 'can1'],
            'group_devices': {
                'base': ['can0'],
                'gripper': ['can0'],
                'flippers': ['can1'],
                'arm': ['can0', 'can1'],
            },
            'can_interfaces': {
                'private_interface': 'can0',
                'canopen_interface': 'can1',
            },
            'startup_timeout': 90,
        }
        hw_profile['config_revision'] = compute_revision(hw_profile)
        hw_path.write_text(json.dumps(hw_profile, indent=2, ensure_ascii=False) + '\n')
    elif hw_path.exists():
        hw_profile = json.loads(hw_path.read_text())
        if hw_profile.get('build_version') != build_version:
            hw_profile['build_version'] = build_version
            hw_profile['config_revision'] = compute_revision(hw_profile)
            hw_path.write_text(json.dumps(hw_profile, indent=2, ensure_ascii=False) + '\n')


def prepare_runtime(
    ws_root: Path,
    install_dir: Path,
    runtime_dir: Path,
    bind: str = '0.0.0.0',
    advertise_host: str = '127.0.0.1',
    port: int = 9091,
    control_port: int = 9090,
) -> Path:
    ws_root = ws_root.resolve(strict=True)
    source_root = ws_root / 'src'
    install_dir = install_dir.resolve(strict=True)
    if not (install_dir / 'setup.bash').is_file():
        raise ValueError(f'ROS2 setup.bash missing in {install_dir}')
    addr = ipaddress.ip_address(advertise_host)
    if addr.version != 4 or addr.is_unspecified or addr.is_multicast:
        raise ValueError(f'invalid advertise_host: {advertise_host}')

    build_version = ensure_release_manifest(install_dir, source_root)

    runtime_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(runtime_dir, 0o700)
    state_dir = runtime_dir / 'state'
    state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.chmod(state_dir, 0o700)

    # Refuse to mutate runtime configuration while a manager instance holds the lock.
    with (state_dir / 'lock').open('a') as lock_file:
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f'manager already running against {state_dir}') from exc

        token_path = runtime_dir / 'manager.token'
        if token_path.exists():
            token = token_path.read_text().strip()
            if len(token) < 32:
                raise ValueError(f'existing token in {token_path} has fewer than 32 characters')
            os.chmod(token_path, 0o600)
        else:
            with open(token_path, 'x', opener=lambda p, f: os.open(p, f, 0o600)) as stream:
                stream.write(secrets.token_urlsafe(32) + '\n')

        seed_profiles(runtime_dir / 'profiles', source_root, build_version)

        config_path = runtime_dir / 'manager.json'
        config = json.loads(config_path.read_text()) if config_path.is_file() else {}
        config.update(
            {
                'robot_id': config.get('robot_id', 'robot24'),
                'bind': bind,
                'bridge_bind': config.get('bridge_bind', bind),
                'port': port,
                'config_root': 'profiles',
                'state_dir': 'state',
                'token_file': 'manager.token',
                'setup': str(install_dir / 'setup.bash'),
                'build_manifest': str(install_dir / 'release.json'),
                'build_version': build_version,
                'enable_config_transactions': config.get('enable_config_transactions', False),
                'network_policy_confirmed': config.get('network_policy_confirmed', True),
                'control_endpoint': {
                    'host': advertise_host,
                    'port': control_port,
                    'protocol_version': 2,
                },
            }
        )
        tmp = config_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(config, indent=2, ensure_ascii=False) + '\n')
        tmp.replace(config_path)
    return config_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    ws_default = Path(__file__).resolve().parents[2]
    runtime_default = Path(os.environ.get('ROBOT24_RUNTIME_DIR') or (Path.home() / '.local/share/robot24'))
    parser.add_argument('--ws-root', type=Path, default=ws_default)
    parser.add_argument('--install', type=Path, default=None)
    parser.add_argument('--runtime', type=Path, default=runtime_default)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--advertise-host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9091)
    parser.add_argument('--control-port', type=int, default=9090)
    args = parser.parse_args()

    install_dir = args.install or (args.ws_root / 'install')
    try:
        config_path = prepare_runtime(
            ws_root=args.ws_root,
            install_dir=install_dir,
            runtime_dir=args.runtime,
            bind=args.bind,
            advertise_host=args.advertise_host,
            port=args.port,
            control_port=args.control_port,
        )
    except Exception as exc:
        print(f'prepare_runtime error: {exc}', file=sys.stderr)
        return 1
    print(json.dumps({'config': str(config_path), 'install': str(install_dir.resolve()), 'runtime': str(args.runtime.resolve())}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
