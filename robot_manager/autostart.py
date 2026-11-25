#!/usr/bin/env python3
"""Prepare or remove systemd autostart for Robot24 on the target lower-computer host.

Does not install system-wide files unless 'install' is explicitly requested.
"""
import argparse
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys


def _copy_tree(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(
        src,
        dst,
        symlinks=False,
        ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '*.pyo'),
    )


def _check_stopped(state_file: Path, force: bool) -> None:
    if force or not state_file.is_file():
        return
    data = json.loads(state_file.read_text())
    active = any(op.get('state') in ('accepted', 'running') for op in data.get('operations', {}).values())
    if data.get('resources') or data.get('software_state') not in (None, 'stopped') or active:
        raise RuntimeError(
            f'refusing deployment change while managed state is active in {state_file}; stop robot first'
        )


def _resolve_prefix(root_prefix: Path, absolute_path: str) -> Path:
    return root_prefix / absolute_path.lstrip('/')


def plan_layout(ws_root: Path, root_prefix: Path) -> dict:
    install_src = (ws_root / 'install').resolve()
    manager_src = (ws_root / 'src/robot_manager').resolve()
    return {
        'source_install': str(install_src),
        'source_manager': str(manager_src),
        'install_exists': (install_src / 'setup.bash').is_file(),
        'manager_exists': (manager_src / 'manager.py').is_file(),
        'targets': {
            'opt_install': str(_resolve_prefix(root_prefix, '/opt/robot24/install')),
            'opt_manager': str(_resolve_prefix(root_prefix, '/opt/robot24/manager')),
            'etc_config': str(_resolve_prefix(root_prefix, '/etc/robot24/manager.json')),
            'etc_token': str(_resolve_prefix(root_prefix, '/etc/robot24/manager.token')),
            'etc_profiles': str(_resolve_prefix(root_prefix, '/etc/robot24/profiles')),
            'state_dir': str(_resolve_prefix(root_prefix, '/var/lib/robot24-manager')),
            'systemd_unit': str(_resolve_prefix(root_prefix, '/etc/systemd/system/robot24-manager.service')),
            'logrotate': str(_resolve_prefix(root_prefix, '/etc/logrotate.d/robot24-manager')),
        },
        'can_installer': str((ws_root / 'src/robot_bringup/scripts/install_can_autosetup.sh').resolve()),
    }


def install_autostart(
    ws_root: Path,
    root_prefix: Path = Path('/'),
    source_runtime: Path | None = None,
    bind: str = '0.0.0.0',
    advertise_host: str = '127.0.0.1',
    port: int = 9091,
    control_port: int = 9090,
    with_can: bool = False,
    enable_service: bool = True,
    force: bool = False,
) -> dict:
    ws_root = ws_root.resolve(strict=True)
    root_prefix = root_prefix.resolve()
    is_live_system = root_prefix == Path('/')

    if is_live_system and os.geteuid() != 0:
        raise PermissionError('system autostart installation requires root (sudo)')

    install_src = (ws_root / 'install').resolve(strict=True)
    manager_src = (ws_root / 'src/robot_manager').resolve(strict=True)
    if not (install_src / 'setup.bash').is_file() or not (manager_src / 'manager.py').is_file():
        raise FileNotFoundError('complete ROS2 install and robot_manager sources are required')

    state_dir = _resolve_prefix(root_prefix, '/var/lib/robot24-manager')
    _check_stopped(state_dir / 'state.json', force=force)
    if source_runtime:
        _check_stopped(source_runtime / 'state/state.json', force=force)

    opt_root = _resolve_prefix(root_prefix, '/opt/robot24')
    opt_install = opt_root / 'install'
    opt_manager = opt_root / 'manager'
    etc_root = _resolve_prefix(root_prefix, '/etc/robot24')
    etc_profiles = etc_root / 'profiles'
    unit_path = _resolve_prefix(root_prefix, '/etc/systemd/system/robot24-manager.service')
    logrotate_path = _resolve_prefix(root_prefix, '/etc/logrotate.d/robot24-manager')

    opt_root.mkdir(parents=True, exist_ok=True)
    _copy_tree(install_src, opt_install)
    _copy_tree(manager_src, opt_manager)
    if (opt_manager / 'tests').exists():
        shutil.rmtree(opt_manager / 'tests')

    manifest_path = opt_install / 'release.json'
    build_version = 'robot24-release-1'
    if manifest_path.is_file():
        build_version = json.loads(manifest_path.read_text()).get('build_version') or build_version
        manifest_path.unlink()
    subprocess.run(
        [
            sys.executable,
            '-B',
            str(opt_manager / 'release_manifest.py'),
            '--root',
            str(opt_install),
            '--build-version',
            build_version,
        ],
        check=True,
    )

    etc_root.mkdir(parents=True, exist_ok=True)
    etc_profiles.mkdir(parents=True, exist_ok=True)
    state_dir.mkdir(parents=True, exist_ok=True, mode=0o700)

    if source_runtime and (source_runtime / 'profiles').is_dir():
        for item in (source_runtime / 'profiles').iterdir():
            if item.is_file():
                shutil.copy2(item, etc_profiles / item.name)
    else:
        from tools.prepare_runtime import seed_profiles  # type: ignore

        seed_profiles(etc_profiles, ws_root / 'src', build_version)

    token_path = etc_root / 'manager.token'
    if not token_path.exists():
        if source_runtime and (source_runtime / 'manager.token').is_file():
            shutil.copy2(source_runtime / 'manager.token', token_path)
        else:
            with open(token_path, 'x', opener=lambda p, f: os.open(p, f, 0o600)) as stream:
                stream.write(secrets.token_urlsafe(32) + '\n')
    os.chmod(token_path, 0o600)

    config_data = {
        'robot_id': 'robot24',
        'bind': bind,
        'bridge_bind': bind,
        'port': port,
        'config_root': '/etc/robot24/profiles',
        'state_dir': '/var/lib/robot24-manager',
        'token_file': '/etc/robot24/manager.token',
        'setup': '/opt/robot24/install/setup.bash',
        'build_manifest': '/opt/robot24/install/release.json',
        'build_version': build_version,
        'enable_config_transactions': False,
        'network_policy_confirmed': True,
        'control_endpoint': {
            'host': advertise_host,
            'port': control_port,
            'protocol_version': 2,
        },
    }
    config_path = etc_root / 'manager.json'
    config_path.write_text(json.dumps(config_data, indent=2, ensure_ascii=False) + '\n')

    unit_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(manager_src / 'systemd/robot24-manager.service', unit_path)
    logrotate_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(manager_src / 'systemd/robot24-manager.logrotate', logrotate_path)

    if is_live_system:
        subprocess.run(['id', '-u', 'robot24'], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if subprocess.run(['id', '-u', 'robot24'], capture_output=True).returncode != 0:
            subprocess.run(['useradd', '--system', '--no-create-home', '--shell', '/usr/sbin/nologin', 'robot24'], check=True)
        for grp in ('dialout', 'video'):
            if subprocess.run(['getent', 'group', grp], capture_output=True).returncode == 0:
                subprocess.run(['usermod', '-aG', grp, 'robot24'], check=False)
        subprocess.run(['chown', '-R', 'robot24:robot24', str(state_dir)], check=True)
        subprocess.run(['chown', 'robot24:robot24', str(token_path)], check=True)
        subprocess.run(['chown', '-R', 'root:robot24', str(etc_root)], check=False)
        if with_can:
            subprocess.run(['bash', str(ws_root / 'src/robot_bringup/scripts/install_can_autosetup.sh')], check=True)
        subprocess.run(['systemctl', 'daemon-reload'], check=True)
        if enable_service:
            subprocess.run(['systemctl', 'enable', '--now', 'robot24-manager.service'], check=True)

    return {
        'installed': True,
        'live_system': is_live_system,
        'root_prefix': str(root_prefix),
        'build_version': build_version,
        'config': str(config_path),
        'unit': str(unit_path),
    }


def uninstall_autostart(root_prefix: Path = Path('/'), purge: bool = False, force: bool = False) -> dict:
    root_prefix = root_prefix.resolve()
    is_live_system = root_prefix == Path('/')
    if is_live_system and os.geteuid() != 0:
        raise PermissionError('system autostart removal requires root (sudo)')

    state_dir = _resolve_prefix(root_prefix, '/var/lib/robot24-manager')
    _check_stopped(state_dir / 'state.json', force=force)

    if is_live_system:
        subprocess.run(['systemctl', 'disable', '--now', 'robot24-manager.service'], check=False)

    for rel in ('/etc/systemd/system/robot24-manager.service', '/etc/logrotate.d/robot24-manager'):
        target = _resolve_prefix(root_prefix, rel)
        if target.exists():
            target.unlink()

    opt_root = _resolve_prefix(root_prefix, '/opt/robot24')
    if opt_root.exists():
        shutil.rmtree(opt_root)

    if purge:
        for rel in ('/etc/robot24', '/var/lib/robot24-manager'):
            target = _resolve_prefix(root_prefix, rel)
            if target.exists():
                shutil.rmtree(target)

    if is_live_system:
        subprocess.run(['systemctl', 'daemon-reload'], check=False)

    return {'uninstalled': True, 'live_system': is_live_system, 'purged': purge}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    ws_default = Path(__file__).resolve().parents[2]

    for name in ('plan', 'install', 'uninstall'):
        p = sub.add_parser(name)
        p.add_argument('--ws-root', type=Path, default=ws_default)
        p.add_argument('--root-prefix', type=Path, default=Path('/'))
        if name == 'install':
            p.add_argument('--source-runtime', type=Path, default=None)
            p.add_argument('--bind', default='0.0.0.0')
            p.add_argument('--advertise-host', default='127.0.0.1')
            p.add_argument('--port', type=int, default=9091)
            p.add_argument('--control-port', type=int, default=9090)
            p.add_argument('--with-can', action='store_true')
            p.add_argument('--no-enable', action='store_true')
            p.add_argument('--force', action='store_true')
        elif name == 'uninstall':
            p.add_argument('--purge', action='store_true')
            p.add_argument('--force', action='store_true')

    args = parser.parse_args()
    sys.path.insert(0, str(args.ws_root / 'src'))
    try:
        if args.action == 'plan':
            result = plan_layout(args.ws_root, args.root_prefix)
        elif args.action == 'install':
            result = install_autostart(
                ws_root=args.ws_root,
                root_prefix=args.root_prefix,
                source_runtime=args.source_runtime,
                bind=args.bind,
                advertise_host=args.advertise_host,
                port=args.port,
                control_port=args.control_port,
                with_can=args.with_can,
                enable_service=not args.no_enable,
                force=args.force,
            )
        else:
            result = uninstall_autostart(root_prefix=args.root_prefix, purge=args.purge, force=args.force)
    except Exception as exc:
        print(f'autostart error: {exc}', file=sys.stderr)
        return 1

    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
