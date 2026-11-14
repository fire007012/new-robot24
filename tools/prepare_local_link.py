#!/usr/bin/env python3
"""Prepare a persistent, local mock deployment from a complete ROS installation.

Does not start ROS, access hardware, or modify existing deployment state.
"""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import secrets
import shlex
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--advertise-host', default='127.0.0.1')
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[1]
    install = args.install.resolve(strict=True)
    output = args.output.resolve()
    address = ipaddress.ip_address(args.advertise_host)
    if address.version != 4 or address.is_unspecified or address.is_multicast:
        parser.error('advertise-host must be a reachable IPv4 address')
    if output.exists() or output == install or install in output.parents:
        parser.error('choose a new output directory outside the installation')
    for package in ('robot_bringup', 'host_bridge', 'car_control', 'arm_control'):
        if not (install / package / 'share' / package / 'package.xml').is_file():
            parser.error('incomplete installation: ' + package)
    if not (install / 'setup.bash').is_file():
        parser.error('installation setup.bash missing')
    manifest = install / 'release.json'
    build = 'robot24-field-control-2'
    if not manifest.exists():
        subprocess.run([sys.executable, '-B', str(source / 'robot_manager/release_manifest.py'),
                        '--root', str(install), '--build-version', build], check=True)
    if json.loads(manifest.read_text()).get('build_version') != build:
        parser.error('installation manifest build mismatch')
    output.mkdir(parents=True, mode=0o700)
    (output / 'profiles').mkdir()
    (output / 'state').mkdir(mode=0o700)
    token = output / 'manager.token'
    with open(token, 'x', opener=lambda path, flags: os.open(path, flags, 0o600)) as stream:
        stream.write(secrets.token_urlsafe(32) + '\n')
    profile = dict(profile_id='mock', config_revision='1', mode='mock',
                   enable_servo=True, build_version=build, video=False, video_required=False,
                   required_modules=['core', 'bridge'], optional_modules=[], devices=[], startup_timeout=120)
    (output / 'profiles/mock.json').write_text(json.dumps(profile, indent=2) + '\n')
    config = dict(enable_config_transactions=False, robot_id='robot24',
                  config_root=str(output / 'profiles'), state_dir=str(output / 'state'),
                  token_file=str(token), setup=str(install / 'setup.bash'), build_version=build,
                  bind='127.0.0.1' if address.is_loopback else '0.0.0.0', port=9091,
                  bridge_bind='127.0.0.1' if address.is_loopback else '0.0.0.0',
                  control_endpoint=dict(host=str(address), port=9090, protocol_version=2),
                  build_manifest=str(manifest), network_policy_confirmed=not address.is_loopback)
    (output / 'manager.json').write_text(json.dumps(config, indent=2) + '\n')
    command = ('#!/usr/bin/env bash\nset -eo pipefail\n'
               'export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1\n'
               f'export ROS_HOME={shlex.quote(str(output / "ros-home"))}\n'
               f'export ROS_LOG_DIR={shlex.quote(str(output / "ros-logs"))}\n'
               f'exec /usr/bin/python3 -B {shlex.quote(str(source / "robot_manager/manager.py"))}'
               f' --config {shlex.quote(str(output / "manager.json"))}\n')
    launcher = output / 'run_manager.sh'
    launcher.write_text(command)
    launcher.chmod(0o755)
    print(json.dumps(dict(config=str(output / 'manager.json'), launcher=str(launcher),
                          profile='mock', hardware_started=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
