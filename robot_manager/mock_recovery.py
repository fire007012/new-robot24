"""Bounded cleanup of a failed, software-only mock launch; never hardware."""
import os
from pathlib import Path
import signal
import sys
import time

def process(pid):
    try:
        root = Path('/proc') / str(pid)
        fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return dict(pid=pid, group=int(fields[2]), session=int(fields[3]),
                    start=fields[19], uid=root.stat().st_uid,
                    executable=str((root / 'exe').resolve(strict=True)),
                    args=(root / 'cmdline').read_bytes().decode().rstrip('\0').split('\0'))
    except FileNotFoundError:
        return None


def _mock_runtime(args, state_dir):
    import yaml  # Only the explicit mock-recovery path needs the deployment YAML reader.
    paths = [args[i + 1] for i, arg in enumerate(args[:-1]) if arg == '--params-file']
    for name in paths:
        path = Path(name)
        if not path.is_absolute() or path.stat().st_size > 1048576:
            continue
        # BaseLoader reads launch's python/tuple YAML tag as data, never Python objects.
        data = yaml.load(path.read_text(), Loader=yaml.BaseLoader)
        params = data.get('/**', {}).get('ros__parameters', {})
        if (params.get('simulation') == 'true'
                and Path(params.get('journal', '')).resolve() == state_dir / 'maintenance.json'):
            return True
    return False


def _allowed(member, module, install_root, state_dir):
    args = member['args']
    if not args:
        return False
    # Verify the executable/script position, not an arbitrary later argument.
    python = Path(member['executable']) == Path(sys.executable).resolve()
    program = Path(args[1] if python and len(args) > 1 else args[0])
    if not program.is_absolute():
        return False
    if program == Path('/opt/ros/humble/bin/ros2') and python:
        launch = ['launch', 'robot_bringup', 'full_system.launch.py'] if module == 'core' else ['launch', 'host_bridge', 'host_bridge.launch.py']
        return args[2:5] == launch and (module != 'core' or 'mock:=true' in args[5:])
    core = {
        'robot_hardware/lib/robot_hardware/robot_runtime',
        'robot_bringup/lib/robot_bringup/model_joint_states',
        'mobility_control/lib/mobility_control/base_cmd_node',
        'arm_control/lib/arm_control/gripper_cmd_node',
        'arm_control/lib/arm_control/wrist_roll_service_node',
    }
    allowed = core if module == 'core' else {'host_bridge/lib/host_bridge/host_bridge_node.py'}
    if program not in {install_root / item for item in allowed}:
        return False
    if program.name == 'robot_runtime':
        return _mock_runtime(args, state_dir)
    if module == 'bridge':
        return any(arg == '--control-state-dir' and i + 1 < len(args)
                   and Path(args[i + 1]).resolve() == state_dir / 'control'
                   for i, arg in enumerate(args))
    return True


def verified_members(profile, resources, install_root, state_dir):
    if profile.get('mode') != 'mock':
        raise ValueError('mock cleanup cannot be used for hardware')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    groups = {}
    for resource in resources:
        pid, identity = resource.get('pid'), resource.get('identity')
        if (resource.get('pending') or resource.get('module') not in ('core', 'bridge')
                or type(pid) is not int or pid <= 1 or pid in groups
                or not isinstance(identity, list) or len(identity) != 2
                or identity[0] != boot or not str(identity[1]).isdigit()):
            raise ValueError('mock resource identity is not verifiable')
        stat = Path('/proc') / str(pid) / 'stat'
        try:
            fields = stat.read_text().rsplit(')', 1)[1].split()
            if fields[19] != identity[1]:
                raise ValueError('mock resource leader PID has been reused')
        except FileNotFoundError:
            pass
        groups[pid] = resource
    members = []
    for directory in Path('/proc').glob('[0-9]*'):
        try:
            fields = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) not in groups:
                continue
        except FileNotFoundError:
            continue
        member = process(int(directory.name))
        if member is None or member['group'] not in groups:
            continue
        resource = groups[member['group']]
        if (member['session'] != resource['pid'] or member['uid'] != os.getuid()
                or int(member['start']) < int(resource['identity'][1])
                or not _allowed(member, resource['module'], install_root, state_dir)):
            raise ValueError('mock process group contains an unverified process: ' + str(member['pid']))
        members.append(member)
    return members


def recover_mock_resources(profile, resources, install_root, state_dir):
    """Signal only checked mock PIDs using pidfds; preserve records on any uncertainty.

    This deliberately rejects controller-manager/MoveIt processes: the fallback is
    for this failed partial software launch, not a substitute for runtime halt.
    """
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise ValueError('mock recovery requires pidfd support')
    install_root, state_dir = Path(install_root).resolve(), Path(state_dir).resolve()
    stopped = set()
    for stop_signal, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        members = verified_members(profile, resources, install_root, state_dir)
        if not members:
            return sorted(stopped)
        handles = []
        try:
            # Validate all members before sending any signal in this stage.
            for member in members:
                try:
                    handle = os.pidfd_open(member['pid'])
                except ProcessLookupError:
                    continue
                handles.append((handle, member))
                current = process(member['pid'])
                if current is not None and current != member:
                    raise ValueError('mock process changed during identity verification')
            for handle, member in handles:
                try:
                    signal.pidfd_send_signal(handle, stop_signal)
                    stopped.add(member['pid'])
                except ProcessLookupError:
                    pass
        finally:
            for handle, _ in handles:
                os.close(handle)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not verified_members(profile, resources, install_root, state_dir):
                return sorted(stopped)
            time.sleep(.05)
    if verified_members(profile, resources, install_root, state_dir):
        raise ValueError('verified mock processes remain after cleanup')
    return sorted(stopped)
