"""One explicit CAN topology for device checks and launch construction."""
import re
from group_selection import BACKENDS, selected_groups

FIELDS = {'RobotCanopenSystem': 'canopen_interface', 'CanDriverSystem': 'private_interface'}


def validate_can(profile):
    if profile['mode'] != 'hardware':
        return
    config = profile.get('can_interfaces')
    if (not isinstance(config, dict) or set(config) != set(FIELDS.values()) or
            any(not isinstance(v, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,15}', v)
                for v in config.values())):
        raise ValueError('hardware requires explicit can_interfaces: canopen_interface/private_interface')
    expected = {g: sorted({config[FIELDS[b]] for b in bs}) for g, bs in BACKENDS.items()}
    if ({g: sorted(set(v)) for g, v in profile.get('group_devices', {}).items()} != expected or
            set(profile.get('devices', [])) != set(config.values())):
        raise ValueError('devices/group_devices disagree with can_interfaces and backend ownership')


def effective_can(profile):
    validate_can(profile)
    if profile['mode'] != 'hardware':
        return {}
    config = profile['can_interfaces']
    groups = selected_groups(profile.get('requested_presets'))
    backends = set().union(*(BACKENDS[g] for g in groups))
    if len(backends) > 1 and len(set(config.values())) == 1:
        raise ValueError('CAN_TOPOLOGY: '+config['canopen_interface']+
                         ' shared CANopen/private backend topology is not commissioned; '
                         'use separately verified interfaces in hardware-full or test base/flippers separately')
    return dict(config)
