"""Operator presets over the existing four independent runtime groups."""

CAPABILITY = 'motion_presets_v1'
PRESETS = {
    'flippers': {'label': '摆臂', 'groups': ['flippers']},
    'base': {'label': '底盘', 'groups': ['base']},
    'arm': {'label': '机械臂（含夹爪）', 'groups': ['arm', 'gripper']},
}


def select(value):
    if (not isinstance(value, list) or not value or
            any(not isinstance(p, str) or p not in PRESETS for p in value) or
            len(set(value)) != len(value)):
        raise ValueError('select unique known presets')
    return sorted(value)


def groups(presets):
    return sorted({g for p in presets for g in PRESETS[p]['groups']})
