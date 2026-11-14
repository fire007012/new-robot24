"""Requirements of a requested motion selection; no device I/O or motion grants."""
PRESETS = {'base': {'base'}, 'flippers': {'flippers'}, 'arm': {'arm', 'gripper'}}
BACKENDS = {'base': {'CanDriverSystem'}, 'gripper': {'CanDriverSystem'},
            'flippers': {'RobotCanopenSystem'}, 'arm': {'RobotCanopenSystem', 'CanDriverSystem'}}
CONTROLLERS = {'base': {'wheel_controller'}, 'gripper': {'gripper_controller'},
               'flippers': {'flipper_csp_controller', 'flipper_csv_forward_controller'},
               'arm': {'arm_position_controller'}}


def selected_groups(presets=None):
    if presets is None:
        return set(BACKENDS)  # Legacy clients retain their whole-vehicle checks.
    if (not isinstance(presets, list) or not presets or
            any(not isinstance(p, str) or p not in PRESETS for p in presets) or
            len(set(presets)) != len(presets)):
        raise ValueError('presets must select unique base/flippers/arm entries')
    return set().union(*(PRESETS[p] for p in presets))


def requirements(presets=None):
    groups = selected_groups(presets)
    return (groups, {'joint_state_broadcaster'} | set().union(*(CONTROLLERS[g] for g in groups)),
            set().union(*(BACKENDS[g] for g in groups)))


def required_devices(profile):
    mapping = profile.get('group_devices')
    if mapping is None or 'requested_presets' not in profile:
        return profile.get('devices', [])
    return sorted(set().union(*(set(mapping[g]) for g in selected_groups(profile['requested_presets']))))
