#!/usr/bin/env python3
"""Register operator-confirmed current flipper parameters without device I/O."""
import argparse
import copy
import hashlib
import math
from pathlib import Path
import yaml

FLIPPERS = {'left_front_arm_joint': 1, 'right_front_arm_joint': 2,
            'left_rear_arm_joint': 3, 'right_rear_arm_joint': 4}
COUNTS_PER_REV = 19857408


def register(baseline, registration, bus):
    if registration.get('registration_status') != 'current_configuration_registered':
        raise ValueError('explicit current-configuration confirmation required')
    if registration.get('canopen_interface') != 'can1':
        raise ValueError('CANopen must use can1')
    result = copy.deepcopy(baseline)
    existing = {entry['name']: entry for entry in result['canopen_parameters']['joints']}
    for joint, node in FLIPPERS.items():
        spec, entry, configured = registration['axes'][joint], result['axes'][joint], bus[joint]
        if any(value != node for value in (spec['node_id'], entry['node_id'],
                                          existing[joint]['canopen']['node_id'], configured['node_id'])):
            raise ValueError(joint + ': Node-ID mismatch')
        if (spec['counts_per_rev'] != COUNTS_PER_REV or existing[joint]['counts_per_rev'] != COUNTS_PER_REV
                or spec['default_mode'] != 9 or existing[joint]['canopen']['default_mode'] != 9
                or spec['position_mode'] != configured['position_mode'] or configured['position_mode'] != 8
                or spec['velocity_mode'] != configured['velocity_mode'] or configured['velocity_mode'] != 9
                or not math.isclose(configured['scale_pos_to_dev'], COUNTS_PER_REV / (2 * math.pi))
                or not math.isclose(configured['scale_vel_to_dev'], COUNTS_PER_REV / (2 * math.pi))
                or not spec['physical_label'] or spec['direction_verified'] is not True):
            raise ValueError(joint + ': inconsistent operator configuration')
        entry.update(physical_label=spec['physical_label'], direction_verified=True)
        # Keep unknown serial numbers, firmware and persistent device values unknown.
        entry['parameter_registration'] = {
            'mode': 'operator_confirmed_configuration', 'confirmed': True,
            'confirmed_by': 'operator', 'date': registration['date'],
            'counts_per_rev': COUNTS_PER_REV, 'default_mode': 9,
            'position_mode': configured['position_mode'], 'velocity_mode': configured['velocity_mode'],
            'canopen_interface': 'can1', 'identity_readback_required_before_registration': False}
    result.update(schema_version='robot24-parameter-baseline-v2', parameters_verified=True,
                  commissioned_groups=sorted(set(result.get('commissioned_groups', [])) | {'flippers'}),
                  commissioned_scope='selected_groups_operator_confirmed_configuration')
    result.setdefault('group_registration', {})['flippers'] = {
        'parameters_verified': True, 'axes': sorted(FLIPPERS),
        'registration_mode': 'operator_confirmed_configuration',
        'identity_requirement': 'runtime_communication_checks; no prior identity snapshot required'}
    result['flippers_registration'] = {
        'status': 'registered', 'mode': 'operator_confirmed_configuration', 'date': registration['date'],
        'basis': registration['basis'], 'axes': list(FLIPPERS), 'device_io_performed': False,
        'identity_readback_required_before_registration': False,
        'device_identity_complete': all(value is not None for joint in FLIPPERS
                                        for value in result['axes'][joint]['identity'].values()),
        'source_sha256': registration['source_sha256']}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('baseline', 'registration', 'bus', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    load = lambda path: yaml.safe_load(path.read_text())
    value = register(load(args.baseline), load(args.registration), load(args.bus))
    with args.output.open('x') as stream:
        yaml.safe_dump(value, stream, allow_unicode=True, sort_keys=False)
    print('Registered flippers configuration baseline:', args.output)
    print('SHA256:', hashlib.sha256(args.output.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
