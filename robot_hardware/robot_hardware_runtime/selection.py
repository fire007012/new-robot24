"""Build a bounded hardware description before any hardware plugin is loaded."""
from pathlib import Path
import xml.etree.ElementTree as ET

GROUPS = {'arm', 'base', 'flippers', 'gripper'}


def parse_groups(value):
    groups = value.split(',')
    if not groups or len(set(groups)) != len(groups) or not set(groups) <= GROUPS:
        raise ValueError('selected_groups must contain unique known groups')
    return set(groups)


def select_description(description, contract, groups, config_directory):
    if not groups or not set(groups) <= GROUPS:
        raise ValueError('invalid hardware selection')
    root = ET.fromstring(description)
    for component in list(root.findall('ros2_control')):
        for joint in list(component.findall('joint')):
            owner = contract['axes'][joint.attrib['name']]
            if owner['backend'] != component.attrib['name']:
                raise ValueError('hardware policy ownership mismatch')
            if owner['group'] not in groups:
                component.remove(joint)
        joints = component.findall('joint')
        if not joints:
            root.remove(component)
        elif component.attrib['name'] == 'RobotCanopenSystem':
            # These DCFs are generated at build time, including only selected nodes.
            selected = {contract['axes'][j.attrib['name']]['group'] for j in joints}
            suffix = 'groups_' + '_'.join(sorted(selected)) if selected != {'arm', 'flippers'} else ''
            directory = Path(config_directory) / suffix
            for param in component.findall('hardware/param'):
                if param.attrib['name'] in ('bus_config', 'master_config'):
                    target = directory / ('bus.yml' if param.attrib['name'] == 'bus_config' else 'master.dcf')
                    if not target.is_file():
                        raise ValueError('missing selected CANopen build asset: '+str(target))
                    param.text = str(target)
    return ET.tostring(root, encoding='unicode')


def selected_controllers(contract, groups, servo):
    result = {'joint_state_broadcaster'}
    for group in groups:
        result.update(contract['groups'][group]['controllers'])
    if servo and 'arm' in groups:
        result.add('arm_servo_controller')
    return sorted(result)
