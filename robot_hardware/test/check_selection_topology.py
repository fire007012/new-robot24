#!/usr/bin/env python3
"""Read installed Xacro and selected DCF; never load plugins or open CAN."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from robot_hardware_runtime.selection import select_description
from robot_hardware_runtime.control_period import control_rate

share=Path(get_package_share_directory('robot_hardware'))
canopen=Path(get_package_share_directory('robot_canopen_config'))/'config'
policy=Path(get_package_share_directory('robot_safety'))/'config/group_policy_v2.yaml'
contract=yaml.safe_load(policy.read_text())
evidence=[]
for mock in ('false','true'):
    raw=subprocess.check_output(['xacro',str(share/'urdf/robot.urdf.xacro'),'mock:='+mock,
        'canopen_interface:=can0','private_interface:=can0','group_policy:='+str(policy)],text=True)
    for groups in ({'base'},{'flippers'},{'arm','gripper'},{'base','flippers'},{'arm','base','flippers','gripper'}):
        result=select_description(raw,contract,groups,canopen)
        root=ET.fromstring(result)
        expected={a for a,v in contract['axes'].items() if v['group'] in groups}
        assert {j.attrib['name'] for j in root.findall('ros2_control/joint')}==expected
        backends=[]
        for component in root.findall('ros2_control'):
            plugin=component.findtext('hardware/plugin')
            if mock=='true':assert plugin=='robot_hardware/MockSystem'
            else:assert plugin in ('canopen_ros2_control/RobotSystem','can_driver/CanDriverSystem')
            backends.append(component.attrib['name'])
            for param in component.findall('.//param'):
                if param.attrib['name'] in ('can_device','can_interface_name'):assert param.text=='can0'
            if component.attrib['name']=='RobotCanopenSystem':
                path=Path(component.find("hardware/param[@name='bus_config']").text)
                nodes=set(yaml.safe_load(path.read_text()))-{'master','options'}
                assert nodes=={j.attrib['name'] for j in component.findall('joint')}
                assert control_rate(path.parent)==control_rate(canopen)
        evidence.append(dict(mock=mock,groups=sorted(groups),axes=sorted(expected),backends=backends,
            interfaces='can0',note='description only; shared real topology remains rejected'))
print(json.dumps(evidence,indent=2))
