#!/usr/bin/env python3
"""Installed ROS mock selection test. Never permits a hardware launch."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from std_srvs.srv import Trigger
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
rclpy.init();node=Node('selection_smoke')
policy=Path(get_package_share_directory('robot_safety'))/'config/group_policy_v2.yaml'
evidence=[]
latest={}
node.create_subscription(String,'/robot_runtime/state',lambda msg: latest.update(json.loads(msg.data)),1)

def call(path,kind,timeout=12):
    c=node.create_client(kind,path)
    try:
        if not c.wait_for_service(timeout_sec=timeout):raise RuntimeError('service missing: '+path)
        f=c.call_async(kind.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=timeout)
        if not f.done():raise RuntimeError('service timeout: '+path)
        return f.result()
    finally:node.destroy_client(c)

try:
    for selection,backends,axes,controllers in [
        ('base',{'CanDriverSystem'},2,{'joint_state_broadcaster','wheel_controller'}),
        ('flippers',{'RobotCanopenSystem'},4,{'joint_state_broadcaster','flipper_csv_forward_controller','flipper_csp_controller'}),
        ('arm,gripper',{'RobotCanopenSystem','CanDriverSystem'},7,{'joint_state_broadcaster','arm_position_controller','gripper_controller'}),
        ('arm,base,flippers,gripper',{'RobotCanopenSystem','CanDriverSystem'},13,{'joint_state_broadcaster','arm_position_controller','flipper_csv_forward_controller','flipper_csp_controller','gripper_controller','wheel_controller'})]:
        latest.clear()
        with (a.output/(selection.replace(',','_')+'.log')).open('w') as log:
            proc=subprocess.Popen(['ros2','launch','robot_hardware','hardware.launch.py','mock:=true',
                'selected_groups:='+selection,'group_policy:='+str(policy),'canopen_interface:=can0','private_interface:=can0',
                'journal:='+str(a.output/(selection+'.journal.json'))],stdout=log,stderr=log,start_new_session=True)
            try:
                deadline=time.monotonic()+65
                while True:
                    if proc.poll() is not None:raise RuntimeError('mock launch exited')
                    hs=call('/controller_manager/list_hardware_components',ListHardwareComponents).component
                    cs=call('/controller_manager/list_controllers',ListControllers).controller
                    if {c.name for c in cs if c.state=='inactive'}==controllers:break
                    if time.monotonic()>deadline:raise RuntimeError('controllers failed to load')
                    time.sleep(.1)
                assert {h.name for h in hs}==backends
                assert all(h.class_type=='robot_hardware/MockSystem' for h in hs)
                assert all(h.state.id==1 for h in hs)
                ready_until=time.monotonic()+30
                while latest.get('diagnostics',{}).get('readable') is not True:
                    rclpy.spin_once(node,timeout_sec=.1)
                    if time.monotonic()>ready_until:raise RuntimeError('runtime ownership not ready')
                # Explicit mock-only initialization proves unselected motors are unnecessary.
                result=call('/robot_runtime/init',Trigger,45)
                assert result.success,result.message
                hs=call('/controller_manager/list_hardware_components',ListHardwareComponents).component
                assert all(h.state.id in (2,3) for h in hs)
                assert sum(len(h.state_interfaces) for h in hs)>=axes*2
                halted=call('/robot_runtime/halt',Trigger)
                assert halted.success,halted.message
                evidence.append({'selection':selection,'backends':sorted(backends),
                    'controllers':sorted(controllers),'mock_init':True,'mock_halt':True,'real_motion':False})
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                time.sleep(1)
finally:
    node.destroy_node();rclpy.shutdown()
    (a.output/'evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
print(json.dumps(evidence,indent=2))
