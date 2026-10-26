#!/usr/bin/env python3
"""Actual Humble manager regression for group sessions; opens only simulated hardware."""
import json
import os
import re
import signal
import time
from pathlib import Path
import unittest
from ament_index_python.packages import get_package_share_directory
from hybrid_smoke import HybridSmoke
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist
from robot_hardware.srv import GroupOperation, SetProfile, Maintain
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from canopen_interfaces.srv import CORead
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType

class GroupSmoke(HybridSmoke):
    DOMAIN='92'
    ARGUMENTS=['mock:=true','group_policy:='+str(Path(get_package_share_directory('robot_safety'))/'config/group_policy_v1.yaml')]
    def status(self):
        response=self.call('/robot_runtime/status',Trigger,Trigger.Request())
        self.assertTrue(response.success,response.message)
        return json.loads(response.message)
    def operate(self,operation,groups,success=True,resource=''):
        response=self.call('/robot_runtime/group_operation',GroupOperation,GroupOperation.Request(operation=operation,groups=groups,resource=resource))
        self.assertEqual(response.success,success,response.message)
        return response
    def inject(self,path,value):
        response=self.call(path,SetBool,SetBool.Request(data=value));self.assertTrue(response.success)
    def running(self,group):
        return self.status()['groups'][group]['state']==2
    def active(self):
        response=self.call('/controller_manager/list_controllers',ListControllers,ListControllers.Request())
        return {c.name for c in response.controller if c.state=='active'}
    def test_whole_chain(self):
        def loaded():
            controllers=self.call('/controller_manager/list_controllers',ListControllers,ListControllers.Request()).controller
            return len(controllers)==6 and all(c.state=='inactive' for c in controllers)
        self.wait(loaded,60)
        self.wait(lambda:getattr(self,'runtime_details',{}).get('diagnostics',{}).get('readable',False))
        self.operate('init',['base'])
        self.operate('enable',['base'],False)  # Unknown posture/support must be visible and block motion.
        for condition in ('arm_secured','support_trusted','payload_retained'):
            self.inject('/mock/conditions/'+condition,True)
        self.operate('recover',['base']);self.operate('resume',['base'])
        self.assertEqual(self.active(),{'joint_state_broadcaster','wheel_controller'})
        components=self.call('/controller_manager/list_hardware_components',ListHardwareComponents,ListHardwareComponents.Request())
        self.assertEqual(next(c.state.id for c in components.component if c.name=='RobotCanopenSystem'),1)
        pub=self.node.create_publisher(Twist,'/wheel_controller/cmd_vel_unstamped',1)
        self.wait(lambda:pub.get_subscription_count()>0)
        command=Twist();command.linear.x=.02
        timer=self.node.create_timer(.05,lambda:pub.publish(command))
        self.wait(lambda:abs(self.velocities.get('left_front_wheel_joint',0))>.01)
        timer.cancel();self.operate('halt',['base'])
        self.wait(lambda:abs(self.velocities.get('right_front_wheel_joint',1))<1e-6)
        time.sleep(.04);stopped=self.positions['right_front_wheel_joint']
        self.operate('resume',['base'])
        for _ in range(7):
            time.sleep(.02);self.assertAlmostEqual(self.positions['right_front_wheel_joint'],stopped,delta=1e-5)
        timer.reset();self.wait(lambda:abs(self.velocities.get('left_front_wheel_joint',0))>.01)
        base_epoch=self.status()['groups']['base']['epoch']
        self.operate('init' ,['arm','flippers','gripper']);self.operate('enable',['arm','flippers','gripper'])
        self.operate('resume',['arm','flippers','gripper'])
        jog=self.node.create_publisher(JointJog,'/flipper_control/jog_cmd',1)
        self.wait(lambda:jog.get_subscription_count()>0)
        jog.publish(JointJog(joint_names=['left_front_arm_joint'],velocities=[.1]))
        self.wait(lambda:abs(self.velocities.get('left_front_arm_joint',0))>.03)
        self.operate('halt',['flippers']);self.operate('resume',['flippers'])
        time.sleep(.08)
        self.assertAlmostEqual(self.velocities.get('left_front_arm_joint',1),0,delta=1e-6)
        grip=self.node.create_publisher(Float64MultiArray,'/gripper_controller/command',1)
        self.wait(lambda:grip.get_subscription_count()>0)
        grip.publish(Float64MultiArray(data=[.01]))
        self.wait(lambda:abs(self.velocities.get('left_gripper_finger_joint',0))>.003)
        self.wait(lambda:abs(self.velocities.get('left_gripper_finger_joint',1))<1e-6,2)
        grip.publish(Float64MultiArray(data=[float('nan')]))
        self.assertEqual(self.status()['groups']['base']['epoch'],base_epoch)
        client,handle=self.goal(.1,8)
        response=self.call('/robot_runtime/set_flipper_profile',SetProfile,SetProfile.Request(profile='csp'))
        self.assertTrue(response.success,response.message)
        self.assertEqual(self.status()['groups']['base']['epoch'],base_epoch)
        self.assertTrue(self.running('arm'))
        self.inject('/mock/shoulder_yaw_joint/fault',True)
        self.wait(lambda:not self.running('arm'))
        self.wait(lambda:self.runtime_details['groups']['arm']=='Fault')
        result=self.result(handle.get_result_async());self.assertNotEqual(result.status,4);client.destroy()
        self.assertTrue(self.running('base'));self.assertTrue(self.running('gripper'))
        retained=self.call('/controller_manager/list_hardware_components',ListHardwareComponents,ListHardwareComponents.Request())
        self.assertTrue(all(c.state.id==3 for c in retained.component))
        self.assertEqual(sum(len(c.command_interfaces) for c in retained.component),17)
        self.assertTrue(all(i.is_available for c in retained.component for i in c.command_interfaces))
        response=self.call('/shoulder_yaw_joint/sdo_read',CORead,CORead.Request(index=0x100A,subindex=0))
        self.assertFalse(response.success,'offline read must not return cached firmware')
        self.inject('/mock/shoulder_yaw_joint/fault',False)
        # Timeout does not cancel the remote preparation. It must be fenced until completion.
        self.inject('/mock/RobotCanopenSystem/arm/prepare_delay',True)
        self.call('/robot_runtime/set_parameters',SetParameters,SetParameters.Request(parameters=[
            Parameter(name='prepare_timeout',value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=.25))]))
        self.operate('recover',['arm'],False)
        self.operate('recover',['arm'],False)
        self.assertTrue(self.running('base'))
        self.inject('/mock/RobotCanopenSystem/arm/prepare_delay',False)
        time.sleep(2.1)
        self.call('/robot_runtime/set_parameters',SetParameters,SetParameters.Request(parameters=[
            Parameter(name='prepare_timeout',value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=30.0))]))
        self.operate('recover',['arm'])
        self.assertEqual(self.status()['groups']['arm']['state'],1)
        self.assertEqual(self.status()['groups']['base']['epoch'],base_epoch)
        self.assertNotIn('arm_position_controller',self.active())
        self.operate('resume',['arm']);client,handle=self.goal(.05,3)
        # One track fault must stop the track pair, while the healthy arm trajectory completes.
        self.inject('/mock/left_front_wheel_joint/fault',True)
        self.wait(lambda:not self.running('base'))
        self.wait(lambda:self.runtime_details['groups']['base']=='Fault')
        self.wait(lambda:abs(self.velocities.get('right_front_wheel_joint',1))<1e-6)
        result=self.result(handle.get_result_async());self.assertEqual(result.result.error_code,0);client.destroy()
        self.assertTrue(self.running('arm'))
        timer.cancel();self.inject('/mock/left_front_wheel_joint/fault',False)
        self.operate('recover',['base']);self.assertEqual(self.status()['groups']['base']['state'],1)
        self.operate('resume',['base']);time.sleep(.4)
        self.assertAlmostEqual(self.velocities.get('right_front_wheel_joint',1),0,delta=1e-6)
        timer.reset();self.wait(lambda:abs(self.velocities.get('left_front_wheel_joint',0))>.01)
        base_epoch=self.status()['groups']['base']['epoch']
        self.inject('/mock/RobotCanopenSystem/fault',True)
        self.wait(lambda:not self.running('arm') and not self.running('flippers'))
        self.wait(lambda:all(self.runtime_details['groups'][g]=='Fault' for g in ('arm','flippers')))
        self.assertTrue(self.running('base'));self.assertTrue(self.running('gripper'))
        self.inject('/mock/RobotCanopenSystem/fault',False)
        self.operate('recover_resource',[],resource='RobotCanopenSystem')
        self.assertEqual(self.status()['groups']['base']['epoch'],base_epoch)
        # A target-axis maintenance operation must not deactivate the private component or tracks.
        response=self.call('/robot_runtime/maintain',Maintain,Maintain.Request(joint='shoulder_yaw_joint',operation='set_offset',lower=.01))
        self.assertTrue(response.success,response.message);self.assertTrue(self.running('base'))
        self.inject('/mock/shoulder_yaw_joint/save_failure',True)
        response=self.call('/robot_runtime/maintain',Maintain,Maintain.Request(joint='shoulder_yaw_joint',operation='set_offset',lower=.02,save=True))
        self.assertFalse(response.success)
        self.assertTrue(any(e['category']=='maintenance_uncertain' for e in self.status()['events']))
        self.operate('recover',['arm'],False)
        self.assertTrue(self.running('base'));self.assertEqual(self.status()['groups']['base']['epoch'],base_epoch)
        self.inject('/mock/shoulder_yaw_joint/save_failure',False)
        response=self.call('/robot_runtime/maintain',Maintain,Maintain.Request(joint='shoulder_yaw_joint',operation='restore'))
        self.assertTrue(response.success,response.message)
        self.operate('enable',['arm']);self.assertEqual(self.status()['groups']['arm']['state'],1)
        runtime_pid=int(re.search(r'\[robot_runtime-\d+\]: process started with pid \[(\d+)\]',self.log.read_text()).group(1))
        os.kill(runtime_pid,signal.SIGSTOP)
        try:
            self.wait(lambda:self.status()['global_inhibit'],3)
            self.assertIn('heartbeat',self.status()['global_reason'])
            self.wait(lambda:abs(self.velocities.get('right_front_wheel_joint',1))<1e-6)
        finally:
            os.kill(runtime_pid,signal.SIGCONT)
        self.wait(lambda:self.status()['owner_fresh'])
        self.assertTrue(self.status()['global_inhibit'],'renewing the owner lease must not reopen old motion')
        self.wait(lambda:self.runtime_details['groups']['base']=='Fault')
        response=self.call('/robot_runtime/reset_global_stop',Trigger,Trigger.Request());self.assertTrue(response.success)
        self.operate('recover',['base']);self.operate('resume',['base'])
        response=self.call('/robot_runtime/global_stop' ,Trigger,Trigger.Request());self.assertTrue(response.success)
        self.wait(lambda:all(g['state']!=2 for g in self.status()['groups'].values()))
        self.operate('recover',['base'],False)
        self.assertTrue(self.status()['global_inhibit'])
        self.wait(lambda:self.runtime_details['groups']['base']=='Fault')
        response=self.call('/robot_runtime/reset_global_stop',Trigger,Trigger.Request());self.assertTrue(response.success)
        self.operate('recover',['base']);self.assertEqual(self.status()['groups']['base']['state'],1)
        timer.cancel();self.operate('shutdown',list(('arm','base','flippers','gripper')))

if __name__=='__main__':
    unittest.main(defaultTest='GroupSmoke',verbosity=2)
