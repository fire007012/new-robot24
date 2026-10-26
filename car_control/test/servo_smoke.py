#!/usr/bin/env python3
"""Verify controller ownership, fresh Servo input, and stop on packet loss."""
import importlib.util,os,time,unittest
from pathlib import Path
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import SetBool
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('full_system_smoke',root/'robot_bringup/test/full_system_smoke.py');core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
class ServoSmoke(unittest.TestCase):
    PACKAGE='robot_bringup';LAUNCH='full_system_mock.launch.py';DOMAIN='83'
    ARGUMENTS=['enable_servo:=true','enable_moveit:=false','journal:=/tmp/robot24-servo-test.json']
    setUpClass=classmethod(core.FullSystemSmoke.setUpClass.__func__)
    tearDownClass=classmethod(core.FullSystemSmoke.tearDownClass.__func__)
    joint_state=classmethod(core.FullSystemSmoke.joint_state.__func__)
    runtime_state=classmethod(core.FullSystemSmoke.runtime_state.__func__)
    wait=core.FullSystemSmoke.wait;result=core.FullSystemSmoke.result;call=core.FullSystemSmoke.call;trigger=core.FullSystemSmoke.trigger;goal=core.FullSystemSmoke.goal;prepare=core.FullSystemSmoke.prepare;conditions=core.FullSystemSmoke.conditions
    def controllers(self):
        return {c.name:c.state for c in self.call('/controller_manager/list_controllers',ListControllers,ListControllers.Request()).controller}
    def test_servo_ownership_and_timeout(self):
        self.wait(lambda:len(self.controllers())==7 and all(value=='inactive' for value in self.controllers().values()),60)
        self.assertTrue(all(value=='inactive' for value in self.controllers().values()))
        self.prepare();self.trigger('init');self.conditions();self.trigger('enable');self.trigger('resume')
        self.wait(lambda:self.runtime=='Running');self.wait(lambda:len(self.positions)>=13)
        self.assertEqual(self.controllers()['arm_position_controller'],'active')
        client,handle=self.goal(.3,3);self.assertTrue(handle.accepted)
        self.assertEqual(self.result(handle.get_result_async()).result.error_code,0)
        time.sleep(.3)
        response=self.call('/arm_control/set_servo_mode',SetBool,SetBool.Request(data=True));self.assertTrue(response.success,response.message)
        self.wait(lambda:self.runtime_details.get('arm_profile')=='servo' and self.runtime_details['groups']['arm']=='Running')
        state=self.controllers();self.assertEqual(state['arm_position_controller'],'inactive');self.assertEqual(state['arm_servo_controller'],'active')
        self.output=None
        self.node.create_subscription(Float64MultiArray,'/arm_servo_controller/command',lambda m:setattr(self,'output',list(m.data)),1)
        self.wait(lambda:self.output is not None,15)
        time.sleep(.5);before=dict(self.positions)
        for name in ('shoulder_yaw_joint','wrist_pitch_joint'):self.assertAlmostEqual(self.positions[name],before[name],delta=.002)
        publisher=self.node.create_publisher(TwistStamped,'/servo_server/delta_twist_cmds',1)
        self.wait(lambda:publisher.get_subscription_count()>0,20)
        command=TwistStamped();command.header.frame_id='grasp_tcp';command.twist.angular.z=.1
        def nonzero():
            command.header.stamp=self.node.get_clock().now().to_msg();publisher.publish(command)
            return self.output and any(abs(v)>1e-5 for v in self.output)
        self.wait(nonzero,20)
        self.wait(lambda:self.output is not None and all(v==0. for v in self.output),2)
        response=self.call('/arm_control/set_servo_mode',SetBool,SetBool.Request(data=False));self.assertTrue(response.success,response.message)
        state=self.controllers();self.assertEqual(state['arm_position_controller'],'active');self.assertEqual(state['arm_servo_controller'],'inactive')
        client,handle=self.goal(.02);self.assertTrue(handle.accepted)
        self.assertEqual(self.result(handle.get_result_async()).result.error_code,0)
        self.trigger('halt');self.trigger('shutdown')
if __name__=='__main__':
    os.environ.setdefault('ROBOT_OFFLINE_REPORT','/tmp/robot24-servo.log');unittest.main(verbosity=2)
