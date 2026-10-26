#!/usr/bin/env python3
"""Launch the real Humble controller manager with two simulated hardware plugins.
No CAN interface is opened. Fails on incomplete lifecycle, shared faults, trajectory or timeout behavior.
"""
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SetHardwareComponentState
from robot_hardware.srv import SetProfile, Maintain

class HybridSmoke(unittest.TestCase):
    LAUNCH='hardware.launch.py'
    ARGUMENTS=['mock:=true']
    DOMAIN='89'
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(prefix='robot-hybrid-smoke-')
        cls.log=Path(cls.tmp.name)/'launch.log'
        cls.stream=cls.log.open('w')
        env=dict(os.environ,ROS_DOMAIN_ID=cls.DOMAIN,ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=cls.tmp.name)
        os.environ.update(ROS_DOMAIN_ID=cls.DOMAIN,ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=cls.tmp.name)
        arguments=['ros2','launch',getattr(cls,'PACKAGE','robot_hardware'),cls.LAUNCH]+cls.ARGUMENTS
        if cls.LAUNCH=='hardware.launch.py':arguments+=['journal:='+cls.tmp.name+'/maintenance.json']
        cls.process=subprocess.Popen(arguments,stdout=cls.stream,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        try: rclpy.init()
        except Exception:
            os.kill(cls.process.pid,signal.SIGINT);cls.process.wait(timeout=10);cls.stream.close();cls.tmp.cleanup();raise
        cls.node=Node('hybrid_smoke');cls.executor=MultiThreadedExecutor(4);cls.executor.add_node(cls.node)
        cls.clients={}
        cls.thread=threading.Thread(target=cls.executor.spin);cls.thread.start()
        cls.positions={};cls.velocities={};cls.runtime=''
        cls.sub=cls.node.create_subscription(JointState,'/joint_states',cls.joint_state,10)
        cls.state_sub=cls.node.create_subscription(String,'/robot_runtime/state',cls.runtime_state,10)
    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown();cls.thread.join();cls.node.destroy_node();rclpy.shutdown()
        # Launch forwards this signal to its children. Signalling the whole group
        # also delivers a second SIGINT during their destructor/handler teardown.
        os.kill(cls.process.pid,signal.SIGINT)
        try:cls.process.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(cls.process.pid,signal.SIGKILL);cls.process.wait()
        cls.stream.close()
        # Keep the evidence even on failure; do not discard logs with TemporaryDirectory.cleanup().
        target=Path(os.environ.get('ROBOT_OFFLINE_REPORT','/tmp/robot-hybrid-smoke.log'))
        output=cls.log.read_text()
        # A single unittest invocation runs HybridSmoke and SingleAxisSmoke.
        # Retain each launch independently instead of losing the first class's
        # evidence when the next class writes the shared report path.
        target.with_name(target.stem+'-'+cls.__name__+target.suffix).write_text(output)
        target.write_text(output);cls.tmp.cleanup()
        if re.search(r'\[ERROR\] \[ros2_control_node-\d+\]: process has died',output):
            raise AssertionError('control manager failed during teardown:\n'+output[-6000:])
    @classmethod
    def joint_state(cls,message):
        cls.positions.update(zip(message.name,message.position));cls.velocities.update(zip(message.name,message.velocity))
    @classmethod
    def runtime_state(cls,message):
        cls.runtime_details=json.loads(message.data);cls.runtime=cls.runtime_details['state']
    def wait(self,predicate,timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.process.poll() is not None:self.fail('launch exited:\n'+self.log.read_text()[-8000:])
            if 'process has died' in self.log.read_text():self.fail(self.log.read_text()[-10000:])
            if predicate():return
            time.sleep(.02)
        self.fail('condition timeout; runtime='+self.runtime+'\n'+json.dumps(getattr(self,'runtime_details',{}),ensure_ascii=False)+'\n'+self.log.read_text()[-6000:])
    def result(self,future,timeout=30):
        event=threading.Event();future.add_done_callback(lambda _:event.set())
        self.assertTrue(event.wait(timeout),'future timeout')
        return future.result()
    def call(self,name,kind,request):
        # Reuse DDS endpoints as the runtime adapter does; repeated destruction
        # during backend reconstruction can race service discovery in Humble.
        client=self.clients.get(name)
        if client is None:
            client=self.node.create_client(kind,name);self.clients[name]=client
        self.assertTrue(client.wait_for_service(timeout_sec=30),name+' unavailable\n'+self.log.read_text()[-4000:])
        return self.result(client.call_async(request))
    def trigger(self,operation):
        result=self.call('/robot_runtime/'+operation,Trigger,Trigger.Request())
        self.assertTrue(result.success,operation+': '+result.message)
    def goal(self,value,seconds=1):
        names=['shoulder_yaw_joint','shoulder_pitch_joint','elbow_pitch_joint','wrist_pitch_joint','wrist_yaw_joint','wrist_roll_joint']
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=names
        point=JointTrajectoryPoint(positions=[value]*6,velocities=[0.0]*6)
        point.time_from_start.sec=seconds;goal.trajectory.points=[point]
        client=ActionClient(self.node,FollowJointTrajectory,'/arm_position_controller/follow_joint_trajectory')
        self.assertTrue(client.wait_for_server(timeout_sec=10))
        handle=self.result(client.send_goal_async(goal));self.assertTrue(handle.accepted)
        return client,handle
    def test_whole_chain(self):
        # All controllers are loaded inactive; motion requires explicit init -> enable -> resume.
        def loaded():
            try:
                response=self.call('/controller_manager/list_controllers',ListControllers,ListControllers.Request())
                return len(response.controller)==6 and all(c.state=='inactive' for c in response.controller)
            except Exception:return False
        self.wait(loaded,60)
        self.trigger('init');self.trigger('enable');self.wait(lambda:self.runtime=='Armed')
        self.trigger('resume');self.wait(lambda:self.runtime=='Running')
        self.wait(lambda:'shoulder_yaw_joint' in self.positions)
        client,handle=self.goal(.05)
        result=self.result(handle.get_result_async())
        self.assertEqual(result.result.error_code,0)
        for name in ['shoulder_yaw_joint','shoulder_pitch_joint','elbow_pitch_joint','wrist_pitch_joint','wrist_yaw_joint','wrist_roll_joint']:
            self.assertAlmostEqual(self.positions[name],.05,delta=.01)
        client.destroy()
        client,handle=self.goal(.2,5)
        time.sleep(.2)
        self.assertTrue(self.result(handle.cancel_goal_async()).goals_canceling)
        self.result(handle.get_result_async());client.destroy()
        publisher=self.node.create_publisher(Float64MultiArray,'/flipper_csv_forward_controller/command',1)
        self.wait(lambda:publisher.get_subscription_count()>0)
        # Sustain the command until feedback confirms reception, then stop the
        # publisher to measure the actual controller watchdog behavior.
        def moving():
            if abs(self.velocities.get('left_front_arm_joint',0))>.05:return True
            publisher.publish(Float64MultiArray(data=[.1]*4))
            return False
        self.wait(moving)
        self.wait(lambda:abs(self.velocities.get('left_front_arm_joint',1))<1e-6,2)
        response=self.call('/robot_runtime/set_flipper_profile',SetProfile,SetProfile.Request(profile='csp'))
        self.assertTrue(response.success,response.message);self.wait(lambda:self.runtime=='Running')
        for backend in ['CanDriverSystem','RobotCanopenSystem']:
            response=self.call('/mock/'+backend+'/fault',SetBool,SetBool.Request(data=True))
            self.assertTrue(response.success);self.wait(lambda:self.runtime=='Fault',10)
            stopped=dict(self.positions);time.sleep(.2)
            for name,q in stopped.items():self.assertAlmostEqual(self.positions[name],q,delta=1e-5)
            self.call('/mock/'+backend+'/fault',SetBool,SetBool.Request(data=False))
            self.trigger('recover');self.wait(lambda:self.runtime=='Armed')
            time.sleep(.1);self.assertEqual(self.runtime,'Armed')
            self.trigger('resume');self.wait(lambda:self.runtime=='Running')
        self.trigger('halt')
        response=self.call('/robot_runtime/maintain',Maintain,Maintain.Request(joint='shoulder_yaw_joint',operation='set_offset',lower=.01))
        self.assertTrue(response.success,response.message)
        self.trigger('shutdown')

class SingleAxisSmoke(unittest.TestCase):
    LAUNCH='single_axis.launch.py'
    ARGUMENTS=['joint:=shoulder_yaw_joint','can_interface:=no_can_test0']
    DOMAIN='91'
    setUpClass=classmethod(HybridSmoke.setUpClass.__func__)
    tearDownClass=classmethod(HybridSmoke.tearDownClass.__func__)
    joint_state=classmethod(HybridSmoke.joint_state.__func__)
    runtime_state=classmethod(HybridSmoke.runtime_state.__func__)
    result=HybridSmoke.result
    call=HybridSmoke.call
    wait=HybridSmoke.wait
    def test_actual_single_axis_plugin_loads_without_opening_can(self):
        response=self.call('/controller_manager/list_hardware_components',ListHardwareComponents,ListHardwareComponents.Request())
        self.assertEqual(len(response.component),1)
        self.assertEqual(response.component[0].name,'RobotCanopenSystem')
        self.assertEqual(response.component[0].state.id,1)
        self.assertEqual([i.name for i in response.component[0].command_interfaces],['shoulder_yaw_joint/position'])

    def test_missing_interface_configure_fails_cleanly_on_retry(self):
        from lifecycle_msgs.msg import State
        for _ in range(2):
            request=SetHardwareComponentState.Request(name='RobotCanopenSystem',target_state=State(id=2,label='inactive'))
            response=self.call('/controller_manager/set_hardware_component_state',SetHardwareComponentState,request)
            self.assertFalse(response.ok)
            self.assertIsNone(self.process.poll())

if __name__=='__main__':unittest.main(verbosity=2)
