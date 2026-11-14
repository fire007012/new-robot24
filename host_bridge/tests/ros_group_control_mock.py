#!/usr/bin/env python3
"""TCP v2 -> actual ROS2 coordinator/controllers -> mock hardware. No physical CAN.

Run after sourcing a core build. Uses ROS domain 98 and loopback port 19908.
Exercises default bridge parameters, speed change, an emergency during delayed
preparation, and explicit release without automatic motion recovery.
"""
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import unittest
import uuid
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'host_bridge'))
from control_v2 import SUPPORTED_CAPABILITIES
spec=importlib.util.spec_from_file_location('full_system_smoke',ROOT/'robot_bringup/test/full_system_smoke.py')
core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)


class HostIntegration(unittest.TestCase):
    PACKAGE='robot_bringup';LAUNCH='full_system_mock.launch.py';DOMAIN='98'
    ARGUMENTS=['enable_servo:=true','enable_moveit:=false','journal:=/tmp/robot24-host-regression-maintenance.json']
    setUpClass=classmethod(core.FullSystemSmoke.setUpClass.__func__)
    tearDownClass=classmethod(core.FullSystemSmoke.tearDownClass.__func__)
    joint_state=classmethod(core.FullSystemSmoke.joint_state.__func__)
    runtime_state=classmethod(core.FullSystemSmoke.runtime_state.__func__)
    wait=core.FullSystemSmoke.wait;result=core.FullSystemSmoke.result;call=core.FullSystemSmoke.call
    trigger=core.FullSystemSmoke.trigger;goal=core.FullSystemSmoke.goal
    prepare=core.FullSystemSmoke.prepare;conditions=core.FullSystemSmoke.conditions

    def test_group_control_and_emergency_during_prepare(self):
        def loaded():
            r=self.call('/controller_manager/list_controllers',ListControllers,ListControllers.Request())
            return len(r.controller)==7 and all(c.state=='inactive' for c in r.controller)
        self.wait(loaded,60);self.prepare()
        bridge_log=Path('/tmp/robot24-host-group-bridge.log')
        stream=bridge_log.open('w')
        bridge=subprocess.Popen([sys.executable,str(ROOT/'host_bridge/host_bridge_node.py'),
            '--ros','--no-debug-ui','--host','127.0.0.1','--port','19908',
            '--robot-id','robot24','--control-state-dir',self.tmp.name+'/bridge-state'],stdout=stream,stderr=subprocess.STDOUT)
        conn=None;reader=None
        try:
            deadline=time.monotonic()+10
            while conn is None:
                self.assertIsNone(bridge.poll(),bridge_log.read_text())
                try:conn=socket.create_connection(('127.0.0.1',19908),timeout=1)
                except OSError:self.assertLess(time.monotonic(),deadline);time.sleep(.05)
            conn.settimeout(3);conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            reader=conn.makefile('rb');session={};seq=0
            def request(kind, retries=5, **fields):
                nonlocal seq
                seq+=1;current_seq=seq
                message=dict(type=kind,protocol_version=2,seq=current_seq,session_id=session.get('session_id'),
                    epoch=session.get('epoch'),freshness_token=session.get('freshness_token'),ttl_ms=300,**fields)
                conn.sendall((json.dumps(message)+'\n').encode())
                while True:
                    response=json.loads(reader.readline())
                    if response.get('type')!='control_result' or response.get('seq')!=current_seq:continue
                    # Only retry explicit rejection, never accepted/unknown operations or non-neutral motion.
                    if (not response['ok'] and response.get('error',{}).get('code')=='EPOCH' and retries and
                            (kind in ('freshness_request','operation_query','authorize_operations','authorize') or
                             kind=='operator_input' and fields.get('keyboard')=={'pressed_keys':[]})):
                        session['epoch']=request('control_status')['epoch']
                        if kind!='freshness_request':session.update(request('freshness_request'))
                        return request(kind,retries=retries-1,**fields)
                    self.assertTrue(response['ok'],response);return response['result']
            time.sleep(1)  # Allow the bridge's ROS services/subscriptions to discover the cold stack.
            session.update(request('control_hello',robot_id='robot24',role='control',
                capabilities=SUPPORTED_CAPABILITIES,presets=['base','flippers','arm']))
            self.assertEqual(request('control_status')['speed_level'],1)
            def refresh():
                session['epoch']=request('control_status')['epoch'];session.update(request('freshness_request'))
            def authorize(motion=False):
                refresh();request('operator_input',keyboard={'pressed_keys':[]});request('authorize_operations')
                if motion:request('authorize')
            def wait_operation(op, expected='succeeded'):
                deadline=time.monotonic()+45
                while op['state'] in ('accepted','running'):
                    self.assertLess(time.monotonic(),deadline);time.sleep(.05);refresh()
                    op=request('operation_query',operation_id=op['operation_id'])
                self.assertEqual(op['state'],expected,op)
                time.sleep(.3)  # Let runtime epoch publication catch up before a subsequent authorization.
                return op
            def operation(command,params=None):
                authorize()
                op=request('system_command',command=command,params=params or {},request_id=str(uuid.uuid4()))
                return wait_operation(op)
            operation('init');self.conditions();operation('enable');operation('resume')
            self.wait(lambda:self.runtime=='Running')
            refresh();client,handle=self.goal(.3,3);future=handle.get_result_async()
            while not future.done():refresh();time.sleep(.1)
            self.assertEqual(future.result().result.error_code,0)
            operation('set_speed_level',{'level':2})
            status=request('control_status');self.assertEqual(status['speed_level'],2);self.assertFalse(status['authorized'])
            authorize(True)
            def moving():
                refresh();request('operator_input',keyboard={'pressed_keys':['w','y']})
                return self.velocities.get('left_front_wheel_joint',0.)>.1 and self.velocities.get('left_front_arm_joint',0.)>.01
            self.wait(moving,10)
            print('TCP init/enable/resume, explicit speed and base/flipper mock motion passed',flush=True)
            operation('set_control_mode',{'mode':'arm'})
            self.wait(lambda:self.runtime_details.get('arm_profile')=='servo');time.sleep(.5)
            self.servo_output=[];self.gripper_output=[]
            self.node.create_subscription(Float64MultiArray,'/arm_servo_controller/command',lambda m:setattr(self,'servo_output',list(m.data)),1)
            self.node.create_subscription(Float64MultiArray,'/gripper_controller/command',lambda m:setattr(self,'gripper_output',list(m.data)),1)
            authorize(True)
            def arm_moving():
                refresh();request('operator_input',keyboard={'pressed_keys':['l','f']})
                return any(abs(v)>1e-5 for v in self.servo_output) and self.gripper_output and self.gripper_output[0]>.001
            self.wait(arm_moving,20)
            print('TCP Servo/gripper mock motion passed',flush=True)
            # Leave base Running while the arm preset is in a long backend preparation.
            operation('halt',{'presets':['arm']});operation('init',{'presets':['arm']})
            response=self.call('/mock/RobotCanopenSystem/arm/prepare_delay',SetBool,SetBool.Request(data=True))
            self.assertTrue(response.success);authorize()
            pending=request('system_command',command='enable',params={'presets':['arm']},request_id=str(uuid.uuid4()))
            time.sleep(.4);request('emergency_stop')
            begin=time.monotonic()
            def globally_inhibited():
                r=self.call('/robot_runtime/status',Trigger,Trigger.Request());d=json.loads(r.message)
                return d['global_inhibit'] and all(g['state'] not in (1,2) for g in d['groups'].values())
            self.wait(globally_inhibited,1)
            print('Emergency during delayed arm enable revoked all C++ permits in %.3f s'%(time.monotonic()-begin),flush=True)
            self.wait(lambda:request('control_status')['global_stop']['state']=='succeeded',3)
            wait_operation(pending,'failed')
            self.wait(lambda:not any(o.get('state')=='in_progress' for o in self.runtime_details.get('operations',{}).values()),5)
            refresh();request('operator_input',keyboard={'pressed_keys':[]});request('clear_emergency')
            self.assertTrue(request('control_status')['runtime_global_inhibit'])
            operation('reset_global_stop')
            refresh();status=request('control_status')
            self.assertFalse(status['authorized']);self.assertFalse(status['motion_permitted'])
            self.assertTrue(all(v!='Running' for v in self.runtime_details['groups'].values()))
            self.trigger('halt');self.trigger('shutdown')
            print('Clear/reset left motion stopped and unauthorized passed',flush=True)
        finally:
            if reader:reader.close()
            if conn:conn.close()
            if bridge.poll() is None:bridge.send_signal(signal.SIGINT)
            try:bridge.wait(timeout=15)
            except subprocess.TimeoutExpired:bridge.kill();bridge.wait()
            stream.close()


if __name__=='__main__':
    os.environ.setdefault('ROBOT_OFFLINE_REPORT','/tmp/robot24-host-group-launch.log')
    unittest.main(verbosity=2)
