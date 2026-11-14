#!/usr/bin/env python3
"""Actual ROS2 transport, mocked runtime/services, no CAN or physical devices.

Requires built robot_hardware and arm_control interfaces. Domain 96 is mandatory.
"""
import json
import os
from pathlib import Path
import socket
import sys
import threading
import tempfile
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from control_v2 import CAPABILITIES
from host_bridge_node import HostBridgeServer
from output_adapters import RosOutput


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='96':
        raise SystemExit('set ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=96 (isolated mock only)')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import Twist
    class Output(RosOutput):
        def lifecycle_service_name(self,command):return '/p0_mock/'+command
    output=Output('p0_bridge','/p0_mock/cmd_vel','/p0_mock/servo','/p0_mock/gripper','/p0_mock/flipper','/p0_mock/profile','/p0_mock')
    node=Node('p0_mock_backend')
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    group=ReentrantCallbackGroup()
    publisher=node.create_publisher(String,'/robot_runtime/state',1)
    observed=[];calls=[];entered=threading.Event();finished=threading.Event()
    def slow(request,response):
        calls.append(('init',time.monotonic()));entered.set();time.sleep(30.5)
        response.success=True;response.message='late success after 30.5s mock service';finished.set();return response
    def halt(request,response):
        calls.append(('halt',time.monotonic()));response.success=True;response.message='mock halted';return response
    node.create_service(Trigger,'/p0_mock/init',slow,callback_group=group)
    node.create_service(Trigger,'/p0_mock/halt',halt,callback_group=group)
    def global_stop(request,response):
        calls.append(('global_stop',time.monotonic()));response.success=True;response.message='mock C++ gate latched';return response
    node.create_service(Trigger,'/robot_runtime/global_stop',global_stop,callback_group=group)
    node.create_subscription(Twist,'/p0_mock/cmd_vel',lambda msg:observed.append((time.monotonic(),msg.linear.x)),100,callback_group=group)
    groups=('base','arm','flippers','gripper')
    state=dict(groups={g:'Running' for g in groups},input_epochs={g:1 for g in groups},diagnostics=dict(
        readable=True,owner_fresh=True,global_inhibit=False,
        groups={g:dict(state=2,epoch=1,ready=True,blockers=[]) for g in groups}))
    node.create_timer(.05,lambda:publisher.publish(String(data=json.dumps(state))),callback_group=group)
    spin=threading.Thread(target=executor.spin);spin.start()
    journal=tempfile.TemporaryDirectory(prefix='robot24-p0-ros-journal-')
    server=HostBridgeServer('127.0.0.1',19903,output,watchdog_ms=500,control_state_dir=journal.name)
    server.core.robot_id='robot24';output.runtime_reset_callback=server.control.runtime_changed
    serving=threading.Thread(target=server.serve_forever);serving.start()
    conn=None;stream=None
    try:
        deadline=time.monotonic()+15
        while not server.control.permission():
            assert time.monotonic()<deadline,'runtime mock not received';time.sleep(.05)
        conn=socket.create_connection(('127.0.0.1',19903),timeout=2);conn.settimeout(2)
        stream=conn.makefile('rb');stream.readline();stream.readline()
        session={};seq=0
        def call(kind,**fields):
            nonlocal seq
            seq+=1
            request=dict(type=kind,protocol_version=2,seq=seq,ttl_ms=300,session_id=session.get('session_id'),
                epoch=session.get('epoch'),freshness_token=session.get('freshness_token'),**fields)
            conn.sendall((json.dumps(request)+'\n').encode())
            while True:
                response=json.loads(stream.readline())
                if response.get('type')=='control_result' and response.get('seq')==seq:
                    assert response['ok'],response
                    return response['result']
        session.update(call('control_hello',robot_id='robot24',role='control',capabilities=CAPABILITIES))
        call('operator_input',keyboard={'pressed_keys':[]});call('authorize_operations');call('authorize')
        call('operator_input',keyboard={'pressed_keys':['w']})
        until=time.monotonic()+2
        while not any(v>0 for _,v in observed):
            assert time.monotonic()<until,'no ROS motion sample';time.sleep(.01)
        operation=call('system_command',request_id='ros-slow-init',command='init')
        assert entered.wait(2)
        begin=time.monotonic();heartbeats=0;max_roundtrip=0;estop_begin=None
        for i in range(310):
            started=time.monotonic();session.update(call('freshness_request'))
            if i%10==0:
                conn.sendall((json.dumps(dict(type='heartbeat',protocol_version=1,seq=90000+i))+'\n').encode())
                while True:
                    response=json.loads(stream.readline())
                    if response.get('type')=='heartbeat_ack' and response.get('seq')==90000+i:break
                heartbeats+=1
            if i==15:
                estop_begin=time.monotonic();call('emergency_stop')
                halt_until=time.monotonic()+1
                while not any(name=='global_stop' and at>=estop_begin for name,at in calls):
                    assert time.monotonic()<halt_until,'independent ROS global stop stalled during init';time.sleep(.01)
            max_roundtrip=max(max_roundtrip,time.monotonic()-started)
            time.sleep(.1)
        assert finished.wait(2)
        until=time.monotonic()+2
        while not server.control.operations[operation['operation_id']].get('late_backend_results'):
            assert time.monotonic()<until,'missing late ROS completion';time.sleep(.01)
        last_nonzero=max(t for t,v in observed if v>0)
        first_zero=min(t for t,v in observed if t>last_nonzero and v==0)
        assert first_zero-last_nonzero<.65,(first_zero,last_nonzero)
        assert max_roundtrip<.25,max_roundtrip
        op=call('operation_query',operation_id=operation['operation_id'])
        assert op['state']=='result_unknown',op
        assert op['late_backend_results'][0]['success'] is True,op
        report=dict(passed=True,kind='actual_ros2_mock_transport',physical_devices=False,
            elapsed=time.monotonic()-begin,heartbeat_acks=heartbeats,max_roundtrip_sec=max_roundtrip,
            last_motion_to_zero_sec=first_zero-last_nonzero,emergency_during_slow_service_at=estop_begin,operation=op,
            calls=calls,received_ros_outputs=observed)
        print(json.dumps(report),flush=True)
    finally:
        if stream:stream.close()
        if conn:conn.close()
        server.stop_event.set();serving.join(3)
        executor.shutdown();spin.join(3);node.destroy_node();output.close();journal.cleanup()

if __name__=='__main__':main()
