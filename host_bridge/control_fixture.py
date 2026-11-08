#!/usr/bin/env python3
"""Print real v2 request/response NDJSON using an explicitly simulated runtime."""
import json
import tempfile
import time
from output_adapters import DryRunOutput
from bridge_core import BridgeCore
from control_v2 import ControlV2, CAPABILITIES


class Clock:
    def __init__(self): self.value=10.0
    def __call__(self): return self.value
    def step(self, seconds): self.value+=seconds


class Capture(DryRunOutput):
    def __init__(self, clock):
        super().__init__()
        self.runtime_received=clock()
        groups=('base','arm','flippers','gripper')
        self.runtime_state=dict(groups={g:'Running' for g in groups},diagnostics=dict(
            readable=True,global_inhibit=False,owner_fresh=True,
            groups={g:dict(state=2,ready=True,blockers=[]) for g in groups}))
    def call_lifecycle(self, command): return True,0,'fixture ok'
    def publish_twist(self, value): pass
    def publish_servo(self, value): pass
    def publish_gripper(self, value): pass
    def publish_flipper(self, value): pass


def main():
    clock=Clock()
    output=Capture(clock)
    core=BridgeCore(output,500,.8,1.5)
    core.robot_id='robot24'
    with tempfile.TemporaryDirectory() as folder:
        c=ControlV2(core,clock,folder)
        seq=0
        session={}
        def exchange(kind, **fields):
            nonlocal seq
            seq+=1
            request=dict(type=kind,protocol_version=2,session_id=session.get('session_id'),epoch=session.get('epoch'),
                         freshness_token=session.get('freshness_token'),seq=seq,ttl_ms=300,**fields)
            print(json.dumps(dict(direction='request',frame=request),allow_nan=False))
            response=c.handle('fixture',request)
            print(json.dumps(dict(direction='response',frame=response),allow_nan=False))
            return response
        try:
            session.update(exchange('control_hello',robot_id='robot24',role='control',capabilities=CAPABILITIES)['result'])
            exchange('operator_input',keyboard={'pressed_keys':['w']})
            exchange('operator_input',keyboard={'pressed_keys':[]})
            exchange('authorize_operations')
            exchange('authorize')
            exchange('operator_input',keyboard={'pressed_keys':['w']});c.tick()
            clock.step(.301)
            exchange('operator_input',keyboard={'pressed_keys':['w']})
            session.update(exchange('freshness_request')['result'])
            output.call_lifecycle=lambda command:(False,2301,'ROS2 request timeout; outcome uncertain')
            op=exchange('system_command',request_id='timeout-example',command='init')['result']
            deadline=time.monotonic()+2
            while c.operations[op['operation_id']]['state'] in ('accepted','running') and time.monotonic()<deadline:
                time.sleep(.005)
            exchange('operation_query',operation_id=op['operation_id'])
            exchange('system_command',request_id='timeout-example',command='enable')
            c.runtime_changed()
            exchange('operator_input',keyboard={'pressed_keys':['w']})
            session['epoch']=exchange('control_status')['result']['epoch']
            session.update(exchange('freshness_request')['result'])
            exchange('operator_input',keyboard={'pressed_keys':[]})
            exchange('authorize')
            exchange('emergency_stop')
        finally:
            c.close()

if __name__=='__main__':main()
