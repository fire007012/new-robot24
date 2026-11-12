import json
from pathlib import Path
import socket
import sys
import threading
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from host_bridge_node import HostBridgeServer, BridgeClient
from control_v2 import CAPABILITIES
from output_adapters import DryRunOutput


class Tests(unittest.TestCase):
    def setUp(self):
        self.server=HostBridgeServer('127.0.0.1',0,DryRunOutput())
        self.peers=[]
    def tearDown(self):
        self.server.stop_event.set()
        for peer in self.peers: peer.close()
        for client in list(self.server.active_clients): client.close()
        self.server.control.close()
    def connect(self):
        a,b=socket.socketpair()
        a.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,2048)
        b.settimeout(2)
        client=BridgeClient(self.server,a,('127.0.0.1',0))
        self.server.active_clients.append(client)
        self.peers.append(b)
        thread=threading.Thread(target=client.run,daemon=True);thread.start()
        return client,b,b.makefile('rb')
    def test_slow_monitor_does_not_block_heartbeat_emergency(self):
        slow,peer,stream=self.connect()
        stream.readline();stream.readline()
        fast,sock,f=self.connect()
        f.readline();f.readline()
        begin=time.monotonic()
        for _ in range(40): slow.send_json(dict(type='telemetry',data='x'*60000))
        self.assertLess(time.monotonic()-begin,.2)
        self.assertTrue(slow.closed)
        sock.sendall(b'{"type":"heartbeat","protocol_version":1,"seq":4}\n')
        self.assertEqual(json.loads(f.readline())['type'],'heartbeat_ack')
        sock.sendall(b'{"type":"emergency_stop"}\n')
        r=json.loads(f.readline())
        self.assertTrue(r['ok'])
        self.assertTrue(self.server.core.state.emergency_active)
        self.assertEqual(self.server.core.state.last_twist.linear_x,0)
        self.assertLess(time.monotonic()-begin,.5)
        f.close();stream.close()
    def test_legacy_upgrade_and_v2_monitor(self):
        client,sock,f=self.connect()
        f.readline();caps=json.loads(f.readline())
        self.assertFalse(caps['legacy_control_allowed'])
        sock.sendall(b'{"type":"operator_input","seq":1}\n')
        self.assertEqual(json.loads(f.readline())['error']['code'],'UPGRADE_REQUIRED')
        sock.sendall((json.dumps(dict(type='control_hello',protocol_version=2,robot_id='loopback_robot',role='monitor',capabilities=CAPABILITIES))+'\n').encode())
        response=json.loads(f.readline())
        self.assertTrue(response['ok'])
        self.assertEqual(response['result']['role'],'monitor')
        self.assertIsNone(self.server.control.owner)
        f.close()
    def test_video_timeout_is_operation_unknown(self):
        from control_fixture import Clock, Capture
        from bridge_core import BridgeCore
        from control_v2 import ControlV2
        clock=Clock();core=BridgeCore(Capture(clock),500,.8,1.5)
        gate=threading.Event()
        def camera(params):
            gate.wait(2)
            return False,2400,'video IPC timeout; outcome uncertain',None
        core.camera_stream_handler=camera
        c=ControlV2(core,clock)
        try:
            hello=c.handle('qt',dict(type='control_hello',protocol_version=2,robot_id=core.robot_id,role='control',capabilities=CAPABILITIES))['result']
            def call(kind,seq,**fields):
                return c.handle('qt',dict(type=kind,seq=seq,protocol_version=2,session_id=hello['session_id'],epoch=hello['epoch'],freshness_token=hello['freshness_token'],ttl_ms=300,**fields))
            call('operator_input',1,keyboard={'pressed_keys':[]})
            call('authorize_operations',2)
            oid=call('camera_stream_request',3,request_id='video-1',params={'action':'start'})['result']['operation_id']
            begin=time.monotonic()
            c.emergency();self.assertTrue(core.state.emergency_active)
            self.assertLess(time.monotonic()-begin,.1)
            gate.set()
            until=time.monotonic()+2
            while c.operations[oid]['state'] in ('accepted','running'):
                self.assertLess(time.monotonic(),until);time.sleep(.01)
            # If emergency won admission before worker started, no IPC call is allowed.
            self.assertIn(c.operations[oid]['state'],('failed','result_unknown'))
        finally:
            gate.set();c.close()

    def test_preset_negotiation_and_group_operation_over_socket(self):
        from test_motion_presets import PresetCapture
        from control_v2 import SUPPORTED_CAPABILITIES
        output = PresetCapture(time.monotonic)
        self.server.core.output = output
        client, sock, stream = self.connect()
        stream.readline()
        caps = json.loads(stream.readline())
        self.assertIn('motion_presets_v1', caps['control_capabilities'])
        self.assertEqual(caps['motion_presets']['arm']['groups'], ['arm','gripper'])
        session = {}
        seq = 0
        def call(kind, **fields):
            nonlocal seq
            seq += 1
            request = dict(type=kind, protocol_version=2, seq=seq, ttl_ms=300,
                session_id=session.get('session_id'), epoch=session.get('epoch'),
                freshness_token=session.get('freshness_token'), **fields)
            sock.sendall((json.dumps(request)+'\n').encode())
            return json.loads(stream.readline())
        try:
            hello = call('control_hello', robot_id=self.server.core.robot_id, role='control',
                capabilities=SUPPORTED_CAPABILITIES, presets=['flippers','arm'])
            self.assertTrue(hello['ok'], hello)
            session.update(hello['result'])
            call('operator_input', keyboard={'pressed_keys':[]})
            self.assertTrue(call('authorize_operations')['ok'])
            accepted = call('system_command', command='init', request_id='socket-preset', params={'presets':['arm']})
            self.assertTrue(accepted['ok'], accepted)
            deadline = time.monotonic()+2
            while True:
                op = call('operation_query', operation_id=accepted['result']['operation_id'])['result']
                if op['state'] not in ('accepted','running'):
                    break
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.005)
            self.assertEqual(op['state'], 'succeeded')
            self.assertEqual(output.group_calls, [('init',['arm','gripper'])])
            status = call('control_status')['result']
            self.assertEqual(status['selected_presets'], ['arm','flippers'])
            self.assertFalse(status['presets']['base']['selected'])
        finally:
            stream.close()

if __name__=='__main__':unittest.main(verbosity=2)
