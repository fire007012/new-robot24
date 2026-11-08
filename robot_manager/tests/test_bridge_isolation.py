import json
from pathlib import Path
import socket
import subprocess
import sys
import time
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'host_bridge'))
from bridge_core import BridgeCore
from output_adapters import DryRunOutput

class Tests(unittest.TestCase):
    def test_watchdog_ignores_wall_clock(self):
        core=BridgeCore(DryRunOutput(),30,.8,1.5)
        with patch('bridge_core.now_ms',return_value=-10**12):
            core.handle_operator_input(dict(seq=1,keyboard={'pressed_keys':['w']}))
        time.sleep(.04)
        with patch('bridge_core.now_ms',return_value=-10**12):
            self.assertEqual(core.check_watchdog()[0]['operator_input_status'],'watchdog_timeout')
    def test_monitor_and_owner(self):
        script=Path(__file__).resolve().parents[2]/'host_bridge/host_bridge_node.py'
        state=tempfile.TemporaryDirectory(prefix='robot24-legacy-test-')
        proc=subprocess.Popen([sys.executable,str(script),'--host','127.0.0.1','--port','19901','--no-debug-ui','--dry-run','--watchdog-ms','5000','--control-state-dir',state.name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        sockets=[]
        try:
            deadline=time.monotonic()+5
            while True:
                try:
                    owner=socket.create_connection(('127.0.0.1',19901),timeout=1);break
                except OSError:
                    self.assertLess(time.monotonic(),deadline);time.sleep(.05)
            def connect(s=None):
                s=s or socket.create_connection(('127.0.0.1',19901),timeout=1)
                sockets.append(s);f=s.makefile('rb');f.readline();f.readline();return s,f
            owner,of=connect(owner)
            def send(s,obj):s.sendall((json.dumps(obj)+'\n').encode())
            def until(f,kind):
                for _ in range(30):
                    m=json.loads(f.readline())
                    if m['type']==kind:return m
                self.fail('missing '+kind)
            send(owner,dict(type='operator_input',seq=42,keyboard={'pressed_keys':['w']}))
            self.assertEqual(until(of,'control_result')['error']['code'],'UPGRADE_REQUIRED')
            monitor,mf=connect()
            send(monitor,dict(type='sync_request',seq=1))
            self.assertEqual(until(mf,'system_snapshot')['last_operator_input_seq'],0)
            send(monitor,dict(type='operator_input',seq=1,keyboard={'pressed_keys':['w']}))
            self.assertEqual(until(mf,'control_result')['error']['code'],'UPGRADE_REQUIRED')
            mf.close();monitor.close()
            time.sleep(.05)
            send(owner,dict(type='sync_request',seq=43))
            snapshot=until(of,'system_snapshot')
            self.assertEqual(snapshot['last_operator_input_seq'],0)
            self.assertEqual(snapshot['last_twist']['linear_x'],0)
        finally:
            for s in sockets:s.close()
            proc.terminate();proc.wait(timeout=5);state.cleanup()

if __name__=='__main__':unittest.main()
