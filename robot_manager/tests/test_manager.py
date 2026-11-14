import concurrent.futures
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from manager import Manager, Fault, Server, wire
from client import request

class Backend:
    def __init__(self):
        self.started=[]; self.stopped=[]; self.bad=False; self.optional=False; self.wait=0
    def validate(self,p): pass
    def preflight(self,p): return dict(passed=not self.bad,checks=[])
    def required(self,p): return ['core','bridge']
    def modules(self,p): return ['core','bridge']
    def start(self,module,p,oid):
        self.started.append(module)
        return dict(module=module,pid=len(self.started),identity=['test','1'])
    def health(self,p):
        time.sleep(self.wait)
        return dict(modules=dict(core=dict(ok=not self.bad),bridge=dict(ok=not self.bad),video=dict(ok=not self.optional)),runtime=dict(state='Stopped'),observed_at=time.time())
    def halt(self,p): self.stopped.append('halt')
    def stop(self,r): self.stopped.append(r['module'])

class Tests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='robot24-manager-test-')
        self.root=Path(self.tmp.name)
        (self.root/'profiles').mkdir()
        (self.root/'profiles/mock.json').write_text(json.dumps(dict(profile_id='mock',config_revision='1',mode='mock',build_version='test',video=False,startup_timeout=.05)))
        (self.root/'token').write_text('x'*32)
        self.config=self.root/'config.json'
        self.config.write_text(json.dumps(dict(config_root=str(self.root/'profiles'),state_dir=str(self.root/'state'),token_file=str(self.root/'token'),robot_id='robot24',build_version='test',control_endpoint=dict(host='127.0.0.1',port=9090),port=0)))
        self.backend=Backend()
        self.m=Manager(self.config,self.backend)
    def tearDown(self):
        self.m.closed.set(); self.m.lock_file.close(); self.tmp.cleanup()
    def call(self,method,rid=None,**p):
        return self.m.dispatch(dict(version=1,token='x'*32,request_id=rid or str(time.monotonic_ns()),method=method,params=p))
    def start(self,rid=None): return self.call('start',rid,robot_id='robot24',profile_id='mock',config_revision='1')
    def done(self):
        deadline=time.monotonic()+5
        while self.m.active():
            self.assertLess(time.monotonic(),deadline)
            time.sleep(.01)
        return self.call('operation')
    def test_twenty_cycles(self):
        for i in range(20):
            self.start(); self.assertEqual(self.done()['state'],'succeeded')
            self.call('stop'); self.assertEqual(self.done()['state'],'succeeded')
            self.assertFalse(self.m.data['resources'])
        self.assertEqual(len(self.backend.started),40)
        self.assertEqual(self.backend.stopped,['halt','bridge','core']*20)

    def test_requested_selection_is_validated_and_persisted_without_changing_profile(self):
        args=dict(robot_id='robot24',profile_id='mock',config_revision='1')
        for selection in ([],['unknown'],['base','base'],None):
            with self.assertRaises(Fault):self.call('preflight',**args,presets=selection)
        self.call('start',**args,presets=['base'])
        self.assertEqual(self.done()['state'],'succeeded')
        self.assertEqual(self.m.data['profile']['requested_presets'],['base'])
        deployed=json.loads((self.root/'profiles/mock.json').read_text())
        self.assertNotIn('requested_presets',deployed)
        self.call('stop');self.done()
    def test_parallel_and_replay(self):
        self.backend.wait=.2
        with concurrent.futures.ThreadPoolExecutor(8) as pool:
            ops=list(pool.map(lambda i:self.start(str(i)),range(8)))
        self.assertEqual(len({o['operation_id'] for o in ops}),1)
        self.done()
        self.assertEqual(self.start('0')['operation_id'],ops[0]['operation_id'])
        with self.assertRaises(Fault): self.call('stop','0')
        self.assertEqual(self.backend.started,['core','bridge'])
        self.call('stop');self.done()
    def test_failure_and_cleanup(self):
        self.backend.bad=True
        self.start(); self.assertEqual(self.done()['state'],'failed')
        self.assertEqual(self.backend.started,[])
        self.backend.bad=False
        self.backend.health=lambda p:dict(modules=dict(core=dict(ok=False)),runtime=None)
        self.start();self.assertEqual(self.done()['error']['code'],'HEALTH')
        self.assertEqual(self.backend.stopped,['halt','bridge','core'])
    def test_degraded_and_bridge_crash(self):
        self.backend.optional=True
        self.start();self.done()
        self.assertEqual(self.call('snapshot')['software_state'],'degraded')
        self.backend.bad=True
        deadline=time.monotonic()+4
        while self.call('snapshot')['software_state']!='failed':
            self.assertLess(time.monotonic(),deadline);time.sleep(.05)
        self.assertEqual(self.call('hello')['protocol_version'],1)
        self.call('stop');self.done()
    def test_reconcile(self):
        self.start();self.done()
        self.m.data['operations'][self.m.data['current']]['state']='running'
        self.m.save()
        self.m.closed.set();self.m.lock_file.close()
        self.m=Manager(self.config,self.backend)
        self.assertEqual(self.call('operation')['error']['code'],'RECOVERY_REQUIRED')
        self.assertTrue(self.call('snapshot')['recovery_required'])
        with self.assertRaises(Fault):self.start()
        self.call('stop');self.done()
    def test_invalid(self):
        with self.assertRaises(Fault):self.call('start',robot_id='robot24',profile_id='../mock',config_revision='1')
        with self.assertRaises(Fault):self.call('start',robot_id='robot24',profile_id='mock',config_revision='2')
        with self.assertRaises(Fault):self.call('logs',operation_id='../token')
    def test_log_pagination_obeys_wire_budget(self):
        self.start();self.done()
        oid=self.m.data['current']
        (self.m.state_dir/(oid+'.log')).write_text(('汉'*4000+'\n')*100)
        page=self.call('logs',operation_id=oid,limit=200)
        self.assertGreater(page['next_cursor'],0)
        self.assertLess(page['next_cursor'],100)
        self.assertLess(len(wire(page)),65536)
        self.call('stop');self.done()

    def test_corrupt_deployed_profile(self):
        (self.root/'profiles/mock.json').write_text('{broken}')
        with self.assertRaises(Fault) as caught:self.start()
        self.assertEqual(caught.exception.data['code'],'CONFIG')
        self.assertFalse(self.backend.started)

    def test_disconnect_and_query(self):
        self.backend.wait=.15
        server=Server(self.m)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        port=server.server_address[1]
        try:
            payload=dict(version=1,token='x'*32,method='start',request_id='disconnected',params=dict(robot_id='robot24',profile_id='mock',config_revision='1'))
            with socket.create_connection(('127.0.0.1',port)) as conn:
                conn.sendall((json.dumps(payload)+'\n').encode())
            time.sleep(.05)
            op=request('127.0.0.1',port,'x'*32,'operation')['result']
            self.assertEqual(op['operation_id'],self.m.data['current'])
            self.done()
            self.call('stop');self.done()
            with socket.create_connection(('127.0.0.1',port)) as conn:
                conn.sendall(b'{broken}\n')
                self.assertEqual(json.loads(conn.makefile('rb').readline())['error']['code'],'BAD_REQUEST')
        finally:
            server.shutdown();server.server_close();thread.join()

if __name__=='__main__':unittest.main()
