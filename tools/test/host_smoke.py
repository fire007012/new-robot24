#!/usr/bin/env python3
import json,os,signal,socket,subprocess,sys,tempfile,time,unittest
from pathlib import Path
root=Path(__file__).resolve().parents[2];sys.path.insert(0,str(root/'host_bridge'))
from bridge_core import BridgeCore
from output_adapters import DryRunOutput
from bridge_protocol import now_ms
from video_manager_ipc import VideoManagerClient,VideoManagerClientError
class Capture(DryRunOutput):
    def publish_twist(self,x):self.twist=x
    def publish_servo(self,x):self.servo=x
    def publish_gripper(self,x):self.gripper=x
    def publish_flipper(self,x):self.flipper=x
class HostSmoke(unittest.TestCase):
    def test_installed_video_stack(self):
        # Empty sources exercise orchestration without opening physical cameras
        # or relying on an external RTSP server. Real video remains a field test.
        with tempfile.TemporaryDirectory(prefix='robot24-video-test-') as folder:
            config=Path(folder)/'sources.yaml'
            config.write_text('rtsp:\n  public_host: 127.0.0.1\n  publish_host: 127.0.0.1\n  port: 18554\ndirect_sources: []\nstreams: []\n')
            env=dict(os.environ,ROS_DOMAIN_ID='80',ROS_LOCALHOST_ONLY='1')
            with open('/tmp/robot24-video-stack.log','w') as log:
                process=subprocess.Popen(['ros2','launch','host_bridge','video_stack.launch.py',
                    'video_config:='+str(config),'host:=127.0.0.1','port:=19091',
                    'manager_port:=18082','log_dir:='+folder],stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
                try:
                    manager=VideoManagerClient('127.0.0.1',18082,timeout=.5)
                    deadline=time.monotonic()+20
                    while True:
                        self.assertIsNone(process.poll(),Path(log.name).read_text())
                        try:
                            self.assertTrue(manager.ping()['ok'])
                            break
                        except VideoManagerClientError:
                            self.assertLess(time.monotonic(),deadline,'video manager unavailable')
                            time.sleep(.1)
                    self.assertEqual(manager.list_cameras(),[])
                    self.assertTrue(manager.request({'type':'stop_all'})['ok'])
                    self.assertFalse(manager.stream_request(5,'start')['ok'])
                    while True:
                        try:
                            connection=socket.create_connection(('127.0.0.1',19091),timeout=.5)
                            break
                        except OSError:
                            self.assertLess(time.monotonic(),deadline,'video host bridge unavailable')
                            time.sleep(.1)
                    with connection:
                        connection.settimeout(5)
                        connection.sendall(b'{"type":"heartbeat","seq":7,"protocol_version":1}\n')
                        stream=connection.makefile('rb')
                        while True:
                            message=json.loads(stream.readline())
                            if message['type']=='heartbeat_ack':break
                        self.assertEqual(message['seq'],7)
                    deadline=time.monotonic()+10
                    while 'started 0 stream(s)' not in Path(log.name).read_text():
                        self.assertLess(time.monotonic(),deadline,'ROS2 image/RTSP node did not start')
                        time.sleep(.1)
                finally:
                    if process.poll() is None:os.kill(process.pid,signal.SIGINT)
                    try:process.wait(timeout=12)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
                self.assertEqual(process.returncode,0,Path(log.name).read_text())
                self.assertNotIn('process has died',Path(log.name).read_text())

    def test_watchdog_reconnect_and_mapping(self):
        output=Capture();core=BridgeCore(output,30,.8,1.5,servo_frame='grasp_tcp')
        request={'type':'operator_input','seq':1,'timestamp_ms':now_ms(),'ttl_ms':200,'keyboard':{'pressed_keys':['w']}}
        list(core.handle_message(request));self.assertGreater(output.twist.linear_x,0)
        time.sleep(.04);core.check_watchdog();self.assertEqual(output.twist.linear_x,0)
        core.reset_operator_input_session('reconnect');self.assertEqual(core.state.last_pressed_keys,[])
        request.update(timestamp_ms=now_ms(),seq=1);list(core.handle_message(request));self.assertGreater(output.twist.linear_x,0)
        core.reset_operator_input_session('runtime_epoch');self.assertEqual(output.twist.linear_x,0);self.assertEqual(output.gripper.velocity,0)
    def test_installed_tcp_launch(self):
        env=dict(os.environ,ROS_DOMAIN_ID='80',ROS_LOCALHOST_ONLY='1')
        log=open('/tmp/robot24-host.log','w');p=subprocess.Popen(['ros2','launch','host_bridge','host_bridge.launch.py','host:=127.0.0.1','port:=19090'],stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        try:
            def connect():
                end=time.monotonic()+20
                while time.monotonic()<end:
                    self.assertIsNone(p.poll(),Path('/tmp/robot24-host.log').read_text())
                    try:return socket.create_connection(('127.0.0.1',19090),timeout=.5)
                    except OSError:time.sleep(.1)
                self.fail('bridge TCP port unavailable')
            for seq in (1,2):
                with connect() as connection:
                    connection.settimeout(5);connection.sendall((json.dumps({'type':'heartbeat','seq':seq,'protocol_version':1})+'\n').encode());stream=connection.makefile('rb')
                    messages=[]
                    while True:
                        response=json.loads(stream.readline());messages.append(response)
                        if response['type']=='heartbeat_ack':break
                    self.assertEqual(response['seq'],seq)
                    self.assertTrue(any(m['type']=='hello' for m in messages),str(messages))
        finally:
            os.kill(p.pid,signal.SIGINT)
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            log.close()
        self.assertEqual(p.returncode,0,Path('/tmp/robot24-host.log').read_text())
if __name__=='__main__':unittest.main(verbosity=2)
