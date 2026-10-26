#!/usr/bin/env python3
import importlib.util,unittest,os,signal,subprocess,time,uuid
from pathlib import Path
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image,Imu,CameraInfo
from rosgraph_msgs.msg import Clock
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('full_system_smoke',root/'robot_bringup/test/full_system_smoke.py');core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
class SimulationSmoke(unittest.TestCase):
    PACKAGE='robot_simulation';LAUNCH='full_system_simulate.launch.py';DOMAIN='84'
    ARGUMENTS=['enable_moveit:=false','gui:=false']
    @classmethod
    def setUpClass(cls):
        # Ignition discovery is independent of ROS_DOMAIN_ID. Isolate the world
        # from other simulations, including a developer's running GUI session.
        cls.previous_partition=os.environ.get('IGN_PARTITION')
        run_id=uuid.uuid4()
        cls.DOMAIN=str(100+run_id.int%100)
        os.environ['IGN_PARTITION']='robot24-smoke-'+run_id.hex
        core.FullSystemSmoke.setUpClass.__func__(cls)
    @classmethod
    def tearDownClass(cls):
        try:core.FullSystemSmoke.tearDownClass.__func__(cls)
        finally:
            # The ign Ruby launcher can exit before its server child. All
            # descendants belong to the session created by this test fixture.
            try:os.killpg(cls.process.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            if cls.previous_partition is None:os.environ.pop('IGN_PARTITION',None)
            else:os.environ['IGN_PARTITION']=cls.previous_partition
    joint_state=classmethod(core.FullSystemSmoke.joint_state.__func__)
    runtime_state=classmethod(core.FullSystemSmoke.runtime_state.__func__)
    wait=core.FullSystemSmoke.wait;result=core.FullSystemSmoke.result;call=core.FullSystemSmoke.call;trigger=core.FullSystemSmoke.trigger;goal=core.FullSystemSmoke.goal;prepare=core.FullSystemSmoke.prepare;conditions=core.FullSystemSmoke.conditions
    def test_physical_plant(self):
        self.frames={};self.imu=0;self.camera_info={};self.sim_time=0
        def clock(message):self.sim_time=message.clock.sec+message.clock.nanosec*1e-9
        self.node.create_subscription(Clock,'/clock',clock,10)
        for name in ('paw_camera','behind_camera'):
            self.node.create_subscription(CameraInfo,'/'+name+'/color/camera_info',lambda m,n=name:self.camera_info.update({n:m}),10)
        for topic in ('/paw_camera/color/image_raw','/paw_camera/depth/image_raw','/behind_camera/color/image_raw'):
            self.node.create_subscription(Image,topic,lambda m,t=topic:self.frames.update({t:(m.width,m.height,m.encoding)}),10)
        self.node.create_subscription(Imu,'/imu/data',lambda _:setattr(self,'imu',self.imu+1),10)
        def loaded():
            result=self.call('/controller_manager/list_controllers',ListControllers,ListControllers.Request())
            return len(result.controller)==6 and all(c.state=='inactive' for c in result.controller)
        self.wait(loaded,90);self.prepare();self.trigger('init');self.conditions();self.trigger('enable');self.trigger('resume')
        self.wait(lambda:self.runtime=='Running');self.wait(lambda:len(self.positions)>=13)
        client,handle=self.goal(.04,2)
        self.assertEqual(self.result(handle.get_result_async(),30).result.error_code,0)
        self.assertAlmostEqual(self.positions['shoulder_yaw_joint'],.04,delta=.01)
        pub=self.node.create_publisher(Twist,'/cmd_vel',1);self.wait(lambda:pub.get_subscription_count()>0)
        command=Twist();command.linear.x=.1
        def moving():
            pub.publish(command);return abs(self.velocities.get('left_front_wheel_joint',0))>.1
        self.wait(moving,10)
        self.wait(lambda:abs(self.velocities.get('left_front_wheel_joint',1))<.02,3)
        self.wait(lambda:len(self.frames)>=3 and self.imu>0,30)
        self.wait(lambda:len(self.camera_info)==2 and self.sim_time>0,15)
        for name,info in self.camera_info.items():
            self.assertEqual((info.width,info.height),(320,240))
            self.assertEqual(info.header.frame_id,name+'_color_optical_frame')
            self.assertGreater(info.k[0],0);self.assertGreater(info.k[4],0)
        self.assertEqual(self.frames['/paw_camera/depth/image_raw'][2],'32FC1')
        def pause(value):
            response=subprocess.run(['ign','service','-s','/world/robot24/control',
                '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
                '--timeout','3000','--req','pause: '+str(value).lower()],text=True,capture_output=True,timeout=6)
            self.assertEqual(response.returncode,0,response.stdout+response.stderr)
            self.assertIn('data: true',response.stdout)
        pause(True)
        try:
            time.sleep(.2);paused=self.sim_time
            # A command received while the plant is paused must expire in wall
            # time, so it cannot start moving the robot when the clock resumes.
            pub.publish(command);time.sleep(.7)
            self.assertEqual(self.sim_time,paused,'/clock advanced while paused')
        finally:pause(False)
        self.wait(lambda:self.sim_time>paused+.3,10)
        self.assertLess(abs(self.velocities.get('left_front_wheel_joint',1)),.02)
        # Pausing physics also pauses hardware feedback. The wall-time safety
        # watchdog closes the groups and the monitor stops their controllers
        # asynchronously. Wait for that published convergence before requesting
        # another operation; resuming /clock must not rearm the groups.
        def converged():
            groups=self.runtime_details.get('groups',{})
            return set(groups)=={'arm','base','flippers','gripper'} and all(
                state=='Fault' for state in groups.values())
        self.wait(converged,20)
        self.trigger('halt');self.trigger('shutdown')
        print('Fortress frames:',self.frames,'imu samples:',self.imu)
if __name__=='__main__':
    os.environ.setdefault('ROBOT_OFFLINE_REPORT','/tmp/robot24-fortress.log');unittest.main(verbosity=2)
