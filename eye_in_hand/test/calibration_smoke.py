#!/usr/bin/env python3
"""Native calibration, known-transform solving, persistence and restarted TF."""
import os,signal,subprocess,tempfile,threading,time
from pathlib import Path
import numpy as np
from transforms3d.affines import compose
from transforms3d.euler import euler2mat
from transforms3d.quaternions import mat2quat,quat2mat
os.environ.update(QT_QPA_PLATFORM='offscreen',ROS_DOMAIN_ID='79',ROS_LOCALHOST_ONLY='1')
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import TransformBroadcaster,Buffer,TransformListener
from geometry_msgs.msg import TransformStamped
from easy_handeye2_msgs.srv import TakeSample,ComputeCalibration,SaveCalibration
from easy_handeye2.handeye_rqt_calibrator_widget import RqtHandeyeCalibratorWidget
from PyQt5 import QtWidgets
rclpy.init();node=Node('calibration_ui_test');broadcaster=TransformBroadcaster(node)
folder=tempfile.TemporaryDirectory(prefix='robot24-handeye-')
buffer=Buffer();listener=TransformListener(buffer,node)
expected=compose([.03,-.02,.08],euler2mat(.1,-.05,.08),[1,1,1])
base_marker=compose([.4,.1,.8],euler2mat(.05,.1,-.1),[1,1,1])
poses=[np.eye(4),np.linalg.inv(expected)@base_marker];lock=threading.Lock()
def matrix(transform):
    q,t=transform.rotation,transform.translation
    return compose([t.x,t.y,t.z],quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
def broadcast():
    transforms=[]
    stamp=node.get_clock().now().to_msg()
    with lock:
        for (parent,child),pose in zip([('base_link','tool0'),('paw_camera_link','aruco_marker_frame')],poses):
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=[float(x) for x in pose[:3,3]]
            t.transform.rotation.w,t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z=[float(x) for x in mat2quat(pose[:3,:3])]
            transforms.append(t)
    broadcaster.sendTransform(transforms)
node.create_timer(.03,broadcast);executor=SingleThreadedExecutor();executor.add_node(node);thread=threading.Thread(target=executor.spin);thread.start()
params=['name:=paw_d405_handeye','calibration_type:=eye_in_hand','robot_base_frame:=base_link','robot_effector_frame:=tool0','tracking_base_frame:=paw_camera_link','tracking_marker_frame:=aruco_marker_frame','freehand_robot_movement:=true']
def command(module):
    # Isolate upstream persistence without touching an operator's ~/.ros2 files.
    bootstrap='from pathlib import Path; import easy_handeye2.handeye_calibration as storage; '+f'storage.CALIBRATIONS_DIRECTORY=Path({folder.name!r}); from easy_handeye2.{module} import main; main()'
    return ['python3','-c',bootstrap,'--ros-args']
def stop(process):
    if process.poll() is None:
        os.kill(process.pid,signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    # The publisher can deliver its static TF before entering spin(); SIGINT in
    # that short startup window may terminate Python directly.
    assert process.returncode in (0,-signal.SIGINT),(process.returncode,Path('/tmp/robot24-handeye-server.log').read_text()[-5000:])
args=command('handeye_server')
for param in params:args+=['-p',param]
log=open('/tmp/robot24-handeye-server.log','w');p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
app=QtWidgets.QApplication([])
class Context:
    def __init__(self):self.node=Node('handeye_rqt_test');self.widgets=[]
    def argv(self):return []
    def serial_number(self):return 1
    def add_widget(self,w):self.widgets.append(w);w.show()
context=Context();executor.add_node(context.node)
widget=None;publisher=None
try:
    sample=node.create_client(TakeSample,'/easy_handeye2/calibration/take_sample');assert sample.wait_for_service(timeout_sec=20),'sampling services unavailable'
    result=sample.call(TakeSample.Request());assert len(result.samples.samples)==1,'TF sample not accepted'
    compute=node.create_client(ComputeCalibration,'/easy_handeye2/calibration/compute_calibration');assert compute.wait_for_service(timeout_sec=5)
    assert not compute.call(ComputeCalibration.Request()).valid,'one sample must not yield a valid calibration'
    save=node.create_client(SaveCalibration,'/easy_handeye2/calibration/save_calibration');assert save.wait_for_service(timeout_sec=5)
    assert not save.call(SaveCalibration.Request()).success,'invalid calibration must not be saved'
    widget=RqtHandeyeCalibratorWidget(None,context)
    for _ in range(50):app.processEvents();time.sleep(.01)
    assert context.widgets
    context.widgets[0].grab().save('/tmp/robot24-handeye-ui.png')
    # Settle beyond the native sampler's 200 ms lag. Independent rotations make
    # the hand-eye equation observable; no robot or real camera is involved.
    for index,angles in enumerate([(.4,0.,0.),(0.,-.4,0.),(0.,0.,.4),(-.3,.25,.1),(.2,-.3,-.25),(-.15,-.2,.35),(.3,.2,-.2)],start=2):
        robot=compose([.02*index,-.01*index,.1],euler2mat(*angles),[1,1,1])
        with lock:poses[:]=[robot,np.linalg.inv(robot@expected)@base_marker]
        time.sleep(.4)
        assert len(sample.call(TakeSample.Request()).samples.samples)==index
    result=compute.call(ComputeCalibration.Request());assert result.valid
    np.testing.assert_allclose(matrix(result.calibration.transform),expected,atol=1e-5)
    saved=save.call(SaveCalibration.Request());assert saved.success
    assert Path(saved.filepath.data)==Path(folder.name)/'paw_d405_handeye.calib'
    assert Path(saved.filepath.data).is_file()
    stop(p)
    for _ in range(2):
        buffer.clear()
        publisher=subprocess.Popen(command('handeye_publisher')+['-p','name:=paw_d405_handeye'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+8
        while not buffer.can_transform('tool0','paw_camera_link',rclpy.time.Time()):
            assert publisher.poll() is None,'calibration publisher exited'
            assert time.monotonic()<deadline,'saved calibration TF unavailable'
            time.sleep(.03)
        transform=buffer.lookup_transform('tool0','paw_camera_link',rclpy.time.Time())
        np.testing.assert_allclose(matrix(transform.transform),expected,atol=1e-5)
        stop(publisher)
    widget.shutdown()
    for window in context.widgets:window.close()
    print('Native rqt, TF sampling, insufficient-sample refusal, known-transform solve, .calib save and two fresh TF publishers passed')
finally:
    try:
        if widget is not None:widget.shutdown()
        for window in context.widgets:window.close()
        if publisher is not None:stop(publisher)
        stop(p)
    finally:
        log.close();executor.shutdown();thread.join();listener.unregister();node.destroy_node();context.node.destroy_node();rclpy.shutdown();folder.cleanup()
