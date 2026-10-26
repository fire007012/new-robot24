#!/usr/bin/env python3
"""Execute commissioned ROS2 poses and request synchronized hand-eye samples."""
import time,math
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from arm_control.action import ExecuteArmGoal
from easy_handeye2_msgs.srv import TakeSample,ComputeCalibration,SaveCalibration
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('eye_in_hand_auto_sample');client=ActionClient(node,ExecuteArmGoal,'/arm_control/execute_goal');active=None
    node.declare_parameter('config_path','');path=node.get_parameter('config_path').value
    latest={'at':0.,'stamp':0.}
    def marker(msg):latest.update(at=time.monotonic(),stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    node.create_subscription(PoseStamped,'/aruco_single/pose',marker,1)
    def wait(future,seconds=90):
        rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
        if not future.done():raise TimeoutError('ROS2 operation timeout')
        return future.result()
    def request(kind,name):
        service=node.create_client(kind,'/easy_handeye2/calibration/'+name)
        if not service.wait_for_service(timeout_sec=5):raise RuntimeError(name+' unavailable')
        result=wait(service.call_async(kind.Request()),10);return result
    try:
        if not path:raise ValueError('config_path must identify a commissioned pose file')
        config=yaml.safe_load(Path(path).read_text())
        if config.get('commissioned') is not True:raise ValueError('placeholder poses are not commissioned')
        names=config['joint_names'];home=config['home_positions']
        if len(names)!=6 or len(set(names))!=6 or len(home)!=6:raise ValueError('six unique arm joints required')
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('arm action unavailable')
        previous=len(request(TakeSample,'get_sample_list').samples.samples)
        for pose in config['poses']:
            if len(pose['delta'])!=6:raise ValueError('invalid pose dimensions')
            values=[float(a)+float(b) for a,b in zip(home,pose['delta'])]
            if not all(math.isfinite(x) for x in values):raise ValueError('non-finite pose')
            goal=ExecuteArmGoal.Goal(target_type=ExecuteArmGoal.Goal.TARGET_JOINTS,joint_names=names,joint_positions=values)
            active=wait(client.send_goal_async(goal),5)
            if not active.accepted:raise RuntimeError('arm rejected sample pose')
            result=wait(active.get_result_async()).result;active=None
            if not result.success:raise RuntimeError(result.message)
            end=time.monotonic()+float(config['settle_time'])
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            settled=time.monotonic();deadline=settled+float(config['marker_timeout'])
            while time.monotonic()<deadline and (latest['at']<settled or abs(node.get_clock().now().nanoseconds*1e-9-latest['stamp'])>.3):rclpy.spin_once(node,timeout_sec=.05)
            if latest['at']<settled or abs(node.get_clock().now().nanoseconds*1e-9-latest['stamp'])>.3:raise RuntimeError('no fresh marker after settling; sampling stopped')
            sample=request(TakeSample,'take_sample');count=len(sample.samples.samples)
            if count<=previous:raise RuntimeError('server did not accept a new synchronized sample')
            previous=count;node.get_logger().info(pose['name']+' sampled')
        result=request(ComputeCalibration,'compute_calibration')
        if not result.valid:raise RuntimeError('calibration solver returned invalid result')
        saved=request(SaveCalibration,'save_calibration')
        if not saved.success:raise RuntimeError('calibration save failed')
        node.get_logger().info('Saved '+saved.filepath.data)
    finally:
        if active is not None and active.accepted and rclpy.ok():
            try:wait(active.cancel_goal_async(),3)
            except Exception:pass
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
