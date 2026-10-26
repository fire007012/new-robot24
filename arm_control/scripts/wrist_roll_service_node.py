#!/usr/bin/env python3
"""ROS2 compatibility Trigger services; all execution goes through ExecuteArmGoal."""
import math,threading,time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from arm_control.action import ExecuteArmGoal
NAMES=['shoulder_yaw_joint','shoulder_pitch_joint','elbow_pitch_joint','wrist_pitch_joint','wrist_yaw_joint','wrist_roll_joint']
class WristNode(Node):
    def __init__(self):
        super().__init__('wrist_roll_service_node');self.values={};self.measured=0.;self.lock=threading.Lock()
        self.client=ActionClient(self,ExecuteArmGoal,'/arm_control/execute_goal',callback_group=ReentrantCallbackGroup())
        self.create_subscription(JointState,'/joint_states',self.joints,1)
        self.wrist_services=[]
        for suffix,delta in [('positive_180',math.pi),('negative_180',-math.pi),('zero',None)]:
            self.wrist_services.append(self.create_service(Trigger,'/arm_control/wrist_roll_'+suffix,lambda req,res,d=delta:self.execute(res,d),callback_group=ReentrantCallbackGroup()))
    def joints(self,msg):self.values=dict(zip(msg.name,msg.position));self.measured=time.monotonic()
    def wait(self,future,seconds):
        event=threading.Event();future.add_done_callback(lambda _:event.set())
        if not event.wait(seconds):raise TimeoutError('operation timeout')
        return future.result()
    def execute(self,response,delta):
        if not self.lock.acquire(False):response.message='wrist service busy';return response
        handle=None
        try:
            values=dict(self.values)
            if time.monotonic()-self.measured>.2 or any(name not in values or not math.isfinite(values[name]) for name in NAMES):raise ValueError('fresh six-axis joint feedback required')
            target=values['wrist_roll_joint']+delta if delta is not None else 0.0
            if not -7.069<=target<=7.069:raise ValueError('wrist target outside URDF limits')
            values['wrist_roll_joint']=target
            if not self.client.wait_for_server(timeout_sec=2):raise RuntimeError('ExecuteArmGoal unavailable')
            goal=ExecuteArmGoal.Goal(target_type=ExecuteArmGoal.Goal.TARGET_JOINTS,joint_names=NAMES,joint_positions=[values[x] for x in NAMES])
            handle=self.wait(self.client.send_goal_async(goal),3)
            if not handle.accepted:raise RuntimeError('arm action busy')
            result=self.wait(handle.get_result_async(),90).result;response.success=result.success;response.message=result.message
        except Exception as error:
            if handle is not None and handle.accepted:
                try:self.wait(handle.cancel_goal_async(),3)
                except Exception:pass
            response.message=str(error)
        finally:self.lock.release()
        return response

def main():
    rclpy.init();node=WristNode();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
