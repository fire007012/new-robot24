"""Fence Servo input/output at runtime epochs; require a fresh post-start command."""
import json,math,time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float64MultiArray
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

class ServoGate(Node):
    def __init__(self):
        super().__init__('servo_gate');self.session=None;self.ready=False;self.last_input=0.;self.state_at=0.;self.input=None;self.pending=None;self.retry_at=0.
        self.pub=self.create_publisher(TwistStamped,'/servo_internal/delta_twist_cmds',1)
        self.output=self.create_publisher(Float64MultiArray,'/arm_servo_controller/command',1)
        self.start=self.create_client(Trigger,'/servo_server/start_servo');self.stop=self.create_client(Trigger,'/servo_server/stop_servo')
        self.create_subscription(String,'/robot_runtime/state',self.state,1)
        self.create_subscription(Float64MultiArray,'/servo_internal/joint_commands',self.command,1)
        self.create_timer(.05,self.watchdog)
    def state(self,msg):
        try:
            state=json.loads(msg.data)
            running=state.get('groups',{}).get('arm')=='Running' and state.get('arm_profile')=='servo'
            session=(running,json.dumps(state.get('operations',{}).get('arm',{}),sort_keys=True))
        except (ValueError,TypeError):running=False;session=(False,'invalid')
        if session!=self.session or time.monotonic()-self.state_at>.3:
            self.ready=False;self.last_input=0.;self.session=session
            if self.input:self.destroy_subscription(self.input);self.input=None
            self.output.publish(Float64MultiArray(data=[0.]*6))
            self.pending=None;self.retry_at=0.
            self.try_start()
        self.state_at=time.monotonic()
    def try_start(self):
        if not self.session or not self.session[0] or self.ready or self.pending is not None:return
        if time.monotonic()<self.retry_at or not self.stop.service_is_ready():return
        self.retry_at=time.monotonic()+1.
        self.pending=self.stop.call_async(Trigger.Request())
        self.pending.add_done_callback(lambda f,token=self.session:self.stopped(f,token))

    def stopped(self,future,token):
        if token!=self.session:return
        self.pending=None
        try:
            if not token[0] or not self.start.service_is_ready() or not future.result().success:return
            self.pending=self.start.call_async(Trigger.Request())
            self.pending.add_done_callback(lambda f,t=token:self.started(f,t))
        except Exception as error:self.get_logger().warning(str(error))
    def started(self,future,token):
        if token!=self.session:return
        self.pending=None
        try:
            if not future.result().success:return
        except Exception as error:
            self.get_logger().warning(str(error));return
        self.ready=True;self.activated=self.get_clock().now().nanoseconds
        self.input=self.create_subscription(TwistStamped,'/servo_server/delta_twist_cmds',lambda m,t=token:self.twist(m,t),1)
    def twist(self,msg,token):
        if token!=self.session or not self.ready or time.monotonic()-self.state_at>.3:return
        values=[getattr(vector,axis) for vector in (msg.twist.linear,msg.twist.angular) for axis in ('x','y','z')]
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if not all(math.isfinite(x) for x in values) or not msg.header.frame_id:return
        if stamp<self.activated or stamp>self.get_clock().now().nanoseconds:return
        self.last_input=time.monotonic();self.pub.publish(msg)
    def command(self,msg):
        if not self.ready or time.monotonic()-self.last_input>.1 or time.monotonic()-self.state_at>.3:return
        if len(msg.data)==6 and all(math.isfinite(x) for x in msg.data):self.output.publish(msg)
    def watchdog(self):
        if time.monotonic()-self.state_at<=.3:self.try_start()
        if not self.ready or time.monotonic()-self.last_input>.1 or time.monotonic()-self.state_at>.3:
            self.output.publish(Float64MultiArray(data=[0.]*6))

def main():
    rclpy.init();node=ServoGate()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
