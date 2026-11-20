"""Fence Servo input/output at runtime epochs; require a fresh post-start command."""
import json,math,time,signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float64MultiArray
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

class ServoGate(Node):
    def __init__(self):
        super().__init__('servo_gate');self.session=None;self.ready=False;self.last_input=0.;self.state_at=0.;self.input=None;self.pending=None;self.retry_at=0.;self.servo_at=0.
        self.pub=self.create_publisher(TwistStamped,'/servo_internal/delta_twist_cmds',1)
        self.output=self.create_publisher(Float64MultiArray,'/arm_servo_controller/command',1)
        self.start=self.create_client(Trigger,'/servo_server/start_servo');self.stop=self.create_client(Trigger,'/servo_server/stop_servo')
        self.resume=self.create_client(Trigger,'/servo_server/unpause_servo');self.engine_started=False
        self.create_subscription(String,'/robot_runtime/state',self.state,1)
        self.create_subscription(Float64MultiArray,'/servo_internal/joint_commands',self.command,1)
        self.create_timer(.05,self.watchdog)
        self.transport_status=self.create_service(Trigger,'/arm_control/servo_transport_ready',self.status)
    def status(self, request, response):
        response.success=bool(self.ready and self.session and self.session[0] and
                              time.monotonic()-self.state_at<.3 and self.input is not None and
                              self.pub.get_subscription_count()>0 and time.monotonic()-self.servo_at<.3)
        response.message='Servo transport ready; fresh input required' if response.success else 'Servo transport is preparing'
        return response

    def state(self,msg):
        try:
            state=json.loads(msg.data)
            running=state.get('groups',{}).get('arm')=='Running' and state.get('arm_profile')=='servo'
            session=(running,json.dumps(state.get('operations',{}).get('arm',{}),sort_keys=True))
        except (ValueError,TypeError):running=False;session=(False,'invalid')
        if session!=self.session or time.monotonic()-self.state_at>.3:
            self.ready=False;self.last_input=0.;self.servo_at=0.;self.session=session
            if self.input:self.destroy_subscription(self.input);self.input=None
            self.output.publish(Float64MultiArray(data=[0.]*6))
            # Keep any in-flight RPC until its callback drains. Clearing it
            # here would queue overlapping starts across runtime epochs.
            self.retry_at=0.
            self.try_start()
        self.state_at=time.monotonic()
    def try_start(self):
        if not self.session or not self.session[0] or self.ready or self.pending is not None:return
        if time.monotonic()<self.retry_at or not self.stop.service_is_ready():return
        self.retry_at=time.monotonic()+1.
        self.pending=self.stop.call_async(Trigger.Request())
        self.pending.add_done_callback(lambda f,token=self.session:self.stopped(f,token))

    def stopped(self,future,token):
        if self.pending is not future:return
        self.pending=None
        if token!=self.session:
            self.try_start();return
        try:
            initial=not self.engine_started
            client=self.start if initial else self.resume
            if not token[0] or not client.service_is_ready() or not future.result().success:return
            # Humble start_servo recreates calculation threads/timers. Start
            # once, then use pause/unpause across epochs and feedback gaps.
            self.pending=client.call_async(Trigger.Request())
            self.pending.add_done_callback(lambda f,t=token,i=initial:self.started(f,t,i))
        except Exception as error:self.get_logger().warning(str(error))
    def started(self,future,token,initial=True):
        if self.pending is not future:return
        self.pending=None
        try:
            if not future.result().success:return
        except Exception as error:
            self.get_logger().warning(str(error));return
        if initial:self.engine_started=True
        if token!=self.session:
            self.try_start();return
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
        if (self.ready and len(msg.data)==6 and all(math.isfinite(x) for x in msg.data)
                and all(abs(x)<1e-6 for x in msg.data)):
            self.servo_at=time.monotonic()
        if not self.ready or time.monotonic()-self.last_input>.1 or time.monotonic()-self.state_at>.3:return
        if len(msg.data)==6 and all(math.isfinite(x) for x in msg.data):self.output.publish(msg)
    def watchdog(self):
        if time.monotonic()-self.state_at<=.3:
            self.try_start()
            if self.ready and time.monotonic()-self.last_input>.1:
                # Warm the calculation loop with neutral input before reporting
                # mode readiness. This never grants or forwards operator motion.
                neutral=TwistStamped();neutral.header.stamp=self.get_clock().now().to_msg()
                neutral.header.frame_id='grasp_tcp';self.pub.publish(neutral)
        if not self.ready or time.monotonic()-self.last_input>.1 or time.monotonic()-self.state_at>.3:
            self.output.publish(Float64MultiArray(data=[0.]*6))

def main():
    rclpy.init();node=ServoGate()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
