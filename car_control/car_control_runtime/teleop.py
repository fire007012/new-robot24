"""Native ROS2 keyboard/gamepad translator. No cached input survives a runtime epoch."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import SetBool

FLIPPERS=['left_front_arm_joint','right_front_arm_joint','left_rear_arm_joint','right_rear_arm_joint']

class KeyboardTeleopNode(Node):
    def __init__(self, joystick=False):
        super().__init__('ds5_teleop_node' if joystick else 'keyboard_teleop_node')
        self.mode='base'; self.speed=2; self.keys=set(); self.focused=False; self.input_at=0.0
        self.runtime={}; self.runtime_at=0.0; self.epochs={};self.joystick=joystick;self.joy=None;self.last_options=False
        self.mode_client=self.create_client(SetBool,'/arm_control/set_servo_mode');self.mode_future=None
        self.base=self.create_publisher(Twist,'/cmd_vel',1)
        self.servo=self.create_publisher(TwistStamped,'/servo_server/delta_twist_cmds',1)
        self.gripper=self.create_publisher(Float64,'/arm_control/gripper_velocity',1)
        self.flippers=self.create_publisher(JointJog,'/flipper_control/jog_cmd',1)
        self.status=self.create_publisher(String,'/car_control/keyboard_teleop/status',1)
        self.create_subscription(String,'/robot_runtime/state',self.state,1)
        if joystick:self.create_subscription(Joy,'/joy',self.joy_received,1)
        else:self.create_subscription(String,'/car_control/keyboard_teleop/raw_state',self.raw,1)
        self.timer=self.create_timer(1/30,self.step,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def state(self,msg):
        try:
            state=json.loads(msg.data)
            epochs={g:(v,json.dumps(state.get('operations',{}).get(g,{}),sort_keys=True)) for g,v in state.get('groups',{}).items()}
            if epochs!=self.epochs or time.monotonic()-self.runtime_at>.3:self.clear()
            self.runtime=state;self.epochs=epochs;self.runtime_at=time.monotonic()
        except (ValueError,TypeError):self.clear();self.runtime={}

    def clear(self):
        self.keys=set();self.joy=None;self.input_at=0.0;self.focused=False

    def allowed(self,group):
        return time.monotonic()-self.runtime_at<.3 and self.runtime.get('groups',{}).get(group,self.runtime.get('state'))=='Running'

    def raw(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data.get('pressed',[]),list) or not isinstance(data.get('pulse',[]),list):raise ValueError()
            pulses=set(data.get('pulse',[]));self.focused=data.get('focused') is True
            self.keys=set(data.get('pressed',[])) if self.focused else set()
            if 'tab' in pulses:self.toggle_mode()
            if 'enter' in pulses:self.keys=set();self.focused=False
            for key in sorted(pulses):
                if key in ('1','2','3','4','5'):self.speed=int(key)
            self.input_at=time.monotonic()
        except (ValueError,TypeError):self.clear()

    def joy_received(self,msg):
        if len(msg.axes)<6 or len(msg.buttons)<10 or any(not math.isfinite(x) for x in msg.axes):self.clear();return
        pressed=bool(msg.buttons[9])
        if pressed and not self.last_options:self.toggle_mode()
        self.last_options=pressed
        # Cross is hold-to-run; L1/R1 retain wrist roll.
        self.focused=bool(msg.buttons[1]);self.joy=msg;self.input_at=time.monotonic()

    def toggle_mode(self):
        self.clear()
        if self.mode_future is not None and not self.mode_future.done():return
        desired='arm' if self.mode=='base' else 'base'
        if not self.mode_client.service_is_ready():
            self.get_logger().warning('Servo mode service unavailable; mode unchanged');return
        self.mode_future=self.mode_client.call_async(SetBool.Request(data=desired=='arm'))
        def completed(future):
            try:
                if future.result().success:self.mode=desired
                else:self.get_logger().warning(future.result().message)
            except Exception as error:self.get_logger().warning(str(error))
            self.clear()
        self.mode_future.add_done_callback(completed)

    def axis(self,a,b):return float(a in self.keys)-float(b in self.keys)

    def step(self):
        active=self.focused and time.monotonic()-self.input_at<.25
        level=self.speed-1
        base=Twist();servo=TwistStamped();servo.header.stamp=self.get_clock().now().to_msg();servo.header.frame_id='grasp_tcp'
        grip=0.0;vel=[0.0]*4
        if active:
            if self.joystick and self.joy:
                axes=self.joy.axes
                axis=lambda i:axes[i] if abs(axes[i])>.12 else 0.0
                if self.mode=='base':base.linear.x=axis(1)*.4;base.angular.z=axis(0)*.8
                else:
                    servo.twist.linear.x=axis(1)*.08;servo.twist.linear.y=axis(0)*.08;servo.twist.linear.z=axis(4)*.08
                    servo.twist.angular.z=axis(3)*.4
                    servo.twist.angular.x=(self.joy.buttons[4]-self.joy.buttons[5])*.4
                    servo.twist.angular.y=(axis(2)-axis(5))*.2
                    grip=(float(bool(self.joy.buttons[3]))-float(bool(self.joy.buttons[0])))*.01
            elif self.mode=='base':
                base.linear.x=self.axis('w','s')*[.2,.4,.55,.7,.8][level]
                base.angular.z=self.axis('a','d')*[.4,.8,1.,1.25,1.5][level]
                vel=[self.axis(a,b)*[.05,.1,.15,.2,.3][level] for a,b in [('y','h'),('u','j'),('i','k'),('o','l')]]
            else:
                linear=[.02,.04,.06,.08,.1][level];angular=[.1,.2,.3,.4,.5][level]
                servo.twist.linear.x=self.axis('u','o')*linear;servo.twist.linear.y=self.axis('a','d')*linear;servo.twist.linear.z=self.axis('w','s')*linear
                servo.twist.angular.x=self.axis('q','e')*angular;servo.twist.angular.y=self.axis('k','i')*angular;servo.twist.angular.z=self.axis('j','l')*angular
                grip=self.axis('f','h')*[.004,.008,.012,.016,.02][level]
        if not self.allowed('base'):base=Twist()
        if not self.allowed('arm'):servo.twist=Twist()
        if not self.allowed('gripper'):grip=0.0
        if not self.allowed('flippers'):vel=[0.0]*4
        self.base.publish(base);self.servo.publish(servo);self.gripper.publish(Float64(data=grip))
        self.flippers.publish(JointJog(header=servo.header,joint_names=FLIPPERS,velocities=vel,duration=.15))
        self.status.publish(String(data=json.dumps({'mode':self.mode,'speed_level':self.speed,'focused':self.focused,'stale':not active,'servo_frame':'grasp_tcp','gripper_velocity_cmd':grip})))


def main(joystick=False):
    rclpy.init();node=KeyboardTeleopNode(joystick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.clear()
        if rclpy.ok():node.step()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
