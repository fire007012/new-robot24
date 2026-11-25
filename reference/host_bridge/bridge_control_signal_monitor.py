#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TwistStamped
from control_msgs.msg import JointJog

def main():
 rclpy.init();node=Node('bridge_control_signal_monitor')
 for kind,topic in [(Twist,'/cmd_vel'),(TwistStamped,'/servo_server/delta_twist_cmds'),(JointJog,'/flipper_control/jog_cmd')]:
  node.create_subscription(kind,topic,lambda msg,t=topic:node.get_logger().info(t+' '+str(msg)),1)
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
