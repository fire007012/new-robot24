#!/usr/bin/env python3
"""ROS2 SenseAir S8 over TCP/Modbus RTU. Validate complete response and CRC."""
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

def crc16(data):
    crc=0xffff
    for byte in data:
        crc^=byte
        for _ in range(8):crc=(crc>>1)^0xa001 if crc&1 else crc>>1
    return crc

def read_co2(host,port,timeout):
    request=bytes([0xfe,4,0,3,0,1,0xd5,0xc5])
    with socket.create_connection((host,port),timeout=timeout) as connection:
        connection.settimeout(timeout);connection.sendall(request);response=b''
        while len(response)<7:
            chunk=connection.recv(7-len(response))
            if not chunk:raise ConnectionError('sensor disconnected before complete frame')
            response+=chunk
    if response[:3]!=bytes([0xfe,4,2]) or crc16(response[:5])!=int.from_bytes(response[5:7],'little'):
        raise ValueError('invalid Modbus header or CRC')
    return int.from_bytes(response[3:5],'big')

class Co2Node(Node):
    def __init__(self):
        super().__init__('senseair_co2_publisher')
        self.host=self.declare_parameter('host','192.168.1.200').value
        self.port=self.declare_parameter('port',6600).value
        self.timeout=self.declare_parameter('timeout',2.0).value
        period=self.declare_parameter('period',4.0).value
        if period<=0 or not 0<self.timeout<=period:raise ValueError('require 0 < timeout <= period')
        self.pub=self.create_publisher(Int32,'/co2_ppm',1);self.timer=self.create_timer(period,self.sample)
    def sample(self):
        try:self.pub.publish(Int32(data=read_co2(self.host,self.port,self.timeout)))
        except (OSError,ValueError) as error:self.get_logger().warning(str(error))

def main():
    rclpy.init();node=Co2Node()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
