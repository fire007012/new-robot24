#!/usr/bin/env python3
"""ROS2 SenseAir S8 over TCP/Modbus RTU. Validate complete response and CRC."""
import json
import socket
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

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
        self.period=self.declare_parameter('period',4.0).value
        if self.period<=0 or not 0<self.timeout<=self.period:raise ValueError('require 0 < timeout <= period')
        self.pub=self.create_publisher(Int32,'/co2_ppm',1)
        self.status_pub=self.create_publisher(String,'/co2_status',1)
        self._stop_event=threading.Event()
        self._last_error_log_monotonic=0.0
        self._worker=threading.Thread(target=self._poll_loop,name='co2_poll_worker',daemon=True)
        self._worker.start()

    def _publish_status(self,online,ppm=None,error=''):
        if self._stop_event.is_set():
            return
        payload={
            'online':bool(online),
            'ppm':int(ppm) if ppm is not None else None,
            'error':str(error),
            'timestamp_ms':int(time.time()*1000),
        }
        try:
            self.status_pub.publish(String(data=json.dumps(payload,ensure_ascii=False,separators=(',',':'))))
        except Exception:
            pass

    def sample(self):
        try:
            ppm=read_co2(self.host,self.port,self.timeout)
        except (OSError,ValueError) as error:
            now=time.monotonic()
            if now-self._last_error_log_monotonic>=30.0:
                self._last_error_log_monotonic=now
                self.get_logger().warning(str(error))
            self._publish_status(False,error=str(error))
            return
        self._last_error_log_monotonic=0.0
        if self._stop_event.is_set():
            return
        try:
            self.pub.publish(Int32(data=ppm))
        except Exception:
            pass
        self._publish_status(True,ppm=ppm)

    def _poll_loop(self):
        while not self._stop_event.is_set():
            started=time.monotonic()
            self.sample()
            remaining=max(0.05,float(self.period)-(time.monotonic()-started))
            if self._stop_event.wait(remaining):
                break

    def destroy_node(self):
        self._stop_event.set()
        worker=getattr(self,'_worker',None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=1.0)
        return super().destroy_node()

def main():
    rclpy.init();node=Co2Node()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
