#!/usr/bin/env python3
"""Sensor transport checks without attached hardware."""
import importlib.util,socket,threading,time,unittest
from pathlib import Path
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('co2',root/'senseair_co2/scripts/co2_publisher.py');co2=importlib.util.module_from_spec(spec);spec.loader.exec_module(co2)
class SensorSmoke(unittest.TestCase):
    def exchange(self,corrupt=False):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1',0));listener.listen();port=listener.getsockname()[1]
            def sensor():
                with listener.accept()[0] as connection:
                    request=connection.recv(8);self.assertEqual(co2.crc16(request[:-2]),int.from_bytes(request[-2:],'little'))
                    body=bytes([0xfe,4,2,0x03,0x20]);frame=body+co2.crc16(body).to_bytes(2,'little')
                    if corrupt:frame=frame[:-1]+bytes([frame[-1]^1])
                    for part in (frame[:2],frame[2:5],frame[5:]):connection.sendall(part);time.sleep(.01)
            worker=threading.Thread(target=sensor);worker.start()
            try:return co2.read_co2('127.0.0.1',port,1)
            finally:worker.join()
    def test_fragmented_frame(self):self.assertEqual(self.exchange(),800)
    def test_invalid_crc(self):
        with self.assertRaisesRegex(ValueError,'CRC'):self.exchange(True)
if __name__=='__main__':unittest.main(verbosity=2)
