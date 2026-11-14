"""Decode captured motor frames and reject non-status data without hardware."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from diagnose_dm_velocity import decode_status, speed_payload, mit_velocity_payload

class RawDiagnosticTest(unittest.TestCase):
    def test_captured_stalled_node7_frame(self):
        sample=decode_status(8,bytes.fromhex('1789707ffb63211b'),7,8,(12.5,25.,200.))
        self.assertEqual(sample['state'],1)
        self.assertAlmostEqual(sample['position'],.92184,places=4)
        self.assertLess(abs(sample['velocity']),.01)
        self.assertAlmostEqual(sample['torque'],84.74,places=2)

    def test_captured_moving_node5_frame(self):
        sample=decode_status(6,bytes.fromhex('1579247bf7ec2019'),5,6,(12.5,15.,40.))
        self.assertAlmostEqual(sample['velocity'],-.4725,places=4)
        self.assertEqual(sample['state'],1)

    def test_other_motor_and_register_responses_are_not_status(self):
        data=bytes.fromhex('1789707ffb63211b')
        self.assertIsNone(decode_status(6,data,7,8,(12.5,25.,200.)))
        self.assertIsNone(decode_status(8,data,5,8,(12.5,25.,200.)))
        self.assertIsNone(decode_status(8,bytes.fromhex('0700330a03000000'),7,8,(12.5,25.,200.)))
        self.assertIsNone(decode_status(8,data[:4],7,8,(12.5,25.,200.)))

    def test_velocity_wire_format_and_nonfinite_rejection(self):
        self.assertEqual(speed_payload(.5),bytes.fromhex('0000003f'))
        self.assertEqual(speed_payload(-.5),bytes.fromhex('000000bf'))
        for value in (float('nan'),float('inf'),-float('inf')):
            with self.assertRaises(ValueError):speed_payload(value)

    def test_mit_velocity_has_zero_position_gain_and_feedforward(self):
        for target in (-.5, .5):
            data=mit_velocity_payload(target,(12.5,25.,200.),4.)
            velocity=((data[2]<<4)|(data[3]>>4))*50/4095-25
            kp=(((data[3]&15)<<8)|data[4])*500/4095
            kd=((data[5]<<4)|(data[6]>>4))*5/4095
            torque=(((data[6]&15)<<8)|data[7])*400/4095-200
            self.assertAlmostEqual(velocity,target,delta=50/4095)
            self.assertEqual(kp,0.)
            self.assertAlmostEqual(kd,4.,places=5)
            self.assertLess(abs(torque),400/4095)

if __name__=='__main__':unittest.main()
