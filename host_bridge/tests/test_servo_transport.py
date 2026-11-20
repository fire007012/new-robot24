"""Mode completion waits for live Servo transport, never for queued input."""
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from output_adapters import RosOutput


class ServoTransportTest(unittest.TestCase):
    def test_waiting_mode_only_completes_after_transport_is_ready(self):
        output=RosOutput.__new__(RosOutput)
        replies=iter([(False,2301,'preparing'),(True,0,'ready')])
        output._call_trigger_service=lambda path:next(replies)
        with patch('output_adapters.time.sleep'):
            ok,code,message=output.wait_servo_transport()
        self.assertTrue(ok)
        self.assertEqual(code,0)
        self.assertIn('explicit authorization',message)

    def test_unavailable_transport_does_not_report_mode_success(self):
        output=RosOutput.__new__(RosOutput)
        output._call_trigger_service=lambda path:(False,2301,'unavailable')
        with patch('output_adapters.time.monotonic',side_effect=[0,0,2]),patch('output_adapters.time.sleep'):
            ok,code,message=output.wait_servo_transport(timeout=1)
        self.assertFalse(ok)
        self.assertEqual(code,2301)
        self.assertIn('unavailable',message)
