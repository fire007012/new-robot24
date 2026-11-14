"""Controller notice handling; uses no physical CAN interface."""
import contextlib
import io
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import motor_discovery as discovery


class ControllerNoticeTest(unittest.TestCase):
    def scan(self, error_id=0x20000004, payload=bytes(8), changed=False, missing=False):
        stats = dict.fromkeys(("restarts", "bus_error", "arbitration_lost",
                              "error_warning", "error_passive", "bus_off"), 0)
        final = dict(stats, error_warning=int(changed))
        ticks = iter(i * .01 for i in range(1000))

        class Bus:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def setsockopt(self, *args): pass
            def bind(self, *args): pass
            def settimeout(self, *args): pass
            def send(self, frame):
                self.pending = [struct.pack("=IB3x8s", error_id, 8, payload),
                                struct.pack("=IB3x8s", 6, 8, bytes.fromhex("0500330a03000000"))]
            def recv(self, size):
                if self.pending: return self.pending.pop(0)
                raise TimeoutError

        snapshots = [{}, {}, {}] if missing else [stats, stats, final]
        with patch.object(discovery, "link_state", side_effect=snapshots), \
                patch.object(discovery.socket, "socket", return_value=Bus()), \
                patch.object(discovery.time, "monotonic", side_effect=lambda: next(ticks)), \
                contextlib.redirect_stdout(io.StringIO()):
            return discovery.scan_can("DM", "can0", [5], .1, 1)

    def test_empty_notice_requires_healthy_link_and_unchanged_counters(self):
        self.assertEqual(self.scan(), [dict(motor_id=5, master_id=6, mode=3, device="can0")])

    def test_empty_notice_with_new_faults_is_rejected(self):
        with self.assertRaises(RuntimeError): self.scan(changed=True)

    def test_empty_notice_without_kernel_counters_is_rejected(self):
        with self.assertRaises(RuntimeError): self.scan(missing=True)

    def test_ack_busoff_and_warning_frames_still_abort(self):
        for ident, payload in ((0x20000024, bytes(8)), (0x20000040, bytes(8)),
                               (0x20000004, bytes.fromhex("000c000000000000"))):
            with self.subTest(ident=ident, payload=payload), self.assertRaises(RuntimeError):
                self.scan(ident, payload)


if __name__ == "__main__": unittest.main()
