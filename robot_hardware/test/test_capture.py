"""Queued CAN evidence must preserve bus timing despite slow userspace reads."""
import socket
import struct
import unittest
from unittest.mock import patch

from robot_hardware_runtime.capture import (
    CanStatistics, FRAME, SO_RXQ_OVFL, SO_TIMESTAMPNS, TIMESPEC, receive_batch,
)


def packet(stamp, identifier=0x80, dropped=None, flags=0):
    ancillary = [(socket.SOL_SOCKET, SO_TIMESTAMPNS,
                  TIMESPEC.pack(stamp // 10**9, stamp % 10**9))]
    if dropped is not None:
        ancillary.append((socket.SOL_SOCKET, SO_RXQ_OVFL, struct.pack('=I', dropped)))
    return FRAME.pack(identifier, 0, b''), ancillary, flags, ('vcan0',)


class Queue:
    def __init__(self, packets):
        self.packets = iter(packets)

    def recvmsg(self, *_args):
        try:
            return next(self.packets)
        except StopIteration:
            raise BlockingIOError


class CaptureEvidence(unittest.TestCase):
    def test_backlog_drains_with_kernel_sync_spacing_and_drop_evidence(self):
        channel = Queue([packet(10**9 + i * 5_000_000, dropped=7) for i in range(600)])
        with patch('robot_hardware_runtime.capture.time.monotonic', return_value=42):
            records = receive_batch(channel, 'vcan0')
        self.assertEqual(len(records), 600)
        stats = CanStatistics(['vcan0', 'silent'])
        for record in records:
            stats.add(record)
        report = stats.report(3, 1_000_000)
        self.assertEqual(report['sync_intervals_s']['vcan0'], [.005] * 599)
        self.assertEqual(report['rx_dropped_total']['vcan0'], 7)
        self.assertEqual(report['frames']['silent'], 0)
        self.assertEqual(report['estimated_bus_load']['silent'], 0)

    def test_batch_limit_preserves_fairness_without_discarding_backlog(self):
        channel = Queue([packet(i) for i in range(3)])
        self.assertEqual(len(receive_batch(channel, 'vcan0', limit=2)), 2)
        self.assertEqual(len(receive_batch(channel, 'vcan0', limit=2)), 1)

    def test_missing_or_truncated_metadata_cannot_be_used_as_timing_evidence(self):
        for entry in [packet(0, flags=socket.MSG_CTRUNC),
                      packet(0, flags=socket.MSG_TRUNC),
                      (FRAME.pack(0x80, 0, b''), [], 0, ())]:
            with self.subTest(entry=entry), self.assertRaises(ValueError):
                receive_batch(Queue([entry]), 'vcan0')

    def test_sync_aliases_and_clock_regression_are_not_valid_intervals(self):
        packets = [packet(10), packet(12, 0x80 | socket.CAN_RTR_FLAG),
                   packet(13, 0x80 | socket.CAN_EFF_FLAG), packet(5), packet(15)]
        stats = CanStatistics(['vcan0'])
        for record in receive_batch(Queue(packets), 'vcan0'):
            stats.add(record)
        report = stats.report(1, 1_000_000)
        self.assertEqual(report['sync_intervals_s']['vcan0'], [10 / 10**9])
        self.assertEqual(report['timestamp_clock_regressions']['vcan0'], 1)


if __name__ == '__main__':
    unittest.main()
