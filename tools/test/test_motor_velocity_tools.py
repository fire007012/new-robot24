"""Offline protocol/selection checks; never opens a real CAN or SDK connection."""
import contextlib
import io
import json
import math
from pathlib import Path
import runpy
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import motor_discovery as discovery
import motor_velocity_common as runner
from run_canopen_velocity import SdoBus


def defaults(protocol):
    values = runpy.run_path(str(Path(__file__).resolve().parents[1] / f"run_{protocol.lower()}_velocity.py"))
    return {"speed": values["DEFAULT_SPEED"], "duration": values["DEFAULT_DURATION"],
            "motor_id": values["DEFAULT_MOTOR_ID"], "device": values["DEFAULT_DEVICE"],
            "position_scale": values["DEFAULT_POSITION_SCALE"], "velocity_scale": values["DEFAULT_VELOCITY_SCALE"],
            "direction": values["DEFAULT_DIRECTION"]}


class DiscoveryTest(unittest.TestCase):
    def test_readonly_queries(self):
        self.assertEqual(discovery.query("DM", 7), (0x7FF, bytes.fromhex("0700330a00000000")))
        self.assertEqual(discovery.query("MT", 1), (0x141, bytes.fromhex("9a00000000000000")))
        self.assertEqual(discovery.query("PP", 19), (19, bytes.fromhex("0302")))
        self.assertEqual(discovery.query("CANOPEN", 6), (0x606, bytes.fromhex("4018100100000000")))

    def test_matching_identity_replies(self):
        self.assertEqual(discovery.decode("DM", 8, bytes.fromhex("0700330a03000000")),
                         {"motor_id": 7, "master_id": 8, "mode": 3})
        self.assertEqual(discovery.decode("MT", 0x241, bytes.fromhex("9a00000000000200")),
                         {"motor_id": 1, "fault_code": 2})
        self.assertEqual(discovery.decode("PP", 19, bytes.fromhex("040212345678")),
                         {"motor_id": 19, "serial": "12345678"})
        self.assertEqual(discovery.decode("CANOPEN", 0x586, bytes.fromhex("4318100178563412")),
                         {"motor_id": 6, "vendor_id": 0x12345678})

    def test_reject_unrelated_frames(self):
        for protocol in ("DM", "MT", "PP", "CANOPEN"):
            self.assertIsNone(discovery.decode(protocol, 0x20000008, bytes(8)))
            self.assertIsNone(discovery.decode(protocol, 8, bytes(2)))
        self.assertIsNone(discovery.decode("DM", 8, bytes.fromhex("0700330aff000000")))
        self.assertIsNone(discovery.decode("PP", 19, bytes.fromhex("030212345678")))
        self.assertIsNone(discovery.decode("CANOPEN", 0x586, bytes.fromhex("8018100100000206")))

    def test_no_target_and_multiple_targets_refuse_motion(self):
        with self.assertRaises(RuntimeError):
            discovery.select_target([])
        candidates = [{"motor_id": 1}, {"motor_id": 7}]
        with self.assertRaises(RuntimeError):
            discovery.select_target(candidates)
        self.assertEqual(discovery.select_target(candidates, 7), {"motor_id": 7})

    def test_duplicate_ecb_id_requires_ip_filter(self):
        candidates = [{"motor_id": 1, "device": "ecb://192.168.1.1"},
                      {"motor_id": 1, "device": "ecb://192.168.1.2"}]
        with self.assertRaises(RuntimeError):
            discovery.select_target(candidates, 1)

    def test_dm_master_filter(self):
        rows = [{"motor_id": 7, "master_id": 8}, {"motor_id": 7, "master_id": 0}]
        self.assertEqual(discovery.select_target(rows, 7, 8), rows[0])

    def test_scan_exhausts_range_even_after_finding_motor(self):
        class Clock:
            now = 0
            def tick(self):
                self.now += 0.01
                return self.now

        class Bus:
            def __init__(self):
                self.sent, self.pending = [], []
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def setsockopt(self, *args): pass
            def settimeout(self, *args): pass
            def bind(self, *args): pass
            def send(self, frame):
                self.sent.append(frame)
                _, _, payload = struct.unpack("=IB3x8s", frame)
                if payload[0] == 7:
                    self.pending.append(struct.pack("=IB3x8s", 8, 8, bytes.fromhex("0700330a03000000")))
            def recv(self, _size):
                if self.pending: return self.pending.pop()
                raise TimeoutError

        bus, clock = Bus(), Clock()
        with patch.object(discovery, "link_state"), patch.object(discovery.socket, "socket", return_value=bus), \
                patch.object(discovery.time, "monotonic", side_effect=clock.tick), contextlib.redirect_stdout(io.StringIO()):
            rows = discovery.scan_can("DM", "can0", [7, 8], 0.1, 2)
        self.assertEqual(len(bus.sent), 4)
        self.assertEqual(rows, [{"motor_id": 7, "master_id": 8, "mode": 3, "device": "can0"}])

    def test_faulty_link_sends_nothing(self):
        with patch.object(discovery, "link_state", side_effect=RuntimeError("BUS-OFF")), \
                patch.object(discovery.socket, "socket") as socket_factory:
            with self.assertRaises(RuntimeError):
                discovery.scan_can("DM", "can0", [7], 0.1, 1)
            socket_factory.assert_not_called()


class ParameterTest(unittest.TestCase):
    def test_each_protocol_builds_single_axis_config(self):
        for protocol, node in [("DM", "7"), ("MT", "1"), ("PP", "0x13"), ("ECB", "1")]:
            argv = ["--motor-id", node]
            if protocol == "DM": argv += ["--master-id", "8"]
            if protocol == "ECB": argv += ["--device", "ecb://192.168.1.30"]
            args = runner.arguments(protocol, defaults(protocol), argv)
            axis = runner.make_config(args)["can_driver_node"]["joints"]
            self.assertEqual(len(axis), 1)
            self.assertEqual(axis[0]["protocol"], protocol)

    def test_id_is_auto_by_default(self):
        for protocol in ("DM", "MT", "PP", "ECB"):
            self.assertIsNone(runner.arguments(protocol, defaults(protocol), []).motor_id)

    def test_pp_ppr_normalizes_to_si(self):
        args = runner.arguments("PP", defaults("PP"), ["--velocity-scale", "65536"])
        self.assertAlmostEqual(args.velocity_scale, 2 * math.pi / 65536)

    def test_mt_command_and_feedback_units(self):
        args = runner.arguments("MT", defaults("MT"), ["--speed", "1"])
        self.assertAlmostEqual(args.speed / args.velocity_scale, math.degrees(1) * 100)
        self.assertAlmostEqual(57 * args.velocity_scale * 100, math.radians(57))

    def test_invalid_parameters_rejected_before_io(self):
        cases = [("DM", ["--speed", "nan"]), ("DM", ["--duration", "0"]),
                 ("DM", ["--velocity-scale", "65536"]), ("DM", ["--motor-id", "16"]),
                 ("MT", ["--motor-id", "0x141"]), ("PP", ["--scan-end", "256"]),
                 ("PP", ["--speed", "3"]), ("ECB", ["--device", "can0"]),
                 ("DM", ["--dry-run"]), ("PP", ["--scan-retries", "0"])]
        for protocol, argv in cases:
            with self.subTest(protocol=protocol, argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    runner.arguments(protocol, defaults(protocol), argv)
                self.assertEqual(error.exception.code, 2)


class SdoTest(unittest.TestCase):
    def test_sdo_read_write_and_abort(self):
        from unittest.mock import Mock
        bus = object.__new__(SdoBus)
        bus.node, bus.timeout = 6, 0.1
        bus.bus = Mock()
        bus.bus.recv.return_value = struct.pack("=IB3x8s", 0x586, 8, bytes.fromhex("436c6000ffffffff"))
        self.assertEqual(bus.transfer(0x606C, signed=True), -1)
        bus.bus.recv.return_value = struct.pack("=IB3x8s", 0x586, 8, bytes.fromhex("60ff600000000000"))
        bus.transfer(0x60FF, value=-100, signed=True)
        frame = bus.bus.send.call_args.args[0]
        ident, count, data = struct.unpack("=IB3x8s", frame)
        self.assertEqual((ident, count, data), (0x606, 8, bytes.fromhex("23ff60009cffffff")))
        bus.bus.recv.return_value = struct.pack("=IB3x8s", 0x586, 8, bytes.fromhex("80ff600000000206"))
        with self.assertRaisesRegex(RuntimeError, "abort"):
            bus.transfer(0x60FF, value=0)


if __name__ == "__main__":
    unittest.main()
